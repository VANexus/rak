"""Fetch the pinned public NGC image as a hash-verified OCI archive.

Optional fallback for links that throttle individual long downloads. Uses bounded
HTTP ranges and retains completed chunks for resuming. No credentials required.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import io
import json
from pathlib import Path
import tarfile
import threading
import time
import urllib.request

IMAGE = "nvcr.io/nvidia/pytorch:24.05-py3"
DIGEST = "sha256:d4030fc006cce2e56497fa07d003c5996de0eb7bde3610599fedbe7e7d03adea"
BASE = "https://nvcr.io"
CHUNK = 4 * 1024 * 1024


def request(url, headers=None):
    return urllib.request.urlopen(urllib.request.Request(url, headers=headers or {}), timeout=60)


def digest_file(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(CHUNK), b""):
            h.update(block)
    return "sha256:" + h.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New or resumable cache directory")
    parser.add_argument("--workers", type=int, default=24, choices=range(1, 33))
    args = parser.parse_args()
    root = args.output.resolve()
    blobs, chunks = root / "blobs/sha256", root / "chunks"
    blobs.mkdir(parents=True, exist_ok=True)
    chunks.mkdir(exist_ok=True)
    auth_lock, auth_cache = threading.Lock(), {}

    def headers():
        with auth_lock:
            if time.monotonic() >= auth_cache.get("deadline", 0):
                with request(BASE + "/proxy_auth?service=registry&scope=repository:nvidia/pytorch:pull") as response:
                    auth = json.load(response)
                auth_cache.update(token=auth["token"],
                    deadline=time.monotonic() + max(30, min(300, auth.get("expires_in", 600) - 60)))
            return {"Authorization": "Bearer " + auth_cache["token"],
                    "Accept": "application/vnd.docker.distribution.manifest.v2+json"}

    with request(BASE + "/v2/nvidia/pytorch/manifests/" + DIGEST, headers()) as response:
        raw = response.read()
    if "sha256:" + hashlib.sha256(raw).hexdigest() != DIGEST:
        raise RuntimeError("NGC manifest digest mismatch")
    manifest = json.loads(raw)
    root_media_type, root_size = manifest["mediaType"], len(raw)
    (blobs / DIGEST.split(":")[1]).write_bytes(raw)
    if "manifests" in manifest:
        platform = next(x for x in manifest["manifests"]
                        if x["platform"]["architecture"] == "amd64"
                        and x["platform"]["os"] == "linux")
        with request(BASE + "/v2/nvidia/pytorch/manifests/" + platform["digest"], headers()) as response:
            raw = response.read()
        if "sha256:" + hashlib.sha256(raw).hexdigest() != platform["digest"]:
            raise RuntimeError("NGC amd64 manifest digest mismatch")
        (blobs / platform["digest"].split(":")[1]).write_bytes(raw)
        manifest = json.loads(raw)
    descriptors = [manifest["config"], *manifest["layers"]]
    work = []
    for item in descriptors:
        digest, size = item["digest"], item["size"]
        output = blobs / digest.split(":")[1]
        if output.exists():
            if output.stat().st_size == size and digest_file(output) == digest:
                continue
            raise RuntimeError(f"Invalid cached blob: {output}")
        for start in range(0, size, CHUNK):
            end = min(start + CHUNK, size) - 1
            work.append((digest, size, start, end))
    total = sum(item["size"] for item in descriptors)
    print(f"Pinned image: {DIGEST}; compressed {total / 2**30:.2f} GiB; {len(work)} chunks", flush=True)

    def fetch(item):
        digest, size, start, end = item
        part = chunks / f"{digest.split(':')[1]}.{start}"
        if part.exists() and part.stat().st_size == end - start + 1:
            return part.stat().st_size
        for attempt in range(5):
            try:
                h = {**headers(), "Range": f"bytes={start}-{end}"}
                with request(BASE + "/v2/nvidia/pytorch/blobs/" + digest, h) as response:
                    if response.status == 206:
                        if response.headers.get("Content-Range") != f"bytes {start}-{end}/{size}":
                            raise RuntimeError("Incorrect Content-Range")
                    elif not (response.status == 200 and start == 0 and end == size - 1):
                        raise RuntimeError("Server ignored bounded range")
                    data = response.read(end - start + 2)
                if len(data) != end - start + 1:
                    raise RuntimeError("Truncated chunk")
                temp = part.with_suffix(part.suffix + ".partial")
                temp.write_bytes(data)
                temp.replace(part)
                return len(data)
            except Exception:
                if attempt == 4:
                    raise
                time.sleep(1 + attempt)

    downloaded, report, started = 0, time.monotonic(), time.monotonic()
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = [pool.submit(fetch, item) for item in work]
        for n, future in enumerate(as_completed(futures), 1):
            downloaded += future.result()
            if time.monotonic() - report > 20:
                print(f"Chunks {n}/{len(work)}, {downloaded / 2**30:.2f} GiB, "
                      f"{downloaded / 2**20 / (time.monotonic()-started):.2f} MiB/s", flush=True)
                report = time.monotonic()
    for item in descriptors:
        digest, size = item["digest"], item["size"]
        output = blobs / digest.split(":")[1]
        if output.exists():
            continue
        temp = output.with_suffix(".partial")
        h = hashlib.sha256()
        with temp.open("wb") as stream:
            for start in range(0, size, CHUNK):
                part = chunks / f"{digest.split(':')[1]}.{start}"
                data = part.read_bytes()
                stream.write(data)
                h.update(data)
        if temp.stat().st_size != size or "sha256:" + h.hexdigest() != digest:
            raise RuntimeError(f"Layer digest mismatch: {digest}; cache retained for inspection")
        temp.replace(output)
        # Remove only the exact chunks of this verified blob, never a directory tree.
        for start in range(0, size, CHUNK):
            (chunks / f"{digest.split(':')[1]}.{start}").unlink()
    index = {"schemaVersion": 2, "manifests": [{
        "mediaType": root_media_type, "digest": DIGEST, "size": root_size,
        "annotations": {"org.opencontainers.image.ref.name": IMAGE,
                        "io.containerd.image.name": IMAGE}}]}
    archive = root / "pytorch-24.05.oci.tar"
    temp = archive.with_suffix(".partial")
    with tarfile.open(temp, "w") as tar:
        for name, value in (("oci-layout", {"imageLayoutVersion": "1.0.0"}), ("index.json", index)):
            data = json.dumps(value).encode()
            info = tarfile.TarInfo(name)
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
        for file in sorted(blobs.iterdir()):
            if file.is_file() and "." not in file.name:
                tar.add(file, arcname=f"blobs/sha256/{file.name}", recursive=False)
    temp.replace(archive)
    print(f"Verified OCI archive: {archive}", flush=True)


if __name__ == "__main__":
    main()
