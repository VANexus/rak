"""Build the TensorRT engine for rak_road_perception and record its provenance.

Engine builds are expensive (Small 377 s, Medium 447 s on the Orin at 10 W — see
`docs/design/vision_nav_v1.md` §0.1), so the engine is a cached artifact, never a
startup step. This script builds once and writes a `.manifest.json` beside the engine
holding the source ONNX hash, TensorRT version and input shape; the node's operator
can tell which weights an engine came from without guessing.

Usage:
    python3 scripts/build_roadseg_engine.py --onnx twinlite_plus_small_da.onnx \\
        --engine data/models/roadseg/twinlite_plus_small_fp16_384x672.plan

Export the ONNX first with scripts/export_twinlite_onnx.py.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build(onnx: Path, engine: Path, width: int, height: int, int8: bool) -> dict:
    trtexec = shutil.which("trtexec") or "/usr/src/tensorrt/bin/trtexec"
    if not Path(trtexec).is_file():
        raise SystemExit("trtexec not found; it ships with JetPack under /usr/src/tensorrt/bin")

    engine.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        trtexec,
        f"--onnx={onnx}",
        f"--saveEngine={engine}",
        f"--shapes=input:1x3x{height}x{width}",
        "--useCudaGraph",
        "--int8" if int8 else "--fp16",
    ]
    print("+ " + " ".join(cmd))
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        sys.stderr.write(result.stdout[-4000:])
        sys.stderr.write(result.stderr[-2000:])
        raise SystemExit(f"trtexec failed with exit code {result.returncode}")

    import tensorrt as trt

    manifest = {
        "onnx": str(onnx.name),
        "onnx_sha256": sha256(onnx),
        "engine": str(engine.name),
        "engine_sha256": sha256(engine),
        "input": f"1x3x{height}x{width}",
        "precision": "int8" if int8 else "fp16",
        "tensorrt": trt.__version__,
        "device": "Jetson Orin Nano (compute 8.7), 10 W mode",
        "note": "Orin-IMPORTANT: engines are not portable across JetPack/TensorRT versions; rebuild rather than copy.",
    }
    manifest_path = engine.with_suffix(engine.suffix + ".manifest.json")
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"engine: {engine} ({engine.stat().st_size/1e6:.1f} MB)")
    print(f"manifest: {manifest_path}")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--onnx", type=Path, required=True)
    parser.add_argument("--engine", type=Path, required=True)
    parser.add_argument("--width", type=int, default=672)
    parser.add_argument("--height", type=int, default=384)
    parser.add_argument("--int8", action="store_true",
                        help="needs a calibration table; FP16 is the supported path")
    args = parser.parse_args()
    build(args.onnx.resolve(), args.engine.resolve(), args.width, args.height, args.int8)


if __name__ == "__main__":
    main()
