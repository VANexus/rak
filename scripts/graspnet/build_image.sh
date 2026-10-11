#!/usr/bin/env bash
set -euo pipefail
# Run in WSL2 after Ubuntu 24.04 and GPU-enabled Docker are installed.
tools="${1:-/mnt/d/car/tools/graspnet}"
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 - "$tools" "$here/assets.json" <<'PY'
import hashlib, json, pathlib, subprocess, sys
root, manifest = pathlib.Path(sys.argv[1]), json.loads(pathlib.Path(sys.argv[2]).read_text())
for name, directory in [('baseline', 'graspnet-baseline'), ('api', 'graspnetAPI')]:
    actual = subprocess.check_output(['git', '-C', str(root/directory), 'rev-parse', 'HEAD'], text=True).strip()
    if actual != manifest[name]['commit']:
        raise SystemExit(f'{directory}: expected pinned commit, got {actual}')
    # Match Windows checkouts when checking CRLF files through WSL's Git.
    if subprocess.check_output(['git', '-c', 'core.autocrlf=true', '-C', str(root/directory), 'status', '--porcelain'], text=True).strip():
        raise SystemExit(f'{directory}: upstream checkout must be clean; patches apply inside image')
weight = root/'models'/manifest['checkpoint']['name']
actual = hashlib.sha256(weight.read_bytes()).hexdigest()
if actual != manifest['checkpoint']['sha256']:
    raise SystemExit('checkpoint SHA256 mismatch')
print('Pinned sources and checkpoint verified')
PY
cp "$here/Dockerfile" "$here/compat.py" "$here/.dockerignore" "$tools/"
docker build --tag rak-graspnet:24.05 --file "$tools/Dockerfile" "$tools"
docker image inspect rak-graspnet:24.05 > "$tools/image-inspect.json"
docker run --rm --entrypoint python rak-graspnet:24.05 -m pip freeze > "$tools/container-python-freeze.txt"
docker run --rm --gpus all rak-graspnet:24.05 python -c \
  'import torch, pointnet2._ext; from knn_pytorch import knn_pytorch; assert torch.cuda.is_available(); print(torch.__version__, torch.version.cuda, torch.cuda.get_device_name())'
