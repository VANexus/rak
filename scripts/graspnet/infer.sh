#!/usr/bin/env bash
set -euo pipefail
if [ "$#" -ne 3 ]; then
  echo 'Usage: bash scripts/graspnet/infer.sh SNAPSHOT_DIR ROI_JSON NEW_RESULT_JSON' >&2
  exit 2
fi
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
tools="${GRASPNET_TOOLS:-/mnt/d/car/tools/graspnet}"
# All three arguments must live under tools. Bind-mount code read-only.
mapfile -t relative < <(python3 - "$tools" "$@" <<'PY'
from pathlib import Path
import sys
root = Path(sys.argv[1]).resolve()
for value in sys.argv[2:]:
    print(Path(value).resolve().relative_to(root))
PY
)
[ "${#relative[@]}" -eq 3 ] || { echo 'Arguments must be inside GRASPNET_TOOLS' >&2; exit 2; }
image_id="$(docker image inspect --format '{{.Id}}' rak-graspnet:24.05)"
docker run --rm --gpus all --shm-size=2g \
  --mount "type=bind,src=$tools,dst=/assets" \
  --mount "type=bind,src=$repo/src/rak_grasp_perception,dst=/code,readonly" \
  --env PYTHONPATH=/code --env "RAK_GRASPNET_IMAGE_ID=$image_id" rak-graspnet:24.05 \
  python -m rak_grasp_perception.infer \
  --snapshot "/assets/${relative[0]}" --roi "/assets/${relative[1]}" \
  --output "/assets/${relative[2]}" --baseline /opt/graspnet-baseline \
  --checkpoint /assets/models/checkpoint-rs.tar
