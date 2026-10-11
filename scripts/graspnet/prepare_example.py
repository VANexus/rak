"""Convert the pinned official demo to the snapshot contract; no generated grasps."""
import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np
from scipy.io import loadmat

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--tools", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
example = args.tools / "graspnet-baseline/doc/example_data"
meta = loadmat(example / "meta.mat")
depth = cv2.imread(str(example / "depth.png"), -1)
color = cv2.imread(str(example / "color.png"))
mask = cv2.imread(str(example / "workspace_mask.png"), -1) > 0
depth_mm = depth.astype(float) * 1000 / float(meta["factor_depth"].item())
if np.any(depth_mm > 65535):
    raise ValueError("example exceeds uint16 mm range")
args.output.mkdir(parents=True, exist_ok=False)
cv2.imwrite(str(args.output / "color.png"), color)
cv2.imwrite(str(args.output / "depth-mm.png"), np.rint(depth_mm).astype(np.uint16))
info = {"width": depth.shape[1], "height": depth.shape[0],
        "k": meta["intrinsic_matrix"].ravel().tolist(), "d": [], "distortion_model": "plumb_bob"}
value = {"schema_version": 1, "depth_unit": "mm", "depth_scale_m": 0.001,
         "color_stamp_ns": 0, "depth_stamp_ns": 0, "depth_frame": "camera_depth_optical_frame",
         "depth_info": info, "color_info": info, "depth_to_color": np.eye(4).tolist(),
         "source": "official registered RGB-D demo; no live acquisition timestamp"}
(args.output / "snapshot.json").write_text(json.dumps(value, indent=2))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src/rak_grasp_perception"))
from rak_grasp_perception.core import load_snapshot, scene_cloud
loaded, rgb, mm = load_snapshot(args.output)
xyz, _ = scene_cloud(loaded, rgb, mm)
selected = xyz[mask[mm > 0]]
roi = {"min": selected.min(axis=0).tolist(), "max": selected.max(axis=0).tolist()}
(args.output / "roi.json").write_text(json.dumps(roi, indent=2))
print(args.output)
