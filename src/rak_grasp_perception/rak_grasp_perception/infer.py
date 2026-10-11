"""One-shot GraspNet CLI for the x86 developer container, not a ROS model service."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

from .core import candidate, load_snapshot, roi_mask, scene_cloud, nms_rows


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", required=True, type=Path)
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--roi", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--seed", type=int, default=0)
    cli = parser.parse_args(args)
    processing_started = time.perf_counter()
    if cli.output.exists():
        parser.error("result already exists; use a new output filename")
    meta, rgb, depth = load_snapshot(cli.snapshot)
    scene, colors = scene_cloud(meta, rgb, depth)
    roi = json.loads(cli.roi.read_text())
    selected = scene[roi_mask(scene, roi)]
    if len(selected) < 64:
        raise ValueError("ROI has fewer than 64 valid depth points")
    # These imports intentionally do not run during normal ROS package import.
    for folder in ("models", "utils", "pointnet2", "knn", "dataset"):
        sys.path.insert(0, str(cli.baseline.resolve() / folder))
    import torch
    from graspnet import GraspNet, pred_decode
    from graspnetAPI import GraspGroup
    from collision_detector import ModelFreeCollisionDetector

    if not torch.cuda.is_available():
        raise RuntimeError("GraspNet CUDA extensions require the x86 GPU developer environment")
    torch.manual_seed(cli.seed)
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    rng = np.random.default_rng(cli.seed)
    sample = selected[rng.choice(len(selected), 20000, replace=len(selected) < 20000)]
    net = GraspNet(input_feature_dim=0, num_view=300, num_angle=12, num_depth=4,
        cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01, 0.02, 0.03, 0.04], is_training=False)
    checkpoint = torch.load(str(cli.checkpoint), map_location="cpu", weights_only=False)
    net.load_state_dict(checkpoint["model_state_dict"])
    net = net.float().cuda().eval()
    points = torch.from_numpy(sample[None].astype(np.float32)).cuda()
    torch.cuda.reset_peak_memory_stats()
    torch.cuda.synchronize()
    started = time.perf_counter()
    with torch.inference_mode():
        raw = pred_decode(net({"point_clouds": points}))[0].cpu().numpy()
    torch.cuda.synchronize()
    inference_s = time.perf_counter() - started
    grasps = GraspGroup(raw)
    rejection = {"raw": len(grasps)}
    grasps = grasps[(grasps.widths > 0) & (grasps.widths <= 0.10)
                    & roi_mask(grasps.translations, roi)]
    rejection["width_or_outside_roi"] = rejection["raw"] - len(grasps)
    # Full scene, including desk and nearby clutter, is used for collision filtering.
    rejection["scene_collision"] = 0
    if len(grasps):
        detector = ModelFreeCollisionDetector(scene, voxel_size=0.005)
        collision = np.concatenate([detector.detect(grasps[start:start+32],
            approach_dist=0.05, collision_thresh=0.01) for start in range(0, len(grasps), 32)])
        rejection["scene_collision"] = int(collision.sum())
        grasps = grasps[~collision]
    filtered = nms_rows(grasps.grasp_group_array)
    rejection["nms_duplicate"] = len(grasps)-len(filtered)
    rejection["display_limit"] = max(0, len(filtered)-50)
    items = [candidate(row, i) for i, row in enumerate(filtered[:50])]
    result = {"schema_version": 1, "frame_id": meta["depth_frame"],
        "stamp_ns": meta["depth_stamp_ns"], "backend": "graspnet-baseline",
        "candidates": items, "force_status": "unmeasured", "roi": roi,
        "metrics": {"inference_seconds": inference_s,
                    "total_processing_seconds": time.perf_counter()-processing_started,
                    "inference_and_filter_seconds": time.perf_counter()-started,
                    "peak_cuda_bytes": torch.cuda.max_memory_allocated(),
                    "peak_cuda_reserved_bytes": torch.cuda.max_memory_reserved(),
                    "scene_points": len(scene), "roi_points": len(selected),
                    "candidate_count": len(items), "filter_counts": rejection},
        "provenance": {"checkpoint_sha256": sha256(cli.checkpoint),
            "baseline_commit": subprocess.check_output(
                ["git", "-C", str(cli.baseline), "rev-parse", "HEAD"], text=True).strip(),
            "torch": torch.__version__, "cuda": torch.version.cuda,
            "gpu": torch.cuda.get_device_name(), "dtype": "float32", "tf32": False,
            "image_id": os.environ.get("RAK_GRASPNET_IMAGE_ID", "unrecorded"),
            "pipeline_sha256": {p: sha256(Path(__file__).with_name(p)) for p in ("core.py", "infer.py")},
            "seed": cli.seed}}
    cli.output.parent.mkdir(parents=True, exist_ok=True)
    cli.output.write_text(json.dumps(result, indent=2, allow_nan=False))
    np.savez_compressed(cli.output.with_suffix(".scene.npz"), xyz=scene, rgb=colors)
    print(json.dumps(result["metrics"], indent=2))


if __name__ == "__main__":
    main()
