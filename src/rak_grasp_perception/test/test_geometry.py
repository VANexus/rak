import copy
import json

import cv2
import numpy as np
import pytest
from rak_grasp_perception.core import (candidate, check_transform, gripper_lines,
    load_snapshot, roi_mask, scene_cloud, validate_result, nms_rows)


def metadata():
    info = {"width": 3, "height": 3, "k": [100, 0, 1, 0, 100, 1, 0, 0, 1], "d": [], "distortion_model": "plumb_bob"}
    return {"schema_version": 1, "depth_unit": "mm", "color_stamp_ns": 1000,
            "depth_stamp_ns": 1000, "depth_frame": "camera_depth_optical_frame",
            "depth_info": info, "color_info": copy.deepcopy(info), "depth_to_color": np.eye(4).tolist()}


def test_metric_plane_and_projection_uses_extrinsics():
    meta = metadata()
    rgb = np.zeros((3, 3, 3), np.uint8)
    rgb[:, 2] = [255, 0, 0]
    depth = np.full((3, 3), 2000, np.uint16)
    xyz, colors = scene_cloud(meta, rgb, depth)
    np.testing.assert_allclose(xyz[4], [0, 0, 2])
    np.testing.assert_allclose(xyz[0], [-0.02, -0.02, 2])
    assert list(colors[4]) == [0, 0, 0]
    meta["depth_to_color"][0][3] = 0.02
    _, shifted = scene_cloud(meta, rgb, depth)
    assert list(shifted[4]) == [255, 0, 0]
    assert list(shifted[5]) == [128]*3


def test_empty_cloud_and_bad_calibration():
    with pytest.raises(ValueError, match="empty"):
        scene_cloud(metadata(), np.zeros((3, 3, 3), np.uint8), np.zeros((3, 3), np.uint16))
    broken = metadata()
    broken["depth_info"]["k"][0] = 0
    with pytest.raises(ValueError, match="intrinsics"):
        scene_cloud(broken, np.zeros((3, 3, 3), np.uint8), np.ones((3, 3), np.uint16))
    with pytest.raises(ValueError):
        check_transform(np.zeros((4, 4)))


@pytest.mark.parametrize("change", ["unit", "skew", "frame", "missing_calibration"])
def test_bad_snapshot_rejected(tmp_path, change):
    meta = metadata()
    cv2.imwrite(str(tmp_path / "color.png"), np.zeros((3, 3, 3), np.uint8))
    cv2.imwrite(str(tmp_path / "depth-mm.png"), np.ones((3, 3), np.uint16))
    if change == "unit":
        meta["depth_unit"] = "m"
    elif change == "skew":
        meta["color_stamp_ns"] += 20_000_001
    elif change == "frame":
        meta["depth_frame"] = ""
    else:
        del meta["depth_info"]
    (tmp_path / "snapshot.json").write_text(json.dumps(meta))
    with pytest.raises(ValueError):
        load_snapshot(tmp_path)


def test_roi_and_grasp_convention():
    points = np.array([[0, 0, 2], [1, 1, 2]])
    np.testing.assert_array_equal(roi_mask(points, {"min": [-.1, -.1, 1], "max": [.1, .1, 3]}), [True, False])
    with pytest.raises(ValueError):
        roi_mask(points, {"min": [0]*3, "max": [0]*3})
    row = np.r_[0.8, 0.04, 0.02, 0.03, np.eye(3).ravel(), [0, 0, 2], -1]
    item = candidate(row, 0)
    lines = gripper_lines(item)
    assert lines[1, 0] > lines[0, 0]
    assert lines[1, 1] == -0.02
    assert lines[3, 1] == 0.02
    result = {"schema_version": 1, "frame_id": metadata()["depth_frame"], "stamp_ns": 1000, "candidates": [item]}
    validate_result(result, metadata())
    result["candidates"].append(item)
    with pytest.raises(ValueError, match="duplicate"):
        validate_result(result, metadata())


def test_invalid_rotation_and_wrong_snapshot():
    row = np.r_[0.8, 0.04, 0.02, 0.03, np.zeros(9), [0, 0, 2], -1]
    with pytest.raises(ValueError):
        candidate(row, 0)
    with pytest.raises(ValueError, match="snapshot"):
        validate_result({"schema_version": 1, "frame_id": "wrong", "stamp_ns": 1000}, metadata())


def test_model_fp32_width_limit_rounding():
    row = np.r_[0.8, 0.1, 0.02, 0.03, np.eye(3).ravel(), [0, 0, 2], -1].astype(np.float32)
    assert candidate(row, 0)["width_m"] == 0.1
    row[1] = 0.10001
    with pytest.raises(ValueError, match="opening"):
        candidate(row, 0)


def test_nms_parallel_finger_symmetry():
    row = np.r_[0.8, 0.04, 0.02, 0.03, np.eye(3).ravel(), [0, 0, 2], -1]
    duplicate = row.copy()
    duplicate[0] = 0.7
    duplicate[4:13] = np.diag([1, -1, -1]).ravel()
    other = row.copy()
    other[13] += 0.1
    assert len(nms_rows(np.array([row, duplicate, other]))) == 2
    assert nms_rows(np.empty((0, 17))).shape == (0, 17)
