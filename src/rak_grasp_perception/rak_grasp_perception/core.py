"""ROS/model-independent snapshot geometry and candidate contracts (SI units)."""
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def check_transform(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-5)):
        raise ValueError("invalid rigid transform")
    return matrix


def check_camera(info, shape):
    k = np.asarray(info["k"], dtype=float).reshape(3, 3)
    if (not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0
            or (info["height"], info["width"]) != tuple(shape[:2])):
        raise ValueError("missing/invalid camera intrinsics or image dimensions")
    if info.get("distortion_model", "plumb_bob") not in ("plumb_bob", "rational_polynomial", ""):
        raise ValueError("unsupported distortion model")
    d = np.asarray(info.get("d", []), dtype=float)
    if not np.isfinite(d).all() or len(d) not in (0, 4, 5, 8, 12, 14):
        raise ValueError("invalid distortion coefficients")
    return k, d


def load_snapshot(directory):
    root = Path(directory)
    meta = json.loads((root / "snapshot.json").read_text())
    if meta.get("schema_version") != 1 or meta.get("depth_unit") != "mm":
        raise ValueError("snapshot requires schema 1 and depth_unit=mm")
    required = {"color_stamp_ns", "depth_stamp_ns", "depth_frame", "depth_info",
                "color_info", "depth_to_color"}
    if not required.issubset(meta) or any("k" not in meta[key] for key in ("depth_info", "color_info")):
        raise ValueError("missing calibration or snapshot metadata")
    if any(type(meta[key]) is not int or meta[key] < 0 for key in ("color_stamp_ns", "depth_stamp_ns")):
        raise ValueError("snapshot timestamps must be nonnegative integer nanoseconds")
    if meta.get("depth_scale_m", 0.001) != 0.001:
        raise ValueError("millimetre snapshot requires depth_scale_m=0.001")
    if abs(meta["color_stamp_ns"] - meta["depth_stamp_ns"]) > 20_000_000:
        raise ValueError("RGB/depth skew exceeds 20 ms")
    color = cv2.imread(str(root / "color.png"), cv2.IMREAD_COLOR)
    depth = cv2.imread(str(root / "depth-mm.png"), cv2.IMREAD_UNCHANGED)
    if color is None or depth is None or depth.dtype != np.uint16 or depth.ndim != 2:
        raise ValueError("missing images or depth is not uint16 millimetres")
    check_camera(meta["depth_info"], depth.shape)
    check_camera(meta["color_info"], color.shape)
    check_transform(meta["depth_to_color"])
    if not meta.get("depth_frame"):
        raise ValueError("missing optical frame")
    return meta, cv2.cvtColor(color, cv2.COLOR_BGR2RGB), depth


def scene_cloud(meta, rgb, depth):
    k, d = check_camera(meta["depth_info"], depth.shape)
    rows, cols = np.nonzero(depth)
    if len(rows) == 0:
        raise ValueError("empty depth cloud")
    uv = np.column_stack((cols, rows)).astype(np.float64).reshape(-1, 1, 2)
    normalized = cv2.undistortPoints(uv, k, d if len(d) else None).reshape(-1, 2)
    z = depth[rows, cols].astype(np.float32) * 0.001
    xyz = np.column_stack((normalized * z[:, None], z)).astype(np.float32)
    transform = check_transform(meta["depth_to_color"])
    color_xyz = xyz @ transform[:3, :3].T + transform[:3, 3]
    kc, dc = check_camera(meta["color_info"], rgb.shape)
    projected, _ = cv2.projectPoints(color_xyz, np.zeros(3), np.zeros(3), kc,
                                    dc if len(dc) else None)
    finite = np.isfinite(projected).all(axis=(1, 2)) & (color_xyz[:, 2] > 0)
    pixels = np.rint(np.nan_to_num(projected.reshape(-1, 2))).astype(np.int64)
    good = finite & (pixels[:, 0] >= 0) & (pixels[:, 0] < rgb.shape[1])
    good &= (pixels[:, 1] >= 0) & (pixels[:, 1] < rgb.shape[0])
    colors = np.full((len(xyz), 3), 128, dtype=np.uint8)
    # Keep occluded projected points neutral rather than painting foreground color on them.
    index = pixels[good, 1] * rgb.shape[1] + pixels[good, 0]
    zbuffer = np.full(rgb.shape[0] * rgb.shape[1], np.inf)
    np.minimum.at(zbuffer, index, color_xyz[good, 2])
    visible_indices = np.flatnonzero(good)[color_xyz[good, 2] <= zbuffer[index] + 0.01]
    colors[visible_indices] = rgb[pixels[visible_indices, 1], pixels[visible_indices, 0]]
    return xyz, colors


def roi_mask(points, roi):
    lower, upper = np.asarray(roi["min"], float), np.asarray(roi["max"], float)
    if (lower.shape != (3,) or upper.shape != (3,) or not np.isfinite([lower, upper]).all()
            or np.any(lower >= upper)):
        raise ValueError("ROI must have finite min < max on every axis")
    return np.all((points >= lower) & (points <= upper), axis=1)


def candidate(row, ident):
    row = np.asarray(row, dtype=float)
    if row.shape != (17,) or not np.isfinite(row).all():
        raise ValueError("invalid GraspNet row")
    rotation = row[4:13].reshape(3, 3)
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = rotation, row[13:16]
    check_transform(matrix)
    # GraspNet clamps FP32 width at 0.1, represented as 0.10000000149.
    # Permit only that rounding error and store the contractual SI limit.
    if not 0 < row[1] <= 0.10 + 1e-7 or row[3] <= 0:
        raise ValueError("invalid gripper opening/depth")
    return {"id": int(ident), "position": row[13:16].tolist(),
            "quaternion_xyzw": Rotation.from_matrix(rotation).as_quat().tolist(),
            "width_m": min(float(row[1]), 0.10), "depth_m": float(row[3]), "score": float(row[0])}


def validate_result(data, meta):
    if (data.get("schema_version") != 1 or data.get("frame_id") != meta["depth_frame"]
            or data.get("stamp_ns") != meta["depth_stamp_ns"]):
        raise ValueError("result does not belong to this snapshot frame/time")
    seen = set()
    for item in data["candidates"]:
        ident = item["id"]
        q = np.asarray(item["quaternion_xyzw"], float)
        p = np.asarray(item["position"], float)
        values = [item["width_m"], item["depth_m"], item["score"]]
        if (not isinstance(ident, int) or not 0 <= ident < 2**32 or ident in seen
                or q.shape != (4,) or p.shape != (3,) or not np.isfinite(p).all()
                or not np.isfinite(q).all() or not np.isclose(np.linalg.norm(q), 1, atol=1e-5)
                or not np.isfinite(values).all() or not 0 < values[0] <= 0.10 or values[1] <= 0):
            raise ValueError("invalid or duplicate grasp candidate")
        seen.add(ident)
    if len(seen) > 50:
        raise ValueError("at most 50 replay candidates")
    return data


def gripper_lines(item):
    """U outline in GraspNet coordinates. +X is approach, +Y is closing."""
    w, depth = item["width_m"] / 2, item["depth_m"]
    local = np.array([[-0.04, -w, 0], [depth, -w, 0],
                      [-0.04, w, 0], [depth, w, 0],
                      [-0.04, -w, 0], [-0.04, w, 0],
                      [-0.07, 0, 0], [-0.04, 0, 0]])
    rotation = Rotation.from_quat(item["quaternion_xyzw"]).as_matrix()
    return local @ rotation.T + np.asarray(item["position"])


def nms_rows(rows, distance=0.03, angle=np.pi/6):
    """Greedy SE(3) NMS with parallel-finger 180-degree symmetry."""
    rows = np.asarray(rows)
    remaining = np.argsort(-rows[:, 0], kind="stable")
    kept = []
    symmetry = np.diag([1., -1., -1.])
    while len(remaining):
        best, remaining = remaining[0], remaining[1:]
        kept.append(best)
        if not len(remaining):
            break
        r = rows[best, 4:13].reshape(3, 3)
        rotations = rows[remaining, 4:13].reshape(-1, 3, 3)
        cos_a = (np.einsum("ij,nij->n", r, rotations)-1)/2
        cos_b = (np.einsum("ij,nij->n", r @ symmetry, rotations)-1)/2
        nearby = np.linalg.norm(rows[remaining, 13:16]-rows[best, 13:16], axis=1) < distance
        same_angle = np.maximum(cos_a, cos_b) > np.cos(angle)
        remaining = remaining[~(nearby & same_angle)]
    return rows[kept]
