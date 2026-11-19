#!/usr/bin/env python3
"""Fit the front camera's mount HEIGHT and PITCH from a road mask.

Why this exists: `cam_height_m` / `cam_pitch_rad` in
`rak_road_perception.road_follow_math.GOAL_DEFAULTS` were `[AI]` placeholders
(0.30 m / 0.0 rad). They set the metric scale of the whole MPPI path, so a wrong
value does not merely bias speeds -- it puts the corner arm at the wrong
distance. A Charuco calibration gives the intrinsic matrix but says nothing
about how the camera sits on the robot, and no ROS package recovers extrinsics
from a lane width.

The trick: the competition mat's lane is a KNOWN physical width (45 cm, measured
from the supplied map). For a flat-ground pinhole camera the width of a lane at
row y is

    width_px(y) = fx * LANE_M / d(y),      d(y) = h / tan(atan((y-cy)/fy) + pitch)

so two unknown scalars (h, pitch) and one observable curve. This script sweeps
them on a grid (deterministic, no scipy needed) and reports the residual.

Usage:
    # 1) capture masks from the live chain into an .npy (see the note below)
    # 2) fit
    python3 scripts/fit_camera_mount.py frames.npy --lane-m 0.45

Input: an .npy file holding one mask (HxW) or a stack (NxHxW) of mono8 masks,
all captured with the robot ON a straight lane of known width. numpy arrays are
loaded with allow_pickle=False, so a crafted file cannot execute code.

The fit is only as good as its assumptions -- it is a measurement, not a proof:
  * the image is assumed undistorted (check `camera_info`; if no rectification
    is wired, projection error is folded into these two numbers),
  * the lane is assumed flat and its mask is assumed to include the painted
    border the 45 cm refers to,
  * fx/fy/cy must belong to the SAME resolution as the masks.
"""

from __future__ import annotations

import argparse
import json
import sys

import numpy as np

# ChArUco calibration 2026-11-15 at 1280x720 (vision_nav_v1.md §5A.7). These
# mirror GOAL_DEFAULTS.fx_1280 / fy_1280 / cx_1280 / cy_1280.
FX_1280 = 844.40
FY_1280 = 851.46
CX_1280 = 614.20
CY_1280 = 401.18


def intrinsics(width: int, height: int):
    """Operating-resolution camera matrix, honouring the 640x480 crop mode.

    Measured 2026-11-19: 640x480 shares the VERTICAL field of view with
    1280x720 and crops horizontally (NCC 0.988 for scale-then-crop, 0.326 for a
    plain stretch), so fx/cx rescale by 480/720 like fy/cy after shifting the
    principal point for the crop. Scaling fx by 640/1280 is 33% low at 640 wide
    and silently shrinks every projected distance by the same factor.
    """
    s = height / 720.0
    crop_w = width / s                      # calibration-space width that survives
    x0 = (1280.0 - crop_w) / 2.0
    return (FX_1280 * s,
            FY_1280 * s,
            (CX_1280 - x0) * s,
            CY_1280 * s)


def lane_rows(mask: np.ndarray, saturate_frac: float = 0.93):
    """Rows usable for the fit: road present, not saturated against the frame."""
    h, w = mask.shape[:2]
    road = mask > 127
    rows, spans = [], []
    for y in range(h - 1, 0, -1):
        cols = np.flatnonzero(road[y])
        if cols.size < 3:
            continue
        span = float(cols[-1] - cols[0] + 1)
        if span > saturate_frac * w:
            continue          # closer than the frame: width is clipped
        rows.append(float(y))
        spans.append(span)
    return np.array(rows), np.array(spans)


def alignment_quality(mask: np.ndarray):
    """How far this frame is from "robot centred on a straight lane".

    The whole fit rests on that precondition, and it is easy to break: after the
    board restarted mid-session the robot had moved, and the same code returned
    h=0.52 m / 34.4 deg instead of 0.321 m / 26.2 deg with an equally small
    residual (measured 2026-11-19). A plausible-looking wrong answer is worse
    than a refusal, so every frame is screened before it is allowed to vote.

    Returns a dict with:
      centre_bias  mean ((left+right)/2 - frame_centre) as a fraction of width
      slant        mean edge drift per row, i.e. |d(centre)/dy| -- a lane the
                   robot is driving along converges symmetrically, a lane seen
                   at an angle drifts sideways
      usable       True when both are small enough to trust the fit
    """
    h, w = mask.shape[:2]
    road = mask > 127
    ys, mid = [], []
    for y in range(h - 1, 0, -1):
        cols = np.flatnonzero(road[y])
        if cols.size < 3:
            continue
        span = float(cols[-1] - cols[0] + 1)
        if span > 0.93 * w:
            continue
        ys.append(float(y))
        mid.append(((cols[0] + cols[-1]) / 2.0 - w / 2.0) / w)
    if len(ys) < 10:
        return {"centre_bias": 1.0, "slant": 1.0, "usable": False}
    ys = np.array(ys)
    mid = np.array(mid)
    bias = float(abs(np.mean(mid)))
    order = np.argsort(ys)
    slant = float(abs(np.polyfit(ys[order], mid[order], 1)[0]))
    return {"centre_bias": round(bias, 4), "slant": round(slant, 5),
            "usable": bias < 0.06 and slant < 1.2e-3}


def fit(mask: np.ndarray, lane_m: float, h_range=(0.10, 0.81), p_range=(-0.05, 0.75),
        step=0.005):
    h_img, w_img = mask.shape[:2]
    fx, fy, _cx, cy = intrinsics(w_img, h_img)
    rows, spans = lane_rows(mask)
    if rows.size < 10:
        return None
    best = None
    for h in np.arange(h_range[0], h_range[1], step):
        for pitch in np.arange(p_range[0], p_range[1], step):
            dep = np.arctan((rows - cy) / fy) + pitch
            if np.any(dep <= 0.03):
                continue
            pred = fx * lane_m / (h / np.tan(dep))
            rms = float(np.sqrt(np.mean((pred - spans) ** 2)))
            if best is None or rms < best[0]:
                best = (rms, float(h), float(pitch))
    if best is None:
        return None
    rms, h, pitch = best
    dep = np.arctan((rows - cy) / fy) + pitch
    dist = h / np.tan(dep)
    return {
        "height_m": round(h, 3),
        "pitch_rad": round(pitch, 4),
        "pitch_deg": round(float(np.degrees(pitch)), 1),
        "rms_px": round(rms, 1),
        "rows_used": int(rows.size),
        "range_m": [round(float(dist.min()), 2), round(float(dist.max()), 2)],
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("masks", help=".npy with one mask (HxW) or a stack (NxHxW)")
    ap.add_argument("--lane-m", type=float, default=0.45,
                    help="true lane width in metres (from the track map)")
    ap.add_argument("--step", type=float, default=0.005)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    # allow_pickle=False: these files may come from another machine, and a
    # pickle payload executes on load. Plain ndarray serialization is enough.
    frames = np.load(args.masks, allow_pickle=False)
    if frames.ndim == 2:
        frames = frames[None, ...]

    results = []
    rejected = []
    for i, frame in enumerate(frames):
        q = alignment_quality(frame)
        if not q["usable"]:
            rejected.append((i, q))
            print(f"frame {i}: REJECTED (centre_bias={q['centre_bias']:.3f}, "
                  f"slant={q['slant']:.1e}) -- robot not centred on a straight lane")
            continue
        r = fit(frame, args.lane_m, step=args.step)
        if r is None:
            print(f"frame {i}: not enough usable rows (is the robot on the lane?)")
            continue
        r["frame"] = i
        r["centre_bias"] = q["centre_bias"]
        results.append(r)
        print(f"frame {i}: h={r['height_m']:.3f} m  pitch={r['pitch_deg']:5.1f}° down  "
              f"rms={r['rms_px']:5.1f} px  rows={r['rows_used']}  "
              f"range={r['range_m'][0]:.2f}..{r['range_m'][1]:.2f} m")

    if not results:
        print(f"\nall {len(frames)} frames rejected: park the robot centred and "
              "square on a straight lane, then capture again.", file=sys.stderr)
        return 1
    if len(results) < 2:
        print("\nWARNING: fewer than 2 usable frames -- this is not a "
              "cross-validated number. Capture a few more.", file=sys.stderr)

    heights = np.array([r["height_m"] for r in results])
    pitches = np.array([r["pitch_rad"] for r in results])
    summary = {
        "height_m": round(float(np.median(heights)), 3),
        "pitch_rad": round(float(np.median(pitches)), 4),
        "pitch_deg": round(float(np.degrees(np.median(pitches))), 1),
        "spread_height_m": round(float(heights.max() - heights.min()), 3),
        "spread_pitch_deg": round(float(np.degrees(pitches.max() - pitches.min())), 1),
        "frames": len(results),
    }
    if args.json:
        print(json.dumps(summary, indent=1))
    else:
        print("\nmedian over frames:")
        print(f"  cam_height_m   = {summary['height_m']}")
        print(f"  cam_pitch_rad  = {summary['pitch_rad']}  ({summary['pitch_deg']}° down)")
        print(f"  spread: h ±{summary['spread_height_m']} m, "
              f"pitch ±{summary['spread_pitch_deg']}° over {summary['frames']} frames")
        print("\nA spread under ~0.02 m / ~2° means the fit is stable. Back-fill "
              "GOAL_DEFAULTS with the median, and note in the commit that the "
              "value is fitted from the map's 45 cm lane, not taped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
