"""Pure-math checks for road_follow_math (no ROS, numpy only)."""

import numpy as np

from rak_road_perception.road_follow_math import (
    FOLLOW_DEFAULTS,
    TRACE_DEFAULTS,
    follow_command,
    trace_centerline,
)

W, H = 640, 480
P = {**TRACE_DEFAULTS, **FOLLOW_DEFAULTS}


def band(m, x0, x1, y0=0, y1=H):
    m[y0:y1, x0:x1] = 255
    return m


def run(mask):
    return trace_centerline(mask, P)


def test_straight_centered():
    m = band(np.zeros((H, W), np.uint8), 280, 360)
    pts, spin, has = run(m)
    assert has and spin is None and len(pts) > 10
    vx, vy, wz = follow_command(pts, spin, has, P)
    assert abs(pts[-1][1]) < 0.02  # keeps center all the way up
    assert abs(wz) < 0.05 and abs(vy) < 0.01 and abs(vx - P["max_vx"]) < 1e-6


def test_offset_road_strafes_right():
    # Road to the RIGHT of heading: mecanum strafes right (vy<0), little yaw.
    m = band(np.zeros((H, W), np.uint8), 380, 460)
    pts, spin, has = run(m)
    vx, vy, wz = follow_command(pts, spin, has, P)
    assert vy < -0.05 and abs(wz) < 0.2 and vx > 0


def test_sharp_gradual_bend_followed_and_slowed():
    # Road curves left quadratically above y=320 (145deg-style bend).
    m = np.zeros((H, W), np.uint8)
    for y in range(H):
        cx = 320 - int(180 * (max(0, 320 - y) / 320) ** 2)
        band(m, cx - 40, cx + 40, y, y + 1)
    pts, spin, has = run(m)
    assert has and len(pts) > 20 and pts[-1][1] < -0.2  # trace reaches the bend
    vx, vy, wz = follow_command(pts, spin, has, P)
    # Corrective LEFT action: yaw or mecanum strafe (vy>0 = glide left).
    assert wz > 0.1 or vy > 0.05
    assert vx < P["max_vx"] * 0.8  # curvature slowdown


def test_crossroad_goes_straight():
    # Straight lane + wide junction band above (side branches merged).
    m = np.zeros((H, W), np.uint8)
    band(m, 290, 350, 200, H)          # straight lane bottom 60%
    band(m, 100, 540, 0, 200)          # junction: wide run (many branches)
    pts, spin, has = run(m)
    assert has and pts[-1][1] > -0.02 and pts[-1][1] < 0.05
    vx, vy, wz = follow_command(pts, spin, has, P)
    assert abs(wz) < 0.08 and abs(vy) < 0.05 and vx > 0  # keeps heading


def test_road_left_bottom_spins_toward_it():
    # Nothing under the robot; road visible in the upper-left only.
    m = np.zeros((H, W), np.uint8)
    band(m, 40, 120, 0, 300)
    pts, spin, has = run(m)
    assert not has and spin is not None and spin[1] < 0
    vx, vy, wz = follow_command(pts, spin, has, P)
    assert vx == 0.0 and vy == 0.0 and wz > 0.3  # spin left in place


def test_no_road_at_all_zero():
    m = np.zeros((H, W), np.uint8)
    pts, spin, has = run(m)
    assert not has and spin is None
    assert follow_command(pts, spin, has, P) == (0.0, 0.0, 0.0)


def test_branch_gate_refuses_side_branch():
    # Straight lane + a SIDE branch whose run does not contain the heading.
    # Straight-through policy must not jump onto it: trace stays on the lane.
    m = np.zeros((H, W), np.uint8)
    band(m, 290, 350, 100, H)
    band(m, 40, 120, 0, 100)           # separate side branch upper-left
    pts, spin, has = run(m)
    assert has and all(abs(p[1]) < 0.05 for p in pts)
    vx, vy, wz = follow_command(pts, spin, has, P)
    assert abs(wz) < 0.08 and vx > 0


# --- ground projection (MPPI path mode) ---

from rak_road_perception.road_follow_math import GOAL_DEFAULTS, extend_to_length, ground_points

G = {**GOAL_DEFAULTS}


def test_ground_projection_centered():
    m = band(np.zeros((H, W), np.uint8), 280, 360)
    pts, spin, has = run(m)
    gp = ground_points(pts, {**G, "image_width": W, "image_height": H})
    assert len(gp) >= 5
    ds = [p[0] for p in gp]
    assert all(b >= a for a, b in zip(ds, ds[1:]))      # further up = further away
    assert all(abs(p[1]) < 0.10 for p in gp)            # centered: near-zero lateral
    assert ds[0] > 0.1 and ds[-1] <= G["max_range_m"]


def test_ground_projection_right_road_is_negative_y():
    m = band(np.zeros((H, W), np.uint8), 380, 460)
    pts, spin, has = run(m)
    gp = ground_points(pts, {**G, "image_width": W, "image_height": H})
    assert all(p[1] < -0.02 for p in gp)                # right of center -> y<0


def test_extend_reaches_path_len():
    m = band(np.zeros((H, W), np.uint8), 280, 360)
    pts, spin, has = run(m)
    gp = extend_to_length(
        ground_points(pts, {**G, "image_width": W, "image_height": H}), G)
    total = float(np.sum(np.hypot(np.diff([p[0] for p in gp]),
                                  np.diff([p[1] for p in gp]))))
    assert total >= G["path_len_m"] - G["extend_step_m"]
