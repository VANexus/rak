"""Pure-math checks for road_follow_math (no ROS, numpy only)."""

import numpy as np

from rak_road_perception.road_follow_math import (
    CORNER_DEFAULTS,
    FOLLOW_DEFAULTS,
    TRACE_DEFAULTS,
    corner_signal,
    follow_command,
    render_trace_debug,
    trace_centerline,
)

W, H = 640, 480
P = {**TRACE_DEFAULTS, **FOLLOW_DEFAULTS, **CORNER_DEFAULTS}


def band(m, x0, x1, y0=0, y1=H):
    m[y0:y1, x0:x1] = 255
    return m


def run(mask):
    return trace_centerline(mask, P)


def test_straight_centered():
    # Geometry test: lane at the IMAGE centre, so the camera-mount bias is
    # switched off explicitly. With the measured bias in play a lane at the
    # image centre is 0.032 off the robot's axis, which is a different case
    # (see test_lane_center_bias_removes_the_constant_offset).
    p = {**P, "lane_center_bias": 0.0}
    m = band(np.zeros((H, W), np.uint8), 280, 360)
    pts, spin, has = trace_centerline(m, p)
    assert has and spin is None and len(pts) > 10
    vx, vy, wz = follow_command(pts, spin, has, p)
    assert abs(pts[-1][1]) < 0.02  # keeps center all the way up
    assert abs(wz) < 0.05 and abs(vy) < 0.01 and abs(vx - p["max_vx"]) < 1e-6


def test_offset_road_strafes_right():
    # Road to the RIGHT of heading -> steer right. Since 2026-11-19 lateral
    # correction is YAW-ONLY (k_vy=0): mecanum strafe was measured not to
    # translate on the competition mat, so vy must stay at zero.
    m = band(np.zeros((H, W), np.uint8), 380, 460)
    pts, spin, has = run(m)
    vx, vy, wz = follow_command(pts, spin, has, P)
    assert abs(vy) < 1e-9, f"strafe must stay disabled, got vy={vy:+.4f}"
    assert wz < -0.03, f"should yaw right toward the road, got wz={wz:+.4f}"
    assert vx > 0


def test_sharp_gradual_bend_followed_and_slowed():
    # Road curves left quadratically above y=320 (145deg-style bend).
    m = np.zeros((H, W), np.uint8)
    for y in range(H):
        cx = 320 - int(180 * (max(0, 320 - y) / 320) ** 2)
        band(m, cx - 40, cx + 40, y, y + 1)
    pts, spin, has = run(m)
    assert has and len(pts) > 20 and pts[-1][1] < -0.2  # trace reaches the bend
    vx, vy, wz = follow_command(pts, spin, has, P)
    # Corrective LEFT action. Yaw only -- strafe is disabled by default.
    assert wz > 0.05, f"should yaw left, got wz={wz:+.4f}"
    assert abs(vy) < 1e-9
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


# --- real-camera regressions (2026-11-18 live mask on open floor) ---

def test_open_floor_goes_straight_not_to_side_run():
    # Measured case: bottom row == two runs (24..265 and 423..631) split by a
    # 158 px floor seam; row coverage 0.83 of the width. The trace must hold the
    # optical axis, NOT the center of either edge run (was bottom_x=-0.28).
    m = np.zeros((H, W), np.uint8)
    band(m, 24, 266, H - 2, H)
    band(m, 423, 632, H - 2, H)
    band(m, 22, 638, 0, H - 4)          # open floor above
    pts, spin, has = run(m)
    assert has and pts
    assert abs(pts[0][1]) < 0.05 and abs(pts[-1][1]) < 0.05


def test_narrow_seam_does_not_split_lane():
    # Lane split by a 20 px shadow: bridged into ONE run, so the seam cannot
    # become a second "branch" the gate has to reject.
    m = np.zeros((H, W), np.uint8)
    band(m, 200, 430, 0, H)
    m[:, 310:330] = 0
    pts, spin, has = run(m)
    assert has and abs(pts[0][1]) < 0.05      # ~0.0 center, not the left run




# --- corner judgement (2026-11-19) -----------------------------------------
#
# Execution is stock ROS 2 (nav2_rotation_shim_controller + nav2_mppi_controller).
# What this module owns is the PATH: `trace_centerline` must append the outgoing
# arm so the polyline actually turns. These tests pin that, and nothing about
# wheel speeds or in-place rotation -- those are no longer our code.
#
# Geometry: the robot sits in a vertical corridor (bottom of frame), the
# corridor ENDS partway up, and the only way on is a narrow horizontal arm on
# one side -- which is what a square corner looks like from a forward camera.

def _corner_mask(side):
    m = np.zeros((H, W), np.uint8)
    m[300:H, 260:380] = 255          # incoming corridor under the robot
    if side == "left":
        m[0:300, 40:170] = 255       # outgoing arm, far away and to one side
    else:
        m[0:300, 470:600] = 255
    return m


def test_corner_signal_left_and_right():
    for side in ("left", "right"):
        m = _corner_mask(side)
        pts, _spin, has = run(m)
        assert has and pts
        # The trace without the arm ends early; feed that row to the detector.
        base = [p for p in pts if p[0] > H * P["corner_min_depth_frac"]]
        assert base, "the straight part must stop early"
        direction, _score = corner_signal(m, max(p[0] for p in base), 0.0, P)
        assert direction == side, (side, direction)


def test_corner_signal_gentle_bend_is_straight():
    m = np.zeros((H, W), np.uint8)
    for y in range(H):
        c = int(320 + 0.15 * (H - y))
        m[y, c - 50:c + 50] = 255
    pts, _spin, has = run(m)
    assert has and pts
    assert pts[-1][0] <= H * P["corner_min_depth_frac"], "bend must reach the far rows"


def test_path_turns_into_a_left_corner():
    pts, _spin, has = run(_corner_mask("left"))
    assert has and pts
    assert pts[-1][0] < H * P["corner_min_depth_frac"], "arm must be appended"
    assert pts[-1][1] < -0.2, f"last point must sit left, got {pts[-1][1]:.3f}"


def test_path_turns_into_a_right_corner():
    pts, _spin, has = run(_corner_mask("right"))
    assert has and pts
    assert pts[-1][0] < H * P["corner_min_depth_frac"]
    assert pts[-1][1] > 0.2, f"last point must sit right, got {pts[-1][1]:.3f}"


def test_straight_corridor_is_not_bent():
    m = band(np.zeros((H, W), np.uint8), 280, 360)
    pts, _spin, has = run(m)
    assert has and pts
    assert all(abs(p[1]) < 0.05 for p in pts)


def test_corner_follow_can_be_disabled():
    from rak_road_perception.road_follow_math import trace_centerline
    pts, _spin, has = trace_centerline(
        _corner_mask("left"), {**P, "corner_follow": False})
    assert has and pts
    assert pts[-1][0] > H * P["corner_min_depth_frac"], "no arm when disabled"
    assert abs(pts[-1][1]) < 0.1


def test_turn_policy_overrides_a_symmetric_junction():
    from rak_road_perception.road_follow_math import corner_signal, trace_centerline
    # A T junction: both arms present and comparable, so `auto` must NOT guess.
    tee = np.zeros((H, W), np.uint8)
    tee[300:H, 260:380] = 255
    tee[0:300, 20:150] = 255
    tee[0:300, 470:600] = 255
    assert corner_signal(tee, 297, 0.0, P)[0] == "ambiguous"
    pts, _spin, has = trace_centerline(tee, P)
    assert has and pts
    assert abs(pts[-1][1]) < 0.1, "auto must not invent a branch at a T"
    # An explicit policy resolves it deterministically.
    for side, expected in (("left", -1), ("right", 1)):
        pts, _spin, has = trace_centerline(tee, {**P, "turn_policy": side})
        assert has and pts
        assert pts[-1][1] * expected > 0.2, f"policy {side} did not win: {pts[-1][1]:.3f}"


def test_turn_policy_does_not_invent_missing_road():
    from rak_road_perception.road_follow_math import trace_centerline
    # Right corner, policy says left: there is no left arm to follow, so the
    # path must stay honest instead of drawing a road that is not there.
    pts, _spin, has = trace_centerline(
        _corner_mask("right"), {**P, "turn_policy": "left"})
    assert has and pts
    assert abs(pts[-1][1]) < 0.1


def test_render_trace_debug_marks_the_line():
    m = band(np.zeros((H, W), np.uint8), 280, 360)
    pts, _spin, _has = run(m)
    img = render_trace_debug(m, pts)
    assert img.shape == (H, W) and img.dtype == np.uint8
    assert img.max() == 255
    assert img.min() == 0


# --- perspective regressions (2026-11-19, real 45 cm competition lane) ------
#
# Both rules below were absolute fractions of frame width, which silently
# assumed a camera geometry this robot does not have. Measured on a real frame
# from the mat: bottom-row road coverage 0.994, and the lane's own edges converge
# from (0.02, 0.99) at y=479 to (0.44, 0.56) at y=71. A 45 cm lane seen from
# ~0.3 m fills the near field, so:
#   * open_frac alone called it "open floor" -> trace pinned to x=0 -> e == 0
#     forever -> the controller never corrects (the reported drift).
#   * wide_run_frac alone called rows 215..455 a junction -> "keep heading"
#     -> straight driving was treated as a junction traversal.

def _perspective_lane(x_left=0.0, taper=0.00125, rows=(479, 60)):
    """A lane whose edges converge going up, like the real mat frame."""
    m = np.zeros((H, W), np.uint8)
    for y in range(rows[0], rows[1] - 1, -1):
        f = (rows[0] - y) / (rows[0] - rows[1])
        l = (0.02 + x_left) + f * taper * y
        r = (0.99 + x_left) - f * taper * y
        a, b = int(l * W), int(r * W)
        if b > a:
            m[y, max(0, a):min(W, b)] = 255
    return m


def test_narrow_lane_is_not_open_floor():
    m = _perspective_lane()
    pts, spin, has = run(m)
    assert has and pts
    # The old single-row rule pinned EVERY point to the optical axis.
    assert _coverage(m, 479) > P["open_frac"], "fixture must saturate the old rule"
    assert not all(abs(p[1]) < 1e-9 for p in pts), "trace must not be pinned to x=0"


def test_narrow_lane_corrects_a_lateral_offset():
    # Offset-response test: bias off so the shift is the only lateral error.
    p = {**P, "lane_center_bias": 0.0}
    for shift in (0.04, 0.08, 0.12):
        m = _perspective_lane(x_left=shift)
        pts, spin, has = trace_centerline(m, p)
        cmd = follow_command(pts, spin, has, p)
        assert has and pts
        # road sits to the right -> steer right; strafe is disabled by default
        assert abs(cmd[1]) < 1e-9, (shift, cmd)
        assert cmd[2] < -0.01, (shift, cmd)


def test_offset_response_is_monotonic():
    wzs = []
    for shift in (0.0, 0.03, 0.06, 0.09):
        m = _perspective_lane(x_left=shift)
        pts, spin, has = run(m)
        _vx, _vy, wz = follow_command(pts, spin, has, P)
        wzs.append(wz)
    assert all(b < a for a, b in zip(wzs, wzs[1:])), f"not monotonic: {wzs}"


def test_straight_lane_near_field_is_not_a_junction():
    # The near field of a real lane is frame-saturating; it must still be
    # tracked as a corridor (following its centre), not clamped as a junction.
    m = _perspective_lane()
    pts, spin, has = run(m)
    assert has and len(pts) > 15
    # A corridor converging to the frame centre must be followed to the centre.
    assert abs(pts[-1][1]) < 0.10, f"far end should approach centre, got {pts[-1][1]:.3f}"


def test_real_junction_is_still_a_junction():
    # A T on top of a corridor: the cross bar is much wider than the corridor
    # we are in, so the relative test must still fire.
    m = np.zeros((H, W), np.uint8)
    for y in range(300, H):
        m[y, 260:380] = 255
    m[0:300, 20:620] = 255
    pts, spin, has = run(m)
    assert has and pts
    assert abs(pts[-1][1]) < 0.25, "junction policy keeps heading, not chase an edge"


def _coverage(mask, y):
    cols = np.flatnonzero(mask[y] > 127)
    return 0.0 if cols.size == 0 else (cols[-1] - cols[0] + 1) / mask.shape[1]


# --- projection layer (2026-11-19) -----------------------------------------
# ground_points used to CLAMP far points to max_range_m while keeping their
# lateral offset, and extend_to_length used the last two points as heading.
# Together a corner arm became "forward 0, sideway 1.2 m" -- a path the MPPI
# controller would either refuse or drive into a wall.

from rak_road_perception.road_follow_math import (  # noqa: E402
    GOAL_DEFAULTS, extend_to_length, ground_points)

G = {**GOAL_DEFAULTS}


def _arc(pts):
    return sum(float(np.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]))
               for i in range(1, len(pts)))


def test_ground_points_are_monotone_in_forward_distance():
    m = _perspective_lane()
    pts, _spin, has = run(m)
    assert has and pts
    gp = ground_points(pts, G)
    assert len(gp) >= 5
    d = [p[0] for p in gp]
    assert all(b >= a - 1e-9 for a, b in zip(d, d[1:])), d


def test_ground_points_drop_out_of_range_instead_of_clamping():
    # Every kept point must be within range; nothing may pile up at max_range.
    m = _perspective_lane()
    pts, _spin, _has = run(m)
    gp = ground_points(pts, {**G, "max_range_m": 0.8})
    assert gp, "still needs points inside the reduced range"
    assert all(p[0] <= 0.8 + 1e-9 for p in gp)
    assert sum(1 for p in gp if abs(p[0] - 0.8) < 1e-9) == 0, "clamped, not dropped"


def test_extend_does_not_turn_a_saturated_tail_into_a_lateral_jump():
    # A tail of identical points has zero difference; the heading must come
    # from the last pair that actually moves.
    pts = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.2, 0.0), (1.2, 0.0)]
    out = extend_to_length(pts, {**G, "path_len_m": 2.0, "extend_step_m": 0.15})
    assert _arc(out) >= 2.0 - 0.15
    lateral = abs(out[-1][1] - out[0][1])
    assert lateral < 0.05, f"extended sideways: {lateral:.3f} m"


def test_extend_reaches_requested_length():
    pts = [(0.0, 0.0), (0.4, 0.0), (0.8, 0.1)]
    out = extend_to_length(pts, {**G, "path_len_m": 1.8, "extend_step_m": 0.15})
    assert _arc(out) >= 1.8 - 0.15
    assert abs(out[-1][1] - 0.1) < 0.6, "should continue roughly along the last heading"


def test_corner_path_goes_forward_and_sideways():
    m = np.zeros((H, W), np.uint8)
    m[300:H, 260:380] = 255
    m[0:300, 40:170] = 255
    pts, _spin, has = run(m)
    gp = ground_points(pts, G)
    ext = extend_to_length(gp, G)
    assert ext
    forward = ext[-1][0] - ext[0][0]
    lateral = ext[-1][1] - ext[0][1]
    assert forward > 1.0, f"corner path must advance, got {forward:.2f} m"
    assert abs(lateral) > 0.3, f"corner path must turn, got {lateral:.2f} m"


# --- camera intrinsics at the operating resolution (2026-11-19) ------------
# The 640x480 mode is a horizontal CROP of the 1280x720 calibration, not a
# uniform scale: feature matching on the same scene scored NCC 0.988 for
# "scale by 480/720 then centre-crop the width" vs 0.326 for a plain stretch.
# Scaling fx by 640/1280 (the obvious-looking thing) is 33% low and shrinks
# every projected distance by the same factor -- silently, since the direct
# follower never uses fx at all.

from rak_road_perception.road_follow_math import intrinsics  # noqa: E402


def test_intrinsics_honour_the_crop_model():
    fx, fy, cx, cy = intrinsics(G)
    assert abs(fy - 851.46 * 480 / 720) < 0.01
    assert abs(cy - 401.18 * 480 / 720) < 0.01
    # fx must NOT be 844.40 * 640/1280 == 422.2
    naive = 844.40 * 640 / 1280
    assert fx > naive * 1.3, f"fx={fx:.1f} looks like the naive uniform scale"
    assert abs(fx - 844.40 * 480 / 720) < 0.01


def test_intrinsics_principal_point_follows_the_crop():
    _fx, _fy, cx, _cy = intrinsics(G)
    # cx shifts left by half the cropped width, then scales
    calib_crop = (1280 - 640 / (480 / 720)) / 2
    expected = (614.20 - calib_crop) * 480 / 720
    assert abs(cx - expected) < 0.01


def test_intrinsics_scale_with_resolution():
    small = intrinsics({**G, "image_width": 320, "image_height": 240,
                        "crop_to_width": 320})
    full = intrinsics(G)
    for a, b in zip(small, full):
        assert abs(a * 2 - b) < 0.02, (small, full)


# --- camera mount yaw bias (2026-11-19) ------------------------------------
# With the robot parked dead centre on a straight lane the trace settles at
# x_norm = +0.032 (6 frames, spread 0.0016): ~2.1 deg of camera-mount yaw.
# Uncorrected, the controller chases that constant offset forever and settles
# off-centre -- the reported "never reaches the middle of the lane".

def test_lane_center_bias_removes_the_constant_offset():
    bias = P["lane_center_bias"]
    assert bias > 0.0, "the measured bias must be wired in"
    # A lane drawn exactly at the measured bias is the robot's own centre.
    m = np.zeros((H, W), np.uint8)
    cx_px = int((0.5 + bias) * W)
    for y in range(H):
        m[y, max(0, cx_px - 60):min(W, cx_px + 60)] = 255
    pts, spin, has = run(m)
    cmd = follow_command(pts, spin, has, P)
    assert abs(cmd[1]) < 0.01, f"strafe should be ~0, got {cmd[1]:+.4f}"
    assert abs(cmd[2]) < 0.02, f"yaw should be ~0, got {cmd[2]:+.4f}"


def test_lane_center_bias_is_one_constant_for_both_control_modes():
    from rak_road_perception.road_follow_math import LANE_CENTER_BIAS
    assert FOLLOW_DEFAULTS["lane_center_bias"] == LANE_CENTER_BIAS
    assert GOAL_DEFAULTS["lane_center_bias"] == LANE_CENTER_BIAS


def test_bias_shifts_the_ground_path_back_to_the_axis():
    m = np.zeros((H, W), np.uint8)
    cx_px = int((0.5 + P["lane_center_bias"]) * W)
    for y in range(H):
        m[y, max(0, cx_px - 60):min(W, cx_px + 60)] = 255
    pts, _spin, _has = run(m)
    gp = ground_points(pts, G)
    assert gp
    lateral = max(abs(p[1]) for p in gp)
    assert lateral < 0.10, f"path should sit on the axis, max |y|={lateral:.3f} m"


# --- command de-spiking (2026-11-19) ---------------------------------------
# One bad segmentation frame swung wz from 0.015 to -0.097 while cruising the
# real mat (printed colour blocks). A median over recent commands removes it.

from rak_road_perception.road_follow_math import despike_command  # noqa: E402


def test_despike_rejects_a_single_frame_spike():
    h = []
    for c in [(0.06, 0.0, 0.015), (0.06, 0.0, 0.016), (0.06, 0.0, -0.097)]:
        h.append(c)
        out = despike_command(h, 3)
    # median of [0.015, 0.016, -0.097] is 0.015: the spike is gone
    assert abs(out[2]) < 0.05, f"spike leaked through: {out}"
    assert out[2] != -0.097


def test_despike_passes_a_real_change_through():
    h = []
    for c in [(0.06, 0.0, 0.02), (0.06, 0.0, 0.05), (0.06, 0.0, 0.09)]:
        h.append(c)
        out = despike_command(h, 3)
    assert abs(out[2] - 0.05) < 1e-9, out
    h.append((0.06, 0.0, 0.12))
    assert abs(despike_command(h, 3)[2] - 0.09) < 1e-9


def test_despike_does_not_invent_a_command_before_the_window_fills():
    h = [(0.06, 0.0, 0.1)]
    assert despike_command(h, 3) == (0.06, 0.0, 0.1)
    h.append((0.05, 0.0, 0.2))
    assert despike_command(h, 3) == (0.05, 0.0, 0.2)


def test_despike_window_one_is_a_passthrough():
    h = [(0.06, 0.0, 0.1), (0.06, 0.0, -0.9)]
    assert despike_command(h, 1) == (0.06, 0.0, -0.9)


def test_lookahead_stays_inside_the_trustworthy_band():
    # Measured on 8 real frames from the competition mat (2026-11-19): command
    # noise vs lookahead was 0.0004 (0.20), 0.0022 (0.40), 0.0013 (0.50) and
    # then jumps to 0.0208 (0.60) / 0.0195 (0.70) because the mat's printed
    # artwork contains roads the segmenter also detects. Above ~0.55 the
    # lookahead sits in that garbage. This test exists so nobody tunes it back.
    assert P["lookahead_frac"] <= 0.55, (
        f"lookahead_frac={P['lookahead_frac']} is in the noisy upper band")


def test_bend_still_produces_meaningful_yaw():
    m = np.zeros((H, W), np.uint8)
    for y in range(H):
        cx = 320 - int(180 * (max(0, 320 - y) / 320) ** 2)
        band(m, cx - 40, cx + 40, y, y + 1)
    pts, spin, has = run(m)
    vx, vy, wz = follow_command(pts, spin, has, P)
    assert wz > 0.04, f"bend must still command a real turn, got wz={wz:+.4f}"
    assert abs(vy) < 1e-9
    assert vx < P["max_vx"]
