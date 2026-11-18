"""Centerline tracing + pure-pursuit control from a binary road mask.

numpy-only (no ROS, no cv2). Replaces the single-scanline follower:
- trace_centerline(): walks scanlines bottom->top, following the run closest to
  the current heading. Junctions keep the heading (wide merged run -> clamp to
  prediction), which IS the "路口默认直走" policy. Sharp bends are followed as
  far as the branch gate allows; a road that leaves the frame horizontally is
  recovered by spin_search.
- follow_command(): pure pursuit on the traced polyline. Lookahead point gives
  the lateral error; window slope gives heading; a quadratic fit over the
  polyline gives curvature -> speed shaping (slow into sharp turns).

Coordinates: x normalized [-0.5, 0.5] (camera x-right positive), y in pixels.
ROS convention wz>0 = turn left, so wz = -(k_lat*e + k_head*h).
"""

import numpy as np

TRACE_DEFAULTS = {
    "row_step_frac": 0.03,     # scanline stride as fraction of height
    "branch_gate": 0.20,       # max |run_center - predicted| per step (norm x)
    "gap_px": 12,              # max column gap inside one run
    "wide_run_frac": 0.45,     # run wider than this = junction: keep heading
    "bottom_search_frac": 0.25,  # band above bottom searched before spin mode
}

FOLLOW_DEFAULTS = {
    "max_vx": 0.12,
    "max_vy": 0.08,            # mecanum strafe cap (kept below vx: rollers slip)
    "max_wz": 0.5,
    "k_lateral": 0.7,          # wz share of lateral error (vy handles most)
    "k_vy": 0.9,               # strafe gain: e>0 (road right) -> vy<0 (go right)
    "k_heading": 0.9,
    "lookahead_frac": 0.70,    # pursuit target this far above the bottom row
    "curv_slow": 1.5,          # vx scale = 1 - curv_slow * normalized curvature
    "min_vx_scale": 0.4,
    "slow_lateral": 0.35,
    "spin_wz": 0.6,            # in-place turn rate when road left the bottom
    "spin_search": True,
}


def _runs(row_bool, gap_px):
    """Connected runs of a boolean row as (start, end_exclusive) pairs."""
    cols = np.flatnonzero(row_bool)
    if cols.size == 0:
        return []
    breaks = np.flatnonzero(np.diff(cols) > gap_px)
    edges = np.concatenate(([0], breaks + 1, [cols.size]))
    return [(int(cols[a]), int(cols[b - 1]) + 1) for a, b in zip(edges[:-1], edges[1:])]


def trace_centerline(mask, params):
    """Trace the road center bottom->top.

    Returns (points, spin_target, has_bottom):
    points = [(y, x_norm)] bottom->top along ONE branch; spin_target = (y,
    x_norm) when the road left the bottom band and only exists higher up
    (None otherwise); has_bottom = a run was found in the bottom band.
    """
    h, w = mask.shape[:2]
    if h == 0 or w == 0:
        return [], None, False
    road = mask > 127
    step = max(1, int(h * params["row_step_frac"]))
    gap = int(params["gap_px"])
    gate = float(params["branch_gate"])
    wide = float(params["wide_run_frac"]) * w

    def run_centers(y):
        out = []
        for a, b in _runs(road[y], gap):
            out.append(((a + b) / 2.0 / w - 0.5, b - a))
        return out

    # Bottom band: road must be present under the robot, else spin-search.
    band0 = max(0, h - max(1, int(h * params["bottom_search_frac"])))
    bottom = None
    for y in range(h - 1, band0 - 1, -1):
        cs = run_centers(y)
        if cs:
            bottom = min(cs, key=lambda c: abs(c[0]))
            break
    if bottom is None:
        if not params.get("spin_search", True):
            return [], None, False
        for y in range(band0 - 1, -1, -1):
            cs = run_centers(y)
            if cs:
                target = min(cs, key=lambda c: abs(c[0]))
                return [], (y, target[0]), False
        return [], None, False

    points = []
    y0 = y
    x = bottom[0]
    points.append((y0, x))
    slope = 0.0
    for y in range(y0 - step, -1, -step):
        predicted = x + slope * step
        cs = run_centers(y)
        if not cs:
            continue  # gap (occlusion / mark noise): coast straight up
        best = min(cs, key=lambda c: abs(c[0] - predicted))
        if abs(best[0] - predicted) > gate:
            inside = [c for c in cs if abs(c[0] - predicted) <= c[1] / w / 2]
            if inside:  # merged junction run: keep heading, clamp inside
                best = min(inside, key=lambda c: abs(c[0] - predicted))
                new_x = float(np.clip(predicted, best[0] - best[1] / w / 2,
                                      best[0] + best[1] / w / 2))
            else:
                break  # branch gate failed: road turned away
        else:
            if best[1] > wide:
                new_x = float(np.clip(predicted, best[0] - best[1] / w / 2,
                                      best[0] + best[1] / w / 2))
            else:
                new_x = best[0]
        slope = 0.7 * slope + 0.3 * (new_x - x) / step
        x = new_x
        points.append((y, x))
    return points, None, True


def follow_command(points, spin_target, has_bottom, params):
    """Trace result -> (vx, vy, wz). Pure pursuit + mecanum strafe.

    Mecanum trait: kill the lateral offset with strafe vy (y-left positive,
    road right e>0 -> vy<0) while wz only aligns heading, so the robot glides
    back onto the road without spinning.
    """
    if not has_bottom:
        if spin_target is not None and params.get("spin_search", True):
            return 0.0, 0.0, float(np.clip(
                params["spin_wz"] * -np.sign(spin_target[1]),
                -params["max_wz"], params["max_wz"]))
        return 0.0, 0.0, 0.0
    if not points:
        return 0.0, 0.0, 0.0
    n_look = max(1, int(params["lookahead_frac"] / max(params["row_step_frac"], 1e-6)))
    look_i = min(n_look, len(points) - 1)
    look_x = points[look_i][1]
    e = float(np.clip(look_x, -0.6, 0.6))
    dy = max(1, points[0][0] - points[look_i][0])
    h = (look_x - points[0][1]) / dy
    curv = 0.0
    if len(points) >= 4:
        t = np.arange(len(points), dtype=float)
        xs = np.array([p[1] for p in points])
        # Normalize c2 by span^2 so curvature is scale-free across trace lengths.
        curv = abs(float(np.polyfit(t, xs, 2)[0])) * (len(points) - 1) ** 2
    vx_scale = float(np.clip(1.0 - params["curv_slow"] * curv,
                             params["min_vx_scale"], 1.0))
    vx = params["max_vx"] * vx_scale
    if abs(e) > params["slow_lateral"]:
        vx *= 0.5
    vy = float(np.clip(-params["k_vy"] * e,
                       -params["max_vy"], params["max_vy"]))
    wz = -(params["k_lateral"] * e + params["k_heading"] * h)
    wz = float(np.clip(wz, -params["max_wz"], params["max_wz"]))
    return float(vx), vy, wz


# --- Vision path -> metric ground path (FollowPath action, MPPI mode) ---

GOAL_DEFAULTS = {
    "image_width": 640,
    "image_height": 480,
    # ChArUco calibration 2026-11-15 at 1280x720 (vision_nav_v1.md §5A.7).
    "fx_1280": 844.40,
    "fy_1280": 851.46,
    "cx_1280": 614.20,
    "cy_1280": 401.18,
    # [AI] placeholder: camera mount height/pitch NOT measured yet. Scale error
    # is linear in height; measure with a tape and set before trusting speeds.
    "cam_height_m": 0.30,
    "cam_pitch_rad": 0.0,
    "max_range_m": 2.5,
    "horizon_min_dep_rad": 0.05,   # rows near/above the horizon project to inf
    "path_len_m": 1.8,             # total FollowPath arc length
    "extend_step_m": 0.15,
}


def ground_points(points, params):
    """Trace polyline (pixels) -> base-frame ground path [(forward, y_left)].

    Flat-ground pinhole projection: depression angle from the row, forward
    distance d = h / tan(dep), lateral from the calibrated fx. y_left follows
    ROS convention (left positive), so a road right of the image center is
    negative. Rows near/above the horizon are skipped (d -> infinity).
    """
    w = float(params["image_width"])
    h_img = float(params["image_height"])
    fx = params["fx_1280"] * w / 1280.0
    fy = params["fy_1280"] * h_img / 720.0
    cx = params["cx_1280"] * w / 1280.0
    cy = params["cy_1280"] * h_img / 720.0
    pitch = params["cam_pitch_rad"]
    out = []
    for y, xn in points:
        dep = float(np.arctan((y - cy) / fy)) + pitch
        if dep < params["horizon_min_dep_rad"]:
            continue
        d = params["cam_height_m"] / np.tan(dep)
        d = float(min(d, params["max_range_m"]))
        x_px = (xn + 0.5) * w          # x_norm is centered; convert to pixels
        y_left = -(x_px - cx) * d / fx
        out.append((d, float(y_left)))
    return out


def extend_to_length(pts, params):
    """Extend a base-frame path with dead reckoning to path_len_m arc length."""
    if not pts:
        return pts
    out = list(pts)
    step = params["extend_step_m"]
    if len(out) >= 2:
        dx = out[-1][0] - out[-2][0]
        dy = out[-1][1] - out[-2][1]
        norm = float(np.hypot(dx, dy)) or step
        dx, dy = dx / norm, dy / norm
    else:
        dx, dy = 1.0, 0.0
    total = float(np.sum(np.hypot(np.diff([p[0] for p in out]),
                                  np.diff([p[1] for p in out]))))
    for _ in range(64):
        if total >= params["path_len_m"]:
            break
        out.append((out[-1][0] + dx * step, out[-1][1] + dy * step))
        total += step
    return out
