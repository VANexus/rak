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

# Camera-mount yaw, expressed as a constant offset of the traced lane centre in
# normalised image x. MEASURED 2026-11-19 with the robot parked dead centre and
# square on a straight competition lane: the trace settles at x_norm = +0.032
# (6 frames, spread 0.0016), i.e. ~2.1 deg of camera yaw.
#
# ONE definition on purpose: the direct follower and the MPPI path builder both
# subtract this, so the two control modes can never disagree about where the
# lane centre is. Re-measure after any camera re-mount -- the procedure is in
# docs/how-to/road_follow_split.md (park centred, read the stable trace x).
LANE_CENTER_BIAS = 0.032

TRACE_DEFAULTS = {
    "row_step_frac": 0.03,     # scanline stride as fraction of height
    "branch_gate": 0.20,       # max |run_center - predicted| per step (norm x)
    "gap_px": 12,              # max column gap inside one run
    "bridge_frac": 0.30,       # merge runs split by < this frac of width (shadow/seam)
    "open_frac": 0.62,         # bottom-row coverage above this is a CANDIDATE for open floor
    "open_taper_min": 0.85,    # ...and coverage at open_probe_frac must stay this close to it
    "open_probe_frac": 0.45,   # mid-frame row (fraction of height) the taper is read at
    "wide_run_frac": 0.45,     # DEPRECATED absolute test, kept only as a floor
    "junction_gain": 1.7,      # run wider than gain x the CURRENT corridor width = opening
    "corridor_ema": 0.6,       # smoothing on the running corridor-width estimate
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
    # See LANE_CENTER_BIAS: a constant image-space offset from a yawed camera
    # mount. Left uncorrected, the controller chases it forever and settles
    # off-centre -- exactly the reported "never reaches the middle of the lane".
    "lane_center_bias": LANE_CENTER_BIAS,
}

# --- Corner handling: this file's job is the PATH, not the motion ----------
#
# 2026-11-19 decision (user): the execution layer is stock ROS 2. Turning,
# in-place rotation and path tracking belong to
# nav2_rotation_shim_controller + nav2_mppi_controller, NOT to a hand written
# controller here. What has no stock equivalent is "which way did the road go",
# so that judgement lives in this module and its ONLY output is a polyline that
# actually turns the corner.
#
# The 2026-11-19 field symptoms map like this:
#   * "cannot take a corner"        -> RotationShim rotates in place; nothing here.
#   * "no left/right at a corner"   -> `corner_signal()` + `_corner_arm()` (below).
#   * "never settles in the centre" -> MPPI (optimizer) instead of a P loop.
CORNER_DEFAULTS = {
    # A corner is declared only when the trace STOPPED EARLY (it did not reach
    # the upper part of the frame) AND the far band puts the road mass on one
    # side.  The early-stop test is what keeps a gentle bend (trace reaches the
    # top) from being mistaken for a corner.
    "corner_min_depth_frac": 0.35,  # trace end above this row = it went far
    "corner_band_frac": 0.35,       # far band = top this fraction of the frame
    "corner_center_frac": 0.16,     # |x - x_ref| < this = "straight ahead" column
    "corner_mass_ratio": 1.8,       # side must beat the other by this factor
    "corner_min_mass": 0.05,        # and carry at least this road fraction
    "turn_policy": "auto",          # auto | left | right | straight (T junctions)
    "corner_follow": True,          # append the outgoing arm to the polyline
    "corner_arm_max_rows": 24,      # cap on appended arm points
}


def _runs(row_bool, gap_px, bridge_px=0):
    """Connected runs of a boolean row as (start, end_exclusive) pairs.

    Runs separated by <= bridge_px are merged: a shadow, floor seam or dark
    patch must not split one lane into two fake branches (measured on the real
    camera 2026-11-18: the bottom row came back as two runs split by a 158 px
    seam, and the run centers pulled the trace to the left edge of the frame).
    """
    cols = np.flatnonzero(row_bool)
    if cols.size == 0:
        return []
    split = np.diff(cols) > max(gap_px, bridge_px)
    edges = np.concatenate(([0], np.flatnonzero(split) + 1, [cols.size]))
    return [(int(cols[a]), int(cols[b - 1]) + 1) for a, b in zip(edges[:-1], edges[1:])]


def _coverage_taper(road, y_bottom, probe_frac):
    """Coverage(y_bottom) -> coverage(mid row) ratio, capped at the top.

    Returns `coverage(probe_row) / coverage(y_bottom)`. Open floor: ~1 or more
    (the whole frame is road at every row). A lane under perspective: well
    below 1, because the same physical width subtends fewer columns higher up.
    Uses the SPAN (first..last road column) rather than the sum of runs, so a
    lane split by a seam still reads as one corridor.
    """
    h = road.shape[0]
    base = np.flatnonzero(road[int(y_bottom)])
    if base.size == 0:
        return 0.0
    span_bottom = float(base[-1] - base[0] + 1)
    y_probe = int(np.clip(h * float(probe_frac), 0, h - 1))
    probe = np.flatnonzero(road[y_probe])
    if probe.size == 0:
        return 0.0
    return float(min(1.5, (probe[-1] - probe[0] + 1) / max(span_bottom, 1.0)))


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
    bridge = int(float(params.get("bridge_frac", 0.0)) * w)
    gate = float(params["branch_gate"])
    wide = float(params["wide_run_frac"]) * w

    def run_centers(y):
        out = []
        for a, b in _runs(road[y], gap, bridge):
            out.append(((a + b) / 2.0 / w - 0.5, b - a))
        return out

    # Bottom band: road must be present under the robot (image bottom center), else
    # spin-search. Picking the run CONTAINING the center column is the robot's own
    # lane; picking the run "nearest to center" instead grabs any narrow blob off to
    # a side (measured 2026-11-18 on the real camera: a wide open floor produced a
    # tiny edge run whose center beat the real lane and pulled the trace to x=-0.33).
    band0 = max(0, h - max(1, int(h * params["bottom_search_frac"])))
    bottom = None
    open_row = False
    center_row = None
    for y in range(h - 1, band0 - 1, -1):
        cs = run_centers(y)
        if not cs:
            continue
        here = [c for c in cs if abs(c[0]) <= c[1] / w / 2]
        far = min(cs, key=lambda c: abs(c[0]))
        if bottom is None:
            bottom = (y, here[0] if here else far)
        if here:                      # the robot's own lane is under the center
            center_row = (y, here[0], cs)
            break
    if center_row is not None:
        y, run, cs = center_row
        coverage = sum(c[1] for c in cs) / w
        # Open floor vs narrow lane CANNOT be decided from one row's coverage.
        # The 2026-11-18 rule (`coverage > open_frac`) was measured on an open
        # warehouse floor and silently mis-fires on the 45 cm competition lane:
        # a real frame from the mat (2026-11-19) has bottom-row coverage 0.994
        # while the same road is only 0.29 wide at mid-frame, and the rule
        # pinned the whole trace to the optical axis (e == 0 forever -> the
        # controller never corrects -> the drift the user reported).
        # Perspective is the discriminator that needs no calibration: an open
        # floor stays near full-width up to mid-frame, a lane tapers away.
        taper = _coverage_taper(road, y, float(params.get("open_probe_frac", 0.45)))
        if (coverage >= float(params.get("open_frac", 1.1))
                and taper >= float(params.get("open_taper_min", 0.85))):
            # Almost all of the row is road AND it stays road up to mid-frame:
            # open floor, no lane structure. Anchor on the optical axis and
            # drive straight instead of chasing whichever edge is nearer the
            # center -- measured 2026-11-18 on an open floor.
            open_row = True
            bottom = (y, (0.0, run[1]))
        else:
            bottom = (y, run)
    y0 = None
    if bottom is not None:
        y0, bottom = bottom
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
    x = bottom[0]
    points.append((y0, x))
    slope = 0.0
    # Running estimate of the corridor's own width, seeded from the bottom run.
    # `wide_run_frac` alone was measured (2026-11-19, real 45 cm lane) to label
    # rows 215..455 -- the whole near field of a STRAIGHT lane -- as a junction,
    # because a close lane fills the frame. Under perspective a corridor narrows
    # monotonically going up, so "wide" only means wide RELATIVE to the corridor
    # we are already in. That is what a junction actually is.
    corridor = float(bottom[1]) or (float(wide) / 2.0)
    ema = float(params.get("corridor_ema", 0.6))
    gain = float(params.get("junction_gain", 1.7))
    for y in range(y0 - step, -1, -step):
        predicted = x + slope * step
        cs = run_centers(y)
        if not cs:
            continue  # gap (occlusion / mark noise): coast straight up
        best = min(cs, key=lambda c: abs(c[0] - predicted))
        opening = best[1] > max(gain * corridor, wide)
        if abs(best[0] - predicted) > gate:
            inside = [c for c in cs if abs(c[0] - predicted) <= c[1] / w / 2]
            if inside and opening:  # merged junction run: keep heading, clamp inside
                best = min(inside, key=lambda c: abs(c[0] - predicted))
                new_x = float(np.clip(predicted, best[0] - best[1] / w / 2,
                                      best[0] + best[1] / w / 2))
            else:
                break  # branch gate failed: road turned away
        else:
            if opening:
                new_x = float(np.clip(predicted, best[0] - best[1] / w / 2,
                                      best[0] + best[1] / w / 2))
            else:
                new_x = best[0]
                corridor = ema * corridor + (1.0 - ema) * best[1]
        if open_row:
            new_x = 0.0   # open floor: hold the optical axis, no lateral chase
        slope = 0.7 * slope + 0.3 * (new_x - x) / step
        x = new_x
        points.append((y, x))

    # Corner: the walk above stopped early because the corridor ended. Decide
    # which way the road went and append the outgoing arm, so the polyline the
    # planner/controller receives actually turns.  Execution (rotation in place,
    # path tracking) is NOT done here -- see nav2_rotation_shim_controller.
    if (params.get("corner_follow", True) and points
            and points[-1][0] > h * float(params["corner_min_depth_frac"])):
        direction, _score = corner_signal(mask, points[-1][0], points[-1][1], params)
        policy = str(params.get("turn_policy", "auto")).lower()
        if policy in ("left", "right"):
            direction = policy
        elif policy == "straight":
            direction = "straight"
        if direction in ("left", "right"):
            arm = _corner_arm(run_centers, points[-1][0], points[-1][1],
                              direction, step, int(params.get("corner_arm_max_rows", 24)))
            points.extend(arm)
    return points, None, True


def _corner_arm(run_centers, y_from, x_from, direction, step, max_rows):
    """Follow the outgoing arm of a corner upward, on the declared side.

    `direction` is "left"/"right" in camera convention (x-right positive), so a
    left arm has run centres strictly below the JUNCTION x.  That reference is
    the fixed `x_from`, not the moving x: a real outgoing arm recedes at a
    roughly constant lateral offset, so measuring "on the same side" against the
    previous point filters out the arm itself one step after the first
    (measured 2026-11-19: the arm returned exactly one point and the path never
    turned).

    Returns a list of (y, x_norm) continuing from `y_from` upward; empty when
    the arm is not actually there (the caller then keeps the straight trace).
    """
    side = -1.0 if direction == "left" else 1.0
    out = []
    x = x_from
    for y in range(int(y_from) - step, -1, -step):
        runs = run_centers(y)
        on_side = [c for c in runs if (c[0] - x_from) * side > 0.0]
        if not on_side:
            break
        best = min(on_side, key=lambda c: abs(c[0] - x))
        x = best[0]
        out.append((y, x))
        if len(out) >= max_rows:
            break
    return out


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
    bias = float(params.get("lane_center_bias", 0.0))
    look_x = points[look_i][1] - bias
    e = float(np.clip(look_x, -0.6, 0.6))
    # A constant bias cancels in the difference, so the heading term is unbiased.
    dy = max(1, points[0][0] - points[look_i][0])
    h = (points[look_i][1] - points[0][1]) / dy
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


def corner_signal(mask, y_stop, x_ref, params):
    """Left/right/straight discriminator over the band ABOVE the trace end.

    The reference column is `x_ref` (the traced bottom-run centre), not the
    image centre: it is the corridor the robot is actually in, so a lane offset
    does not fake a ninth-of-frame asymmetry.

    Returns (direction, score):
      direction in {"straight", "left", "right", "ambiguous"}
      score = dominant_side_mass / other_side_mass (0 when not applicable)

    "straight" means the road still covers the column band in front of the
    robot, i.e. the corridor continues.  That is the case that must NOT be
    turned into a corner -- a gentle bend reaches the top of the frame and has
    road in the forward band, so it never gets here anyway.
    """
    h, w = mask.shape[:2]
    band_end = int(min(h, max(4, h * float(params.get("corner_band_frac", 0.35)))))
    band_end = int(min(band_end, max(4, y_stop)))
    band = mask[:band_end] > 127
    col = band.mean(axis=0)                      # road fraction per column

    cc = float(params.get("corner_center_frac", 0.16))
    lo = int(np.clip((x_ref + 0.5 - cc) * w, 0, w))
    hi = int(np.clip((x_ref + 0.5 + cc) * w, 0, w))
    if hi - lo < 4 or lo < 4 or w - hi < 4:
        return "ambiguous", 0.0

    def region(a, b):
        return float(col[a:b].mean()) if b > a else 0.0

    straight = region(lo, hi)
    left = region(0, lo)
    right = region(hi, w)
    min_mass = float(params.get("corner_min_mass", 0.05))
    ratio = float(params.get("corner_mass_ratio", 1.8))

    if straight >= max(left, right) or straight >= min_mass:
        return "straight", 0.0
    if left >= min_mass and left >= right * ratio:
        return "left", left / max(right, 1e-6)
    if right >= min_mass and right >= left * ratio:
        return "right", right / max(left, 1e-6)
    return "ambiguous", 0.0



def render_trace_debug(mask, points):
    """Mask (dimmed) with the traced centreline burned in, as mono8.

    For the local RViz preview: it needs no TF and shows the trace and the mask
    that produced it in one image.  Published by the node that CONSUMES the
    mask, so it never crosses the network.
    """
    out = (np.asarray(mask, dtype=np.uint8) // 3).copy()
    if out.ndim != 2 or not points:
        return out
    h, w = out.shape
    for y, xn in points:
        yy = int(np.clip(round(float(y)), 0, h - 1))
        xx = int(np.clip(round((float(xn) + 0.5) * w), 0, w - 1))
        out[max(0, yy - 1):min(h, yy + 2), max(0, xx - 1):min(w, xx + 2)] = 255
    return out


# --- Vision path -> metric ground path (FollowPath action, MPPI mode) ---

GOAL_DEFAULTS = {
    "image_width": 640,
    "image_height": 480,
    # ChArUco calibration 2026-11-15 at 1280x720 (vision_nav_v1.md §5A.7).
    "fx_1280": 844.40,
    "fy_1280": 851.46,
    "cx_1280": 614.20,
    "cy_1280": 401.18,
    # MEASURED 2026-11-19: the 640x480 mode is NOT a uniform scale of 1280x720.
    # Feature matching on the same scene gives NCC 0.988 for "scale by 480/720
    # then centre-crop the width" versus 0.326 for "stretch to 640x480" -- i.e.
    # the two modes share the VERTICAL field of view and 640x480 sees a narrower
    # horizontal one (central 960 of the 1280 rows). Scaling fx by 640/1280
    # instead (the obvious-looking thing) is 33% low and silently shrinks every
    # projected distance. `intrinsics()` below derives the operating-resolution
    # camera matrix from this crop model instead.
    "crop_to_width": 640,          # horizontal crop applied at the operating size
    # Fitted, not taped: from the competition mat's known 45 cm lane (map is
    # 4.8 x 3.6 m). Repro: scripts/fit_camera_mount.py on RAW 640x480 masks
    # (rectify:=false -- see the launch file for why rectification is not
    # paired with these numbers). 4 frames agreed to h +-0.005 m /
    # pitch +-0.3 deg at 15-22 px residual; the old 0.30 m / 0.0 rad
    # placeholders scored ~380 px, so they put every far point at the wrong
    # distance. Visible ground range is 0.32..2.1 m.
    "cam_height_m": 0.343,
    "cam_pitch_rad": 0.4775,
    "max_range_m": 2.5,
    "horizon_min_dep_rad": 0.05,   # rows near/above the horizon project to inf
    "path_len_m": 1.8,             # total FollowPath arc length
    "extend_step_m": 0.15,
    # Same single source as FOLLOW_DEFAULTS.lane_center_bias (LANE_CENTER_BIAS).
    "lane_center_bias": LANE_CENTER_BIAS,
}


def intrinsics(params):
    """Operating-resolution (fx, fy, cx, cy) from the 1280x720 calibration.

    See the field-of-view note in GOAL_DEFAULTS: 640x480 shares the vertical FOV
    with 1280x720 and crops horizontally, so fx and cx rescale by the same
    480/720 factor as fy and cy, after shifting the principal point for the
    crop. Getting this wrong is a 33% scale error at 640 wide.
    """
    w = float(params["image_width"])
    h_img = float(params["image_height"])
    calib_w, calib_h = 1280.0, 720.0
    s_y = h_img / calib_h
    # horizontal extent of the calibration image that survives the crop
    crop_w = float(params.get("crop_to_width", w)) / s_y
    x0 = (calib_w - crop_w) / 2.0
    fx = params["fx_1280"] * s_y
    fy = params["fy_1280"] * s_y
    cx = (params["cx_1280"] - x0) * s_y
    cy = params["cy_1280"] * s_y
    return fx, fy, cx, cy


def ground_points(points, params):
    """Trace polyline (pixels) -> base-frame ground path [(forward, y_left)].

    Flat-ground pinhole projection: depression angle from the row, forward
    distance d = h / tan(dep), lateral from the calibrated fx. y_left follows
    ROS convention (left positive), so a road right of the image center is
    negative. Rows near/above the horizon are skipped (d -> infinity).

    Rows BEYOND `max_range_m` are dropped, not clamped. Clamping piled every
    far point onto the same forward distance while keeping its lateral offset,
    which turned a corner arm into a sideways jump at constant distance --
    measured 2026-11-19: a 2.97 m path whose forward progress was 1.70 m and
    lateral 1.22 m. Dropping leaves a monotone path and lets `extend_to_length`
    continue along the last real direction.
    """
    w = float(params["image_width"])
    h_img = float(params["image_height"])
    fx, fy, cx, cy = intrinsics(params)
    pitch = params["cam_pitch_rad"]
    max_range = float(params["max_range_m"])
    out = []
    for y, xn in points:
        dep = float(np.arctan((y - cy) / fy)) + pitch
        if dep < params["horizon_min_dep_rad"]:
            continue
        d = params["cam_height_m"] / np.tan(dep)
        if not np.isfinite(d) or d > max_range:
            continue                      # beyond range: position is unknowable
        x_px = (xn + 0.5) * w          # x_norm is centered; convert to pixels
        # Same camera-mount yaw correction the direct follower applies, so the
        # MPPI path is centred on the robot rather than on the camera axis.
        x_px -= float(params.get("lane_center_bias", 0.0)) * w
        y_left = -(x_px - cx) * d / fx
        out.append((float(d), float(y_left)))
    return out


def extend_to_length(pts, params):
    """Extend a base-frame path with dead reckoning to path_len_m arc length.

    The heading is taken from the last pair of points that actually move, not
    simply the final two: a saturated or duplicated tail makes that difference
    zero, and normalising a zero vector used to produce a purely lateral
    extension (measured 2026-11-19: the path grew 1.8 m sideways at constant
    forward distance while the robot was supposed to be following a lane).
    """
    if not pts:
        return pts
    out = list(pts)
    step = params["extend_step_m"]
    dx, dy = 1.0, 0.0
    for i in range(len(out) - 1, 0, -1):
        ex = out[i][0] - out[i - 1][0]
        ey = out[i][1] - out[i - 1][1]
        norm = float(np.hypot(ex, ey))
        if norm > 1e-6:
            dx, dy = ex / norm, ey / norm
            break
    total = float(np.sum(np.hypot(np.diff([p[0] for p in out]),
                                  np.diff([p[1] for p in out]))))
    for _ in range(64):
        if total >= params["path_len_m"]:
            break
        out.append((out[-1][0] + dx * step, out[-1][1] + dy * step))
        total += step
    return out
