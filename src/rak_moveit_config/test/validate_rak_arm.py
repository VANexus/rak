#!/usr/bin/env python3
"""Validate the real six-axis arm URDF/SRDF pair (rak_arm.*).

[AI] Checks (2026-10-08 measured calibration):
- xacro expands and parses; joint set, axes, limits match the TUI-measured
  calibration in properties_arm.xacro
- SRDF declares the arm group chain and a home group_state with all zeros
- joint names agree between URDF and SRDF
Run: python3 test/validate_rak_arm.py (from the package directory)
"""
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "..", "..", "rak_description", "urdf", "rak_arm.urdf.xacro")
SRDF = os.path.join(HERE, "..", "config", "rak_arm.srdf")

EXPECTED_JOINTS = {
    # name: (axis, lower_deg, upper_deg)  limits relative to zero pose
    # (zero = commanded J1=0 J2=135 J3=180 J4=0 J5=0 J6=0, user 2026-10-08)
    "base_yaw_joint":       ("0 0 1",    0.0, 270.0),
    "shoulder_pitch_joint": ("0 1 0", -135.0, 120.0),
    "elbow_pitch_joint":    ("0 -1 0", -180.0,  90.0),
    "forearm_roll_joint":   ("0 0 1",    0.0, 270.0),
    "wrist_pitch_joint":    ("0 1 0",    0.0, 270.0),
    "wrist_roll_joint":     ("0 0 1",    0.0, 270.0),
}


def expand_urdf() -> str:
    try:
        import xacro  # noqa: PLC0415
        doc = xacro.process_file(URDF)
        return doc.toprettyxml(indent="  ")
    except ImportError:
        out = subprocess.run(
            ["ros2", "run", "xacro", "xacro", URDF],
            check=True, capture_output=True, text=True)
        return out.stdout


def fail(msg: str) -> None:
    print(f"FAIL: {msg}")
    sys.exit(1)


def main() -> None:
    for path in (URDF, SRDF):
        if not os.path.isfile(path):
            fail(f"missing file: {path}")

    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as fh:
        fh.write(expand_urdf())
        urdf_path = fh.name

    root = ET.parse(urdf_path).getroot()
    if root.get("name") != "rak":
        fail(f"URDF robot name is {root.get('name')!r}, expected 'rak'")

    joints = {}
    for joint in root.findall("joint"):
        axis = joint.find("axis")
        limit = joint.find("limit")
        if axis is None or limit is None:
            continue
        joints[joint.get("name")] = (
            axis.get("xyz").strip(),
            float(limit.get("lower")),
            float(limit.get("upper")),
        )

    for name, (axis, lo_deg, hi_deg) in EXPECTED_JOINTS.items():
        if name not in joints:
            fail(f"URDF missing joint {name}")
        got_axis, got_lo, got_hi = joints[name]
        tol = 1e-3
        if got_axis != axis:
            fail(f"{name} axis {got_axis!r}, expected {axis!r}")
        if abs(got_lo - lo_deg * 3.141592653589793 / 180.0) > tol:
            fail(f"{name} lower {got_lo:.4f} rad, expected {lo_deg} deg")
        if abs(got_hi - hi_deg * 3.141592653589793 / 180.0) > tol:
            fail(f"{name} upper {got_hi:.4f} rad, expected {hi_deg} deg")
    print(f"URDF OK: {len(joints)} actuated joints, axes/limits match calibration")

    # chain link count: base..tool0
    links = {link.get("name") for link in root.findall("link")}
    expected_links = {"arm_base_link", "shoulder_link", "upper_arm_link",
                      "forearm_link", "forearm_roll_link", "wrist_pitch_link",
                      "tool0"}
    missing = expected_links - links
    if missing:
        fail(f"URDF missing links: {sorted(missing)}")
    print("URDF OK: link chain complete")

    srdf = ET.parse(SRDF).getroot()
    if srdf.get("name") != "rak":
        fail(f"SRDF robot name is {srdf.get('name')!r}, expected 'rak'")
    groups = {g.get("name"): g for g in srdf.findall("group")}
    if "arm" not in groups:
        fail("SRDF missing group 'arm'")
    chain = groups["arm"].find("chain")
    if chain is None or chain.get("base_link") != "arm_base_link" \
            or chain.get("tip_link") != "tool0":
        fail("SRDF arm group chain must be arm_base_link -> tool0")
    homes = [gs for gs in srdf.findall("group_state") if gs.get("name") == "home"]
    if not homes:
        fail("SRDF missing home group_state")
    # home = safe pose (user 2026-10-08): commands 0/225/270/0/0/0
    # → q = cmd − zero_cmd(0/135/180/0/0/0) = J2 +90°, J3 +90°, rest 0
    expected_home = {
        "base_yaw_joint": 0.0,
        "shoulder_pitch_joint": 90.0,
        "elbow_pitch_joint": 90.0,
        "forearm_roll_joint": 0.0,
        "wrist_pitch_joint": 0.0,
        "wrist_roll_joint": 0.0,
    }
    home_joints = {j.get("name"): float(j.get("value")) for j in homes[0].findall("joint")}
    if set(home_joints) != set(EXPECTED_JOINTS):
        fail(f"SRDF home joints {sorted(home_joints)} != URDF joints")
    for name, deg in expected_home.items():
        got = home_joints[name]
        if abs(got - deg * 3.141592653589793 / 180.0) > 1e-3:
            fail(f"SRDF home {name} = {got:.4f} rad, expected {deg} deg")
    print("SRDF OK: arm group, home (safe pose), joint names agree")

    print("ALL OK")


if __name__ == "__main__":
    main()
