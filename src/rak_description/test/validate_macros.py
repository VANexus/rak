#!/usr/bin/env python3
"""validate_macros.py - assertion tests for xacro macros.

Spec section 7.3 review focus item #5: properties.xacro parameter consistency.
Asserts default link_length = 0.20 m for the 3 arm links.

Review focus item #1 (parameter drift) is NOT tested programmatically here:
xacro does not support overriding a <xacro:property> declared in the SAME file
(only when passed into an included file via <xacro:arg>). So `link_length`
override via CLI requires editing the URDF source. The single source of truth
IS preserved - it lives at the top of rak.urdf.xacro. (Ruling R4.1.)
"""
import subprocess
import sys
from pathlib import Path


def expand_urdf(urdf_path: Path) -> str:
    """Expand the URDF via xacro CLI (no overrides)."""
    cmd = ["ros2", "run", "xacro", "xacro", str(urdf_path)]
    result = subprocess.run(cmd, capture_output=True, text=True, check=True)
    return result.stdout


def test_link_length_consistency(urdf_path: Path) -> None:
    """Spec D7 / review focus item #5: 3 arm links must each be 0.20 m by default."""
    from urdf_parser_py.urdf import URDF

    robot = URDF.from_xml_string(expand_urdf(urdf_path))
    for link_name in ("upper_arm_link", "forearm_link", "wrist_link"):
        link = next((l for l in robot.links if l.name == link_name), None)
        assert link is not None, f"missing link {link_name}"
        size = link.collision.geometry.size
        assert len(size) == 3, f"{link_name} geometry.size not 3-element list: {size}"
        lx = float(size[0])
        assert abs(lx - 0.20) < 1e-6, f"{link_name} length {lx} != 0.20"


def test_seven_links_present(urdf_path: Path) -> None:
    """Spec section 7.2: URDF must have exactly 7 links."""
    from urdf_parser_py.urdf import URDF

    robot = URDF.from_xml_string(expand_urdf(urdf_path))
    expected = {"base_link", "arm_root_link", "shoulder_link", "upper_arm_link",
                "forearm_link", "wrist_link", "gripper_link"}
    actual = {link.name for link in robot.links}
    assert actual == expected, f"link set mismatch: missing={expected-actual}, extra={actual-expected}"


def test_six_joints_present(urdf_path: Path) -> None:
    """Spec section 7.2: URDF must have exactly 6 joints."""
    from urdf_parser_py.urdf import URDF

    robot = URDF.from_xml_string(expand_urdf(urdf_path))
    expected = {"base_pan_joint", "shoulder_pitch_joint", "elbow_pitch_joint",
                "wrist_pitch_joint", "wrist_roll_joint", "gripper_mount_joint"}
    actual = {j.name for j in robot.joints}
    assert actual == expected, f"joint set mismatch: missing={expected-actual}, extra={actual-expected}"


def main() -> int:
    pkg_root = Path(__file__).resolve().parents[1]
    urdf_path = pkg_root / "urdf" / "rak.urdf.xacro"
    assert urdf_path.is_file(), f"missing URDF: {urdf_path}"

    test_seven_links_present(urdf_path)
    test_six_joints_present(urdf_path)
    test_link_length_consistency(urdf_path)
    print("validate_macros: ALL OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
