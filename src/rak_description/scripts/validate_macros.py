#!/usr/bin/env python3
"""validate_macros.py - assertion tests for xacro macros.

Spec section 7.3 review focus item #1: properties.xacro parameter drift.
Asserts that varying link_length regenerates inertias proportionally.
Asserts link_length consistency (review focus item #5).
"""
import sys
from pathlib import Path

import xacro


def expand_urdf(urdf_path: Path, link_length_override: float | None = None) -> str:
    """Expand the URDF, optionally overriding link_length for the test."""
    mappings = {"link_length": str(link_length_override)} if link_length_override is not None else {}
    doc = xacro.process_file(str(urdf_path), mappings=mappings)
    return doc.toxml()


def test_inertia_scales_with_link_length(urdf_path: Path) -> None:
    """When link_length grows, ixx of the affected link must grow proportionally."""
    from urdf_parser_py.urdf import URDF

    base = URDF.from_xml_string(expand_urdf(urdf_path, link_length_override=0.20))
    bigger = URDF.from_xml_string(expand_urdf(urdf_path, link_length_override=0.40))

    def get_ixx(robot, link_name: str) -> float:
        for link in robot.links:
            if link.name == link_name:
                return link.inertial.inertia.ixx
        raise KeyError(link_name)

    base_ixx = get_ixx(base)
    bigger_ixx = get_ixx(bigger)
    # ixx scales with Lz^2 for a box (Ly = Lz = link_length); expect ~4x
    ratio = bigger_ixx / base_ixx
    assert 3.5 < ratio < 4.5, f"ixx ratio {ratio:.2f} not ~4x when link_length doubles"


def test_link_length_consistency(urdf_path: Path) -> None:
    """Spec D7 / review focus item #5: 3 arm links must each be 0.20 m."""
    from urdf_parser_py.urdf import URDF

    robot = URDF.from_xml_string(expand_urdf(urdf_path))
    for link_name in ("upper_arm_link", "forearm_link", "wrist_link"):
        link = next((l for l in robot.links if l.name == link_name), None)
        assert link is not None, f"missing link {link_name}"
        size_str = link.collision.geometry.box.size
        lx, ly, lz = (float(v) for v in size_str.split())
        assert abs(lx - 0.20) < 1e-6, f"{link_name} length {lx} != 0.20"


def main() -> int:
    pkg_root = Path(__file__).resolve().parents[1]
    urdf_path = pkg_root / "urdf" / "rak.urdf.xacro"
    if not urdf_path.exists():
        print(f"SKIP: {urdf_path} not yet created (Task 4); rerun after Task 4.")
        return 0

    test_link_length_consistency(urdf_path)
    test_inertia_scales_with_link_length(urdf_path)
    print("validate_macros: ALL OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())