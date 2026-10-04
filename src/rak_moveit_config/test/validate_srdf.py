#!/usr/bin/env python3
"""validate_srdf.py - assert SRDF structure via defusedxml.

Spec section 7.4 review focus item #2: group name and chain must match
kinematics.yaml. Asserts:
  - <group name="arm_with_base"> exists with chain base_link->gripper_link
  - <end_effector name="gripper_eef"> exists
  - <virtual_joint name="world_to_base"> exists
  - 6 <disable_collisions> entries (adjacent pairs only)

srdf_parser_py is not available on PyPI; we use defusedxml for safe
structural validation only (semantic validation is MoveIt's job at
move_group startup).
"""
import sys
from pathlib import Path

import defusedxml.ElementTree as ET


def canonicalize_pair(a: str, b: str) -> tuple[str, str]:
    """Sort pair to alphabetical order for unordered comparison."""
    return (a, b) if a < b else (b, a)


def main() -> int:
    pkg_root = Path(__file__).resolve().parents[1]
    srdf_path = pkg_root / "config" / "rak.srdf"
    assert srdf_path.is_file(), f"missing SRDF: {srdf_path}"

    tree = ET.parse(srdf_path)
    root = tree.getroot()
    assert root.tag == "robot", f"root tag is {root.tag}, not robot"

    # 1. planning group
    groups = root.findall("group")
    g = next((x for x in groups if x.get("name") == "arm_with_base"), None)
    assert g is not None, "group arm_with_base missing"
    chain = g.find("chain")
    assert chain is not None, "group arm_with_base missing <chain>"
    assert chain.get("base_link") == "base_link", f"chain base_link={chain.get('base_link')}"
    assert chain.get("tip_link") == "gripper_link", f"chain tip_link={chain.get('tip_link')}"
    print("OK: group arm_with_base has chain base_link -> gripper_link")

    # 2. end effector
    ees = root.findall("end_effector")
    ee = next((x for x in ees if x.get("name") == "gripper_eef"), None)
    assert ee is not None, "end_effector gripper_eef missing"
    assert ee.get("parent_link") == "gripper_link"
    assert ee.get("group") == "arm_with_base"
    print("OK: end_effector gripper_eef attached to arm_with_base group")

    # 3. virtual joint
    vjs = root.findall("virtual_joint")
    vj = next((x for x in vjs if x.get("name") == "world_to_base"), None)
    assert vj is not None, "virtual_joint world_to_base missing"
    assert vj.get("type") == "planar"
    assert vj.get("parent_frame") == "world"
    assert vj.get("child_link") == "base_link"
    print("OK: virtual_joint world_to_base is planar world -> base_link")

    # 4. disable_collisions: 6 entries (adjacent pairs only, unordered)
    dcs = root.findall("disable_collisions")
    expected_pairs = {
        canonicalize_pair("base_link", "arm_root_link"),
        canonicalize_pair("arm_root_link", "shoulder_link"),
        canonicalize_pair("shoulder_link", "upper_arm_link"),
        canonicalize_pair("upper_arm_link", "forearm_link"),
        canonicalize_pair("forearm_link", "wrist_link"),
        canonicalize_pair("wrist_link", "gripper_link"),
    }
    actual_pairs = {canonicalize_pair(dc.get("link1"), dc.get("link2")) for dc in dcs}
    missing = expected_pairs - actual_pairs
    extra = actual_pairs - expected_pairs
    assert not missing, f"missing disable_collisions pairs: {missing}"
    assert not extra, f"unexpected disable_collisions pairs: {extra}"
    print(f"OK: {len(dcs)} disable_collisions entries, all adjacent pairs present")

    print("validate_srdf: ALL OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
