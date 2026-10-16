#!/usr/bin/env python3
"""Validate the isolated six-axis URDF/SRDF and simulated controller contract."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml


ARM_JOINTS = {
    "j1_base",
    "j2_shoulder",
    "j3_elbow",
    "j4_forearm",
    "j5_wrist_pitch",
    "j6_wrist_roll",
}


def main() -> int:
    source_root = Path(__file__).resolve().parents[2]
    description_root = source_root / "rak_description"
    config_root = Path(__file__).resolve().parents[1] / "config"
    urdf_path = description_root / "urdf" / "six_axis_sim.urdf.xacro"

    expanded = subprocess.run(
        ["xacro", str(urdf_path)], capture_output=True, text=True, check=True
    ).stdout
    robot = ET.fromstring(expanded)
    assert robot.get("name") == "rak_six_axis_sim"
    joints = {joint.get("name"): joint for joint in robot.findall("joint")}
    revolute = {name for name, joint in joints.items() if joint.get("type") == "revolute"}
    assert revolute == ARM_JOINTS, f"positioning joints mismatch: {revolute ^ ARM_JOINTS}"
    assert joints["gripper_joint"].get("type") == "prismatic"
    assert "ros2_control" not in expanded

    source_text = urdf_path.read_text(encoding="utf-8")
    assert "Simulation fixture only" in source_text
    srdf = ET.parse(config_root / "six_axis_sim.srdf").getroot()
    arm_group = next(group for group in srdf.findall("group") if group.get("name") == "arm")
    assert arm_group.find("chain").get("tip_link") == "tool0"
    gripper_group = next(
        group for group in srdf.findall("group") if group.get("name") == "gripper"
    )
    assert gripper_group.find("joint").get("name") == "gripper_joint"

    controllers = yaml.safe_load(
        (config_root / "six_axis_sim_controllers.yaml").read_text(encoding="utf-8")
    )
    assert set(controllers["arm_controller"]["joints"]) == ARM_JOINTS
    assert controllers["gripper_controller"]["joints"] == ["gripper_joint"]
    print("validate_six_axis_sim: six positioning joints and independent gripper OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
