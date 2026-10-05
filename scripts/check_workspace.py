#!/usr/bin/env python3
"""Check package boundaries; --installed also resolves launch files without starting nodes."""

from __future__ import annotations

import argparse
import ast
import importlib
import os
from pathlib import Path
import re
import runpy
import subprocess
import tempfile
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
EXPECTED = {
    "rak_agent", "rak_arm_simulation", "rak_astra_driver", "rak_base_driver",
    "rak_bringup", "rak_can_driver", "rak_description", "rak_gpu", "rak_interfaces",
    "rak_motion_stub", "rak_moveit_config", "rak_navigation", "rak_sensor_bringup",
    "rak_stereo_driver", "rak_task_executor",
}
OLD_NAMES = {"core", "eye", "boot", "brain", "motor_stub", "astra_bridge"}


def packages() -> dict[str, tuple[Path, set[str]]]:
    found = {}
    manifests = sorted((ROOT / "src").glob("rak_*/package.xml"))
    manifests.append(ROOT / "brain/package.xml")
    for manifest in manifests:
        package = ET.parse(manifest).getroot()
        name = package.findtext("name")
        assert name not in found, f"duplicate package: {name}"
        assert re.fullmatch(r"rak_[a-z][a-z0-9_]*", name), name
        assert manifest.parent.name == name or (name == "rak_agent" and manifest.parent.name == "brain")
        dependencies = {
            element.text for element in package
            if element.tag == "depend" or element.tag.endswith("_depend")
        }
        assert not dependencies & OLD_NAMES, (name, dependencies & OLD_NAMES)
        assert package.findtext("export/build_type") in {"ament_cmake", "ament_python"}
        if package.findtext("export/build_type") == "ament_python":
            assert (manifest.parent / "resource" / name).is_file(), name
            assert f"$base/lib/{name}" in (manifest.parent / "setup.cfg").read_text(), name
        else:
            cmake = (manifest.parent / "CMakeLists.txt").read_text()
            assert re.search(rf"project\({name}(?:\s|\))", cmake), name
        found[name] = manifest.parent, dependencies
    assert set(found) == EXPECTED, set(found) ^ EXPECTED
    for old in OLD_NAMES:
        assert not (ROOT / "src" / old).exists(), f"legacy source directory: {old}"
    return found


def check_boundaries(found) -> None:
    def visit(name, stack):
        assert name not in stack, f"dependency cycle: {stack + [name]}"
        for dep in found[name][1] & found.keys():
            visit(dep, stack + [name])

    for name in found:
        visit(name, [])
    assert not found["rak_interfaces"][1] & EXPECTED
    assert not found["rak_gpu"][1] & EXPECTED
    assert "rak_interfaces" in found["rak_agent"][1]
    assert "rak_gpu" not in found["rak_agent"][1]
    assert not found["rak_description"][1] & EXPECTED
    assert "rak_description" in found["rak_moveit_config"][1]
    assert "rak_interfaces" in found["rak_base_driver"][1]
    assert "rak_agent" in found["rak_task_executor"][1]
    assert "rak_interfaces" in found["rak_task_executor"][1]
    assert "rak_gpu" in found["rak_stereo_driver"][1]
    assert "rak_interfaces" not in found["rak_stereo_driver"][1]


def check_launch_references(found) -> None:
    for name, (path, dependencies) in found.items():
        for launch in path.glob("launch/*.launch.py"):
            tree = ast.parse(launch.read_text(), filename=str(launch))
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call):
                    continue
                references = [
                    kw.value for kw in node.keywords if kw.arg in {"package", "package_name"}
                ]
                if isinstance(node.func, ast.Name) and node.func.id == "get_package_share_directory":
                    references += node.args[:1]
                for ref in references:
                    if isinstance(ref, ast.Constant) and isinstance(ref.value, str):
                        assert ref.value not in OLD_NAMES, (launch, ref.value)
                        assert ref.value == name or ref.value in dependencies, (
                            launch, f"undeclared runtime dependency: {ref.value}",
                        )
    urdf = (found["rak_description"][0] / "urdf/rak.urdf.xacro").read_text()
    assert "$(find boot)" not in urdf


def check_installed(found) -> None:
    from ament_index_python.packages import get_package_share_directory

    for name, (source, _) in found.items():
        share = Path(get_package_share_directory(name))
        assert ET.parse(share / "package.xml").getroot().findtext("name") == name
        for directory in ("launch", "config", "urdf", "meshes"):
            for resource in (source / directory).rglob("*"):
                if resource.is_file() and "__pycache__" not in resource.parts:
                    installed = share / resource.relative_to(source)
                    assert installed.is_file(), installed
                    assert installed.read_bytes() == resource.read_bytes(), f"stale install: {installed}"
        for launch in sorted((share / "launch").glob("*.launch.py")):
            result = subprocess.run(
                ["ros2", "launch", name, launch.name, "--show-args"],
                text=True, capture_output=True, timeout=60,
            )
            assert result.returncode == 0, f"{name}/{launch.name}\n{result.stdout}\n{result.stderr}"
            print(f"OK launch: {name}/{launch.name}")

    msgs = importlib.import_module("rak_interfaces.msg")
    from rclpy.serialization import deserialize_message, serialize_message

    for message_type in (
        msgs.Action,
        msgs.BaseTelemetry,
        msgs.Decision,
        msgs.TaskStatus,
        msgs.TrackedObject,
        msgs.TrackedObjects,
    ):
        message = message_type()
        assert deserialize_message(serialize_message(message), message_type) == message
    from rak_interfaces.action import ExecuteTask

    goal = ExecuteTask.Goal()
    assert deserialize_message(serialize_message(goal), ExecuteTask.Goal) == goal
    importlib.import_module("rak_motion_stub.cmd_vel_stub")
    from ament_index_python.packages import get_package_prefix

    modules = (
        ("rak_ai.task_intent", "rak_agent"),
        ("rak_arm_simulation.controllers", "rak_arm_simulation"),
        ("rak_base_driver.node", "rak_base_driver"),
        ("rak_task_executor.node", "rak_task_executor"),
    )
    for module_name, package_name in modules:
        module = importlib.import_module(module_name)
        prefix = Path(get_package_prefix(package_name)).resolve()
        module_path = Path(module.__file__).resolve()
        assert prefix in module_path.parents, f"{module_name} imported outside install: {module_path}"

    bootstrap_path = Path(get_package_prefix("rak_agent")) / "lib/rak_agent/agent_node"
    bootstrap = runpy.run_path(str(bootstrap_path))
    with patch.object(Path, "cwd", return_value=Path("/tmp")):
        candidates = bootstrap["_candidates"]()
    interpreter = next((path for path in candidates if bootstrap["_usable"](path)), None)
    assert interpreter is not None, "agent bootstrap found no usable interpreter"
    result = subprocess.run(
        [str(interpreter), str(ROOT / "brain/tests/test_ros_smoke.py")],
        cwd="/tmp", text=True, capture_output=True, timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    print("OK installed interfaces, Python relay and agent bootstrap (outside workspace)")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--installed", action="store_true")
    args = parser.parse_args()
    found = packages()
    check_boundaries(found)
    check_launch_references(found)
    if args.installed:
        with tempfile.TemporaryDirectory(prefix="rak-workspace-check-") as logs:
            with patch.dict(os.environ, {"ROS_LOG_DIR": logs, "PYTHONDONTWRITEBYTECODE": "1"}):
                check_installed(found)
    print(f"OK: {len(found)} owned packages, dependency graph and launch references")


if __name__ == "__main__":
    main()
