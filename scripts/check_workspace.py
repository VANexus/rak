#!/usr/bin/env python3
"""Check package boundaries; --installed also resolves launch files without starting nodes.

Structure per ADR-0013: all owned packages live in src/ (no brain/ special case,
CAN v3 stack retired; the local attic/ archive was deleted 2026-10-10), rak_agent is the
decision layer and must have zero dependents, and the task-intent contract lives in
rak_intent.
"""

from __future__ import annotations

import argparse
import ast
import importlib
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
EXPECTED = {
    "rak_agent", "rak_arm_pinocchio_ik", "rak_arm_simulation", "rak_astra_driver",
    "rak_bringup", "rak_chassis",
    "rak_commissioning", "rak_description", "rak_gpu", "rak_grasp_perception",
    "rak_imu_driver",
    "rak_harness",
    "rak_intent",
    "rak_canopen", "rak_interfaces", "rak_lifecycle", "rak_motion_stub", "rak_moveit_config",
    "rak_metrics", "rak_orchestration",
    "rak_pointcloud", "rak_speech", "rak_stereo_driver",
    "rak_task_executor", "rak_tool_executor", "rak_wbc",
}
OLD_NAMES = {"core", "eye", "boot", "brain", "motor_stub", "astra_bridge"}


def packages() -> dict[str, tuple[Path, set[str]]]:
    found = {}
    for manifest in sorted((ROOT / "src").glob("rak_*/package.xml")):
        package = ET.parse(manifest).getroot()
        name = package.findtext("name")
        assert name not in found, f"duplicate package: {name}"
        assert re.fullmatch(r"rak_[a-z][a-z0-9_]*", name), name
        assert manifest.parent.name == name, (name, manifest.parent.name)
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
    assert not (ROOT / "brain").exists(), "brain/ moved to src/rak_agent (ADR-0013)"
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
    # Foundation packages stay pure.
    assert not found["rak_interfaces"][1] & EXPECTED
    assert not found["rak_gpu"][1] & EXPECTED
    assert not found["rak_intent"][1] & EXPECTED
    assert not found["rak_description"][1] & EXPECTED
    # ADR-0020: 生命周期机制层是基础设施，谁都能用，但它不反过来依赖任何自有包；
    # 用了 ManagedNode 的包必须显式声明依赖（漏声明会在单独 clone 构建时才炸）。
    assert not found["rak_lifecycle"][1] & EXPECTED, found["rak_lifecycle"][1] & EXPECTED
    # ADR-0022: v5 契约层同样是基础设施，不依赖任何自有包 —— 它只读 SSOT 副本。
    assert not found["rak_canopen"][1] & EXPECTED, found["rak_canopen"][1] & EXPECTED
    # 2026-10-29 第二轮收敛后，这些包的自有节点全部是 ManagedNode（ADR-0020 §4）。
    # 漏声明在单仓库构建时看不出来，单独 clone 一个包才炸 —— 所以在这里钉死。
    for name in (
        "rak_task_executor", "rak_wbc", "rak_imu_driver", "rak_speech",
        "rak_harness", "rak_tool_executor", "rak_agent",
    ):
        assert "rak_lifecycle" in found[name][1], f"{name} 用 ManagedNode 却没声明依赖"
    assert "rak_lifecycle" in found["rak_bringup"][1]
    # The decision layer has interfaces as its only internal dependency ...
    assert "rak_interfaces" in found["rak_agent"][1]
    assert "rak_gpu" not in found["rak_agent"][1]
    # ... and (ADR-0013) nothing may import it back. The only legitimate
    # dependent is the composition layer (rak_bringup launches agent_node);
    # any other dependent means someone is crossing the /brain/decision seam.
    dependents = {
        name for name, (_, deps) in found.items() if "rak_agent" in deps
    }
    assert dependents <= {"rak_bringup"}, (
        f"packages depend on the decision layer beyond bringup: {sorted(dependents - {'rak_bringup'})}"
    )
    # Shared intent contract flows downward to its two consumers.
    assert "rak_intent" in found["rak_task_executor"][1]
    assert "rak_intent" in found["rak_speech"][1]
    assert "rak_interfaces" in found["rak_task_executor"][1]
    # Modeling assets and drivers.
    assert "rak_description" in found["rak_moveit_config"][1]
    assert "rak_gpu" in found["rak_stereo_driver"][1]
    assert "rak_interfaces" not in found["rak_stereo_driver"][1]
    # Harness seam (ADR-0010): advisory proposals only, never motion commands.
    assert "rak_interfaces" in found["rak_harness"][1]
    assert "rak_intent" in found["rak_harness"][1]
    assert "rak_agent" not in found["rak_harness"][1]
    assert "rak_task_executor" not in found["rak_harness"][1]
    assert "rak_harness" not in found["rak_task_executor"][1]
    assert "rak_harness" in found["rak_bringup"][1]
    # Commissioning tools speak standard ROS interfaces only (no transport deps).
    assert "rak_chassis" not in found["rak_commissioning"][1]
    # Bringup composes the chassis control chain.
    assert "rak_chassis" in found["rak_bringup"][1]


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


def check_assembly_layers(found, hardware: set[str]) -> None:
    """ADR-0026：契约层与硬件组合层分开之后，两边都不许越界。

    - `rak_orchestration` 是清单/契约/编排入口，**不得**依赖任何硬件或 GPU 包 ——
      否则"桌面机一条命令建起来"立刻退回今天的补丁状态；
    - `rak_bringup` 是唯一硬件组合层，必须依赖契约层（它 include 编排入口）；
    - `rak_bringup` 里不得再出现清单与编排测试（那是契约层的职责）。
    """
    leaked = found["rak_orchestration"][1] & hardware
    assert not leaked, f"rak_orchestration 依赖了硬件包: {sorted(leaked)}"
    assert "rak_orchestration" in found["rak_bringup"][1], (
        "rak_bringup 必须依赖 rak_orchestration（robot.launch.py include 编排入口）"
    )
    bringup = found["rak_bringup"][0]
    assert not (bringup / "config/system_manifest.yaml").exists(), (
        "system_manifest.yaml 属于 rak_orchestration（ADR-0026）"
    )
    assert not (bringup / "launch/supervisor.launch.py").exists(), (
        "supervisor.launch.py 属于 rak_orchestration（ADR-0026）"
    )


def check_installed(found, profile=None) -> None:
    from ament_index_python.packages import get_package_share_directory, get_package_prefix

    # 桌面档的缺口是**声明**出来的（build_profiles.yaml），不是从环境里嗅出来的。
    # 旧行为（--skip-absent）在失败后按报错字符串猜包名；现在先查表，再执行，
    # 语义从"猜它为什么失败"变成"它本来就不该在这台机器上跑"。
    absent: set[str] = set()
    tolerable: set[str] = set()
    if profile is not None:
        for name in profile.skip_packages:
            absent.add(name)
        for name in profile.tolerate_absent:
            try:
                get_package_prefix(name)
            except Exception:  # noqa: BLE001 - 不存在就是不存在
                tolerable.add(name)
        absent |= tolerable
        print(f"[{profile.name}] 声明跳过 {len(profile.skip_packages)} 个包；"
              f"本机缺失的可选件 {len(tolerable)} 个: {', '.join(sorted(tolerable)) or '-'}")

    skipped_launches: list[str] = []
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
            key = f"{name}/{launch.name}"
            contract = profile.launch_contracts.get(key) if profile is not None else None
            if contract is not None and not contract.desktop and profile.name == "desktop":
                # 档位已声明本档不可用 —— 不去执行它，也不让它的失败污染结论。
                skipped_launches.append(f"{key}（{contract.reason or '硬件前置'}）")
                print(f"SKIP launch (档位声明): {key} -> {contract.reason or '硬件前置'}")
                continue
            result = subprocess.run(
                ["ros2", "launch", name, launch.name, "--show-args"],
                text=True, capture_output=True, timeout=60,
            )
            if result.returncode != 0 and absent:
                missing = _only_missing_package(result.stderr, absent)
                if missing:
                    # 语法错误仍然红：只有"仅仅缺包"才允许跳过。
                    skipped_launches.append(f"{key}（缺 {missing}）")
                    print(f"SKIP launch (缺包): {key} -> {missing}")
                    continue
            assert result.returncode == 0, f"{name}/{launch.name}\n{result.stdout}\n{result.stderr}"
            print(f"OK launch: {name}/{launch.name}")
    if skipped_launches:
        print(f"SKIP 汇总: {len(skipped_launches)} 个 launch 未在本档验证")

    msgs = importlib.import_module("rak_interfaces.msg")
    from rclpy.serialization import deserialize_message, serialize_message

    for message_type in (
        msgs.Action,
        msgs.BaseTelemetry,
        msgs.Decision,
        msgs.TaskStatus,
        msgs.TrackedObject,
        msgs.TrackedObjects,
        msgs.HarnessRequest,
        msgs.HarnessProposal,
        msgs.HarnessEvent,
    ):
        message = message_type()
        assert deserialize_message(serialize_message(message), message_type) == message
    from rak_interfaces.action import ExecuteTask

    goal = ExecuteTask.Goal()
    assert deserialize_message(serialize_message(goal), ExecuteTask.Goal) == goal
    importlib.import_module("rak_motion_stub.cmd_vel_stub")
    from ament_index_python.packages import get_package_prefix

    modules = (
        ("rak_intent.task_intent", "rak_intent"),
        ("rak_agent.node", "rak_agent"),
        ("rak_harness.contract", "rak_harness"),
        ("rak_harness.dialogue", "rak_harness"),
        ("rak_arm_simulation.controllers", "rak_arm_simulation"),
        ("rak_task_executor.node", "rak_task_executor"),
        # ADR-0020：导入一次编排层，至少保证 manifest/manager 的 import 拼写没坏
        ("rak_lifecycle.manifest", "rak_lifecycle"),
        ("rak_lifecycle.system_manager", "rak_lifecycle"),
    )
    for module_name, package_name in modules:
        module = importlib.import_module(module_name)
        prefix = Path(get_package_prefix(package_name)).resolve()
        # --symlink-install 下 ament_python 的模块真身在 build/<pkg>（符号链接回 src），
        # 所以 build 前缀同样算"来自 colcon 环境"；这仍然能抓住真正的源码树污染
        # （sys.path 里混进 ~/rak/src 之类的情形，路径前缀对不上 build/install）。
        build_prefix = prefix.parent.parent / "build" / package_name
        module_path = Path(module.__file__)
        assert prefix in module_path.parents or build_prefix in module_path.parents, (
            f"{module_name} imported outside install: {module_path} "
            f"(install={prefix}, build={build_prefix})")
        assert "build" not in module_path.parents[1].parts or build_prefix in module_path.parents, (
            f"{module_name} imported from a foreign build dir: {module_path}")

    agent_node = Path(get_package_prefix("rak_agent")) / "lib/rak_agent/agent_node"
    assert agent_node.is_file(), f"missing ROS console entry point: {agent_node}"
    # Verify the entry point by resolving it, not by string-matching the script:
    # with the setuptools EASY-INSTALL form, colcon's generated console script
    # (plain and --symlink-install alike) contains only the distribution/entry-point
    # spec, never the literal module path. Resolution is generation-independent.
    from importlib.metadata import distribution as _distribution

    entry_points = {
        ep.name: ep.value
        for ep in _distribution("rak-agent").entry_points
        if ep.group == "console_scripts"
    }
    assert entry_points.get("agent_node") == "rak_agent.node:main", (
        f"agent_node is not the rak_agent.node:main console entry point: {entry_points!r}"
    )
    result = subprocess.run(
        [sys.executable, str(ROOT / "src/rak_agent/test/test_ros_smoke.py")],
        cwd="/tmp", text=True, capture_output=True, timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    print("OK installed interfaces, Python relay and ROS agent entry point")



def _only_missing_package(stderr: str, absent) -> str | None:
    """失败原因是否**仅仅**是"引用了本机不存在的包"。是则返回包名，否则 None。

    不用正则：这条判据一旦写复杂就会被引号转义咬到（本仓今天真踩过），
    子串匹配足够且可读。
    """
    if "PackageNotFoundError" not in stderr:
        return None
    for package in sorted(absent):
        if f"package '{package}' not found" in stderr:
            return package
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--installed", action="store_true")
    parser.add_argument(
        "--profile", default="",
        help="构建档位（desktop / jetson）；按档位声明跳过硬件/GPU 包与对应 launch",
    )
    parser.add_argument(
        "--skip-absent", action="store_true",
        help="[已弃用] 等价于 --profile desktop；仅为兼容旧命令行保留",
    )
    args = parser.parse_args()
    sys.path.insert(0, str(ROOT / "src" / "rak_orchestration"))
    from rak_orchestration import profiles as profile_mod

    name = args.profile or ("desktop" if args.skip_absent else "")
    profile = profile_mod.load(name) if name else None
    if profile is not None:
        problems = profile_mod.validate(profile)
        assert not problems, problems

    found = packages()
    check_boundaries(found)
    check_launch_references(found)
    if profile is not None:
        check_assembly_layers(found, set(profile.skip_packages))
    if args.installed:
        with tempfile.TemporaryDirectory(prefix="rak-workspace-check-") as logs:
            with patch.dict(os.environ, {"ROS_LOG_DIR": logs, "PYTHONDONTWRITEBYTECODE": "1"}):
                check_installed(found, profile=profile)
    suffix = f"，档位 {profile.name}" if profile is not None else ""
    print(f"OK: {len(found)} owned packages, dependency graph and launch references{suffix}")


if __name__ == "__main__":
    main()
