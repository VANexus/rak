"""真实清单与 bringup 的一致性测试（ADR-0020 的可执行门禁）。

这里钉的是"注释里的规矩"最容易漂的三件事：

1. 两份清单（真机 / 夹具）都能加载并满足 manifest 不变量；
2. **每个 `owner: system_manager` 的组件，在它声明的那个 launch 里必须是
   `LifecycleNode`** —— 用 `Node` 起一个受管节点 = 一个静默空转的节点，
   这是本仓库已经 documented 过的最坑故障形态，不能靠人记住；
3. 清单引用的 launch 文件与包都真实存在（安装空间里查得到）。

跑法：`colcon test --packages-select rak_orchestration` 或
`ROS_DOMAIN_ID=77 python3 -m pytest src/rak_orchestration/test/test_system_manifest.py`。

本文件随清单迁到 rak_orchestration（ADR-0026）：清单是组装契约，不是硬件组合，
所以桌面档也必须能跑这条门禁。
"""

from __future__ import annotations

import ast
from pathlib import Path
from types import SimpleNamespace

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

from rak_lifecycle.manifest import from_dict, load

SHARE = Path(get_package_share_directory("rak_orchestration"))
#: 源码树里的全部 launch（rak/src）。清单迁到本包之后，不能再只看本包的 share，
#: 否则"被改成 LifecycleNode 却没进清单"这类漂移就看不见了。
SRC = Path(__file__).resolve().parents[2]
#: CMake 是 install(DIRECTORY launch config ...)，所以清单在 share/<pkg>/config/ 下，
#: 不在 share 根 —— 这条路径今天被本地桌面档测试抓出来（Orin 上也会一样失败）。
RES = SHARE / "config"
MANIFESTS = ("system_manifest.yaml", "system_manifest_fixture.yaml")


def _manifests():
    return [load(RES / name) for name in MANIFESTS]


def launch_path(component):
    """`launch:` 允许写 `pkg/name.launch.py`；不带前缀时归属 rak_bringup。

    今天被本地跑出来才发现：`wbc.launch.py` 属于 rak_wbc，按"都在 bringup"去解析
    会把一个合法声明判成缺失。归属信息属于清单，不该由测试猜。
    """
    spec = component.launch.split("|")[0]
    pkg, _, name = spec.rpartition("/")
    pkg = pkg or "rak_bringup"
    return Path(get_package_share_directory(pkg)) / "launch" / name


@pytest.mark.parametrize("name", MANIFESTS)
def test_manifest_loads_and_validates(name):
    manifest = load(RES / name)
    assert manifest.order, f"{name} 里没有任何可编排的域"
    # 域依赖的传递闭包不能把自己包含进去（那意味着环，from_dict 已拒绝，这里再确认形状）
    for domain in manifest.order:
        assert domain not in manifest.dependents_of(domain)


def test_component_launch_file_exists_and_declares_the_node():
    for manifest in _manifests():
        for domain in manifest.domains:
            for component in domain.components:
                launch = launch_path(component)
                assert launch.is_file(), f"{component.node} 声明的 launch 不存在: {launch}"
                text = launch.read_text()
                assert component.executable in text, (
                    f"{launch.name} 里没有 {component.executable}"
                )


def test_system_managed_components_are_launched_as_lifecycle_nodes():
    """把"漏改 launch"变成显式失败，而不是运行时"节点在但没激活"。"""
    for manifest in _manifests():
        for domain in manifest.domains:
            for component in domain.components:
                if not component.system_managed:
                    continue
                for spec in component.launch.split("|"):
                    launch = launch_path(SimpleNamespace(launch=spec))
                    kind = _node_actions(launch).get(component.executable)
                    assert kind is not None, f"{launch} 没有启动 {component.executable}"
                    assert kind == "LifecycleNode", (
                        f"{component.node} 在清单里是 owner=system_manager 的受管组件，"
                        f"但 {launch.name} 用 {kind} 起它 —— 会停在 unconfigured 静默空转"
                    )


def test_observed_components_are_not_claimed_as_lifecycle_nodes():
    """反过来也钉：observed 的组件如果被改成 LifecycleNode，清单必须同步改 managed。

    2026-11-13 修正：这个检查以前是**在清单循环里**做的，也就是拿
    system_manifest_fixture.yaml（只声明 task_executor 一个 managed 组件）去校验
    **整个 src/ 的所有 launch**，于是 agent_node / slip_watch / harness 这些只在
    真机清单里登记的受管节点全部被误判成"没人激活"。夹具清单本来就只覆盖夹具，
    不该拿它当全集。

    正确做法：先取**所有**清单的 managed 可执行文件并集，再对每个 launch 只检查一次。
    """
    managed_executables = {
        component.executable
        for manifest in _manifests()
        for domain in manifest.domains
        for component in domain.components
        if component.mode == "managed"
    }
    assert managed_executables, "两份清单都没有任何 managed 组件，检查会变成空转"
    for launch in sorted(SRC.glob("*/launch/*.launch.py")):
        for executable, kind in _node_actions(launch).items():
            if kind != "LifecycleNode":
                continue
            assert executable in managed_executables or _is_third_party_lifecycle(
                executable
            ), (
                f"{launch.name} 把 {executable} 起成 LifecycleNode，"
                "但没有任何域把它声明为 managed —— 谁来激活它？"
            )


def _is_third_party_lifecycle(executable: str) -> bool:
    """slam_toolbox 与 Nav2 server 是上游就有的生命周期组件，拥有者不是本 manager。"""
    return executable in {
        "async_slam_toolbox_node",
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "velocity_smoother",
        "collision_monitor",
        "smoother_server",
        "waypoint_follower",
    }


def test_manifest_raw_dicts_are_the_same_shape_as_the_loaded_model(tmp_path):
    """yaml 里不能出现 schema 不认识的键 —— 否则改判据时静默无效（拼错一个 key 就够呛）。"""
    known_component_keys = {
        "node", "package", "executable", "mode", "owner", "launch", "presence",
        "role", "interface", "provides", "needs", "enabled",
    }
    known_probe_keys = {"topic", "kind", "max_age_ms", "basis"}
    known_domain_keys = {"name", "requires", "components", "timeout_ms", "enabled"}
    for name in MANIFESTS:
        raw = yaml.safe_load((RES / name).read_text())
        for domain in raw["domains"]:
            assert set(domain) <= known_domain_keys, (
                f"{name}: 未知域键 {set(domain) - known_domain_keys}"
            )
            for component in domain["components"]:
                assert set(component) <= known_component_keys, (
                    f"{name}/{component.get('node')}: 未知组件键 "
                    f"{set(component) - known_component_keys}"
                )
                for probe in list(component.get("provides", [])) + list(component.get("needs", [])):
                    assert set(probe) <= known_probe_keys, (
                        f"{name}/{component['node']}: 未知判据键 {set(probe) - known_probe_keys}"
                    )
        from_dict(raw)  # 再跑一遍不变量，确保键名修正后仍然成立


def _node_actions(launch: Path) -> dict[str, str]:
    """从 launch 源码里取 executable → 动作类名（Node / LifecycleNode / ComposableNode）。

    `managed(...)` 是 rak_orchestration.launch_helpers 的受管节点助手（它内部一定是
    LifecycleNode + configure→activate 事件），所以按 LifecycleNode 计。助手名一改，
    这里会红 —— 这正是我们要的耦合。
    """
    tree = ast.parse(launch.read_text(), filename=str(launch))
    found: dict[str, str] = {}
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call) or not call.keywords:
            continue
        action = _action_name(call.func)
        if action == "managed":
            action = "LifecycleNode"
        if action not in {"Node", "LifecycleNode", "ComposableNode"}:
            continue
        executable = _keyword(call, "executable")
        if executable:
            found[executable] = action
    return found


def _action_name(func: ast.expr) -> str:
    if isinstance(func, ast.Name):
        return func.id
    if isinstance(func, ast.Attribute):
        return func.attr
    return ""


def _keyword(call: ast.Call, name: str) -> str | None:
    for kw in call.keywords:
        if kw.arg == name and isinstance(kw.value, ast.Constant):
            return str(kw.value.value)
    return None
