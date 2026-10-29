"""`SystemManager` 的编排语义测试（需要 ROS：`ROS_DOMAIN_ID=77`）。

这里不碰任何真组件：两个域用本文件内定义的 `ManagedNode` 夹具，manifest 落在 tmp 目录。
验证的是 ADR-0020 §2/§7 的三件事：**按依赖闭包拉起**、**上游断了只降级下游**、
**操作员能停用某个域而不波及其他**。
"""

from __future__ import annotations

import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.lifecycle import TransitionCallbackReturn  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import String  # noqa: E402
import yaml  # noqa: E402

from rak_lifecycle.managed_node import ManagedNode  # noqa: E402
from rak_lifecycle.system_manager import SystemManager  # noqa: E402
from rak_lifecycle.testing import new_driver, state_of  # noqa: E402


class _Stub(ManagedNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.ticks = 0
        self.silent = False
        self._topic = f"/{name}/out"

    def on_configure(self, state):
        super().on_configure(state)
        self._pub = self.lifecycle_publisher(String, self._topic, 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        super().on_activate(state)
        self.add_timer(0.02, self._tick)
        return TransitionCallbackReturn.SUCCESS

    def _tick(self):
        self.ticks += 1
        if not self.silent:
            self._pub.publish(String(data=str(self.ticks)))


def _manifest(up: str, down: str) -> dict:
    """节点名按用例生成：同名 LifecycleNode 在同一个进程里连续创建时，CycloneDDS
    还没摘掉旧参与者，manager 的 change_state 可能被上一个用例的僵尸节点应答。"""
    return {
        "version": 1,
        "domains": [
            {
                "name": "up",
                "requires": [],
                "timeout_ms": 8000,
                "components": [{
                    "node": f"/{up}", "package": "rak_lifecycle",
                    "executable": "system_manager", "mode": "managed",
                    "owner": "system_manager", "launch": "test",
                    "provides": [{
                        "topic": f"/{up}/out", "kind": "age",
                        "max_age_ms": 300, "basis": "measured",
                    }],
                }],
            },
            {
                "name": "down",
                "requires": ["up"],
                "timeout_ms": 8000,
                "components": [{
                    "node": f"/{down}", "package": "rak_lifecycle",
                    "executable": "system_manager", "mode": "managed",
                    "owner": "system_manager", "launch": "test",
                }],
            },
        ],
    }


class _StateReader:
    def __init__(self):
        self.domains: dict[str, str] = {}

    def on_message(self, message: DiagnosticArray) -> None:
        for status in message.status:
            name = status.name.rsplit("/", 1)[-1]
            values = {pair.key: pair.value for pair in status.values}
            self.domains[name] = values.get("state", "?")


@pytest.fixture()
def rig(tmp_path, request):
    if not rclpy.ok():
        rclpy.init()
    slug = "".join(c if c.isalnum() else "_" for c in request.node.name).lower()[:16]
    up, down = f"rak_lc_up_{slug}", f"rak_lc_dn_{slug}"
    manifest = tmp_path / "manifest.yaml"
    manifest.write_text(yaml.safe_dump(_manifest(up, down)), encoding="utf-8")

    up, down = _Stub(up), _Stub(down)
    manager = SystemManager(
        manifest_path=str(manifest),
        parameter_overrides=[
            Parameter("fault_confirm_s", value=0.4),
            Parameter("monitor_rate_hz", value=10.0),
            Parameter("component_wait_timeout_s", value=8.0),
        ],
    )
    driver = new_driver()
    reader = _StateReader()
    manager.create_subscription(DiagnosticArray, "/rak/system_state", reader.on_message, 10)

    executor = MultiThreadedExecutor(num_threads=4)
    for node in (up, down, manager, driver):
        executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        yield up, down, manager, driver, reader
    finally:
        manager.shutdown()
        executor.shutdown()
        for node in (up, down, manager, driver):
            node.destroy_node()


def _wait_for(predicate, timeout=12.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return predicate()


def test_manager_brings_domains_up_in_dependency_order(rig):
    up, down, _manager, driver, reader = rig
    assert _wait_for(lambda: reader.domains.get("up") == "ready"), reader.domains
    # 上游刚 ready 时下游才刚开始被拉起；最终两者都 active
    assert _wait_for(lambda: reader.domains.get("down") == "ready"), reader.domains
    assert state_of(driver, f"/{up.get_name()}") == "active"
    assert state_of(driver, f"/{down.get_name()}") == "active"
    assert up.ticks > 0 and down.ticks > 0


def test_upstream_data_loss_degrades_only_the_downstream(rig):
    up, down, _manager, driver, reader = rig
    assert _wait_for(lambda: reader.domains.get("down") == "ready"), reader.domains

    up.silent = True  # 上游进程还活着，只是不再发数据
    assert _wait_for(lambda: reader.domains.get("up") == "faulted"), reader.domains
    assert _wait_for(lambda: state_of(driver, f"/{down.get_name()}") == "inactive"), "下游没被降级"
    # 故障传导是**向下**的：上游自己停在 inactive，不被 manager 顺手 cleanup
    assert state_of(driver, f"/{up.get_name()}") in ("active", "inactive")
    assert reader.domains.get("down") == "faulted", reader.domains


def test_operator_can_disable_a_domain_without_touching_others(rig):
    up, down, manager, driver, reader = rig
    assert _wait_for(lambda: reader.domains.get("down") == "ready"), reader.domains

    manager.set_parameters([Parameter("enabled_domains", value="up")])
    assert _wait_for(lambda: state_of(driver, f"/{down.get_name()}") == "unconfigured"), "停用没生效"
    assert _wait_for(lambda: reader.domains.get("down") == "stopped"), reader.domains
    # 上游不受影响，仍然在发布
    before = up.ticks
    time.sleep(0.3)
    assert up.ticks > before
    assert state_of(driver, f"/{up.get_name()}") == "active"


def test_retry_epoch_brings_a_faulted_domain_back(rig):
    up, down, manager, driver, reader = rig
    assert _wait_for(lambda: reader.domains.get("down") == "ready"), reader.domains

    up.silent = True
    assert _wait_for(lambda: reader.domains.get("up") == "faulted"), reader.domains
    up.silent = False

    manager.set_parameters([Parameter("retry_epoch", value=1)])
    assert _wait_for(lambda: reader.domains.get("down") == "ready"), reader.domains
    assert state_of(driver, f"/{down.get_name()}") == "active", "recover 之后下游没重新拉起"
