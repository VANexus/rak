"""`ManagedNode` 的转移语义测试（需要 ROS：`ROS_DOMAIN_ID=77` + source 好的 overlay）。

钉住的是 ADR-0020 §5 那几条约定，不是实现细节：

1. 未激活 = 一个命令/一帧数据都不发（`LifecyclePublisher` + 基类 timer 登记）；
2. `deactivate` 释放的是"运行"，不是"资源"（原地恢复不该重开设备/重载模型）；
3. `cleanup` 才释放资源，且 cleanup→configure 可以重来（参数声明必须幂等）；
4. 转移失败要带**可读 reason** 并出现在 `/diagnostics` 里，而不是崩掉进程。
"""

from __future__ import annotations

import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.lifecycle import TransitionCallbackReturn  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from rak_lifecycle.managed_node import ManagedNode  # noqa: E402
from rak_lifecycle.testing import bring_up, new_driver, state_of, transition  # noqa: E402

NODE = "/rak_lc_probe"
TOPIC = "/rak_lc_probe/out"


class _Probe(ManagedNode):
    """一个假资源：opened/released 计数就是"资源与运行分离"这件事的可观察证据。"""

    def __init__(self, *, fail_configure=False):
        super().__init__("rak_lc_probe")
        self.opened = 0
        self.released = 0
        self.ticks = 0
        self._fail = fail_configure

    def on_configure(self, state):
        if self._fail:
            # 刻意不调用 super()：失败的 configure 不该把节点标成已配置。
            return self.fail("模拟设备打不开")
        super().on_configure(state)
        self.declare("rate_hz", 50.0)
        self._rate = float(self.get_parameter("rate_hz").value)
        self.opened += 1
        self._pub = self.lifecycle_publisher(String, TOPIC, 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        super().on_activate(state)
        self.add_timer(1.0 / self._rate, self._tick)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        super().on_cleanup(state)
        self.released += 1
        return TransitionCallbackReturn.SUCCESS

    def _tick(self):
        self.ticks += 1
        self._pub.publish(String(data=f"{self.ticks}"))


class _Watcher(Node):
    def __init__(self):
        super().__init__("rak_lc_probe_watcher")
        self.received = 0
        self.diagnostics = []
        self.create_subscription(
            String, TOPIC, lambda _m: setattr(self, "received", self.received + 1), 10
        )
        self.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diag, 10
        )

    def _on_diag(self, message):
        self.diagnostics.append(message)


@pytest.fixture()
def rig():
    if not rclpy.ok():
        rclpy.init()
    probe = _Probe()
    watcher = _Watcher()
    driver = new_driver()
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (probe, watcher, driver):
        executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        yield probe, watcher, driver
    finally:
        executor.shutdown()
        for node in (probe, watcher, driver):
            node.destroy_node()


def _wait_for(predicate, timeout=3.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return predicate()


def test_configure_does_not_publish(rig):
    probe, watcher, driver = rig
    assert transition(driver, NODE, "configure")
    assert state_of(driver, NODE) == "inactive"
    assert probe.ticks == 0
    before = watcher.received
    time.sleep(0.3)
    assert watcher.received == before, "配置未激活却在发布"


def test_activate_runs_and_publishes(rig):
    probe, watcher, driver = rig
    assert bring_up(driver, NODE)
    assert state_of(driver, NODE) == "active"
    assert _wait_for(lambda: probe.ticks > 5)
    assert _wait_for(lambda: watcher.received > 5)


def test_deactivate_stops_running_but_keeps_resource(rig):
    probe, watcher, driver = rig
    assert bring_up(driver, NODE)
    assert _wait_for(lambda: probe.ticks > 3)
    assert transition(driver, NODE, "deactivate")
    assert probe.opened == 1, "deactivate 不该释放资源（原地重启要能免重载）"
    frozen = probe.ticks
    before = watcher.received
    time.sleep(0.4)
    assert probe.ticks == frozen, "停用后 timer 仍在跑"
    assert watcher.received == before, "停用后仍在发布"


def test_cleanup_releases_and_reconfigure_is_idempotent(rig):
    probe, _watcher, driver = rig
    assert bring_up(driver, NODE)
    assert transition(driver, NODE, "deactivate")
    assert transition(driver, NODE, "cleanup")
    assert probe.released == 1
    assert state_of(driver, NODE) == "unconfigured"
    # cleanup 之后再 configure：参数声明会走第二遍，幂等性在这里被真正考验
    assert transition(driver, NODE, "configure")
    assert probe.opened == 2


def test_failed_configure_reports_a_readable_reason(rig):
    if not rclpy.ok():
        rclpy.init()
    broken = _Probe(fail_configure=True)
    watcher = _Watcher()
    driver = new_driver()
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (broken, watcher, driver):
        executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        assert not bring_up(driver, NODE)
        assert state_of(driver, NODE) == "unconfigured"
        assert _wait_for(lambda: any(
            status.level == DiagnosticStatus.ERROR and "模拟设备打不开" in status.message
            for array in watcher.diagnostics
            for status in array.status
        )), "转移失败必须在 /diagnostics 里留下可读原因"
    finally:
        executor.shutdown()
        for node in (broken, watcher, driver):
            node.destroy_node()
