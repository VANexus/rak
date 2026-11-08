"""`ManagedNode` 的转移语义测试（需要 ROS：`ROS_DOMAIN_ID=77` + source 好的 overlay）。

钉住的是 ADR-0020 §5 那几条约定，不是实现细节：

1. 未激活 = 一帧都不发（受管发布器由 rclpy 的默认钩子启用，基类必须转发 super()）；
2. `deactivate` 释放的是"运行"，不是"资源"（原地恢复不该重开设备/重载模型）；
3. `cleanup` 才释放资源，且 cleanup→configure 可以重来（参数声明必须幂等）；
4. 转移失败要带**可读 reason** 并出现在 `/diagnostics` 里，而不是崩掉进程。

每个用例用**自己的节点名**：同一进程里连续创建同名 LifecycleNode 时，CycloneDDS 的发现图
还没来得及摘掉上一个参与者，`change_state` 会被旧节点应答，表现为"失败用例意外成功"
（本轮实测踩过：healthy 探针替 broken 探针答了转移）。
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


class _Probe(ManagedNode):
    """一个假资源：opened/released 计数就是"资源与运行分离"的可观察证据。"""

    def __init__(self, name: str, *, fail_configure: bool = False) -> None:
        super().__init__(name)
        self.topic = f"/{name}/out"
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
        self._pub = self.lifecycle_publisher(String, self.topic, 10)
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
        self._pub.publish(String(data=str(self.ticks)))


class _Watcher(Node):
    def __init__(self, name: str, topic: str) -> None:
        super().__init__(f"{name}_watcher")
        self.received = 0
        self.diagnostics: list[DiagnosticArray] = []
        self.create_subscription(
            String, topic, lambda _m: setattr(self, "received", self.received + 1), 10
        )
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diag, 10)

    def _on_diag(self, message: DiagnosticArray) -> None:
        self.diagnostics.append(message)


class _Rig:
    """一次实验的全部进程内对象：一个探针 + 一个观察者 + 一个转移驱动者。"""

    def __init__(self, name: str, probe: _Probe) -> None:
        self.probe = probe
        self.watcher = _Watcher(name, probe.topic)
        self.driver = new_driver()
        self.node_fqn = f"/{name}"
        self.executor = MultiThreadedExecutor(num_threads=4)
        for node in (probe, self.watcher, self.driver):
            self.executor.add_node(node)
        self.spin = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin.start()

    def teardown(self) -> None:
        self.executor.shutdown()
        for node in (self.probe, self.watcher, self.driver):
            node.destroy_node()

    def error_reasons(self) -> list[str]:
        return [
            status.message
            for array in self.watcher.diagnostics
            for status in array.status
            if status.level == DiagnosticStatus.ERROR
        ]


def _slug(test_name: str) -> str:
    """节点名只允许 [a-z0-9_] 且要短 —— 用例名当后缀，天然互不重名。"""
    return "".join(c if c.isalnum() else "_" for c in test_name).lower()[:24]


def _make_rig(prefix: str, test_name: str, **probe_kwargs) -> _Rig:
    if not rclpy.ok():
        rclpy.init()
    name = f"rak_lc_{prefix}_{_slug(test_name)}"
    return _Rig(name, _Probe(name, **probe_kwargs))


@pytest.fixture()
def rig(request):
    built = _make_rig("p", request.node.name)
    try:
        yield built
    finally:
        built.teardown()


@pytest.fixture()
def failing_rig(request):
    built = _make_rig("f", request.node.name, fail_configure=True)
    try:
        yield built
    finally:
        built.teardown()


def _wait_for(predicate, timeout: float = 3.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return bool(predicate())


def test_configure_does_not_publish(rig):
    assert transition(rig.driver, rig.node_fqn, "configure")
    assert state_of(rig.driver, rig.node_fqn) == "inactive"
    assert rig.probe.ticks == 0
    before = rig.watcher.received
    time.sleep(0.3)
    assert rig.watcher.received == before, "配置未激活却在发布"


def test_activate_runs_and_publishes(rig):
    assert bring_up(rig.driver, rig.node_fqn)
    assert state_of(rig.driver, rig.node_fqn) == "active"
    assert _wait_for(lambda: rig.probe.ticks > 5)
    assert _wait_for(lambda: rig.watcher.received > 5), "激活后受管发布器一帧都没送达"


def test_deactivate_stops_running_but_keeps_resource(rig):
    assert bring_up(rig.driver, rig.node_fqn)
    assert _wait_for(lambda: rig.probe.ticks > 3)
    assert transition(rig.driver, rig.node_fqn, "deactivate")
    assert rig.probe.opened == 1, "deactivate 不该释放资源（原地重启要能免重载）"
    frozen = rig.probe.ticks
    before = rig.watcher.received
    time.sleep(0.4)
    assert rig.probe.ticks == frozen, "停用后 timer 仍在跑"
    assert rig.watcher.received == before, "停用后仍在发布"


def test_cleanup_releases_and_reconfigure_is_idempotent(rig):
    assert bring_up(rig.driver, rig.node_fqn)
    assert transition(rig.driver, rig.node_fqn, "deactivate")
    assert transition(rig.driver, rig.node_fqn, "cleanup")
    assert rig.probe.released == 1
    assert state_of(rig.driver, rig.node_fqn) == "unconfigured"
    # cleanup 之后再 configure：参数声明会走第二遍，幂等性在这里被真正考验
    assert transition(rig.driver, rig.node_fqn, "configure")
    assert rig.probe.opened == 2


def test_failed_configure_reports_a_readable_reason(failing_rig):
    assert not bring_up(failing_rig.driver, failing_rig.node_fqn)
    assert state_of(failing_rig.driver, failing_rig.node_fqn) == "unconfigured"
    assert _wait_for(lambda: any(
        "模拟设备打不开" in message for message in failing_rig.error_reasons()
    )), f"转移失败没在 /diagnostics 里留下可读原因：{failing_rig.error_reasons()}"
