"""按 manifest 编排全栈的 manager（ADR-0020 §1/§7）。

它只做三件事：**按依赖闭包把该起来的域拉起来**、**用同一套判据持续判断这个域还可用吗**、
**故障时把自己拥有的下游组件降级并把原因写进诊断**。它不是新的安全权威：
运动许可仍然只有 `allow_motion` 一条路（ADR-0020 §6），底盘兜底仍然是 F407 的
`cmd_stale` deadman（ADR-0017）。

三条设计取舍：

- **转移走标准 `lifecycle_msgs` 服务，不自己发明协议**。上游没有通用 manager
  （Orin 实测 `ros2 pkg executables lifecycle` 无 `lifecycle_manager`），所以策略层
  必须自己写，但机制层复用标准服务。
- **只降级自己拥有的组件**。Nav2 仍归 `nav2_lifecycle_manager`、slam_toolbox 仍归
  `slam.launch.py` 的事件、控制器仍归 spawner —— 对别人的组件只 `GetState` 观测、
  绝不下发转移。违反这一条就等于造出第二个生命周期权威源。
- **启动只看状态，运行只看年龄**。拉起阶段以生命周期状态与发现图为准（到第一帧是
  另一件事，用它当门会让冷启动互相等死）；进入稳态后才用话题新鲜度判降级。
- **编排跑在独立线程**：一串 `change_state` 往返是阻塞的，放在 timer 回调里会把
  参数回调与诊断发布一起卡住（4 GB 单板上 DDS 抖动是常态）。
"""

from __future__ import annotations

import argparse
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory
import diagnostic_msgs.msg
import lifecycle_msgs.msg
import lifecycle_msgs.srv
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
import rosidl_runtime_py.utilities

from .manifest import Manifest, load

STATE_PATH = "config/system_manifest.yaml"

#: `degraded` 只影响诊断级别，不触发降级；`faulted` 才是会往下游传导的那一档。
IDLE = "idle"
BRINGING_UP = "bringing_up"
READY = "ready"
DEGRADED = "degraded"
FAULTED = "faulted"
STOPPED = "stopped"

_GOAL_STATE = {
    "configure": ("inactive", "active"),
    "activate": ("active",),
    "deactivate": ("inactive", "unconfigured", "finalized"),
    "cleanup": ("unconfigured", "finalized"),
}

_LEVEL = {
    READY: diagnostic_msgs.msg.DiagnosticStatus.OK,
    DEGRADED: diagnostic_msgs.msg.DiagnosticStatus.WARN,
    BRINGING_UP: diagnostic_msgs.msg.DiagnosticStatus.WARN,
    IDLE: diagnostic_msgs.msg.DiagnosticStatus.WARN,
    # `STOP` 只存在于 REP-105 的草案里，Jazzy 的 DiagnosticStatus 没有这个常量
    # （实测 AttributeError）——"操作员主动停用"归到 STAT（无信息/未在处理）。
    STOPPED: diagnostic_msgs.msg.DiagnosticStatus.STAT,
    FAULTED: diagnostic_msgs.msg.DiagnosticStatus.ERROR,
}


@dataclass
class ComponentView:
    """一个组件的运行时视图。判据来自 manifest，值来自现场。"""

    component: Any
    state: str = IDLE
    reason: str = ""
    #: topic -> 最近一次收到帧的单调时刻（只有 `kind: age` 的判据才有）。
    last_frame: dict[str, float] = field(default_factory=dict)
    #: 年龄超限的起始时刻；连续超过 `fault_confirm_s` 才认定 FAULTED（一次抖动不算故障）。
    exceeded_since: float | None = None

    @property
    def node_name(self) -> str:
        return str(self.component.node)


@dataclass
class DomainView:
    domain: Any
    state: str = IDLE
    reason: str = ""
    components: dict[str, ComponentView] = field(default_factory=dict)

    def roll_up(self) -> tuple[str, str]:
        """把组件状态归并成域状态：最坏的那个说话，且保留是谁在说话。"""
        states = {view.state for view in self.components.values()}
        for worst in (FAULTED, BRINGING_UP, DEGRADED):
            if worst in states:
                details = "; ".join(
                    f"{view.node_name}: {view.reason}"
                    for view in self.components.values()
                    if view.state == worst and view.reason
                )
                return worst, details
        if states and states <= {STOPPED}:
            return STOPPED, ""
        if states and states <= {READY}:
            return READY, ""
        return IDLE, ""


class SystemManager(Node):
    def __init__(self, *, manifest_path: str | None = None, **node_kwargs: Any) -> None:
        super().__init__("rak_system_manager", **node_kwargs)
        self.declare_parameter("manifest", str(manifest_path or self._default_manifest()))
        self.declare_parameter("autostart", True)
        # 逗号分隔的域名单，空 = 全部启用。不用字符串数组：rclpy 从空列表推不出参数类型，
        # 之后 `ros2 param set` 会撞 WRONG_TYPE（这是操作员一定会碰到的路径）。
        self.declare_parameter("enabled_domains", "")
        #: 改这个值 = 要求把所有 FAULTED 域再试一次（避免再造一个自定义 srv）。
        self.declare_parameter("retry_epoch", 0)
        self.declare_parameter("monitor_rate_hz", 2.0)
        self.declare_parameter("component_wait_timeout_s", 30.0)
        self.declare_parameter("fault_confirm_s", 3.0)

        self.manifest: Manifest = load(self.get_parameter("manifest").value)
        self._timeout_s = float(self.get_parameter("component_wait_timeout_s").value)
        self._fault_confirm_s = float(self.get_parameter("fault_confirm_s").value)
        self._monitor_period = 1.0 / float(self.get_parameter("monitor_rate_hz").value)

        self._domains = {
            d.name: DomainView(
                domain=d,
                components={c.node: ComponentView(component=c) for c in d.components},
            )
            for d in self.manifest.domains
        }
        self._lock = threading.RLock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._transition_clients: dict[str, Any] = {}
        self._state_clients: dict[str, Any] = {}
        self._age_subs: dict[str, Any] = {}
        self._wanted: set[str] | None = None
        self._retry_epoch = int(self.get_parameter("retry_epoch").value)

        self._state_pub = self.create_publisher(
            diagnostic_msgs.msg.DiagnosticArray, "/rak/system_state", 10
        )
        self.create_timer(self._monitor_period, self._tick)
        self.add_on_set_parameters_callback(self._on_params)

        self._log(
            f"manifest={self.get_parameter('manifest').value} order={list(self.manifest.order)}"
        )
        self._worker = threading.Thread(target=self._run, name="rak-orchestrator", daemon=True)
        self._worker.start()

    @staticmethod
    def _default_manifest() -> str:
        return str(Path(get_package_share_directory("rak_bringup")) / STATE_PATH)

    def _log(self, message: str) -> None:
        self.get_logger().info(f"[system_manager] {message}")

    # ------------------------------------------------------------- 期望状态
    def _on_params(self, parameters: Any) -> SetParametersResult:
        wanted: set[str] | None = None
        for parameter in parameters:
            if parameter.name == "enabled_domains":
                wanted = {
                    name.strip()
                    for name in str(parameter.value or "").split(",")
                    if name.strip()
                }
            elif parameter.name == "retry_epoch":
                with self._lock:
                    if int(parameter.value) != self._retry_epoch:
                        self._retry_epoch = int(parameter.value)
                        for view in self._domains.values():
                            if view.state == FAULTED:
                                view.state, view.reason = IDLE, ""
                                for component_view in view.components.values():
                                    component_view.state, component_view.reason = IDLE, ""
                        self._wake.set()
        if wanted is not None:
            unknown = sorted(wanted - set(self.manifest.order))
            if unknown:
                result = SetParametersResult(successful=False)
                result.reason = f"unknown domains: {unknown}"
                return result
            with self._lock:
                self._wanted = wanted or None
        self._wake.set()
        return SetParametersResult(successful=True)

    def _desired(self) -> set[str]:
        """该活的域。`enabled_domains` 为空且 autostart=true 时 = 全部启用域。"""
        enabled = set(self.manifest.order)
        if not bool(self.get_parameter("autostart").value):
            enabled = set()
        with self._lock:
            wanted = self._wanted
        return (wanted if wanted is not None else enabled) & enabled

    # ---------------------------------------------------------------- 编排
    def _run(self) -> None:
        while not self._stop.is_set():
            self._wake.wait(timeout=self._monitor_period)
            self._wake.clear()
            if self._stop.is_set():
                break
            try:
                self._reconcile()
            except Exception as exc:  # noqa: BLE001 - 编排线程不能因一次异常退出
                self.get_logger().error(f"[system_manager] reconcile error: {exc!r}")

    def _reconcile(self) -> None:
        desired = self._desired()
        for name in self.manifest.order:
            view = self._domains[name]
            if name in desired:
                if view.state in (IDLE, STOPPED, FAULTED, BRINGING_UP) and self._upstream_ready(name):
                    self._bring_up_domain(view)
            else:
                self._tear_down_domain(view)
        self._monitor()

    def _upstream_ready(self, name: str) -> bool:
        return all(
            self._domains[requirement].state in (READY, DEGRADED)
            for requirement in self._domains[name].domain.requires
        )

    def _bring_up_domain(self, view: DomainView) -> None:
        view.state, view.reason = BRINGING_UP, ""
        self._log(f"bringing up {view.domain.name} ({len(view.components)} components)")
        deadline = time.monotonic() + max(
            view.domain.timeout_ms / 1000.0, self._timeout_s
        )
        for component in view.domain.components:
            if not component.enabled:
                self._component(view, component.node).state = STOPPED
                continue
            self._bring_up_component(view, component, deadline)
        view.state, view.reason = view.roll_up()
        suffix = f" ({view.reason})" if view.reason else ""
        self._log(f"domain {view.domain.name} -> {view.state}{suffix}")

    def _bring_up_component(self, view: DomainView, component: Any, deadline: float) -> None:
        cv = self._component(view, component.node)
        cv.state, cv.reason = BRINGING_UP, ""
        if component.presence == "node":
            if not self._wait_for_node(component.node, deadline):
                cv.state, cv.reason = FAULTED, (
                    f"节点 {component.node} 没出现在发现图里"
                    f"（launch 起了吗？package={component.package} executable={component.executable}）"
                )
                return
        for probe in component.provides:
            self._watch(cv, probe)
        if component.presence == "topic":
            ok, detail = self._await_topics(component, deadline)
            if not ok:
                cv.state, cv.reason = FAULTED, detail
                return

        if component.system_managed:
            ok, detail = self._ensure_active(component.node, deadline)
            if not ok:
                cv.state, cv.reason = FAULTED, detail
                return
            cv.state = READY
            return

        if component.mode == "managed":
            # 外部拥有（nav2 manager / slam.launch.py 事件 / spawner）：只等不推。
            ok, detail = self._await_state(component.node, ("active",), deadline)
            cv.state = READY if ok else FAULTED
            cv.reason = "" if ok else f"等不到 active（拥有者 {component.owner}）：{detail}"
            return

        cv.state = READY

    def _tear_down_domain(self, view: DomainView) -> None:
        if view.state == STOPPED:
            return
        for component in reversed(view.domain.components):
            cv = view.components[component.node]
            if component.system_managed and cv.state != STOPPED:
                horizon = time.monotonic() + 10.0
                self._transition(component.node, "deactivate", horizon)
                self._transition(component.node, "cleanup", horizon)
            cv.state, cv.reason, cv.exceeded_since = STOPPED, "", None
        view.state, view.reason = STOPPED, ""
        self._log(f"domain {view.domain.name} stopped")

    def _downgrade_dependents(self, name: str) -> None:
        """上游故障 → 下游里**自己拥有的**组件停用；外部拥有的只报错不碰（单一所有者规则）。"""
        for dependent in sorted(self.manifest.dependents_of(name)):
            view = self._domains[dependent]
            if view.state in (IDLE, STOPPED):
                continue
            for component in view.domain.components:
                cv = view.components[component.node]
                if component.system_managed and cv.state != STOPPED:
                    self._transition(
                        component.node, "deactivate", time.monotonic() + 10.0
                    )
                    cv.state, cv.reason, cv.exceeded_since = (
                        STOPPED,
                        f"上游域 {name} FAULTED",
                        None,
                    )
            view.state, view.reason = view.roll_up()
            if view.state not in (STOPPED, FAULTED):
                # 剩下的都是别人拥有的：本 manager 无权停它们，只能把域标成不可用。
                view.state, view.reason = FAULTED, (
                    f"上游域 {name} FAULTED；本域内外部拥有的组件由其拥有者负责"
                )
            self._log(f"degraded dependent domain {dependent} -> {view.state}")

    # ------------------------------------------------------------- 运行期判据
    def _monitor(self) -> None:
        now = time.monotonic()
        for name, view in self._domains.items():
            if view.state not in (READY, DEGRADED):
                continue
            changed = False
            for component in view.domain.components:
                cv = view.components[component.node]
                if cv.state not in (READY, DEGRADED, FAULTED):
                    continue
                for probe in component.provides:
                    if probe.kind != "age" or probe.topic not in cv.last_frame:
                        continue
                    last = cv.last_frame[probe.topic]
                    age_ms = (now - last) * 1000.0
                    if age_ms <= (probe.max_age_ms or 0.0):
                        if cv.state in (DEGRADED, FAULTED):
                            cv.state, cv.reason, cv.exceeded_since = READY, "", None
                            changed = True
                        continue
                    if not probe.escalation_allowed:
                        cv.state, cv.reason = DEGRADED, (
                            f"{probe.topic} 断了 {age_ms:.0f} ms"
                            f"（basis={probe.basis}，只告警不降级）"
                        )
                        changed = True
                        continue
                    if cv.exceeded_since is None:
                        cv.exceeded_since = now
                    elif now - cv.exceeded_since >= self._fault_confirm_s:
                        cv.state, cv.reason = FAULTED, (
                            f"{probe.topic} 超过 {probe.max_age_ms:.0f} ms 没来数据"
                        )
                        changed = True
            if changed:
                view.state, view.reason = view.roll_up()
                if view.state == FAULTED:
                    self._log(f"domain {name} FAULTED: {view.reason}")
                    self._downgrade_dependents(name)

    def _component(self, view: DomainView, node: str) -> ComponentView:
        return view.components[node]

    def _watch(self, cv: ComponentView, probe: Any) -> None:
        """`kind: age` 需要知道消息类型才能订阅；解析不到就退化成发现图检查。"""
        if probe.kind != "age" or probe.topic in self._age_subs:
            return
        message_type = self._resolve_type(probe.topic)
        if message_type is None:
            return

        def on_message(_msg: Any, topic: str = probe.topic, view: ComponentView = cv) -> None:
            view.last_frame[topic] = time.monotonic()

        # 点云/图像这类重负载话题在 manifest 里应写 kind: exists；这里仍给最小队列深度兜底。
        depth = 1 if ("image" in probe.topic or "points" in probe.topic) else 10
        self._age_subs[probe.topic] = self.create_subscription(
            message_type, probe.topic, on_message, depth
        )

    def _await_topics(self, component: Any, deadline: float) -> tuple[bool, str]:
        """`presence: topic` 的组件：等它的 `exists` 判据在发现图里出现。"""
        wanted = [probe.topic for probe in component.provides if probe.kind == "exists"]
        missing = list(wanted)
        while time.monotonic() < deadline and not self._stop.is_set():
            names = {name for name, _types in self.get_topic_names_and_types()}
            missing = [topic for topic in wanted if topic not in names]
            if not missing:
                return True, ""
            time.sleep(0.1)
        return False, (
            f"这些话题一直没出现: {missing}"
            f"（{component.package}/{component.executable} 起来了吗？）"
        )

    def _resolve_type(self, topic: str) -> Any:
        for name, types in self.get_topic_names_and_types():
            if name == topic and types:
                try:
                    return rosidl_runtime_py.utilities.get_message(types[0])
                except (ImportError, ValueError):
                    return None
        return None

    # ------------------------------------------------------- 发现与服务往返
    def _wait_for_node(self, node_fqn: str, deadline: float) -> bool:
        short = node_fqn.lstrip("/")
        while time.monotonic() < deadline and not self._stop.is_set():
            if short in self.get_node_names():
                return True
            time.sleep(0.1)
        return False

    def _change_client(self, node_fqn: str) -> Any:
        key = f"{node_fqn}/change_state"
        if key not in self._transition_clients:
            self._transition_clients[key] = self.create_client(
                lifecycle_msgs.srv.ChangeState, key
            )
        return self._transition_clients[key]

    def _state_client(self, node_fqn: str) -> Any:
        key = f"{node_fqn}/get_state"
        if key not in self._state_clients:
            self._state_clients[key] = self.create_client(lifecycle_msgs.srv.GetState, key)
        return self._state_clients[key]

    def _await(self, future: Any, timeout: float) -> Any:
        """rclpy 的 `task.Future` 只有 `done()`/`result()`，没有 `wait_for_result` —— 显式轮询。"""
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline and not self._stop.is_set():
            time.sleep(0.02)
        return future.result() if future.done() else None

    def _current_state(self, node_fqn: str) -> str | None:
        client = self._state_client(node_fqn)
        if not client.wait_for_service(timeout_sec=0.2):
            return None
        response = self._await(
            client.call_async(lifecycle_msgs.srv.GetState.Request()), 1.0
        )
        return response.current_state.label if response is not None else None

    def _await_state(self, node_fqn: str, goals: tuple[str, ...], deadline: float) -> tuple[bool, str]:
        while time.monotonic() < deadline and not self._stop.is_set():
            state = self._current_state(node_fqn)
            if state in goals:
                return True, ""
            time.sleep(0.1)
        return False, f"当前 {self._current_state(node_fqn)!r}，目标 {goals}"

    def _ensure_active(self, node_fqn: str, deadline: float) -> tuple[bool, str]:
        """把组件推到 active，**按当前状态走最短路径**。

        盲目发 configure 是错的：recover 一个只是"上游断了"的域时节点还停在 active/inactive，
        从 inactive 发 configure 是非法转移，会被节点拒绝 —— 于是"恢复"必然失败。
        """
        while time.monotonic() < deadline and not self._stop.is_set():
            state = self._current_state(node_fqn)
            if state == "active":
                return True, ""
            if state in ("unconfigured", "finalized"):
                ok, detail = self._transition(node_fqn, "configure", deadline)
                if not ok:
                    return False, f"configure 失败：{detail}"
                continue
            if state == "inactive":
                ok, detail = self._transition(node_fqn, "activate", deadline)
                if not ok:
                    return False, f"activate 失败：{detail}"
                continue
            time.sleep(0.1)  # configuring/transitioning，或转移服务还没就绪
        return False, f"{deadline - time.monotonic():.1f} s 内没能推进到 active（最后状态 {self._current_state(node_fqn)!r}）"

    def _transition(self, node_fqn: str, verb: str, deadline: float) -> tuple[bool, str]:
        client = self._change_client(node_fqn)
        remaining = max(0.5, deadline - time.monotonic())
        if not client.wait_for_service(timeout_sec=min(5.0, remaining)):
            return False, f"{node_fqn}/change_state 服务不可用"
        description = getattr(lifecycle_msgs.msg.TransitionDescription, verb.upper())
        future = client.call_async(
            lifecycle_msgs.srv.ChangeState.Request(
                transition=lifecycle_msgs.msg.Transition(id=description, label=verb)
            )
        )
        response = self._await(future, remaining)
        if response is None:
            return False, f"{verb} 请求 {remaining:.1f} s 内没有回应"
        if not response.success:
            return False, f"{verb} 被节点拒绝（节点自己上报失败原因，见 /diagnostics）"
        return self._await_state(node_fqn, _GOAL_STATE[verb], deadline)

    # ------------------------------------------------------------ 聚合发布
    def _tick(self) -> None:
        self._wake.set()
        self._publish()

    def _publish(self) -> None:
        now = time.monotonic()
        desired = self._desired()
        statuses = []
        for name in self.manifest.order:
            view = self._domains[name]
            values = [
                diagnostic_msgs.msg.KeyValue(key="state", value=view.state),
                diagnostic_msgs.msg.KeyValue(key="desired", value=str(name in desired)),
                diagnostic_msgs.msg.KeyValue(
                    key="requires", value=",".join(view.domain.requires) or "-"
                ),
            ]
            for node, cv in sorted(view.components.items()):
                values.append(
                    diagnostic_msgs.msg.KeyValue(
                        key=node.lstrip("/"), value=f"{cv.state} {cv.reason}".strip()
                    )
                )
                for topic, last in sorted(cv.last_frame.items()):
                    values.append(
                        diagnostic_msgs.msg.KeyValue(
                            key=f"age_ms {topic}", value=f"{(now - last) * 1000.0:.0f}"
                        )
                    )
            if name not in desired:
                message = f"disabled by operator ({view.state})"
            else:
                message = view.reason or view.state
            statuses.append(
                diagnostic_msgs.msg.DiagnosticStatus(
                    level=_LEVEL[view.state],
                    name=f"rak/domain/{name}",
                    message=message,
                    hardware_id=name,
                    values=values,
                )
            )
        self._state_pub.publish(diagnostic_msgs.msg.DiagnosticArray(statuses=statuses))

    def shutdown(self) -> None:
        self._stop.set()
        self._wake.set()


def main() -> int:
    parser = argparse.ArgumentParser(prog="rak_system_manager")
    parser.add_argument("--manifest", default=None, help="system_manifest.yaml 路径")
    args, _unknown = parser.parse_known_args()
    rclpy.init()
    node = SystemManager(manifest_path=args.manifest)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
