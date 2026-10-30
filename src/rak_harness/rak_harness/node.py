"""harness_node: arbitration seam + voice dialogue loop (ADR-0010).

只读上下文:/brain/decision(keep-last-N)、/speech/intent、/task/status、
/base/telemetry、/speech/wake。写出:/harness/proposal、/harness/event、
/task/execute(结构化目标)、/task/control、/speech/say(回话)。
永不发布速度/关节命令。

生命周期（ADR-0020 §5）:
  `on_configure` 声明参数、建 pub/sub/client、装载 persona 与对话引擎；
  `on_activate` 才起云端 worker 线程并**拉起 agent 子进程**——子进程是本节点最贵的
  资源，也是最容易被中断的那一个，所以它必须活在 activate/inactive 之间，而不是
  构造期。`on_deactivate` 关掉子进程与线程，`on_cleanup` 释放全部句柄。
"""
from __future__ import annotations

import json
import threading
import time
import uuid
from collections import deque
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import String

from rak_interfaces.action import ExecuteTask
from rak_interfaces.msg import (
    BaseTelemetry,
    Decision,
    HarnessEvent,
    HarnessProposal,
    HarnessRequest,
    TaskStatus,
    WakeEvent,
)
from rak_interfaces.srv import Say, TaskControl
from rak_intent.task_intent import parse_control_command, parse_navigation_intent
from rak_lifecycle.managed_node import ManagedNode

from .cloud_client import CloudUnavailable, HarnessCloud
from .contract import (
    KIND_NO_OP,
    KIND_REQUEST_CLARIFICATION,
    KIND_SELECT_WAYPOINT,
    SCHEMA_VERSION,
)
from .dialogue import DialogueEngine, TERMINAL_STATES
from .persona import (
    VehicleSnapshot,
    load_persona,
    render,
    snapshot_to_text,
)

_STATE_LABEL = {
    TaskStatus.IDLE: "IDLE", TaskStatus.ACCEPTED: "ACCEPTED",
    TaskStatus.PLANNING: "PLANNING", TaskStatus.NAVIGATING: "NAVIGATING",
    TaskStatus.PAUSING: "PAUSING", TaskStatus.PAUSED: "PAUSED",
    TaskStatus.RESUMING: "RESUMING", TaskStatus.CANCELING: "CANCELING",
    TaskStatus.SUCCEEDED: "SUCCEEDED", TaskStatus.FAILED: "FAILED",
    TaskStatus.CANCELED: "CANCELED", TaskStatus.SAFETY_HOLD: "SAFETY_HOLD",
}

_EVENT_KIND = {"ACK": HarnessEvent.KIND_ACK, "PROGRESS": HarnessEvent.KIND_PROGRESS,
               "RECOVERY": HarnessEvent.KIND_RECOVERY,
               "CLARIFICATION": HarnessEvent.KIND_CLARIFICATION}


class HarnessNode(ManagedNode):
    """ADR-0010 仲裁缝 + 语音对话环，按 ADR-0020 §5 受管化。

    configure：参数、pub/sub、对话引擎、云端客户端（纯内存 + 一次 persona 读）。
    activate ：云端 worker 线程 + **agent 子进程**（`agent_bridge`）。
    deactivate：停线程、关子进程（`deactivate → activate` 不泄漏子进程）。
    cleanup  ：再释放 client/订阅/发布器，回到可重配状态。
    """

    def __init__(self) -> None:
        # 只做字段初始化：不建 pub/sub、不起线程、不拉 persona（ADR-0020 §5）。
        super().__init__("harness")
        self._lock = threading.RLock()
        self._snapshot = VehicleSnapshot()
        self._decisions: deque[str] = deque(maxlen=20)
        self._last_destination = ""
        self._proposal_slot: dict | None = None  # executor request, cap-1 queue

        # configure 期填充
        self._configured = False
        self._persona = None
        self._engine = None
        self._cloud = None
        self._ttl = 20.0
        self._attention_sec = 12.0
        self._voice_path = "auto"
        self._agent_sdk_path = ""
        self._allow_motion_tools = False
        self._proposal_pub = None
        self._event_pub = None
        self._trace_pub = None
        self._intent_sub = None
        self._request_sub = None
        self._subs: list = []
        self._say_client = None
        self._control_client = None
        self._task_client = None

        # activate 期填充
        self._agent = None
        self._agent_active = False
        self._agent_state = "down"
        self._agent_detail = ""
        self._proposals_sent = 0
        self._inflight_ticks = 0
        self._stats_t0 = time.monotonic()
        self._stop = threading.Event()
        self._worker: threading.Thread | None = None

    # ---------------------------------------------------------------- configure
    def on_configure(self, state):
        ret = super().on_configure(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret
        try:
            self._attention_sec = float(self.declare("attention_sec", 12.0))
            persona_file = str(self.declare("persona_file", "")) or self._default_persona()
            self._ttl = float(self.declare("proposal_ttl_sec", 20.0))
            cloud_model = str(self.declare("cloud_model", "deepseek-chat"))
            cloud_calls = int(self.declare("cloud_calls_per_task", 3))
            # voice_path: auto(agent if SDK present) | agent | dialogue(ADR-0010 only)
            self._voice_path = str(self.declare("voice_path", "auto"))
            # 空 = 自动解析（参数 → RAK_HARNESS_SDK_DIR → ament 索引 → 源码树）。
            # 绝不能写死家目录：换人/换机即坏，且坏法伪装成"模型没配好"（AGENTS §2 第 3 条）。
            self._agent_sdk_path = str(self.declare("agent_sdk_path", ""))
            self._allow_motion_tools = bool(self.declare("agent_allow_motion_tools", False))

            self._persona = load_persona(persona_file)
            self._engine = DialogueEngine(attention_sec=self._attention_sec)
            self._cloud = HarnessCloud(model=cloud_model, max_calls_per_task=cloud_calls)

            # 未激活不发布：LifecyclePublisher 在 on_activate 之前丢弃 publish()
            self._proposal_pub = self.lifecycle_publisher(
                HarnessProposal, "/harness/proposal", 10)
            self._event_pub = self.lifecycle_publisher(HarnessEvent, "/harness/event", 10)
            # /harness/trace 在 configure 就建：agent 子进程在 activate 起来后立刻会写，
            # 那时再 create_publisher 会拿到一个未启用的发布器，trace 静默丢帧。
            self._trace_pub = self.lifecycle_publisher(String, "/harness/trace", 10)

            self._request_sub = self._subscribe(
                HarnessRequest, "/harness/request", self._on_request)
            self._subscribe(WakeEvent, "/speech/wake", self._on_wake)
            # Voice routing: the agent owns /speech/transcript; the ADR-0010 dialogue
            # engine keeps /speech/intent until the agent proves itself (status up).
            self._subscribe(TaskStatus, "/task/status", self._on_status)
            self._subscribe(BaseTelemetry, "/base/telemetry", self._on_telemetry)
            self._subscribe(Decision, "/brain/decision", self._on_decision)

            self._task_client = ActionClient(self, ExecuteTask, "/task/execute")
            self._control_client = self.create_client(TaskControl, "/task/control")
            self._say_client = self.create_client(Say, "/speech/say")
        except Exception as exc:  # noqa: BLE001 - 配置失败必须变成可读的 FAILURE
            self._release_ros()
            return self.fail(f"配置 harness 失败: {exc}")

        self._configured = True
        cloud_state = "live" if self._cloud.available() else "stub"
        self.report(
            "voice", DiagnosticStatus.OK,
            f"voice_path={self._voice_path} cloud={cloud_state}",
            {"voice_path": self._voice_path, "cloud": cloud_state,
             "persona": self._persona.name},
        )
        self.get_logger().info(
            f"harness configured: persona={self._persona.name} cloud={cloud_state} "
            f"voice_path={self._voice_path}")
        return TransitionCallbackReturn.SUCCESS

    # ----------------------------------------------------------------- activate
    def on_activate(self, state):
        ret = super().on_activate(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret
        try:
            # 每次激活一个新 event：旧 worker 即便还在收尾，也只认自己那个。
            self._stop = threading.Event()
            self._worker = threading.Thread(
                target=self._run, args=(self._stop,), name="harness-cloud", daemon=True)
            self._worker.start()
            self._proposals_sent = 0
            self._stats_t0 = time.monotonic()
            # 最贵的资源在 activate 才拉起来（可被打断 → 值得放在这里）
            self._start_agent()
        except Exception as exc:  # noqa: BLE001 - 子进程起不来不该杀掉进程
            self._stop_agent()
            self._stop_worker()
            return self.fail(f"激活 harness 失败: {exc}")
        self.add_timer(5.0, self._stats)
        return TransitionCallbackReturn.SUCCESS

    # --------------------------------------------------------------- deactivate
    def on_deactivate(self, state):
        self._stop_agent()
        self._stop_worker()
        self._agent_active = False
        self._agent_state = "down"
        with self._lock:
            self._proposal_slot = None
        return super().on_deactivate(state)

    # ------------------------------------------------------------------ cleanup
    def on_cleanup(self, state):
        self._stop_agent()
        self._stop_worker()
        self._release_ros()
        self._engine = None
        self._cloud = None
        self._persona = None
        self._agent_active = False
        self._agent_state = "down"
        with self._lock:
            self._snapshot = VehicleSnapshot()
            self._decisions.clear()
            self._last_destination = ""
            self._proposal_slot = None
        return super().on_cleanup(state)

    def on_shutdown(self, state):
        self._stop_agent()
        self._stop_worker()
        ret = super().on_shutdown(state)
        self._release_ros()
        return ret

    # -- P1 agentic loop ------------------------------------------------------

    def _start_agent(self) -> None:
        if self._voice_path == "dialogue":
            self.get_logger().info("voice_path=dialogue: agent loop disabled")
            self._agent_state, self._agent_detail = "down", "voice_path=dialogue"
            self._subscribe_intent()
            return
        from .sdk import explain, resolve

        sdk_path = resolve(self._agent_sdk_path)
        if sdk_path is None:
            message = (f"voice_path={self._voice_path}: 没有可用的 SDK 覆盖层; "
                       f"{explain(self._agent_sdk_path)}")
            self._agent_state, self._agent_detail = "down", "sdk overlay missing"
            if self._voice_path == "auto":
                self.get_logger().warning(message + " — 降级到 dialogue")
                self._subscribe_intent()
                return
            # 显式 agent 却没 SDK：这是配置错误，不要让操作员以为在跑 agent。
            self.get_logger().error(message)
            self._subscribe_intent()
            return
        self.get_logger().info(f"agent SDK overlay: {sdk_path}")
        from .agent_bridge import AgentBridge

        self._agent = AgentBridge(
            self,
            sdk_path=str(sdk_path),
            model=str(self.get_parameter("cloud_model").value),
            allow_motion_tools=self._allow_motion_tools,
            trace_pub=self._trace_pub,
            say=self._say,
            on_state=self._on_agent_state,
            context_provider=self._agent_context,
        )

    def _stop_agent(self) -> None:
        """关子进程；对 deactivate/cleanup 重复调用安全。"""
        agent, self._agent = self._agent, None
        if agent is None:
            return
        try:
            agent.close()
        except Exception as exc:  # noqa: BLE001 - 关不干净也不许影响转移
            self.get_logger().warning(f"agent bridge close failed: {exc}")

    def _agent_context(self) -> dict:
        """What the loop needs about the body each turn (digest renders it)."""
        with self._lock:
            context = {"vehicle": snapshot_to_text(self._snapshot)}
            if self._decisions:
                context["decision"] = "近期S1决策: " + "/".join(
                    str(d) for d in list(self._decisions)[-3:])
            if self._snapshot.wake_angle is not None:
                context["wake"] = f"声源方位≈{self._snapshot.wake_angle:.0f}°"
        return context

    def _on_agent_state(self, state: str, detail: str) -> None:
        self._agent_state = state
        self._agent_detail = detail or ""
        self._agent_active = state == "up"
        if state == "up":
            self.get_logger().info(f"agent loop voice path active: {detail}")
        else:
            self.get_logger().warn(
                f"agent loop {state}: {detail or 'no detail'}; falling back to dialogue voice")
            self._subscribe_intent()

    def _subscribe_intent(self) -> None:
        # 未 configure / 已 cleanup 时不许再建订阅（子进程收尾线程可能晚一步回来）。
        if self._intent_sub is not None or not self._configured:
            return
        self._intent_sub = self._subscribe(String, "/speech/intent", self._on_intent)

    @staticmethod
    def _default_persona() -> str:
        for start in (Path(__file__).resolve().parent, Path.cwd()):
            for base in (start, *start.parents):
                candidate = base / "config" / "persona.toml"
                if candidate.is_file():
                    return str(candidate)
                candidate = base / "share" / "rak_harness" / "config" / "persona.toml"
                if candidate.is_file():
                    return str(candidate)
        return ""

    # -- context snapshot ---------------------------------------------------

    def _vehicle_ok(self) -> tuple[bool, str]:
        snap = self._snapshot
        if snap.base_fault:
            return False, f"底盘故障0x{snap.base_fault:04X}"
        if not snap.base_connected:
            return False, "底盘未连接"
        return True, ""

    # -- subscriptions (executor thread: fast paths only) --------------------

    def _on_wake(self, msg: WakeEvent) -> None:
        with self._lock:
            self._snapshot.wake_angle = float(msg.angle)
            self._snapshot.wake_beam = int(msg.beam)
            self._engine.on_wake(now=time.time())
        self.get_logger().info(
            f"attentive: '{msg.keyword}' angle={msg.angle:.0f}° beam={msg.beam}")

    def _on_intent(self, msg: String) -> None:
        if self._agent_active:
            return  # the agent loop owns the voice path once it is up
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        text = str(payload.get("source_text", ""))
        kind = payload.get("kind")
        destination = str(payload.get("destination", "")) if kind == "navigate" else ""
        command = None
        if kind == "control":
            parsed = parse_control_command(text)
            command = parsed.value if parsed is not None else None
            if command is None:
                kind = None
        if kind == "navigate":
            nav = parse_navigation_intent(text)
            if nav is None:
                kind = None
            else:
                destination = nav.destination
        with self._lock:
            if kind == "navigate":
                self._last_destination = destination
            out = self._engine.on_intent(
                kind=kind, text=text, destination=destination,
                command=command, now=time.time())
            ok, note = self._vehicle_ok()
            if kind == "navigate" and not ok:
                out.submit_task.clear()
                out.say = [("fault_hold" if self._snapshot.base_fault else "disconnected", note)]
        self._execute_commands(out)

    def _on_status(self, msg: TaskStatus) -> None:
        with self._lock:
            self._snapshot.task_state = _STATE_LABEL.get(msg.state, "?")
            self._snapshot.task_id = msg.task_id
            ok, note = self._vehicle_ok()
            dest = self._last_destination
            if msg.state in (TaskStatus.SUCCEEDED,) and msg.task_id:
                self._last_destination = ""
            out = self._engine.on_task_status(
                task_id=msg.task_id, revision=msg.revision,
                state=_STATE_LABEL.get(msg.state, "?"), reason=msg.reason,
                dest=dest, vehicle_ok=ok, vehicle_note=note, now=time.time())
        self._execute_commands(out)

    def _on_telemetry(self, msg: BaseTelemetry) -> None:
        with self._lock:
            self._snapshot.base_connected = bool(msg.connected and msg.handshake_ok)
            self._snapshot.base_armed = bool(msg.armed)
            self._snapshot.base_fault = int(msg.fault_bitmap)
            self._snapshot.twist_x = float(msg.measured_twist.linear.x)
            self._snapshot.twist_z = float(msg.measured_twist.angular.z)

    def _on_decision(self, msg: Decision) -> None:
        with self._lock:
            self._decisions.append(f"{msg.source}:{msg.tier}")
            self._snapshot.recent_tiers = tuple(self._decisions)[-3:]

    def _on_request(self, msg: HarnessRequest) -> None:
        """Executor planning request: cap-1 slot, worker answers with a proposal."""
        with self._lock:
            self._proposal_slot = {
                "task_id": msg.task_id, "revision": msg.revision,
                "request_id": msg.request_id, "text": msg.text,
                "allowed": list(msg.allowed_waypoints),
                "deadline": float(msg.deadline_epoch_sec),
            }

    # -- worker: cloud proposals (fire-and-forget, never blocks executor) ----

    def _run(self, stop: threading.Event) -> None:
        while not stop.is_set():
            time.sleep(0.2)
            cloud = self._cloud  # cleanup 可能已经把客户端放掉了
            if cloud is None or stop.is_set():
                return
            with self._lock:
                slot = self._proposal_slot
                self._proposal_slot = None
                snapshot = self._snapshot
            if slot is None:
                continue
            now = time.time()
            if now > slot["deadline"]:
                continue  # expired before we started: never retry
            nav = parse_navigation_intent(slot["text"])
            destination = nav.destination if nav is not None else ""
            context = snapshot_to_text(snapshot)
            expires = min(slot["deadline"], now + self._ttl)
            try:
                if cloud.available():
                    data = cloud.propose(
                        text=slot["text"], destination=destination,
                        allowed_waypoints=slot["allowed"], context_text=context,
                        task_id=slot["task_id"], expires_epoch_sec=expires)
                else:
                    data = cloud.propose_local(
                        text=slot["text"], destination=destination,
                        allowed_waypoints=slot["allowed"], task_id=slot["task_id"])
            except CloudUnavailable as exc:
                self._publish_proposal(slot, KIND_NO_OP, "", f"云不可用:{exc}", expires)
                continue
            if data.kind == KIND_SELECT_WAYPOINT and data.waypoint not in slot["allowed"]:
                self._publish_proposal(slot, KIND_NO_OP, "", "云提案越过白名单,已丢弃", expires)
                continue
            self._publish_proposal(slot, data.kind, data.waypoint, data.rationale, expires)

    def _publish_proposal(self, slot: dict, kind: int, waypoint: str,
                          rationale: str, expires: float) -> None:
        msg = HarnessProposal()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = slot["task_id"]
        msg.revision = slot["revision"]
        msg.request_id = slot["request_id"]
        msg.kind = kind
        msg.waypoint = waypoint
        msg.rationale = rationale[:200]
        msg.expires_epoch_sec = expires
        pub = self._proposal_pub
        if pub is None:
            return
        if self.publish(msg, publisher=pub):
            self._proposals_sent += 1

    # -- dialogue command execution -------------------------------------------

    def _execute_commands(self, out) -> None:
        for key, slot_text in out.say:
            template_slots = {"dest": slot_text, "reason": slot_text}
            self._say(render(key, self._persona, **template_slots))
        for task_id, kind, text in out.events:
            event = HarnessEvent()
            event.header.stamp = self.get_clock().now().to_msg()
            event.task_id = task_id
            event.kind = _EVENT_KIND.get(kind, HarnessEvent.KIND_PROGRESS)
            event.text = text
            event.reason = text
            if self._event_pub is not None:
                self.publish(event, publisher=self._event_pub)
        for text in out.submit_task:
            goal = ExecuteTask.Goal()
            goal.request_id = f"voice-{uuid.uuid4()}"
            goal.text = text
            goal.has_pose = False
            if self._task_client is not None and self._task_client.server_is_ready():
                self._task_client.send_goal_async(goal)
            else:
                self._say(render("failed", self._persona, reason="任务服务不在"))
        for task_id, op in out.control:
            if self._control_client is None or not self._control_client.service_is_ready():
                continue
            request = TaskControl.Request()
            request.task_id = task_id
            request.operation = {
                "pause": TaskControl.Request.PAUSE,
                "resume": TaskControl.Request.RESUME,
                "cancel": TaskControl.Request.CANCEL,
            }.get(op, TaskControl.Request.PAUSE)
            self._control_client.call_async(request)

    def _say(self, text: str) -> None:
        client = self._say_client
        if not text or client is None or not client.service_is_ready():
            return
        request = Say.Request()
        request.text = text
        client.call_async(request)

    # ------------------------------------------------------------------ health
    def _stats(self) -> None:
        now = time.monotonic()
        dt = max(1e-3, now - self._stats_t0)
        rate = self._proposals_sent * 60.0 / dt
        with self._lock:
            inflight = 1 if self._proposal_slot is not None else 0
        # 在飞本身是正常态（cap-1 队列，worker 200 ms 一轮）；连续两个周期还不消费才算异常。
        self._inflight_ticks = self._inflight_ticks + 1 if inflight else 0
        self.report(
            "proposals",
            DiagnosticStatus.WARN if self._inflight_ticks >= 2 else DiagnosticStatus.OK,
            f"{rate:.1f} 提案/min, 在飞={inflight}",
            {"proposals_per_min": f"{rate:.2f}", "inflight": inflight,
             "proposals_total": self._proposals_sent},
        )
        if self._agent_state == "up":
            level, message = DiagnosticStatus.OK, "up"
        elif self._voice_path == "dialogue":
            level, message = DiagnosticStatus.OK, "down (voice_path=dialogue)"
        else:
            level, message = DiagnosticStatus.WARN, "degraded"
        self.report(
            "agent", level, message,
            {"state": self._agent_state or "down", "voice_path": self._voice_path,
             "detail": self._agent_detail},
        )
        self._stats_t0 = now
        self._proposals_sent = 0

    # ------------------------------------------------------------------ helpers
    def _subscribe(self, msg_type, topic: str, callback) -> object:
        sub = self.create_subscription(msg_type, topic, callback, 10)
        self._subs.append(sub)
        return sub

    def _release_ros(self) -> None:
        """cleanup/shutdown 用：把 configure 建的实体撤掉，允许重新配置。

        发布器**不销毁**：`LifecyclePublisher` 由基类登记与启停，销毁它需要动 rclpy 的
        内部列表；再 configure 时重建一个同话题发布器即可（参考 i2c_imu 的写法）。
        """
        self._configured = False
        for sub in self._subs:
            try:
                self.destroy_subscription(sub)
            except Exception:  # noqa: BLE001 - 已经没了就别再抛
                pass
        self._subs.clear()
        self._intent_sub = None
        self._request_sub = None
        for client in (self._say_client, self._control_client):
            if client is not None:
                try:
                    self.destroy_client(client)
                except Exception:  # noqa: BLE001
                    pass
        self._say_client = None
        self._control_client = None
        if self._task_client is not None:
            try:
                self._task_client.destroy()
            except Exception:  # noqa: BLE001
                pass
            self._task_client = None

    def _stop_worker(self) -> None:
        self._stop.set()
        worker, self._worker = self._worker, None
        if worker is not None and worker.is_alive():
            worker.join(timeout=1.0)

    def destroy_node(self) -> bool:
        self._stop_agent()
        self._stop_worker()
        self._release_ros()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HarnessNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
