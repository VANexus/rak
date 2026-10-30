"""harness_node: arbitration seam + voice dialogue loop (ADR-0010).

只读上下文:/brain/decision(keep-last-N)、/speech/intent、/task/status、
/base/telemetry、/speech/wake。写出:/harness/proposal、/harness/event、
/task/execute(结构化目标)、/task/control、/speech/say(回话)。
永不发布速度/关节命令。
"""
from __future__ import annotations

import json
import threading
import time
import uuid
from collections import deque
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
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


class HarnessNode(Node):
    def __init__(self) -> None:
        super().__init__("harness")
        self.declare_parameter("attention_sec", 12.0)
        # 唤醒应答的两道闸(只压应答,不压注意力窗口/方位箭头):
        # 板阈值 500 贴地,环境语音会擦线误触发;分数闸按实测缝调(误触发
        # 501-546、真唤醒 589+),去抖压连环触发。
        self.declare_parameter("ack_min_score", 650.0)
        self.declare_parameter("ack_cooldown_sec", 4.0)
        self.declare_parameter("persona_file", "")
        self.declare_parameter("proposal_ttl_sec", 20.0)
        self.declare_parameter("cloud_model", "deepseek-chat")
        self.declare_parameter("cloud_calls_per_task", 3)
        self.declare_parameter("voice_path", "auto")
        self.declare_parameter("agent_sdk_path", "/home/xrak/rak/tools/harness-sdk")
        self.declare_parameter("agent_allow_motion_tools", False)

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        persona_file = str(g("persona_file")) or self._default_persona()
        self._persona = load_persona(persona_file)
        self._engine = DialogueEngine(
            attention_sec=float(g("attention_sec")),
            ack_min_score=float(g("ack_min_score")),
            ack_cooldown_sec=float(g("ack_cooldown_sec")),
        )
        self._ttl = float(g("proposal_ttl_sec"))
        self._cloud = HarnessCloud(
            model=str(g("cloud_model")),
            max_calls_per_task=int(g("cloud_calls_per_task")),
        )
        # P1 agentic loop (Claude Agent SDK in a subprocess, see agent_bridge.py).
        # voice_path: auto(agent if SDK present) | agent | dialogue(ADR-0010 only)
        self._voice_path = str(g("voice_path"))
        self._agent_active = False
        self._agent = None

        self._lock = threading.RLock()
        self._snapshot = VehicleSnapshot()
        self._decisions: deque[str] = deque(maxlen=20)
        self._last_destination = ""
        self._proposal_slot: dict | None = None  # executor request, cap-1 queue

        self._proposal_pub = self.create_publisher(HarnessProposal, "/harness/proposal", 10)
        self._event_pub = self.create_publisher(HarnessEvent, "/harness/event", 10)
        self._request_sub = self.create_subscription(
            HarnessRequest, "/harness/request", self._on_request, 10)
        self.create_subscription(WakeEvent, "/speech/wake", self._on_wake, 10)
        # Voice routing: the agent owns /speech/transcript; the ADR-0010 dialogue
        # engine keeps /speech/intent until the agent proves itself (status up).
        self._intent_sub = None
        self.create_subscription(TaskStatus, "/task/status", self._on_status, 10)
        self.create_subscription(BaseTelemetry, "/base/telemetry", self._on_telemetry, 10)
        self.create_subscription(Decision, "/brain/decision", self._on_decision, 10)

        self._task_client = ActionClient(self, ExecuteTask, "/task/execute")
        self._control_client = self.create_client(TaskControl, "/task/control")
        self._say_client = self.create_client(Say, "/speech/say")

        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._run, name="harness-cloud", daemon=True)
        self._worker.start()
        self._start_agent(g)
        self.get_logger().info(
            f"harness up: persona={self._persona.name} cloud={'live' if self._cloud.available() else 'stub'}")

    # -- P1 agentic loop ------------------------------------------------------

    def _start_agent(self, g) -> None:
        if self._voice_path == "dialogue":
            self.get_logger().info("voice_path=dialogue: agent loop disabled")
            self._subscribe_intent()
            return
        sdk_path = str(g("agent_sdk_path"))
        if self._voice_path == "auto" and not __import__("os").path.isdir(sdk_path):
            self.get_logger().info(
                f"voice_path=auto: no SDK overlay at {sdk_path}, staying on dialogue")
            self._subscribe_intent()
            return
        from .agent_bridge import AgentBridge

        self._trace_pub = self.create_publisher(String, "/harness/trace", 10)
        self._agent = AgentBridge(
            self,
            sdk_path=sdk_path,
            model=str(g("cloud_model")),
            allow_motion_tools=bool(g("agent_allow_motion_tools")),
            trace_pub=self._trace_pub,
            say=self._say,
            on_state=self._on_agent_state,
            context_provider=self._agent_context,
        )

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
        self._agent_active = state == "up"
        if state == "up":
            self.get_logger().info(f"agent loop voice path active: {detail}")
        else:
            self.get_logger().warn(
                f"agent loop {state}: {detail or 'no detail'}; falling back to dialogue voice")
            self._subscribe_intent()

    def _subscribe_intent(self) -> None:
        if self._intent_sub is None:
            self._intent_sub = self.create_subscription(
                String, "/speech/intent", self._on_intent, 10)

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
            out = self._engine.on_wake(now=time.time(), score=float(msg.score))
        self.get_logger().info(
            f"attentive: '{msg.keyword}' angle={msg.angle:.0f}° beam={msg.beam}")
        # 引擎在唤醒时会带应答指令(ack_wake,带分数闸与去抖);不执行的话,
        # 听到名字却不吭声。
        self._execute_commands(out)

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

    def _run(self) -> None:
        while not self._stop.is_set():
            time.sleep(0.2)
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
                if self._cloud.available():
                    data = self._cloud.propose(
                        text=slot["text"], destination=destination,
                        allowed_waypoints=slot["allowed"], context_text=context,
                        task_id=slot["task_id"], expires_epoch_sec=expires)
                else:
                    data = self._cloud.propose_local(
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
        self._proposal_pub.publish(msg)

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
            self._event_pub.publish(event)
        for text in out.submit_task:
            goal = ExecuteTask.Goal()
            goal.request_id = f"voice-{uuid.uuid4()}"
            goal.text = text
            goal.has_pose = False
            if self._task_client.server_is_ready():
                self._task_client.send_goal_async(goal)
            else:
                self._say(render("failed", self._persona, reason="任务服务不在"))
        for task_id, op in out.control:
            if not self._control_client.service_is_ready():
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
        if not text or not self._say_client.service_is_ready():
            return
        request = Say.Request()
        request.text = text
        self._say_client.call_async(request)

    def destroy_node(self) -> bool:
        self._stop.set()
        if self._agent is not None:
            self._agent.destroy()
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
