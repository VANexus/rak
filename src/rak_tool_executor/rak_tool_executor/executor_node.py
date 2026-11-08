"""The tool executor: the single writer for tool actions.

Subscribes to /tool_proposals, validates against the registry, applies the
deterministic permission policy, dispatches to handlers, and publishes
exactly one terminal ToolResult per call_id (plus a JSONL audit line).

Handled tools: say, query.joint_states, task.goto/pause/resume/cancel,
arm.home. Everything else in the registry answers rejected/unimplemented
with the registry's hint. Motion-class tools additionally require
allow_motion:=true on this node AND operator_confirmed on the proposal.

生命周期（ADR-0020 §5）:
  `on_configure` 声明参数、建发布器/订阅/客户端、打开审计目录；
  `on_activate` 才起 worker 线程（提案处理可能阻塞数分钟，是最容易被中断的资源）；
  `on_deactivate` 停线程，`on_cleanup` 再释放全部实体。
  未激活时到达的提案**直接丢弃**：一条几分钟前的旧提案不该在重新激活后被执行。
"""

from __future__ import annotations

import json
import queue
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.task import Future

from control_msgs.action import FollowJointTrajectory
from rak_interfaces.msg import ToolProposal, ToolResult
from rak_interfaces.srv import Say, TaskControl
from rak_lifecycle.managed_node import ManagedNode
from sensor_msgs.msg import JointState

from . import tool_registry as reg
from .audit import Audit, call_record
from .permissions import evaluate

# docs/arm_moveit_contract.md section 1 -- FROZEN joint names.
ARM_JOINTS = ("j1_base", "j2_shoulder", "j3_elbow", "j4_forearm", "j5_wrist_pitch", "j6_wrist_roll")
HOME_POSE_DEG = (0.0, 225.0, 270.0, 0.0, 0.0, 0.0)


def _deg2rad(values):
    import math

    return [math.radians(v) for v in values]


class ToolExecutor(ManagedNode):
    def __init__(self) -> None:
        # 只做字段初始化：proposal 订阅、pub/client、worker 线程都在 configure/activate。
        super().__init__("tool_executor")
        self._proposal_topic = "/tool_proposals"
        self._result_topic = "/tool_results"
        self._allow_motion = False
        self._say_timeout = 10.0
        self._joint_states_timeout = 2.0
        self._task_timeout = 180.0
        self._arm_home_pose_deg = list(HOME_POSE_DEG)
        self._arm_home_duration = 12.0
        self._arm_server_wait = 3.0
        self._audit = None
        self._call_id_scratch = ""

        self._result_pub = None
        self._subs: list = []
        self._say_client = None
        self._task_client = None
        self._arm_client = None
        self._wait_executor = None
        self._proposal_queue: queue.Queue = queue.Queue()
        self._stop = threading.Event()
        self._worker: threading.Thread | None = None
        self._joint_states = None

        # 诊断统计（_stats 每 5 s 换算成 calls/min）
        self._handled = 0
        self._dropped = 0
        self._stats_t0 = time.monotonic()

    # ---------------------------------------------------------------- configure
    def on_configure(self, state):
        ret = super().on_configure(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret
        try:
            self._proposal_topic = str(self.declare("proposal_topic", "/tool_proposals"))
            self._result_topic = str(self.declare("result_topic", "/tool_results"))
            self._allow_motion = bool(self.declare("allow_motion", False))
            self._say_timeout = float(self.declare("say_timeout_s", 10.0))
            self._joint_states_timeout = float(self.declare("joint_states_timeout_s", 2.0))
            self._task_timeout = float(self.declare("task_goto_timeout_s", 180.0))
            self._arm_home_pose_deg = [
                float(v) for v in self.declare("arm_home_pose_deg", list(HOME_POSE_DEG))]
            self._arm_home_duration = float(self.declare("arm_home_duration_s", 12.0))
            self._arm_server_wait = float(self.declare("arm_server_wait_s", 3.0))
            self._audit = Audit(str(self.declare("audit_dir", "data/episodes")))

            # 未激活不发布：LifecyclePublisher 在 on_activate 之前丢弃 publish()
            self._result_pub = self.lifecycle_publisher(
                ToolResult, self._result_topic, 10)
            self._subs.append(self.create_subscription(
                ToolProposal, self._proposal_topic, self._on_proposal, 10))
            self._subs.append(self.create_subscription(
                JointState, "/joint_states", self._on_joint_states, 10))

            self._say_client = self.create_client(Say, "/speech/say")
            self._task_client = self.create_client(TaskControl, "/task/control")
            self._arm_client = ActionClient(
                self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
            # Proposals are handled on a WORKER thread (a proposal handler may wait
            # on service/action results for minutes); re-entrant spin_once inside
            # a subscription callback is illegal in rclpy, so the worker owns a
            # private executor for waiting and the main thread keeps spinning.
            self._wait_executor = SingleThreadedExecutor()
            self._wait_executor.add_node(self)
        except Exception as exc:  # noqa: BLE001 - 配置失败必须变成可读的 FAILURE
            self._release()
            return self.fail(f"配置 tool_executor 失败: {exc}")

        self._joint_states = None
        self._call_id_scratch = ""
        self.report("tools", DiagnosticStatus.OK,
                    f"allow_motion={self._allow_motion} tools={len(reg.TOOLS)}",
                    {"allow_motion": str(self._allow_motion),
                     "tools": len(reg.TOOLS),
                     "implemented": sum(1 for t in reg.TOOLS if t.status == "implemented")})
        self.get_logger().info(
            f"tool_executor configured allow_motion={self._allow_motion} tools={len(reg.TOOLS)} "
            f"(implemented {sum(1 for t in reg.TOOLS if t.status == 'implemented')})"
        )
        return TransitionCallbackReturn.SUCCESS

    # ----------------------------------------------------------------- activate
    def on_activate(self, state):
        ret = super().on_activate(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret
        try:
            # 每次激活一个新 event：上一轮还在收尾的 handler 只认自己那个，
            # 不会因为这次 clear 又活过来。
            self._stop = threading.Event()
            self._handled = 0
            self._dropped = 0
            self._stats_t0 = time.monotonic()
            self._worker = threading.Thread(
                target=self._work_loop, args=(self._stop,), name="tool-worker", daemon=True)
            self._worker.start()
        except Exception as exc:  # noqa: BLE001
            self._stop_worker()
            return self.fail(f"激活 tool_executor 失败: {exc}")
        self.add_timer(5.0, self._stats)
        return TransitionCallbackReturn.SUCCESS

    # --------------------------------------------------------------- deactivate
    def on_deactivate(self, state):
        self._stop_worker()
        return super().on_deactivate(state)

    # ------------------------------------------------------------------ cleanup
    def on_cleanup(self, state):
        self._stop_worker()
        self._release()
        self._joint_states = None
        self._call_id_scratch = ""
        return super().on_cleanup(state)

    def on_shutdown(self, state):
        self._stop_worker()
        ret = super().on_shutdown(state)
        self._release()
        return ret

    # ------------------------------------------------------------------ callbacks

    def _on_joint_states(self, msg: JointState) -> None:
        self._joint_states = msg
        self.mark_seen("/joint_states")

    def _on_proposal(self, msg: ToolProposal) -> None:
        # Fast path only: heavy handling happens on the worker thread.
        if not self.active:
            # 未激活不接单：旧提案在重新激活后执行等于过期运动命令。
            self._dropped += 1
            return
        self._proposal_queue.put(msg)

    def _work_loop(self, stop: threading.Event) -> None:
        while not stop.is_set():
            try:
                msg = self._proposal_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            result = self._handle(msg)
            pub = self._result_pub
            if pub is not None:
                self.publish(result, publisher=pub)
            self._handled += 1
            if self._audit is not None:
                self._audit.event("tool_result", call_id=result.call_id, tool=result.tool,
                                  accepted=result.accepted, status=result.status,
                                  reason=result.reason,
                                  detail=json.loads(result.detail_json or "{}"))
            self.get_logger().info(
                f"{result.call_id} {result.tool} -> {result.status}"
                + (f" ({result.reason})" if result.reason else "")
            )

    # ------------------------------------------------------------------ dispatch

    def _handle(self, msg: ToolProposal) -> ToolResult:
        self._call_id_scratch = msg.call_id

        def done(status: str, accepted: bool, reason: str, detail: dict) -> ToolResult:
            return ToolResult(call_id=msg.call_id, tool=msg.tool, accepted=accepted,
                              status=status, reason=reason,
                              detail_json=json.dumps(detail, ensure_ascii=False))

        if not self.active:
            return done("error", False, "executor not active (lifecycle)", {})

        spec = reg.get_tool(msg.tool)
        if spec is None:
            return done("rejected", False, f"unknown tool '{msg.tool}'",
                        {"menu": reg.tool_names()})

        try:
            args = json.loads(msg.args_json) if msg.args_json else {}
        except json.JSONDecodeError as exc:
            return done("rejected", False, f"args_json not valid JSON: {exc}", {})

        errors = reg.validate_args(spec, args)
        if errors:
            return done("rejected", False, "args failed schema: " + "; ".join(errors),
                        {"schema": spec.args_schema})

        perm = evaluate(spec.safety, msg.operator_confirmed)
        if perm.mode == "deny":
            return done("rejected", False, perm.reason, {})
        if perm.mode == "rejected":
            return done("rejected", False, perm.reason, {"safety": spec.safety})

        if self._audit is not None:
            self._audit.event("tool_call", call_id=msg.call_id,
                              **call_record(msg.tool, msg.source, msg.requested_by, args,
                                            spec.safety, msg.operator_confirmed))

        if spec.status != "implemented":
            return done("rejected", False, spec.hint or f"tool '{msg.tool}' not available yet",
                        {"status": spec.status})

        handler = {
            "say": self._h_say,
            "query.joint_states": self._h_joint_states,
            "task.goto": self._h_task_goto,
            "task.pause": lambda a: self._h_task_control(1),
            "task.resume": lambda a: self._h_task_control(2),
            "task.cancel": lambda a: self._h_task_control(3),
            "arm.home": self._h_arm_home,
        }.get(msg.tool)
        if handler is None:
            return done("error", False, "no handler wired for implemented tool", {})
        try:
            return handler(args)
        except Exception as exc:  # a broken handler must still answer with a result
            return done("error", False, f"handler error: {exc}", {})

    # ------------------------------------------------------------------ handlers

    def _h_say(self, args: dict) -> ToolResult:
        return self._finish("say", self._do_say(args))

    def _finish(self, tool: str, outcome: tuple[bool, str, dict]) -> ToolResult:
        ok, reason, detail = outcome
        return ToolResult(call_id=self._current_call_id, tool=tool, accepted=ok,
                          status="ok" if ok else "error", reason=reason,
                          detail_json=json.dumps(detail, ensure_ascii=False))

    def _do_say(self, args: dict) -> tuple[bool, str, dict]:
        text = str(args.get("text", "")).strip()
        if not text:
            return False, "empty text", {}
        if not self._say_client.wait_for_service(timeout_sec=self._say_timeout):
            return False, "tts /speech/say service unavailable", {"text": text}
        fut = self._say_client.call_async(Say.Request(text=text))
        if not self._wait(fut, self._say_timeout):
            return False, "tts service timed out", {"text": text}
        resp = fut.result()
        if not resp.accepted:
            return False, f"tts refused: {resp.message}", {"text": text}
        return True, "queued for synthesis", {"text": text}

    def _h_joint_states(self, _args: dict) -> ToolResult:
        msg = self._joint_states
        if msg is None and not self._wait_joint_states(self._joint_states_timeout):
            return self._finish("query.joint_states", (False, "no /joint_states received", {}))
        msg = self._joint_states
        return self._finish("query.joint_states", (
            True, "ok", {"names": list(msg.name), "positions": [round(p, 5) for p in msg.position]}))

    def _wait_joint_states(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and self._joint_states is None:
            self._spin_once(0.05)
        return self._joint_states is not None

    def _h_task_goto(self, args: dict) -> ToolResult:
        alias = str(args.get("waypoint", "")).strip()
        if not alias:
            return self._finish("task.goto", (False, "empty waypoint alias", {}))
        from rak_interfaces.action import ExecuteTask

        client = ActionClient(self, ExecuteTask, "/execute_task")
        if not client.wait_for_server(timeout_sec=self._arm_server_wait):
            return self._finish("task.goto", (False, "/execute_task action unavailable", {}))
        goal = ExecuteTask.Goal(request_id=self._current_call_id, text=alias, has_pose=False)
        goal_fut = client.send_goal_async(goal)
        if not self._wait(goal_fut, 5.0):
            return self._finish("task.goto", (False, "goal not accepted in time", {}))
        handle = goal_fut.result()
        if handle is None or not handle.accepted:
            return self._finish("task.goto", (False, "task executor rejected goal", {"waypoint": alias}))
        res_fut = handle.get_result_async()
        if not self._wait(res_fut, self._task_timeout):
            return self._finish("task.goto", (False, "task still running at timeout", {"waypoint": alias}))
        res = res_fut.result().result
        return self._finish("task.goto", (
            bool(res.succeeded), res.message,
            {"waypoint": alias, "terminal_state": int(res.terminal_state)}))

    def _h_task_control(self, operation: int) -> tuple[bool, str, dict]:
        if not self._task_client.wait_for_server(timeout_sec=self._arm_server_wait):
            return False, "/task/control service unavailable", {"operation": operation}
        fut = self._task_client.call_async(TaskControl.Request(task_id="", operation=operation))
        if not self._wait(fut, 10.0):
            return False, "task control timed out", {"operation": operation}
        resp = fut.result()
        return bool(resp.accepted), resp.message, {"operation": operation}

    def _h_arm_home(self, args: dict) -> ToolResult:
        if not self._allow_motion:
            return self._finish("arm.home", (
                False, "allow_motion:=false on tool_executor (operator gate closed)", {}))
        if self._joint_states is None or not set(ARM_JOINTS) <= set(self._joint_states.name):
            return self._finish("arm.home", (
                False, "arm feedback missing in /joint_states (arm_controller not active?)", {}))
        if not self._arm_client.wait_for_server(timeout_sec=self._arm_server_wait):
            return self._finish("arm.home", (False, "arm_controller action server unavailable", {}))

        speed = float(args.get("speed_scale", 0.25))
        duration = min(120.0, self._arm_home_duration * (0.25 / max(speed, 0.05)))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        from trajectory_msgs.msg import JointTrajectoryPoint

        point = JointTrajectoryPoint()
        point.positions = _deg2rad(self._arm_home_pose_deg)
        point.time_from_start = Duration(seconds=duration).to_msg()
        goal.trajectory.points = [point]

        goal_fut = self._arm_client.send_goal_async(goal)
        if not self._wait(goal_fut, 5.0):
            return self._finish("arm.home", (False, "goal not accepted in time", {}))
        handle = goal_fut.result()
        if handle is None or not handle.accepted:
            return self._finish("arm.home", (False, "arm_controller rejected goal", {}))
        res_fut = handle.get_result_async()
        if not self._wait(res_fut, duration + 30.0):
            return self._finish("arm.home", (False, "arm trajectory still running at timeout",
                                                 {"duration_s": duration}))
        res = res_fut.result().result
        return self._finish("arm.home", (
            bool(res.error_code == FollowJointTrajectory.Result.SUCCESSFUL),
            f"arm_controller error_code={res.error_code} string='{res.error_string}'",
            {"duration_s": duration, "commanded_deg": list(self._arm_home_pose_deg),
             "speed_scale": speed}))

    # ------------------------------------------------------------------ helpers

    @property
    def _current_call_id(self) -> str:  # set in _handle via closure-free path
        return self._call_id_scratch

    def _spin_once(self, timeout: float) -> None:
        executor = self._wait_executor
        if executor is not None:
            executor.spin_once(timeout_sec=timeout)

    def _wait(self, fut: Future, timeout: float) -> bool:
        """Pump the worker's private executor while waiting (never the caller's)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not fut.done():
            self._spin_once(0.05)
        return fut.done()

    # ------------------------------------------------------------------ health
    def _stats(self) -> None:
        now = time.monotonic()
        dt = max(1e-3, now - self._stats_t0)
        rate = self._handled * 60.0 / dt
        level = DiagnosticStatus.WARN if self._dropped else DiagnosticStatus.OK
        self.report(
            "tools", level,
            f"{rate:.1f} 调用/min, 队列={self._proposal_queue.qsize()}",
            {"calls_per_min": f"{rate:.2f}", "handled_total": self._handled,
             "dropped_inactive": self._dropped,
             "queue_depth": self._proposal_queue.qsize(),
             "allow_motion": str(self._allow_motion)},
        )
        age = self.age_ms("/joint_states")
        arm_level = DiagnosticStatus.OK if age is not None and age < 2000.0 else DiagnosticStatus.WARN
        self.report(
            "joint_states", arm_level,
            "never" if age is None else f"{age:.0f}ms old",
            {"age_ms": "never" if age is None else f"{age:.0f}"},
        )
        self._stats_t0 = now
        self._handled = 0
        self._dropped = 0

    # ----------------------------------------------------------------- lifecycle
    def _stop_worker(self) -> None:
        self._stop.set()
        worker, self._worker = self._worker, None
        if worker is not None and worker.is_alive():
            # 一个 handler 可能正在等 action 结果；只等一小会儿，剩余由 daemon 线程收尾
            # （它只看自己那个 event，重新激活不会让它复活）。
            worker.join(timeout=1.0)

    def _release(self) -> None:
        """cleanup/shutdown 用：撤掉订阅/客户端/私有执行器，允许重新配置。

        结果发布器**不销毁**：`LifecyclePublisher` 由基类登记与启停；再 configure 时
        重建一个同话题发布器即可（参考 i2c_imu 的写法）。
        """
        for sub in self._subs:
            try:
                self.destroy_subscription(sub)
            except Exception:  # noqa: BLE001 - 已经没了就别再抛
                pass
        self._subs.clear()
        for client in (self._say_client, self._task_client):
            if client is not None:
                try:
                    self.destroy_client(client)
                except Exception:  # noqa: BLE001
                    pass
        self._say_client = None
        self._task_client = None
        if self._arm_client is not None:
            try:
                self._arm_client.destroy()
            except Exception:  # noqa: BLE001
                pass
            self._arm_client = None
        if self._wait_executor is not None:
            try:
                self._wait_executor.remove_node(self)
            except Exception:  # noqa: BLE001
                pass
            self._wait_executor = None
        self._audit = None

    def destroy_node(self) -> bool:
        self._stop_worker()
        self._release()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ToolExecutor()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
