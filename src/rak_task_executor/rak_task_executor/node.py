"""Task action server with one authoritative Nav2 goal lifecycle."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
import threading
import time
import uuid

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rak_intent.task_intent import (
    ControlCommand,
    PoseProposal,
    RaiWaypointPlanner,
    parse_control_command,
    parse_navigation_intent,
    resolve_waypoint,
    validate_pose_proposal,
)
from rak_interfaces.action import ExecuteTask
from rak_interfaces.msg import BaseTelemetry, TaskStatus
from rak_interfaces.srv import TaskControl
from std_srvs.srv import SetBool


@dataclass
class ActiveTask:
    task_id: str
    request_id: str
    revision: int = 1
    state: int = TaskStatus.IDLE
    target: PoseStamped | None = None
    nav_goal_handle: object | None = None
    command: str | None = None
    control_started: float | None = None
    cancel_timeout_reported: bool = False
    timed_out: bool = False
    control_event: threading.Event = field(default_factory=threading.Event)


class TaskExecutor(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__("task_executor", parameter_overrides=parameter_overrides)
        self.declare_parameter("navigate_to_pose_action", "/navigate_to_pose")
        self.declare_parameter("task_action", "/task/execute")
        self.declare_parameter("task_control_service", "/task/control")
        self.declare_parameter("task_status_topic", "/task/status")
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("slow_planner", "local")
        self.declare_parameter("llm_config_path", "")
        self.declare_parameter("llm_vendor", "")
        self.declare_parameter("nav_server_timeout_sec", 5.0)
        self.declare_parameter("cancel_timeout_sec", 5.0)
        self.declare_parameter("task_timeout_sec", 1800.0)
        self.declare_parameter("slow_planner_timeout_sec", 35.0)
        self.declare_parameter("base_disarm_service", "/base_uart_driver/arm")
        self.declare_parameter("base_telemetry_topic", "/base/telemetry")
        self.declare_parameter("stop_feedback_samples", 3)
        self.declare_parameter("stop_vxy_threshold_m_s", 0.02)
        self.declare_parameter("stop_wz_threshold_rad_s", 0.03)

        self._lock = threading.RLock()
        self._active: ActiveTask | None = None
        self._reserved = False
        self._reserved_request_id: str | None = None
        self._pending_cancel_ids: set[str] = set()
        self._seen_request_ids: set[str] = set()
        self._seen_order: deque[str] = deque()
        self._waypoints = self._load_waypoints()
        planner_name = str(self.get_parameter("slow_planner").value)
        self._slow_planner = None
        if planner_name == "rai":
            config_path = str(self.get_parameter("llm_config_path").value)
            if not config_path:
                raise RuntimeError("slow_planner=rai requires llm_config_path")
            self._slow_planner = RaiWaypointPlanner(
                config_path=config_path,
                vendor=str(self.get_parameter("llm_vendor").value) or None,
            )
        elif planner_name != "local":
            raise RuntimeError("slow_planner must be 'local' or 'rai'")
        self._stop_samples = 0
        self._last_telemetry_time = 0.0
        self._telemetry_event = threading.Event()
        self._group = ReentrantCallbackGroup()
        self._nav_client = ActionClient(
            self, NavigateToPose, str(self.get_parameter("navigate_to_pose_action").value),
            callback_group=self._group,
        )
        self._disarm_client = self.create_client(
            SetBool, str(self.get_parameter("base_disarm_service").value),
            callback_group=self._group,
        )
        self._status_pub = self.create_publisher(
            TaskStatus, str(self.get_parameter("task_status_topic").value), 10
        )
        self._base_telemetry_sub = self.create_subscription(
            BaseTelemetry,
            str(self.get_parameter("base_telemetry_topic").value),
            self._on_base_telemetry,
            10,
        )
        self._control_srv = self.create_service(
            TaskControl,
            str(self.get_parameter("task_control_service").value),
            self._on_control,
            callback_group=self._group,
        )
        self._action_server = ActionServer(
            self,
            ExecuteTask,
            str(self.get_parameter("task_action").value),
            execute_callback=self._execute,
            goal_callback=self._goal_request,
            cancel_callback=self._cancel_request,
            callback_group=self._group,
        )
        self._publish_idle()
        self.get_logger().info(
            "task executor ready; one task at a time, task restart does not resume old goals"
        )

    def _load_waypoints(self):
        path = str(self.get_parameter("waypoints_file").value)
        if not path:
            from ament_index_python.packages import get_package_share_directory
            import os

            path = os.path.join(
                get_package_share_directory("rak_task_executor"), "config", "waypoints.yaml"
            )
        try:
            with open(path, encoding="utf-8") as source:
                data = yaml.safe_load(source) or {}
        except (OSError, yaml.YAMLError) as exc:
            raise RuntimeError(f"cannot load waypoint file {path}: {exc}") from exc
        if data.get("schema_version") != 1 or data.get("frame_id") != "map":
            raise RuntimeError("waypoint file must use schema_version=1 and frame_id=map")
        waypoints = {}
        for name, item in (data.get("waypoints") or {}).items():
            proposal = PoseProposal(
                frame_id="map",
                x=float(item["x"]),
                y=float(item["y"]),
                z=float(item.get("z", 0.0)),
                qx=float(item.get("qx", 0.0)),
                qy=float(item.get("qy", 0.0)),
                qz=float(item.get("qz", 0.0)),
                qw=float(item.get("qw", 1.0)),
            )
            validate_pose_proposal(proposal)
            waypoints[str(name)] = proposal
        return waypoints

    def _goal_request(self, request):
        request_id = request.request_id.strip()
        text_set = bool(request.text.strip())
        pose_set = bool(request.has_pose)
        if not request_id or len(request_id) > 128:
            return GoalResponse.REJECT
        if text_set == pose_set:
            return GoalResponse.REJECT
        if text_set:
            if parse_control_command(request.text) is not None:
                return GoalResponse.REJECT
            intent = parse_navigation_intent(request.text)
            if intent is None or (
                intent.destination not in self._waypoints and self._slow_planner is None
            ):
                return GoalResponse.REJECT
        else:
            try:
                self._pose_from_message(request.target_pose)
            except ValueError:
                return GoalResponse.REJECT
        with self._lock:
            if self._reserved or self._active is not None or request_id in self._seen_request_ids:
                return GoalResponse.REJECT
            self._reserved = True
            self._reserved_request_id = request_id
            self._remember_request_id(request_id)
        return GoalResponse.ACCEPT

    def _remember_request_id(self, request_id):
        self._seen_request_ids.add(request_id)
        self._seen_order.append(request_id)
        while len(self._seen_order) > 256:
            self._seen_request_ids.discard(self._seen_order.popleft())

    def _cancel_request(self, goal_handle):
        with self._lock:
            task = self._active
            if task is None:
                request_id = goal_handle.request.request_id.strip()
                if self._reserved and request_id == self._reserved_request_id:
                    self._pending_cancel_ids.add(request_id)
                    return CancelResponse.ACCEPT
                return CancelResponse.REJECT
            task.command = "cancel"
            task.revision += 1
            task.control_started = time.monotonic()
            task.cancel_timeout_reported = False
            task.control_event.set()
            nav_handle = task.nav_goal_handle
        self._set_state(task, TaskStatus.CANCELING, "action client requested cancellation")
        if nav_handle is not None:
            nav_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        request = goal_handle.request
        task = ActiveTask(
            task_id=str(uuid.uuid4()),
            request_id=request.request_id.strip(),
        )
        with self._lock:
            self._reserved = False
            self._reserved_request_id = None
            if request.request_id.strip() in self._pending_cancel_ids:
                task.command = "cancel"
                task.revision += 1
                task.control_started = time.monotonic()
                task.cancel_timeout_reported = False
                self._pending_cancel_ids.discard(request.request_id.strip())
            self._active = task
        try:
            self._set_state(task, TaskStatus.ACCEPTED, "task accepted")
            if task.command == "cancel" or goal_handle.is_cancel_requested:
                self._set_state(task, TaskStatus.CANCELED, "canceled before task planning")
                result = ExecuteTask.Result()
                result.succeeded = False
                result.terminal_state = TaskStatus.CANCELED
                result.message = "canceled before task planning"
                goal_handle.canceled()
                return result
            self._set_state(task, TaskStatus.PLANNING, "resolving a safe map-frame goal")
            task.target = (
                self._pose_from_message(request.target_pose)
                if request.has_pose
                else self._waypoint_from_text(request.text, task)
            )
            if task.command == "cancel" or goal_handle.is_cancel_requested:
                self._set_state(task, TaskStatus.CANCELED, "canceled during goal resolution")
                result = ExecuteTask.Result()
                result.succeeded = False
                result.terminal_state = TaskStatus.CANCELED
                result.message = "canceled during goal resolution"
                goal_handle.canceled()
                return result
            terminal, message = self._navigate_until_terminal(goal_handle, task)
            result = ExecuteTask.Result()
            result.succeeded = terminal == TaskStatus.SUCCEEDED
            result.terminal_state = terminal
            result.message = message
            if terminal == TaskStatus.SUCCEEDED:
                goal_handle.succeed()
            elif terminal == TaskStatus.CANCELED:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
            else:
                goal_handle.abort()
            return result
        except Exception as exc:
            self.get_logger().error(f"task {task.task_id} failed: {exc}")
            self._set_state(task, TaskStatus.FAILED, str(exc))
            result = ExecuteTask.Result()
            result.succeeded = False
            result.terminal_state = TaskStatus.FAILED
            result.message = str(exc)
            goal_handle.abort()
            return result
        finally:
            with self._lock:
                self._active = None
                self._reserved = False
                self._reserved_request_id = None

    def _navigate_until_terminal(self, action_handle, task):
        timeout = float(self.get_parameter("task_timeout_sec").value)
        started = time.monotonic()
        while True:
            if action_handle.is_cancel_requested:
                return TaskStatus.CANCELED, "task action canceled"
            if not self._nav_client.wait_for_server(
                timeout_sec=float(self.get_parameter("nav_server_timeout_sec").value)
            ):
                return TaskStatus.FAILED, "Nav2 NavigateToPose action server unavailable"

            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = task.target
            self._set_state(
                task,
                TaskStatus.RESUMING if task.revision > 1 else TaskStatus.PLANNING,
                "submitting goal for fresh Nav2 planning",
            )
            send_future = self._nav_client.send_goal_async(
                nav_goal, feedback_callback=lambda _: self._publish_feedback(action_handle, task)
            )
            if not self._wait_future(send_future, 5.0):
                def cancel_late_goal(future):
                    try:
                        late_handle = future.result()
                        if late_handle.accepted:
                            late_handle.cancel_goal_async()
                    except Exception as exc:
                        self.get_logger().warning(
                            f"late Nav2 goal response could not be canceled: {exc}"
                        )

                send_future.add_done_callback(cancel_late_goal)
                return TaskStatus.FAILED, "timed out waiting for Nav2 goal response"
            nav_handle = send_future.result()
            if not nav_handle.accepted:
                return TaskStatus.FAILED, "Nav2 rejected the navigation goal"
            with self._lock:
                task.nav_goal_handle = nav_handle
                task.command = None
                task.control_event.clear()
            self._set_state(task, TaskStatus.NAVIGATING, "navigation active")
            result_future = nav_handle.get_result_async()
            while not result_future.done():
                if action_handle.is_cancel_requested and task.command != "cancel":
                    with self._lock:
                        task.command = "cancel"
                        task.revision += 1
                        task.control_started = time.monotonic()
                    nav_handle.cancel_goal_async()
                if time.monotonic() - started > timeout and not task.timed_out:
                    nav_handle.cancel_goal_async()
                    self._disarm_base()
                    task.timed_out = True
                    task.command = "cancel"
                    task.control_started = time.monotonic()
                    task.cancel_timeout_reported = True
                    self._set_state(task, TaskStatus.SAFETY_HOLD, "navigation task timed out")
                    started = time.monotonic()
                command = task.command
                if command in ("pause", "cancel"):
                    if task.state != TaskStatus.SAFETY_HOLD:
                        self._set_state(
                            task,
                            TaskStatus.PAUSING if command == "pause" else TaskStatus.CANCELING,
                            f"waiting for Nav2 {command} confirmation",
                        )
                    if (
                        task.control_started is not None
                        and time.monotonic() - task.control_started
                        > float(self.get_parameter("cancel_timeout_sec").value)
                    ):
                        if not task.cancel_timeout_reported:
                            self._disarm_base()
                            self._set_state(
                                task,
                                TaskStatus.SAFETY_HOLD,
                                "Nav2 cancel confirmation timed out; base disarm requested",
                            )
                            task.cancel_timeout_reported = True
                time.sleep(0.02)

            nav_result = result_future.result()
            with self._lock:
                task.nav_goal_handle = None
            if nav_result.status == GoalStatus.STATUS_SUCCEEDED:
                self._set_state(task, TaskStatus.SUCCEEDED, "Nav2 confirmed goal success")
                return TaskStatus.SUCCEEDED, "navigation succeeded"
            if nav_result.status == GoalStatus.STATUS_CANCELED:
                command = task.command
                if task.timed_out:
                    self._set_state(task, TaskStatus.SAFETY_HOLD, "Nav2 cancellation confirmed after task timeout")
                    return TaskStatus.SAFETY_HOLD, "task timed out; base disarm requested"
                if command == "cancel" or action_handle.is_cancel_requested:
                    self._set_state(task, TaskStatus.CANCELED, "Nav2 cancellation confirmed")
                    return TaskStatus.CANCELED, "navigation canceled"
                if command == "pause":
                    self._set_state(task, TaskStatus.PAUSING, "Nav2 cancellation confirmed; checking base stop")
                    stop_check_started = time.monotonic()
                    self._stop_samples = 0
                    if not self._wait_for_base_stop(
                        float(self.get_parameter("cancel_timeout_sec").value),
                        not_before=stop_check_started,
                    ):
                        self._disarm_base()
                        self._set_state(
                            task,
                            TaskStatus.SAFETY_HOLD,
                            "base stop was not confirmed; disarm requested",
                        )
                        return TaskStatus.SAFETY_HOLD, "base stop feedback missing or moving"
                    self._set_state(task, TaskStatus.PAUSED, "Nav2 canceled and base telemetry confirms stop")
                    pause_started = time.monotonic()
                    while task.command == "pause" and not action_handle.is_cancel_requested:
                        if time.monotonic() - pause_started > timeout:
                            self._disarm_base()
                            self._set_state(task, TaskStatus.SAFETY_HOLD, "pause remained unresolved")
                            return TaskStatus.SAFETY_HOLD, "pause timed out; base disarm requested"
                        task.control_event.wait(0.1)
                        task.control_event.clear()
                    if action_handle.is_cancel_requested or task.command == "cancel":
                        self._set_state(task, TaskStatus.CANCELED, "canceled while paused")
                        return TaskStatus.CANCELED, "navigation canceled while paused"
                    if task.command == "resume":
                        task.command = None
                        continue
                self._set_state(task, TaskStatus.FAILED, "Nav2 canceled without a task control request")
                return TaskStatus.FAILED, "unexpected Nav2 cancellation"
            self._set_state(task, TaskStatus.FAILED, f"Nav2 terminal status {nav_result.status}")
            return TaskStatus.FAILED, f"Nav2 failed with status {nav_result.status}"

    def _wait_future(self, future, timeout):
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        return completed.wait(timeout)

    def _on_control(self, request, response):
        state_to_publish = None
        with self._lock:
            task = self._active
            if task is None or task.task_id != request.task_id:
                response.accepted = False
                response.state = TaskStatus.IDLE if task is None else task.state
                response.message = "unknown or no longer active task"
                return response
            if request.operation == TaskControl.Request.PAUSE:
                if task.state in (TaskStatus.PAUSING, TaskStatus.PAUSED):
                    response.accepted = True
                    response.state = task.state
                    response.message = "pause already requested"
                    return response
                if task.state != TaskStatus.NAVIGATING:
                    response.accepted = False
                    response.state = task.state
                    response.message = "pause is valid only while navigating"
                    return response
                task.command = "pause"
                task.revision += 1
                task.control_started = time.monotonic()
                task.cancel_timeout_reported = False
                state_to_publish = TaskStatus.PAUSING
            elif request.operation == TaskControl.Request.RESUME:
                if task.state == TaskStatus.RESUMING:
                    response.accepted = True
                    response.state = task.state
                    response.message = "resume already requested"
                    return response
                if task.state != TaskStatus.PAUSED:
                    response.accepted = False
                    response.state = task.state
                    response.message = "resume is valid only after confirmed pause"
                    return response
                task.command = "resume"
                task.revision += 1
                state_to_publish = TaskStatus.RESUMING
            elif request.operation == TaskControl.Request.CANCEL:
                if task.state in (TaskStatus.CANCELING, TaskStatus.CANCELED):
                    response.accepted = True
                    response.state = task.state
                    response.message = "cancel already requested"
                    return response
                task.command = "cancel"
                task.revision += 1
                task.control_started = time.monotonic()
                task.cancel_timeout_reported = False
                state_to_publish = TaskStatus.CANCELING
            else:
                response.accepted = False
                response.state = task.state
                response.message = "unknown operation"
                return response
            task.control_event.set()
            nav_handle = task.nav_goal_handle
            response.accepted = True
            response.state = state_to_publish or task.state
            response.message = f"{task.command} requested"
        if state_to_publish is not None:
            self._set_state(task, state_to_publish, response.message)
        if nav_handle is not None and task.command in ("pause", "cancel"):
            nav_handle.cancel_goal_async()
        if task.command == "cancel" and task.state == TaskStatus.PAUSED:
            task.control_event.set()
        return response

    def _on_base_telemetry(self, msg):
        now = time.monotonic()
        self._last_telemetry_time = now
        healthy = (
            msg.connected
            and msg.handshake_ok
            and msg.valid
            and 0.0 <= msg.age_sec <= 0.1
            and msg.fault_bitmap == 0
            and abs(msg.measured_twist.linear.x)
            <= float(self.get_parameter("stop_vxy_threshold_m_s").value)
            and abs(msg.measured_twist.linear.y)
            <= float(self.get_parameter("stop_vxy_threshold_m_s").value)
            and abs(msg.measured_twist.angular.z)
            <= float(self.get_parameter("stop_wz_threshold_rad_s").value)
        )
        self._stop_samples = self._stop_samples + 1 if healthy else 0
        self._telemetry_event.set()

    def _wait_for_base_stop(self, timeout, *, not_before):
        required = max(1, int(self.get_parameter("stop_feedback_samples").value))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if (
                self._stop_samples >= required
                and self._last_telemetry_time >= not_before
                and time.monotonic() - self._last_telemetry_time <= 0.1
            ):
                return True
            self._telemetry_event.wait(0.05)
            self._telemetry_event.clear()
        return False

    def _pose_from_message(self, pose_msg):
        pose = pose_msg.pose
        frame = pose_msg.header.frame_id
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
        )
        if frame != "map":
            raise ValueError("navigation target must be expressed in map frame")
        if not all(math.isfinite(value) for value in values):
            raise ValueError("navigation pose contains non-finite geometry")
        proposal = PoseProposal(
            frame, pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
        )
        validate_pose_proposal(proposal)
        result = PoseStamped()
        result.header = pose_msg.header
        if result.header.stamp.sec == 0 and result.header.stamp.nanosec == 0:
            result.header.stamp = self.get_clock().now().to_msg()
        result.pose = pose
        return result

    def _waypoint_from_text(self, text, task):
        intent = parse_navigation_intent(text)
        if intent is None:
            raise ValueError("text is not a supported navigation request")
        name = intent.destination
        if name not in self._waypoints:
            if self._slow_planner is None:
                raise ValueError(f"unknown waypoint {name!r}; configure it before use")
            completed = threading.Event()
            selection = {}

            def call_planner():
                try:
                    selection["result"] = self._slow_planner.propose(
                        text,
                        task_id=task.task_id,
                        revision=task.revision,
                        allowed_waypoints=tuple(self._waypoints),
                    )
                except Exception as exc:
                    selection["error"] = exc
                finally:
                    completed.set()

            threading.Thread(target=call_planner, daemon=True).start()
            planner_deadline = (
                time.monotonic()
                + float(self.get_parameter("slow_planner_timeout_sec").value)
            )
            while not completed.is_set():
                if task.command == "cancel":
                    return None
                remaining = planner_deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError("slow task planner exceeded its configured timeout")
                completed.wait(min(0.1, remaining))
            with self._lock:
                if self._active is not task or task.command == "cancel":
                    return None
                if task.revision != 1:
                    raise RuntimeError("task revision changed while proposal was pending")
            if "error" in selection:
                raise ValueError(f"slow task planner failed: {selection['error']}")
            name = selection["result"].waypoint
        proposal = resolve_waypoint(name, self._waypoints)
        result = PoseStamped()
        result.header.frame_id = "map"
        result.header.stamp = self.get_clock().now().to_msg()
        result.pose.position.x = proposal.x
        result.pose.position.y = proposal.y
        result.pose.position.z = proposal.z
        result.pose.orientation.x = proposal.qx
        result.pose.orientation.y = proposal.qy
        result.pose.orientation.z = proposal.qz
        result.pose.orientation.w = proposal.qw
        return result

    def _set_state(self, task, state, reason):
        if task.state == state:
            return
        task.state = state
        msg = TaskStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = task.task_id
        msg.revision = task.revision
        msg.state = state
        msg.has_target = task.target is not None
        if task.target is not None:
            msg.target = task.target
        msg.reason = reason
        self._status_pub.publish(msg)
        self.get_logger().info(f"task {task.task_id} r{task.revision}: {state} ({reason})")

    def _publish_feedback(self, action_handle, task):
        feedback = ExecuteTask.Feedback()
        feedback.task_id = task.task_id
        feedback.revision = task.revision
        feedback.state = task.state
        feedback.message = "Nav2 is executing the current map-frame goal"
        feedback.has_target = task.target is not None
        if task.target is not None:
            feedback.resolved_goal = task.target
        action_handle.publish_feedback(feedback)

    def _publish_idle(self):
        msg = TaskStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = TaskStatus.IDLE
        msg.reason = "no task is active"
        self._status_pub.publish(msg)

    def _disarm_base(self):
        if not self._disarm_client.service_is_ready():
            self._disarm_client.wait_for_service(timeout_sec=0.1)
        if self._disarm_client.service_is_ready():
            request = SetBool.Request()
            request.data = False
            self._disarm_client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = TaskExecutor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
