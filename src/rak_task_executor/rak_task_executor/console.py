"""Interactive text entry point for navigation tasks and quick control phrases."""

import threading
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rak_intent.task_intent import ControlCommand, parse_control_command
from rak_interfaces.action import ExecuteTask
from rak_interfaces.msg import TaskStatus
from rak_interfaces.srv import TaskControl


class TaskConsole(Node):
    def __init__(self):
        super().__init__("task_console")
        self._action = ActionClient(self, ExecuteTask, "/task/execute")
        self._control = self.create_client(TaskControl, "/task/control")
        self._status_sub = self.create_subscription(
            TaskStatus, "/task/status", self._on_status, 10
        )
        self._lock = threading.Lock()
        self._active_task_id = ""
        self._request_ids = {}
        self._terminal_event = threading.Event()

    def _on_status(self, status):
        if status.task_id and status.state not in (
            TaskStatus.SUCCEEDED,
            TaskStatus.FAILED,
            TaskStatus.CANCELED,
            TaskStatus.SAFETY_HOLD,
        ):
            with self._lock:
                self._active_task_id = status.task_id
                self._terminal_event.clear()
        elif status.task_id:
            with self._lock:
                if self._active_task_id == status.task_id:
                    self._active_task_id = ""
            self._terminal_event.set()
        label = {
            TaskStatus.IDLE: "IDLE",
            TaskStatus.ACCEPTED: "ACCEPTED",
            TaskStatus.PLANNING: "PLANNING",
            TaskStatus.NAVIGATING: "NAVIGATING",
            TaskStatus.PAUSING: "PAUSING",
            TaskStatus.PAUSED: "PAUSED",
            TaskStatus.RESUMING: "RESUMING",
            TaskStatus.CANCELING: "CANCELING",
            TaskStatus.SUCCEEDED: "SUCCEEDED",
            TaskStatus.FAILED: "FAILED",
            TaskStatus.CANCELED: "CANCELED",
            TaskStatus.SAFETY_HOLD: "SAFETY_HOLD",
        }.get(status.state, f"UNKNOWN({status.state})")
        print(f"[task] {status.task_id or '-'} r{status.revision} state={label}: {status.reason}")

    def submit_text(self, text):
        command = parse_control_command(text)
        if command is not None:
            self._send_control(command)
            return
        goal = ExecuteTask.Goal()
        request_id = str(uuid.uuid4())
        goal.request_id = request_id
        goal.text = text
        goal.has_pose = False
        future = self._action.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(lambda result: self._on_goal_response(request_id, result))

    def _on_goal_response(self, request_id, future):
        try:
            handle = future.result()
        except Exception as exc:
            print(f"[task] goal request failed: {exc}")
            return
        if not handle.accepted:
            print("[task] request rejected; check task status and configured waypoint aliases")
            return
        self._request_ids[request_id] = handle
        print(f"[task] accepted request {request_id}")
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda result: self._on_result(request_id, result))

    def _on_feedback(self, feedback):
        data = feedback.feedback
        with self._lock:
            self._active_task_id = data.task_id
        print(f"[task] {data.task_id} r{data.revision}: {data.message}")

    def _on_result(self, request_id, future):
        try:
            wrapped = future.result()
            result = wrapped.result
            print(
                f"[task] terminal={result.terminal_state} succeeded={result.succeeded}: "
                f"{result.message}"
            )
        except Exception as exc:
            print(f"[task] result failed: {exc}")
        self._request_ids.pop(request_id, None)

    def _send_control(self, command):
        with self._lock:
            task_id = self._active_task_id
        if not task_id:
            print("[task] no active task to control")
            return
        operation = {
            ControlCommand.PAUSE: TaskControl.Request.PAUSE,
            ControlCommand.RESUME: TaskControl.Request.RESUME,
            ControlCommand.CANCEL: TaskControl.Request.CANCEL,
        }[command]
        if not self._control.service_is_ready():
            print("[task] control service is not available")
            return
        request = TaskControl.Request()
        request.task_id = task_id
        request.operation = operation
        future = self._control.call_async(request)
        future.add_done_callback(
            lambda done: self._print_control_result(command, done)
        )

    @staticmethod
    def _print_control_result(command, future):
        try:
            response = future.result()
            print(
                f"[task] {command.value}: accepted={response.accepted}, "
                f"state={response.state}: {response.message}"
            )
        except Exception as exc:
            print(f"[task] {command.value} failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = TaskConsole()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    print("Enter a navigation request, pause/resume/cancel, or Ctrl-D to exit.")
    try:
        while rclpy.ok():
            try:
                text = input("> ").strip()
            except EOFError:
                break
            if text:
                node.submit_text(text)
    except KeyboardInterrupt:
        pass
    finally:
        with node._lock:
            active_task_id = node._active_task_id
        if active_task_id and node._control.service_is_ready():
            request = TaskControl.Request()
            request.task_id = active_task_id
            request.operation = TaskControl.Request.CANCEL
            node._terminal_event.clear()
            cancel_future = node._control.call_async(request)
            control_done = threading.Event()
            cancel_future.add_done_callback(lambda _: control_done.set())
            control_done.wait(2.0)
            if control_done.is_set():
                node._terminal_event.wait(7.0)
            if not node._terminal_event.is_set():
                print("[task] cancellation is still pending; verify /task/status")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=1)
