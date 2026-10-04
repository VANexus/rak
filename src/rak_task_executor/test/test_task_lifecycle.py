"""ROS integration test for pause-confirm-resume-cancel against fake Nav2."""

import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rak_interfaces.action import ExecuteTask
from rak_interfaces.msg import TaskStatus
from rak_interfaces.srv import TaskControl

from rak_task_executor.node import TaskExecutor
from rak_task_executor.sim_nav import FakeNavigation


class TaskProbe(Node):
    def __init__(self):
        super().__init__("task_lifecycle_probe")
        self.action = ActionClient(self, ExecuteTask, "/task/execute")
        self.control = self.create_client(TaskControl, "/task/control")
        self.condition = threading.Condition()
        self.statuses = []
        self.create_subscription(TaskStatus, "/task/status", self._on_status, 10)

    def _on_status(self, msg):
        with self.condition:
            self.statuses.append(msg)
            self.condition.notify_all()

    def wait_state(self, state, *, revision=0, timeout=8.0):
        deadline = time.monotonic() + timeout
        with self.condition:
            while time.monotonic() < deadline:
                match = next(
                    (
                        msg for msg in reversed(self.statuses)
                        if msg.state == state and msg.revision >= revision
                    ),
                    None,
                )
                if match is not None:
                    return match
                self.condition.wait(max(0.0, deadline - time.monotonic()))
        raise AssertionError(f"state {state} revision >= {revision} was not observed")

    def call_control(self, task_id, operation):
        request = TaskControl.Request()
        request.task_id = task_id
        request.operation = operation
        future = self.control.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        assert event.wait(3.0), "task-control service timed out"
        return future.result()


def test_pause_resume_cancel_lifecycle(tmp_path):
    rclpy.init()
    waypoint_file = tmp_path / "waypoints.yaml"
    waypoint_file.write_text(
        "schema_version: 1\nframe_id: map\nwaypoints:\n"
        "  simulation_origin: {x: 0.0, y: 0.0}\n",
        encoding="utf-8",
    )
    fake_nav = FakeNavigation()
    executor_node = TaskExecutor(parameter_overrides=[
        Parameter("waypoints_file", value=str(waypoint_file)),
        Parameter("cancel_timeout_sec", value=2.0),
        Parameter("task_timeout_sec", value=30.0),
    ])
    probe = TaskProbe()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(fake_nav)
    executor.add_node(executor_node)
    executor.add_node(probe)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        assert probe.action.wait_for_server(timeout_sec=3.0)
        assert probe.control.wait_for_service(timeout_sec=3.0)

        goal = ExecuteTask.Goal()
        goal.request_id = "lifecycle-test-1"
        goal.text = "go to simulation_origin"
        goal.has_pose = False
        send_future = probe.action.send_goal_async(goal)
        send_done = threading.Event()
        send_future.add_done_callback(lambda _: send_done.set())
        assert send_done.wait(3.0), "task action goal response timed out"
        goal_handle = send_future.result()
        assert goal_handle.accepted

        duplicate = ExecuteTask.Goal()
        duplicate.request_id = goal.request_id
        duplicate.text = goal.text
        duplicate.has_pose = False
        duplicate_future = probe.action.send_goal_async(duplicate)
        duplicate_done = threading.Event()
        duplicate_future.add_done_callback(lambda _: duplicate_done.set())
        assert duplicate_done.wait(3.0), "duplicate request response timed out"
        assert not duplicate_future.result().accepted

        navigating = probe.wait_state(TaskStatus.NAVIGATING)
        pause = probe.call_control(navigating.task_id, TaskControl.Request.PAUSE)
        assert pause.accepted
        probe.wait_state(TaskStatus.PAUSED, revision=2)

        resume = probe.call_control(navigating.task_id, TaskControl.Request.RESUME)
        assert resume.accepted
        probe.wait_state(TaskStatus.NAVIGATING, revision=3)
        assert fake_nav.accepted_goal_count == 2

        cancel = probe.call_control(navigating.task_id, TaskControl.Request.CANCEL)
        assert cancel.accepted
        probe.wait_state(TaskStatus.CANCELED, revision=4)

        result_future = goal_handle.get_result_async()
        result_done = threading.Event()
        result_future.add_done_callback(lambda _: result_done.set())
        assert result_done.wait(3.0), "task action result timed out"
        wrapped_result = result_future.result()
        result = wrapped_result.result
        assert not result.succeeded
        assert result.terminal_state == TaskStatus.CANCELED
        assert wrapped_result.status == GoalStatus.STATUS_ABORTED
    finally:
        executor.shutdown()
        for node in (probe, executor_node, fake_nav):
            executor.remove_node(node)
            node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


def test_navigation_success(tmp_path):
    rclpy.init()
    waypoint_file = tmp_path / "waypoints.yaml"
    waypoint_file.write_text(
        "schema_version: 1\nframe_id: map\nwaypoints:\n"
        "  simulation_origin: {x: 0.0, y: 0.0}\n",
        encoding="utf-8",
    )
    fake_nav = FakeNavigation(parameter_overrides=[
        Parameter("goal_duration_sec", value=0.15),
    ])
    executor_node = TaskExecutor(parameter_overrides=[
        Parameter("waypoints_file", value=str(waypoint_file)),
        Parameter("task_timeout_sec", value=5.0),
    ])
    probe = TaskProbe()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(fake_nav)
    executor.add_node(executor_node)
    executor.add_node(probe)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        assert probe.action.wait_for_server(timeout_sec=3.0)
        goal = ExecuteTask.Goal()
        goal.request_id = "success-test-1"
        goal.text = "go to simulation_origin"
        goal.has_pose = False
        send_future = probe.action.send_goal_async(goal)
        send_done = threading.Event()
        send_future.add_done_callback(lambda _: send_done.set())
        assert send_done.wait(3.0), "task action goal response timed out"
        goal_handle = send_future.result()
        assert goal_handle.accepted
        probe.wait_state(TaskStatus.SUCCEEDED, timeout=5.0)
        result_future = goal_handle.get_result_async()
        result_done = threading.Event()
        result_future.add_done_callback(lambda _: result_done.set())
        assert result_done.wait(3.0), "task action result timed out"
        result = result_future.result().result
        assert result.succeeded
        assert result.terminal_state == TaskStatus.SUCCEEDED
    finally:
        executor.shutdown()
        for node in (probe, executor_node, fake_nav):
            executor.remove_node(node)
            node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
