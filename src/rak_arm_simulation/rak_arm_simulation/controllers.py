"""Kinematic-only action servers; this node must never be used for hardware."""

from __future__ import annotations

import asyncio
import math
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINTS = (
    "base_yaw_joint",
    "shoulder_pitch_joint",
    "elbow_pitch_joint",
    "wrist_yaw_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
)
JOINT_BOUNDS = {
    "base_yaw_joint": (-3.14159, 3.14159),
    "shoulder_pitch_joint": (-1.57, 1.57),
    "elbow_pitch_joint": (-2.2, 2.2),
    "wrist_yaw_joint": (-3.14159, 3.14159),
    "wrist_pitch_joint": (-1.57, 1.57),
    "wrist_roll_joint": (-3.14159, 3.14159),
    "gripper_joint": (0.0, 0.05),
}
ALL_JOINTS = (*ARM_JOINTS, "gripper_joint")


def _duration_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


class FakeJointControllers(Node):
    def __init__(self):
        super().__init__("fake_joint_controllers")
        self._lock = threading.RLock()
        self._positions = {name: 0.0 for name in ALL_JOINTS}
        self._group = ReentrantCallbackGroup()
        self._joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._joint_timer = self.create_timer(
            0.02, self._publish_joint_state, callback_group=self._group
        )
        self._arm_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            execute_callback=self._execute_arm,
            goal_callback=self._arm_goal,
            cancel_callback=self._cancel_goal,
            callback_group=self._group,
        )
        self._gripper_server = ActionServer(
            self,
            GripperCommand,
            "/gripper_controller/gripper_cmd",
            execute_callback=self._execute_gripper,
            goal_callback=self._gripper_goal,
            cancel_callback=self._cancel_goal,
            callback_group=self._group,
        )
        self.get_logger().warning(
            "software-only kinematic controllers active; no physics or hardware interface"
        )

    def _arm_goal(self, goal):
        trajectory = goal.trajectory
        names = tuple(trajectory.joint_names)
        if len(names) != len(ARM_JOINTS) or set(names) != set(ARM_JOINTS):
            return GoalResponse.REJECT
        if not trajectory.points:
            return GoalResponse.REJECT
        previous_time = -1.0
        for point in trajectory.points:
            point_time = _duration_seconds(point.time_from_start)
            if (
                len(point.positions) != len(names)
                or not math.isfinite(point_time)
                or point_time < previous_time
                or not all(math.isfinite(value) for value in point.positions)
            ):
                return GoalResponse.REJECT
            for name, value in zip(names, point.positions):
                lower, upper = JOINT_BOUNDS[name]
                if not lower <= value <= upper:
                    return GoalResponse.REJECT
            previous_time = point_time
        return GoalResponse.ACCEPT

    @staticmethod
    def _gripper_goal(goal):
        position = goal.command.position
        lower, upper = JOINT_BOUNDS["gripper_joint"]
        if not math.isfinite(position) or not lower <= position <= upper:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_goal(_goal_handle):
        return CancelResponse.ACCEPT

    async def _execute_arm(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        names = tuple(trajectory.joint_names)
        with self._lock:
            start_positions = tuple(self._positions[name] for name in names)

        points = []
        previous_time = 0.0
        previous_positions = start_positions
        for point in trajectory.points:
            point_time = _duration_seconds(point.time_from_start)
            positions = tuple(float(value) for value in point.positions)
            if point_time < previous_time or len(positions) != len(names):
                return self._abort_trajectory(goal_handle, "invalid trajectory point order")
            points.append(
                (previous_time, point_time, previous_positions, positions)
            )
            previous_time = point_time
            previous_positions = positions

        started = time.monotonic()
        duration = points[-1][0]
        while True:
            elapsed = min(time.monotonic() - started, duration)
            desired = self._sample(points, elapsed)
            with self._lock:
                for name, value in zip(names, desired):
                    self._positions[name] = value
            self._publish_joint_state()
            self._publish_arm_feedback(goal_handle, names, desired)

            if goal_handle.is_cancel_requested:
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "trajectory canceled by client"
                goal_handle.canceled()
                return result
            if elapsed >= duration:
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = ""
                goal_handle.succeed()
                return result
            await asyncio.sleep(0.02)

    @staticmethod
    def _sample(points, elapsed):
        for start_time, end_time, start_positions, end_positions in points:
            if elapsed <= end_time:
                span = end_time - start_time
                if span <= 0.0:
                    return end_positions
                ratio = min(1.0, max(0.0, (elapsed - start_time) / span))
                return tuple(
                    start + (end - start) * ratio
                    for start, end in zip(start_positions, end_positions)
                )
        return points[-1][3]

    def _publish_arm_feedback(self, goal_handle, names, positions):
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = list(names)
        feedback.desired = JointTrajectoryPoint()
        feedback.desired.positions = list(positions)
        feedback.actual = JointTrajectoryPoint()
        feedback.actual.positions = list(positions)
        feedback.error = JointTrajectoryPoint()
        feedback.error.positions = [0.0] * len(names)
        goal_handle.publish_feedback(feedback)

    def _abort_trajectory(self, goal_handle, reason):
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        result.error_string = reason
        goal_handle.abort()
        return result

    def _execute_gripper(self, goal_handle):
        position = float(goal_handle.request.command.position)
        with self._lock:
            self._positions["gripper_joint"] = position
        self._publish_joint_state()

        feedback = GripperCommand.Feedback()
        feedback.position = position
        feedback.effort = 0.0
        feedback.stalled = False
        feedback.reached_goal = True
        goal_handle.publish_feedback(feedback)

        result = GripperCommand.Result()
        result.position = position
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = True
        goal_handle.succeed()
        return result

    def _publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self._lock:
            msg.name = list(ALL_JOINTS)
            msg.position = [self._positions[name] for name in ALL_JOINTS]
        self._joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeJointControllers()
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
