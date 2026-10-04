"""Exercise the software-only arm and independent gripper action servers."""

import threading

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
import pytest
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

from rak_arm_simulation.controllers import ARM_JOINTS, FakeJointControllers


class ControllerProbe(Node):
    def __init__(self):
        super().__init__("fake_controller_probe")
        self.arm = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self.gripper = ActionClient(
            self, GripperCommand, "/gripper_controller/gripper_cmd"
        )


def _wait(future, timeout=5.0):
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    assert done.wait(timeout), "action operation timed out"
    return future.result()


def test_arm_trajectory_and_independent_gripper_actions():
    rclpy.init()
    controllers = FakeJointControllers()
    probe = ControllerProbe()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(controllers)
    executor.add_node(probe)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        assert probe.arm.wait_for_server(timeout_sec=3.0)
        assert probe.gripper.wait_for_server(timeout_sec=3.0)

        arm_goal = FollowJointTrajectory.Goal()
        arm_goal.trajectory.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [0.1, 0.2, -0.3, 0.4, -0.5, 0.6]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200_000_000
        arm_goal.trajectory.points = [point]
        arm_handle = _wait(probe.arm.send_goal_async(arm_goal))
        assert arm_handle.accepted
        arm_result = _wait(arm_handle.get_result_async())
        assert arm_result.status == GoalStatus.STATUS_SUCCEEDED
        assert arm_result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        assert [controllers._positions[name] for name in ARM_JOINTS] == pytest.approx(
            list(point.positions)
        )

        gripper_goal = GripperCommand.Goal()
        gripper_goal.command.position = 0.03
        gripper_handle = _wait(probe.gripper.send_goal_async(gripper_goal))
        assert gripper_handle.accepted
        gripper_result = _wait(gripper_handle.get_result_async())
        assert gripper_result.status == GoalStatus.STATUS_SUCCEEDED
        assert gripper_result.result.reached_goal
        assert controllers._positions["gripper_joint"] == 0.03
    finally:
        executor.shutdown()
        for node in (probe, controllers):
            executor.remove_node(node)
            node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
