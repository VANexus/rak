"""Fake navigation and base feedback for task lifecycle testing only."""

import math
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rak_interfaces.msg import BaseTelemetry
from std_srvs.srv import SetBool


class FakeNavigation(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__(
            "fake_navigation_server", parameter_overrides=parameter_overrides
        )
        self.declare_parameter("goal_duration_sec", 60.0)
        self._goal_duration = float(self.get_parameter("goal_duration_sec").value)
        if not math.isfinite(self._goal_duration) or self._goal_duration <= 0.0:
            raise ValueError("goal_duration_sec must be finite and positive")
        self._moving = False
        self._armed = True
        self.accepted_goal_count = 0
        self._telemetry = self.create_publisher(BaseTelemetry, "/base/telemetry", 10)
        self._timer = self.create_timer(0.02, self._publish_telemetry)
        self._group = ReentrantCallbackGroup()
        self._arm_service = self.create_service(
            SetBool, "/base_uart_driver/arm", self._set_armed,
            callback_group=self._group,
        )
        self._action = ActionServer(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            execute_callback=self._execute,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self._group,
        )
        self.get_logger().warning(
            "SIMULATION ONLY: NavigateToPose is held until canceled or its configured duration"
        )

    @staticmethod
    def _accept_goal(_goal):
        return GoalResponse.ACCEPT

    @staticmethod
    def _accept_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def _set_armed(self, request, response):
        self._armed = bool(request.data)
        response.success = True
        response.message = "simulation arm state updated"
        return response

    def _execute(self, goal_handle):
        self._moving = True
        self.accepted_goal_count += 1
        started = time.monotonic()
        while time.monotonic() - started < self._goal_duration:
            if goal_handle.is_cancel_requested:
                self._moving = False
                result = NavigateToPose.Result()
                result.error_code = NavigateToPose.Result.NONE
                result.error_msg = "simulation goal canceled"
                goal_handle.canceled()
                return result
            time.sleep(0.02)

        self._moving = False
        result = NavigateToPose.Result()
        result.error_code = NavigateToPose.Result.NONE
        result.error_msg = "simulation goal completed"
        goal_handle.succeed()
        return result

    def _publish_telemetry(self):
        msg = BaseTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.connected = True
        msg.handshake_ok = True
        msg.valid = True
        msg.armed = self._armed
        msg.mode = 1 if self._armed else 0
        if self._moving and self._armed:
            msg.measured_twist.linear.x = 0.1
        msg.age_sec = 0.0
        msg.status = (
            "SIMULATION ONLY: commanded-motion estimate, not physical telemetry"
        )
        self._telemetry.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeNavigation()
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
