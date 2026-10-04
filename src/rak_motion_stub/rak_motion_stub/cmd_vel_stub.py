"""cmd_vel_stub -- unicycle /cmd_vel bridge for 4-wheel mecanum robots.

nav2 controller outputs Twist(linear.x, angular.z). On a mecanum robot the
onboard MCU handles all 4-wheel kinematics; the ROS side only needs to forward
that Twist verbatim. This node:

  - subscribes to /cmd_vel (default; configurable)
  - republishes on /cmd_vel_raw for the MCU to consume (same Twist type)
  - logs every command at INFO so we can confirm nav2 is producing output
  - exposes a /cmd_vel_stub/enable service: when disabled, /cmd_vel is dropped
    (so a runaway nav2 doesn't make a half-built test platform drive into a
    wall). Default state on startup is enabled.

The unicycle contract is the only contract. There is no diff-drive decomposition
here -- that's deliberate; the MCU owns the wheel kinematics.

Run:
    ros2 run rak_motion_stub cmd_vel_stub
    ros2 run rak_motion_stub cmd_vel_stub --ros-args -p cmd_vel_topic:=/cmd_vel2 -p raw_topic:=/teensy/cmd_vel
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool


class CmdVelStub(Node):
    def __init__(self):
        super().__init__('cmd_vel_stub')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('raw_topic', '/cmd_vel_raw')
        self.declare_parameter('log_every_n', 10)

        cmd_topic = self.get_parameter('cmd_vel_topic').value
        raw_topic = self.get_parameter('raw_topic').value
        self._log_every = max(1, int(self.get_parameter('log_every_n').value))

        self.enabled_ = True
        self._count = 0

        self.raw_pub_ = self.create_publisher(Twist, raw_topic, 10)
        self.sub_ = self.create_subscription(Twist, cmd_topic, self._cb, 10)
        self.srv_ = self.create_service(SetBool, '~/enable', self._enable_srv)

        self.get_logger().info(
            f"cmd_vel_stub up: forwarding '{cmd_topic}' -> '{raw_topic}', "
            f"enabled={self.enabled_}, log every {self._log_every} msg"
        )

    def _enable_srv(self, req, resp):
        self.enabled_ = bool(req.data)
        resp.success = True
        resp.message = f"enabled={self.enabled_}"
        self.get_logger().warn(f"ENABLE toggled -> {self.enabled_}")
        return resp

    def _cb(self, msg):
        if not self.enabled_:
            return
        self.raw_pub_.publish(msg)
        self._count += 1
        if self._count % self._log_every == 0:
            self.get_logger().info(
                f"cmd[{self._count}]: lin.x={msg.linear.x:+.3f} m/s, "
                f"ang.z={msg.angular.z:+.3f} rad/s"
            )


def main():
    rclpy.init()
    node = CmdVelStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
