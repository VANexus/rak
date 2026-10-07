#!/usr/bin/env python3
# [AI] 轮式里程计:订阅 /base/telemetry 的实测体速度,积分发布 /wheel/odom
# 单位: BaseTelemetry.measured_twist 已是 SI(m/s, rad/s)。不发 TF(EKF 是权威)。
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from rak_interfaces.msg import BaseTelemetry

def yaw_to_quat(yaw):
    half = yaw / 2.0
    q = Quaternion()
    q.z, q.w = math.sin(half), math.cos(half)
    return q

class WheelOdom(Node):
    def __init__(self):
        super().__init__('wheel_odom')
        self.declare_parameter('telemetry_topic', '/base/telemetry')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.x = self.y = self.yaw = 0.0
        self._last_t = None
        self.create_subscription(BaseTelemetry,
            self.get_parameter('telemetry_topic').value, self._on_tel, 10)
        self._odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self._tf = TransformBroadcaster(self)

    def _on_tel(self, msg: BaseTelemetry):
        if not msg.valid:                     # 失联/超时:不积分
            self._last_t = None
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_t is None:
            self._last_t = t
            return
        dt = t - self._last_t
        if dt <= 0.0 or dt > 0.5:             # 时间戳跳变保护
            self._last_t = t
            return
        self._last_t = t
        vx = msg.measured_twist.linear.x      # m/s
        vy = msg.measured_twist.linear.y
        wz = msg.measured_twist.angular.z     # rad/s
        self.yaw += wz * dt
        self.x  += (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        self.y  += (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt

        o = Odometry()
        o.header.stamp = stamp = msg.header.stamp
        o.header.frame_id = self.get_parameter('odom_frame').value
        o.child_frame_id = self.get_parameter('base_frame').value
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation = yaw_to_quat(self.yaw)
        o.twist.twist = msg.measured_twist
        o.pose.covariance[0]  = 0.05
        o.pose.covariance[7]  = 0.05
        o.pose.covariance[35] = 0.07
        self._odom_pub.publish(o)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.get_parameter('odom_frame').value
        tf.child_frame_id = self.get_parameter('base_frame').value
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation = yaw_to_quat(self.yaw)
        self._tf.sendTransform(tf)

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
