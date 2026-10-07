#!/usr/bin/env python3
"""[AI] 底盘麦轮运动测试节点(键盘)。

WASD/QE 调节机体目标速度, 20 Hz 持续发布 geometry_msgs/Twist, 经
Nav2/速度平滑/mecanum_drive_controller 链路驱动四轮, 用于观察
前后/左右/自转的运动能力。它只是测试输入源, 不替代 Nav2。

默认发布 /cmd_vel_unstamped (mecanum_drive_controller 的未Stamped入口);
参数 --ros-args -p topic:=/cmd_vel -p stamped:=true 可切换 TwistStamped。
键位: w/s 前后  a/d 左右  q/e 自转  空格全零  x 退出。
"""
import select
import sys
import termios
import threading
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile

KEY_STEP_LIN = 0.05    # m/s 每按一次步长
KEY_STEP_ANG = 0.25    # rad/s
MAX_LIN = 0.30         # m/s 上限(未标定前保持保守)
MAX_ANG = 1.20         # rad/s


def clamp(v, lim):
    return max(-lim, min(lim, v))


class ChassisKeyTest(Node):

    def __init__(self):
        super().__init__("chassis_key_test")
        self.declare_parameter("topic", "/cmd_vel_unstamped")
        self.declare_parameter("stamped", False)
        self.declare_parameter("rate", 20.0)
        topic = self.get_parameter("topic").value
        self.stamped = bool(self.get_parameter("stamped").value)
        rate = float(self.get_parameter("rate").value)
        self.pub = self.create_publisher(
            TwistStamped if self.stamped else Twist, topic, QoSProfile(depth=10))
        self.vx = self.vy = self.wz = 0.0
        try:
            self.old_term = termios.tcgetattr(0)
            self.running = True
            self.keys = threading.Thread(target=self._key_loop, daemon=True)
            self.keys.start()
        except termios.error:
            # [AI] 无 TTY(如 launch 后台)时只按当前零目标发布, 键盘禁用
            self.old_term = None
            self.running = True
            self.get_logger().warning("stdin 不是终端, 键盘输入禁用(仅持续发布当前目标)")
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.loginfo = self.get_logger().info
        self.loginfo(
            f"底盘键盘测试: 发布 {topic} ({'TwistStamped' if self.stamped else 'Twist'})"
            f" @{rate:.0f}Hz。键位: w/s 前后  a/d 左右  q/e 自转  空格全零  x 退出")

    def _key_loop(self):
        while self.running and rclpy.ok():
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not r:
                continue
            ch = sys.stdin.read(1)
            lin, ang = KEY_STEP_LIN, KEY_STEP_ANG
            if ch == "w":
                self.vx = clamp(self.vx + lin, MAX_LIN)
            elif ch == "s":
                self.vx = clamp(self.vx - lin, MAX_LIN)
            elif ch == "d":
                self.vy = clamp(self.vy + lin, MAX_LIN)
            elif ch == "a":
                self.vy = clamp(self.vy - lin, MAX_LIN)
            elif ch == "e":
                self.wz = clamp(self.wz + ang, MAX_ANG)
            elif ch == "q":
                self.wz = clamp(self.wz - ang, MAX_ANG)
            elif ch == " ":
                self.vx = self.vy = self.wz = 0.0
            elif ch in "xX\x03":
                self.running = False

    def _tick(self):
        twist = Twist()
        twist.linear.x = self.vx
        twist.linear.y = self.vy
        twist.angular.z = self.wz
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist = twist
            self.pub.publish(msg)
        else:
            self.pub.publish(twist)
        self.get_logger().info(
            f"vx={self.vx:+.2f} vy={self.vy:+.2f} wz={self.wz:+.2f}",
            throttle_duration_sec=1.0)

    def shutdown(self):
        self.running = False
        if self.old_term is not None:
            termios.tcsetattr(0, termios.TCSADRAIN, self.old_term)
        self.vx = self.vy = self.wz = 0.0
        self._tick()   # 退出前发一拍零目标
        time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = ChassisKeyTest()
    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
