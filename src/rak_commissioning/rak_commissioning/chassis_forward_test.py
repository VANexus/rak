#!/usr/bin/env python3
"""[AI] 底盘真机"前移 10 s"标准化测试（G3 门）。

契约与 chassis_tui 完全一致：发布 /mecanum_drive_controller/reference
(TwistStamped) @20 Hz；控制器侧 reference_timeout=0.5 s 兜底，脚本退出前
补发零目标。速度曲线：1 s 梯形加速 -> 保持 speed 持续 duration -> 1 s 减速
-> 2 s 零目标。

安全门（全部不满足拒绝执行）：
  1. 必须显式 `--yes`（真实运动许可，对应 bringup 侧 allow_motion:=true）；
  2. 启动前 /joint_states 必须已有四轮速度反馈（驱动层未就绪不开步）；
  3. 加速后 1.5 s 轮反馈仍全零 -> 立即归零退出（八成是 allow_motion=false
     或 enable 位没使能，绝不继续加速）。

用法（bringup 已起、底盘 G0/G1/G2 已过）:
  ros2 run rak_commissioning chassis_forward_test            # 预演，不运动
  ros2 run rak_commissioning chassis_forward_test -- --yes   # 真实前移 10 s
参数: -p speed:=0.10 -p duration:=10.0（m/s、保持时长）
"""
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState

WHEELS = ["front_left_wheel", "front_right_wheel",
          "rear_left_wheel", "rear_right_wheel"]
WHEEL_LABELS = ["FL", "FR", "RL", "RR"]
DEFAULT_TOPIC = "/mecanum_drive_controller/reference"


class ForwardTest(Node):

    def __init__(self):
        super().__init__("chassis_forward_test")
        self.declare_parameter("topic", DEFAULT_TOPIC)
        self.declare_parameter("speed", 0.10)
        self.declare_parameter("duration", 10.0)
        self.declare_parameter("rate", 20.0)
        topic = str(self.get_parameter("topic").value)
        self.speed = float(self.get_parameter("speed").value)
        self.duration = float(self.get_parameter("duration").value)
        rate = float(self.get_parameter("rate").value)
        if self.duration <= 0.0 or not 0.0 < self.speed <= 0.15:
            raise SystemExit("duration 必须 >0；speed 取 (0, 0.15] m/s（未标定保守值）")

        self.pub = self.create_publisher(TwistStamped, topic, QoSProfile(depth=10))
        self.wheel_vel = [None] * 4
        self.joints_ts = None
        self.create_subscription(JointState, "/joint_states", self._on_joints,
                                 QoSProfile(depth=10))
        self._zero()
        self.get_logger().info(
            f"前移测试: {topic} @20Hz, speed={self.speed} m/s, "
            f"保持={self.duration}s。Ctrl-C 随时急停归零。")

    def _on_joints(self, msg):
        self.joints_ts = time.monotonic()
        for i, name in enumerate(WHEELS):
            if name in msg.name:
                j = msg.name.index(name)
                if j < len(msg.velocity):
                    self.wheel_vel[i] = msg.velocity[j]

    def feedback_fresh(self, within=1.0):
        return (self.joints_ts is not None
                and time.monotonic() - self.joints_ts <= within
                and all(v is not None for v in self.wheel_vel))

    def _cmd(self, x):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = x
        self.pub.publish(msg)

    def _zero(self):
        self._cmd(0.0)


def main(args=None):
    rclpy.init(args=args)
    node = ForwardTest()

    if "--yes" not in (sys.argv[1:] or []):
        node._zero()
        node.get_logger().info(
            "预演模式（未给 --yes，不运动）。确认车轮架空/场地清空后加 --yes 执行。")
        node.destroy_node()
        rclpy.shutdown()
        return 0

    stop = False

    def on_sig(_s, _f):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    # 门1：反馈就绪
    deadline = time.monotonic() + 3.0
    while not stop and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.feedback_fresh():
            break
    if not node.feedback_fresh():
        node.get_logger().error(
            "拒绝执行：/joint_states 无四轮速度反馈（bringup 未起 / chassis 板"
            "不在总线 / joint_state_broadcaster 未激活）。先过 G0-G2 门。")
        node._zero()
        node.destroy_node()
        rclpy.shutdown()
        return 1

    max_fb = [0.0] * 4   # 带符号峰值（判方向用）
    stalled = False
    period = 1.0 / float(node.get_parameter("rate").value)

    phases = [("加速", 1.0, lambda t: node.speed * t),           # t: 0..1
              ("保持", node.duration, lambda _t: node.speed),
              ("减速", 1.0, lambda t: node.speed * (1.0 - t))]
    node.get_logger().info(f"运动开始：前移 {node.speed} m/s，保持 {node.duration}s。")
    for name, span, law in phases:
        t0 = time.monotonic()
        while not stop and time.monotonic() - t0 < span:
            rclpy.spin_once(node, timeout_sec=0.0)
            frac = (time.monotonic() - t0) / span
            node._cmd(law(frac))
            rclpy.spin_once(node, timeout_sec=period)
            for i, v in enumerate(node.wheel_vel):
                if v is not None:
                    # 带符号峰值：判定方向要用它，停车后的末尾读数已归零
                    if abs(v) > abs(max_fb[i]):
                        max_fb[i] = v
            # 门2：保持段开跑 1.5 s 后轮反馈仍全零 -> 让步（多半 allow_motion=false）
            if name == "保持" and not stalled:
                if time.monotonic() - t0 > 1.5 and all(
                        abs(v or 0.0) < 0.01 for v in node.wheel_vel):
                    stalled = True
                    node.get_logger().error(
                        "轮反馈全零：目标已发但车未动。最可能 allow_motion=false"
                        "（桥只发零目标）或 MCU enable 位未开。停止本次测试。")
                    stop = True

    node._zero()
    zero_until = time.monotonic() + 2.0
    while time.monotonic() < zero_until:
        node._zero()
        rclpy.spin_once(node, timeout_sec=period)

    fb = " ".join(f"{lbl}={v:.2f}" if v is not None else f"{lbl}=--"
                  for lbl, v in zip(WHEEL_LABELS, node.wheel_vel))
    sign_ok = (not stalled) and all(v > 0.05 for v in max_fb)
    node.get_logger().info(
        f"结束。停止瞬间反馈 {fb}；带符号峰值 {[round(m, 2) for m in max_fb]}；"
        f"方向一致性(四轮峰值均为正): {'OK' if sign_ok else '未过——查轮序/极性'}。")
    if stalled:
        node.destroy_node()
        rclpy.shutdown()
        return 1
    node.destroy_node()
    rclpy.shutdown()
    return 0 if sign_ok else 1


if __name__ == "__main__":
    sys.exit(main())
