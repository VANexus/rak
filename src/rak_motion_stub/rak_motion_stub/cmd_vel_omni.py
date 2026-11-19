"""cmd_vel_omni —— `/cmd_vel`(Twist) → `mecanum_drive_controller/reference`(TwistStamped)。

这是**底盘唯一的运动入口**：Nav2/寻路/键盘都发 `/cmd_vel`，控制器要的是带时间戳的
`TwistStamped`。在这之前本仓没有这一环（`/cmd_vel_raw` 没有任何订阅者，Nav2 发出去
的指令到不了轮子）。

为什么需要它而不是让上游直发：本车是**全向底盘**，`Twist` 的三路 `vx/vy/ωz` 都要
通；而 `mecanum_drive_controller` 的运动学假定的是标准（O 布局）麦轮。2026-11-18
实测本车滚子排布与假定左右相反：

  - `angular.z = +0.30` → IMU 积分 **−37.0°**、激光航向（rf2o，独立于 IMU）**−24.6°**，
    而轮系差动算出 +43.5°：轮子按命令转了，车体却反向转。
  - 反号再测（`−0.30`）→ IMU +6.8°、激光 +23.8°。两次都反向，确定的符号错误，不是噪声。

所以本节点提供两个**显式**的符号开关（默认按本车实测打开），而不是把它藏在
控制器里让人猜：

  - ``invert_angular_z``（默认 **true**）：转向取反，已由上述两次实测确定。
  - ``invert_lateral_y``（默认 **false**）：横移取反。**未裁决，且实测表明横移本身
    就不可靠**——见下。

横移实测（2026-11-19，PU/喷绘布赛道，``linear.y=+0.08`` 持续 1.5 s）：

    轮位置增量  FL -4.05  FR +0.71  RL +1.14  RR -3.53 (rad)
    车体表现    没有干净平移，绕 W1(前左) 转角附近自转，目视略偏左

斜对角同号说明命令确实被当成横移下发；但四轮幅度差约 4 倍，纯横移本应等幅。
结论：**在这块软质赛道上不要依赖 ``vy`` 做横向修正**——麦轮靠滚子侧滑，软布上
侧向摩擦大，横向指令会被转成自转。横向纠偏请走 ``wz``（偏航），需要横移前先在
目标地面上重测并据此裁决 ``invert_lateral_y``。

``vx`` 不动 —— 直行方向实测是对的（轮系 0.389 m ↔ 深度 0.378 m ↔ 雷达 0.381 m）。

安全：``cmd_timeout_s`` 内没有新指令就**主动发零**（控制器自己也有 0.5 s
`reference_timeout`，两条独立的兜底，任一条生效都停轮）。
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CmdVelOmni(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_omni")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("reference_topic", "/mecanum_drive_controller/reference")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("invert_angular_z", True)
        # 横移符号**未裁决**（见模块 docstring）：默认不取反，别拿猜测改行为。
        self.declare_parameter("invert_lateral_y", False)
        self.declare_parameter("cmd_timeout_s", 0.30)
        self.declare_parameter("publish_rate_hz", 20.0)

        self._invert_w = bool(self.get_parameter("invert_angular_z").value)
        self._invert_vy = bool(self.get_parameter("invert_lateral_y").value)
        self._timeout = float(self.get_parameter("cmd_timeout_s").value)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._frame = str(self.get_parameter("frame_id").value)

        self._vx = self._vy = self._w = 0.0
        self._stamp = None

        self._pub = self.create_publisher(
            TwistStamped, str(self.get_parameter("reference_topic").value), 10)
        self.create_subscription(
            Twist, str(self.get_parameter("cmd_vel_topic").value), self._on_cmd, 10)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.get_logger().info(
            f"cmd_vel_omni up: {self.get_parameter('cmd_vel_topic').value} -> "
            f"{self.get_parameter('reference_topic').value}; "
            f"invert_angular_z={self._invert_w} invert_lateral_y={self._invert_vy} "
            f"timeout={self._timeout}s")

    def _on_cmd(self, msg: Twist) -> None:
        self._vx = msg.linear.x
        self._vy = msg.linear.y
        self._w = msg.angular.z
        self._stamp = self.get_clock().now()

    def _tick(self) -> None:
        stale = (self._stamp is None or
                 (self.get_clock().now() - self._stamp).nanoseconds * 1e-9 > self._timeout)
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._frame
        if stale:
            # 主动发零：不要靠"不发"来停车，那要求下游看门狗一定在工作。
            out.twist.linear.x = 0.0
            out.twist.linear.y = 0.0
            out.twist.angular.z = 0.0
        else:
            out.twist.linear.x = self._vx
            out.twist.linear.y = -self._vy if self._invert_vy else self._vy
            out.twist.angular.z = -self._w if self._invert_w else self._w
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelOmni()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
