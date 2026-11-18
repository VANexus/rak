"""odom_cov_guard —— 打滑期间轮式里程计协方差膨胀中继（L2，§7.2）。

robot_localization 不支持运行期改协方差（见 `rak_bringup/config/ekf.yaml`
注释块），所以用最小中间层做运行期保护：

    /mecanum_drive_controller/odometry ──原样转发──► /odom_wheel ──► EKF odom1
                                              ▲
    /slip_watch/slip_suspected (std_msgs/Bool) ┘

规则：
  - 默认原样转发（含 header，**绝不重写时间戳**——EKF 的融合时序依赖
    原始采样时刻，重写时间戳会让 EKF 把两路传感器错配到同一滤波步）。
  - slip_suspected=true 期间，pose(x,y) 与 twist(vx,vy) 对角 × cov_factor；
    恢复后按 recover_ramp 决定立刻回落 1.0 或每帧 ÷ramp_step 斜坡回落。
  - 打滑信号缺失（slip_watch 没跑 / 超时 / 没激活）按"无打滑"处理
    （fail-open：不打断数据链），但诊断里持续报 "slip signal missing"。
    fail-open 的依据：本节点只是观测保护层，不该成为里程计的单点故障；
    真正的运动防线是 L3 gate 与固件 allow_motion（ADR-0020 §6）。

QoS：发布/订阅都用 RELIABLE + keep-last 10。sensor_data（BEST_EFFORT）
的发布者会让 RELIABLE 的订阅端静默断流——robot_localization 的 odom
订阅默认 RELIABLE，RELIABLE 发布者对 BEST_EFFORT 订阅者同样兼容，
所以 RELIABLE 是唯一两侧都不踩坑的选择。
"""

from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from rak_lifecycle.managed_node import ManagedNode

from .cov_guard import inflate_covariances, next_multiplier


class OdomCovGuardNode(ManagedNode):
    """受管节点：configure 建 relay，activate 后才转发。"""

    def __init__(self) -> None:
        super().__init__("odom_cov_guard", health_rate_hz=0.2)
        self._sub_odom = None
        self._sub_slip = None
        self._out = None
        self._slip_flag = False
        self._slip_at: float | None = None
        self._multiplier = 1.0
        self._forwarded = 0
        self._inflated = 0

    # ---------------------------------------------------------------- lifecycle
    def on_configure(self, state):
        super().on_configure(state)
        self._in_topic = str(self.declare(
            "input_topic", "/mecanum_drive_controller/odometry"))
        self._out_topic = str(self.declare("output_topic", "/odom_wheel"))
        self._slip_topic = str(self.declare(
            "slip_topic", "/slip_watch/slip_suspected"))
        # K 默认 10.0：ekf.yaml 注释既定的保守回退量级（"×N，N 取 10 起步"），
        # 落地期用真打滑数据标定后调。
        self._factor = float(self.declare("cov_factor", 10.0))
        self._recover_ramp = bool(self.declare("recover_ramp", True))
        self._ramp_step = float(self.declare("ramp_step", 4.0))
        self._slip_timeout_s = float(self.declare("slip_timeout_s", 2.0))
        if self._factor < 1.0 or self._ramp_step < 1.0:
            return self.fail("cov_factor 与 ramp_step 都必须 ≥ 1.0")

        qos = QoSProfile(
            depth=10, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE)
        self._sub_odom = self.create_subscription(
            Odometry, self._in_topic, self._on_odom, qos)
        self._sub_slip = self.create_subscription(
            Bool, self._slip_topic, self._on_slip, qos)
        self._out = self.lifecycle_publisher(Odometry, self._out_topic, 10)
        self.get_logger().info(
            f"odom_cov_guard: {self._in_topic} -> {self._out_topic} "
            f"cov_factor={self._factor} recover_ramp={self._recover_ramp} "
            f"ramp_step={self._ramp_step} slip={self._slip_topic}")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        ret = super().on_activate(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret
        self._multiplier = 1.0
        self._forwarded = 0
        self._inflated = 0
        self.add_timer(2.0, self._publish_diag)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self._sub_odom = None
        self._sub_slip = None
        self._out = None
        self._slip_at = None
        return super().on_cleanup(state)

    # ---------------------------------------------------------------- callbacks
    def _on_slip(self, msg: Bool) -> None:
        self._slip_flag = bool(msg.data)
        self._slip_at = time.monotonic()
        self.mark_seen("slip_suspected")

    def _slip_now(self) -> tuple[bool, bool]:
        """返回 (slip, signal_ok)。信号缺失/超时 = (False, False)（fail-open）。"""
        if self._slip_at is None or \
                time.monotonic() - self._slip_at > self._slip_timeout_s:
            return False, False
        return self._slip_flag, True

    def _on_odom(self, msg: Odometry) -> None:
        if not self.active:
            return
        slip, _ok = self._slip_now()
        ramp_step = self._ramp_step if self._recover_ramp else 1.0
        self._multiplier = next_multiplier(
            self._multiplier, slip, self._factor, ramp_step)
        if self._multiplier > 1.0:
            msg.pose.covariance, msg.twist.covariance = inflate_covariances(
                msg.pose.covariance, msg.twist.covariance, self._multiplier)
            self._inflated += 1
        self._forwarded += 1
        # header（含时间戳）原样透传，绝不重写。
        self.publish(msg, publisher=self._out)

    # ---------------------------------------------------------------- diagnostics
    def _publish_diag(self) -> None:
        _, signal_ok = self._slip_now()
        values = {
            "forwarded": self._forwarded,
            "inflated": self._inflated,
            "multiplier": round(self._multiplier, 3),
            "slip_signal": "ok" if signal_ok else "missing",
        }
        if signal_ok:
            self.report("relay", DiagnosticStatus.OK,
                        "inflating" if self._multiplier > 1.0 else "passthrough",
                        values)
        else:
            self.report("relay", DiagnosticStatus.WARN,
                        "slip signal missing (fail-open passthrough)", values)


def main(args=None) -> None:
    import rclpy

    rclpy.init(args=args)
    node = OdomCovGuardNode()
    # 不要在 main() 里自己 trigger_configure/activate：受管节点由 launch 的
    # managed()（或 system_manager）驱动转移，自己再来一次会和它抢同一个状态机，
    # 结果是 "Transition is not registered" 直接退出（本轮实测）。
    # 单独 `ros2 run` 调试时才手工驱动：
    #   ros2 lifecycle set /odom_cov_guard configure && ros2 lifecycle set /odom_cov_guard activate
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
