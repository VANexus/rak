"""slip_watch —— 抗打滑 L1 检测 + L3 执行保护的观测节点。

设计出处：`rak/docs/firmware-capability-spec.md` §7.2（三级响应）。
职责边界（AGENTS/架构红线）：
  - L1（本节点核心）：残差 = ω_kinematic(四轮速+几何) − ω_gyro_z（权威 =
    Jetson LSM6DSR /imu/data_raw），p50/p95 进 /rak/metrics（DiagnosticArray）。
  - L2（不在这里）：robot_localization 不支持动态协方差 —— 静态保守回退
    （切 differential / 降 odom0 权重）写在 `rak_bringup/config/ekf.yaml` 注释块。
  - L3（本节点提供通路，**默认不接线**）：打滑嫌疑期间把 cmd_vel_in 的
    平移分量 ×0.5 发到 cmd_vel_out（恢复期每帧 +1/8 斜坡回 1.0）。
    bringup 重接线（collision_monitor → 本节点 → 控制器）是**落地期**动作：
    架空期真打滑不可复现，先只验证数据通路（§7.2 架空期限制）。

新鲜度：/joint_states ≈100 Hz 为计算节拍，取最近一次 gyro-z；
IMU 年龄 > imu_max_age_ms 视为不新鲜，该拍不计残差（计数器上报）。

阈值依据：sigma_baseline 默认 0（未标定）→ 只记数不判定，绝不产生
打滑结论；落地期用 rak_metrics 首日基线的 p95 回填。
"""

from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState

from rak_lifecycle.managed_node import ManagedNode

from .slip import SlipDetector
from .stats import Window

WINDOW = 200
#: wheel 关节名子串（chassis_controllers.yaml：front/rear_left/right_wheel）
WHEEL_STEM = "wheel"
#: 四轮顺序规范（与 chassis_bringup wheel_sign 参数一致）：FL FR RL RR
_WHEEL_ORDER = (
    ("front_left", "FL"), ("front_right", "FR"),
    ("rear_left", "RL"), ("rear_right", "RR"),
)


class SlipWatchNode(ManagedNode):
    """受管节点：configure 建订阅/发布器，activate 后窗口才滚。"""

    def __init__(self) -> None:
        super().__init__("slip_watch", health_rate_hz=0.2)
        self._wheel_vel: dict[str, float] = {}
        self._gyro_z: float | None = None
        self._gyro_at: float | None = None
        self._detector: SlipDetector | None = None
        self._residuals = Window(WINDOW)
        self._stale_pairs = 0
        self._evaluated = 0
        self._slip = False
        self._gate_scale = 1.0
        self._metrics_pub = None
        self._gate_pub = None

    # ---------------------------------------------------------------- lifecycle
    def on_configure(self, state):
        super().on_configure(state)
        radius = float(self.declare("wheels_radius", 0.0275))
        wheel_base_sum = float(self.declare("wheel_base_sum", 0.26))
        n_sigma = float(self.declare("n_sigma", 3.0))
        sigma = float(self.declare("sigma_baseline", 0.0))
        hold = int(self.declare("hold_cycles", 5))
        imu_age = float(self.declare("imu_max_age_ms", 50.0))
        gate = bool(self.declare("gate_enabled", False))
        self._imu_max_age_s = imu_age * 1e-3
        self._gate_enabled = gate
        # 权重符号未现场核对（CAN-BRINGUP §7.1）——全局翻转不影响 |residual| 判据
        weights = self.declare("wheel_weights", "[-1.0, 1.0, -1.0, 1.0]")
        import json
        try:
            weight_list = tuple(float(x) for x in json.loads(str(weights)))
        except (ValueError, TypeError):
            return self.fail(f"wheel_weights 不是 JSON 数字数组: {weights!r}")
        if len(weight_list) != 4:
            return self.fail(f"wheel_weights 需要四个数，得到 {len(weight_list)}")
        self._detector = SlipDetector(
            wheels_radius=radius, wheel_base_sum=wheel_base_sum,
            wheel_weights=weight_list, n_sigma=n_sigma,
            sigma_baseline=sigma, hold_cycles=hold)

        qos = QoSProfile(
            depth=50, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT)
        self._subs = [
            self.create_subscription(
                JointState, "joint_states", self._on_joint_states, qos),
            self.create_subscription(Imu, "imu/data_raw", self._on_imu, qos),
        ]
        self._metrics_pub = self.lifecycle_publisher(
            DiagnosticArray, "/rak/metrics", 10)
        self._gate_pub = self.lifecycle_publisher(Twist, "cmd_vel_gated", 10)
        if gate:
            # L3 接线由 bringup 完成：cmd_vel_in 重映射到 /cmd_vel（上游），
            # cmd_vel_gated 重映射到 /cmd_vel（下游）。未重映射时本订阅指向
            # 一个无人发布的私有话题 = 安全的空转。
            self._gate_sub = self.create_subscription(
                Twist, "cmd_vel_in", self._on_cmd_vel, 10)
        self.get_logger().info(
            f"slip_watch: r={radius} L={wheel_base_sum} n_sigma={n_sigma} "
            f"sigma_baseline={sigma}（0=未标定，只记数不判定） "
            f"gate={'ON(重映射后生效)' if gate else 'off'}")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        ret = super().on_activate(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret
        self._residuals = Window(WINDOW)
        self._stale_pairs = 0
        self._evaluated = 0
        self._slip = False
        self._gate_scale = 1.0
        self.add_timer(2.0, self._publish_metrics)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self._subs.clear()
        self._metrics_pub = None
        self._gate_pub = None
        self._wheel_vel.clear()
        self._gyro_z = None
        self._gyro_at = None
        return super().on_cleanup(state)

    # ---------------------------------------------------------------- callbacks
    def _on_imu(self, msg: Imu) -> None:
        self._gyro_z = msg.angular_velocity.z
        self._gyro_at = time.monotonic()
        self.mark_seen("imu/data_raw")

    def _on_joint_states(self, msg: JointState) -> None:
        if not self.active:
            return
        for name, vel in zip(msg.name, msg.velocity):
            if WHEEL_STEM in name:
                self._wheel_vel[name] = float(vel)
        self.mark_seen("joint_states")
        self._evaluate()

    def _evaluate(self) -> None:
        if self._detector is None:
            return
        speeds = self._ordered_speeds()
        if speeds is None or self._gyro_z is None:
            return
        if (self._gyro_at is None or
                time.monotonic() - self._gyro_at > self._imu_max_age_s):
            self._stale_pairs += 1
            return
        residual, slip = self._detector.update(speeds, self._gyro_z)
        self._residuals.add(residual * 1e3)   # mrad/s，和其它 metrics 的 ms 习惯一致
        self._evaluated += 1
        self._slip = slip

    def _ordered_speeds(self) -> tuple[float, float, float, float] | None:
        """按 FL FR RL RR 顺序取四轮 URDF 轴速度；缺任一轮返回 None。"""
        out = []
        for stem, _tag in _WHEEL_ORDER:
            matches = [v for name, v in self._wheel_vel.items() if stem in name]
            if not matches:
                return None
            out.append(matches[0])
        return tuple(out)

    # ------------------------------------------------------------------ L3 gate
    def _on_cmd_vel(self, msg: Twist) -> None:
        """打滑嫌疑期间平移分量 ×0.5；恢复期每帧 +1/8 斜坡回 1.0（§7.2 L3）。"""
        if not self.active or not self._gate_enabled:
            return
        if self._slip:
            self._gate_scale = 0.5
        else:
            self._gate_scale = min(1.0, self._gate_scale + 0.125)
        if self._gate_scale >= 1.0:
            self._gate_pub.publish(msg)
            return
        gated = Twist()
        gated.linear.x = msg.linear.x * self._gate_scale
        gated.linear.y = msg.linear.y * self._gate_scale
        gated.angular.z = msg.angular.z * self._gate_scale
        self._gate_pub.publish(gated)

    # ------------------------------------------------------------------ publish
    def _publish_metrics(self) -> None:
        summary = self._residuals.summary()
        values: dict[str, str | float] = {
            "evaluated": self._evaluated,
            "stale_pairs": self._stale_pairs,
            "slip_suspected": int(self._slip),
            "gate_scale": round(self._gate_scale, 3),
        }
        if summary.get("count"):
            values["residual_p50_mrad_s"] = round(float(summary.get("p50", 0.0)), 2)
            values["residual_p95_mrad_s"] = round(float(summary.get("p95", 0.0)), 2)
        status = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name="rak_metrics: slip_watch",
            message=("slip suspected" if self._slip else "watching"),
            hardware_id="slip_watch",
            values=[KeyValue(key=str(k), value=str(v)) for k, v in sorted(values.items())],
        )
        self.report("slip_watch", DiagnosticStatus.OK,
                    "slip suspected" if self._slip else "watching", values)
        if self._metrics_pub is not None:
            self.publish(DiagnosticArray(status=[status]), publisher=self._metrics_pub)


def main(args=None) -> None:
    import rclpy

    rclpy.init(args=args)
    node = SlipWatchNode()
    if (node.configure() != TransitionCallbackReturn.SUCCESS
            or node.activate() != TransitionCallbackReturn.SUCCESS):
        node.get_logger().error("slip_watch failed to reach active")
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
