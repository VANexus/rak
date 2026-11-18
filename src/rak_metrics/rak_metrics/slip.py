"""抗打滑检测的纯逻辑（不依赖 ROS，可离线单测）。

设计出处：`rak/docs/firmware-capability-spec.md` §7.2（三级响应的 L1 检测）。
权威源 = Jetson LSM6DSR 的 gyro-z；固件 `imu_evt.SLIP`（0x02）只作旁证，本模块不消费。

残差定义：``residual = ω_kinematic(四轮速 + 几何) − ω_gyro_z``。

麦轮逆运动学给出车身 yaw 角速度：

    ω_kin = r / (4·L) · Σᵢ wᵢ · ωᵢ        (L = 轴距半和+轮距半和 = lx+ly)

wᵢ 是每轮 URDF 轴速度对逆时针 yaw 的贡献符号。**符号未现场核对**
（CAN-BRINGUP-2026-10-30 §7.1 的角位回填未完成）：全局符号翻转不影响本模块
——嫌疑判据用的是 |residual|；只有"个别轮权重错"才会破坏检测，那要靠
落地期的标定流程（首日基线 → σ 回填）暴露。

阈值依据（AGENTS §4 事实分级）：``sigma_baseline`` 是 placeholder，
由 rak_metrics 首日基线（p50/p95 分布）标定后回填；baseline=0 时检测
只记数不判嫌疑（发布指标，不产生打滑判定）。回填工具：``slip_sigma
<baseline.csv|json>``（见 cov_guard.suggest_sigma）吃一段记录下来的
残差序列，建议 σ = p95(|residual|)。
"""

from __future__ import annotations


class SlipDetector:
    """打滑嫌疑状态机：|residual| > nσ 持续 ≥ hold 个采样周期 → 嫌疑成立。"""

    def __init__(
        self,
        # 物理权威 wheel_config.h（WHEEL_RADIUS_UM / WHEEL_LEVER_UM）；此处仅为指标默认值。
        # 2026-11-18 实测回填：0.0275 是轮盘本体半径，mecanum 实际滚在滚子外缘，
        # 有效滚动半径 0.0309（与 chassis_controllers.yaml mecanum kinematics 同步）。
        wheels_radius: float = 0.0309,
        wheel_base_sum: float = 0.26,
        # yaw 贡献权重（左 -1 / 右 +1），与固件 wheel_config.h 的
        # WHEEL_YAW_DIR_LIST 同一物理事实；与“前进方向表”无关。
        wheel_weights: tuple[float, float, float, float] = (-1.0, 1.0, -1.0, 1.0),
        n_sigma: float = 3.0,
        sigma_baseline: float = 0.0,
        hold_cycles: int = 5,
    ) -> None:
        self._r = float(wheels_radius)
        self._l = float(wheel_base_sum)
        self._w = tuple(float(x) for x in wheel_weights)
        self._n_sigma = float(n_sigma)
        self._sigma = float(sigma_baseline)
        self._hold = max(1, int(hold_cycles))
        self._run = 0

    @property
    def threshold(self) -> float:
        """当前嫌疑阈值（rad/s）；σ 未标定时返回 inf（只记数不判定）。"""
        if self._sigma <= 0.0:
            return float("inf")
        return self._n_sigma * self._sigma

    def kinematic_yaw_rate(self, wheel_speeds) -> float:
        """四轮 URDF 轴速度 (rad/s) → 车身逆时针 yaw 角速度 (rad/s)。"""
        if len(wheel_speeds) != 4:
            raise ValueError("需要恰好四轮速度")
        acc = 0.0
        for weight, speed in zip(self._w, wheel_speeds):
            acc += weight * float(speed)
        return self._r * acc / (4.0 * self._l)

    def update(self, wheel_speeds, gyro_z: float) -> tuple[float, bool]:
        """喂一个同步采样对，返回 (residual, slip_suspected)。"""
        residual = self.kinematic_yaw_rate(wheel_speeds) - float(gyro_z)
        if abs(residual) > self.threshold:
            self._run += 1
        else:
            self._run = 0
        return residual, self._run >= self._hold

    @property
    def run_length(self) -> int:
        return self._run
