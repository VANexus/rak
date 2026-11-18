"""SlipDetector 的离线单测（不需要 ROS）。"""

from __future__ import annotations

import pytest

from rak_metrics.slip import SlipDetector


def _detector(sigma: float = 0.1, **kw) -> SlipDetector:
    return SlipDetector(sigma_baseline=sigma, **kw)


def test_kinematic_yaw_rate_pure_translation_is_zero():
    d = _detector()
    # 纯平移在默认权重 (-1,1,-1,1) 下四项相消 → 0
    assert d.kinematic_yaw_rate([5.0, 5.0, 5.0, 5.0]) == pytest.approx(0.0)


def test_kinematic_yaw_rate_rotation_sign():
    d = _detector(wheels_radius=0.0275, wheel_base_sum=0.26)
    # 权重下 Σ w·ω = 20 > 0（逆时针 yaw 的正方向样本）；显式传几何，不跟默认值走
    w = d.kinematic_yaw_rate([-5.0, 5.0, -5.0, 5.0])
    assert w == pytest.approx(0.0275 * 20.0 / (4 * 0.26))
    assert w > 0.0


def test_default_radius_is_measured_value():
    """默认 wheels_radius 必须是 2026-11-18 实测的有效滚动半径（滚子外缘）。"""
    d = SlipDetector()
    assert d._r == pytest.approx(0.0309)


def test_residual_is_kinematic_minus_gyro():
    d = _detector()
    residual, _ = d.update([5.0, 5.0, 5.0, 5.0], gyro_z=-0.7)
    assert residual == pytest.approx(0.0 - (-0.7))


def test_suspicion_needs_persistence():
    """单次越限不算打滑（噪声/冲击旁瓣），持续 ≥hold 才成立。"""
    d = _detector(sigma=0.1, hold_cycles=3)
    # kinematic=0, gyro=-0.5 → residual=0.5 > 3σ=0.3
    _, slip = d.update([0.0, 0.0, 0.0, 0.0], gyro_z=-0.5)
    assert not slip
    _, slip = d.update([0.0, 0.0, 0.0, 0.0], gyro_z=-0.5)
    assert not slip
    _, slip = d.update([0.0, 0.0, 0.0, 0.0], gyro_z=-0.5)
    assert slip
    # 一次正常 → 立即解除（连续性计数归零）
    _, slip = d.update([0.0, 0.0, 0.0, 0.0], gyro_z=0.0)
    assert not slip
    assert d.run_length == 0


def test_uncalibrated_sigma_never_flags():
    """σ 未标定（0）= 只记数不判定——阈值没有依据就不产生打滑结论（AGENTS §4）。"""
    d = SlipDetector(sigma_baseline=0.0)
    for _ in range(20):
        residual, slip = d.update([0.0, 0.0, 0.0, 0.0], gyro_z=9.0)
    assert not slip
    assert d.threshold == float("inf")
    assert residual == pytest.approx(-9.0)


def test_global_sign_flip_does_not_change_suspicion():
    """全局符号翻转只翻残差符号，|residual| 判据不受影响——角位未核对期的兜底。"""
    a = _detector(sigma=0.1)
    b = SlipDetector(
        sigma_baseline=0.1, wheel_weights=(1.0, -1.0, 1.0, -1.0))
    wheels = [1.0, 2.0, 3.0, 4.0]
    ra, _ = a.update(wheels, gyro_z=0.0)
    rb, _ = b.update(wheels, gyro_z=0.0)
    assert ra == pytest.approx(-rb)
