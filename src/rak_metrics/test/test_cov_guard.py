"""cov_guard（L2 协方差膨胀）与 σ 基线回填的纯逻辑单测。

全部不导入 rclpy / nav_msgs —— 不需要 ROS 环境即可运行：
    PYTHONPATH=src/rak_metrics:$PYTHONPATH python3 -m pytest src/rak_metrics/test/test_cov_guard.py
"""

from __future__ import annotations

import json

import pytest

from rak_metrics.cov_guard import (
    POSE_DIAGONAL,
    TWIST_DIAGONAL,
    inflate_covariances,
    load_baseline,
    next_multiplier,
    suggest_sigma,
)

ZEROS = [0.0] * 36


# ------------------------------------------------------------------ 倍数状态机
def test_slip_jumps_to_factor_immediately():
    """升程不做斜坡：保护宁可早到（斜坡只用于恢复）。"""
    assert next_multiplier(1.0, slip=True, factor=10.0, ramp_step=4.0) == 10.0


def test_recovery_instant_when_ramp_disabled():
    assert next_multiplier(10.0, slip=False, factor=10.0, ramp_step=1.0) == 1.0


def test_recovery_ramp_is_geometric_and_never_below_one():
    m = 10.0
    seen = [m]
    for _ in range(10):
        m = next_multiplier(m, slip=False, factor=10.0, ramp_step=4.0)
        seen.append(m)
    assert seen[1] == pytest.approx(2.5)
    assert seen[2] == pytest.approx(1.0)
    assert min(seen) == 1.0          # 绝不低于 1.0（1.0 = 原样转发）
    assert all(a >= b for a, b in zip(seen, seen[1:]))


def test_no_slip_from_one_stays_one():
    assert next_multiplier(1.0, slip=False, factor=10.0, ramp_step=4.0) == 1.0


def test_factor_below_one_clamped():
    assert next_multiplier(1.0, slip=True, factor=0.5, ramp_step=4.0) == 1.0


# ------------------------------------------------------------------ 协方差膨胀
def test_inflate_multiplies_only_xy_and_vx_vy_diagonals():
    pose = [1.0 if i in POSE_DIAGONAL else 2.0 for i in range(36)]
    twist = [1.0 if i in TWIST_DIAGONAL else 2.0 for i in range(36)]
    out_pose, out_twist = inflate_covariances(pose, twist, 10.0)
    assert out_pose[POSE_DIAGONAL[0]] == pytest.approx(10.0)
    assert out_pose[POSE_DIAGONAL[1]] == pytest.approx(10.0)
    assert out_twist[TWIST_DIAGONAL[0]] == pytest.approx(10.0)
    assert out_twist[TWIST_DIAGONAL[1]] == pytest.approx(10.0)
    # 非对角与未膨胀项原样保留
    assert sum(v for i, v in enumerate(out_pose) if i not in POSE_DIAGONAL) == 34 * 2.0
    assert sum(v for i, v in enumerate(out_twist) if i not in TWIST_DIAGONAL) == 34 * 2.0


def test_inflate_passthrough_when_factor_one():
    pose, twist = inflate_covariances(ZEROS, ZEROS, 1.0)
    assert pose == ZEROS and twist == ZEROS
    pose, twist = inflate_covariances(ZEROS, ZEROS, 0.5)
    assert pose == ZEROS and twist == ZEROS


def test_inflate_rejects_wrong_length():
    with pytest.raises(ValueError):
        inflate_covariances([0.0] * 35, ZEROS, 2.0)
    with pytest.raises(ValueError):
        inflate_covariances(ZEROS, [0.0] * 35, 2.0)


# ------------------------------------------------------------------ σ 基线回填
def test_suggest_sigma_is_p95_of_abs():
    residuals = [-0.5, 0.5, -0.4, 0.4, -0.3, 0.3, -0.2, 0.2, -0.1, 0.1]
    # 与 numpy linear 插值一致：|residual| 的 p95（stats.percentile 同一实现）
    assert suggest_sigma(residuals) == pytest.approx(0.5)


def test_suggest_sigma_ignores_nonfinite_and_empty():
    assert suggest_sigma([]) is None
    assert suggest_sigma([float("nan"), float("inf"), float("-inf")]) is None
    assert suggest_sigma([float("nan"), 0.5, float("inf")]) == pytest.approx(0.5)


def test_load_baseline_csv_skips_comments_and_headers(tmp_path):
    path = tmp_path / "baseline.csv"
    path.write_text("# 2026-11-18 首日基线\nresidual_mrad_s\n1.5\n0.25\nbad line\n2.0\n")
    assert load_baseline(str(path)) == pytest.approx([1.5, 0.25, 2.0])


def test_load_baseline_json_array_and_dict(tmp_path):
    arr = tmp_path / "arr.json"
    arr.write_text(json.dumps([1.0, 2.0]))
    obj = tmp_path / "obj.json"
    obj.write_text(json.dumps({"residuals": [3.0]}))
    assert load_baseline(str(arr)) == [1.0, 2.0]
    assert load_baseline(str(obj)) == [3.0]


def test_load_baseline_missing_file_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_baseline(str(tmp_path / "nope.csv"))


def test_sigma_tool_end_to_end(tmp_path, capsys):
    """slip_sigma CLI 的核心路径：序列 → 建议 σ → n_sigma 阈值（可执行回填路径自检）。"""
    from rak_metrics.cov_guard import main as sigma_main
    path = tmp_path / "baseline.json"
    path.write_text(json.dumps([0.1] * 9 + [0.5]))  # 健康基线 + 一个离群
    sigma_main([str(path), "--n-sigma", "3.0"])
    out = json.loads(capsys.readouterr().out)
    assert out["sigma"] == pytest.approx(0.32)
    assert out["threshold"] == pytest.approx(3.0 * out["sigma"], rel=1e-6)
