"""滑窗与分位数的离线单测（不需要 ROS）。"""

from __future__ import annotations

import math

import pytest

from rak_metrics.stats import FlatnessTracker, RateMeter, Window, percentile


def test_percentile_matches_linear_interpolation():
    values = [1.0, 2.0, 3.0, 4.0]
    assert percentile(values, 0.0) == 1.0
    assert percentile(values, 0.5) == 2.5
    assert percentile(values, 1.0) == 4.0
    # 单点与空集不抛，空集给 nan（调用方据此判"还没数据"）
    assert percentile([7.0], 0.95) == 7.0
    assert math.isnan(percentile([], 0.5))


def test_window_keeps_only_the_last_n_samples():
    window = Window(size=3)
    for value in (1.0, 2.0, 3.0, 4.0):
        window.add(value)
    summary = window.summary()
    assert summary["count"] == 3
    assert summary["max"] == 4.0
    assert summary["p50"] == 3.0


def test_window_counts_long_gaps_against_the_nominal_period():
    window = Window(size=10, nominal_period_s=0.1)
    for stamp in (0.0, 0.1, 0.2, 0.9):   # 最后一跳 700 ms >> 2.5 × 100 ms
        window.observe_gap(stamp)
    assert window.gap_events == 1


def test_window_without_nominal_period_never_reports_gaps():
    """没有标称周期就不许猜丢帧 —— 宁可没有这个指标，也不要有假指标。"""
    window = Window(size=10)
    window.observe_gap(0.0)
    window.observe_gap(10.0)
    assert window.gap_events == 0


def test_rate_meter_uses_a_sliding_window():
    meter = RateMeter(window_s=1.0)
    for index in range(11):
        rate = meter.tick(index * 0.1)
    assert rate == pytest.approx(10.0, rel=0.2)


def test_nan_samples_are_ignored():
    """now - stamp 在时钟回跳时会是负数或 nan，不能污染分位数。"""
    window = Window(size=4)
    window.add(float("nan"))
    window.add(5.0)
    assert window.summary()["count"] == 1


# ---- FlatnessTracker（低速质量指标，chassis-control-design §6） ----

def test_flatness_counts_exact_repeats_and_jumps():
    tracker = FlatnessTracker(size=10)
    for value in (1.0, 1.0, 1.0, 1.4, 1.4):
        tracker.observe(value)
    s = tracker.summary()
    # 4 个相邻对里 3 对完全相等（M 法台阶化的指纹）
    assert s["flat_ratio"] == pytest.approx(3 / 4)
    assert s["flat_max_jump"] == pytest.approx(0.4)
    assert s["flat_jump_events"] == 1


def test_flatness_moving_ratio_ignores_standing_still():
    """停着不动样本全是平坦的，flat_ratio 会虚高——moving 档只看有一端 |v|>eps 的样本。"""
    tracker = FlatnessTracker(size=10, moving_eps=0.05)
    for value in (0.0, 0.0, 0.0, 0.0):   # 静止：不该计入 moving 档
        tracker.observe(value)
    for value in (0.2, 0.2, 0.5, 0.5):   # 低速爬行：4 对里 2 对平坦
        tracker.observe(value)
    s = tracker.summary()
    # moving 档 = (0→0.2 非平坦), (0.2→0.2 平坦), (0.2→0.5 非平坦), (0.5→0.5 平坦)
    assert s["flat_ratio_moving"] == pytest.approx(0.5)
    assert s["flat_ratio"] > s["flat_ratio_moving"]


def test_flatness_sliding_window_drops_old_samples():
    tracker = FlatnessTracker(size=4)
    for value in (1.0, 2.0, 3.0, 4.0, 5.0, 5.0):
        tracker.observe(value)
    s = tracker.summary()
    assert s["flat_count"] == 4
    assert s["flat_ratio"] == pytest.approx(1 / 4)
    assert s["flat_max_jump"] == pytest.approx(1.0)


def test_flatness_ignores_nan_and_first_sample():
    tracker = FlatnessTracker(size=10)
    tracker.observe(float("nan"))
    tracker.observe(1.0)
    s = tracker.summary()
    assert s["flat_count"] == 0
    assert "flat_ratio" not in s
