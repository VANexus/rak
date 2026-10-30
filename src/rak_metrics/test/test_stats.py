"""滑窗与分位数的离线单测（不需要 ROS）。"""

from __future__ import annotations

import math

import pytest

from rak_metrics.stats import RateMeter, Window, percentile


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
