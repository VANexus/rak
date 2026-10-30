# -*- coding: utf-8 -*-
"""wake_doa_viz 纯函数测试:方位→扇形几何、轨迹色带、环形轨迹缓冲。

不依赖 ROS 图与真板子:节点类只在 main() 里 spin,几何/缓存逻辑可以
在没有 rviz、没有麦克风的环境里直接断言(与 test_vtn_protocol 同一约定)。
运行:python3 -m pytest test_wake_doa_viz.py(colcon test 同样收集)。
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rak_speech.wake_doa_viz import (  # noqa: E402
    TrailBuffer,
    fan_points,
    trail_color,
)


# -- 方位 → 扇形几何(TRIANGLE_LIST:每 3 个点一个三角形) ---------------------
def test_fan_points_vertex_layout():
    segments = 4
    pts = fan_points(0.0, 1.0, 15.0, segments)
    assert len(pts) == 3 * segments
    for i in range(segments):
        assert pts[3 * i] == (0.0, 0.0)  # 每个三角形都以圆心为顶点


def test_fan_points_direction_and_width():
    angle = 0.0
    half = 15.0
    pts = fan_points(angle, 1.0, half, 8)
    tips = [p for i, p in enumerate(pts) if i % 3 != 0]
    lo = math.cos(math.radians(half))
    for x, y in tips:
        assert x > 0.0                      # 0° 扇区朝向 +x
        assert math.isclose(math.hypot(x, y), 1.0, abs_tol=1e-9)  # 顶点在半径圆上
    assert min(x for x, _ in tips) >= lo - 1e-9  # 张角不超过 ±half
    # 90° 方向朝 +y
    tips90 = [p for i, p in enumerate(fan_points(90.0, 1.0, half, 8)) if i % 3 != 0]
    assert min(y for _, y in tips90) > 0.0


def test_fan_points_wraps_365_deg():
    """板子固件会报 0-365 度;365 必须与 5 等价,不能画反。"""
    a = fan_points(365.0, 1.0, 15.0, 8)
    b = fan_points(5.0, 1.0, 15.0, 8)
    for (ax, ay), (bx, by) in zip(a, b):
        assert math.isclose(ax, bx, abs_tol=1e-9)
        assert math.isclose(ay, by, abs_tol=1e-9)


# -- 轨迹色带 ----------------------------------------------------------------
def test_trail_color_ramp_monotonic():
    old, new = trail_color(0.0), trail_color(1.0)
    assert old[3] < new[3]  # 越新越不透明
    assert new[1] > old[1]  # 越新越亮(青色主分量)
    for channel in trail_color(0.5):
        assert 0.0 <= channel <= 1.0


def test_trail_color_clamps_out_of_range():
    """环形缓冲淘汰/重排时 frac 可能瞬时越界,必须钳位而不是画出花。"""
    assert trail_color(-3.0) == trail_color(0.0)
    assert trail_color(9.0) == trail_color(1.0)


# -- 环形轨迹缓冲 ------------------------------------------------------------
def test_trail_buffer_fifo_and_bound():
    trail = TrailBuffer(3)
    for angle in (10.0, 20.0, 30.0, 40.0):
        trail.push(angle)
    assert len(trail) == 3
    assert list(trail) == [20.0, 30.0, 40.0]  # 淘汰最旧,顺序 oldest -> newest


def test_trail_buffer_wraps_angle():
    trail = TrailBuffer(2)
    trail.push(365.0)
    trail.push(-90.0)
    assert list(trail) == [5.0, 270.0]  # 统一折到 0-360 再入环


def test_trail_buffer_capacity_never_zero():
    trail = TrailBuffer(0)
    trail.push(45.0)
    assert trail.capacity == 1
    assert len(trail) == 1
