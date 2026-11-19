"""巡路闭环验收：记录横向误差随时间的走势，判定"收敛还是发散"。

为什么需要它：视觉链的符号链（trace -> follow_command -> /cmd_vel ->
cmd_vel_omni 取反 -> mecanum 控制器 -> 车轮）从没在真机上闭环验证过。一旦
任何一个环节反号，车会**越修越偏**，而单看一帧 cmd_vel 是看不出来的——必须看
|e| 随时间的趋势。

    ros2 run 无此入口；直接 python3 scripts/road_closed_loop_check.py --seconds 6

输出每 0.2 s 一行，最后给判定：
    CONVERGING   |e| 显著下降（符号链正确）
    DIVERGING    |e| 显著上升（符号链反了，立刻停车）
    INCONCLUSIVE 变化太小或数据不足
"""

from __future__ import annotations

import argparse
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from rak_road_perception.road_follow_math import (
    CORNER_DEFAULTS,
    FOLLOW_DEFAULTS,
    LANE_CENTER_BIAS,
    TRACE_DEFAULTS,
    trace_centerline,
)


def lateral_error(points, params):
    """Same lookahead the controller uses, so this measures what it reacts to."""
    if not points:
        return None
    n_look = max(1, int(params["lookahead_frac"] / max(params["row_step_frac"], 1e-6)))
    i = min(n_look, len(points) - 1)
    return float(points[i][1] - params.get("lane_center_bias", 0.0))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--period", type=float, default=0.2)
    args = ap.parse_args()

    params = {**TRACE_DEFAULTS, **FOLLOW_DEFAULTS, **CORNER_DEFAULTS}
    params["lane_center_bias"] = LANE_CENTER_BIAS

    rclpy.init()
    node = Node("road_closed_loop_check")
    state = {"mask": None, "cmd": (0.0, 0.0, 0.0), "t": 0.0}

    def on_mask(m):
        state["mask"] = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width).copy()
        state["t"] = time.monotonic()

    def on_cmd(m):
        state["cmd"] = (m.linear.x, m.linear.y, m.angular.z)

    node.create_subscription(Image, "/road_perception/road_mask", on_mask,
                             qos_profile_sensor_data)
    node.create_subscription(Twist, "/cmd_vel", on_cmd, 10)

    samples = []
    t0 = time.monotonic()
    print(f"{'t':>5} {'e':>8} {'bottom':>8} {'vx':>6} {'vy':>7} {'wz':>7}")
    last = 0.0
    while time.monotonic() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic() - t0
        if state["mask"] is None or now - last < args.period:
            continue
        last = now
        pts, _spin, has = trace_centerline(state["mask"], params)
        if not has or not pts:
            print(f"{now:5.1f}  (no trace)")
            continue
        e = lateral_error(pts, params)
        vx, vy, wz = state["cmd"]
        samples.append((now, e))
        print(f"{now:5.1f} {e:+8.3f} {pts[0][1]:+8.3f} {vx:6.3f} {vy:+7.3f} {wz:+7.3f}")

    node.destroy_node()
    rclpy.shutdown()

    if len(samples) < 5:
        print("\nINCONCLUSIVE: 采样不足")
        return 2
    t = np.array([s[0] for s in samples])
    e = np.array([s[1] for s in samples])
    slope = float(np.polyfit(t, np.abs(e), 1)[0])       # d|e|/dt
    first = float(np.mean(np.abs(e[:3])))
    lastv = float(np.mean(np.abs(e[-3:])))
    print(f"\n|e| 首段 {first:.3f} -> 末段 {lastv:.3f}   d|e|/dt = {slope:+.4f} /s")
    if first > 0.02 and lastv < first * 0.6:
        print("CONVERGING   符号链正确，横向误差在收敛")
        return 0
    if lastv > first * 1.4 + 0.01:
        print("DIVERGING    符号链反了 —— 立刻停车，检查 wz 取反")
        return 1
    print("INCONCLUSIVE 变化不明显（可能本来就接近正中，或速度太低）")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
