#!/usr/bin/env python3
"""带硬性墙距护栏的直行/倒车烟测（手工验收用，**唯一允许的手动运动入口**）。

为什么存在这个脚本：2026-11-18 首次烟测，操作员的口头许可"前方安全"被当成了
安全边界，测试脚本直接往 `/mecanum_drive_controller/reference` 灌速度，**没有
任何墙距判断**，三趟把车顶到墙上。这个仓库的运动防护只有两条（`allow_motion`
许可门 + Nav2 的 `collision_monitor`），而手工调试两者都不在链路上。
所以：手工运动必须有本脚本给的护栏，别自己写一个裸 publisher。

三层护栏（任一触发立即零速）：
  1. 起跑前：墙距 < ``--min-start`` 拒绝跑（默认 0.90 m）；
  2. 运行中：墙距 < ``--min-run`` 立即停车（默认 0.45 m）；
  3. 时长到点即停；之后持续 1 s 零速，让 `reference_timeout` 的看门狗也落地。

墙距取**深度相机与雷达两路的最小值**（任一异常都可能漏判，故取 min 而不是 max）。

默认**倒车**（`--speed` 取负）：正前方通常是墙，倒车方向距离单调增大，护栏天然安全，
对里程计的检验等价（位移与航向只是符号相反）。

用法::

    # 默认：-0.08 m/s × 5 s（倒车 0.4 m），起跑墙距须 ≥0.9 m
    python3 scripts/smoke_guarded.py --raw

    # 只写报告、不动车（先确认两路真值都活着）
    python3 scripts/smoke_guarded.py --check-only

自检::

    python3 scripts/smoke_guarded.py --selftest        # 无 ROS、无硬件
"""
from __future__ import annotations

import argparse
import math
import sys


# ----------------------------------------------------------------- 纯逻辑
def clearance(depth, lidar):
    """两路真值取最小（NaN 忽略）。返回 NaN 表示两路都不可用 —— 必须拒绝运动。"""
    vals = [v for v in (depth, lidar) if v == v]
    return min(vals) if vals else float("nan")


def guard_verdict(clearance_m, *, phase, min_start, min_run):
    """跑前/跑中两种判据。返回 (允许?, 原因)。纯函数，便于自检。"""
    if clearance_m != clearance_m:            # NaN
        return False, "两路墙距都不可用"
    if phase == "start":
        return (True, "") if clearance_m >= min_start else (
            False, f"起跑墙距 {clearance_m:.3f} m < {min_start:.2f} m")
    return (True, "") if clearance_m >= min_run else (
        False, f"运行中墙距 {clearance_m:.3f} m < {min_run:.2f} m，急停")


def distance_worthy(spread_pct, spread_limit=6.0):
    """四轮转角极差 >6% = 打滑或转向，这趟的"位移"不能算直行真值。"""
    return spread_pct <= spread_limit


def wheel_distance(deltas, r_eff):
    """四轮位置增量 → 底盘位移（m）。deltas 为四个轮的 Δθ。"""
    if not deltas:
        return float("nan")
    return r_eff * sum(deltas) / len(deltas)


def integrate_rate(samples):
    """[(t, rate)] → ∫rate dt。梯形法，纯 Python（numpy 版本差异不该影响结论）。"""
    if len(samples) < 2:
        return float("nan")
    total = 0.0
    for (t0, r0), (t1, r1) in zip(samples, samples[1:]):
        total += 0.5 * (r0 + r1) * (t1 - t0)
    return total


def selftest() -> int:
    # 护栏：NaN / 起跑不足 / 跑中逼近
    assert clearance(float("nan"), float("nan")) != clearance(1, 1)
    assert clearance(1.2, 0.8) == 0.8, "两路取最小，不是最大"
    assert guard_verdict(1.5, phase="start", min_start=0.9, min_run=0.45)[0]
    assert not guard_verdict(0.5, phase="start", min_start=0.9, min_run=0.45)[0]
    assert not guard_verdict(0.4, phase="run", min_start=0.9, min_run=0.45)[0]
    assert not guard_verdict(float("nan"), phase="start",
                             min_start=0.9, min_run=0.45)[0], "NaN 必须拒绝"
    # 直行判据
    assert distance_worthy(1.5) and not distance_worthy(9.7)
    # 位移换算：2026-11-18 实测 24.71 rad × 0.0309 ≈ 0.763 m
    assert abs(wheel_distance([24.714, 24.626, 24.958, 24.989], 0.0309) - 0.7667) < 1e-3
    # 积分：常量 0.1 rad/s × 2 s = 0.2 rad
    assert abs(integrate_rate([(0, 0.1), (1, 0.1), (2, 0.1)]) - 0.2) < 1e-12
    # 扇区换算的符号：前方 0° 的物体在 laser 系里是 -lidar_yaw 处。
    # 若符号写反，前方 0° 会对应到 +2·lidar_yaw ≈ +75° 的物体上（实测踩过）。
    yaw, forward = -0.6561, 0.0
    laser_angle = forward - yaw            # 0.6561 rad：该物体在 laser 系里的方位
    back = math.atan2(math.sin(laser_angle + yaw), math.cos(laser_angle + yaw))
    assert abs(back - forward) < 1e-9, (back, forward)
    wrong = math.atan2(math.sin(laser_angle - yaw), math.cos(laser_angle - yaw))
    assert abs(math.degrees(wrong)) > 70, "符号写反时会偏 +75°，这条断言就是证据"
    # 空样本必须给 NaN 而不是 0（0 会被当成"没转向"）
    assert integrate_rate([(0, 0.1)]) != integrate_rate([(0, 0.1)])
    print("[selftest] 全部通过（护栏/直行判据/位移换算/积分）。")
    return 0


# ----------------------------------------------------------------- 运行时
def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speed", type=float, default=-0.08,
                    help="m/s；负数=倒车（默认 -0.08，正前方是墙时别用正数）")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--min-start", type=float, default=0.90)
    ap.add_argument("--min-run", type=float, default=0.45)
    ap.add_argument("--angular", type=float, default=0.0,
                    help="原地旋转角速度 rad/s（非零时走 angular.z，不产生平移）")
    ap.add_argument("--invert-angular", action="store_true",
                    help="把 angular.z 取反后再发。2026-11-18 实测：本车麦轮是 X 布局，"
                         "控制器假定 O 布局，正 angular.z 实际**顺时针**转"
                         "（IMU −37° / 激光 −24.6° 双源互证）。前进不受影响。")
    ap.add_argument("--r-eff", type=float, default=0.0309)
    ap.add_argument("--check-only", action="store_true", help="只报两路真值，不动车")
    ap.add_argument("--raw", action="store_true", help="真的驱动底盘（默认只 dry-run）")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    # 迟导入：--selftest 不需要 ROS
    import time

    import numpy as np
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Image, Imu, JointState, LaserScan

    LIDAR_YAW = -0.6561
    WHEELS = ("front_left_wheel", "front_right_wheel",
              "rear_left_wheel", "rear_right_wheel")

    class Probe(Node):
        def __init__(self):
            super().__init__("smoke_guarded")
            self.pos = None
            self.gyro = []
            self.od = {}
            for key, topic in (("ekf", "/odometry/filtered"),
                               ("wheel", "/odom_wheel"), ("rf2o", "/odom")):
                self.create_subscription(
                    Odometry, topic,
                    (lambda k: lambda m: self.od.__setitem__(k, m))(key), 10)
            self.create_subscription(JointState, "/joint_states", self.on_js, 20)
            self.create_subscription(Imu, "/imu/data_raw", self.on_imu, 50)
            self.lidar = self.depth = float("nan")
            self.create_subscription(LaserScan, "/scan", self.on_scan, 5)
            self.create_subscription(Image, "/camera/depth/image_raw", self.on_depth, 5)
            self.pub = self.create_publisher(
                TwistStamped, "/mecanum_drive_controller/reference", 10)
            self.cmd = 0.0
            self.spin = 0.0
            self.invert = False
            self.create_timer(0.05, self.tick)

        def tick(self):
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = self.cmd
            msg.twist.angular.z = -self.spin if self.invert else self.spin
            self.pub.publish(msg)

        def on_js(self, msg):
            d = dict(zip(msg.name, msg.position))
            if all(w in d for w in WHEELS):
                self.pos = np.array([d[w] for w in WHEELS], dtype=float)

        def on_imu(self, msg):
            self.gyro.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                              msg.angular_velocity.z))

        def on_scan(self, msg):
            r = np.array(msg.ranges, dtype=float)
            a = msg.angle_min + np.arange(r.size) * msg.angle_increment
            # base 系方位 = laser 系方位 + lidar_yaw（laser_link 相对 base_link 的偏航）。
            # 2026-11-18 实测标定：深度相机（真前方 2.434 m）当标尺，
            # `a + lidar_yaw` → 2.554 m（差 0.12 m = 雷达前置 0.11 m，符合）；
            # `a - lidar_yaw` → 0.790 m（那是 +75° 方向的物体）。符号写反会让
            # "前方护栏"去看侧墙，实测导致三趟顶墙。
            rel = np.arctan2(np.sin(a + LIDAR_YAW), np.cos(a + LIDAR_YAW))
            m = (np.abs(rel) < math.radians(10)) & np.isfinite(r) & (r > 0.05)
            if m.sum() >= 5:
                self.lidar = float(np.median(r[m]))

        def on_depth(self, msg):
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.width).astype(float) / 1000.0
            h, w = arr.shape
            c = arr[h // 2 - 20:h // 2 + 20, w // 2 - 20:w // 2 + 20]
            c = c[c > 0]
            if c.size >= 100:
                self.depth = float(np.median(c))

    def wait(node, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    rclpy.init()
    node = Probe()
    wait(node, 4.0)
    start = clearance(node.depth, node.lidar)
    print(f"墙距：深度 {node.depth:.3f} / 雷达 {node.lidar:.3f} / 取最小 {start:.3f} m")
    ok, why = guard_verdict(start, phase="start",
                            min_start=args.min_start, min_run=args.min_run)
    if not ok:
        print(f"[拒绝] {why}")
        rclpy.shutdown()
        return 3
    if args.check_only or not args.raw:
        print("[dry-run] 两路真值正常；加 --raw 才真的驱动底盘。")
        rclpy.shutdown()
        return 0

    p0 = None if node.pos is None else node.pos.copy()
    d0, l0 = node.depth, node.lidar
    t0 = node.get_clock().now().nanoseconds * 1e-9
    aborted = ""
    node.cmd = args.speed
    node.spin = args.angular
    node.invert = args.invert_angular
    end = time.time() + args.seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        ok, why = guard_verdict(clearance(node.depth, node.lidar), phase="run",
                                min_start=args.min_start, min_run=args.min_run)
        if not ok:
            aborted = why
            break
    node.cmd = 0.0
    node.spin = 0.0
    wait(node, 1.5)
    t1 = node.get_clock().now().nanoseconds * 1e-9
    p1, d1, l1 = node.pos, node.depth, node.lidar

    if aborted:
        print(f"[护栏停车] {aborted}")
    cmd = args.speed * args.seconds
    print(f"指令 {cmd:+.3f} m（{args.speed:+.2f} m/s × {args.seconds}s）")
    if p0 is not None and p1 is not None:
        dth = p1 - p0
        dist = wheel_distance(list(dth), args.r_eff)
        spread = float(np.ptp(dth)) / max(abs(float(np.mean(dth))), 1e-9) * 100
        verdict = "" if distance_worthy(spread) else "  ← 极差超 6%，这趟不算直行"
        print(f"轮系 Δθ={np.round(dth, 2)} rad  极差 {spread:.1f}%  → {dist:+.3f} m{verdict}")
        seg = [(t, g) for t, g in node.gyro if t0 <= t <= t1]
        yaw = integrate_rate(seg)
        if yaw == yaw and dist == dist and abs(dist) > 1e-3:
            print(f"IMU 积分航向 {math.degrees(yaw):+.2f}°  "
                  f"({math.degrees(yaw)/abs(dist):+.2f} °/m)")
    if args.angular:
        print(f"原地旋转 {args.angular:+.2f} rad/s × {args.seconds}s "
              f"= {math.degrees(args.angular*args.seconds):+.1f}° 指令")
        seg = [(t, g) for t, g in node.gyro if t0 <= t <= t1]
        yaw = integrate_rate(seg)
        if yaw == yaw:
            print(f"  IMU 积分航向 {math.degrees(yaw):+.2f}°（真值）")
        for key in ("odom", "wheel"):
            pass
    print(f"墙距 {start:.3f} -> {clearance(d1, l1):.3f} m "
          f"（深度 {d0:.3f}->{d1:.3f}，雷达 {l0:.3f}->{l1:.3f}）")
    for key in ("ekf", "wheel", "rf2o"):
        msg = node.od.get(key)
        if msg is None:
            continue
        p = msg.pose.pose.position
        yaw = math.atan2(2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z
                                + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
                         1.0 - 2.0 * (msg.pose.pose.orientation.y ** 2
                                      + msg.pose.pose.orientation.z ** 2))
        print(f"  {key:6s} ({p.x:+.3f},{p.y:+.3f})  yaw {math.degrees(yaw):+.2f}°")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
