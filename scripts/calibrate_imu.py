#!/usr/bin/env python3
"""把"协方差该填多少"从拍脑袋变成可复现的测量（ADR-0023 §5）。

两种模式，对应 ADR-0002 的两类输入：

**静态 Allan（陀螺/加计噪声）**::

    # 静止放置 30-60 min，录 /imu/data_raw
    ros2 bag record -o imu_static /imu/data_raw
    python3 scripts/calibrate_imu.py static --bag imu_static --axis gz

    # 或者先导出 CSV（t,gx,gy,gz,ax,ay,az，t 单位秒）
    python3 scripts/calibrate_imu.py static --csv imu.csv --axis gz --rate 1000

输出：角度随机游走 N（deg/√h）、零偏不稳定性 B（deg/h）、1 s 平均的方差
（= EKF 里 `imu0_covariance[vyaw]` 的量纲一致值），并打印可直接粘贴的数值块。

**直行 2 m 回归（航向漂移）**::

    python3 scripts/calibrate_imu.py straight --csv run.csv --axis yaw --distance 2.0

输入列：`t` 秒、`yaw` 弧度（来自 `/odometry/filtered` 或 IMU 积分）。输出：
2 m 距离上的航向误差与横向偏移，用来判断"融合后到底能不能走直"。

依赖：numpy 可选；没有它则用纯 Python（慢但结论一致，样本会先做 10× 抽稀）。
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import statistics
import sys

try:  # numpy 只是加速，不是依赖
    import numpy as np
except ImportError:  # pragma: no cover
    np = None


def read_csv(path: Path) -> dict[str, list[float]]:
    with path.open(encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        columns: dict[str, list[float]] = {name: [] for name in (reader.fieldnames or [])}
        for row in reader:
            for name in columns:
                try:
                    columns[name].append(float(row[name]))
                except (TypeError, ValueError):
                    columns[name].append(float("nan"))
    missing = [name for name, values in columns.items() if not values]
    if missing:
        raise SystemExit(f"CSV 缺列或全空: {missing}")
    return columns


def read_bag(path: Path, axis: str) -> dict[str, list[float]]:
    """从 rosbag2 读 /imu/data_raw。需要 rosbag2_py（只在装了 ROS 的机器上可用）。"""
    try:
        from rclpy.serialization import deserialize_message
        import rosbag2_py
        from sensor_msgs.msg import Imu
    except ImportError as exc:  # pragma: no cover
        raise SystemExit(f"读 bag 需要 ROS 环境（{exc}）；可先用 ros2 bag 导出 CSV") from exc

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                    output_serialization_format="cdr"),
    )
    columns = {"t": [], "gx": [], "gy": [], "gz": [], "ax": [], "ay": [], "az": []}
    while reader.has_next():
        _topic, data, stamp = reader.read_next()
        msg = deserialize_message(data, Imu)
        columns["t"].append(stamp / 1e9)
        columns["gx"].append(msg.angular_velocity.x)
        columns["gy"].append(msg.angular_velocity.y)
        columns["gz"].append(msg.angular_velocity.z)
        columns["ax"].append(msg.linear_acceleration.x)
        columns["ay"].append(msg.linear_acceleration.y)
        columns["az"].append(msg.linear_acceleration.z)
    if axis not in columns:
        raise SystemExit(f"axis 必须是 {sorted(columns)} 之一")
    return columns


def allan_deviation(values: list[float], rate: float) -> list[tuple[float, float]]:
    """重叠式 Allan 偏差，返回 [(cluster_time_s, adev)]（σ 用**度**或 m/s² 原单位）。"""
    tau0 = 1.0 / rate
    n = len(values)
    points: list[tuple[float, float]] = []
    cluster = 1
    while cluster <= n // 4:
        count = n // cluster
        means = []
        for index in range(count):
            chunk = values[index * cluster:(index + 1) * cluster]
            means.append(sum(chunk) / cluster)
        diffs = [(means[i + 1] - means[i]) ** 2 for i in range(len(means) - 1)]
        if diffs:
            variance = sum(diffs) / (2.0 * len(diffs))
            points.append((cluster * tau0, math.sqrt(variance)))
        cluster *= 2
    return points


def fit_slope(points: list[tuple[float, float]], start_decade: float = 0.02) -> float:
    """在 log-log 上拟合 τ 较小的一段，斜率 -1/2 区间的截距即角度随机游走。"""
    sample = [(math.log10(t), math.log10(s)) for t, s in points if s > 0 and t >= start_decade]
    if len(sample) < 3:
        return float("nan")
    xs = [x for x, _ in sample]
    ys = [y for _, y in sample]
    mean_x, mean_y = statistics.fmean(xs), statistics.fmean(ys)
    denominator = sum((x - mean_x) ** 2 for x in xs)
    if denominator == 0:
        return float("nan")
    return sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / denominator


def static_report(columns: dict[str, list[float]], axis: str, rate: float,
                  unit: str) -> None:
    values = [v for v in columns[axis] if v == v]
    if len(values) < int(rate):
        raise SystemExit(f"{axis} 有效样本只有 {len(values)}，至少需要 1 s 数据")
    duration_h = len(values) / rate / 3600.0
    mean = statistics.fmean(values)
    stdev = statistics.pstdev(values)

    adev = allan_deviation(values, rate)
    if not adev:
        raise SystemExit("样本太短，无法算 Allan 偏差（建议 ≥ 600 s）")

    # τ=1 s 的 Allan 偏差 ≈ 1 s 平均的 1σ；直接与 EKF 的方差量纲对应。
    one_second = min(adev, key=lambda pair: abs(pair[0] - 1.0))[1]
    slope = fit_slope(adev)
    # ARW: 斜率 -1/2 段在 τ=1 处的值 × 1（单位/√s → 单位/√h 需 ×60）
    arw_per_root_h = one_second * 60.0
    bias_instability = min(adev, key=lambda pair: pair[1])[1]

    print(f"轴 {axis}：{len(values)} 样本 / {duration_h * 60:.1f} min @ {rate:g} Hz")
    print(f"  均值 {mean:.6g}，标准差 {stdev:.6g} {unit}")
    print(f"  Allan: τ=1s σ={one_second:.6g} {unit}；最小 σ={bias_instability:.6g}（零偏不稳定性）")
    print(f"  角度随机游走 N ≈ {arw_per_root_h:.4g} {unit}/√h（由 τ=1s 点 ×60 估计）")
    print(f"  log-log 斜率 ≈ {slope:.2f}（理想 -0.5 段才是白噪声；偏离说明有有色噪声）")
    print()
    print("可直接粘贴进 src/rak_bringup/config/ekf.yaml（把 TODO(measured) 换成实际值）：")
    print(f"  # measured {duration_h * 60:.0f} min static, axis={axis}, rate={rate:g}Hz")
    print(f"  # imu0_covariance[vyaw] = {one_second ** 2:.6g}   # ({one_second:.4g} {unit})^2")
    print("  # basis: measured（scripts/calibrate_imu.py static）")


def straight_report(columns: dict[str, list[float]], axis: str, distance: float) -> None:
    if "t" not in columns or axis not in columns:
        raise SystemExit(f"需要列 t 与 {axis}")
    times = columns["t"]
    yaw = [v for v in columns[axis]]
    if len(times) < 10:
        raise SystemExit("样本太少")
    total = times[-1] - times[0]
    if total <= 0:
        raise SystemExit("时间列不是单调的")
    # 以首尾 10% 的平均航向差作为漂移（去掉端点噪声）
    edge = max(1, len(yaw) // 10)
    yaw_start = statistics.fmean(yaw[:edge])
    yaw_end = statistics.fmean(yaw[-edge:])
    drift_deg = math.degrees(yaw_end - yaw_start)
    speed = distance / total
    lateral = distance * math.sin(math.radians(drift_deg))
    print(f"直行 {distance:g} m：用时 {total:.2f} s（均速 {speed:.2f} m/s）")
    print(f"  航向漂移 {drift_deg:+.2f}°  → 终端横向偏移 {lateral:+.3f} m")
    print(f"  折合 {abs(drift_deg) / max(distance, 1e-9):.3f} °/m")
    if abs(lateral) > 0.05:
        print("  ⚠ 横向偏移 > 5 cm：先查轮径/轮距标定与 gyro 零偏，再谈融合参数")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="mode", required=True)

    static = sub.add_parser("static", help="Allan 方差（静止段）")
    static.add_argument("--csv", type=Path, default=None)
    static.add_argument("--bag", type=Path, default=None)
    static.add_argument("--axis", default="gz", choices=["gx", "gy", "gz", "ax", "ay", "az"])
    static.add_argument("--rate", type=float, default=1000.0)
    static.add_argument("--unit", default="", help="打印用单位（默认按轴自动: rad/s 或 m/s^2）")

    straight = sub.add_parser("straight", help="直行回归（航向漂移）")
    straight.add_argument("--csv", type=Path, required=True)
    straight.add_argument("--axis", default="yaw")
    straight.add_argument("--distance", type=float, default=2.0)

    args = parser.parse_args()
    if args.mode == "static":
        if not args.csv and not args.bag:
            raise SystemExit("static 需要 --csv 或 --bag")
        columns = read_csv(args.csv) if args.csv else read_bag(args.bag, args.axis)
        unit = args.unit or ("rad/s" if args.axis.startswith("g") else "m/s^2")
        static_report(columns, args.axis, args.rate, unit)
        return 0
    columns = read_csv(args.csv)
    straight_report(columns, args.axis, args.distance)
    return 0


if __name__ == "__main__":
    sys.exit(main())
