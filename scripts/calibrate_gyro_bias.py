#!/usr/bin/env python3
"""标定陀螺零偏（`gyro_bias`），一条命令出可直接粘贴的值。

为什么需要它：`i2c_imu` 的 `gyro_bias` 参数是**唯一**抑制航向漂移的来源，而它随
**温度**漂。2026-11-18 实测：同一块板在另一次热状态下标定的值
`0.001186,-0.007734,-0.003310`，在跑完一轮热机后再测，Z 轴残差 **−0.00119 rad/s
（−4.1°/min）**——EKF 的航向就是积这个残差，10 分钟就歪 40°。
所以：**每次要跑长时先在这个热状态下重标一次**，别指望一个值用一周。

原理：节点发布值 = map(raw) − bias，所以让发布值归零所需的 bias
= 当前发布值均值 + 当前 bias（即 map(raw) 的均值）。

用法::

    ros2 run rak_imu_driver i2c_imu &            # 必须先起节点并 activate
    python3 scripts/calibrate_gyro_bias.py       # 默认 90 s
    python3 scripts/calibrate_gyro_bias.py --seconds 180 --apply

`--apply` 会用新值重启……不，它只把新值写进 `sensors.launch.py` 的那一行；
重启节点仍是你自己的事（本工具不杀进程、不起进程）。

自检::

    python3 scripts/calibrate_gyro_bias.py --selftest     # 无 ROS、无硬件
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SENSORS = REPO / "src" / "rak_bringup" / "launch" / "sensors.launch.py"


def parse_bias(text: str) -> tuple[float, float, float]:
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 3:
        raise ValueError(f"gyro_bias 需要 3 个数，得到 {text!r}")
    return tuple(float(p) for p in parts)  # type: ignore[return-value]


def current_bias() -> tuple[float, float, float]:
    """从 sensors.launch.py 读出现用值（它是唯一权威，不在别处重复）。"""
    m = re.search(r'"gyro_bias":\s*"([^"]+)"', SENSORS.read_text(encoding="utf-8"))
    if not m:
        raise SystemExit(f"{SENSORS} 里找不到 \"gyro_bias\" 行")
    return parse_bias(m.group(1))


def corrected_bias(old, mean) -> tuple[float, float, float]:
    """新 bias = 旧 bias + 当前发布残差均值。纯函数，自检就测它。"""
    return tuple(old[i] + mean[i] for i in range(3))  # type: ignore[return-value]


def write_bias(new) -> None:
    text = SENSORS.read_text(encoding="utf-8")
    formatted = ",".join(f"{v:.6f}" for v in new)
    text2 = re.sub(r'("gyro_bias":\s*")[^"]+(")',
                   lambda m: f"{m.group(1)}{formatted}{m.group(2)}", text, count=1)
    if text2 == text:
        raise SystemExit("没有替换成功：sensors.launch.py 里的格式可能变了")
    SENSORS.write_text(text2, encoding="utf-8")


def measure(seconds: float):
    """订阅 /imu/data_raw 求均值。返回 (mean_gyro, |a|, frames)。"""
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    import time

    class Collector(Node):
        def __init__(self):
            super().__init__("calibrate_gyro_bias")
            self.g, self.a = [], []
            self.create_subscription(Imu, "/imu/data_raw", self._cb, 50)

        def _cb(self, msg):
            self.g.append((msg.angular_velocity.x, msg.angular_velocity.y,
                           msg.angular_velocity.z))
            self.a.append((msg.linear_acceleration.x, msg.linear_acceleration.y,
                           msg.linear_acceleration.z))

    rclpy.init()
    node = Collector()
    deadline = time.time() + seconds
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    gyro = np.array(node.g, dtype=float)
    accel = np.array(node.a, dtype=float)
    rclpy.shutdown()
    if len(gyro) < 500:
        raise SystemExit(f"样本太少（{len(gyro)} 帧）：节点没起/没 activate，或话题名不对")
    return gyro.mean(axis=0), float(np.linalg.norm(accel.mean(axis=0))), len(gyro)


def selftest() -> int:
    # 1) 解析与格式化往返
    assert parse_bias("1,2,3") == (1.0, 2.0, 3.0)
    assert parse_bias(" 0.5 , -0.25 , 0 ") == (0.5, -0.25, 0.0)
    # 2) 修正公式：发布值 = map(raw) - bias，故 new = old + published
    old = (0.001186, -0.007734, -0.003310)
    published = (0.0, -0.000565, -0.001178)
    new = corrected_bias(old, published)
    assert abs(new[2] - (-0.004488)) < 1e-9, new
    # 反代回去：new bias 下的残差应为 0
    residual = tuple(published[i] + old[i] - new[i] for i in range(3))
    assert all(abs(r) < 1e-12 for r in residual), residual
    # 3) 漂移换算：z 残差 -0.001178 rad/s ≈ -4.05 °/min
    assert abs(math.degrees(-0.001178) * 60 - (-4.05)) < 0.05
    # 4) 现用值必须是三个数（防止有人手改坏 sensors.launch.py）
    assert len(current_bias()) == 3
    print("[selftest] 全部通过：解析/修正公式/漂移换算/现用值可读。")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=90.0, help="静止采样时长（默认 90 s）")
    ap.add_argument("--apply", action="store_true", help="把新值写进 sensors.launch.py")
    ap.add_argument("--selftest", action="store_true", help="无硬件自检")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    old = current_bias()
    mean, norm_a, frames = measure(args.seconds)
    print(f"采样 {frames} 帧 / {args.seconds:.0f} s；|a| = {norm_a:.3f} m/s² "
          f"（≈9.7-9.8 才说明没冻结）")
    if abs(norm_a - 9.75) > 0.5:
        print(f"[!!] |a| 偏离重力太多，可能车在动或 IMU 异常；结论不可用。")
        return 2
    drift = math.degrees(mean[2]) * 60
    print(f"当前残差 = [{mean[0]:+.6f} {mean[1]:+.6f} {mean[2]:+.6f}] rad/s")
    print(f"  其中 Z 轴 = {drift:+.2f} °/min（这就是 EKF 的航向漂移率）")
    new = corrected_bias(old, mean)
    print("new gyro_bias = " + ",".join(f"{v:.6f}" for v in new))
    if args.apply:
        write_bias(new)
        print(f"已写入 {SENSORS.relative_to(REPO)}；重启 i2c_imu 生效。")
    else:
        print("（加 --apply 才会写进 sensors.launch.py）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
