"""打滑期间轮式里程计协方差膨胀的纯逻辑（不依赖 ROS，可离线单测）。

L2 中继（odom_cov_guard 节点）用这里的三件事：

1. ``next_multiplier``：膨胀倍数状态机。打滑嫌疑成立 → 立刻升到 K；
   恢复后按参数决定立刻回落 1.0 或每帧 ÷ramp_step 斜坡回落（斜坡避免
   协方差阶跃引起 EKF 状态突跳）。
2. ``inflate_covariances``：只乘对角，绝不动非对角与时间戳。

膨胀范围与依据（firmware-capability-spec §7.2 + ekf.yaml 注释的团队规矩）：
    pose x/y（cov[0]/cov[7]）：打滑期间轮式积分位置漂移，直接不可信；
    twist vx/vy（cov[0]/cov[7]）：麦轮 vx、vy 全部由四轮速合成，打滑期间
        同源不可信。§7.2 只点名 vx，vy 是同一物理来源，一并膨胀不加状态。
    vyaw 不膨胀 —— 轮式 vyaw 本来就不进 EKF（ekf.yaml odom1_config 排除，
    航向权威 = IMU gyro-z，ADR-0002）。

K 的默认值（odom_cov_guard 的 ``cov_factor`` 参数）：10.0 起步，出处是
`rak_bringup/config/ekf.yaml` 注释里既定的保守回退量级（"×N，N 取 10 起步，
落地期标定"）。未标定前宁可让 EKF 少信轮式，也不让它信错。
"""

from __future__ import annotations

import json
import math

from .stats import percentile

#: pose.covariance 行主序 6×6：x=0, y=7
POSE_DIAGONAL = (0, 7)
#: twist.covariance 行主序 6×6：vx=0, vy=7
TWIST_DIAGONAL = (0, 7)


def next_multiplier(current: float, slip: bool, factor: float,
                    ramp_step: float) -> float:
    """按当前嫌疑状态推进膨胀倍数；返回值 ≥ 1.0（1.0 = 原样转发）。

    ``slip=True``  → 直接 ``factor``（升程不做斜坡：保护宁可早到）。
    ``slip=False`` → ``ramp_step <= 1`` 视为"立刻回落"；否则每帧除以
    ``ramp_step`` 几何回落，且不低于 1.0。
    """
    factor = max(1.0, float(factor))
    if slip:
        return factor
    current = max(1.0, float(current))
    step = float(ramp_step)
    if step <= 1.0 or current <= 1.0:
        return 1.0
    return max(1.0, current / step)


def inflate_covariances(
        pose_cov, twist_cov, factor: float) -> tuple[list[float], list[float]]:
    """把 pose x/y 与 twist vx/vy 对角乘 ``factor``，返回新的两条 36 元素列表。

    非对角原样保留；``factor <= 1`` 等价于复制原值（不放大）。
    长度不是 36 直接抛 ValueError —— 消息字段长度错误宁可炸在中间层，
    也不能把残缺协方差喂给 EKF。
    """
    pose = list(pose_cov)
    twist = list(twist_cov)
    if len(pose) != 36 or len(twist) != 36:
        raise ValueError(
            f"covariance 必须是 36 元素，得到 pose={len(pose)} twist={len(twist)}")
    f = max(1.0, float(factor))
    if f == 1.0:
        return pose, twist
    for i in POSE_DIAGONAL:
        pose[i] *= f
    for i in TWIST_DIAGONAL:
        twist[i] *= f
    return pose, twist


def suggest_sigma(residuals, q: float = 0.95) -> float | None:
    """从一段健康行驶的残差序列回填建议 σ（``abs`` 分位数，默认 p95）。

    用法对应 slip.py 的标定路径：首日基线（正常行驶，无打滑）→
    σ = p95(|residual|) → 阈值 = n_sigma·σ。NaN/±inf 直接忽略；
    全部无效或空序列返回 None（调用方决定报错还是跳过）。
    """
    vals = sorted(abs(float(x)) for x in residuals
                  if x == x and math.isfinite(float(x)))
    if not vals:
        return None
    return percentile(vals, q)


def load_baseline(path: str) -> list[float]:
    """读取记录下来的残差序列：JSON（数字数组或 {"residuals": [...]}）或 CSV。

    CSV 规则：一行一个数；``#`` 开头的行与无法解析的行跳过（容忍表头/杂注）。
    文件不存在抛 FileNotFoundError —— 由调用方决定这是错误还是跳过。
    """
    with open(path, "r", encoding="utf-8") as fh:
        text = fh.read()
    if path.endswith(".json"):
        data = json.loads(text)
        if isinstance(data, dict):
            data = data["residuals"]
        if not isinstance(data, list):
            raise ValueError(f"{path}: JSON 需要数字数组或 {{'residuals': [...]}}")
        return [float(x) for x in data]
    out: list[float] = []
    for line in text.splitlines():
        token = line.split("#", 1)[0].strip().rstrip(",")
        if not token:
            continue
        try:
            out.append(float(token))
        except ValueError:
            continue  # 表头/杂注行：跳过而不是报错
    return out


def main(args: list[str] | None = None) -> None:
    """``slip_sigma <baseline.csv|json> [--n-sigma N]``：打印建议 σ 与阈值（JSON）。"""
    import argparse
    import sys

    parser = argparse.ArgumentParser(description="从基线残差序列回填建议 σ")
    parser.add_argument("path", help="残差序列文件（CSV 一行一个数，或 JSON 数组）")
    parser.add_argument("--n-sigma", type=float, default=3.0)
    parser.add_argument("--q", type=float, default=0.95, help="分位数，默认 p95")
    ns = parser.parse_args(args)
    sigma = suggest_sigma(load_baseline(ns.path), q=ns.q)
    if sigma is None:
        print(f"{ns.path}: 序列里没有有效残差", file=sys.stderr)
        raise SystemExit(2)
    print(json.dumps({
        "sigma": round(sigma, 6),
        "n_sigma": ns.n_sigma,
        "threshold": round(ns.n_sigma * sigma, 6),
    }))
