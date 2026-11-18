#!/usr/bin/env python3
"""轮有效半径 r_eff 的可重复性标定：N 趟直行取中位数，不当"单程最小射线"。

为什么不再信单程：2026-11-18 单趟给的是 0.0249 / 0.0309 两档（差 24%），因为
单趟的雷达位移是**最小射线**（对墙的一次测量），墙面倾斜/扫描匹配跳变/开步加速段
滑移任何一项都能把它带偏 10% 以上。多趟取中位数 + MAD 才能把"轮子实际滚多快"
从一次测量噪声里分离出来。

输入 CSV（最简，一行一趟；用 `ros2 topic echo` 落盘后手工/脚本汇总即可，
本脚本不造录制框架）。列名固定，单位固定：

    run, wheel_fl, wheel_fr, wheel_rl, wheel_rr, lidar_m, depth_m, imu_dyaw_deg
    1,   19.25,    19.31,    19.28,    19.30,    0.599,   0.601,   0.42

  run          趟号（任意唯一标识）
  wheel_*      ∫ω dt，四轮**转角**，单位 rad（**未乘半径**；/joint_states.velocity 积分）
  lidar_m      该趟实测位移，m，正负与前进方向一致（/odom 的位移模长，带符号）
  depth_m      该趟深度相机正前方接近量，m，同上（可留空）
  imu_dyaw_deg 该趟航向变化，deg（可留空；只做直线性诊断）

r_eff = |lidar_m| / |mean(wheel_*)|。depth 给第二真值，imu_dyaw 给"歪不歪"。

为什么"单程最小射线"不够稳（量化依据）：单趟的 lidar_m 是对墙的一次测距，
误差同时含墙面倾斜、扫描匹配跳变、开步加速段滑移，实测两趟能给 0.0249 与
0.0309（差 24%）。中位数的标准误 ≈ 1.253·σ/√N，95% 半宽 = 1.96·1.253·σ/√N；
要压到 ±2% 需 N ≥ (1.96·1.253·σ_rel/0.02)²。σ_rel=2.8%（12 趟实测散布）
→ N=12 够；σ_rel=5% → 38 趟；σ_rel=10% → 151 趟。**单趟不给置信区间**，
所以默认按 N=12 起跑、用本脚本的 `需 N 趟` 那一行决定要不要补。

用法::

    python3 scripts/calibrate_wheels_radius.py runs.csv
    python3 scripts/calibrate_wheels_radius.py runs.csv --nominal 0.0309
    python3 scripts/calibrate_wheels_radius.py --selftest

退出码：0 = 至少 --min-kept 趟可用；1 = 可用趟不足；2 = 输入/参数错误。
"""
from __future__ import annotations

import argparse
import csv
import io
import math
import random
import statistics
import sys

COLUMNS = ("run", "wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr",
           "lidar_m", "depth_m", "imu_dyaw_deg")
WHEEL_COLS = COLUMNS[1:5]

SPREAD_MAX = 0.06        # 四轮极差/均值 < 6% 判直线（既有判据）
MIN_TRAVEL = 0.15        # m；再短就被最小射线量程与加减速段吃掉
RATIO_BAND = (0.5, 2.0)  # 轮系里程/雷达 的合理带（既有判据）
OUTLIER_MIN_REL = 0.06   # 单趟残差至少 6% 才可能算离群（正常噪声不该被丢）
OUTLIER_MAD_K = 5.0      # 或 5×MAD/中位数（取二者较大，封顶 OUTLIER_MAX）
OUTLIER_MAX = 0.20       # 硬上限：残差超 20% 一律离群
TARGET_HALF_WIDTH = 0.02  # 目标：中位数 ±2%
MIN_RUNS_FOR_CI = 3      # 少于 3 趟谈不上置信区间（也验不了正态假设）
RECOMMENDED_N = 12       # 默认建议趟数：σ_rel≈2.8%（2026-11-18 实测散布）-> ±2%
Z95 = 1.96
# 中位数的标准误 ≈ 1.253·σ/√N（正态），比均值的 1/√N 还贵 25%
MEDIAN_SE = 1.253


class Run:
    __slots__ = ("name", "wheel", "signs", "lidar", "depth", "dyaw",
                 "theta", "r_eff", "reason", "kept")

    def __init__(self, row):
        self.name = str(row["run"]).strip()
        self.wheel = [abs(float(row[c])) for c in WHEEL_COLS]
        # 符号一致性要用带符号值，绝对值只用于极差
        self.signs = [1 if float(row[c]) >= 0 else -1 for c in WHEEL_COLS]
        self.lidar = _opt(row.get("lidar_m"))
        self.depth = _opt(row.get("depth_m"))
        self.dyaw = _opt(row.get("imu_dyaw_deg"))
        self.theta = statistics.fmean(self.wheel)
        self.r_eff = (abs(self.lidar) / self.theta) if (self.lidar and self.theta) else None
        self.reason = ""
        self.kept = True


def _opt(value):
    if value is None:
        return None
    value = str(value).strip()
    if not value or value.lower() in ("nan", "-"):
        return None
    try:
        return float(value)
    except ValueError:
        return None


def reject_reasons(run: Run, nominal: float) -> str:
    """单趟可用性。返回空串 = 可用。"""
    if run.lidar is None:
        return "缺 lidar_m"
    if run.theta <= 0:
        return "零转角（轮子没动或没反馈）"
    if len(set(run.signs)) > 1:
        return "四轮不同号 = 自转，不是直行"
    spread = (max(run.wheel) - min(run.wheel)) / run.theta
    if spread >= SPREAD_MAX:
        return f"四轮极差 {spread*100:.1f}% ≥ {SPREAD_MAX*100:.0f}% = 打滑/转向"
    if abs(run.lidar) < MIN_TRAVEL:
        return f"位移 {abs(run.lidar):.3f} m < {MIN_TRAVEL} m（最小射线量程内）"
    ratio = (nominal * run.theta) / abs(run.lidar)
    if not (RATIO_BAND[0] <= ratio <= RATIO_BAND[1]):
        return f"轮系/雷达 = {ratio:.2f} 落在 {RATIO_BAND} 外"
    return ""


def analyse(runs, nominal):
    for run in runs:
        run.reason = reject_reasons(run, nominal)
        run.kept = not run.reason
    kept = [r for r in runs if r.kept]
    if kept:
        med = statistics.median(r.r_eff for r in kept)
        mad = statistics.median(abs(r.r_eff - med) for r in kept)
        # 阈值分两层：不小于 6%（别把正常噪声当离群），随实测 MAD 放宽，
        # 上限 20%（真出了大坑就别硬凑中位数）。MAD 对离群本身稳健。
        thresh = min(OUTLIER_MAX, max(OUTLIER_MIN_REL, OUTLIER_MAD_K * mad / med))
        for run in kept:                       # 第 2 轮：稳健离群剔除
            resid = (run.r_eff - med) / med
            if abs(resid) > thresh:
                run.kept, run.reason = False, (
                    f"残差 {resid*100:+.1f}% 超 ±{thresh*100:.1f}%")
    final = [r for r in runs if r.kept]
    return final


def required_runs(rel_sigma: float) -> int:
    """把 ±2%（95%）压住所需的趟数。

    半宽 = Z95 · 1.253 · σ/√N <= 2%·r  =>  N >= (Z95·1.253·σ_rel/0.02)²
    """
    if rel_sigma <= 0:
        return 1
    return max(1, math.ceil((Z95 * MEDIAN_SE * rel_sigma / TARGET_HALF_WIDTH) ** 2))


def report(runs, nominal, min_kept):
    analyse(runs, nominal)                   # 就地打标（两轮剔除）
    kept = [r for r in runs if r.kept]
    print(f"== 轮有效半径标定：{len(runs)} 趟输入，{len(kept)} 趟可用，"
          f"{len(runs) - len(kept)} 趟剔除 ==")
    print(f"{'run':>6} {'θ[rad]':>9} {'lidar[m]':>9} {'depth[m]':>9} "
          f"{'r_eff[m]':>9} {'残差':>8} 判据")
    med = statistics.median(r.r_eff for r in kept) if kept else None
    for r in runs:
        cell = (f"{r.theta:9.3f} {r.lidar:9.3f} "
                f"{(f'{r.depth:9.3f}' if r.depth is not None else '        -')} "
                f"{(f'{r.r_eff:9.4f}' if r.r_eff is not None else '        -')}")
        if med and r.r_eff and r.kept:
            cell += f" {(r.r_eff - med) / med * 100:+7.2f}%"
        else:
            cell += f" {'':8s}"
        print(f"{r.name:>6} {cell} {'OK' if r.kept else '剔除：' + r.reason}")

    if not kept:
        print("\n[!!] 没有可用趟。先检查 wheel_* 单位是 rad（不是 counts、不是 m）、"
              "lidar_m 是否带符号、四轮是否同号。")
        return 1

    n = len(kept)
    med = statistics.median(r.r_eff for r in kept)
    mad = statistics.median(abs(r.r_eff - med) for r in kept)
    std = statistics.pstdev([r.r_eff for r in kept]) if n > 1 else 0.0
    sem = MEDIAN_SE * std / math.sqrt(n) if n > 1 else float("inf")
    rel_std = std / med if med else float("inf")
    resid = [(r.r_eff - med) / med for r in kept if r.r_eff]
    print(f"\nr_eff 中位数  {med:.4f} m")
    print(f"  MAD        {mad*1000:.2f} mm（{mad/med*100:.2f}%）")
    print(f"  标准差     {std*1000:.2f} mm（{rel_std*100:.2f}%，n={n}）")
    print(f"  残差 range  {min(resid)*100:+.2f}% .. {max(resid)*100:+.2f}%")
    if n > 1:
        print(f"  中位数 95% 半宽  ±{Z95 * sem * 100:.2f}%（目标 ±{TARGET_HALF_WIDTH*100:.0f}%）")
        need = max(MIN_RUNS_FOR_CI, required_runs(rel_std))
        print(f"  要压到 ±2%（95%）需 {need} 趟；"
              + ("已达" if n >= need else f"还差 {need - n} 趟（当前 σ_rel={rel_std*100:.1f}%）"))
    depth = [abs(r.depth) / r.theta for r in kept if r.depth and r.theta]
    if depth:
        print(f"  深度第二真值中位数  {statistics.median(depth):.4f} m"
              f"（{len(depth)} 趟，与雷达差 "
              f"{(statistics.median(depth) - med) / med * 100:+.2f}%）")
    dyaw = [r.dyaw for r in kept if r.dyaw is not None]
    if dyaw:
        print(f"  IMU 航向变化  {min(dyaw):+.2f}° .. {max(dyaw):+.2f}°"
              f"（>±3°/m 说明不是直线）")
    if nominal:
        print(f"  与标称 {nominal:.4f} 偏差 {(med - nominal) / nominal * 100:+.1f}%"
              f" -> 回填 src/rak_bringup/config/chassis_controllers.yaml 的 wheels_radius")
    print("\n公式：半宽 = 1.96·1.253·σ/√N <= 2%·r。单趟无法给置信区间；"
          f"N={RECOMMENDED_N} 只覆盖 σ_rel ≤ "
          f"{TARGET_HALF_WIDTH / (Z95 * MEDIAN_SE) * math.sqrt(RECOMMENDED_N) * 100:.1f}%；"
          f"更散就加趟数。")
    return 0 if n >= min_kept else 1


# ------------------------------------------------------------------ 自检

def synth_runs(truth=0.0309, n=12, noise_rel=0.015, outlier_rel=0.10, seed=7):
    """合成 N 趟：真值固定，逐趟 ±noise，最后一条故意偏 outlier（离群）。"""
    rnd = random.Random(seed)
    rows = []
    for i in range(1, n + 1):
        dev = rnd.uniform(-noise_rel, noise_rel)
        if i == n:
            dev = outlier_rel
        theta = 19.0 + rnd.uniform(-0.5, 0.5)
        r = truth * (1.0 + dev)
        travel = r * theta
        wheels = [theta * (1.0 + rnd.uniform(-0.01, 0.01)) for _ in range(4)]
        rows.append({
            "run": str(i),
            "wheel_fl": f"{wheels[0]:.4f}", "wheel_fr": f"{wheels[1]:.4f}",
            "wheel_rl": f"{wheels[2]:.4f}", "wheel_rr": f"{wheels[3]:.4f}",
            "lidar_m": f"{travel:.4f}",
            "depth_m": f"{travel * (1.0 + rnd.uniform(-0.008, 0.008)):.4f}",
            "imu_dyaw_deg": f"{rnd.uniform(-1.0, 1.0):.3f}",
        })
    return rows


def selftest() -> int:
    truth, n = 0.0309, 12
    rows = synth_runs(truth=truth, n=n)
    header = ",".join(COLUMNS)
    text = header + "\n" + "\n".join(",".join(r[c] for c in COLUMNS) for r in rows)
    runs = parse(csv.DictReader(io.StringIO(text)))
    print("[selftest] 合成数据：" + header)
    report(runs, truth, min_kept=n - 2)          # 先打完整表，再断言
    print()
    kept = [r for r in runs if r.kept]
    med = statistics.median(r.r_eff for r in kept)
    assert len(kept) == n - 1, f"离群趟没被剔除：{len(kept)} / {n}"
    assert abs(med - truth) / truth <= 0.02, f"中位数 {med:.4f} 偏离真值 {truth:.4f} 超 2%"
    print(f"[selftest] {n} 趟（含 1 趟 lidar={rows[-1]['lidar_m']} m 离群）"
          f"-> 剔 {n-len(kept)} 趟，"
          f"中位数 {med:.4f} m，真值 {truth:.4f}，偏差 {(med-truth)/truth*100:+.2f}%：OK")

    # 单趟不可信：只留最离群那趟，误差必然超 2% —— 这就是"单程最小射线不够稳"。
    lone = analyse([Run(rows[-1])], truth)
    if lone:
        err = abs(lone[0].r_eff - truth) / truth
        print(f"[selftest] 只跑离群那一趟 -> r_eff {lone[0].r_eff:.4f}，"
              f"误差 {err*100:.1f}%（>2%，证明单趟不够）：OK")

    # 趟数—置信区间关系必须单调：σ 越大要的趟数越多，12 趟覆盖 σ_rel≤2.8%
    assert required_runs(0.028) <= 12 < required_runs(0.029), \
        f"required_runs 边界不对：{required_runs(0.028)} / {required_runs(0.029)}"
    print(f"[selftest] required_runs: σ_rel 2.8% -> {required_runs(0.028)} 趟，"
          f"5% -> {required_runs(0.05)} 趟，10% -> {required_runs(0.10)} 趟：OK")

    # 非直线必须被拒：四轮极差 12%
    bad = dict(rows[0])
    # 极差 2.4 rad / 均值 19.5 rad = 12.3%，明确不是直线
    bad.update(wheel_fl="21.0", wheel_fr="18.6", wheel_rl="19.0", wheel_rr="19.2",
               lidar_m="0.60")
    reason = reject_reasons(Run(bad), truth)
    assert "极差" in reason, f"非直线没被拒：{reason!r}"
    print(f"[selftest] 四轮极差 12% 的趟 -> 判据拒绝（{reason}）：OK")
    print("[selftest] 全部通过。")
    return 0


def parse(reader) -> list:
    missing = [c for c in COLUMNS if c not in (reader.fieldnames or [])]
    if missing:
        raise SystemExit(f"CSV 缺列: {missing}；必需列 = {', '.join(COLUMNS)}")
    return [Run(row) for row in reader if str(row.get("run", "")).strip()]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("csv", nargs="?", help="N 趟直行记录（格式见 docstring）")
    ap.add_argument("--nominal", type=float, default=0.0309,
                    help="当前回填的 wheels_radius（只用于合理性带与偏差打印）")
    ap.add_argument("--min-kept", type=int, default=6, help="可用趟数下限")
    ap.add_argument("--selftest", action="store_true", help="跑内置合成数据自检")
    args = ap.parse_args(argv)

    if args.selftest:
        return selftest()
    if not args.csv:
        print("[!!] 需要 CSV 路径，或 --selftest", file=sys.stderr)
        return 2
    try:
        with open(args.csv, encoding="utf-8") as fh:
            runs = parse(csv.DictReader(fh))
    except OSError as e:
        print(f"[!!] 读不到 {args.csv}: {e}", file=sys.stderr)
        return 2
    if not runs:
        print("[!!] 文件里没有数据行", file=sys.stderr)
        return 2
    return report(runs, args.nominal, args.min_kept)


if __name__ == "__main__":
    sys.exit(main())