#!/usr/bin/env python3
"""双 IMU 互校：Jetson 直连 LSM6DSRTR（`/imu/data_raw`）vs 底盘板载 IMU（CANopen 0x2030..）。

两块 IMU 的安装与单位都**实测过**（2026-11-18）：

  Jetson 直连：`axis_map: "-z,-x,y"`，正常姿态 `acc=(0.02, 0.15, 9.84)`、`|a|=9.84`、
               重力沿 **+Z**（驱动已把芯片系旋到 ROS 系）；`/imu/data_raw` 里
               `angular_velocity` 已是 rad/s，`orientation` 不填。
  板载（chassis F407）：模块**平装**，重力沿芯片 **+Z**；OD `0x2030 imu_dyaw`
               是**每 SYNC 的毫弧度增量**（不是 °/s！），`0x2031 pitch` / `0x2032 roll`
               是 **deg ×0.01**，改后实测 −2.14° / −0.21°（R78 用
               `pitch=atan2(-ax,az)`、`roll=atan2(-ay,az)`）。
               `0x2033 evt` / `0x2034 age` 只做存活判据（age 饱和 255 = 采样停了）。

三种模式：

**离线（默认，无 ROS 也能跑）**：两路 CSV/JSON，时间戳各自单调。最简格式：

    # jetson.csv   （`ros2 topic echo /imu/data_raw` 落盘后抽列即可）
    t, gx, gy, gz, ax, ay, az          # 秒, rad/s, m/s²
    # board.csv
    t, dyaw_mrad, pitch_deg, roll_deg, age_ms
    # 也接受 JSON：[{"t":..,"gx":..,...}, ...] 或 {"times":[...], "gx":[...]}

**在线（`--online`，显式才 import rclpy）**：`ros2 topic echo` 两路，或直接订阅
`/imu/data_raw` + `/chassis/tpdo`（后者需要 CANopen 主站在跑）。

    用法（离线）：
      python3 scripts/compare_imu.py --jetson jetson.csv --board board.csv
      python3 scripts/compare_imu.py --selftest
    用法（在线，真机、Lead 用）：
      python3 scripts/compare_imu.py --online --duration 20

判据（两条物理事实各出一条）：
  ① **原地慢转**：`yaw rate` 的 scale、偏置、相关系数、**符号一致性**。板载 dyaw 是
     增量，按 dt 换成角速度再比。scale 落在 [0.9,1.11] 且 r ≥ 0.95 且同号 -> 同源可信；
     scale 外 -> 需重标定；异号 -> 轴或符号不一致。
  ② **直线**：`|a|`（Jetson ≈9.8、板载不参与）、倾角（−2/−0.2° 量级）、零偏漂移
     （静止段 gyro 均值随时间的斜率）。倾角超 ±5° 或零偏漂移 > 2°/min -> 需重标定。

退出码：0 = 同源可信；1 = 需重标定/轴不一致；2 = 输入/参数错误。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import random
import statistics
import sys

G = 9.80665
TILT_RANGE = 5.0            # deg，板载倾角该在几度内（实测 −2.14 / −0.21）
SCALE_BAND = (0.90, 1.11)   # 互校 scale 容差（±10%，轮半径同类量级的实测散布）
MIN_CORR = 0.95
DRIFT_MAX_DEG_MIN = 2.0     # 静止零偏漂移上限，deg/min
QUIET_RAD_S = 0.05          # |w| 低于它算静止样本（零偏漂移只在这些样本上算）
DRIFT_MIN_SPAN_S = 10.0     # 静止样本覆盖不足这个时长 -> 不报漂移（n/a）
MDRAD_TO_RADS = 1e-3


def _nums(values):
    """CSV 里的数值列 -> float 列表；空/'-'/nan 变 None。"""
    if not values:
        return values
    out = []
    for v in values:
        if v is None or str(v).strip() in ("", "-", "nan", "None"):
            out.append(None)
            continue
        try:
            out.append(float(v))
        except (TypeError, ValueError):
            out.append(None)
    return out


class Series:
    """一路按时间的标量序列；t 单调，用于线性插值到共同时基。"""

    __slots__ = ("t", "v")

    def __init__(self, t, v):
        # CSV 读进来都是字符串；非数值（空/'-'/nan）直接丢掉这一拍
        pairs = sorted((float(a), float(b)) for a, b in zip(t, v)
                       if b is not None and str(b).strip() not in ("", "-", "nan", "None"))
        self.t = [p[0] for p in pairs]
        self.v = [p[1] for p in pairs]

    def __len__(self):
        return len(self.t)

    def at(self, t0):
        """线性插值；超出范围返回 None。"""
        if not self.t or t0 < self.t[0] or t0 > self.t[-1]:
            return None
        lo, hi = 0, len(self.t) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self.t[mid] <= t0:
                lo = mid
            else:
                hi = mid
        span = self.t[hi] - self.t[lo]
        if span <= 0:
            return self.v[lo]
        w = (t0 - self.t[lo]) / span
        return self.v[lo] * (1 - w) + self.v[hi] * w


class Track:
    """一组同源样本（含各自的时基与可选分量）。"""

    def __init__(self, name, t, gz=None, dyaw=None, ax=None, ay=None, az=None,
                 pitch=None, roll=None, age=None, yaw=None):
        self.name = name
        self.t = t
        self.gz = Series(t, gz) if gz else None
        self.dyaw = Series(t, dyaw) if dyaw else None
        self.ax, self.ay, self.az = (_nums(ax), _nums(ay), _nums(az))
        self.pitch = Series(t, pitch) if pitch else None
        self.roll = Series(t, roll) if roll else None
        self.age = Series(t, age) if age else None
        self.yaw = Series(t, yaw) if yaw else None


# ------------------------------------------------------------------ 读取

def _cols(rows, names):
    out = []
    for n in names:
        out.append([row[n] for row in rows] if n in rows[0] else None)
    return out


def load(path):
    """读 CSV 或 JSON -> list[dict]。CSV 首行是列名。"""
    with open(path, encoding="utf-8") as fh:
        head = fh.read(2)
        fh.seek(0)
        if head.lstrip()[:1] == "{" or head.lstrip()[:1] == "[":
            doc = json.load(fh)
            if isinstance(doc, dict):                # {"t":[...], "gx":[...]}
                keys = list(doc)
                n = len(doc[keys[0]])
                return [{k: doc[k][i] for k in keys} for i in range(n)]
            return doc
        return [r for r in csv.DictReader(fh)]


def _f(row, *names):
    for n in names:
        if n in row and str(row[n]).strip() not in ("", "-", "nan", "None"):
            try:
                return float(row[n])
            except ValueError:
                continue
    return None


def as_jetson(path):
    rows = [r for r in load(path) if _f(r, "t", "time", "stamp") is not None]
    t = [_f(r, "t", "time", "stamp") for r in rows]
    gz, ax, ay, az = _cols(rows, ["gz", "ax", "ay", "az"])
    return Track("jetson", t, gz=gz, ax=ax, ay=ay, az=az)


def as_board(path):
    rows = [r for r in load(path) if _f(r, "t", "time", "stamp") is not None]
    t = [_f(r, "t", "time", "stamp") for r in rows]
    dyaw = _cols(rows, ["dyaw_mrad", "dyaw"])[0]
    pitch = _cols(rows, ["pitch_deg", "pitch"])[0]
    roll = _cols(rows, ["roll_deg", "roll"])[0]
    age = _cols(rows, ["age_ms", "age"])[0]
    # 板载平装、芯片 +Z 朝上：竖直轴加速度就是 az（本工具不拿它算 |a|）
    return Track("board", t, dyaw=dyaw, pitch=pitch, roll=roll, age=age)


# ------------------------------------------------------------------ 互校

def yaw_rate_from_deltas(track):
    """板载 dyaw 是每 SYNC 的 mrad 增量 -> 逐拍 rad/s（dt 用相邻时间戳）。"""
    if track.dyaw is None or len(track.dyaw) < 2:
        return None
    t, v = track.dyaw.t, track.dyaw.v
    rate_t, rate_v = [], []
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        if dt <= 0 or dt > 1.0:            # 时间戳跳变/掉帧：这拍不算角速度
            continue
        rate_t.append(0.5 * (t[i] + t[i - 1]))
        rate_v.append(v[i] * MDRAD_TO_RADS / dt)
    return Series(rate_t, rate_v) if rate_t else None


def compare_yaw(jetson_rate, board_rate):
    """把两路角速度对齐到共同时基，回归 board = a + b·jetson。"""
    t_ref = [t for t in board_rate.t if jetson_rate.at(t) is not None]
    if len(t_ref) < 20:
        return None
    xs, ys = [], []
    for t in t_ref:
        xs.append(jetson_rate.at(t))
        ys.append(board_rate.at(t))
    n = len(xs)
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx <= 0 or syy <= 0:
        return None
    slope = sxy / sxx
    offset = my - slope * mx
    corr = sxy / math.sqrt(sxx * syy)
    same_sign = (slope > 0) and (sum(1 for x, y in zip(xs, ys) if x * y > 0) / n) > 0.8
    return {"n": n, "scale": slope, "offset": offset, "corr": corr,
            "same_sign": same_sign, "span": max(t_ref) - min(t_ref),
            "jetson_rms": math.sqrt(statistics.fmean(x * x for x in xs)),
            "board_rms": math.sqrt(statistics.fmean(y * y for y in ys))}


def compare_static(jt, bt=None):
    """直线/静止段的 |a|、倾角、零偏漂移。

    |a| 与 gz 零偏漂移只在 Jetson 路上算（板载遥测没有加速度/陀螺原始量），
    倾角与 imu_age 只在板载路上算。jt=Jetson、bt=板载。
    """
    out = {}
    if jt.ax and jt.ay and jt.az:
        mags = [math.sqrt(x * x + y * y + z * z)
                for x, y, z in zip(jt.ax, jt.ay, jt.az)
                if None not in (x, y, z)]
        if mags:
            out["acc_mag"] = statistics.median(mags)
    if jt.gz is not None and len(jt.gz) > 20:
        # 零偏漂移只看**静止样本**（|w| < QUIET_RAD_S）：转动段的均值本来就不一样，
        # 混进去算出来是运动不是零偏。前一半 vs 后一半的均值差 / 时间 -> deg/min。
        quiet = [(t, v) for t, v in zip(jt.gz.t, jt.gz.v) if abs(v) < QUIET_RAD_S]
        # 静止样本太少（纯转台数据/全程在动）就别算漂移 —— 会把"在动"报成"零偏漂"。
        span = (quiet[-1][0] - quiet[0][0]) if quiet else 0.0
        if len(quiet) >= 20 and span >= DRIFT_MIN_SPAN_S:
            half = len(quiet) // 2
            first = [v for _t, v in quiet[:half]]
            last = [v for _t, v in quiet[half:]]
            dt = quiet[-1][0] - quiet[0][0]
            if dt > 0:
                drift = ((statistics.fmean(last) - statistics.fmean(first)) / dt
                         * 60.0 * 180.0 / math.pi)
                out["drift_deg_min"] = drift
                out["quiet_frac"] = len(quiet) / len(jt.gz.v)
        out["gz_bias"] = statistics.fmean(jt.gz.v)
    board = bt if bt is not None else jt
    if board.pitch is not None and len(board.pitch):
        out["pitch"] = statistics.median(board.pitch.v)
    if board.roll is not None and len(board.roll):
        out["roll"] = statistics.median(board.roll.v)
    if board.age is not None and len(board.age):
        out["age_max"] = max(board.age.v)
        out["age_stalled"] = sum(1 for a in board.age.v if a >= 250) / len(board.age.v)
    return out


def verdict(yaw, static):
    """一句裁决。"""
    if yaw is None:
        return "数据不足", "无法比对 yaw rate（共同时基样本 <20 或某一路全零）"
    if not yaw["same_sign"]:
        return "轴或符号不一致", (
            f"scale={yaw['scale']:+.3f} r={yaw['corr']:+.3f}；两路整体异号 -> "
            "先对轴映射（Jetson axis_map / 板载平装 vs 竖装）")
    lo, hi = SCALE_BAND
    if not (lo <= yaw["scale"] <= hi):
        return "需重标定", (
            f"scale={yaw['scale']:.3f} 落在 [{lo},{hi}] 外 -> 一方量纲或零偏需重标"
            f"（板载 dyaw 确认是 mrad 增量、0x2031/0x2032 是 deg×0.01）")
    if yaw["corr"] < MIN_CORR:
        return "需重标定", (
            f"r={yaw['corr']:.3f} < {MIN_CORR} -> 两路波形不同步；查时间戳/丢帧/"
            "板载 age 是否饱和")
    if static.get("acc_mag") is not None and abs(static["acc_mag"] - G) > 0.5:
        return "需重标定", f"|a|={static['acc_mag']:.2f} 偏离 9.81 超 0.5 -> 加计刻度/轴映射"
    for key, label in (("pitch", "pitch"), ("roll", "roll")):
        if key in static and abs(static[key]) > TILT_RANGE:
            return "需重标定", (
                f"板载 {label}={static[key]:+.2f}° 超 ±{TILT_RANGE}° -> "
                "倾角解析式/安装轴仍不对（R78 后实测 −2.14/−0.21）")
    if static.get("drift_deg_min") is not None and abs(static["drift_deg_min"]) > DRIFT_MAX_DEG_MIN:
        return "需重标定", (
            f"静止零偏漂移 {static['drift_deg_min']:+.2f} °/min > "
            f"{DRIFT_MAX_DEG_MIN} °/min -> 重标零偏（60 s 静置）")
    if static.get("age_stalled", 0) > 0.2:
        return "需重标定", (
            f"板载 imu_age 饱和（{static['age_stalled']*100:.0f}% 帧）-> 采样停了，"
            "先修板载 IMU")
    return "同源可信", (
        f"scale={yaw['scale']:.3f} r={yaw['corr']:.3f} 同号；"
        f"|a|={static.get('acc_mag', float('nan')):.2f} "
        f"pitch={static.get('pitch', float('nan')):+.2f}° "
        f"roll={static.get('roll', float('nan')):+.2f}°")


def report(jt, bt, yaw, static, tag=""):
    print(f"== 双 IMU 互校{('（' + tag + '）') if tag else ''}："
          f"Jetson {len(jt.t)} 帧，板载 {len(bt.t)} 帧 ==")
    print("\n[表 1] 原地慢转 — yaw rate 比对")
    if yaw is None:
        print("  (样本不足或某一路全零，无法比对)")
    else:
        print(f"  共同时基  {yaw['n']} 帧 / {yaw['span']:.1f} s")
        print(f"  scale     {yaw['scale']:+.4f}（板载/Jetson，容差 {SCALE_BAND}）")
        print(f"  偏置      {yaw['offset']:+.4f} rad/s（板载相对 Jetson）")
        print(f"  相关系数  {yaw['corr']:+.4f}（下限 {MIN_CORR}）")
        print(f"  符号       {'一致' if yaw['same_sign'] else '不一致'}"
              f"；RMS Jetson {yaw['jetson_rms']:.4f} / 板载 {yaw['board_rms']:.4f} rad/s")
    print("\n[表 2] 直线/静止 — |a|、倾角、零偏漂移")
    def row(k, v, unit=""):
        print(f"  {k:12s} {v:>12s}{unit}")
    row("|a| Jetson", f"{static['acc_mag']:.3f}" if "acc_mag" in static else "n/a", " m/s²")
    row("板载 pitch", f"{static['pitch']:+.2f}" if "pitch" in static else "n/a", " °")
    row("板载 roll", f"{static['roll']:+.2f}" if "roll" in static else "n/a", " °")
    row("gz 零偏", f"{static['gz_bias']:+.5f}" if "gz_bias" in static else "n/a", " rad/s")
    row("零偏漂移", f"{static['drift_deg_min']:+.2f}" if "drift_deg_min" in static else "n/a",
        " °/min")
    if "age_max" in static:
        row("imu_age 峰值", f"{static['age_max']:.0f}", " (255=停)")
    code, why = verdict(yaw, static)
    print(f"\n== 裁决：{code} ==")
    print(f"   {why}")
    return 0 if code == "同源可信" else 1


# ------------------------------------------------------------------ 自检

def synth(truth_scale=1.0, flip=False, seed=3, dur=20.0, rate_hz=50.0, noise=0.01):
    """合成原地慢转 + 后半段静止：Jetson rad/s，板载 mrad 增量（每拍）。"""
    rnd = random.Random(seed)
    n = int(dur * rate_hz)
    t, gz, dyaw, pitch, roll, age = [], [], [], [], [], []
    acc_x, acc_y, acc_z = [], [], []
    for i in range(n):
        tt = i / rate_hz
        # 0..8 s 慢转 ±0.5 rad/s 正弦；之后静止（测零偏漂移）
        w = 0.5 * math.sin(2 * math.pi * tt / 8.0) if tt < 8.0 else 0.0
        w += 0.004 * (tt / dur)                      # 轻微零偏漂移，仍远低于 2°/min
        g = w + rnd.gauss(0, noise)
        scale = truth_scale * (-1.0 if flip else 1.0)
        t.append(tt)
        gz.append(g)
        dyaw.append(g * scale * (1.0 / rate_hz) * 1000.0)   # rad -> mrad 增量
        pitch.append(-2.14 + rnd.gauss(0, 0.05))
        roll.append(-0.21 + rnd.gauss(0, 0.05))
        age.append(10.0)
        acc_x.append(0.02 + rnd.gauss(0, 0.01))
        acc_y.append(0.15 + rnd.gauss(0, 0.01))
        acc_z.append(9.84 + rnd.gauss(0, 0.02))
    return Track("jetson", t, gz=gz, ax=acc_x, ay=acc_y, az=acc_z), \
        Track("board", t, dyaw=dyaw, pitch=pitch, roll=roll, age=age)


def selftest() -> int:
    # ① 正常同源：scale≈1、同号 -> 同源可信
    jt, bt = synth()
    yaw = compare_yaw(jt.gz, yaw_rate_from_deltas(bt))
    static = compare_static(jt, bt)
    report(jt, bt, yaw, static, tag="selftest-1 正常")
    assert yaw is not None and abs(yaw["scale"] - 1.0) < 0.02, f"scale={yaw}"
    assert yaw["corr"] > 0.99, f"r={yaw['corr']}"
    assert verdict(yaw, static)[0] == "同源可信", verdict(yaw, static)
    print()

    # ② 故意反号（板载轴反了）-> 轴或符号不一致
    jt2, bt2 = synth(flip=True, seed=11)
    yaw2 = compare_yaw(jt2.gz, yaw_rate_from_deltas(bt2))
    static2 = compare_static(jt2, bt2)
    report(jt2, bt2, yaw2, static2, tag="selftest-2 反号")
    assert yaw2 is not None and not yaw2["same_sign"], f"反号没被抓到：{yaw2}"
    assert verdict(yaw2, static2)[0] == "轴或符号不一致", verdict(yaw2, static2)
    print()

    # ③ scale 1.03 -> 仍在 ±10% 带内 -> 同源可信
    jt3, bt3 = synth(truth_scale=1.03, seed=13)
    yaw3 = compare_yaw(jt3.gz, yaw_rate_from_deltas(bt3))
    static3 = compare_static(jt3, bt3)
    report(jt3, bt3, yaw3, static3, tag="selftest-3 scale 1.03")
    assert SCALE_BAND[0] <= yaw3["scale"] <= SCALE_BAND[1], yaw3["scale"]
    assert verdict(yaw3, static3)[0] == "同源可信", verdict(yaw3, static3)
    print()

    # ④ scale 1.25（超出带）-> 需重标定
    jt4, bt4 = synth(truth_scale=1.25, seed=17)
    yaw4 = compare_yaw(jt4.gz, yaw_rate_from_deltas(bt4))
    static4 = compare_static(jt4, bt4)
    report(jt4, bt4, yaw4, static4, tag="selftest-4 scale 1.25")
    assert verdict(yaw4, static4)[0] == "需重标定", verdict(yaw4, static4)
    print()

    # ⑤ 倾角垃圾（R76 老算法的 ±163°）-> 需重标定
    jt5, bt5 = synth(seed=19)
    bt5.pitch = Series(bt5.t, [89.0] * len(bt5.t))
    yaw5 = compare_yaw(jt5.gz, yaw_rate_from_deltas(bt5))
    static5 = compare_static(jt5, bt5)
    assert verdict(yaw5, static5)[0] == "需重标定", verdict(yaw5, static5)
    print(f"[selftest-5] 倾角 89° -> {verdict(yaw5, static5)[0]}：OK")

    # ⑥ imu_age 饱和 -> 采样停了
    jt6, bt6 = synth(seed=23)
    bt6.age = Series(bt6.t, [255.0] * len(bt6.t))
    static6 = compare_static(jt6, bt6)
    assert verdict(compare_yaw(jt6.gz, yaw_rate_from_deltas(bt6)), static6)[0] == "需重标定"
    print(f"[selftest-6] imu_age 恒 255 -> {verdict(yaw, static6)[0]}：OK")
    print("\n[selftest] 全部断言通过。")
    return 0


# ------------------------------------------------------------------ 在线（可选）

def run_online(duration):
    """在线采集。只在 --online 时 import rclpy / 订阅话题。"""
    import rclpy                                   # noqa: PLC0415 - 显式延迟导入
    from rclpy.node import Node
    from sensor_msgs.msg import Imu

    class Collector(Node):
        def __init__(self):
            super().__init__("compare_imu_collector")
            self.jt, self.bt = [], []
            self.create_subscription(Imu, "/imu/data_raw", self._on_imu, 50)
            # 板载遥测：CANopen proxy 的 tpdo 话题；对象 0x2030..0x2034 的 COData
            try:
                from canopen_interfaces.msg import COData
            except ImportError:
                print("[!!] 缺 canopen_interfaces（未装 ros2_canopen？）；只用 Jetson 一路")
                return self
            self.create_subscription(COData, "/chassis/tpdo", self._on_tpdo, 50)
            self._co = COData
            self._t0 = self.get_clock().now().nanoseconds * 1e-9
            return self

        def _now(self):
            return self.get_clock().now().nanoseconds * 1e-9 - self._t0

        def _on_imu(self, msg):
            self.jt.append({"t": self._now(), "gx": msg.angular_velocity.x,
                            "gy": msg.angular_velocity.y, "gz": msg.angular_velocity.z,
                            "ax": msg.linear_acceleration.x, "ay": msg.linear_acceleration.y,
                            "az": msg.linear_acceleration.z})

        def _on_tpdo(self, msg):
            idx = msg.index
            t = self._now()
            if idx == 0x2030:
                self.bt.append({"t": t, "dyaw_mrad": _int16(msg.data)})
            elif idx == 0x2031:
                self.bt.append({"t": t, "pitch_deg": _int16(msg.data) * 0.01})
            elif idx == 0x2032:
                self.bt.append({"t": t, "roll_deg": _int16(msg.data) * 0.01})
            elif idx == 0x2034:
                self.bt.append({"t": t, "age_ms": msg.data & 0xFF})

    def _int16(v):
        v &= 0xFFFF
        return v - 0x10000 if v >= 0x8000 else v

    def _merge(rows, keys):
        """同一时间轴上按对象分行到达 -> 对齐成宽表（就近取最近值）。"""
        rows = sorted(rows, key=lambda r: r["t"])
        out, last = [], {}
        for r in rows:
            last.update({k: v for k, v in r.items() if k != "t"})
            out.append({"t": r["t"], **{k: last[k] for k in keys if k in last}})
        return out

    rclpy.init()
    node = Collector()
    print(f"[online] 订阅 /imu/data_raw + /chassis/tpdo，采 {duration:.0f} s…")
    try:
        rclpy.spin_once(node, timeout_sec=duration)
    except KeyboardInterrupt:
        pass
    jt_rows, bt_rows = node.jt, node.bt
    node.destroy_node()
    rclpy.shutdown()

    jt = Track("jetson", [r["t"] for r in jt_rows], gz=[r["gz"] for r in jt_rows],
               ax=[r["ax"] for r in jt_rows], ay=[r["ay"] for r in jt_rows],
               az=[r["az"] for r in jt_rows])
    merged = _merge(bt_rows, ("dyaw_mrad", "pitch_deg", "roll_deg", "age_ms"))
    bt = Track("board", [r["t"] for r in merged],
               dyaw=[r.get("dyaw_mrad") for r in merged],
               pitch=[r.get("pitch_deg") for r in merged],
               roll=[r.get("roll_deg") for r in merged],
               age=[r.get("age_ms") for r in merged])
    return jt, bt


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--jetson", help="Jetson /imu/data_raw 的 CSV/JSON")
    ap.add_argument("--board", help="板载 IMU 遥测的 CSV/JSON")
    ap.add_argument("--online", action="store_true",
                    help="在线采集（需要 ROS 2；显式才 import rclpy）")
    ap.add_argument("--duration", type=float, default=20.0, help="在线采集秒数")
    ap.add_argument("--selftest", action="store_true", help="合成数据自检（无硬件/无 ROS）")
    args = ap.parse_args(argv)

    if args.selftest:
        return selftest()

    if args.online:
        try:
            jt, bt = run_online(args.duration)
        except ImportError as e:
            print(f"[!!] --online 需要 ROS 2 环境（{e}）；离线跑请给 --jetson/--board",
                  file=sys.stderr)
            return 2
        if not jt.t or not bt.t:
            print("[!!] 在线采集为空：确认 /imu/data_raw 有帧、CANopen 主站在跑", file=sys.stderr)
            return 2
        yaw = compare_yaw(jt.gz, yaw_rate_from_deltas(bt)) if jt.gz and bt.dyaw else None
        return report(jt, bt, yaw, compare_static(jt, bt), tag=f"在线 {args.duration:.0f}s")

    if not (args.jetson and args.board):
        print("[!!] 需要 --jetson 与 --board（或 --selftest / --online）", file=sys.stderr)
        return 2
    try:
        jt, bt = as_jetson(args.jetson), as_board(args.board)
    except (OSError, KeyError, ValueError, json.JSONDecodeError) as e:
        print(f"[!!] 读取失败：{e}", file=sys.stderr)
        return 2
    if not jt.t or not bt.t:
        print("[!!] 至少一路没有数据行", file=sys.stderr)
        return 2
    yaw = compare_yaw(jt.gz, yaw_rate_from_deltas(bt)) if jt.gz and bt.dyaw else None
    return report(jt, bt, yaw, compare_static(jt, bt), tag="离线")


if __name__ == "__main__":
    sys.exit(main())