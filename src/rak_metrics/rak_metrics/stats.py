"""分位数与丢帧统计 —— 纯 Python，不依赖 ROS，因此可以离线跑单测。

为什么不用 numpy：这个节点要在 1 kHz 的 IMU 流旁边常驻，每 200 个样本排一次
200 元素列表的代价可以忽略，而多一个 numpy 依赖会让"桌面档能不能装"再多一个变量。
"""

from __future__ import annotations

from collections import deque
import math


def percentile(sorted_values: list[float], q: float) -> float:
    """线性插值分位数（与 numpy 的默认 linear 方法一致）。空列表返回 nan。"""
    if not sorted_values:
        return float("nan")
    if len(sorted_values) == 1:
        return float(sorted_values[0])
    position = (len(sorted_values) - 1) * max(0.0, min(1.0, q))
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return float(sorted_values[int(position)])
    weight = position - lower
    return float(sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight)


class Window:
    """固定长度滑窗，给出 p50/p95/p99、均值与最大间隔。

    ``gaps_over`` 用来估丢帧：间隔超过``nominal_period_s * factor`` 记一次，
    这是**估计**而不是计数 —— 话题里没有序号时，丢帧与"晚到"无法区分，
    所以键名一律写 ``gap_events``，不写 ``drops``（AGENTS §4 的事实分级）。
    """

    def __init__(self, size: int = 200, nominal_period_s: float | None = None,
                 gap_factor: float = 2.5) -> None:
        self._values: deque[float] = deque(maxlen=int(size))
        self._last_stamp: float | None = None
        self._gap_events = 0
        self._nominal = nominal_period_s
        self._gap_factor = float(gap_factor)

    def add(self, value_ms: float) -> None:
        if value_ms != value_ms or value_ms in (float("inf"), float("-inf")):
            return
        self._values.append(float(value_ms))

    def observe_gap(self, stamp_s: float) -> bool:
        """喂一个到达时刻，返回该间隔是否被判为"异常长"。"""
        previous = self._last_stamp
        self._last_stamp = stamp_s
        if previous is None or self._nominal is None:
            return False
        if stamp_s - previous > self._nominal * self._gap_factor:
            self._gap_events += 1
            return True
        return False

    @property
    def gap_events(self) -> int:
        return self._gap_events

    def summary(self) -> dict[str, float]:
        values = sorted(self._values)
        if not values:
            return {"count": 0}
        return {
            "count": len(values),
            "p50": percentile(values, 0.50),
            "p95": percentile(values, 0.95),
            "p99": percentile(values, 0.99),
            "mean": sum(values) / len(values),
            "max": values[-1],
            "gap_events": self._gap_events,
        }


class RateMeter:
    """按到达时刻估算频率（滑动窗口，避免"启动瞬间不计"的假象）。"""

    def __init__(self, window_s: float = 5.0) -> None:
        self._window = float(window_s)
        self._times: deque[float] = deque()

    def tick(self, now_s: float) -> float:
        self._times.append(now_s)
        while self._times and now_s - self._times[0] > self._window:
            self._times.popleft()
        if len(self._times) < 2:
            return 0.0
        span = self._times[-1] - self._times[0]
        return 0.0 if span <= 0 else (len(self._times) - 1) / span


class FlatnessTracker:
    """低速质量指标：反馈台阶统计（rak/docs/chassis-control-design.md §6）。

    台架现象：M 法测速在低速段 1 count 量化占读数 >50%，反馈呈"长时间完全
    平坦 + 偶发大台阶"（2026-10-30 实测 82% 平坦样本、每轮 270-394 次跳变）。
    三个数：flat_ratio（全体样本中与上一样本完全相等的占比）、
    flat_ratio_moving（只在 |v| > moving_eps 的样本上算——停着不动全平坦，
    没有意义）、max_jump（相邻样本最大单步跳变）。

    basis：moving_eps 是 placeholder，待落地期用 rak_metrics 首日基线标定。
    """

    def __init__(self, size: int = 200, moving_eps: float = 0.05) -> None:
        self._moving_eps = abs(float(moving_eps))
        self._flat = deque(maxlen=int(size))          # 1=平坦(含静止), 0=变化
        self._flat_moving = deque(maxlen=int(size))   # 只含 |v|>eps 的样本
        self._prev: float | None = None
        self._max_jump = 0.0
        self._jump_events = 0

    def observe(self, value: float) -> None:
        if value != value or value in (float("inf"), float("-inf")):
            return
        v = float(value)
        if self._prev is not None:
            flat = v == self._prev
            self._flat.append(1 if flat else 0)
            if abs(v) > self._moving_eps or abs(self._prev) > self._moving_eps:
                self._flat_moving.append(0 if flat else 1)
            jump = abs(v - self._prev)
            if jump > self._max_jump:
                self._max_jump = jump
            if jump > 0.0:
                self._jump_events += 1
        self._prev = v

    def summary(self) -> dict[str, float]:
        out: dict[str, float] = {"flat_count": float(len(self._flat))}
        if self._flat:
            out["flat_ratio"] = sum(self._flat) / len(self._flat)
        if self._flat_moving:
            out["flat_ratio_moving"] = sum(self._flat_moving) / len(self._flat_moving)
        out["flat_max_jump"] = self._max_jump
        out["flat_jump_events"] = float(self._jump_events)
        return out
