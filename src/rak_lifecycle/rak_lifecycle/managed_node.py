"""自有节点的统一生命周期基类（ADR-0020 §5）。

约定 —— 违反其中任一条都会在生产上付出代价，所以集中在基类里做：

- **资源在 `on_configure` 打开，I/O 在 `on_activate` 才开始**：4 GB 单板上重载模型
  是秒级到 GB 级代价，`deactivate → activate` 的原地恢复不该重付一次。
- **未激活不发布**：发布器一律 `create_lifecycle_publisher`，写话题统一走 `self.publish()`；
  靠人记得判状态早晚漏（静默空转与越权发布是同一个故障的两面）。
- **timer 由基类显式登记与取消**，不假设 rclpy 会把 timer 随激活自动启停。
- **失败必须带 reason**，并且**健康发布器在 `__init__` 就建好**：configure 失败的节点
  如果因为没有激活而不发诊断，运维侧看到的就是"进程在、状态未知"——比崩溃更难查。
  这是本基类唯一一处刻意不受生命周期约束的资源。
"""

from __future__ import annotations

import time
from collections.abc import Callable, Mapping
from typing import Any

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
import rclpy.logging


class ManagedNode(LifecycleNode):
    """`LifecycleNode` + 参数声明幂等 + timer 登记 + 诊断上报。

    子类只需要：在 `on_configure` 里声明参数并打开资源、在 `on_activate` 里
    `self.add_timer(...)`、失败时 `return self.fail("...")`。
    """

    def __init__(self, node_name: str, *, health_rate_hz: float = 1.0, **kwargs: Any) -> None:
        super().__init__(node_name, **kwargs)
        self._managed_timers: list[Any] = []
        self._reports: dict[str, DiagnosticStatus] = {}
        self._seen: dict[str, float] = {}
        self._state_name = "unconfigured"
        self._last_reason = ""
        self._health_rate_hz = float(health_rate_hz)
        # 见模块 docstring：诊断必须穿透到 configure 失败的场景，所以不受生命周期约束。
        self._health_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._health_timer = self.create_timer(
            1.0 / self._health_rate_hz, self._flush_health
        )

    # ------------------------------------------------------------------ 状态
    @property
    def active(self) -> bool:
        return self._state_name == "active"

    @property
    def state_name(self) -> str:
        return self._state_name

    # ------------------------------------------------------------------ 参数
    def declare(self, name: str, default: Any, *, descriptor: Any = None) -> Any:
        """幂等声明：cleanup→configure 会重复走这一步，第二次不得抛异常。

        `declare_parameter` 会读到 launch 以 `-p`/`--params-file` 传入的 override，
        所以声明放在 `on_configure` 不影响参数注入。
        """
        if not self.has_parameter(name):
            if descriptor is None:
                self.declare_parameter(name, default)
            else:
                self.declare_parameter(name, default, descriptor)
        return self.get_parameter(name).value

    # ------------------------------------------------------------------ 发布
    def lifecycle_publisher(self, msg_type: type, topic: str, qos: Any = 10) -> Any:
        """未激活时 `publish()` 被 rclpy 静默丢弃 —— 这正是我们要的语义（ADR-0020 §5）。

        注意：必须让 `on_activate` 走到 rclpy 的默认实现（见下面的生命周期段），
        否则发布器永远不会被启用，且**不会报任何错**。
        """
        return self.create_lifecycle_publisher(msg_type, topic, qos)

    # ------------------------------------------------------------------ timer
    def add_timer(self, period_s: float, callback: Callable[[], None]) -> Any:
        timer = self.create_timer(float(period_s), callback)
        self._managed_timers.append(timer)
        return timer

    def cancel_timers(self) -> None:
        for timer in self._managed_timers:
            timer.cancel()
            self.destroy_timer(timer)
        self._managed_timers.clear()

    def publish(self, message: Any, publisher: Any = None) -> bool:
        """受管发布：未启用就**不发**，并返回是否真的发出去了。

        为什么要这层薄封装：`LifecyclePublisher.publish()` 在未启用时会**静默丢弃**，
        调用方拿不到任何信号。绝大多数"节点活着但不发数据"的现场故障都是这么来的
        （on_configure 里建了发布器、忘写 on_activate，或反过来提前发）。
        返回值让调用方能把"没发"记进统计，而不是当作成功。
        """
        target = publisher
        if target is None:
            raise RuntimeError("publish() 需要显式 publisher=（没有全局默认发布器）")
        if hasattr(target, "is_activated") and not target.is_activated:
            return False
        target.publish(message)
        return True

    # ------------------------------------------------------------- 新鲜度判据
    def mark_seen(self, topic: str) -> None:
        """在已有的回调里调用，记录"这一帧确实进来了"的单调时刻。"""
        self._seen[topic] = time.monotonic()

    def age_ms(self, topic: str) -> float | None:
        stamp = self._seen.get(topic)
        return None if stamp is None else (time.monotonic() - stamp) * 1000.0

    # ------------------------------------------------------------------ 诊断
    def report(
        self,
        name: str,
        level: int,
        message: str,
        values: Mapping[str, str] | None = None,
    ) -> None:
        self._reports[name] = DiagnosticStatus(
            level=level,
            name=f"{self.get_name()}: {name}",
            message=message,
            hardware_id=self.get_name(),
            values=[KeyValue(key=k, value=str(v)) for k, v in sorted((values or {}).items())],
        )

    def fail(self, reason: str) -> TransitionCallbackReturn:
        """转移失败的唯一出口：把可读原因写进诊断（ADR-0020 §5 最后一条）。"""
        self._last_reason = reason
        self.report(
            "lifecycle",
            DiagnosticStatus.ERROR,
            f"{self._state_name}: {reason}",
            {"reason": reason},
        )
        rclpy.logging.get_logger(self.get_name()).error(
            f"[lifecycle] {self._state_name} failed: {reason}"
        )
        return TransitionCallbackReturn.FAILURE

    def _flush_health(self) -> None:
        statuses = list(self._reports.values())
        statuses.append(
            DiagnosticStatus(
                level=DiagnosticStatus.OK if self.active else DiagnosticStatus.WARN,
                name=f"{self.get_name()}: state",
                message=self._state_name + (f": {self._last_reason}" if self._last_reason else ""),
                hardware_id=self.get_name(),
                values=[KeyValue(key="state", value=self._state_name)]
                + [
                    KeyValue(
                        key=f"age_ms/{topic}",
                        value="never" if age is None else f"{age:.0f}",
                    )
                    for topic, age in sorted(
                        (topic, self.age_ms(topic)) for topic in self._seen
                    )
                ],
            )
        )
        self._health_pub.publish(DiagnosticArray(status=statuses))

    # -------------------------------------------------------------- 生命周期
    # rclpy 的 LifecycleNode 在这些默认实现里逐个驱动**受管实体**
    # （LifecyclePublisher 的 enable/disable 就是这么发生的）。覆盖钩子却不调 super()
    # 的话，发布器永远停在未启用状态，`publish()` 被 `when_enabled` 静默丢弃 ——
    # 节点看起来完全正常，但一帧都不发。这是本轮在 Orin 上实测抓到的坑。
    def _chain(self, hook: str, state) -> "TransitionCallbackReturn":
        ret = getattr(super(), hook)(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            self.report(
                "lifecycle",
                DiagnosticStatus.ERROR,
                f"{self._state_name}: 受管实体的 {hook}() 未成功",
                {"entity_hook": hook},
            )
        return ret

    def on_configure(self, state) -> TransitionCallbackReturn:
        ret = self._chain("on_configure", state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret
        self._state_name = "inactive"
        self._last_reason = ""
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        ret = self._chain("on_cleanup", state)
        self._state_name = "unconfigured"
        self.cancel_timers()
        return ret

    def on_activate(self, state) -> TransitionCallbackReturn:
        ret = self._chain("on_activate", state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret
        self._state_name = "active"
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        ret = self._chain("on_deactivate", state)
        self._state_name = "inactive"
        self.cancel_timers()
        return ret

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        ret = self._chain("on_shutdown", state)
        self.cancel_timers()
        return ret
