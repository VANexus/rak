"""Voice dialogue loop: turn-based state machine (stdlib-only, unit-tested).

GPT-voice 式体验,但具身约束不同:轮次制(非全双工打断),唤醒门控注意力
窗口(ASR 小模型幻觉多,无门控会乱动),Harness 只发结构化目标(导航文本/
控制),永不发速度/关节。

Events in (plain data) -> commands out (plain data); node.py 只做 ROS 接线。
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

TERMINAL_STATES = frozenset({"SUCCEEDED", "FAILED", "CANCELED", "SAFETY_HOLD"})


@dataclass
class DialogueCommands:
    # (template_key, slot_text): node renders via persona.render().
    say: list[tuple[str, str]] = field(default_factory=list)
    submit_task: list[str] = field(default_factory=list)  # navigation texts
    control: list[tuple[str, str]] = field(default_factory=list)  # (task_id, op)
    events: list[tuple[str, str, str]] = field(default_factory=list)  # (task_id, kind, text)


@dataclass
class ActiveTaskView:
    task_id: str = ""
    revision: int = 0
    state: str = "IDLE"


class DialogueEngine:
    """IDLE -> ATTENTIVE (wake opens a window) -> ACT -> SPEAK(outcome)."""

    def __init__(self, *, attention_sec: float = 12.0, clarify_cooldown_sec: float = 10.0,
                 ack_cooldown_sec: float = 4.0, ack_min_score: float = 580.0):
        self._attention_sec = attention_sec
        self._clarify_cooldown = clarify_cooldown_sec
        self._ack_cooldown = ack_cooldown_sec
        self._ack_min_score = ack_min_score
        self._attentive_until = 0.0
        self._active = ActiveTaskView()
        self._last_clarify = 0.0
        self._last_ack = 0.0
        self._recent_intents: deque[str] = deque(maxlen=5)

    def is_attentive(self, now: float) -> bool:
        return now <= self._attentive_until

    # -- events ------------------------------------------------------------

    def on_wake(self, *, now: float, score: float) -> DialogueCommands:
        """唤醒:开注意力窗口;出声应答有两道闸,都只压应答不压窗口。

        板上阈值 500 贴地,实测环境语音成片擦线触发(501-546),而应答走即时层
        毫秒级出声,误唤醒从"看不见"变成"吵"。闸一:score < ack_min_score 不应答
        (实测真唤醒 589+、误触发堆 501-546,580 落在缝里);闸二:ack_cooldown
        内的连环触发只回第一声。注意力窗口照开,方位箭头(vtn→/speech/wake→viz)
        不经这里,不受影响。
        """
        self._attentive_until = now + self._attention_sec
        out = DialogueCommands()
        if score >= self._ack_min_score and now - self._last_ack >= self._ack_cooldown:
            self._last_ack = now
            out.say.append(("ack_wake", ""))
        return out

    def on_task_status(self, *, task_id: str, revision: int, state: str,
                       reason: str, dest: str, vehicle_ok: bool,
                       vehicle_note: str, now: float) -> DialogueCommands:
        out = DialogueCommands()
        if task_id and (not self._active.task_id or task_id == self._active.task_id):
            self._active = ActiveTaskView(task_id, revision, state)
        elif task_id:
            self._active = ActiveTaskView(task_id, revision, state)
        if state in TERMINAL_STATES and self._active.task_id == task_id:
            if state == "SUCCEEDED":
                out.say.append(("done", dest))
                out.events.append((task_id, "PROGRESS", f"到达{dest}"))
            elif state == "SAFETY_HOLD":
                out.say.append(("fault_hold", vehicle_note or reason))
                out.events.append((task_id, "RECOVERY", reason))
            elif state == "CANCELED":
                out.say.append(("ack_cancel", ""))
                out.events.append((task_id, "PROGRESS", "任务已取消"))
            else:
                out.say.append(("failed", reason))
                out.events.append((task_id, "RECOVERY", reason))
            self._active = ActiveTaskView()
        return out

    def on_intent(self, *, kind: str | None, text: str, destination: str,
                  command: str | None, now: float) -> DialogueCommands:
        """kind: navigate | control | None(unknown). command: pause/resume/cancel."""
        out = DialogueCommands()
        self._recent_intents.append(text)
        if kind == "navigate":
            if not self.is_attentive(now):
                out.say.append(("need_wake", ""))
                out.events.append(("", "CLARIFICATION", f"门外意图已忽略:{text}"))
                return out
            out.submit_task.append(text)
            out.say.append(("ack_navigate", destination))
            out.events.append(("", "ACK", f"接受语音任务:{text}"))
            return out
        if kind == "control":
            if not self._active.task_id:
                out.say.append(("no_task", ""))
                return out
            out.control.append((self._active.task_id, command or ""))
            key = {"pause": "ack_pause", "resume": "ack_resume",
                   "cancel": "ack_cancel"}.get(command or "", "ack_pause")
            out.say.append((key, ""))
            out.events.append((self._active.task_id, "ACK", f"语音控制:{command}"))
            return out
        if now - self._last_clarify >= self._clarify_cooldown:
            self._last_clarify = now
            out.say.append(("unknown", ""))
            out.events.append(("", "CLARIFICATION", f"未理解:{text}"))
        return out
