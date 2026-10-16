"""Long-context engineering for the agent loop (stdlib-only).

The loop is stateless per query(); conversation continuity lives here, in a
digest we compose into each turn's user message. That keeps the exact
context the model sees under our control (and inspectable in the audit):

  * rolling window: last N turns verbatim, older turns collapsed to one-liners
  * tool results: last K full, older single-line (detail is in the audit)
  * budget: CJK-aware token estimate; overflow triggers deterministic
    local compaction (never an LLM round-trip)
  * memory: tails of the episode corpus + tool-event audit fold in as
    "recently happened" one-liners, so the robot recalls yesterday
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


def estimate_tokens(text: str) -> int:
    """Cheap CJK-aware estimate: 1 token per CJK char, ~0.3 per ASCII char."""
    cjk = sum(1 for ch in text if "\u4e00" <= ch <= "\u9fff" or ch in "，。！？；：、")
    other = len(text) - cjk
    return int(cjk + other * 0.3)


@dataclass
class _Turn:
    role: str            # user | assistant | tool
    text: str
    full: bool = True


@dataclass
class SessionDigest:
    max_tokens: int = 3000
    keep_recent_turns: int = 6
    tool_full_results: int = 3
    memory_lines: int = 4
    memory_dir: str = "data/episodes"
    turns: list[_Turn] = field(default_factory=list)
    context: dict[str, Any] = field(default_factory=dict)
    _memory_cache: tuple[float, list[str]] = (0.0, [])

    # ------------------------------------------------------------- mutations

    def add_user(self, text: str) -> None:
        self.turns.append(_Turn("user", text.strip()))

    def add_assistant(self, text: str) -> None:
        text = text.strip()
        if text:
            self.turns.append(_Turn("assistant", text))

    def add_tool(self, name: str, ok: bool, summary: str) -> None:
        mark = "ok" if ok else "fail"
        self.turns.append(_Turn("tool", f"{name}[{mark}] {summary}"))

    def update_context(self, payload: dict[str, Any]) -> None:
        self.context.update(payload)

    # ------------------------------------------------------------- rendering

    def _compact(self) -> list[_Turn]:
        """Keep the recent window verbatim, collapse older turns to one-liners."""
        if len(self.turns) <= self.keep_recent_turns:
            return self.turns
        old, recent = self.turns[:-self.keep_recent_turns], self.turns[-self.keep_recent_turns:]
        tools = [t for t in old if t.role == "tool"]
        users = [t for t in old if t.role == "user"]
        collapsed: list[_Turn] = []
        if users:
            collapsed.append(_Turn(
                "user", "（此前聊过：" + "；".join(t.text[:18] for t in users[-4:]) + "）"))
        if tools:
            collapsed.append(_Turn(
                "tool", "（此前的工具调用：" + "；".join(t.text[:24] for t in tools[-4:]) + "）"))
        return collapsed + recent

    def _trim_tool_details(self, turns: list[_Turn]) -> list[_Turn]:
        kept = 0
        out: list[_Turn] = []
        for turn in reversed(turns):
            if turn.role == "tool" and turn.full:
                kept += 1
                if kept > self.tool_full_results:
                    turn = _Turn("tool", turn.text.split(" ")[0] + " …")
            out.append(turn)
        return list(reversed(out))

    def _context_lines(self) -> list[str]:
        lines = []
        if self.context.get("vehicle"):
            lines.append(f"[车况] {self.context['vehicle']}")
        if self.context.get("decision"):
            lines.append(f"[决策上下文] {self.context['decision']}")
        if self.context.get("wake"):
            lines.append(f"[唤醒] {self.context['wake']}")
        return lines

    def _memory_lines(self) -> list[str]:
        lines = self._read_memory()
        return [f"[近期发生] {line}" for line in lines[-self.memory_lines:]]

    def render(self, utterance: str) -> str:
        """The user message for this turn: context + memory + history + utterance."""
        self.add_user(utterance)
        parts = self._context_lines() + self._memory_lines()
        history = self._trim_tool_details(self._compact())
        if history:
            recent = "\n".join(
                f"{'用户' if t.role == 'user' else '助手' if t.role == 'assistant' else '工具'}: {t.text}"
                for t in history
            )
            parts.append("[最近对话]\n" + recent)
        parts.append(f"[用户此刻说] {utterance.strip()}")

        message = "\n".join(parts)
        # Hard budget: if still oversized, drop history (context+matter most).
        while estimate_tokens(message) > self.max_tokens and "[最近对话]" in message:
            message = message[: message.index("[最近对话]")] + \
                f"[用户此刻说] {utterance.strip()}"
            break
        return message

    # ------------------------------------------------------------- memory

    def _read_memory(self) -> list[str]:
        """Tail-read the audit + episode corpus; cached for 30 s."""
        now = time.time()
        if now - self._memory_cache[0] < 30.0:
            return self._memory_cache[1]
        lines: list[str] = []
        base = Path(self.memory_dir)
        if base.is_dir():
            for path in sorted(base.glob("tool_events-*.jsonl"))[-1:]:
                lines.extend(self._tail_lines(path, "tool_events"))
            for path in sorted(base.glob("episodes-*.jsonl"))[-1:]:
                lines.extend(self._tail_lines(path, "episodes"))
        self._memory_cache = (now, lines)
        return lines

    @staticmethod
    def _tail_lines(path: Path, kind: str) -> list[str]:
        try:
            with path.open("rb") as fh:
                fh.seek(0, 2)
                size = fh.tell()
                fh.seek(max(0, size - 65536))
                tail = fh.read().decode("utf-8", "ignore").splitlines()
        except OSError:
            return []
        out: list[str] = []
        for line in tail[-12:]:
            try:
                record = json.loads(line)
            except ValueError:
                continue
            stamp = time.strftime("%m-%d %H:%M", time.localtime(record.get("t", 0)))
            if kind == "tool_events":
                out.append(f"{stamp} {record.get('kind', '?')}:{record.get('tool', record.get('label', ''))}"
                           f" {record.get('status', record.get('reason', ''))}".strip())
            else:
                out.append(f"{stamp} {record.get('kind', '?')}:{str(record.get('state_text', ''))[:40]}")
        return out
