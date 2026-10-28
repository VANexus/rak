"""Expression layer: reserved contract + gated no-op stub (ADR-0010 D4).

硬件门全关:本轮唯一实现恒 no-op,门恒返回 False。软件侧“呼吸感”走
/harness/event,不走这里。
"""
from __future__ import annotations

from dataclasses import dataclass

EXPRESSION_INTENT_TOPIC = "/expression/intent"  # reserved, no publisher yet
EXPRESSION_STATE_TOPIC = "/expression/state"  # reserved, no publisher yet

ACKNOWLEDGE = "ACKNOWLEDGE"
THINKING = "THINKING"
PAUSE = "PAUSE"
ORIENT_ATTENTION = "ORIENT_ATTENTION"
IDLE_POSTURE = "IDLE_POSTURE"
SPEAK = "SPEAK"

KINDS = frozenset({ACKNOWLEDGE, THINKING, PAUSE, ORIENT_ATTENTION,
                   IDLE_POSTURE, SPEAK})


@dataclass
class ExpressionIntent:
    schema_version: int = 1
    kind: str = ACKNOWLEDGE
    ttl_sec: float = 2.0
    text: str = ""
    reason: str = ""
    requires_idle_resources: bool = True


class ExpressionGate:
    """Hardware gates are closed (see docs/hardware_acceptance.md)."""

    def allowed(self, intent: ExpressionIntent) -> bool:
        if intent.kind not in KINDS:
            return False
        return False  # 硬件门未过:恒 no-op,理由见 state()


class NoOpExpressionSink:
    def send(self, intent: ExpressionIntent) -> str:
        return f"no-op ({intent.kind}): hardware gates closed"
