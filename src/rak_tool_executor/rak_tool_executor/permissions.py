"""Permission policy: safety class -> decision. Deterministic, stdlib-only.

Maps the design's authority table to a pure function so it is testable
without ROS. The node applies this before any handler runs; the loop's own
approval UX is a convenience layer on top, never the authority.
"""

from __future__ import annotations

from dataclasses import dataclass

from .tool_registry import SAFETY_MOTION, SAFETY_MOTION_OBJECT, SAFETY_OPERATOR, SAFETY_READ, SAFETY_SPEAK

AUTO = "auto"          # no confirmation needed
CONFIRM = "confirm"    # requires operator_confirmed on the proposal
DENY = "deny"          # loop may never propose this


@dataclass(frozen=True)
class Permission:
    mode: str
    reason: str = ""


def policy_for(safety: str) -> Permission:
    if safety in (SAFETY_READ, SAFETY_SPEAK):
        return Permission(AUTO)
    if safety in (SAFETY_MOTION, SAFETY_MOTION_OBJECT):
        return Permission(CONFIRM)
    if safety == SAFETY_OPERATOR:
        return Permission(DENY, "operator-only hardware gate; not part of the tool menu")
    return Permission(DENY, f"unknown safety class '{safety}'")


def evaluate(safety: str, operator_confirmed: bool) -> Permission:
    """Decision for a concrete proposal. Confirmation is recorded in audit."""
    p = policy_for(safety)
    if p.mode == CONFIRM and not operator_confirmed:
        return Permission("rejected", "motion-class tool requires operator_confirmed first")
    return p


def chassis_interlock(present_nodes, watched_nodes) -> list[str]:
    """底盘单一写者互锁：返回在场且会写底盘的 Nav2 节点名。

    为什么用"节点在场"而不是"Nav2 正在导航"：Nav2 是随时可以起速度的写者，
    空闲时也在场。判定成本为零（读图），不依赖 Nav2 的 action 反馈；代价是
    Nav2 起了但没在跑时也会挡住底盘工具 —— 这是安全侧的错误方向，要放行就显式
    `nav2_interlock:=false`，那一动作会进日志与审计。
    """
    present = set(present_nodes)
    return sorted(name for name in watched_nodes if name in present)

