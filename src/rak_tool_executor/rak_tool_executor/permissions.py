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
        return Permission("rejected", "motion-class tool requires operator confirmation first")
    return p
