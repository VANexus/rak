"""Harness arbitration seam (ADR-0010): versioned proposal contract.

stdlib-only at import time, same rule as rak_intent: unit tests run on any
machine without ROS installed.
"""
from __future__ import annotations

from dataclasses import dataclass

SCHEMA_VERSION = 1

KIND_NO_OP = 0
KIND_SELECT_WAYPOINT = 1
KIND_REVISE_PLAN = 2
KIND_REQUEST_CLARIFICATION = 3

KINDS = frozenset({KIND_NO_OP, KIND_SELECT_WAYPOINT, KIND_REVISE_PLAN,
                   KIND_REQUEST_CLARIFICATION})


@dataclass
class ProposalCheck:
    accepted: bool
    reason: str


def is_expired(expires_epoch_sec: float, now_epoch_sec: float) -> bool:
    """A proposal whose deadline passed must never be acted on."""
    return now_epoch_sec > expires_epoch_sec


def check_proposal(
    *,
    task_id: str,
    revision: int,
    request_id: str,
    kind: int,
    waypoint: str,
    expires_epoch_sec: float,
    active_task_id: str,
    active_revision: int,
    allowed_waypoints: object,
    now_epoch_sec: float,
) -> ProposalCheck:
    """Executor-side acceptance gate. Every condition is required (ADR-0010 D2).

    `allowed_waypoints` is any container of known alias strings. On any
    mismatch the proposal is discarded -- never partially applied.
    """
    if task_id != active_task_id:
        return ProposalCheck(False, "task_id mismatch (stale task)")
    if revision != active_revision:
        return ProposalCheck(False, "revision moved while proposal was pending")
    if is_expired(expires_epoch_sec, now_epoch_sec):
        return ProposalCheck(False, "proposal expired")
    if kind not in KINDS:
        return ProposalCheck(False, f"unknown kind {kind}")
    if kind == KIND_SELECT_WAYPOINT:
        if not waypoint or waypoint not in allowed_waypoints:
            return ProposalCheck(False, f"waypoint {waypoint!r} not whitelisted")
    if kind == KIND_REVISE_PLAN:
        # P1: plan revision is defined but not yet consumed by the executor.
        return ProposalCheck(False, "REVISE_PLAN not consumed yet")
    if kind == KIND_REQUEST_CLARIFICATION:
        return ProposalCheck(True, "clarification (no motion)")
    return ProposalCheck(True, "NO_OP (no motion)")
