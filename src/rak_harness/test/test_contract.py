"""Proposal acceptance gate: every ADR-0010 D2 condition, plus the failure set."""
from rak_harness.contract import (
    KIND_NO_OP,
    KIND_REQUEST_CLARIFICATION,
    KIND_REVISE_PLAN,
    KIND_SELECT_WAYPOINT,
    check_proposal,
)

BASE = dict(task_id="t1", revision=3, request_id="r1",
            kind=KIND_SELECT_WAYPOINT, waypoint="home",
            expires_epoch_sec=200.0, active_task_id="t1",
            active_revision=3, allowed_waypoints={"home", "dock"},
            now_epoch_sec=100.0)


def test_accept_matching_proposal():
    assert check_proposal(**BASE).accepted


def test_reject_stale_task():
    assert not check_proposal(**{**BASE, "task_id": "old"}).accepted


def test_reject_moved_revision():
    assert not check_proposal(**{**BASE, "revision": 4}).accepted


def test_reject_expired():
    assert not check_proposal(**{**BASE, "now_epoch_sec": 201.0}).accepted


def test_reject_off_whitelist():
    assert not check_proposal(**{**BASE, "waypoint": "moon"}).accepted


def test_reject_unknown_kind():
    assert not check_proposal(**{**BASE, "kind": 9}).accepted


def test_revise_plan_not_consumed_yet():
    check = check_proposal(**{**BASE, "kind": KIND_REVISE_PLAN})
    assert not check.accepted


def test_clarification_carries_no_motion():
    check = check_proposal(**{**BASE, "kind": KIND_REQUEST_CLARIFICATION})
    assert check.accepted


def test_no_op_always_safe():
    assert check_proposal(**{**BASE, "kind": KIND_NO_OP}).accepted
