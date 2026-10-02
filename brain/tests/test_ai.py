"""The gating logic is the part worth testing -- it decides what can move.

Plain asserts; run directly (`python tests/test_ai.py`) or under pytest.
"""

from __future__ import annotations

import json
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rak_ai.body import MockBody  # noqa: E402
from rak_ai.loop import run  # noqa: E402
from rak_ai.system1 import LayaDecider, StubDecider, gate, guard, resolve  # noqa: E402
from rak_ai.system2 import StubConsultant, narrows_only  # noqa: E402
from rak_ai.types import Action, Decision, Entity, SceneState, Tier  # noqa: E402


def test_guard_holds_veto():
    """A confident `continue` at a person 0.3 m away must not survive."""
    close = SceneState(t=0.0, entities=[Entity(id=1, label="person", z=0.3)])
    veto = guard(close)
    assert veto is not None and veto.kind == "stop"

    confident = Decision(action=Action(kind="continue"), source="s1", p=0.99, margin=0.9)
    assert gate(confident) is Tier.HIGH, "precondition: s1 is confident"
    assert guard(close) is not None, "guard must override regardless of tier"

    # Outside the corridor laterally -> not in the path -> no veto.
    assert guard(SceneState(t=0.0, entities=[Entity(id=1, z=0.3, x=5.0)])) is None
    assert guard(SceneState(t=0.0, entities=[Entity(id=1, z=9.0)])) is None


def test_margin_matters_as_much_as_p():
    """0.9 with a 0.01 margin is a coin flip wearing a confidence number."""
    coin_flip = Decision(action=Action(kind="stop"), source="s1", p=0.9, margin=0.01)
    assert gate(coin_flip) is Tier.MEDIUM
    assert gate(Decision(action=Action(kind="stop"), source="s1", p=0.75, margin=0.5)) is Tier.HIGH
    assert gate(Decision(action=Action(kind="stop"), source="s1", p=0.1, margin=0.9)) is Tier.LOW


def test_out_of_domain_laya_degrades_safely():
    """Malformed output is a normal path, not an exception."""
    junk_cases = [
        {"garbage": 1},
        None,
        {"answers": {"action": {"options": {}}}},  # pre-0.3.21 shape
        {"answers": {"action": {"type": "choice"}}},  # no distribution at all
        {"answers": {"action": {"choice": "stop", "probabilities": {}}}},
    ]
    for junk in junk_cases:
        d = LayaDecider._parse(junk, SceneState(t=0.0), 1.0)
        assert d.p == 0.0 and gate(d) is Tier.LOW, f"{junk!r} should degrade to LOW"

    # Well-formed, but the label is not an ActionKind. Validation is the trust
    # boundary, so it degrades instead of reaching the body.
    bad_kind = {"answers": {"action": {"choice": "levitate", "probabilities": {"levitate": 0.9, "stop": 0.1}}}}
    assert gate(LayaDecider._parse(bad_kind, SceneState(t=0.0), 1.0)) is Tier.LOW

    # Same, with no `choice` to fall back on: argmax picks the bad label.
    bad_argmax = {"answers": {"action": {"probabilities": {"levitate": 0.9, "stop": 0.1}}}}
    assert gate(LayaDecider._parse(bad_argmax, SceneState(t=0.0), 1.0)) is Tier.LOW


def test_laya_well_formed_answer_feeds_the_gate():
    """The documented 0.3.21 shape must land in gate()'s terms: p and margin."""
    raw = {
        "answers": {
            "action": {
                "type": "choice",
                "choice": "stop",
                "probabilities": {"stop": 0.80, "slow": 0.15, "continue": 0.05},
            }
        }
    }
    d = LayaDecider._parse(raw, SceneState(t=0.0), 12.0)
    assert d.action.kind == "stop"
    assert abs(d.p - 0.80) < 1e-9
    assert abs(d.margin - 0.65) < 1e-9, "margin is top minus second, not top minus floor"
    assert gate(d) is Tier.HIGH


def test_resolve_is_the_single_safety_path():
    """guard wins over a confident s1, and force_s2 must not bypass it."""
    close = SceneState(t=0.0, entities=[Entity(id=1, label="person", z=0.3)])
    confident = Decision(action=Action(kind="continue"), source="s1", p=0.99, margin=0.9)

    final, tier, veto = resolve(close, confident)
    assert veto is not None and final.source == "guard" and final.action.kind == "stop"
    assert tier is Tier.LOW, "a veto is a safety event: it always reports LOW"

    final, tier, veto = resolve(close, confident, force_s2=True)
    assert final.source == "guard", "forcing System 2 must not widen past the guard"

    # Low tier with something inside 1 m: fall back to stop, never to s1.
    near = SceneState(t=0.0, entities=[Entity(id=2, label="box", z=0.8)])
    weak = Decision(action=Action(kind="continue"), source="s1", p=0.3, margin=0.05)
    final, tier, veto = resolve(near, weak)
    assert veto is None and final.source == "fallback" and final.action.kind == "stop"
    assert tier is Tier.LOW


def test_s2_may_only_narrow():
    assert narrows_only(Action(kind="stop"), Action(kind="continue"))
    assert narrows_only(Action(kind="slow"), Action(kind="inspect"))
    assert not narrows_only(Action(kind="continue"), Action(kind="stop"))
    assert not narrows_only(Action(kind="turn_left"), Action(kind="stop"))


class _SlowConsultant(StubConsultant):
    """A consultant with realistic latency.

    An instant consultant answers within the same tick, so the scene never has
    time to move and a staleness check can never fire. Real System 2 latency is
    seconds, so testing staleness requires the delay.
    """

    def consult(self, state, s1, reason):
        time.sleep(0.05)
        return super().consult(state, s1, reason)


def test_staleness_catches_motion_not_just_ids():
    """An entity keeping its id while moving metres must read as stale.

    `cross` has one entity, constant id, moving 4.8 m/s -- so an id-only check
    passes vacuously. That is the bug this test exists for.
    """
    with tempfile.TemporaryDirectory() as td:
        log = run("cross", ticks=30, consultant=_SlowConsultant(), force_s2=10, out_dir=Path(td))
        lines = [json.loads(ln) for ln in log.path.read_text().splitlines()]

    replies = [ln for ln in lines if ln["kind"] == "s2_reply"]
    assert replies, "cross must escalate"
    assert any(r["stale"] for r in replies), (
        "scene moved metres with a constant id -- staleness must fire, not pass vacuously"
    )
    assert all(not r["applied"] for r in replies if r["stale"]), "a stale reply must never be applied"


def test_loop_escalates_and_logs():
    with tempfile.TemporaryDirectory() as td:
        log = run("approach", ticks=40, consultant=StubConsultant(), out_dir=Path(td))
        lines = [json.loads(ln) for ln in log.path.read_text().splitlines()]

    ticks = [ln for ln in lines if ln["kind"] == "tick"]
    assert len(ticks) == 40, "every tick is logged, including vetoes"

    escalations = [ln for ln in lines if ln["kind"] == "s2_reply"]
    assert escalations, "the approach scenario must trigger escalation"

    vetoes = [ln for ln in ticks if ln["veto"]]
    assert vetoes, "the entity must reach the stop zone"
    assert all(ln["final"]["source"] == "guard" for ln in vetoes)
    assert all(ln["final"]["action"]["kind"] == "stop" for ln in vetoes)

    # The corpus must be training-shaped: verbatim text + a null label slot.
    for ln in ticks:
        assert ln["state_text"] and "id=" in ln["state_text"]
        assert ln["label"] is None
        SceneState.model_validate(ln["state"])  # round-trips

    # Low tiers must be answered by the fallback, never by a low-confidence s1.
    low = [ln for ln in ticks if ln["tier"] == "low" and ln["veto"] is None]
    assert all(ln["final"]["source"] in ("fallback", "s1") for ln in low)


def test_stub_decider_is_deterministic():
    a, b = StubDecider(), StubDecider()
    m1, m2 = MockBody("cross"), MockBody("cross")
    assert [a.decide(m1.step()).action.kind for _ in range(10)] == [
        b.decide(m2.step()).action.kind for _ in range(10)
    ]


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"  ok  {name}")
    print("all tests passed")
