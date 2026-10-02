"""System 1: the fast path. A direct function call -- no framework, ever.

Three things live here, and they are deliberately separate:

  guard()   -- deterministic geometric veto. Holds real authority.
  gate()    -- escalation policy. Holds NO authority, only decides how loudly
               we ask for help.
  resolve() -- combines the two into what actually gets executed. One place,
               so the CLI loop and the ROS node cannot drift apart: two copies
               of the safety path would eventually disagree.

The split matters because Laya is out-of-domain (trained on invoices, customer
service, security incidents and agent traces) with ECE 0.213 and 0.362
zero-shot accuracy. A model with that calibration cannot be the thing that
authorises motion, so nothing here trusts its confidence. See gate().
"""

from __future__ import annotations

import os
import time
from typing import Protocol

from .types import Action, Decision, Entity, SceneState, Tier

# Calibration knobs (metres). The real robot needs these tuned against its own
# geometry -- a minimal model cannot see the actual stopping distance.
STOP_ZONE_M = 0.6
CORRIDOR_M = 0.5

# Gate thresholds. GUESSES, not measurements -- they get fitted from
# logged episodes once there is data.
# ponytail: hand-set thresholds; fitted from brain/data/episodes once it exists.
HIGH_P, HIGH_MARGIN, MEDIUM_P = 0.70, 0.30, 0.40

# The one question Laya is asked.
#
# Its `choice` head shares a fixed token budget (`head_max_len`, 256 on
# laya-typed-decisions) across all options, so accuracy collapses with a large
# label space: stay well under ~20 options. Seven ActionKind labels fit with
# room for a description each.
#
# Labels are semantic and MUST be a subset of ActionKind -- `Action` is the
# trust boundary, so a label it does not recognise degrades to LOW instead of
# reaching the body. Do not swap them for boolean-word labels (yes/no,
# true/false): Laya's own limits say the checkpoints follow those labels rather
# than the criteria, and semantic labels are not negation-safe either, which is
# why the criteria below are phrased as instructions, not as restatements.
ACTION_CRITERIA: dict[str, str] = {
    "stop": "halt now, something is in or entering the path",
    "slow": "hazard ahead but not immediate, reduce speed and keep watching",
    "continue": "path clear, nothing to do",
    "turn_left": "path blocked, steer left around it",
    "turn_right": "path blocked, steer right around it",
    "inspect": "the scene is ambiguous or degraded, look before committing",
    "ask_operator": "no safe autonomous choice, hand it to a human",
}

QUESTIONS: dict[str, dict[str, object]] = {
    "action": {
        "type": "choice",
        "instructions": "Which action should the robot take for this scene?",
        "criteria": ACTION_CRITERIA,
    }
}


def guard(state: SceneState) -> Action | None:
    """Deterministic veto. Runs every tick. Overrides System 1 AND System 2.

    This is the only component allowed to authorise a stop on safety grounds.
    A high-confidence Laya saying `continue` at a person 0.3 m away is
    overridden here -- that is the entire point of not trusting the model.
    """
    for e in state.entities:
        if 0.0 < e.z < STOP_ZONE_M and abs(e.x) < CORRIDOR_M:
            return Action(kind="stop", target_id=e.id, reason=f"{e.label} in stop zone at {e.z:.2f} m")
    return None


def gate(d: Decision) -> Tier:
    """Escalation policy only. Confidence never authorises anything.

    Note `margin` carries as much weight as `p`: a 0.45/0.43 split is a coin
    flip wearing a confident-looking number, and `p` alone cannot see that.
    """
    if d.p >= HIGH_P and d.margin >= HIGH_MARGIN:
        return Tier.HIGH
    if d.p >= MEDIUM_P:
        return Tier.MEDIUM
    return Tier.LOW


def resolve(state: SceneState, s1: Decision, *, force_s2: bool = False) -> tuple[Decision, Tier, Action | None]:
    """Turn a System 1 proposal into what actually gets executed.

    Returns `(final decision, tier to report, veto or None)`.

    This is policy, not authority -- `guard()` is still the only thing that can
    override a model on safety grounds, and it is called exactly once here, on
    the state this tick is about. It exists so `loop.py` and `node.py` run the
    same safety path; a second copy would eventually disagree with the first.

    A veto reports LOW whatever the model's confidence was: a safety event is
    always worth telling System 2 about.
    """
    veto = guard(state)
    tier = gate(s1)
    if veto is not None:
        return Decision(action=veto, source="guard", latency_ms=s1.latency_ms, t=state.t), Tier.LOW, veto
    if force_s2:
        return s1, Tier.LOW, None
    if tier in (Tier.HIGH, Tier.MEDIUM):
        return s1, tier, None
    # Low confidence: fall back to something that does not depend on the model.
    # Inside 1 m that is a stop, not a coin flip -- an unvalidated s1 output may
    # not be what lets the robot keep moving.
    fallback = Decision(
        action=Action(kind="stop", reason="unvalidated s1 output")
        if any(0 < e.z < 1.0 for e in state.entities)
        else Action(kind="continue", reason="s1 low confidence, path clear"),
        source="fallback",
        t=state.t,
    )
    return fallback, Tier.LOW, None


class Decider(Protocol):
    def decide(self, state: SceneState) -> Decision: ...


class StubDecider:
    """Rule-based stand-in so the loop runs end to end without Laya.

    This is a REQUIRED component, not a convenience: `torch` on aarch64/JetPack
    is a real install risk, and nothing about the architecture can be verified
    if the whole loop refuses to start without a 421M model.
    """

    def decide(self, state: SceneState) -> Decision:
        t0 = time.perf_counter()
        nearest = min((e for e in state.entities if e.z > 0), key=lambda e: e.z, default=None)
        if nearest is None:
            action, p, margin = Action(kind="continue", reason="path clear"), 0.9, 0.8
        elif nearest.z < 1.5:
            action, p, margin = Action(kind="slow", target_id=nearest.id, reason="closing"), 0.6, 0.2
        else:
            action, p, margin = Action(kind="continue", reason="distant object"), 0.75, 0.45
        return Decision(
            action=action,
            source="s1",
            p=p,
            margin=margin,
            latency_ms=(time.perf_counter() - t0) * 1000,
            t=state.t,
        )


class LayaDecider:
    """Laya adapter. Loads once, predicts per tick.

    The shape below is `laya` 0.3.21's documented one:

        {"answers": {"action": {"type": "choice",
                                "choice": "slow",     # the model's argmax
                                "probabilities": {"slow": 0.71, ...},
                                "confidence": ...,    # 1 - normalised entropy
                                "answer_confidence": ...}},
         "usage": {"input_tokens": ..., "output_tokens": ...}}

    `probabilities` is what we read: `Decision.p` means "the top option's raw
    probability" and `gate()` is tuned in those terms. `confidence` is a
    different quantity (entropy, not p-max) so it is deliberately ignored --
    a threshold carried over from another model would not transfer.

    Everything is read defensively: Laya is out-of-domain by construction, so a
    malformed answer is a normal path that degrades to LOW, never an exception.
    """

    def __init__(self, model: str = "convaiinnovations/laya-typed-decisions") -> None:
        # Must precede the import: a TF/abseil deadlock hangs loading otherwise.
        os.environ.setdefault("USE_TF", "0")
        import laya  # noqa: PLC0415 -- deliberately deferred, see USE_TF above

        self._agent = laya.load(model)
        # laya falls back to CPU on CUDA OOM *without raising* -- on this 3.4 GB
        # Jetson that is ~190 ms -> ~1.6 s per predict, slower than the sensor
        # rate. Whoever runs us needs to know which one they got.
        self.device = str(getattr(self._agent, "device", "unknown")).split(":")[0]

    def decide(self, state: SceneState) -> Decision:
        t0 = time.perf_counter()
        raw = self._agent.predict(state.to_text(), QUESTIONS)
        latency = (time.perf_counter() - t0) * 1000
        return self._parse(raw, state, latency)

    @staticmethod
    def _parse(raw: object, state: SceneState, latency: float) -> Decision:
        """Coerce Laya's output through Action validation.

        Anything that fails -- wrong shape, no probabilities, or a label
        `Action` rejects -- degrades to LOW with an `inspect` action and gets
        escalated. That is the safe direction to fail in, and it is a normal
        path rather than an error: this model was trained on other domains.
        """
        fallback = Decision(
            action=Action(kind="inspect", reason="unparseable laya output"),
            source="s1",
            p=0.0,
            margin=0.0,
            latency_ms=latency,
            t=state.t,
        )
        try:
            answers = raw["answers"]["action"]  # type: ignore[index]
            probs = answers["probabilities"]  # type: ignore[index]
            if not isinstance(probs, dict) or not probs:
                raise ValueError("no probabilities")
            ranked = sorted(((str(k), float(v)) for k, v in probs.items()), key=lambda kv: -kv[1])
            by_kind = dict(ranked)

            # Prefer the model's own argmax when it reports one; fall back to
            # the argmax of the distribution. Either way the label still goes
            # through `Action`, so an unknown one degrades instead of acting.
            choice = answers.get("choice")  # type: ignore[union-attr]
            kind = choice if isinstance(choice, str) and choice in by_kind else ranked[0][0]
            p = by_kind[kind]
            margin = p - max(v for k, v in ranked if k != kind) if len(ranked) > 1 else p

            return Decision(
                action=Action.model_validate({"kind": kind, "reason": "laya"}),
                source="s1",
                p=p,
                margin=margin,
                latency_ms=latency,
                t=state.t,
            )
        except Exception:
            return fallback


def demo() -> None:
    """Self-check: the veto beats a confident model, and the gate gates."""
    from .body import MockBody

    m = MockBody("approach")
    vetoed = None
    for _ in range(40):
        s = m.step()
        if (v := guard(s)) is not None:
            vetoed = v
            break
    assert vetoed is not None and vetoed.kind == "stop", "guard must fire as the entity enters"
    assert vetoed.target_id == 3

    # A confident `continue` does NOT survive the guard.
    close = SceneState(t=0.0, entities=[Entity(id=1, label="person", z=0.3)])
    assert guard(close) is not None

    d = StubDecider().decide(SceneState(t=0.0))
    assert d.action.kind == "continue" and gate(d) is Tier.HIGH
    assert gate(Decision(action=Action(kind="stop"), source="s1", p=0.45, margin=0.02)) is Tier.MEDIUM
    assert gate(Decision(action=Action(kind="stop"), source="s1", p=0.1, margin=0.5)) is Tier.LOW
    assert gate(Decision(action=Action(kind="stop"), source="s1", p=0.9, margin=0.01)) is Tier.MEDIUM, (
        "high p with a tiny margin is a coin flip, not confidence"
    )

    bad = LayaDecider._parse({"garbage": True}, SceneState(t=0.0), 1.0)
    assert bad.p == 0.0 and gate(bad) is Tier.LOW, "unparseable output must degrade to LOW"

    # A well-formed 0.3.21 response parses into the confidence gate's terms.
    ok = LayaDecider._parse(
        {
            "answers": {
                "action": {
                    "type": "choice",
                    "choice": "stop",
                    "probabilities": {"stop": 0.80, "slow": 0.15, "continue": 0.05},
                }
            }
        },
        SceneState(t=0.0),
        12.0,
    )
    assert ok.action.kind == "stop" and abs(ok.p - 0.80) < 1e-9 and abs(ok.margin - 0.65) < 1e-9

    # resolve(): the veto beats a confident model, and reports LOW.
    confident = Decision(action=Action(kind="continue"), source="s1", p=0.99, margin=0.9)
    final, tier, veto = resolve(close, confident)
    assert veto is not None and final.source == "guard" and final.action.kind == "stop"
    assert tier is Tier.LOW, "a veto is a safety event: always tell System 2"

    far = SceneState(t=0.0, entities=[Entity(id=1, label="person", z=3.0)])
    final, tier, veto = resolve(far, confident)
    assert veto is None and final.source == "s1" and tier is Tier.HIGH

    print("system1.py ok")


if __name__ == "__main__":
    demo()
