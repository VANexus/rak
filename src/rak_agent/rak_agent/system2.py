"""System 2: the slow path. RAI lives here and nowhere else.

Everything RAI-shaped is behind the `Consultant` protocol, so the loop never
depends on how the slow path is implemented.

On the choice to use RAI's LLM plumbing rather than its agent loop: RAI's
`ReActAgent.run()` starts a background thread and returns nothing -- it is a
service driven by connectors, with no synchronous entry point. A control loop
needs request/response, so we take the half of RAI that fits (`get_llm_model`,
its vendor-agnostic config layer) and call the model directly.

Two invariants this module enforces, independent of backend:

  1. System 2 may only NARROW authority -- it can make an action more
     conservative, never less. Widening past the deterministic guard requires a
     human, not a language model.
  2. Confidence never grants authority. System 2 is advisory input to a
     decision the guard still gets to veto.
"""

from __future__ import annotations

import time
from typing import Protocol

from .types import Action, Decision, SceneState

# Lower rank == more conservative. System 2 may move a proposal UP this ordering
# (more cautious) or leave it alone, never down.
_CONSERVATISM = {
    "stop": 0,
    "slow": 1,
    "turn_left": 2,
    "turn_right": 2,
    "inspect": 3,
    "ask_operator": 4,
    "continue": 5,
}


def narrows_only(proposed: Action, prior: Action) -> bool:
    """True if `proposed` is no less cautious than `prior`."""
    return _CONSERVATISM.get(proposed.kind, 9) <= _CONSERVATISM.get(prior.kind, 9)


class Consultant(Protocol):
    def consult(self, state: SceneState, s1: Decision, reason: str) -> Decision | None: ...


class StubConsultant:
    """Deterministic stand-in so escalation is exercisable without an LLM key.

    Mirrors what a sane slow path would do: be more cautious than a low-tier
    fast path. Also lets the closing-speed failure mode be reproduced.
    """

    def consult(self, state: SceneState, s1: Decision, reason: str) -> Decision | None:
        closing = [e for e in state.entities if e.z > 0 and (e.vx**2 + e.vy**2 + e.vz**2) ** 0.5 > 1.0]
        if closing:
            e = min(closing, key=lambda e: e.z)
            action = Action(kind="slow", target_id=e.id, reason=f"s2: closing entity ({reason})")
        else:
            action = Action(kind="continue", reason=f"s2: reviewed, path clear ({reason})")
        return Decision(action=action, source="s2", p=0.0, margin=0.0, t=state.t)


_PROMPT = """You are the slow, deliberate half of a robot's decision system.

The fast half already proposed an action. Review it. You may make it MORE
cautious; you may not make it less cautious. Stopping is always acceptable.

Environment:
{scene}

Fast half proposed: {proposed}
Why you were asked: {reason}

Choose one action."""


class RaiConsultant:
    """RAI-backed slow path -- via RAI's LLM plumbing, not its agent loop.

    RAI's `ReActAgent` is a long-running service: `run()` spawns a thread and
    returns nothing, and there is no synchronous entry point. System 2 needs a
    request/response at control-loop rate, so we take the half of RAI that fits
    -- `get_llm_model`, its vendor-agnostic config layer -- and call the model
    directly. LangGraph still sits underneath.

    Asking the model for an `Action` rather than a `Decision` is deliberate:
    `source`, `latency_ms` and the confidence fields are ours to fill, not
    something a language model should get to invent.
    """

    def __init__(
        self,
        config_path: str = "config.toml",
        vendor: str | None = None,
        model_type: str = "complex_model",
    ) -> None:
        from rai import get_llm_model  # noqa: PLC0415 -- optional dep, import late

        self._llm = get_llm_model(model_type, vendor=vendor, config_path=config_path)
        self._chain = self._llm.with_structured_output(Action)

    def consult(self, state: SceneState, s1: Decision, reason: str) -> Decision | None:
        t0 = time.perf_counter()
        try:
            action = self._chain.invoke(
                _PROMPT.format(scene=state.to_text(), proposed=s1.action.kind, reason=reason)
            )
        except Exception:
            return None  # a slow path that fails must not stall the loop
        if not narrows_only(action, s1.action):
            return None  # widening is not System 2's call to make
        return Decision(
            action=action,
            source="s2",
            latency_ms=(time.perf_counter() - t0) * 1000,
            t=state.t,
        )


def demo() -> None:
    """Self-check: narrowing rule, and the stub produces a valid Decision."""
    assert narrows_only(Action(kind="stop"), Action(kind="continue"))
    assert narrows_only(Action(kind="slow"), Action(kind="slow"))
    assert not narrows_only(Action(kind="continue"), Action(kind="stop")), (
        "s2 must not widen past an executed stop"
    )

    from .body import MockBody

    m = MockBody("approach")
    state = [m.step() for _ in range(6)][-1]
    out = StubConsultant().consult(state, Decision(action=Action(kind="continue"), source="s1"), "low")
    assert out is not None and out.source == "s2"
    assert out.action.kind == "slow", "closing entity should draw a cautious s2 answer"

    print("system2.py ok")


if __name__ == "__main__":
    demo()
