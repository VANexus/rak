"""The body: what the brain senses from and acts on.

Today this is a mock. Real hardware (V4L2 camera, CAN actuators) implements the
same two methods later -- see the seam notes on `Body`.
"""

from __future__ import annotations

from typing import Protocol

from .types import Action, Entity, SceneState


class Body(Protocol):
    """The hardware seam.

    `step()` MUST NOT BLOCK. For the mock it is instant. A real implementation
    keeps a background reader thread holding `latest_state` and returns it
    lock-free. Making this a synchronous fetch instead would stall System 1 for
    hundreds of milliseconds on the first dropped frame -- that is a design-time
    decision, not something that can be tuned away later.
    """

    def step(self) -> SceneState: ...

    def apply(self, action: Action) -> None: ...


class MockBody:
    """Deterministic scripted body. No hardware, no randomness."""

    def __init__(self, scenario: str = "approach") -> None:
        if scenario not in _SCENARIOS:
            raise ValueError(f"unknown scenario {scenario!r}; pick from {sorted(_SCENARIOS)}")
        self.scenario = scenario
        self.tick = 0
        self.applied: list[Action] = []

    def step(self) -> SceneState:
        state = _SCENARIOS[self.scenario](self.tick)
        self.tick += 1
        return state

    def apply(self, action: Action) -> None:
        self.applied.append(action)


def _approach(tick: int) -> SceneState:
    """A person walks straight in, crossing the stop zone around tick 25."""
    z = 3.0 - 0.1 * tick
    return SceneState(
        t=tick * 0.033,
        scene="corridor",
        entities=[Entity(id=3, label="person", x=0.1, z=max(z, 0.2), vz=-3.0 if z > 0.2 else 0.0)],
    )


def _cross(tick: int) -> SceneState:
    """An entity crosses laterally, entering and leaving the corridor."""
    x = -2.0 + 0.16 * tick
    return SceneState(
        t=tick * 0.033,
        scene="open_floor",
        entities=[Entity(id=5, label="person", x=x, z=1.5, vx=4.8)],
    )


def _clear(tick: int) -> SceneState:
    """Nothing in front. The boring case that must still be logged."""
    return SceneState(t=tick * 0.033, scene="open_floor", flags=["low_light"])


_SCENARIOS = {"approach": _approach, "cross": _cross, "clear": _clear}


def demo() -> None:
    """Self-check: scripts are deterministic and actually exercise the stop zone."""
    for name in _SCENARIOS:
        a, b = MockBody(name), MockBody(name)
        assert [a.step() for _ in range(10)] == [b.step() for _ in range(10)], f"{name} not deterministic"

    m = MockBody("approach")
    zs = [m.step().entities[0].z for _ in range(40)]
    assert zs[0] > 2.9 and min(zs) <= 0.6, "approach must enter the stop zone"

    m = MockBody("clear")
    assert all(not s.entities for s in (m.step() for _ in range(5)))

    m.apply(Action(kind="stop", reason="test"))
    assert [a.kind for a in m.applied] == ["stop"]

    print("body.py ok")


if __name__ == "__main__":
    demo()
