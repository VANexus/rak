"""The System 1 / System 2 contract.

Everything crossing between the fast path and the slow path is defined here and
only here. Both sides import these objects directly -- same process, no bus, no
serialization except at two edges (the JSONL log, and the future ROS bridge).

Treat this module as FROZEN once the first episode is logged. The gate
thresholds, the finetune corpus and the eventual ROS bridge are all coupled to
it, and migrating later is far more expensive than getting it right now.
"""

from __future__ import annotations

from enum import Enum
from typing import Literal

from pydantic import BaseModel, Field

ActionKind = Literal[
    "stop",
    "slow",
    "continue",
    "turn_left",
    "turn_right",
    "inspect",
    "ask_operator",
]


class Entity(BaseModel):
    """One tracked object, metres, camera frame (+z forward, +x right)."""

    id: int
    label: str = ""  # english, lowercase -- Laya is English-only
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0  # closing speed matters most for a forward-facing robot
    score: float = 0.0


class SceneState(BaseModel):
    """The world as both System 1 and System 2 see it."""

    t: float = 0.0
    entities: list[Entity] = Field(default_factory=list)
    scene: str = "open_floor"
    flags: list[str] = Field(default_factory=list)

    def to_text(self) -> str:
        """The ONLY bridge into Laya.

        Laya has no vision channel and is English-only, so perception must be
        serialized to text here. This is an acknowledged information bottleneck,
        not an oversight -- it is the boundary of what a text encoder can see.

        Deterministic and order-stable: the same SceneState must always render
        the same string. Logged episodes store this verbatim, so changing the
        format silently invalidates every existing training line.
        """
        head = f"t={self.t:.1f} scene={self.scene}"
        if self.flags:
            head += f" flags={','.join(sorted(self.flags))}"
        if not self.entities:
            return head + " | no objects"
        parts = []
        for e in sorted(self.entities, key=lambda e: e.id):
            speed = (e.vx**2 + e.vy**2 + e.vz**2) ** 0.5
            motion = "static" if speed < 0.05 else f"moving {speed:.1f} m/s"
            parts.append(f"id={e.id} {e.label} at ({e.x:.1f}, {e.y:.1f}, {e.z:.1f}) {motion}")
        return head + " | " + " | ".join(parts)


class Action(BaseModel):
    kind: ActionKind
    target_id: int | None = None
    reason: str = ""


class Tier(str, Enum):
    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"


class Decision(BaseModel):
    """A decision plus enough provenance to audit and to train on."""

    action: Action
    source: Literal["s1", "s2", "guard", "fallback"]
    p: float = 0.0  # Laya's raw top-option probability
    margin: float = 0.0  # p(top) - p(second): the real signal, see gate()
    latency_ms: float = 0.0
    t: float = 0.0


def demo() -> None:
    """Self-check: to_text() must be deterministic and order-stable."""
    a = SceneState(
        t=12.44,
        entities=[
            Entity(id=7, label="box", x=-0.4, z=2.8),
            Entity(id=3, label="person", x=0.1, z=1.2, vx=0.9),
        ],
    )
    b = SceneState(t=12.44, entities=list(reversed(a.entities)))

    assert a.to_text() == b.to_text(), "to_text must not depend on entity order"
    assert "person" in a.to_text() and "moving 0.9 m/s" in a.to_text()
    assert "static" in a.to_text(), "zero-velocity entity should read as static"
    assert SceneState(t=1.0).to_text().endswith("no objects")
    assert "flags=low_light,occluded" in SceneState(t=1.0, flags=["occluded", "low_light"]).to_text()

    # Action validation is the trust boundary for out-of-domain Laya output.
    Action.model_validate({"kind": "stop", "target_id": 3})
    try:
        Action.model_validate({"kind": "levitate"})
    except Exception:
        pass
    else:
        raise AssertionError("Action must reject unknown kinds")

    print("types.py ok")


if __name__ == "__main__":
    demo()
