"""The tick loop: sense, guard, decide, maybe escalate, act, log.

This is the only module where System 1 and System 2 meet. `system1.py` never
imports `system2.py` and vice versa -- that is what makes "the harness only
lives on the slow path" a structural fact rather than a convention.

Run:  python -m rak_agent.loop --scenario approach --decider stub --ticks 50
"""

from __future__ import annotations

import argparse
import json
import threading
import time
import uuid
from pathlib import Path

from .body import MockBody
from .system1 import Decider, StubDecider, resolve
from .system2 import Consultant, StubConsultant
from .types import Decision, SceneState, Tier


def _default_episode_dir() -> Path:
    """One corpus for CLI runs: `<repo>/data/episodes` (ADR-0013).

    Source tree: module sits at `src/rak_agent/rak_agent/loop.py`, so the
    package root holds `package.xml` and the repo root holds `data/`.
    Installed copies (site-packages) have neither -- the ROS node walks out to
    the workspace itself (`node._episode_dir`); a bare CLI there keeps the
    corpus beside the install, which is disposable by design.
    """
    pkg_root = Path(__file__).resolve().parent.parent  # <repo>/src/rak_agent (source)
    if (pkg_root / "package.xml").is_file():
        return pkg_root.parent.parent / "data" / "episodes"  # <repo>/data/episodes
    return pkg_root / "data" / "episodes"  # installed/unknown: beside the code


EPISODE_DIR = _default_episode_dir()


def _drain(
    replies: list[tuple[int, Decision | None]],
    applied_tick: dict[int, bool],
    pending: dict[int, tuple[SceneState, Decision, str]],
    state: SceneState,
    log: "Logger",
    verbose: bool,
) -> None:
    """Apply any landed System 2 replies, staleness-checked.

    A reply refers to the scene it was asked about, which may be seconds old by
    now. Applying it blind means acting on a confident decision about a scene
    that no longer exists.
    """
    for reply_tick, reply in [r for r in replies if r[0] not in applied_tick and r[1] is not None]:
        applied_tick[reply_tick] = True
        asked_state, prior, _reason = pending[reply_tick]
        assert reply is not None
        stale = _scene_changed(asked_state, state)
        applied = (not stale) and reply.action.kind != prior.action.kind
        log.log(
            kind="s2_reply",
            tick=reply_tick,
            asked_at=asked_state.t,
            s2=reply.model_dump(),
            applied=applied,
            stale=stale,
            narrowed=reply.action.kind,
        )
        if applied and verbose:
            print(f"  s2 reply for tick {reply_tick}: {reply.action.kind}")


class Logger:
    """Append-only JSONL. The episode corpus that feeds finetuning.

    Note what is NOT here: a training pipeline. `rai_finetune` is not
    pip-installable and may not be implemented yet, so the deliverable is a
    corpus in an already-finetune-ready shape, not a training run.

    Label priority: operator override > System 2 > hindsight. Operator override
    is the only source of real ground truth and should exist from day one.
    """

    def __init__(self, episode: str | None = None, out_dir: Path = EPISODE_DIR) -> None:
        out_dir.mkdir(parents=True, exist_ok=True)
        self.episode = episode or uuid.uuid4().hex[:12]
        self.path = out_dir / f"{self.episode}.jsonl"
        self._lock = threading.Lock()

    def log(self, **fields: object) -> None:
        fields["episode"] = self.episode
        fields["ts"] = round(time.time(), 3)
        with self._lock, self.path.open("a") as fh:
            fh.write(json.dumps(fields, default=str) + "\n")


def _scene_changed(asked: SceneState, now: SceneState, tol_m: float = 0.25) -> bool:
    """Is `now` no longer the scene the slow path was asked about?

    Comparing ids alone is NOT enough: a single entity keeping its id while
    moving metres reads as unchanged, and that is exactly the case staleness
    exists to catch. Positions count too.
    """
    before = {e.id: e for e in asked.entities}
    after = {e.id: e for e in now.entities}
    if before.keys() != after.keys():
        return True
    return any(
        e.label != after[i].label
        or ((e.x - after[i].x) ** 2 + (e.y - after[i].y) ** 2 + (e.z - after[i].z) ** 2) ** 0.5 > tol_m
        for i, e in before.items()
    )


def run(
    scenario: str = "approach",
    ticks: int = 50,
    decider: str = "stub",
    consultant: Consultant | None = None,
    force_s2: int | None = None,
    out_dir: Path = EPISODE_DIR,
    verbose: bool = True,
) -> Logger:
    body = MockBody(scenario)
    if decider != "stub":
        raise RuntimeError(
            f"decider {decider!r} is unavailable; use stub until the TensorRT backend is integrated"
        )
    s1: Decider = StubDecider()
    s2 = consultant or StubConsultant()
    log = Logger(out_dir=out_dir)
    log.log(
        kind="meta",
        scenario=scenario,
        ticks=ticks,
        decider=decider,
        consultant=type(s2).__name__,
        force_s2=force_s2,
        s1_device=str(getattr(s1, "device", "")) or None,
        s1_weights=str(getattr(s1, "weights", "")) or None,
    )

    pending: dict[int, tuple[SceneState, Decision, str]] = {}
    replies: list[tuple[int, Decision | None]] = []
    applied_tick: dict[int, bool] = {}
    threads: list[threading.Thread] = []

    for tick in range(ticks):
        state = body.step()
        decision = s1.decide(state)
        # The guard/gate/fallback policy lives in system1.resolve() so the ROS
        # node runs exactly this path -- see the docstring there.
        final, tier, veto = resolve(
            state, decision, force_s2=force_s2 is not None and tick >= force_s2
        )

        body.apply(final.action)

        # Escalate: fire-and-forget, applied on a LATER tick against the scene it
        # was asked about. Blocking here would put the slow path on the fast path.
        # A veto already reports LOW (resolve), so this is the single trigger.
        if tier is not Tier.HIGH:
            reason = "guard" if veto is not None else f"tier={tier.value}"
            pending[tick] = (state, final, reason)
            th = threading.Thread(
                target=lambda k=tick, s=state, d=final, r=reason: replies.append(
                    (k, s2.consult(s, d, r))  # type: ignore[arg-type]
                ),
                daemon=True,
            )
            th.start()
            threads.append(th)

        _drain(replies, applied_tick, pending, state, log, verbose)

        log.log(
            kind="tick",
            tick=tick,
            t=state.t,
            state=state.model_dump(),
            state_text=state.to_text(),  # verbatim: never regenerate at train time
            s1=decision.model_dump(),
            tier=tier.value,
            veto=veto.model_dump() if veto is not None else None,
            final=final.model_dump(),
            label=None,
        )
        if verbose and (tick % 10 == 0 or veto is not None):
            tag = "VETO" if veto else final.source
            print(f"[{tick:3d}] {state.to_text()[:68]:<68} -> {final.action.kind:<10} ({tag})")

    # Flush escalations still in flight before closing the episode. Dropping
    # them discards System 2's answers -- which is most of the training signal.
    for th in threads:
        th.join(timeout=30)
    if threads:
        _drain(replies, applied_tick, pending, state, log, verbose)

    log.log(kind="end", ticks=ticks)
    return log


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scenario", default="approach", choices=["approach", "cross", "clear"])
    p.add_argument("--decider", default="stub", choices=["stub"])
    p.add_argument("--ticks", type=int, default=50)
    p.add_argument("--force-s2", type=int, default=None, help="force escalation from this tick")
    p.add_argument("--out", default=str(EPISODE_DIR))
    a = p.parse_args()

    log = run(a.scenario, a.ticks, a.decider, force_s2=a.force_s2, out_dir=Path(a.out))
    print(f"\nepisode: {log.path}")


if __name__ == "__main__":
    main()
