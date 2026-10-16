"""The TensorRT decider: token layout, decode, and (when the engine is built) end to end.

The engine and tokenizer live under the gitignored repo-root `data/` (ADR-0013), so
every test that needs them skips rather than fails on a checkout that has not run the
export runbook.
The token-layout test is the one that matters most: it pins the `[MASK]` marker
positions to the numbers `docs/laya_tensorrt.md` records, because a sequence that is
subtly wrong still produces logits and still looks like a working model.

Plain asserts; run directly (`python tests/test_tensorrt_decider.py`) or under pytest.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rak_agent.tensorrt_decider import (  # noqa: E402
    DEFAULT_MODEL_DIR,
    TensorRtDecider,
    _render_options,
    _softmax,
    _temp_bucket,
)
from rak_agent.types import Entity, SceneState  # noqa: E402

#: Recorded in docs/laya_tensorrt.md from the pinned tokenizer and the current question.
EXPECTED_MARKERS = [15, 28, 42, 51, 63, 75, 88]
EMPTY_SCENE_TOKENS = 118


def _has_assets() -> bool:
    return (DEFAULT_MODEL_DIR / "tokenizer" / "tokenizer.json").is_file()


def test_temp_bucket_matches_upstream():
    """`laya.common.temp_bucket`: the bucket the checkpoint's temperatures are keyed by."""
    assert _temp_bucket(0, 2) == "choice:2"
    assert _temp_bucket(0, 7) == "choice:6-10"
    assert _temp_bucket(0, 11) == "choice:11+"
    assert _temp_bucket(2, 2) == "noul:2"


def test_render_options_is_label_indexed():
    opts = _render_options({"stop": "halt now", "go": "path clear"})
    assert opts == ["stop: halt now", "go: path clear"], "order must follow the criteria dict"


def test_softmax_is_a_distribution():
    p = _softmax(np.array([0.0, 1.0, 2.0], dtype=np.float32))
    assert abs(float(p.sum()) - 1.0) < 1e-6
    assert p.argmax() == 2


def test_token_layout_matches_the_recorded_markers():
    if not _has_assets():
        print("  skip: model assets not present")
        return
    d = TensorRtDecider.__new__(TensorRtDecider)  # noqa: SLF001 -- tokenizer only, no engine
    d.model_dir = DEFAULT_MODEL_DIR
    d.question_id = "action"
    d.max_len, d.head_max_len = 1024, 256
    d._load_tokenizer()  # noqa: SLF001
    d._build_question()  # noqa: SLF001

    ids, markers = d._build_sequence(SceneState(t=0.0).to_text())  # noqa: SLF001
    assert markers == EXPECTED_MARKERS, f"marker layout drifted: {markers}"
    assert len(ids) == EMPTY_SCENE_TOKENS, f"empty-scene length drifted: {len(ids)}"

    # A longer state must not move the head; only the tail grows.
    longer, longer_markers = d._build_sequence(  # noqa: SLF001
        SceneState(t=0.0, entities=[Entity(id=1, label="person", z=1.2)]).to_text()
    )
    assert longer_markers == EXPECTED_MARKERS
    assert len(longer) > len(ids)


def test_decide_returns_a_decision_and_never_raises():
    """End-to-end through the engine. Skipped when the plan has not been built."""
    if not (DEFAULT_MODEL_DIR / "laya-typed-decisions-bf16.plan").is_file():
        print("  skip: TensorRT engine not built")
        return
    d = TensorRtDecider()

    far = d.decide(SceneState(t=0.0, entities=[Entity(id=5, label="box", z=2.8)]))
    assert far.source == "s1"
    assert 0.0 <= far.p <= 1.0 and far.latency_ms > 0.0

    # The trust boundary: a label the model emits must still be one `Action` accepts.
    from rak_agent.types import ActionKind  # noqa: PLC0415

    assert far.action.kind in ActionKind.__args__

    # A state that cannot be tokenized into the profile must degrade, not raise.
    broken = d.decide(SceneState(t=0.0, entities=[Entity(id=1, label="x" * 5000, z=1.0)]))
    assert broken.source == "s1" and 0.0 <= broken.p <= 1.0

    # guard() still owns the veto, whatever the model said.
    from rak_agent.system1 import guard  # noqa: PLC0415

    assert guard(SceneState(t=0.0, entities=[Entity(id=1, label="person", z=0.4)])) is not None
    d.close()


def main() -> None:
    failures = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"ok  {name}")
            except Exception as exc:  # noqa: BLE001 -- a test runner reports, it does not raise
                failures += 1
                print(f"FAIL {name}: {type(exc).__name__}: {exc}")
    if failures:
        raise SystemExit(f"{failures} test(s) failed")
    print("tensorrt decider tests passed")


if __name__ == "__main__":
    main()
