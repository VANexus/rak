"""Dialogue loop: wake gating, control binding, terminal speech."""
from rak_harness.dialogue import DialogueEngine


def test_navigate_without_wake_is_ignored():
    engine = DialogueEngine(attention_sec=12.0)
    out = engine.on_intent(kind="navigate", text="去充电桩",
                           destination="充电桩", command=None, now=100.0)
    assert not out.submit_task
    assert out.say and out.say[0][0] == "need_wake"


def test_navigate_inside_attention_window_submits():
    engine = DialogueEngine(attention_sec=12.0)
    engine.on_wake(now=100.0)
    out = engine.on_intent(kind="navigate", text="去充电桩",
                           destination="充电桩", command=None, now=105.0)
    assert out.submit_task == ["去充电桩"]
    assert out.say[0][0] == "ack_navigate"


def test_attention_window_expires():
    engine = DialogueEngine(attention_sec=12.0)
    engine.on_wake(now=100.0)
    out = engine.on_intent(kind="navigate", text="去充电桩",
                           destination="充电桩", command=None, now=200.0)
    assert not out.submit_task


def test_control_without_active_task_says_so():
    engine = DialogueEngine()
    out = engine.on_intent(kind="control", text="停下",
                           destination="", command="pause", now=100.0)
    assert not out.control
    assert out.say[0][0] == "no_task"


def test_terminal_status_speaks_outcome_and_clears():
    engine = DialogueEngine()
    engine.on_wake(now=100.0)
    engine.on_intent(kind="navigate", text="去充电桩", destination="充电桩",
                     command=None, now=101.0)
    out = engine.on_task_status(task_id="t", revision=1, state="SUCCEEDED",
                                reason="ok", dest="充电桩",
                                vehicle_ok=True, vehicle_note="", now=102.0)
    assert out.say[0][0] == "done"
    # cleared: further control finds no task
    out2 = engine.on_intent(kind="control", text="停下", destination="",
                            command="pause", now=103.0)
    assert out2.say[0][0] == "no_task"


def test_unknown_intent_clarifies_with_cooldown():
    engine = DialogueEngine(clarify_cooldown_sec=10.0)
    first = engine.on_intent(kind=None, text="嗯嗯", destination="",
                             command=None, now=100.0)
    assert first.say and first.say[0][0] == "unknown"
    second = engine.on_intent(kind=None, text="啊啊", destination="",
                              command=None, now=101.0)
    assert not second.say  # throttled, no screen flooding
