"""Dialogue loop: wake gating, control binding, terminal speech."""
from rak_harness.dialogue import DialogueEngine


def test_wake_opens_attention_and_answers():
    """唤醒不只要开窗口,还要回话——沉默的唤醒读起来像死机。"""
    engine = DialogueEngine(attention_sec=12.0)
    out = engine.on_wake(now=100.0, score=700.0)
    assert engine.is_attentive(101.0)
    assert out.say and out.say[0][0] == "ack_wake"


def test_wake_below_score_floor_stays_silent():
    """实测误触发堆 501-546、真唤醒 589+:低分不吭声,窗口照开。"""
    engine = DialogueEngine(ack_min_score=580.0)
    out = engine.on_wake(now=100.0, score=520.0)
    assert not out.say
    assert engine.is_attentive(101.0)          # 注意力窗口不受分数闸影响
    assert engine.on_wake(now=102.0, score=700.0).say


def test_wake_ack_debounced_within_cooldown():
    """误触发成片时(实测 9 秒 5 次)只回第一声,别当复读机;窗口每次都开。"""
    engine = DialogueEngine(ack_cooldown_sec=4.0)
    assert engine.on_wake(now=100.0, score=700.0).say    # 第一声应答
    assert not engine.on_wake(now=101.5, score=700.0).say  # 连环:静默,窗口照开
    assert engine.is_attentive(102.0)
    assert engine.on_wake(now=104.5, score=700.0).say    # 冷却过了:再应答


def test_navigate_without_wake_is_ignored():
    engine = DialogueEngine(attention_sec=12.0)
    out = engine.on_intent(kind="navigate", text="去充电桩",
                           destination="充电桩", command=None, now=100.0)
    assert not out.submit_task
    assert out.say and out.say[0][0] == "need_wake"


def test_navigate_inside_attention_window_submits():
    engine = DialogueEngine(attention_sec=12.0)
    engine.on_wake(now=100.0, score=700.0)
    out = engine.on_intent(kind="navigate", text="去充电桩",
                           destination="充电桩", command=None, now=105.0)
    assert out.submit_task == ["去充电桩"]
    assert out.say[0][0] == "ack_navigate"


def test_attention_window_expires():
    engine = DialogueEngine(attention_sec=12.0)
    engine.on_wake(now=100.0, score=700.0)
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
    engine.on_wake(now=100.0, score=700.0)
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
