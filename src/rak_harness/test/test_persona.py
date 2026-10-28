"""Persona card + vehicle snapshot: degrade, never crash."""
from rak_harness.persona import (
    VehicleSnapshot,
    load_persona,
    render,
    snapshot_to_text,
)


def test_missing_card_falls_back_to_default():
    persona = load_persona("/nonexistent/persona.toml")
    assert persona.name


def test_snapshot_names_fault_first():
    text = snapshot_to_text(VehicleSnapshot(task_state="NAVIGATING", base_fault=0x8))
    assert "故障" in text and "NAVIGATING" in text


def test_snapshot_disconnected_when_no_link():
    assert "未连接" in snapshot_to_text(VehicleSnapshot())


def test_render_clips_to_max_chars():
    persona = load_persona("/nonexistent/persona.toml")
    persona.templates["x"] = "0123456789" * 20
    persona.max_reply_chars = 10
    assert len(render("x", persona)) <= 10
