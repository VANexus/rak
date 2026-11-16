"""Pure tests: registry validation and the permission policy. No ROS."""

from rak_tool_executor.permissions import AUTO, CONFIRM, DENY, chassis_interlock, evaluate
from rak_tool_executor.tool_registry import (
    SAFETY_MOTION,
    SAFETY_OPERATOR,
    SAFETY_READ,
    SAFETY_SPEAK,
    get_tool,
    tool_definitions,
    tool_names,
    validate_args,
)


def test_menu_has_nine_menu_rows_ten_tools():
    # v1 menu = 9 rows in the design doc; task.pause/resume/cancel share a row.
    # +3 (2026-11-16): chassis.drive / chassis.stop / query.chassis_state ——
    # MCP 闭环需要的底盘运动与读回，仍是同一张表。
    assert len(tool_names()) == 13


def test_validate_accepts_minimal_say():
    assert validate_args(get_tool("say"), {"text": "你好"}) == []


def test_validate_rejects_missing_required():
    errors = validate_args(get_tool("say"), {})
    assert any("text" in e for e in errors)


def test_validate_rejects_unknown_key():
    errors = validate_args(get_tool("say"), {"text": "x", "volume": 5})
    assert any("volume" in e for e in errors)


def test_validate_speed_scale_bounds():
    assert validate_args(get_tool("arm.home"), {"speed_scale": 0.25}) == []
    assert validate_args(get_tool("arm.home"), {"speed_scale": 0.5})  # > 0.25 rejected


def test_validate_type_check():
    errors = validate_args(get_tool("say"), {"text": 3})
    assert any("must be string" in e for e in errors)


def test_permission_read_speak_auto():
    assert evaluate(SAFETY_READ, False).mode == AUTO
    assert evaluate(SAFETY_SPEAK, False).mode == AUTO


def test_permission_motion_needs_confirmation():
    assert evaluate(SAFETY_MOTION, False).mode == "rejected"
    assert evaluate(SAFETY_MOTION, True).mode == CONFIRM


def test_permission_operator_only_denied():
    assert evaluate(SAFETY_OPERATOR, True).mode == DENY


def test_tool_definitions_match_registry():
    defs = {d["name"]: d for d in tool_definitions()}
    assert set(defs) == set(tool_names())
    for name, definition in defs.items():
        assert definition["input_schema"] == get_tool(name).args_schema
    assert "input_schema" in defs["say"]


def test_unimplemented_tools_flag_in_definition():
    defs = {d["name"]: d for d in tool_definitions()}
    assert "不可用" in defs["arm.reach"]["description"]
    assert "夹爪" in defs["gripper.set"]["description"] or "物理通道" in defs["gripper.set"]["description"]


# --------------------------------------------------- chassis single-writer interlock
WATCHED = ["bt_navigator", "controller_server", "planner_server", "velocity_smoother"]


def test_interlock_free_when_only_tool_executor_present():
    assert chassis_interlock(["tool_executor", "controller_manager"], WATCHED) == []


def test_interlock_reports_blocking_nav2_nodes():
    present = ["tool_executor", "bt_navigator", "controller_server", "joint_state_broadcaster"]
    assert chassis_interlock(present, WATCHED) == ["bt_navigator", "controller_server"]


def test_interlock_ignores_unwatched_and_partial_names():
    # 名字必须精确匹配：nav2_controller_server / controller_server_v2 不算占底盘。
    present = ["nav2_controller_server", "controller_server_v2", "tool_executor"]
    assert chassis_interlock(present, WATCHED) == []
