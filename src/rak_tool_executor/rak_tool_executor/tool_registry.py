"""Tool registry: the single source of truth for the tool menu.

Every entry carries a JSON Schema (keys/types/ranges) consumed by three
consumers without drift: the executor's stdlib validator, the LLM loop's
function-calling tool definitions, and tests. Keep this module stdlib-only.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from . import chassis_limits

# Safety classes (design docs/brain_arm_tools_design.md section 4):
#   read/speak        -> loop auto-approves
#   motion_free       -> operator confirmation required before proposing
#   motion_on_object  -> same, plus physical-effect caveats
SAFETY_READ = "read"
SAFETY_SPEAK = "speak"
SAFETY_MOTION = "motion_free"
SAFETY_MOTION_OBJECT = "motion_on_object"
SAFETY_OPERATOR = "operator_only"


@dataclass(frozen=True)
class ToolSpec:
    name: str
    description: str
    args_schema: dict[str, Any]
    safety: str
    status: str  # implemented | unimplemented | deferred
    hint: str = ""  # reason when not implemented, guidance for the LLM
    deprecated: bool = field(default=False)
    # 默认超时：提案到终态结果的等待上限。MCP 层直接用它，不另立一张表。
    timeout_s: float = field(default=30.0)


_NUM = {"type": "number"}

TOOLS: tuple[ToolSpec, ...] = (
    ToolSpec(
        name="say",
        description="让机器人从扬声器说一句话（中文）。返回已入队，不代表播完。",
        args_schema={
            "type": "object",
            "properties": {
                "text": {
                    "type": "string",
                    "description": "要说的内容，用自然中文口语",
                }
            },
            "required": ["text"],
        },
        safety=SAFETY_SPEAK,
        status="implemented",
    ),
    ToolSpec(
        name="query.joint_states",
        description="读取全部关节当前角度（含六轴臂 j1_base..j6_wrist_roll 与底盘轮）。",
        args_schema={"type": "object", "properties": {}, "required": []},
        safety=SAFETY_READ,
        status="implemented",
    ),
    ToolSpec(
        name="query.chassis_state",
        description=(
            "读取底盘状态：四轮实测位置/速度（/joint_states）与里程计位姿/速度"
            "（mecanum_drive_controller odometry）。运动后用它确认车真的动了。"
        ),
        args_schema={"type": "object", "properties": {}, "required": []},
        safety=SAFETY_READ,
        status="implemented",
        timeout_s=5.0,
    ),
    ToolSpec(
        name="task.goto",
        description="导航去 waypoint.yaml 里配置的别名点（由任务执行器解析，本工具不自己解 pose）。",
        args_schema={
            "type": "object",
            "properties": {
                "waypoint": {"type": "string", "description": "任务执行器侧配置的别名"}
            },
            "required": ["waypoint"],
        },
        safety=SAFETY_MOTION,
        status="implemented",
    ),
    ToolSpec(
        name="task.pause",
        description="暂停当前导航任务（等 Nav2 取消确认与静止遥测后才上报暂停）。",
        args_schema={"type": "object", "properties": {}, "required": []},
        safety=SAFETY_READ,
        status="implemented",
    ),
    ToolSpec(
        name="task.resume",
        description="恢复被暂停的导航任务（重新提交语义目标）。",
        args_schema={"type": "object", "properties": {}, "required": []},
        safety=SAFETY_READ,
        status="implemented",
    ),
    ToolSpec(
        name="task.cancel",
        description="取消当前导航任务。",
        args_schema={"type": "object", "properties": {}, "required": []},
        safety=SAFETY_READ,
        status="implemented",
    ),
    ToolSpec(
        name="chassis.drive",
        description=(
            "底盘按机体坐标系速度行驶一段固定时长后自动置零停车。"
            "vx 前进为正 (m/s)、vy 左为正 (m/s)、wz 逆时针为正 (rad/s)；"
            f"上限 |vx|,|vy|≤{chassis_limits.MAX_LINEAR_MPS} m/s、"
            f"|wz|≤{chassis_limits.MAX_ANGULAR_RPS} rad/s，"
            f"时长 0.05..{chassis_limits.MAX_DURATION_S} s —— 不存在无限期运动。"
            "Nav2 在场上时会被底盘单写者互锁拒绝（结构化返回 nav2_active）。"
        ),
        args_schema={
            "type": "object",
            "properties": {
                "vx_mps": dict(_NUM, minimum=-chassis_limits.MAX_LINEAR_MPS,
                               maximum=chassis_limits.MAX_LINEAR_MPS,
                               description="前进为正, m/s"),
                "vy_mps": dict(_NUM, minimum=-chassis_limits.MAX_LINEAR_MPS,
                               maximum=chassis_limits.MAX_LINEAR_MPS,
                               description="向左为正, m/s"),
                "wz_rps": dict(_NUM, minimum=-chassis_limits.MAX_ANGULAR_RPS,
                               maximum=chassis_limits.MAX_ANGULAR_RPS,
                               description="逆时针为正, rad/s"),
                "duration_s": dict(_NUM, minimum=chassis_limits.MIN_DURATION_S,
                                   maximum=chassis_limits.MAX_DURATION_S,
                                   description="持续时长, s；到点置零"),
            },
            "required": ["duration_s"],
        },
        safety=SAFETY_MOTION,
        status="implemented",
        timeout_s=chassis_limits.MAX_DURATION_S + 5.0,
    ),
    ToolSpec(
        name="chassis.stop",
        description="立即把底盘目标速度置零（只写零，不产生任何运动）。",
        args_schema={"type": "object", "properties": {}, "required": []},
        # 只写零目标、永不产生位移，因此按只读级自动放行：越急停越不该卡在确认上。
        safety=SAFETY_READ,
        status="implemented",
        timeout_s=5.0,
    ),
    ToolSpec(
        name="arm.home",
        description="六轴臂回到上电折叠 Home 位（0/225/270/0/0/0 度）。低速、单写者。",
        args_schema={
            "type": "object",
            "properties": {
                "speed_scale": dict(_NUM, minimum=0.05, maximum=0.25,
                                    description="相对 URDF 限位的速度比例，上限 0.25")
            },
            "required": [],
        },
        safety=SAFETY_MOTION,
        status="implemented",
    ),
    ToolSpec(
        name="arm.reach",
        description="臂末端 tool0 到 base_link 系目标位姿（IK 到位）。",
        args_schema={
            "type": "object",
            "properties": {
                "x_m": dict(_NUM, description="base_link 系 x, m"),
                "y_m": dict(_NUM, description="base_link 系 y, m"),
                "z_m": dict(_NUM, description="base_link 系 z, m"),
                "speed_scale": dict(_NUM, minimum=0.05, maximum=0.25),
            },
            "required": ["x_m", "y_m", "z_m"],
        },
        safety=SAFETY_MOTION,
        status="unimplemented",
        hint="P2 才接通（pinocchio IK + 护栏实机标定），现在调用会被拒绝并带此说明。",
    ),
    ToolSpec(
        name="gripper.set",
        description="夹爪开合（0..1）。",
        args_schema={
            "type": "object",
            "properties": {
                "position": dict(_NUM, minimum=0.0, maximum=1.0)
            },
            "required": ["position"],
        },
        safety=SAFETY_MOTION_OBJECT,
        status="unimplemented",
        hint="D6 硬件门：F407 无夹爪物理通道（回读恒 0），接通前一律拒绝。",
    ),
    ToolSpec(
        name="scene.scan",
        description="底盘原地自转扫描周围深度。",
        args_schema={
            "type": "object",
            "properties": {
                "degrees": dict(_NUM, minimum=-360.0, maximum=360.0)
            },
            "required": ["degrees"],
        },
        safety=SAFETY_MOTION,
        status="deferred",
        hint="与底盘单写者冲突（Wait：D5），P4 WBC 闭合后才放开。",
    ),
)

_BY_NAME: dict[str, ToolSpec] = {t.name: t for t in TOOLS}


def get_tool(name: str) -> ToolSpec | None:
    return _BY_NAME.get(name)


def tool_names() -> list[str]:
    return [t.name for t in TOOLS]


def tool_definitions() -> list[dict[str, Any]]:
    """Anthropic tool-use format, derived from the registry (D2: one table)."""
    out = []
    for t in TOOLS:
        if t.deprecated:
            continue
        desc = t.description if t.status == "implemented" else f"{t.description}（当前不可用：{t.hint}）"
        out.append({"name": t.name, "description": desc, "input_schema": t.args_schema})
    return out


_TYPE_MAP = {"string": str, "number": (int, float), "integer": int, "boolean": bool, "object": dict}


def validate_args(spec: ToolSpec, args: Any) -> list[str]:
    """Minimal stdlib JSON-Schema subset check. Returns error strings (empty=ok).

    Covers: object type, required keys, scalar types, number min/max.
    Unknown keys are rejected -- the loop must not invent arguments.
    """
    if not isinstance(args, dict):
        return ["args must be a JSON object"]
    errors: list[str] = []
    schema = spec.args_schema
    props: dict[str, Any] = schema.get("properties", {})
    for key in schema.get("required", []):
        if key not in args:
            errors.append(f"missing required key '{key}'")
    for key, value in args.items():
        if key not in props:
            errors.append(f"unknown key '{key}'")
            continue
        rule = props[key]
        expected = rule.get("type")
        if expected in _TYPE_MAP and not isinstance(value, _TYPE_MAP[expected]):
            errors.append(f"'{key}' must be {expected}")
            continue
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            lo, hi = rule.get("minimum"), rule.get("maximum")
            if lo is not None and value < lo:
                errors.append(f"'{key}' below minimum {lo}")
            if hi is not None and value > hi:
                errors.append(f"'{key}' above maximum {hi}")
    return errors
