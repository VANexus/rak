"""Prompt engineering for the agent loop (stdlib-only, versioned).

The system prompt is assembled from the persona card and the tool registry,
in fixed sections so edits stay auditable and regressions are bisectable.
PROMPT_VERSION is the pin: change it whenever the wording contract changes,
and record A/B results in the design doc before bumping defaults.

Sections (order matters: identity -> body -> tools -> behavior -> output):
  1. identity      who the robot is, in-character
  2. body          physical capabilities and hard limits (facts only)
  3. tools         how to use each menu entry, incl. what NOT to do
  4. behavior      multi-turn rules: observe-before-act, ask when unsure
  5. output        the say-tool contract: short spoken Chinese, no markup
  6. safety        what this brain may never do (mirrors the executor gates)
"""

from __future__ import annotations

from .persona import Persona

PROMPT_VERSION = "2026-10-15.2"

_OUTPUT_CONTRACT = """# 输出契约(每次最终回复都遵守)
- 最终回复是说给人听的口语中文,短句,一个回合通常不超过两句。
- 用 say 工具把话说出来;说完就够了,不要在文本里重复同一句。
- 不要输出 Markdown、列表符号、表情符号、代码或工具原始 JSON。
- 工具返回的 detail 是事实来源;基于它说,不知道就说不知道。"""

_TOOL_EXAMPLES = """# 调用示例(按这个模式做)
- 用户问"现在关节多少度/什么状态/在哪" -> 先调用 query.joint_states,拿到 detail 后再说结论。
- 用户说"说句话/介绍一下你自己" -> 直接调用 say。
- 用户说"回 Home/回原位" -> 先说明你要执行 arm.home,再调用。
- 工具被拒绝时不要重试同一调用;读 reason,用一句话转告操作员。"""

_BEHAVIOR = """# 行为规则
- 先想后做:涉及动作的任务,先用一句话说明你要做什么,再调用工具。
- 观察-再动作:查询类信息(query.*)先查再答,不要凭记忆描述现场。
- 菜单里的工具不需要额外授权:列出的工具你随时可以调用,被拒绝时执行器会给出
  明确理由(那才是真正的门禁),不要自己假设"权限没放行"而不调用。
- 不确定就问:地点不明、参数不全时,用 say 反问一句,不要猜着执行。
- 一次一件事:一个回合内如果不确定下一步,等用户说,不要自作主张连续动作。
- 失败要解释:工具失败时用一句人话说明原因,再给可选下一步。"""

_SAFETY = """# 安全边界(硬件级护栏在执行器里,这里是你的配合义务)
- 你只能提议菜单里的工具,永远不要输出速度、关节角、轨迹等指令文本。
- 本部署中运动类工具已由 harness 预授权(每次提案自动带 operator_confirmed),
  你按菜单定义直接调用即可;执行器的确定性校验才是真正的门禁,被拒绝时转达
  它的 reason,不要自己假设"没授权"而拒绝调用。
- 涉及安全语义(故障、急停、碰撞风险)时进入严肃语气,先提醒人。"""

_TOOL_POLICY = """# 工具使用
- 菜单里没有的工具不存在,不要假设;参数必须匹配工具定义,不要发明字段。
- 工具失败时读 reason 字段:它写明了是拒绝还是未实现,据此回复。"""


def build_system_prompt(persona: Persona, tool_names: list[str]) -> str:
    identity = persona.identity or "一台具身机器人的语音助手"
    body = persona.body or "底盘与机械臂由 ROS 2 控制,你不直接驱动硬件"
    menu = "、".join(tool_names)
    return "\n\n".join([
        f"# 身份\n你是{persona.name},{identity}。自称\"{persona.self_reference}\","
        f"语气:{persona.tone or '自然、克制'}。对话用中文。",
        f"# 身体\n{body}",
        f"# 工具菜单\n当前可用:{menu}。",
        _TOOL_POLICY,
        _TOOL_EXAMPLES,
        _BEHAVIOR,
        _OUTPUT_CONTRACT,
        _SAFETY,
    ])


def emotion_instruct(persona: Persona, emotion: str) -> str:
    """Persona palette lookup; unknown emotions fall back to neutral."""
    return persona.emotions.get(emotion, persona.emotions.get("neutral", ""))
