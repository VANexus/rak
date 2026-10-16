"""Pure tests for persona / prompt / long-context engineering. No ROS."""

from rak_harness.context import SessionDigest, estimate_tokens
from rak_harness.persona import Persona, load_persona
from rak_harness.prompts import PROMPT_VERSION, build_system_prompt, emotion_instruct


def _persona() -> Persona:
    return Persona(name="小锐", tone="干脆", identity="具身助手", body="底盘+六轴臂")


def test_prompt_has_all_sections_and_menu():
    prompt = build_system_prompt(_persona(), ["say", "query.joint_states"])
    for section in ("# 身份", "# 身体", "# 工具菜单", "# 工具使用", "# 行为规则",
                    "# 输出契约", "# 安全边界"):
        assert section in prompt
    assert "say" in prompt and "query.joint_states" in prompt
    assert PROMPT_VERSION


def test_emotion_instruct_known_and_fallback():
    persona = _persona()
    assert "开心" in emotion_instruct(persona, "happy")
    assert emotion_instruct(persona, "nonexistent") == emotion_instruct(persona, "neutral")


def test_load_persona_repo_card_and_default():
    persona = load_persona("")
    assert persona.name == "小锐" and persona.emotions.get("happy")


def test_estimate_tokens_cjk_aware():
    assert estimate_tokens("你好世界") == 4
    assert estimate_tokens("abcd") <= 2


def test_digest_render_includes_context_and_utterance():
    digest = SessionDigest()
    digest.update_context({"vehicle": "任务=IDLE；底盘待命"})
    message = digest.render("你好")
    assert "[车况] 任务=IDLE" in message
    assert "[用户此刻说] 你好" in message
    assert digest.turns[-1].role == "user"


def test_digest_compacts_old_turns():
    digest = SessionDigest(keep_recent_turns=3)
    for i in range(10):
        digest.add_user(f"第{i}个问题")
        digest.add_assistant(f"第{i}个回答")
    message = digest.render("现在的问题")
    assert "此前聊过" in message
    assert "现在的问题" in message
    assert len(digest.turns) == 21  # 20 rendered turns + the new utterance


def test_digest_trims_old_tool_details():
    digest = SessionDigest(tool_full_results=2)
    for i in range(5):
        digest.add_tool(f"tool{i}", True, "结果详情内容")
    message = digest.render("问一句")
    assert message.count("结果详情内容") == 2


def test_digest_budget_drops_history_not_context():
    digest = SessionDigest(max_tokens=50)
    digest.update_context({"vehicle": "任务=NAVIGATING"})
    for i in range(20):
        digest.add_user("很长的历史问题" * 10)
    message = digest.render("现在说什么")
    assert "[车况] 任务=NAVIGATING" in message
    assert "[用户此刻说] 现在说什么" in message


def test_digest_memory_reads_tool_events(tmp_path):
    record = '{"t": 1792000000.0, "kind": "tool_call", "tool": "say", "status": "ok"}\n'
    (tmp_path / "tool_events-20261015.jsonl").write_text(record, encoding="utf-8")
    digest = SessionDigest(memory_dir=str(tmp_path))
    lines = digest._read_memory()
    assert lines and "say" in lines[0]
