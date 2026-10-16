"""Persona card + embodied context snapshot + emotion palette (stdlib-only).

人格工程和车况上下文都落在这里:云端 prompt 与本地保底回复共用同一份
snapshot,两边不可能说出互相矛盾的车况。

情感调色板(emotion palette)是人格与 TTS 之间的契约:每个情感映射到一句
CosyVoice 兼容的自然语言 instruct(如"用开心的语气说")。TTS 后端未接入
instruct 字段之前,这里只作为人格描述与 say 文本的风格指南。
"""

from __future__ import annotations

from dataclasses import dataclass, field

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover -- py<3.11 never ships here
    tomllib = None  # type: ignore


#: Default emotion -> CosyVoice-style instruct palette. The loop may mention
#: the emotion in its reasoning; the TTS layer maps it to synthesis instructs
#: once that field exists (Additive change to Say.srv, not yet made).
DEFAULT_EMOTIONS: dict[str, str] = {
    "neutral": "用平时稳重、清晰的语气说",
    "happy": "用开心、轻快的语气说",
    "excited": "用兴奋、上扬的语气说",
    "calm": "用平静、温和的语气说",
    "serious": "用严肃、认真的语气说",
    "gentle": "用温柔、安抚的语气说",
    "sorry": "用抱歉、诚恳的语气说",
    "curious": "用好奇、探询的语气说",
    "proud": "用得意、自豪的语气说",
    "sleepy": "用慵懒、倦倦的语气说",
}


@dataclass
class Persona:
    name: str = "小锐"
    language: str = "zh"
    tone: str = ""
    max_reply_chars: int = 60
    identity: str = ""
    body: str = ""
    self_reference: str = "我"
    emotions: dict = field(default_factory=lambda: dict(DEFAULT_EMOTIONS))
    templates: dict = field(default_factory=dict)


def load_persona(path: str) -> Persona:
    """Missing/unparseable card degrades to the built-in default, never crash."""
    if tomllib is None:
        return Persona()
    try:
        with open(path, "rb") as source:
            data = tomllib.load(source).get("persona", {})
    except (OSError, ValueError):
        return Persona()
    emotions = dict(DEFAULT_EMOTIONS)
    emotions.update({str(k): str(v) for k, v in data.get("emotions", {}).items()})
    return Persona(
        name=str(data.get("name", "小锐")),
        language=str(data.get("language", "zh")),
        tone=str(data.get("tone", "")),
        max_reply_chars=int(data.get("max_reply_chars", 60)),
        identity=str(data.get("identity", "")),
        body=str(data.get("body", "")),
        self_reference=str(data.get("self_reference", "我")),
        emotions=emotions,
        templates=dict(data.get("templates", {})),
    )


@dataclass
class VehicleSnapshot:
    """What the robot tells the brain about its own body. All fields optional:
    missing telemetry degrades wording, never blocks dialogue."""
    task_state: str = "IDLE"
    task_id: str = ""
    base_connected: bool = False
    base_armed: bool = False
    base_fault: int = 0
    twist_x: float = 0.0
    twist_z: float = 0.0
    wake_angle: float | None = None
    wake_beam: int | None = None
    recent_tiers: tuple = ()


def snapshot_to_text(snapshot: VehicleSnapshot) -> str:
    """Compact context line for the cloud prompt (Chinese, stable order)."""
    parts = [f"任务={snapshot.task_state}"]
    if snapshot.base_fault:
        parts.append(f"底盘故障=0x{snapshot.base_fault:04X}")
    elif not snapshot.base_connected:
        parts.append("底盘未连接")
    elif snapshot.base_armed:
        parts.append("底盘已使能")
    else:
        parts.append("底盘待命")
    if snapshot.wake_angle is not None:
        parts.append(f"声源方位≈{snapshot.wake_angle:.0f}°")
    if snapshot.recent_tiers:
        parts.append("近期S1=" + "/".join(snapshot.recent_tiers[-3:]))
    return "；".join(parts)


def render(template_key: str, persona: Persona, **slots: object) -> str:
    """Local fallback utterance: deterministic, short, always available."""
    template = persona.templates.get(template_key, "{reason}")
    try:
        text = str(template).format(**slots)
    except (KeyError, ValueError):
        text = str(template)
    if len(text) > persona.max_reply_chars:
        text = text[: persona.max_reply_chars]
    return text
