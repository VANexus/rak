"""TTS 分层纯逻辑(ADR-0018):分句、即时层素材清单、WAV 读取。

刻意与 ROS、与任何模型解耦:这里只有字符串和字节,zunit 可单测
(`test/test_tts_layers.py`)。`tts_node` 只做接线:服务/话题 → 分层决策。

三层(ADR-0018):
  1. 即时反应层 —— `QuickAssets`,预合成 wav 素材,按「文案精确匹配」或
     「素材 key」直出,**不经过任何推理**(<100ms)。
  2. 内容层 —— `split_sentences`,长内容拆句排队,一次只合成一句。
  3. 响应预算 —— 由节点参数控制(`sentence_pause` / 超时),纯逻辑不掺和。
"""

from __future__ import annotations

import array
import re
import tomllib
import wave
from pathlib import Path

#: 句末标点:中英文都算。分号/换行也断句(长内容拆句排队,ADR-0018 层2)。
SENTENCE_END = "。！？!?；;…\n"
#: 句内次级断点:仅当一句硬切后仍超过 max_chars 时才用。
CLAUSE_BREAK = ",，、:： "
#: 素材匹配时忽略的标点/空白:渲染出的文案与预合成文案可能差一个句号。
_STRIP_CHARS = " \t\r\n。！？!?；;…,，、.·~～"


def split_sentences(text: str, max_chars: int = 40) -> list[str]:
    """把一段话切成「一次一句」的队列。

    先按句末标点硬切(标点跟随前句),再对超过 ``max_chars`` 的句子按句内
    标点贪心再切;仍超长则硬切。空白句丢弃。空输入返回空列表。
    """
    text = (text or "").strip()
    if not text:
        return []
    if max_chars < 1:
        max_chars = 1

    rough: list[str] = []
    buf = ""
    for ch in text:
        buf += ch
        if ch in SENTENCE_END:
            rough.append(buf.strip())
            buf = ""
    if buf.strip():
        rough.append(buf.strip())

    out: list[str] = []
    for sentence in rough:
        if not sentence:
            continue
        if len(sentence) <= max_chars:
            out.append(sentence)
            continue
        out.extend(_split_long(sentence, max_chars))
    return out


def _split_long(sentence: str, max_chars: int) -> list[str]:
    """长句在句内标点处贪心切分;仍是单块超长就按字数硬切。"""
    pieces: list[str] = []
    buf = ""
    for ch in sentence:
        buf += ch
        if ch in CLAUSE_BREAK and len(buf.strip()) >= max_chars:
            pieces.append(buf.strip())
            buf = ""
    if buf.strip():
        pieces.append(buf.strip())

    out: list[str] = []
    for piece in pieces:
        while len(piece) > max_chars:
            out.append(piece[:max_chars])
            piece = piece[max_chars:]
        if piece:
            out.append(piece)
    return out


def normalize_phrase(text: str) -> str:
    """素材匹配用的归一化:去首尾空白与常见句末标点,内部空白折叠。"""
    return re.sub(r"\s+", "", (text or "").strip().strip(_STRIP_CHARS))


def read_wav_s16(path: Path) -> tuple[int, bytes]:
    """读单声道 16bit WAV → (采样率, S16LE 裸字节)。

    即时层素材是 `scripts/make_quick_phrases.py` 生成的净单声道 16bit WAV;
    非该形状直接报错,不静默重采样(音频格式错位比不出声更难查)。
    """
    with wave.open(str(path), "rb") as wf:
        if wf.getnchannels() != 1:
            raise ValueError(f"{path}: 需要单声道,实为 {wf.getnchannels()} 声道")
        if wf.getsampwidth() != 2:
            raise ValueError(f"{path}: 需要 16bit,实为 {wf.getsampwidth() * 8}bit")
        return wf.getframerate(), wf.readframes(wf.getnframes())


def trim_silence_s16(
    pcm: bytes,
    sample_rate: int,
    lead_ms: int = 30,
    tail_ms: int = 80,
    threshold_ratio: float = 0.05,
) -> bytes:
    """裁掉首尾静音,保留 lead/tail 毫秒余量(S16LE 单声道)。

    即时层素材是离线合成的,首尾天然带几百毫秒静音——实测 ack_wake 开头
    388ms、结尾 548ms。ALSA 在唤醒后 5ms 就开始"播",但那 388ms 是空白,
    听感上就是"喊完先愣半拍才开口"。加载时裁一次(缓存在内存),播放端零成本。
    阈值按峰值比例取,全静音(peak=0)原样返回,不制造空缓冲区。
    """
    if len(pcm) < 4:
        return pcm
    samples = array.array("h")
    samples.frombytes(pcm)
    peak = max(max(samples), -min(samples))
    if peak == 0:
        return pcm
    threshold = max(1, int(peak * threshold_ratio))
    n = len(samples)
    first = next((i for i, v in enumerate(samples) if abs(v) > threshold), n)
    last = next((i for i in range(n - 1, -1, -1) if abs(samples[i]) > threshold), 0)
    lead = sample_rate * lead_ms // 1000
    tail = sample_rate * tail_ms // 1000
    start = max(0, first - lead)
    end = min(n, last + 1 + tail)
    return samples[start:end].tobytes()


class QuickAssets:
    """即时反应层素材集:key → (文案, PCM 字节)。

    清单(`quick_phrases.toml`)是唯一真源,wav 按 key 命名放在素材目录里。
    wav 缺失的条目**加载时跳过**(素材目录不进 git,新机器上未生成时不能让
    节点起不来)——缺哪些由 `missing` 报给节点去 log。
    """

    def __init__(self, entries: dict[str, tuple[str, int, bytes]],
                 missing: list[str]) -> None:
        self._entries = entries
        self.missing = missing
        self._by_phrase = {normalize_phrase(t): (k, t, sr, pcm)
                           for k, (t, sr, pcm) in entries.items()}

    @property
    def keys(self) -> list[str]:
        return sorted(self._entries)

    @property
    def sample_rates(self) -> set[int]:
        return {sr for _t, sr, _pcm in self._entries.values()}

    def phrases(self) -> dict[str, str]:
        return {k: t for k, (t, _sr, _pcm) in self._entries.items()}

    def sample_rate(self, key: str) -> int:
        return self._entries[key][1]

    def pcm_for_key(self, key: str) -> bytes | None:
        entry = self._entries.get(key)
        return entry[2] if entry else None

    def pcm_for_text(self, text: str) -> bytes | None:
        """按渲染后的文案精确匹配(去掉尾标点差异)。未命中返回 None。"""
        hit = self._by_phrase.get(normalize_phrase(text))
        return hit[3] if hit else None

    def key_for_text(self, text: str) -> str | None:
        hit = self._by_phrase.get(normalize_phrase(text))
        return hit[0] if hit else None

    @classmethod
    def load(cls, spec_path: Path, wav_dir: Path) -> "QuickAssets":
        """读清单 + 素材目录。清单不存在 → 空素材集(即时层关闭)。"""
        entries: dict[str, tuple[str, int, bytes]] = {}
        missing: list[str] = []
        if not spec_path.is_file():
            return cls(entries, missing)

        with open(spec_path, "rb") as fh:
            spec = tomllib.load(fh)
        phrases = spec.get("phrases", {}) or {}
        for key, item in phrases.items():
            text = str(item.get("text", "")).strip()
            if not text:
                continue
            wav = wav_dir / f"{key}.wav"
            if not wav.is_file():
                missing.append(key)
                continue
            try:
                sr, pcm = read_wav_s16(wav)
            except (ValueError, wave.Error) as exc:
                missing.append(f"{key}({exc})")
                continue
            # 首尾静音在这里一次性裁掉(见 trim_silence_s16):素材重新生成后
            # 自动继承,播放路径不再需要任何预处理。
            entries[key] = (text, sr, trim_silence_s16(pcm, sr))
        return cls(entries, missing)
