"""tts_layers / qwentts_backend 的单测:无 ROS、无模型、无 GPU。

覆盖 ADR-0018 三层里可判定的部分:分句(层2)、素材清单匹配(层1)、
流式 PCM 的帧对齐(内容层的字节契约)。
"""

from __future__ import annotations

import http.server
import threading
import wave
from pathlib import Path

import pytest

from rak_speech import qwentts_backend
from rak_speech.qwentts_backend import QwenttsSynthesizer, QwenttsUnavailable
from rak_speech.tts_layers import (
    QuickAssets,
    normalize_phrase,
    read_wav_s16,
    split_sentences,
    trim_silence_s16,
)

# -- 层2:分句 ------------------------------------------------------------------


def test_split_sentences_breaks_on_end_punctuation():
    assert split_sentences("我在。好，走了！") == ["我在。", "好，走了！"]


def test_split_sentences_keeps_trailing_fragment():
    assert split_sentences("先说半句没有结尾") == ["先说半句没有结尾"]


def test_split_sentences_drops_blank():
    assert split_sentences("  \n 。\n") == ["。"]


def test_split_sentences_empty():
    assert split_sentences("") == []
    assert split_sentences("   ") == []


def test_split_sentences_long_clause_gets_cut_at_max_chars():
    text = "第一个分句很长很长," + "第二个分句也不短,"
    out = split_sentences(text, max_chars=8)
    assert all(len(s) <= 8 for s in out), out
    assert "".join(out) == text


def test_split_sentences_hard_cut_when_no_break_char():
    out = split_sentences("啊" * 25, max_chars=10)
    assert out == ["啊" * 10, "啊" * 10, "啊" * 5]


# -- 层1:素材清单 ---------------------------------------------------------------


def _write_wav(path: Path, pcm: bytes, rate: int = 24000) -> None:
    with wave.open(str(path), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(rate)
        wf.writeframes(pcm)


def test_normalize_phrase_ignores_trailing_punctuation_and_space():
    assert normalize_phrase(" 我在。 ") == normalize_phrase("我在")
    assert normalize_phrase("好，停一下！") == normalize_phrase("好，停一下")


def test_quick_assets_match_by_text_and_key(tmp_path: Path):
    _write_wav(tmp_path / "ack_wake.wav", b"\x01\x02\x03\x04")
    spec = tmp_path / "quick.toml"
    spec.write_text(
        '[phrases.ack_wake]\ntext = "我在。"\nemotion = "neutral"\n', encoding="utf-8"
    )
    assets = QuickAssets.load(spec, tmp_path)
    assert assets.keys == ["ack_wake"]
    assert assets.pcm_for_key("ack_wake") == b"\x01\x02\x03\x04"
    # 渲染文案少一个句号也要命中
    assert assets.pcm_for_text("我在") == b"\x01\x02\x03\x04"
    assert assets.key_for_text("我在。") == "ack_wake"
    assert assets.pcm_for_text("别的句子") is None
    assert assets.sample_rates == {24000}
    assert assets.missing == []


def test_quick_assets_skip_missing_wav_without_failing(tmp_path: Path):
    spec = tmp_path / "quick.toml"
    spec.write_text(
        '[phrases.ok]\ntext = "好嘞。"\n[phrases.hold]\ntext = "嗯……"\n',
        encoding="utf-8",
    )
    assets = QuickAssets.load(spec, tmp_path)
    assert assets.keys == []
    assert sorted(assets.missing) == ["hold", "ok"]
    assert assets.pcm_for_text("好嘞。") is None


def test_quick_assets_empty_when_spec_absent(tmp_path: Path):
    assets = QuickAssets.load(tmp_path / "nope.toml", tmp_path)
    assert assets.keys == []
    assert assets.missing == []


def test_read_wav_s16_rejects_stereo(tmp_path: Path):
    path = tmp_path / "stereo.wav"
    with wave.open(str(path), "wb") as wf:
        wf.setnchannels(2)
        wf.setsampwidth(2)
        wf.setframerate(24000)
        wf.writeframes(b"\x00\x00" * 4)
    with pytest.raises(ValueError):
        read_wav_s16(path)


# -- 内容层:流式帧对齐 ------------------------------------------------------------


class _ChunkedHandler(http.server.BaseHTTPRequestHandler):
    body = b""
    status = 200

    def do_POST(self):  # noqa: N802 -- BaseHTTPRequestHandler 接口
        length = int(self.headers.get("Content-Length", "0"))
        self.rfile.read(length)
        self.send_response(self.status)
        self.send_header("Content-Type", "audio/pcm")
        self.send_header("Content-Length", str(len(self.body)))
        self.end_headers()
        self.wfile.write(self.body)

    def log_message(self, *args):  # 静音
        pass


def _fake_server(monkeypatch, body: bytes, read_chunk: int = 3):
    handler = type("H", (_ChunkedHandler,), {"body": body})
    httpd = http.server.HTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    monkeypatch.setattr(qwentts_backend, "READ_CHUNK", read_chunk)
    return httpd


def _synth_without_spawn(port: int, tmp_path: Path) -> QwenttsSynthesizer:
    """绕过 __init__ 的进程管理,直接测 _post_stream 的字节契约。"""
    synth = QwenttsSynthesizer.__new__(QwenttsSynthesizer)
    synth._host = "127.0.0.1"
    synth._port = port
    synth._stall_timeout = 10.0
    synth._lock = threading.Lock()
    synth._speaker = "serena"
    synth._instruct = "清晰自然、亲切温和"
    return synth


def test_stream_yields_even_chunks_and_preserves_bytes(monkeypatch, tmp_path: Path):
    body = bytes(range(7))          # 7 字节:3+3+1 的读取节奏
    httpd = _fake_server(monkeypatch, body, read_chunk=3)
    try:
        synth = _synth_without_spawn(httpd.server_address[1], tmp_path)
        chunks = list(synth.stream("你好"))
    finally:
        httpd.shutdown()
    assert all(len(c) % 2 == 0 for c in chunks), chunks
    assert b"".join(chunks) == body[:6]  # 尾字节是半个样本,丢弃


def test_stream_single_chunk(monkeypatch, tmp_path: Path):
    body = b"\x10\x20\x30\x40"
    httpd = _fake_server(monkeypatch, body, read_chunk=4096)
    try:
        synth = _synth_without_spawn(httpd.server_address[1], tmp_path)
        assert b"".join(synth.stream("你好")) == body
    finally:
        httpd.shutdown()


def test_stream_empty_text_yields_nothing(monkeypatch, tmp_path: Path):
    synth = _synth_without_spawn(1, tmp_path)  # 端口不会被碰到
    assert list(synth.stream("   ")) == []


def test_http_error_raises_qwentts_error(monkeypatch, tmp_path: Path):
    httpd = _fake_server(monkeypatch, b"boom", read_chunk=4096)
    httpd.RequestHandlerClass.status = 500
    try:
        synth = _synth_without_spawn(httpd.server_address[1], tmp_path)
        with pytest.raises(qwentts_backend.QwenttsError):
            list(synth.stream("你好"))
    finally:
        httpd.shutdown()


def test_missing_binary_is_unavailable():
    with pytest.raises(QwenttsUnavailable):
        QwenttsSynthesizer(server_bin="/nonexistent/tts-server",
                           model="/nonexistent/m.gguf", codec="/nonexistent/c.gguf")


def test_node_resolves_repo_paths():
    """节点从安装副本里也要能找回 gitignored 的 tools/ 与 acoustic/(回归:
    曾经把「命中路径的基目录」当成仓库根,于是去找了 <repo>/models)。"""
    pytest.importorskip("rak_interfaces.srv")
    pytest.importorskip("audio_common_msgs.msg")
    from rak_speech.tts_node import SpeechTts

    # 2026-11-13：这两个断言以前直接要求 tools/qwentts.cpp 与 acoustic/ 存在。
    # 但 tools/ 是 **gitignored 的本地目录**（.gitignore 的 /tools/），fresh clone
    # 或新机器上根本没有；acoustic/ 是音色资产目录，也不保证每台机器都有。
    # 于是这条测试在干净环境里必红，而它想验证的其实是"解析逻辑能找回仓库内的
    # 相对路径"，不是"这台机器上恰好有这些文件"。
    # 改法：先用**一定存在**的仓库文件验证解析能力，再对可选目录做条件断言。
    repo_root = Path(__file__).resolve().parents[2]
    anchor = SpeechTts._resolve_repo_path("README.md")
    assert anchor is not None and anchor.name == "README.md", "解析逻辑找不到仓库根下的 README.md"

    for optional in ("tools/qwentts.cpp", "acoustic"):
        if not (repo_root / optional).exists():
            print(f"  skip: {optional} 不在本机（gitignored 本地资产）")
            continue
        found = SpeechTts._resolve_repo_path(optional)
        assert found is not None and found.name == Path(optional).name

    assert SpeechTts._resolve_repo_path("no/such/thing") is None


# -- 层1 素材:首尾静音裁剪(2026-10-17 实测 ack_wake 首 388ms/尾 548ms) -------
def _tone_pcm(sr: int, ms: int, amp: int = 10000) -> bytes:
    import math

    out = bytearray()
    for i in range(sr * ms // 1000):
        v = int(amp * math.sin(2 * math.pi * 200 * i / sr))
        out += v.to_bytes(2, "little", signed=True)
    return bytes(out)


def test_trim_silence_keeps_lead_and_tail_margins():
    sr = 16000
    silence = b"\x00\x00" * (sr // 2)  # 每侧 500ms 静音
    pcm = silence + _tone_pcm(sr, 400) + silence
    out = trim_silence_s16(pcm, sr, lead_ms=30, tail_ms=80)
    ms = len(out) // 2 * 1000 // sr
    assert abs(ms - 510) <= 6  # 400ms 语音 + 30ms 预备 + 80ms 收尾
    assert len(out) < len(pcm)


def test_trim_silence_all_silent_returned_as_is():
    sr = 16000
    pcm = b"\x00\x00" * sr
    assert trim_silence_s16(pcm, sr) == pcm  # 全静音不制造空缓冲区


def test_quick_assets_load_applies_trim(tmp_path):
    """加载即裁剪:文件里 1.3s、命中后应 ~410ms,播放路径零预处理。"""
    sr = 16000
    wav = tmp_path / "ack_wake.wav"
    with wave.open(str(wav), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sr)
        wf.writeframes(b"\x00\x00" * (sr // 2) + _tone_pcm(sr, 300) + b"\x00\x00" * (sr // 2))
    spec = tmp_path / "quick.toml"
    spec.write_text('[phrases.ack_wake]\ntext = "我在。"\n', encoding="utf-8")
    assets = QuickAssets.load(spec, tmp_path)
    pcm = assets.pcm_for_text("我在。")
    assert pcm is not None
    assert len(pcm) // 2 * 1000 // sr < 520  # 从原始 1.3s 显著缩短
