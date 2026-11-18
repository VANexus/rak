"""Speech node: audio-source contract, intent hand-off, and pipeline liveness.

What is *not* tested here: recognition accuracy. There is no TTS on this image
and no captured speech corpus, so nothing asserts that a given utterance decodes
to a given transcript. The pipeline is exercised end to end on a generated tone,
which proves the plumbing runs and does not invent text; accuracy stays open
until real audio is recorded from the robot.

Plain asserts; run directly or under pytest.
"""

from __future__ import annotations

import math
import struct
import sys
import threading
import time
import types
import wave
from pathlib import Path
from queue import Empty
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rak_speech.asr_node import SpeechAsr  # noqa: E402
from rak_speech.audio import (  # noqa: E402
    SAMPLE_RATE,
    AudioInbox,
    TopicSource,
    WavFileSource,
    build_source,
)


def _write_wav(path: Path, seconds: float = 1.0, rate: int = SAMPLE_RATE, channels: int = 1) -> None:
    with wave.open(str(path), "wb") as handle:
        handle.setnchannels(channels)
        handle.setsampwidth(2)
        handle.setframerate(rate)
        frames = bytearray()
        for i in range(int(rate * seconds)):
            frames += struct.pack("<h", int(3000 * math.sin(2 * math.pi * 440 * i / rate)))
        handle.writeframes(bytes(frames))


def test_build_source_rejects_unknown_kind():
    try:
        build_source("telepathy")
    except ValueError as exc:
        assert "telepathy" in str(exc)
    else:
        raise AssertionError("an unknown source kind must not silently pick a default")


def test_build_source_wav_needs_a_path():
    try:
        build_source("wav")
    except ValueError as exc:
        assert "wav_path" in str(exc)
    else:
        raise AssertionError("wav without a path must fail loudly")


def test_wav_source_accepts_16k_mono_s16(tmp_path):
    path = tmp_path / "ok.wav"
    _write_wav(path, seconds=0.5)
    chunks = list(WavFileSource(path, chunk_bytes=4000).chunks())
    assert chunks, "a non-empty file must yield chunks"
    assert sum(len(c) for c in chunks) == int(SAMPLE_RATE * 0.5) * 2


def test_wav_source_refuses_a_rate_the_model_cannot_use(tmp_path):
    """Silently resampling would produce a confident wrong transcript."""
    path = tmp_path / "44k.wav"
    _write_wav(path, seconds=0.2, rate=44100)
    try:
        list(WavFileSource(path).chunks())
    except ValueError as exc:
        assert "44100" in str(exc)
    else:
        raise AssertionError("a mismatched sample rate must be refused")


def test_wav_source_refuses_stereo(tmp_path):
    path = tmp_path / "stereo.wav"
    _write_wav(path, seconds=0.2, channels=2)
    try:
        list(WavFileSource(path).chunks())
    except ValueError as exc:
        assert "mono" in str(exc)
    else:
        raise AssertionError("stereo must be refused, not downmixed silently")


def test_build_source_topic_needs_an_inbox():
    try:
        build_source("topic")
    except ValueError as exc:
        assert "audio_inbox" in str(exc)
    else:
        raise AssertionError("topic without an audio inbox must fail loudly")


def test_topic_source_yields_whole_aligned_chunks():
    """GStreamer buffer sizes are arbitrary; an S16 sample never splits across calls."""
    inbox = AudioInbox()
    stop = threading.Event()
    chunks = TopicSource(inbox, stop=stop, chunk_bytes=100).chunks()

    inbox.put(b"\x01\x02" * 75)  # 150 bytes -> one chunk now, 50 left over
    assert next(chunks) == b"\x01\x02" * 50

    inbox.put(b"\x03\x04" * 30)  # 110 buffered -> chunk of old tail + new head
    second = next(chunks)
    assert len(second) == 100
    assert second == b"\x01\x02" * 25 + b"\x03\x04" * 25

    stop.set()
    assert list(chunks) == [], "a stopped node must not keep draining audio"


def test_topic_source_stop_ends_a_waiting_iterator():
    """A silent topic is a live stream, not an end of file: only stop may end it,
    and it must end promptly so node shutdown cannot hang on a quiet room."""
    inbox = AudioInbox()
    stop = threading.Event()
    chunks = TopicSource(inbox, stop=stop, chunk_bytes=100).chunks()

    threading.Timer(0.3, stop.set).start()
    started = time.monotonic()
    assert list(chunks) == []
    assert time.monotonic() - started < 3.0, "shutdown waited on a silent topic"


def test_inbox_budget_is_bytes_and_evicts_the_oldest():
    """The recogniser is slower than real time (0.84-1.01x measured), so the
    stream must be allowed to run ahead -- and overflow must cost the *oldest*
    audio, never the words just spoken."""
    inbox = AudioInbox(max_bytes=1000)
    assert inbox.put(b"a" * 800) == 0
    assert inbox.put(b"b" * 800) == 1, "over budget must evict the oldest chunk"
    assert inbox.depth_bytes <= 1000
    assert inbox.get(timeout=0.1) == b"b" * 800, "the newest audio must survive"

    assert inbox.put(b"c" * 400) == 0  # room again: eviction is not sticky
    assert inbox.get(timeout=0.1) == b"c" * 400


def test_inbox_reports_a_quiet_stream_instead_of_blocking_forever():
    inbox = AudioInbox()
    try:
        inbox.get(timeout=0.05)
    except Empty:
        pass
    else:
        raise AssertionError("an empty inbox must raise Empty, not block or return junk")


def test_find_model_searches_the_repo_data_models(tmp_path):
    """Since ADR-0013 the assets are at <repo>/data/models, not brain/data/models."""
    expected = tmp_path / "data" / "models" / "vosk-model-small-cn-0.22"
    expected.mkdir(parents=True)
    start = tmp_path / "src" / "rak_speech"  # not created; the walk only goes up
    # [AI] _find_model 现在按 self._engine 分支（zipformer vs vosk）；__new__ 绕过 __init__
    # 不会设该属性，这里显式给 vosk 分支（此处要验的正是 vosk 的 data/models 走查）。
    asr = SpeechAsr.__new__(SpeechAsr)
    asr._engine = "vosk"
    found = SpeechAsr._find_model(asr, starts=(start,))
    assert found == str(expected)


def test_find_model_says_empty_when_no_model_exists(tmp_path):
    start = tmp_path / "nowhere"
    start.mkdir()
    asr = SpeechAsr.__new__(SpeechAsr)
    asr._engine = "vosk"
    found = SpeechAsr._find_model(asr, starts=(start,))
    assert found == "", "an absent model must be empty, not a guess"


def test_intent_handoff_uses_the_task_layer_parser():
    """The spoken path and the typed path must agree, so both go through task_intent."""
    control = SpeechAsr._parse("暂停")
    assert control is not None and control["kind"] == "control" and control["command"] == "pause"

    navigation = SpeechAsr._parse("去 厨房")
    assert navigation is not None and navigation["kind"] == "navigate"
    assert navigation["destination"] == "厨房"

    # A sentence the robot cannot act on is a normal outcome, not an exception.
    assert SpeechAsr._parse("今天天气不错") is None
    assert SpeechAsr._parse("") is None


def test_zipformer_enables_endpoint_detection():
    """A live topic never reaches EOF. With sherpa-onnx's default
    ``enable_endpoint_detection=False`` the node would only publish in the
    final ``get_result()`` after the source ends -- i.e. never for a mic.
    """
    captured = {}

    class _FakeOnlineRecognizer:
        @staticmethod
        def from_transducer(**kwargs):
            captured.update(kwargs)
            return object()

    fake_module = types.SimpleNamespace(OnlineRecognizer=_FakeOnlineRecognizer)
    asr = SpeechAsr.__new__(SpeechAsr)
    asr._model_dir = "/fake/model/dir"  # _load_zipformer only checks non-empty

    with patch.dict(sys.modules, {"sherpa_onnx": fake_module}):
        SpeechAsr._load_zipformer(asr)

    assert captured.get("enable_endpoint_detection") is True, (
        "zipformer must enable endpoint detection or topic/arecord never publish"
    )
    assert captured.get("rule1_min_trailing_silence") == 1.2
    assert captured.get("rule2_min_trailing_silence") == 0.6


def test_topic_endpoint_publishes_transcript_without_eof():
    """Feed a live topic and assert a transcript appears *before* stop.

    This is the regression guard for the silent failure: the recogniser may
    have decoded text, but if the endpoint never fires the node publishes
    nothing and downstream sees an idle microphone forever.
    """

    class _FakeStream:
        def accept_waveform(self, sample_rate, samples):
            pass

    class _FakeRecognizer:
        def __init__(self):
            self.endpoint_calls = 0

        def create_stream(self):
            return _FakeStream()

        def is_ready(self, stream):
            return False

        def decode_stream(self, stream):
            pass

        def is_endpoint(self, stream):
            self.endpoint_calls += 1
            return self.endpoint_calls >= 2

        def get_result(self, stream):
            return "去厨房"

        def reset(self, stream):
            pass

    class _QuietLogger:
        def info(self, *args, **kwargs):
            pass

    asr = SpeechAsr.__new__(SpeechAsr)
    asr._model = _FakeRecognizer()
    asr._model_dir = "/fake/model"
    asr._source_kind = "topic"
    asr._wav_path = ""
    asr._device = "default"
    asr._inbox = AudioInbox()
    asr._stop = threading.Event()
    published = []
    asr._publish = published.append

    with patch.object(SpeechAsr, "get_logger", lambda self: _QuietLogger()):
        worker = threading.Thread(
            target=SpeechAsr._run_zipformer, args=(asr,), daemon=True
        )
        worker.start()
        asr._inbox.put(b"\x00\x00" * 2000)  # 125 ms
        asr._inbox.put(b"\x00\x00" * 2000)

        deadline = time.monotonic() + 3.0
        while not published and time.monotonic() < deadline:
            time.sleep(0.02)

        assert published == ["去厨房"], (
            "endpoint onset must publish a transcript; got "
            f"{published!r} before the topic was stopped"
        )
        assert not asr._stop.is_set(), "transcript must not wait for EOF/stop"

        asr._stop.set()
        worker.join(timeout=3.0)
        assert not worker.is_alive()


def test_recognition_pipeline_runs_and_invents_nothing(tmp_path):
    """A pure tone must not decode into words."""
    try:
        import vosk  # noqa: F401
    except ImportError:
        print("  skip: vosk not installed")
        return

    path = tmp_path / "tone.wav"
    _write_wav(path, seconds=1.0)

    import json

    import vosk

    vosk.SetLogLevel(-1)
    # __new__ 绕过 __init__，_engine 不会被设置，而 _find_model 要按 _engine 分支
    # （zipformer 走 acoustic/models，vosk 走 data/models）。这里显式给 vosk 分支。
    # 2026-11-13：此前缺这一行，测试在新机器上直接 AttributeError —— 不是断言失败，
    # 是测试自己崩了，所以它一直"红着但没人看"。
    bare = SpeechAsr.__new__(SpeechAsr)
    bare._engine = "vosk"
    model_dir = SpeechAsr._find_model(bare)
    if not model_dir:
        print("  skip: no Vosk model present")
        return
    recognizer = vosk.KaldiRecognizer(vosk.Model(model_dir), SAMPLE_RATE)
    for chunk in WavFileSource(path).chunks():
        recognizer.AcceptWaveform(chunk)
    text = json.loads(recognizer.FinalResult()).get("text", "").strip()
    assert text == "", f"a 440 Hz tone decoded as {text!r}"


def main() -> None:
    import tempfile

    failures = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                if "tmp_path" in fn.__code__.co_varnames[: fn.__code__.co_argcount]:
                    with tempfile.TemporaryDirectory() as directory:
                        fn(Path(directory))
                else:
                    fn()
                print(f"ok  {name}")
            except Exception as exc:  # noqa: BLE001 -- a runner reports, it does not raise
                failures += 1
                print(f"FAIL {name}: {type(exc).__name__}: {exc}")
    if failures:
        raise SystemExit(f"{failures} test(s) failed")
    print("speech tests passed")


if __name__ == "__main__":
    main()
