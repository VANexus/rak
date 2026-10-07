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
import wave
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rak_speech.asr_node import SpeechAsr  # noqa: E402
from rak_speech.audio import SAMPLE_RATE, WavFileSource, build_source  # noqa: E402


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
    model_dir = SpeechAsr._find_model(SpeechAsr.__new__(SpeechAsr))
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
