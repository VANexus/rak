"""Model discovery and sherpa-onnx wrappers for speaker identification.

Kept out of `speaker_core` so the pure-math tests never touch onnxruntime, and
out of `speaker_node` so the enrolment CLI reuses the exact same extractor/VAD
construction. Everything here imports sherpa_onnx lazily: a machine without the
pip package must still be able to import modules and run the maths tests.

Model files are gitignored team assets under `acoustic/models/` (same rule as
the TTS models, acoustic/.gitignore):

    3dspeaker_speech_campplus_sv_zh-cn_16k-common.onnx   (CAM++ 中文 16k, ~28MB)
    silero_vad.onnx                                       (~2MB, sherpa-onnx VAD)

Both auto-search by filename, so a newer quantised CAM++ can just replace the
file; `-p model:=` / `-p vad_model:=` take priority over the search.
"""

from __future__ import annotations

from pathlib import Path

SAMPLE_RATE = 16000
CAMPPLUS_GLOB = "*campplus*.onnx"
VAD_GLOB = "silero_vad.onnx"


def find_models() -> tuple[Path | None, Path | None]:
    """Search the repo for the speaker and VAD onnx files (walk-up like tts_node).

    Priority: an explicit `speaker/` subdirectory first — model bundles such as
    CosyVoice carry their own `campplus.onnx` (a torch-side speaker encoder with
    no sherpa metadata), and a blind rglob would happily pick that one and die
    with "Unknown model type". Only when no `speaker/` subdir exists do we fall
    back to a global search that skips the bundled model trees.
    """
    skip = ("cosyvoice", "kokoro", "vits", "zipformer", "moss", "ming")
    for start in (Path(__file__).resolve().parent, Path.cwd()):
        for base in (start, *start.parents):
            for rel in ("acoustic/models", "data/models"):
                candidate = base / rel
                if not candidate.is_dir():
                    continue
                # 1) 专用 speaker/ 子目录(约定位置,下载模型的落脚点)
                for sub in sorted(candidate.glob("*/")):
                    if sub.name != "speaker":
                        continue
                    speaker = next(iter(sorted(sub.glob(CAMPPLUS_GLOB))), None)
                    vad = next(iter(sorted(sub.glob(VAD_GLOB))), None)
                    if speaker is not None and vad is not None:
                        return speaker, vad
                # 2) 兜底全搜索,但坚决跳过别的模型包自带的同名文件
                speaker = next(
                    (
                        p
                        for p in sorted(candidate.rglob(CAMPPLUS_GLOB))
                        if not any(part in str(p).lower() for part in skip)
                    ),
                    None,
                )
                vad = next(
                    (
                        p
                        for p in sorted(candidate.rglob(VAD_GLOB))
                        if not any(part in str(p).lower() for part in skip)
                    ),
                    None,
                )
                if speaker is not None and vad is not None:
                    return speaker, vad
    return None, None


def build_vad(model_path: str | Path):
    """Silero VAD, tuned for short commands: 0.25 s to open, 0.25 s to close."""
    import sherpa_onnx

    config = sherpa_onnx.VadModelConfig()
    config.silero_vad.model = str(model_path)
    config.silero_vad.min_silence_duration = 0.25
    config.silero_vad.min_speech_duration = 0.25
    config.sample_rate = SAMPLE_RATE
    config.num_threads = 1
    try:
        return sherpa_onnx.VoiceActivityDetector(config, buffer_size_in_seconds=30)
    except TypeError:  # 1.13.x 新版参数名
        return sherpa_onnx.VoiceActivityDetector(config, buffer_size=30)


def build_extractor(model_path: str | Path):
    """CAM++ embedding extractor — one thread, CPU, ~20-60 ms per 1.5 s window."""
    import sherpa_onnx

    config = sherpa_onnx.SpeakerEmbeddingExtractorConfig(
        model=str(model_path), num_threads=1, debug=False, provider="cpu"
    )
    return sherpa_onnx.SpeakerEmbeddingExtractor(config)


def embed(extractor, samples) -> "list[float]":
    """Embedding for a float32 [-1,1] waveform (sherpa-onnx 1.13 API: compute)."""
    stream = extractor.create_stream()
    stream.accept_waveform(SAMPLE_RATE, samples)
    stream.input_finished()
    if not extractor.is_ready(stream):
        raise ValueError("embedding stream not ready (audio too short?)")
    return extractor.compute(stream)
