#!/usr/bin/env python3
"""Bootstrap the CosyVoice robot-voice prompt from the existing kokoro TTS.

CosyVoice instruct2 needs a short reference wav that fixes the robot's voice
identity; emotions then ride on top via instruct strings. We generate the
reference once with the local kokoro model so the identity is 100% offline.

Usage: python3 scripts/make_cosyvoice_prompt.py [out.wav]
"""

import sys
from pathlib import Path

PROMPT_TEXT = "你好，我是小锐，一台住在机器人身体里的助手。今天天气不错，我们一起把该做的事情做好。"
OUT = Path(sys.argv[1] if len(sys.argv) > 1
          else "/home/xrak/rak/acoustic/models/cosyvoice_prompt_zh.wav")


def main() -> int:
    import sherpa_onnx

    model_dir = Path("/home/xrak/rak/acoustic/models/kokoro-multi-lang-v1_1")
    onnx = sorted(model_dir.glob("*.onnx"))[0]
    voices = model_dir / "voices.bin"
    tokens = model_dir / "tokens.txt"
    if not (voices.is_file() and tokens.is_file()):
        print(f"kokoro incomplete under {model_dir}")
        return 1
    lexicons = sorted(model_dir.glob("lexicon*.txt"))
    conf = sherpa_onnx.OfflineTtsKokoroModelConfig(
        model=str(onnx), voices=str(voices), tokens=str(tokens),
        lexicon=",".join(str(p) for p in lexicons),
        data_dir=str(model_dir),
        dict_dir=str(model_dir / "dict") if (model_dir / "dict").is_dir() else "",
        lang="zh",
    )
    config = sherpa_onnx.OfflineTtsConfig(
        model=sherpa_onnx.OfflineTtsModelConfig(kokoro=conf, num_threads=4, provider="cpu"))
    tts = sherpa_onnx.OfflineTts(config)
    audio = tts.generate(PROMPT_TEXT, sid=0, speed=1.0)
    if not audio.samples:
        print("kokoro produced no audio")
        return 1
    import soundfile as sf

    OUT.parent.mkdir(parents=True, exist_ok=True)
    sf.write(OUT, audio.samples, audio.sample_rate)
    print(f"prompt wav: {OUT} ({len(audio.samples)/audio.sample_rate:.1f}s @ {audio.sample_rate}Hz)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
