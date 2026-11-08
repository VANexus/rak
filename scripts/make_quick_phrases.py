#!/usr/bin/env python3
"""预合成即时反应层素材(ADR-0018 层1)。

读 `src/rak_speech/config/quick_phrases.toml`(文案 + 情感档),用 qwentts.cpp
的 serena 音色逐条合成,写到 `<repo>/acoustic/models/quick/<key>.wav`。
`tts_node` 启动时按文案精确匹配这些 wav,所以这一步跑过之后,唤醒应答之类
的固定话术就不再经过推理(<100ms)。

用法:
    python3 scripts/make_quick_phrases.py                # 全量生成
    python3 scripts/make_quick_phrases.py --only ack_wake,hold
    python3 scripts/make_quick_phrases.py --force        # 覆盖已存在的 wav

情绪 → instruct 的映射来自人格卡 `[persona.emotions]`(单一真源):
改语气只改人格卡,再重跑本脚本。素材 wav 不进 git(`acoustic/models/` 被忽略)。
"""

from __future__ import annotations

import argparse
import sys
import time
import tomllib
import wave
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src" / "rak_speech"))

from rak_speech.qwentts_backend import (  # noqa: E402
    DEFAULT_INSTRUCT, SAMPLE_RATE, QwenttsSynthesizer,
)


def load_spec(path: Path) -> dict[str, dict]:
    with open(path, "rb") as fh:
        spec = tomllib.load(fh)
    return spec.get("phrases", {}) or {}


def load_emotions(path: Path) -> dict[str, str]:
    if not path.is_file():
        return {}
    with open(path, "rb") as fh:
        persona = tomllib.load(fh)
    return (persona.get("persona", {}) or {}).get("emotions", {}) or {}


def write_wav(path: Path, pcm: bytes) -> float:
    with wave.open(str(path), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(pcm)
    return len(pcm) / 2 / SAMPLE_RATE


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--spec", default=str(REPO / "src/rak_speech/config/quick_phrases.toml"))
    ap.add_argument("--persona", default=str(REPO / "src/rak_harness/config/persona.toml"))
    ap.add_argument("--out-dir", default=str(REPO / "acoustic/models/quick"))
    ap.add_argument("--only", default="", help="逗号分隔的 key 子集")
    ap.add_argument("--force", action="store_true", help="覆盖已存在的 wav")
    ap.add_argument("--speaker", default="serena")
    ap.add_argument("--server", default=str(REPO / "tools/qwentts.cpp/build/tts-server"))
    ap.add_argument("--model", default="")
    ap.add_argument("--codec", default="")
    ap.add_argument("--device", default="CPU")
    ap.add_argument("--max-seq", type=int, default=2048)
    args = ap.parse_args()

    spec = load_spec(Path(args.spec))
    if not spec:
        print(f"清单为空: {args.spec}", file=sys.stderr)
        return 1
    emotions = load_emotions(Path(args.persona))
    only = {k.strip() for k in args.only.split(",") if k.strip()}
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    todo = []
    for key, item in spec.items():
        if only and key not in only:
            continue
        wav = out_dir / f"{key}.wav"
        if wav.is_file() and not args.force:
            print(f"skip {key}(已存在, --force 覆盖)")
            continue
        todo.append((key, str(item.get("text", "")).strip(), str(item.get("emotion", ""))))
    if not todo:
        print("没有需要生成的素材")
        return 0

    model, codec = args.model, args.codec
    if not model or not codec:
        models_dir = REPO / "tools/qwentts.cpp/models"
        talkers = sorted(models_dir.glob("qwen-talker-*-customvoice-*.gguf"))
        talkers.sort(key=lambda p: ("Q4_K_M" not in p.name, p.name))
        codecs = sorted(models_dir.glob("qwen-tokenizer-*.gguf"))
        model = model or (str(talkers[0]) if talkers else "")
        codec = codec or (str(codecs[0]) if codecs else "")

    print(f"启动 tts-server({Path(model).name}, device={args.device}, max_seq={args.max_seq})…")
    synth = QwenttsSynthesizer(
        server_bin=args.server, model=model, codec=codec, device=args.device,
        speaker=args.speaker, max_seq=args.max_seq,
    )
    try:
        print(f"就绪({len(todo)} 条素材, speaker={synth.speaker})\n")
        for key, text, emotion in todo:
            instruct = emotions.get(emotion) or DEFAULT_INSTRUCT
            t0 = time.time()
            pcm = b"".join(synth.stream(text, instruct, args.speaker))
            wall = time.time() - t0
            if not pcm:
                print(f"FAIL {key}: 没有音频", file=sys.stderr)
                continue
            audio_s = write_wav(out_dir / f"{key}.wav", pcm)
            print(f"{key:12s} {emotion or '-':8s} {audio_s:5.2f}s音频 "
                  f"{wall:5.1f}s合成 RTF {wall / max(audio_s, 1e-3):4.1f}  {text}")
    finally:
        synth.close()
    print(f"\n素材目录: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
