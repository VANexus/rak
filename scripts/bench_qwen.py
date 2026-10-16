#!/usr/bin/env python3
"""Bench Qwen3-TTS-12Hz-0.6B-CustomVoice on Orin Nano (2026-10-16)."""
import sys, time
import numpy as np

sys.path.insert(0, "/home/xrak/.local/tts_shims")  # torchaudio shim
sys.path.insert(1, "/home/xrak/rak/tools/Qwen3-TTS")
sys.path.insert(0, "/home/xrak/rak/scripts")
from tts_bench_common import BENCH_TEXTS, report
import torch, soundfile as sf
from qwen_tts import Qwen3TTSModel

MODEL = "/home/xrak/rak/data/models/Qwen3-TTS-12Hz-0.6B-CustomVoice"
DEV = sys.argv[1] if len(sys.argv) > 1 else "cpu"

t0 = time.time()
tts = Qwen3TTSModel.from_pretrained(MODEL, device_map=DEV, dtype=torch.bfloat16)
load_s = time.time() - t0
print("speakers:", sorted(tts._supported_speakers_set() or [])[:20], flush=True)

results = []
for i, text in enumerate(BENCH_TEXTS):
    torch.cuda.synchronize() if DEV.startswith("cuda") else None
    t0 = time.time()
    wavs, sr = tts.generate_custom_voice(
        text=text, language="Chinese", speaker="Ryan" if "Ryan" in (tts._supported_speakers_set() or {"Ryan"}) else sorted(tts._supported_speakers_set())[0],
        instruct="开心地、热情地说" if i == 2 else "")
    torch.cuda.synchronize() if DEV.startswith("cuda") else None
    synth_s = time.time() - t0
    audio_s = len(wavs[0]) / sr
    sf.write(f"/home/xrak/rak/debug/cosyvoice_bench/qwen_{i}.wav", np.asarray(wavs[0], dtype=np.float32), sr)
    results.append({"text": text[:12], "synth_s": round(synth_s,2), "audio_s": round(audio_s,2), "sr": sr})
report("qwen3tts_06b_" + DEV, load_s, results)
