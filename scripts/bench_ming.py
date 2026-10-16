#!/usr/bin/env python3
"""Bench Ming-omni-tts-0.5B on Orin Nano (2026-10-16). Run from cookbooks dir."""
import sys, os, time
sys.path.insert(0, "/home/xrak/.local/tts_shims")  # torchaudio shim
SHIM2 = "/home/xrak/.local/tts_shims/torchaudio"
sys.path.insert(0, "/home/xrak/rak/scripts")
from tts_bench_common import BENCH_TEXTS, report
import numpy as np, torch, soundfile as sf

COOK = "/home/xrak/rak/tools/Ming-omni-tts/cookbooks"
MODEL = "/home/xrak/rak/data/models/Ming-omni-tts-0.5B"
sys.path.append(os.path.dirname(COOK))
os.chdir(COOK)

from modeling_bailingmm import BailingMMNativeForConditionalGeneration
from test import MingAudio

t0 = time.time()
au = MingAudio(MODEL, device=sys.argv[1] if len(sys.argv) > 1 else "cpu")
load_s = time.time() - t0

results = []
for i, text in enumerate(BENCH_TEXTS):
    t0 = time.time()
    resp = au.speech_generation(
        prompt="Please generate speech based on the following description.\n",
        text=text,
        use_zero_spk_emb=True,
        instruction={"情感": "高兴"} if i == 2 else None,
        max_decode_steps=200,
        output_wav_path=f"/home/xrak/rak/debug/cosyvoice_bench/ming_{i}.wav")
    synth_s = time.time() - t0
    data, sr = sf.read(f"/home/xrak/rak/debug/cosyvoice_bench/ming_{i}.wav")
    audio_s = len(data) / sr
    results.append({"text": text[:12], "synth_s": round(synth_s,2), "audio_s": round(audio_s,2), "sr": sr})
report("ming_05b_" + (sys.argv[1] if len(sys.argv)>1 else "cpu"), load_s, results)
