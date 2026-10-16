#!/usr/bin/env python3
"""Bench MOSS-TTS-Nano ONNX on the Orin Nano board (2026-10-16)."""
import os, sys, time, wave
import numpy as np

SHIM = "/home/xrak/.local/tts_shims"
MOSS_REPO = "/home/xrak/rak/tools/MOSS-TTS-Nano"
MOSS_DIR = "/home/xrak/rak/data/models/MOSS-TTS-Nano-100M-ONNX"
sys.path.insert(0, SHIM)
sys.path.insert(0, MOSS_REPO)

sys.path.insert(0, "/home/xrak/rak/scripts")
from tts_bench_common import BENCH_TEXTS, report

from onnx_tts_runtime import OnnxTtsRuntime

t0 = time.time()
rt = OnnxTtsRuntime(model_dir=MOSS_DIR, thread_count=4, output_dir="/home/xrak/rak/debug/cosyvoice_bench", execution_provider="cpu")
load_s = time.time() - t0

results = []
for i, text in enumerate(BENCH_TEXTS):
    t0 = time.time()
    out = rt.synthesize(text=text, voice="Junhao", seed=42, enable_wetext=False)
    synth_s = time.time() - t0
    wav = np.asarray(out["waveform"], dtype=np.float32)
    sr = int(out["sample_rate"])
    audio_s = len(wav) / sr / 2  # stereo: frames / (sr*2ch)
    path = f"/home/xrak/rak/listen.wav"
    results.append({"text": BENCH_TEXTS[i][:12], "synth_s": round(synth_s,2),
                    "audio_s": round(audio_s,2), "sr": sr, "n_frames": len(out.get("generated_frames", []))})
report("moss_nano_ort", load_s, results)
