#!/usr/bin/env python3
"""RTF bench for the CosyVoice backend: one resident load, N utterances.

Usage: python3 scripts/bench_cosyvoice_rtf.py [--no-trt]
RAK_COSYVOICE_TRT is respected; --no-trt forces the PyTorch estimator.
"""

import argparse
import functools
import time

print = functools.partial(print, flush=True)

TEXTS = [
    "你好，我是小锐，很高兴见到你。",
    "现在温度是二十四度，湿度百分之四十，今天适合出门散步。",
    "好的，我这就过去。请稍等一下，我大概需要三分钟到达那里。",
    "这条消息用来测试稍长一点的句子，看看整段合成的实时率大概是多少，以及会不会出现明显的卡顿或者断句问题。",
]

ap = argparse.ArgumentParser()
ap.add_argument("--no-trt", action="store_true")
args = ap.parse_args()

import os
if args.no_trt:
    os.environ["RAK_COSYVOICE_TRT"] = "0"

t0 = time.time()
from rak_speech.cosyvoice_backend import CosyvoiceSynthesizer
synth = CosyvoiceSynthesizer()
print(f"load: {time.time() - t0:.1f}s trt={synth.load_trt}")

total_audio = 0.0
total_wall = 0.0
for i, text in enumerate(TEXTS * 2):
    t = time.time()
    sr, samples = synth.synthesize(text)
    wall = time.time() - t
    dur = len(samples) / sr
    total_audio += dur
    total_wall += wall
    print(f"[{i}] {dur:.2f}s audio, {wall:.2f}s wall, rtf={wall / dur:.2f}")
print(f"MEAN rtf={total_wall / total_audio:.2f}  ({total_audio:.1f}s audio / {total_wall:.1f}s)")
