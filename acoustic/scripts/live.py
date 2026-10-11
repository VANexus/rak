#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 实时流式语音识别：从麦克风读原始 PCM，边说边出字
# 用法:  arecord -D hw:2,0 -f S16_LE -r 16000 -c 1 -t raw | python3 live.py
import sys
import numpy as np
import sherpa_onnx

MODEL = "/home/xrak/rak/acoustic/models/sherpa-onnx-streaming-zipformer-zh-14M-2023-02-23"

rec = sherpa_onnx.OnlineRecognizer.from_transducer(
    tokens=f"{MODEL}/tokens.txt",
    encoder=f"{MODEL}/encoder-epoch-99-avg-1.onnx",
    decoder=f"{MODEL}/decoder-epoch-99-avg-1.onnx",
    joiner=f"{MODEL}/joiner-epoch-99-avg-1.onnx",
    num_threads=2,
    sample_rate=16000,
    feature_dim=80,
    decoding_method="greedy_search",
)

stream = rec.create_stream()
CHUNK = 3200          # 字节 = 0.1 秒 @16kHz/16bit/单声道
last = ""
print("(对着麦克风说话，Ctrl+C 退出)", flush=True)

while True:
    data = sys.stdin.buffer.read(CHUNK)
    if not data:
        break
    samples = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
    stream.accept_waveform(16000, samples)
    while rec.is_ready(stream):
        rec.decode_stream(stream)

    text = rec.get_result(stream)
    if text != last:
        print("\r识别: " + text + " " * 30, end="", flush=True)
        last = text

    # 检测到一句话结束(静音够久) -> 换行、重新开始
    if rec.is_endpoint(stream):
        if text:
            print("\n[成句] " + text)
        rec.reset(stream)
        last = ""
        print("识别: " + " " * 30, end="", flush=True)
