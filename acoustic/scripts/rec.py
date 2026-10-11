#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 用法:  python3 rec.py [音频文件]      (默认 /tmp/m.wav)
# 作用:  用 sherpa-onnx 把一段中文语音 wav 变成文字（流式识别）
import sys, wave
import numpy as np
import sherpa_onnx

MODEL = "/home/xrak/rak/acoustic/models/sherpa-onnx-streaming-zipformer-zh-14M-2023-02-23"

wav_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/m.wav"
print("读音频:", wav_path)

recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(
    tokens=f"{MODEL}/tokens.txt",
    encoder=f"{MODEL}/encoder-epoch-99-avg-1.onnx",
    decoder=f"{MODEL}/decoder-epoch-99-avg-1.onnx",
    joiner=f"{MODEL}/joiner-epoch-99-avg-1.onnx",
    num_threads=2,
    sample_rate=16000,
    feature_dim=80,
    decoding_method="greedy_search",
)

wf = wave.open(wav_path, "rb")
print("声道:", wf.getnchannels(),
      " 采样率:", wf.getframerate(),
      " 时长(秒):", round(wf.getnframes() / wf.getframerate(), 2))

stream = recognizer.create_stream()
while True:
    data = wf.readframes(3200)          # 每次读 0.2 秒
    if not data:
        break
    samples = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
    stream.accept_waveform(16000, samples)
    while recognizer.is_ready(stream):
        recognizer.decode_stream(stream)

print("识别结果:", recognizer.get_result(stream))
