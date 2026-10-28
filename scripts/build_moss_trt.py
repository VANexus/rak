#!/usr/bin/env python3
"""Build one MOSS ONNX graph's TRT engine per process (memory-safe, cached)."""
import sys
MOSS = "/home/xrak/rak/data/models/MOSS-TTS-Nano-100M-ONNX"
CODEC = "/home/xrak/rak/data/models/MOSS-Audio-Tokenizer-Nano-ONNX"
name = sys.argv[1]
graphs = {
    "prefill": (f"{MOSS}/moss_tts_prefill.onnx", MOSS),
    "decode": (f"{MOSS}/moss_tts_decode_step.onnx", MOSS),
    "local_decoder": (f"{MOSS}/moss_tts_local_decoder.onnx", MOSS),
    "local_cached_step": (f"{MOSS}/moss_tts_local_cached_step.onnx", MOSS),
    "local_fixed_sampled_frame": (f"{MOSS}/moss_tts_local_fixed_sampled_frame.onnx", MOSS),
    "codec_decode_full": (f"{CODEC}/moss_audio_tokenizer_decode_full.onnx", CODEC),
    "codec_decode_step": (f"{CODEC}/moss_audio_tokenizer_decode_step.onnx", CODEC),
    "codec_encode": (f"{CODEC}/moss_audio_tokenizer_encode.onnx", CODEC),
}
path, cachedir = graphs[name]
import onnxruntime as ort
from pathlib import Path
Path(f"{cachedir}/trt_cache").mkdir(exist_ok=True)
so = ort.SessionOptions()
so.log_severity_level = 3
try:
    s = ort.InferenceSession(path, so, providers=["TensorrtExecutionProvider", "CPUExecutionProvider"],
        provider_options=[{"trt_fp16_enable": True, "trt_engine_cache_enable": True,
                           "trt_engine_cache_path": f"{cachedir}/trt_cache",
                           "trt_timing_cache_enable": True, "trt_timing_cache_path": f"{cachedir}/trt_cache",
                           "trt_max_workspace_size": "134217728"}, {}])
    print(f"{name}: TRT engine OK, providers={s.get_providers()}")
except Exception as e:
    print(f"{name}: FAIL {str(e)[:300]}")
    sys.exit(1)
