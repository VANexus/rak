#!/usr/bin/env python3
"""Common bench utilities for TTS model benchmarking (2026-10-16).

Measures per-model: load time, RTF (synthesis_time / audio_duration),
peak host RSS and peak CUDA reserved memory. One model per process —
run each bench separately to keep memory readings clean.
"""
import json
import resource
import time

BENCH_TEXTS = [
    # 短句（迎宾/应答场景）
    "你好，我是小 rak，很高兴认识你。",
    # 中长句，含数字与标点
    "今天是 10 月 16 日，我的电池电量还有百分之 87，一切正常。",
    # 情感句（ assess 情感表现力用，人工听评）
    "太好了！我们终于把这个问题解决了，我真的很开心！",
]

WAV_OUT_DIR = "/home/xrak/rak/debug/cosyvoice_bench"


def rss_peak_mb() -> float:
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0


def cuda_peak_mb() -> float:
    try:
        import torch
        if torch.cuda.is_available():
            return torch.cuda.max_memory_reserved() / 1024 / 1024
    except Exception:
        pass
    return 0.0


def report(model: str, load_s: float, results: list) -> None:
    rows = []
    for r in results:
        rtf = r["synth_s"] / r["audio_s"] if r["audio_s"] > 0 else float("inf")
        rows.append({**r, "rtf": round(rtf, 3)})
    summary = {
        "model": model,
        "load_s": round(load_s, 2),
        "rss_peak_mb": round(rss_peak_mb(), 1),
        "cuda_peak_mb": round(cuda_peak_mb(), 1),
        "rows": rows,
        "mean_rtf": round(sum(r["rtf"] for r in rows) / len(rows), 3) if rows else None,
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    out = f"{WAV_OUT_DIR}/bench_{model}.json"
    with open(out, "w") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print(f"saved -> {out}")
