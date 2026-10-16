#!/usr/bin/env python3
"""One-shot TRT engine build for CosyVoice3 flow decoder estimator (Jetson).

Upstream convert_onnx_to_trt requests a 4GB workspace pool; this 3GB board
cannot afford it. Same logic, workspace 1GB, fp16, cached to
flow.decoder.estimator.fp16.mygpu.plan next to the ONNX.

Run once in background (10-40 min on Orin); the plan is version-locked to
the installed TRT (10.16.x) and CC 8.7.
"""

import logging
import sys
from pathlib import Path

sys.path.insert(0, "/home/xrak/rak/tools/cosyvoice")

MODEL_DIR = Path("/home/xrak/rak/acoustic/models/Fun-CosyVoice3-0.5B-2512")
PLAN = MODEL_DIR / "flow.decoder.estimator.fp16.mygpu.plan"
# fp16-weight ONNX (scripts/fp16_onnx_cosyvoice.py): the 1.33GB fp32 weights
# cannot coexist with builder libs + workspace in ~1.9GB free unified memory.
# keep_io_types=True keeps IO fp32, matching flow_matching.forward_estimator,
# which hands raw fp32 tensor pointers to the engine.
_fp16 = MODEL_DIR / "flow.decoder.estimator.fp16.onnx"
ONNX = _fp16 if _fp16.is_file() else MODEL_DIR / "flow.decoder.estimator.fp32.onnx"

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")


def main() -> int:
    import tensorrt as trt

    if PLAN.is_file() and PLAN.stat().st_size > 0:
        logging.info("plan already exists: %s", PLAN)
        return 0
    logging.info("parsing %s", ONNX)
    logger = trt.Logger(trt.Logger.INFO)
    builder = trt.Builder(logger)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, logger)
    with open(ONNX, "rb") as f:
        if not parser.parse(f.read()):
            for i in range(parser.num_errors):
                logging.error(parser.get_error(i))
            return 1
    config = builder.create_builder_config()
    # Low builder optimization level: drastically fewer tactics explored and
    # far less autotuner scratch (the default level 3 wants a 281MB scratch
    # buffer that cannot fit next to the weights on this 3.4GB board). The
    # engine may pick slightly slower kernels; building at all wins.
    config.builder_optimization_level = 0
    # JIT-only tactics: CUBLAS/CUBLAS_LT heuristics demand large workspace
    # buffers the board cannot offer next to the weights. Slower GEMMs, but
    # the engine actually builds.
    config.set_tactic_sources(
        (1 << int(trt.TacticSource.JIT_CONVOLUTIONS))
        | (1 << int(trt.TacticSource.EDGE_MASK_CONVOLUTIONS)))
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 64 << 20)  # 64MB: even 256MB let Myelin pick tactics needing 250-281MB that cannot fit in the ~700MB left after weights+libs
    config.set_flag(trt.BuilderFlag.FP16)
    profile = builder.create_optimization_profile()
    # shapes from CosyVoiceModel.get_trt_kwargs() with max/opt shrunk: the
    # frontend splits text into segments of <=80 text tokens
    # (split_paragraph token_max_n=80), so a segment never approaches the
    # upstream max of 3000 mel frames (~30s). Tactic memory scales with max
    # shape and 3000 OOMs the autotuner on this 3GB board (562MB alloc vs
    # ~284MB free). 1000 frames (~10s/segment) is ample headroom.
    for name, mn, opt, mx in [
        ("x", (2, 80, 4), (2, 80, 150), (2, 80, 800)),
        ("mask", (2, 1, 4), (2, 1, 150), (2, 1, 800)),
        ("mu", (2, 80, 4), (2, 80, 150), (2, 80, 800)),
        ("cond", (2, 80, 4), (2, 80, 150), (2, 80, 800)),
    ]:
        profile.set_shape(name, mn, opt, mx)
    config.add_optimization_profile(profile)
    logging.info("building engine (this is the long part)...")
    plan = builder.build_serialized_network(network, config)
    if plan is None:
        logging.error("build failed")
        return 1
    PLAN.write_bytes(plan.serialize())
    logging.info("plan written: %s (%d bytes)", PLAN, PLAN.stat().st_size)
    return 0


if __name__ == "__main__":
    sys.exit(main())
