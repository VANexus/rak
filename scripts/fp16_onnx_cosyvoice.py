#!/usr/bin/env python3
"""Convert the CosyVoice3 flow-estimator ONNX to fp16 weights.

The 1.33GB fp32 ONNX cannot fit alongside TRT builder libraries + workspace
in this board's ~1.9GB free unified memory during engine build; fp16 halves
the weight residency. LayerNorm/Softmax-class ops stay fp32 for numerics.
Output: flow.decoder.estimator.fp16.onnx (one-time, cached).
"""

import logging
import sys
from pathlib import Path

MODEL_DIR = Path("/home/xrak/rak/acoustic/models/Fun-CosyVoice3-0.5B-2512")
SRC = MODEL_DIR / "flow.decoder.estimator.fp32.onnx"
DST = MODEL_DIR / "flow.decoder.estimator.fp16.onnx"

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")

OP_BLOCK = [
    "LayerNormalization", "GroupNormalization", "InstanceNormalization",
    "Softmax", "LogSoftmax", "Erf", "Log", "Exp", "ReduceMean", "Div",
]


def main() -> int:
    if DST.is_file() and DST.stat().st_size > 0:
        logging.info("already converted: %s", DST)
        return 0
    from onnxruntime.transformers import float16 as fp16
    import onnx

    logging.info("loading %s", SRC)
    model = onnx.load(str(SRC))
    logging.info("converting to fp16 (op_block_list=%s)", OP_BLOCK)
    model = fp16.convert_float_to_float16(
        model, keep_io_types=True, op_block_list=OP_BLOCK,
        disable_shape_infer=False,
    )
    logging.info("saving %s", DST)
    onnx.save(model, str(DST))
    logging.info("done: %d bytes", DST.stat().st_size)
    return 0


if __name__ == "__main__":
    sys.exit(main())
