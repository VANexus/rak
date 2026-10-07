#!/usr/bin/env python3
"""Export the pinned Laya decision checkpoint as a BF16 ONNX graph.

Run with the pinned upstream Laya checkout on PYTHONPATH. We construct the
model in the checkpoint's compute dtype and stream safetensors one tensor at a
time so the FP32 model and a second full checkpoint copy are never allocated.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

MIN_AVAILABLE_BYTES = 2 * 1024**3

# Submodules that must keep fp32 weights: the forward promotes their inputs to fp32.
_FP32_MODULES = ("act_head",)


def _memavailable_bytes() -> int:
    for line in Path("/proc/meminfo").read_text().splitlines():
        if line.startswith("MemAvailable:"):
            return int(line.split()[1]) * 1024
    raise RuntimeError("MemAvailable is not reported in /proc/meminfo")


def _copy_checkpoint_streaming(model, weights_path: Path, torch) -> int:
    from safetensors import safe_open

    state = model.state_dict()
    expected = set(state)
    copied = 0

    with safe_open(str(weights_path), framework="pt", device="cpu") as weights:
        available = set(weights.keys())
        missing = expected - available
        unexpected = available - expected
        if missing or unexpected:
            raise ValueError(
                "Checkpoint keys do not match the constructed model: "
                f"{len(missing)} missing, {len(unexpected)} unexpected."
            )

        with torch.no_grad():
            for name, destination in state.items():
                source = weights.get_tensor(name)
                if tuple(source.shape) != tuple(destination.shape):
                    raise ValueError(
                        f"Shape mismatch for {name}: expected "
                        f"{tuple(destination.shape)}, got {tuple(source.shape)}."
                    )
                destination.copy_(source)
                copied += 1
                del source

    return copied


def _build_model_in_compute_dtype(config, encoder_dir, torch, compute_dtype):
    """Build the pinned model with every parameter in the checkpoint's compute dtype.

    transformers instantiates a model under its own ``config.dtype`` (the pinned encoder
    config records ``float32``) by calling ``torch.set_default_dtype``, which silently
    overrides the dtype the caller set. The head modules honour the caller's default, but
    the encoder would come back in fp32 while its activations are bf16 -- a mixed-dtype
    graph that only fails later, inside the ONNX decomposition. Wrapping
    ``AutoConfig.from_pretrained`` forces the encoder onto the compute dtype without
    editing the pinned, gitignored asset.
    """
    from transformers import AutoConfig

    from laya.common import build_model

    original = AutoConfig.from_pretrained

    def from_pretrained(*args, **kwargs):
        cfg = original(*args, **kwargs)
        cfg.dtype = compute_dtype
        return cfg

    AutoConfig.from_pretrained = from_pretrained
    try:
        return build_model(
            config,
            encoder_dir=str(encoder_dir) if encoder_dir.is_dir() else None,
            pretrained=False,
        )
    finally:
        AutoConfig.from_pretrained = original


def export(model_dir: Path, output_path: Path) -> None:
    available = _memavailable_bytes()
    if available < MIN_AVAILABLE_BYTES:
        raise MemoryError(
            f"Need at least 2 GiB MemAvailable before export; found "
            f"{available / 1024**3:.2f} GiB. Close workloads and retry."
        )

    config_path = model_dir / "rl_agent_config.json"
    weights_path = model_dir / "model.safetensors"
    if not config_path.is_file() or not weights_path.is_file():
        raise FileNotFoundError(f"Missing Laya config or weights under {model_dir}")

    config = json.loads(config_path.read_text())
    compute_name = str(config.get("amp_dtype", "")).lower()

    import torch

    torch.set_num_threads(1)
    dtype_by_name = {
        "bf16": torch.bfloat16,
        "bfloat16": torch.bfloat16,
        "fp16": torch.float16,
        "float16": torch.float16,
    }
    if compute_name not in dtype_by_name:
        raise ValueError(f"Unsupported checkpoint amp_dtype: {compute_name!r}")

    previous_dtype = torch.get_default_dtype()
    compute_dtype = dtype_by_name[compute_name]
    try:
        torch.set_default_dtype(compute_dtype)

        encoder_dir = model_dir / "encoder"
        model = _build_model_in_compute_dtype(config, encoder_dir, torch, compute_dtype)
        # The forward promotes the decision head's input to fp32 (`.float()` on the scorer
        # output, the entropy/top-2 features and `pooled`), while the encoder runs in the
        # compute dtype. `act_head` therefore has to stay fp32 or the graph mixes an fp32
        # activation into a bf16 Linear, which TensorRT rejects ("IMatrixMultiplyLayer must
        # have same input types"). Cast before the streaming copy so the F16 checkpoint
        # weights land in fp32 directly rather than through bf16.
        model.act_head = model.act_head.float()
        copied = _copy_checkpoint_streaming(model, weights_path, torch)
        model.eval()

        stray = sorted(
            f"{name}={param.dtype}"
            for name, param in model.named_parameters()
            if param.dtype != compute_dtype and not name.startswith(_FP32_MODULES)
        )
        if stray:
            raise RuntimeError(
                f"Model has parameters outside {compute_dtype}: {stray}. The graph would "
                "fail to decompose; do not continue."
            )

        batch, seq_len, num_markers = 2, 128, 3
        inputs = (
            torch.randint(0, 100, (batch, seq_len), dtype=torch.long),
            torch.ones((batch, seq_len), dtype=torch.long),
            torch.tensor([[1, 5, 9]] * batch, dtype=torch.long),
            torch.ones((batch, num_markers), dtype=torch.bool),
            torch.zeros(batch, dtype=torch.long),
        )
        input_names = [
            "input_ids",
            "attention_mask",
            "marker_pos",
            "marker_mask",
            "qtype",
        ]
        output_names = ["logits", "act_logits"]
        batch_dim = torch.export.Dim("batch_size", min=1)
        seq_dim = torch.export.Dim("seq_len", min=96, max=1024)
        # The model has separate single-marker and multi-marker branches.
        marker_dim = torch.export.Dim("num_markers", min=2, max=7)
        dynamic_shapes = (
            {0: batch_dim, 1: seq_dim},
            {0: batch_dim, 1: seq_dim},
            {0: batch_dim, 1: marker_dim},
            {0: batch_dim, 1: marker_dim},
            {0: batch_dim},
        )

        output_path.parent.mkdir(parents=True, exist_ok=True)
        print(
            f"Loaded {copied} tensors as {compute_name}; "
            f"exporting {compute_dtype} ONNX to {output_path}",
            flush=True,
        )
        torch.onnx.export(
            model,
            inputs,
            str(output_path),
            export_params=True,
            opset_version=18,
            do_constant_folding=True,
            input_names=input_names,
            output_names=output_names,
            dynamic_shapes=dynamic_shapes,
            dynamo=True,
        )
        print(f"Export complete: {output_path}", flush=True)
    finally:
        torch.set_default_dtype(previous_dtype)


def main() -> None:
    os.environ.setdefault("USE_TF", "0")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    export(args.model, args.output)


if __name__ == "__main__":
    main()
