"""Export TwinLiteNet+ (drivable-area + lane heads) to ONNX for TensorRT on the Orin.

Reproducible export chain for `docs/design/vision_nav_v1.md` §0.2. Run on a dev machine
that has torch; the Jetson only ever consumes the resulting .onnx/.plan.

Two constraints this script exists to capture:
  1. Input must be 384x672 (or any size whose feature map divides by the CAAM (2,4) bins).
     360x640 fails inside forward() with a reshape RuntimeError.
  2. The upstream repo imports matplotlib.pyplot and cv2 at module scope, and the export
     interpreter must be on numpy 1.x (the system cv2 is built against 1.26). Both are
     stubbed/checked here so the failure is legible instead of a C-extension traceback.

Usage:
    python3 scripts/export_twinlite_onnx.py --repo /path/to/TwinLiteNetPlus \\
        --weights small.pth --config small --out twinlite_plus_small_da.onnx
"""
from __future__ import annotations

import argparse
import sys
import types
from pathlib import Path

HEIGHT, WIDTH = 384, 672


def _stub_matplotlib() -> None:
    """model.py does `import matplotlib.pyplot as plt` for demo figures only."""
    pkg = types.ModuleType("matplotlib")
    pyplot = types.ModuleType("matplotlib.pyplot")
    pkg.pyplot = pyplot
    sys.modules.setdefault("matplotlib", pkg)
    sys.modules.setdefault("matplotlib.pyplot", pyplot)


def _check_numpy() -> None:
    import numpy

    major = int(numpy.__version__.split(".")[0])
    if major >= 2:
        raise SystemExit(
            f"numpy {numpy.__version__} breaks the system cv2 the upstream repo imports. "
            "Use a numpy<2 interpreter (or a venv with a matching opencv wheel)."
        )
    import cv2  # noqa: F401  (imported by model.py; fail early with a clear message)


def export(repo: Path, weights: Path, config: str, out: Path) -> None:
    _stub_matplotlib()
    _check_numpy()
    sys.path.insert(0, str(repo))

    import onnx
    import torch
    from model.model import TwinLiteNetPlus

    net = TwinLiteNetPlus(argparse.Namespace(config=config)).eval()
    state = torch.load(str(weights), map_location="cpu")
    if isinstance(state, dict) and "state_dict" in state:
        state = state["state_dict"]
    missing, unexpected = net.load_state_dict(state, strict=False)
    if missing or unexpected:
        raise SystemExit(
            f"weights do not match config={config}: "
            f"{len(missing)} missing, {len(unexpected)} unexpected"
        )

    dummy = torch.randn(1, 3, HEIGHT, WIDTH)
    with torch.no_grad():
        net(dummy)  # smoke the graph before exporting
    torch.onnx.export(
        net, dummy, str(out),
        input_names=["input"], output_names=["da_logits", "lane_logits"],
        opset_version=17, do_constant_folding=True, dynamo=False,
    )

    model = onnx.load(str(out))
    onnx.checker.check_model(model)
    da = [o for o in model.graph.output if o.name == "da_logits"][0]
    shape = [d.dim_value for d in da.type.tensor_type.shape.dim]
    if shape != [1, 2, HEIGHT, WIDTH]:
        raise SystemExit(f"unexpected da head shape {shape}")

    params = sum(p.numel() for p in net.parameters())
    print(f"exported config={config} params={params/1e6:.3f}M -> {out}")
    print(f"next: trtexec --onnx={out} --fp16 --saveEngine=<engine>.plan")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--repo", type=Path, required=True, help="TwinLiteNetPlus checkout")
    ap.add_argument("--weights", type=Path, required=True, help="official .pth")
    ap.add_argument("--config", choices=["nano", "small", "medium", "large"], default="small")
    ap.add_argument("--out", type=Path, default=Path("twinlite_plus_da.onnx"))
    args = ap.parse_args()
    export(args.repo.resolve(), args.weights.resolve(), args.config, args.out.resolve())


if __name__ == "__main__":
    main()
