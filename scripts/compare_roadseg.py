"""A/B compare road-segmentation candidates on the same images, on this board.

Runs:
  * LFD-RoadSeg      — TorchScript `.ts`, 640x480, ImageNet normalization (needs torch)
  * TwinLiteNet+     — TensorRT engine, 384x672, unit normalization (the v1.0 candidate)

Reports per-model latency and a "soft boundary" proxy: the share of pixels whose class
probability sits in (0.25, 0.75) relative to the number of boundary pixels. Lower is a
sharper edge, which is what path following actually consumes. This is a proxy, not an
mIoU: labelled ground truth for the robot's own venue does not exist yet.

Writes overlay PNGs so a human can judge the masks before any deployment decision.

Usage:
    python3 scripts/compare_roadseg.py --images /tmp/imgs --out /tmp/ab \\
        --engine data/models/roadseg/twinlite_plus_small_fp16_384x672.plan \\
        --lfd-model data/models/roadseg/lfd_roadseg_v6.ts
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np

_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def _soft_boundary_ratio(probability: np.ndarray) -> float:
    """Share of ambiguous pixels per boundary pixel (0 = razor edge, 1 = mush)."""
    hard = (probability > 0.5).astype(np.uint8) * 255
    edge = cv2.morphologyEx(hard, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8))
    boundary_pixels = int((edge > 0).sum())
    if boundary_pixels == 0:
        return float("nan")
    ambiguous = ((probability > 0.25) & (probability < 0.75)).sum()
    return float(ambiguous) / boundary_pixels


def _overlay(bgr: np.ndarray, probability: np.ndarray, threshold: float) -> np.ndarray:
    mask = (probability > threshold)
    out = bgr.copy()
    tint = np.array([0, 0, 255], dtype=np.float32)  # road = red in BGR
    out[mask] = (out[mask].astype(np.float32) * 0.5 + tint * 0.5).astype(np.uint8)
    return out


class LfdRunner:
    """Legacy TorchScript path, kept only for the comparison."""

    def __init__(self, model_path: Path, width: int, height: int) -> None:
        import torch  # noqa: PLC0415

        self._torch = torch
        self._width, self._height = width, height
        self._device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self._model = torch.jit.load(str(model_path), map_location=self._device).eval()
        self.name = f"LFD-RoadSeg ({model_path.name}, {width}x{height}, {self._device})"

    def infer(self, bgr: np.ndarray) -> np.ndarray:
        torch = self._torch
        rgb = cv2.cvtColor(cv2.resize(bgr, (self._width, self._height)), cv2.COLOR_BGR2RGB)
        normalized = (rgb.astype(np.float32) / 255.0 - _MEAN) / _STD
        chw = np.ascontiguousarray(normalized.transpose(2, 0, 1)[None, ...])
        with torch.inference_mode():
            logits = self._model(torch.from_numpy(chw).to(self._device))
        if isinstance(logits, (tuple, list)):
            logits = logits[0]
        prob = torch.softmax(logits.float(), dim=1)[0, 1].cpu().numpy()
        return cv2.resize(prob, (bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_LINEAR)


class TrtRunner:
    """v1.0 candidate, through the same code the node uses."""

    def __init__(self, engine_path: Path, width: int, height: int) -> None:
        import sys

        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "rak_road_perception"))
        from rak_road_perception.roadseg_trt import TrtSegmenter

        self._segmenter = TrtSegmenter(str(engine_path), (width, height))
        self._width, self._height = width, height
        self.name = f"TwinLiteNet+ ({engine_path.name}, {width}x{height})"

    def infer(self, bgr: np.ndarray) -> np.ndarray:
        rgb = cv2.cvtColor(cv2.resize(bgr, (self._width, self._height)), cv2.COLOR_BGR2RGB)
        chw = np.ascontiguousarray((rgb.astype(np.float32) / 255.0).transpose(2, 0, 1)[None, ...])
        prob = self._segmenter.class_probability(chw)
        return cv2.resize(prob, (bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_LINEAR)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path("/tmp/roadseg_ab"))
    parser.add_argument("--engine", type=Path,
                        default=Path("data/models/roadseg/twinlite_plus_small_fp16_384x672.plan"))
    parser.add_argument("--engine-wh", type=int, nargs=2, default=[672, 384])
    parser.add_argument("--lfd-model", type=Path,
                        default=Path("data/models/roadseg/lfd_roadseg_v6.ts"))
    parser.add_argument("--lfd-wh", type=int, nargs=2, default=[640, 480])
    parser.add_argument("--limit", type=int, default=8)
    parser.add_argument("--threshold", type=float, default=0.40)
    args = parser.parse_args()

    images = sorted(
        p for p in args.images.rglob("*") if p.suffix.lower() in {".jpg", ".jpeg", ".png"}
    )[: args.limit]
    if not images:
        raise SystemExit(f"no images under {args.images}")
    args.out.mkdir(parents=True, exist_ok=True)

    runners = []
    if args.engine.is_file():
        runners.append(TrtRunner(args.engine, args.engine_wh[0], args.engine_wh[1]))
    else:
        print(f"skip TwinLiteNet+: engine missing at {args.engine}")

    lfd = None
    if args.lfd_model.is_file():
        try:
            lfd = LfdRunner(args.lfd_model, args.lfd_wh[0], args.lfd_wh[1])
            runners.append(lfd)
        except Exception as exc:  # torch absent is expected on a clean board
            print(f"skip LFD-RoadSeg: {exc}")
    else:
        print(f"skip LFD-RoadSeg: model missing at {args.lfd_model}")

    if not runners:
        raise SystemExit("nothing to compare")

    results = {runner.name: {"ms": [], "soft": []} for runner in runners}
    for index, path in enumerate(images):
        bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if bgr is None:
            continue
        tiles = []
        for runner in runners:
            started = time.perf_counter()
            probability = runner.infer(bgr)
            elapsed = (time.perf_counter() - started) * 1000.0
            results[runner.name]["ms"].append(elapsed)
            results[runner.name]["soft"].append(_soft_boundary_ratio(probability))
            overlay = _overlay(bgr, probability, args.threshold)
            cv2.putText(overlay, f"{runner.name.split(' (')[0]} {elapsed:.1f}ms",
                        (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            tiles.append(overlay)
        if index == 0:  # warm-up frame is timed but flagged separately by the median below
            pass
        if len(tiles) > 1:
            cv2.imwrite(str(args.out / f"{index:02d}_{path.stem}_ab.png"), np.hstack(tiles))
        else:
            cv2.imwrite(str(args.out / f"{index:02d}_{path.stem}.png"), tiles[0])

    print(f"\nimages={len(images)} out={args.out}")
    header = f"{'model':56s} {'mean ms':>8s} {'p50 ms':>8s} {'FPS':>6s} {'soft/bnd':>9s}"
    print(header)
    print("-" * len(header))
    for name, data in results.items():
        ms = np.array(data["ms"][1:] or data["ms"])
        soft = np.nanmean(np.array(data["soft"], dtype=np.float64))
        print(f"{name:56s} {ms.mean():8.2f} {np.median(ms):8.2f} "
              f"{1000.0/ms.mean():6.1f} {soft:9.2f}")


if __name__ == "__main__":
    main()
