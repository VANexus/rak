"""Historical parity test for the retired PyTorch Laya backend.

Do not run this test on the Jetson or install PyTorch to satisfy it. Replace it
with TensorRT parity coverage once the exported engine and tokenizer are integrated.

`laya.load()` builds fp32 parameters and copies an F16 checkpoint into them, so
the model wants 1606 MiB of this 3.4 GB box's unified memory when the file on
disk says 803 MiB. That is why System 1 kept sliding off the GPU: the fp32 peak
starved CUDA, laya caught the OOM, and said nothing. `LayaDecider` now loads on
CPU, casts to the checkpoint's own amp dtype (bf16 here), and only then places --
so the GPU peak is the halved model.

Casting the stored weights is not free in the numbers: autocast covers the big
ops but not LayerNorm, embeddings or the softmax, and those now read half
precision parameters. The question this test answers is whether that drift can
move a decision. It compares fp32 against the cast on the *same* states and
judges only what this repo consumes: the action kind that reaches the body and
the tier `gate()` reports.

Both arms must run on the same device or the comparison measures the device, not
the precision -- and this box cannot hold two models, so the parity pass runs in
a child interpreter with CUDA hidden. The GPU pass then proves the cast actually
lands on the GPU and reports what it cost.

This file preserves the previous experiment's method and results for reference.
"""

from __future__ import annotations

import os
import statistics
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rak_agent.body import MockBody  # noqa: E402
from rak_agent.system1 import LayaDecider, gate, resolve  # noqa: E402

# `approach` only reaches the stop zone around tick 25 (z = 3.0 - 0.1*t), so 12
# ticks would test the easy states and call it a day. 30 x 3 scenarios covers the
# geometry that actually vetoes, at the cost of a few minutes on six ARM cores.
TICKS = 30
SCENARIOS = ("approach", "cross", "clear")

# The thresholds are 0.70 / 0.40 with a 0.30 margin, so drift under this cannot
# reach a boundary on its own. Tier equality below is asserted anyway rather than
# inferred from it.
MAX_P_DELTA = 0.02
CPU_PARITY = "--cpu-parity"


def states():
    out = []
    for name in SCENARIOS:
        body = MockBody(name)
        out.extend(body.step() for _ in range(TICKS))
    return out


def avail_mib() -> int:
    for line in open("/proc/meminfo"):
        if line.startswith("MemAvailable"):
            return int(line.split()[1]) // 1024
    return -1


def collect(states, weights: str) -> dict:
    """Load one precision, score every state with it, then hand the memory back.

    `rows` carries what `resolve()` would execute (kind + which component decided
    it), because at LOW tier the model's own label is discarded and a fallback
    runs -- comparing the discarded label would fail on noise that cannot move the
    robot, and pass over nothing. The proposal is kept alongside for reporting.
    """
    before = avail_mib()
    d = LayaDecider(weights=weights)
    rows, lat = [], []
    for s in states:
        dec = d.decide(s)
        final, tier, _veto = resolve(s, dec)
        rows.append((final.action.kind, final.source, dec.action.kind, dec.p, dec.margin, gate(dec)))
        lat.append(dec.latency_ms)
    out = {
        "device": d.device,
        "weights": d.weights,
        "weights_mib": round(d.weights_bytes / 1048576),
        "lat_ms": round(statistics.median(lat), 1),
        "rows": rows,
        "place_error": getattr(d, "place_error", ""),
        "avail_after": avail_mib(),
        "avail_during": before,
    }
    del d
    return out


def parity() -> int:
    """fp32 vs the cast, both on CPU with laya's CPU autocast on.

    The arms have to sit in the same environment the GPU runs in, or the
    comparison measures autocast instead of precision: on CUDA laya autocasts at
    the checkpoint's amp dtype, and its documented equivalent off-GPU is
    LAYA_CPU_AMP. Without it the bf16 arm dies in `F.linear` -- fp32 activations
    against bf16 weights -- which is a real property of half storage, not a
    quirk to hide here.
    """
    import torch

    if torch.cuda.is_available():
        print("FAIL: CUDA is visible to the parity pass; it must run CPU-only")
        return 1

    st = states()
    print(f"[parity] {len(st)} states, CPU, both arms, one model at a time")
    fp32 = collect(st, "fp32")
    cast = collect(st, "auto")
    if cast["device"] != "cpu" or fp32["device"] != "cpu":
        print(f"FAIL: devices differ ({fp32['device']} vs {cast['device']})")
        return 1
    print(f"[parity] arms: fp32 and {cast['weights']}, median {fp32['lat_ms']} ms vs {cast['lat_ms']} ms")

    kind_flips, tier_flips, src_flips, proposal_flips, deltas = [], [], [], [], []
    for r32, rct in zip(fp32["rows"], cast["rows"]):
        k32, s32, prop32, p32, _m32, t32 = r32
        kct, sct, propct, pct, _mct, tct = rct
        if k32 != kct:
            kind_flips.append((k32, kct, p32, pct))
        if s32 != sct:
            src_flips.append((s32, sct, k32, kct))
        if t32 != tct:
            tier_flips.append((prop32, t32.name, tct.name, p32, pct))
        if prop32 != propct:
            proposal_flips.append((prop32, propct, p32, pct, t32.name))
        deltas.append(abs(p32 - pct))

    worst = sorted(zip(deltas, fp32["rows"], cast["rows"]), key=lambda x: x[0], reverse=True)[:5]
    print(f"[parity] max |dp| = {max(deltas):.5f} (tolerance {MAX_P_DELTA})")
    for dl, r32, rct in worst:
        print(
            f"  dp={dl:.5f}  fp32 exec={r32[0]:<9}({r32[1]:<6}) p={r32[3]:.4f} {r32[5].name:<6}"
            f" | {cast['weights']} exec={rct[0]:<9}({rct[1]:<6}) p={rct[3]:.4f} {rct[5].name}"
        )
    print(
        f"[parity] executed-action flips: {len(kind_flips)}   source flips: {len(src_flips)}   "
        f"tier flips: {len(tier_flips)}   (discarded) proposal flips: {len(proposal_flips)}"
    )
    for kf in kind_flips:
        print(f"  EXEC {kf[0]} -> {kf[1]}  (p {kf[2]:.4f} -> {kf[3]:.4f})")
    for sf in src_flips:
        print(f"  SOURCE {sf[0]} -> {sf[1]}  ({sf[2]} -> {sf[3]})")
    for tf in tier_flips:
        print(f"  TIER {tf[1]} -> {tf[2]} on proposal {tf[0]}  (p {tf[3]:.4f} -> {tf[4]:.4f})")
    for pf in proposal_flips:
        # Immaterial by construction: these are the labels resolve() throws away.
        print(f"  proposal-only {pf[0]} -> {pf[1]} at {pf[4]} (p {pf[2]:.4f} -> {pf[3]:.4f})")

    assert max(deltas) <= MAX_P_DELTA, f"p drifted {max(deltas):.4f} > {MAX_P_DELTA}"
    assert not kind_flips, f"{len(kind_flips)} executed actions changed under the cast"
    assert not src_flips, f"{len(src_flips)} deciding components changed under the cast"
    assert not tier_flips, f"{len(tier_flips)} escalation tiers changed under the cast"
    print("[parity] OK: nothing that reaches the body moved.")
    return 0


def gpu_leg() -> int:
    """Prove the cast lands on the GPU, and what it leaves the box holding."""
    import torch

    if not torch.cuda.is_available():
        print("SKIP: no CUDA -- the GPU pass needs the Jetson")
        return 0

    st = states()[:6]
    avail_before = avail_mib()
    d = LayaDecider(weights="auto")
    mib = round(d.weights_bytes / 1048576)
    lat = [d.decide(s).latency_ms for s in st]
    print(
        f"[gpu] device={d.device} weights={d.weights} resident={mib} MiB "
        f"median={statistics.median(lat):.0f} ms"
    )
    if d.place_error:
        print(f"[gpu] placement failed: {d.place_error[:200]}")
    print(f"[gpu] MemAvailable {avail_before} -> {avail_mib()} MiB with the model resident")
    assert d.device == "cuda", f"expected the cast model on CUDA, got {d.device}"
    assert d.weights in ("bf16", "fp16"), f"expected a half-precision cast, got {d.weights}"
    assert 700 <= mib <= 900, f"half of 421M params should be ~803 MiB, got {mib}"
    assert statistics.median(lat) < 1000, f"{statistics.median(lat):.0f} ms/predict looks like CPU, not GPU"
    del d
    torch.cuda.empty_cache()
    print("[gpu] OK: half-precision weights resident on the GPU.")
    return 0


def main() -> int:
    if CPU_PARITY in sys.argv:
        return parity()
    env = dict(os.environ, CUDA_VISIBLE_DEVICES="", LAYA_CPU_AMP="bf16")
    rc = subprocess.run([sys.executable, __file__, CPU_PARITY], env=env).returncode
    if rc:
        print(f"FAIL: parity pass exited {rc}")
        return rc
    return gpu_leg()


if __name__ == "__main__":
    sys.exit(main())
