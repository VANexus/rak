# Laya TensorRT Export Runbook

> 状态：现行 · 最后更新：2026-10-17

Status (2026-10-14): fully integrated and verified end to end on the Jetson. The
full-model BF16 ONNX export and the TensorRT engine build both ran on 2026-10-07.
The artifacts live under the gitignored repo-root `data/models/laya-typed-decisions/`
(ADR-0013 moved them out of the old `brain/data/`, and `tensorrt_decider.py`'s
search subpath followed on 2026-10-14):
`laya-typed-decisions-bf16.onnx` + `.onnx.data` (~843 MB), and
`laya-typed-decisions-bf16.plan` (~807 MB, TensorRT 10.16.2.10). The numerical
acceptance below passed on 2026-10-07 and the tests re-verified it on 2026-10-14:
`src/rak_agent/test/test_tensorrt_decider.py` pins the token layout and runs the
engine end to end. `decider:=tensorrt` (ROS) / `--decider tensorrt` (CLI) load the
engine, report `s1_device=cuda`, and measure 50-80 ms per tick (first call ~130 ms).

Runtime dependencies are the JetPack TensorRT Python binding, system `numpy`, and
`tokenizers` (installed in the user pip site; there is no rosdep key for it — checked
against rosdistro's python.yaml/base.yaml on 2026-10-14 — which is why `package.xml`
does not declare it and `rosdep install` must not be expected to provide it).

The wrapper [`scripts/export_laya_onnx.py`](../../scripts/export_laya_onnx.py)
needed two fixes that the earlier "preparation" pass did not anticipate; both
are recorded under "Export gotchas" below.

## Pinned inputs

- Checkpoint: `convaiinnovations/laya-typed-decisions`, Hugging Face revision
  `e929ae5cf69bc34259cd2f95c9e91145b818b1f0` (2026-10-03).
- Exporter: [`NandhaKishorM/laya`](https://github.com/NandhaKishorM/laya), commit
  `c829d5ac2f0530f3c024c37cb812598ba47ceaa6`, checked out at
  `/tmp/laya-export-src`. The project wrapper retains that exporter's
  input/output contract and uses a non-unit dummy batch with explicit dynamic
  shape bounds.
- Checkpoint configuration: `max_len=1024`, `head_max_len=256`,
  `amp_dtype=bf16`; the model has 421M parameters and its `model.safetensors`
  file is about 843 MB.
- Expected input names: `input_ids`, `attention_mask`, `marker_pos`,
  `marker_mask`, `qtype`. Expected output names: `logits`, `act_logits`.
  Batch, sequence length, and marker count are exported as dynamic dimensions.
- The exporter writes only the ONNX file. The runtime also needs the checkpoint's
  `rl_agent_config.json` and `tokenizer/` files. Keep these assets under
  `data/models/` at the repo root (gitignored), not in source control.

The seven `ACTION_CRITERIA` labels in `src/rak_agent/rak_agent/system1.py` mean the
current agent call has seven markers. With the pinned tokenizer and current question,
empty-scene and one-person states produce 118 and 134 tokens respectively, with
  seven markers at positions 15 through 88. The model has different forward
  branches for one marker versus multiple markers, so this export declares
  marker counts from 2 to 7; the TensorRT profile below uses the current count
  of exactly 7. The sequence profile allows shorter inputs while covering the
  checkpoint's full `max_len`.

## Prepared Toolchain
The Jetson has the TensorRT 10.16.2.10 Python binding and `trtexec`. The
approved CPU-only PyTorch wheel is installed in the user's Python 3.12 site and
was verified as `2.12.0+cpu`; `torch.cuda.is_available()` is false and
`torch.version.cuda` is `None`. It is not a `rak_agent` runtime dependency.
Ubuntu has no `python3-torch` candidate on this image.

The one-time export dependencies are installed in the user's site with these
top-level versions: `transformers==4.57.6`, `safetensors==0.8.0`,
`huggingface_hub==0.36.2`, `onnx==1.19.0`, and `onnxscript==0.7.2`.
The system `numpy==1.26.4` was retained; `protobuf==4.25.8` is in the user site
to satisfy ONNX without moving to protobuf 7. `onnxruntime` was not installed
because the exporter does not need it. The relevant imports, including
`AutoConfig`, `AutoModel`, and `AutoTokenizer`, were verified; `ros2 pkg list`
also succeeds after the install.

The installation was constrained to avoid replacing system NumPy with NumPy 2
and to keep protobuf on major version 4:

```bash
python3 -m pip install --user --break-system-packages \
  --only-binary=:all: \
  'numpy>=1.26,<2' \
  'protobuf==4.25.8' \
  'transformers==4.57.6' \
  'safetensors==0.8.0' \
  'huggingface_hub==0.36.2' \
  'onnx==1.19.0' \
  'onnxscript==0.7.2'
```

The `rak_agent` direct ROS smoke test passed with the installed overlay and
`ROS_DOMAIN_ID=187`; the stub produced simulated decisions. Sourcing this
workspace's partial overlay still prints the known missing
`lslidar_driver/local_setup.bash` notice.

`python3 -m pip check` still reports that the system `PyNaCl` package requires
missing `cffi`; this is unrelated to the exporter dependency set and was not
changed here.

The official PyTorch CPU wheel index provides a CPU-only
`torch==2.12.0+cpu` wheel for CPython 3.12 / aarch64, SHA256
`ce2ddb880b0813fcc91a737f08fdd973a8115a74c64ccb34e9c09a7964b4d448`.
The upstream exporter explicitly constructs its `Agent` on CPU, so this wheel
avoids bringing another CUDA stack into the ROS system Python. It is a one-time
export tool, not a proposed `rak_agent` runtime dependency.

The completed installation used the CPU-only index explicitly. The regular
PyPI `torch==2.12.0` metadata
declares CUDA 13 dependencies on Linux and is not the intended build:

```bash
python3 -m pip install --user --break-system-packages \
  --only-binary=:all: \
  --index-url https://pypi.org/simple \
  --extra-index-url https://download.pytorch.org/whl/cpu \
  'torch==2.12.0+cpu'
```

NVIDIA's JetPack 7.2 forum moderator also says Orin can use the upstream
PyTorch `cu132` SBSA package, but that is the upstream CUDA wheel, not a
Jetson-specific wheel. It is not selected here because the project explicitly
avoids generic PyTorch CUDA wheels.

The upstream exporter constructs an `Agent` with default FP32 parameters, so
do not run it directly. The checkpoint config requests BF16 compute, while all
206 checkpoint tensors are stored as F16. The project wrapper
[`scripts/export_laya_onnx.py`](../../scripts/export_laya_onnx.py) builds the model
in BF16 and copies safetensors one tensor at a time. This avoids the FP32 model
allocation and avoids holding a second full copy of all checkpoint tensors.
Its streaming loader passed a small BF16 conversion test; the full model export
has since run end to end (see "Export gotchas").

### Export gotchas (found on the first real run, 2026-10-07)

Two failures surfaced only once the full export ran; both are fixed in the
wrapper and neither is an OOM:

- **`torch._inductor` fails to import on this torch build.** transformers'
  `modeling_modernbert` applies `@torch.compile(dynamic=True)` at class-definition
  time, which pulls in `torch._inductor`; torch 2.12.0+cpu's
  `_inductor/codegen/cutedsl/cutedsl_kernel.py` then dies with
  `TypeError: Too few arguments for <class 'CSE'>`. Set `TORCHDYNAMO_DISABLE=1`
  for the export: `_dynamo._optimize` returns a null decorator before it ever
  imports inductor, so `torch.compile` becomes a no-op.
- **transformers instantiates the encoder in `config.dtype`, not the caller's
  default dtype.** `encoder/config.json` records `"dtype": "float32"`, so
  `AutoModel.from_config` built the whole encoder in fp32 while the head (built
  by `DecisionModel`) honoured `torch.set_default_dtype(bf16)`. `copy_` then
  cast the F16 checkpoint weights *into* fp32, leaving a bf16-activation /
  fp32-weight graph that failed in ONNX decomposition. The wrapper now wraps
  `AutoConfig.from_pretrained` to force `config.dtype = bf16` (the pinned asset
  is left untouched) and asserts no parameter strays from the compute dtype.
  This also halves encoder construction memory (~1.57 GiB → ~0.79 GiB).
- **`act_head` must stay fp32.** `DecisionModel.forward` promotes the decision
  head's input to fp32 (`.float()` on the scorer output, `pooled`, and the
  entropy/top-2 features). A bf16 `act_head` is rejected by TensorRT:
  `IMatrixMultiplyLayer must have same input types. A is of type Float and B is
  of type BFloat16`. The wrapper casts `act_head` to fp32 before the checkpoint
  copy, so the F16 weights land in fp32 directly.

Recent readings were about 1.3 GiB `MemAvailable` and 49 MiB `CmaFree`, which
is not enough headroom. Close Trae, the desktop session, and robot workloads
before starting the full export. Use 2 GiB `MemAvailable` as a minimum go/no-go
preflight, not a guarantee that export or engine build will fit; the script also
refuses to load the model below that threshold. Recheck the same threshold
before TensorRT build. The target runtime remains
the system TensorRT binding inside the existing `rak_agent` process. Do not
install a generic PyTorch CUDA wheel or add PyTorch to `rak_agent` runtime
dependencies.

## Prepared Assets

The pinned model is already downloaded to the ignored repo-root `data/` directory (ADR-0013).
The safetensors header was inspected without loading its weights: 206 tensors,
all F16. `git check-ignore` confirms the weights will not enter Git. Re-fetch
only if this directory is later removed:

```bash
python3 -c 'from huggingface_hub import snapshot_download; snapshot_download(
    "convaiinnovations/laya-typed-decisions",
    revision="e929ae5cf69bc34259cd2f95c9e91145b818b1f0",
    local_dir="data/models/laya-typed-decisions",
    allow_patterns=["model.safetensors", "rl_agent_config.json",
                    "tokenizer/*", "encoder/*"])'
```

The exporter source is already checked out at the pinned commit under
`/tmp/laya-export-src`. After closing the resource-heavy workloads, open a
terminal at the workspace root and run the following export command:

```bash
set -eu
AVAILABLE_KIB=$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo)
if [ "$AVAILABLE_KIB" -lt 2097152 ]; then
  echo "Need at least 2 GiB MemAvailable; found $((AVAILABLE_KIB / 1024)) MiB." >&2
  exit 1
fi
MODEL="$PWD/data/models/laya-typed-decisions"
PYTHONPATH=/tmp/laya-export-src TORCHDYNAMO_DISABLE=1 \
  python3 scripts/export_laya_onnx.py \
  --model "$MODEL" \
  --output "$MODEL/laya-typed-decisions-bf16.onnx"
```

`TORCHDYNAMO_DISABLE=1` is required (see "Export gotchas"). This command ran
successfully on 2026-10-07 and wrote a BF16 ONNX graph plus its external-data
sidecar. Export failure or memory pressure is a stop condition; do not fall back
to the upstream FP32 exporter.

## TensorRT build candidate

The local TensorRT 10.16.2.10 `trtexec` accepts BF16 and strongly typed
networks. A tiny synthetic BF16 graph with dynamic shapes exported via
`dynamic_shapes` compiled successfully with `--stronglyTyped`. In the five-input
smoke graph, `input_ids` and `attention_mask` were not actually consumed;
TensorRT reported `Unused Input` and int64 binding warnings. This only exercises
part of the profile syntax and BF16 path. It does not validate the Laya graph,
all input bindings or dtypes, or the full-model build.

After the export command succeeds, run this Jetson-only engine build:

```bash
set -eu
AVAILABLE_KIB=$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo)
if [ "$AVAILABLE_KIB" -lt 2097152 ]; then
  echo "Need at least 2 GiB MemAvailable; found $((AVAILABLE_KIB / 1024)) MiB." >&2
  exit 1
fi
MODEL="$PWD/data/models/laya-typed-decisions"
trtexec \
  --onnx="$MODEL/laya-typed-decisions-bf16.onnx" \
  --saveEngine="$MODEL/laya-typed-decisions-bf16.plan" \
  --minShapes=input_ids:1x96,attention_mask:1x96,marker_pos:1x7,marker_mask:1x7,qtype:1 \
  --optShapes=input_ids:1x160,attention_mask:1x160,marker_pos:1x7,marker_mask:1x7,qtype:1 \
  --maxShapes=input_ids:1x1024,attention_mask:1x1024,marker_pos:1x7,marker_mask:1x7,qtype:1 \
  --stronglyTyped \
  --builderOptimizationLevel=1 \
  --maxAuxStreams=0 \
  --memPoolSize=workspace:256 \
  --skipInference
```

The profile covers the tested current inputs; expand it only if actual
tokenization needs it. This command ran successfully on 2026-10-07: the engine
built in 63.6 s and `trtexec` reported `PASSED` (parse 0.33 s, workspace pool
256 MiB). The saved plan deserializes under the system TensorRT Python binding
with 7 I/O tensors — `input_ids`/`attention_mask` `INT64` with a dynamic second
dim, `marker_pos`/`marker_mask` `INT64`/`BOOL` at `(1,7)`, `qtype` `INT64` at
`(1,)`, and `logits` `(1,7)` / `act_logits` `(1,2)` `FLOAT` — and an execution
context accepts the opt shape `1x160`. Note the profile pins the marker count
at 7, so `marker_pos`/`marker_mask` are not dynamic in the plan; the ONNX graph
still declares `num_markers` from 2 to 7 if that ever needs widening.

If TensorRT rejects an operation or shape, stop and retain the error output
rather than silently converting to FP16/INT8. Build plans on the target Jetson;
serialized plans are hardware/software-version specific. Do not enable INT8 as a
memory shortcut: the upstream exporter documents substantial decision drift for
its INT8 path, and its ONNX INT8 path is CPU-only.

## Acceptance before runtime integration

Run on 2026-10-07 against the engine above. The reference is the pinned PyTorch
checkpoint built in the same bf16 compute dtype, on CPU, with the upstream
`laya.common.build_sequence` producing the token layout — comparing TensorRT
directly against PyTorch covers steps 1 and 2 at once and is the stronger check
(ONNX Runtime is not installed on this image).

1. **Token layout — exact.** The port in `src/rak_agent/rak_agent/tensorrt_decider.py`
   produces ids and `[MASK]` markers identical to upstream `build_sequence` on
   four states. The empty scene is 118 tokens and the seven markers land at
   `[15, 28, 42, 51, 63, 75, 88]`, matching the layout recorded above.
2. **Numerics — within bf16.** Over the four states, max |Δlogits| = 0.039 and
   max |Δprobability| = 0.0048; the argmax is identical on every state. The act
   head's logits differ by ≤0.14 % (they are ~±4000 and softmax to 1.0 either
   way). This is the expected spread for a bf16 engine against a bf16 reference.
3. **Contract — unchanged.** `TensorRtDecider.decide()` returns the same
   `Decision` shape as the stub and the retired adapter, so `resolve()` cannot
   tell the backends apart. A state that cannot be tokenized into the profile
   degrades to `inspect`/LOW instead of raising, and `guard()` still vetoed a
   person at 0.4 m in the end-to-end ROS run. Note the model's own answers are
   **always** LOW confidence here (p ≈ 0.22, margin ≈ 0.015) — it is
   out-of-domain and uncalibrated, so `resolve()` correctly falls back every
   tick. That is the documented behaviour, not a wiring fault.
4. **Measured.** Inference latency 45–80 ms per tick (engine execution alone
   ~58 ms at 118 tokens, first call); engine file 807 MB, plus the CUDA context.
   RSS/CMA under the full stack is measured by the stress run, not here.
5. **Held.** The plan, ONNX and weights stay under repo-root `data/`; nothing is
   committed. `src/rak_agent/test/test_tensorrt_decider.py` skips rather than fails when
   the assets are absent, so a clean checkout still runs.

Wired into the node as `decider:=tensorrt` (`-p decider_model_dir:=` overrides
the asset search; `RAK_LAYA_MODEL_DIR` does the same for non-ROS callers). The
`decider:=laya` PyTorch path stays rejected.
