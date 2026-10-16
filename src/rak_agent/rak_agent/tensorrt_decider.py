"""TensorRT System 1 decider: the pinned Laya checkpoint, in-process, without torch.

The engine is built offline (see `docs/laya_tensorrt.md`) and this module needs only the
TensorRT binding, the CUDA runtime (reached through `ctypes`, so no extra wheel is added)
and the checkpoint's tokenizer. `torch` is deliberately absent: the whole point of
exporting is to keep a 421M-parameter PyTorch stack out of the runtime.

Everything here is a faithful port of the upstream `laya` runtime for the one question
this robot asks:

  * `_render_options` / `_build_head` / `_build_sequence` mirror `laya.common`, so the
    token ids and `[MASK]` marker positions are the ones the checkpoint was trained on.
    They are checked against the upstream function in `docs/laya_tensorrt.md`'s
    acceptance step 1 -- the marker layout is not something to eyeball.
  * the decode (temperature lookup, softmax, top-1 / margin) mirrors
    `Agent._decode_answers`.

`decide()` is single-threaded on purpose: one `IExecutionContext` is reused, and the
engine holds one set of buffers. The node calls it from a timer callback, so that is
the natural shape; do not call it concurrently.

Nothing here trusts the model. A malformed or out-of-domain answer degrades to LOW and
gets escalated, exactly like the retired PyTorch adapter -- `guard()` remains the only
thing that can authorise a stop on safety grounds.
"""

from __future__ import annotations

import ctypes
import ctypes.util
import json
import os
import time
from pathlib import Path
from typing import Any

from .system1 import QUESTIONS
from .types import Action, Decision, SceneState

# `laya.common.QTYPES`. The model has one head per question type; this robot only asks
# `choice`, but the ids have to match the checkpoint's embedding table.
QTYPES = {"choice": 0, "score": 1, "noul": 2}
QTYPE_NAMES = {v: k for k, v in QTYPES.items()}

#: The deployment asset. It is gitignored and lives outside any install tree, so it is
#: found by search rather than by `__file__` -- an installed copy of this module sits
#: under `install/.../site-packages/` and would otherwise resolve to nothing.
#: The repo-root `data/` is the one gitignored asset tree since ADR-0013 (the episode
#: corpus lives beside it at `data/episodes/`).
_MODEL_SUBPATH = Path("data") / "models" / "laya-typed-decisions"
MODEL_DIR_ENV = "RAK_LAYA_MODEL_DIR"


def _default_model_dir() -> Path:
    override = os.environ.get(MODEL_DIR_ENV)
    if override:
        return Path(override)
    for start in (Path(__file__).resolve().parent, Path.cwd()):
        for base in (start, *start.parents):
            candidate = base / _MODEL_SUBPATH
            if candidate.is_dir():
                return candidate
    return Path.cwd() / _MODEL_SUBPATH


DEFAULT_MODEL_DIR = _default_model_dir()

# The checkpoint's `max_len` / `head_max_len`. Read from `rl_agent_config.json` at load
# time; these are only the fallback if the file is missing the keys.
_DEFAULT_MAX_LEN = 1024
_DEFAULT_HEAD_MAX_LEN = 256


def _render_criterion(value: Any) -> str:
    """`laya.common.render_criterion`: strings pass through, anything else becomes JSON."""
    if isinstance(value, str):
        return value
    return json.dumps(value, ensure_ascii=False, separators=(", ", ": "), default=str)


def _render_options(crit: dict) -> list[str]:
    """`laya.common.render_options` for a `choice` question."""
    return [
        str(k) if v is None or v == "" else "%s: %s" % (k, _render_criterion(v))
        for k, v in crit.items()
    ]


def _temp_bucket(qtype: int, k: int) -> str:
    """`laya.common.temp_bucket`."""
    size = "2" if k <= 2 else "3-5" if k <= 5 else "6-10" if k <= 10 else "11+"
    return "%s:%s" % (QTYPE_NAMES[int(qtype)], size)


def _softmax(z):
    import numpy as np

    z = z - z.max()
    p = np.exp(z)
    return p / p.sum()


def _load_cudart():
    """The CUDA runtime, without adding a Python CUDA wheel to the image."""
    for name in ("libcudart.so.13", "libcudart.so", "libcudart.so.12"):
        try:
            return ctypes.CDLL(name)
        except OSError:
            continue
    found = ctypes.util.find_library("cudart")
    if found:
        try:
            return ctypes.CDLL(found)
        except OSError:
            pass
    raise RuntimeError(
        "libcudart is not loadable; the TensorRT decider needs the CUDA runtime "
        "that ships with JetPack."
    )


class _Cudart:
    """The four cudart calls the runtime needs, behind one object with error checks.

    A raw status code is never returned to the caller: a failed `cudaMalloc` here means
    the engine cannot run at all, and silently proceeding would produce garbage logits.
    """

    _H2D = 1
    _D2H = 2

    def __init__(self) -> None:
        self.lib = _load_cudart()
        self.lib.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
        self.lib.cudaMalloc.restype = ctypes.c_int
        self.lib.cudaFree.argtypes = [ctypes.c_void_p]
        self.lib.cudaFree.restype = ctypes.c_int
        self.lib.cudaMemcpy.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_size_t,
            ctypes.c_int,
        ]
        self.lib.cudaMemcpy.restype = ctypes.c_int
        self.lib.cudaStreamCreate.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        self.lib.cudaStreamCreate.restype = ctypes.c_int
        self.lib.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]
        self.lib.cudaStreamSynchronize.restype = ctypes.c_int
        self.lib.cudaStreamDestroy.argtypes = [ctypes.c_void_p]
        self.lib.cudaStreamDestroy.restype = ctypes.c_int

    @staticmethod
    def _check(rc: int, what: str) -> None:
        if rc != 0:
            raise RuntimeError(f"{what} failed with cudaError {rc}")

    def malloc(self, nbytes: int) -> ctypes.c_void_p:
        ptr = ctypes.c_void_p()
        self._check(
            self.lib.cudaMalloc(ctypes.byref(ptr), ctypes.c_size_t(nbytes)),
            f"cudaMalloc({nbytes})",
        )
        return ptr

    def free(self, ptr: ctypes.c_void_p) -> None:
        self.lib.cudaFree(ptr)

    def host_to_device(self, dst: ctypes.c_void_p, array) -> None:
        import numpy as np

        contiguous = np.ascontiguousarray(array)
        self._check(
            self.lib.cudaMemcpy(
                dst,
                contiguous.ctypes.data_as(ctypes.c_void_p),
                ctypes.c_size_t(contiguous.nbytes),
                self._H2D,
            ),
            "cudaMemcpy(host->device)",
        )

    def device_to_host(self, src: ctypes.c_void_p, array) -> None:
        self._check(
            self.lib.cudaMemcpy(
                array.ctypes.data_as(ctypes.c_void_p),
                src,
                ctypes.c_size_t(array.nbytes),
                self._D2H,
            ),
            "cudaMemcpy(device->host)",
        )

    def stream(self) -> ctypes.c_void_p:
        stream = ctypes.c_void_p()
        self._check(self.lib.cudaStreamCreate(ctypes.byref(stream)), "cudaStreamCreate")
        return stream

    def synchronize(self, stream: ctypes.c_void_p) -> None:
        self._check(self.lib.cudaStreamSynchronize(stream), "cudaStreamSynchronize")

    def destroy_stream(self, stream: ctypes.c_void_p) -> None:
        self.lib.cudaStreamDestroy(stream)


class TensorRtDecider:
    """Loads the engine once, predicts per tick.

    `decide()` returns the same `Decision` shape the stub and the retired PyTorch adapter
    produce, so `resolve()` cannot tell which backend it is talking to.
    """

    #: The engine profile pins the marker count at 7. A question with a different number
    #: of options would need the engine rebuilt, so say so instead of reading past it.
    _MARKER_COUNT = 7

    def __init__(
        self,
        model_dir: str | Path | None = None,
        engine_path: str | Path | None = None,
        question: str = "action",
    ) -> None:
        self.model_dir = Path(model_dir or DEFAULT_MODEL_DIR)
        self.question_id = question
        self._engine_path = Path(engine_path) if engine_path else self.model_dir / "laya-typed-decisions-bf16.plan"

        if not self._engine_path.is_file():
            raise FileNotFoundError(
                f"TensorRT engine not found at {self._engine_path}. Build it with the "
                "runbook in docs/laya_tensorrt.md."
            )

        cfg_path = self.model_dir / "rl_agent_config.json"
        self.config = json.loads(cfg_path.read_text()) if cfg_path.is_file() else {}
        self.max_len = int(self.config.get("max_len", _DEFAULT_MAX_LEN))
        self.head_max_len = int(self.config.get("head_max_len", _DEFAULT_HEAD_MAX_LEN))
        self._temperature = list(self.config.get("temperature", [1.0]))
        self._temperature_by_options = dict(self.config.get("temperature_by_options", {}))

        self._load_tokenizer()
        self._build_question()
        self._load_engine()
        self._allocate()

        #: Provenance the node records in episode metadata, matching what it reads off
        #: whichever decider backend is active.
        self.device = "cuda"
        self.weights = "bf16"
        self.engine_bytes = self._engine_path.stat().st_size

        #: `act_probability` from the escalate head, the upstream response's `action` field.
        #: `Decision` has no slot for it, so it is exposed here for logging and the
        #: acceptance harness rather than smuggled into a field with another meaning.
        self.last_act_probability: float | None = None

    # -- setup -------------------------------------------------------------------

    def _load_tokenizer(self) -> None:
        from tokenizers import Tokenizer

        tok_dir = self.model_dir / "tokenizer"
        cfg = json.loads((tok_dir / "tokenizer_config.json").read_text())
        self._mask_token = str(cfg.get("mask_token", "[MASK]"))
        self._cls_token = str(cfg.get("cls_token", "[CLS]"))
        self._sep_token = str(cfg.get("sep_token", "[SEP]"))
        self._tokenizer = Tokenizer.from_file(str(tok_dir / "tokenizer.json"))
        for name, token in (
            ("cls", self._cls_token),
            ("sep", self._sep_token),
            ("mask", self._mask_token),
        ):
            if self._tokenizer.token_to_id(token) is None:
                raise ValueError(f"tokenizer has no {name} token {token!r}")
        self._cls_id = self._tokenizer.token_to_id(self._cls_token)
        self._sep_id = self._tokenizer.token_to_id(self._sep_token)
        self._mask_id = self._tokenizer.token_to_id(self._mask_token)

    def _build_question(self) -> None:
        """Normalise the public question into the internal `{t, ins, crit}` form.

        Mirrors `Agent._to_internal`. Keeping it derived from `system1.QUESTIONS` is the
        point: the labels `Action` accepts and the labels the model is asked about cannot
        drift apart.
        """
        qdef = QUESTIONS[self.question_id]
        crit = qdef.get("criteria")
        if qdef["type"] == "choice" and isinstance(crit, list):
            crit = {c: None for c in crit}
        self._qtype = QTYPES[qdef["type"]]
        self._question = {"t": qdef["type"], "ins": str(qdef["instructions"]), "crit": crit}
        self._option_keys = list(crit.keys())
        if len(self._option_keys) != self._MARKER_COUNT:
            raise ValueError(
                f"question {self.question_id!r} has {len(self._option_keys)} options but the "
                f"engine profile expects {self._MARKER_COUNT}; rebuild the engine."
            )

    def _load_engine(self) -> None:
        import tensorrt as trt

        self._trt = trt
        self._cudart = _Cudart()
        logger = trt.Logger(trt.Logger.ERROR)
        with open(self._engine_path, "rb") as handle:
            engine = trt.Runtime(logger).deserialize_cuda_engine(handle.read())
        if engine is None:
            raise RuntimeError(f"failed to deserialize TensorRT engine {self._engine_path}")
        self._engine = engine
        self._context = engine.create_execution_context()
        self._stream = self._cudart.stream()
        self._io_names = [engine.get_tensor_name(i) for i in range(engine.num_io_tensors)]

    def _allocate(self) -> None:
        """One set of device buffers, sized for the profile's maximum sequence."""
        import numpy as np

        self._np = np
        dtype_for = {
            self._trt.DataType.INT64: np.int64,
            self._trt.DataType.INT32: np.int32,
            self._trt.DataType.BOOL: np.bool_,
            self._trt.DataType.FLOAT: np.float32,
        }
        self._host: dict[str, Any] = {
            "input_ids": np.zeros((1, self.max_len), np.int64),
            "attention_mask": np.ones((1, self.max_len), np.int64),
            "marker_pos": np.zeros((1, self._MARKER_COUNT), np.int64),
            "marker_mask": np.ones((1, self._MARKER_COUNT), np.bool_),
            "qtype": np.zeros(1, np.int64),
        }
        self._device: dict[str, Any] = {}
        for name in self._io_names:
            np_dtype = dtype_for.get(self._engine.get_tensor_dtype(name), np.float32)
            shape = tuple(self._engine.get_tensor_shape(name))
            if -1 in shape:  # dynamic axis: the profile maximum is what we buffer for
                shape = tuple(self.max_len if d == -1 else d for d in shape)
            array = np.zeros(shape, np_dtype)
            self._device[name] = (self._cudart.malloc(array.nbytes), array)

    # -- tokenisation (ports of laya.common) -------------------------------------

    def _encode(self, text: str) -> list[int]:
        return self._tokenizer.encode(text, add_special_tokens=False).ids

    def _build_head(self) -> tuple[list[int], list[int]]:
        """`laya.common.build_head`: `[CLS] <type> instructions [SEP] [MASK] opt0 ... [SEP]`."""
        mask_tok = self._mask_token
        options = _render_options(self._question["crit"])
        ins = str(self._question["ins"]).replace(mask_tok, " ")
        head_ids = self._encode("%s question: %s" % (self._question["t"], ins))
        option_ids = []
        for text in options:
            # Upstream caps with `truncation=True, max_length=48`, which is this slice.
            option_ids.append([self._mask_id] + self._encode(" " + text.replace(mask_tok, " "))[:48])
        option_budget = self.head_max_len - sum(len(o) for o in option_ids)
        if option_budget < 16:
            per = max(4, (self.head_max_len - 16) // max(1, len(option_ids)))
            option_ids = [o[:per] for o in option_ids]
            option_budget = self.head_max_len - sum(len(o) for o in option_ids)
        head_ids = head_ids[: max(8, option_budget)]
        ids = [self._cls_id] + head_ids + [self._sep_id]
        markers = []
        for option in option_ids:
            markers.append(len(ids))
            ids.extend(option)
        ids.append(self._sep_id)
        return ids, markers

    def _build_sequence(self, state_text: str) -> tuple[list[int], list[int]]:
        """`laya.common.build_sequence` for a single question, single state."""
        ids, markers = self._build_head()
        room = max(0, self.max_len - len(ids) - 1)
        state_ids = self._encode(state_text.replace(self._mask_token, " "))[:room]
        ids = ids + state_ids + [self._sep_id]
        ids, markers = ids[: self.max_len], [m for m in markers if m < self.max_len]
        return ids, markers

    # -- inference ---------------------------------------------------------------

    def _run(self, ids: list[int], markers: list[int]) -> tuple[Any, Any]:
        np = self._np
        length = len(ids)
        self._host["input_ids"][0, :length] = ids
        self._host["attention_mask"][0, :length] = 1
        self._host["marker_pos"][0, :] = markers
        self._host["qtype"][0] = self._qtype

        context = self._context
        context.set_input_shape("input_ids", (1, length))
        context.set_input_shape("attention_mask", (1, length))
        for name in self._io_names:
            ptr, _ = self._device[name]
            context.set_tensor_address(name, ptr.value)
        for name in ("input_ids", "attention_mask", "marker_pos", "marker_mask", "qtype"):
            ptr, _ = self._device[name]
            self._cudart.host_to_device(ptr, self._host[name][:, : length] if self._host[name].ndim == 2 else self._host[name])

        if not context.execute_async_v3(self._stream.value):
            raise RuntimeError("TensorRT execute_async_v3 reported failure")
        self._cudart.synchronize(self._stream)

        outputs = {}
        for name in ("logits", "act_logits"):
            ptr, array = self._device[name]
            self._cudart.device_to_host(ptr, array)
            outputs[name] = array.copy()
        return outputs["logits"][0], outputs["act_logits"][0]

    def _temperature_for(self, k: int) -> float:
        bucket = _temp_bucket(self._qtype, k)
        fallback = self._temperature[self._qtype] if self._qtype < len(self._temperature) else 1.0
        return float(self._temperature_by_options.get(bucket, fallback))

    def decide(self, state: SceneState) -> Decision:
        """One decision. Any failure degrades to LOW with `inspect`, never raises.

        Laya is out-of-domain by construction (ECE 0.213, 0.362 zero-shot accuracy), so a
        malformed answer is a normal path: it must reach `resolve()` as a low-confidence
        `inspect` and get escalated, not take the tick down.
        """
        t0 = time.perf_counter()
        try:
            ids, markers = self._build_sequence(state.to_text())
            if len(markers) != self._MARKER_COUNT:
                raise ValueError(f"expected {self._MARKER_COUNT} markers, got {len(markers)}")
            logits, act_logits = self._run(ids, markers)
            latency = (time.perf_counter() - t0) * 1000.0

            row = self._np.asarray(logits, dtype=self._np.float32)[: len(self._option_keys)]
            probs = _softmax(row / self._temperature_for(len(self._option_keys)))
            ranked = sorted(zip(self._option_keys, (float(v) for v in probs)), key=lambda kv: -kv[1])
            kind, p = ranked[0]
            margin = p - ranked[1][1] if len(ranked) > 1 else p
            act = _softmax(self._np.asarray(act_logits, dtype=self._np.float32))
            self.last_act_probability = float(act[0])

            return Decision(
                action=Action.model_validate({"kind": kind, "reason": "laya"}),
                source="s1",
                p=p,
                margin=margin,
                latency_ms=latency,
                t=state.t,
            )
        except Exception:
            return Decision(
                action=Action(kind="inspect", reason="unparseable laya output"),
                source="s1",
                p=0.0,
                margin=0.0,
                latency_ms=(time.perf_counter() - t0) * 1000.0,
                t=state.t,
            )

    def close(self) -> None:
        for ptr, _ in self._device.values():
            self._cudart.free(ptr)
        self._device.clear()
        self._cudart.destroy_stream(self._stream)
