"""CosyVoice3 TTS backend (emotional Chinese, local GPU).

Runs the vendored upstream copy in tools/cosyvoice (gitignored local tooling,
ADR discipline: third-party stays out of src/). Two Jetson adaptations:

  * a torchaudio SHIM (tools/cosyvoice/torchaudio) implements load/save/
    Resample/MelSpectrogram; kaldi.fbank is the upstream pure-python source.
  * tools/cosyvoice goes FIRST on sys.path so the shim wins; the system
    interpreter (and pydantic v1) is otherwise untouched.

Emotion/style control is the CosyVoice "instruct" string (natural language),
mapped from the persona palette (rak_harness.persona.DEFAULT_EMOTIONS). The
Say.srv `instruct` field carries it; empty falls back to the neutral default.

Failure contract: any import/init error raises CosyvoiceUnavailable and the
node degrades to the sherpa backend; a per-request error is reported through
the service response, never raised into the executor callback.
"""

from __future__ import annotations

import logging
import os
import sys
import threading
from pathlib import Path

# 仓库根：.../src/rak_speech/rak_speech/cosyvoice_backend.py → parents[3]
_REPO = Path(__file__).resolve().parents[3]


def _repo_path(env: str, relative: str) -> Path:
    """环境变量 > 仓库相对路径。禁止在源码里写死家目录（AGENTS §2 第 3 条）。

    相对路径是**相对仓库**而不是相对当前工作目录：换用户、换盘符、换机器都不变。
    """
    override = os.environ.get(env, "")
    return Path(override).expanduser() if override else _REPO / relative


COSYVOICE_ROOT = _repo_path("RAK_COSYVOICE_ROOT", "tools/cosyvoice")
MODEL_DIR = _repo_path(
    "RAK_COSYVOICE_MODEL_DIR", "acoustic/models/Fun-CosyVoice3-0.5B-2512")
# Robot voice identity: a short neutral Chinese reference clip. Generate once
# with the kokoro backend if missing (see scripts/make_cosyvoice_prompt.py
# or the ROS-side bootstrap); identity carries across emotions.
DEFAULT_PROMPT_WAV = _repo_path(
    "RAK_COSYVOICE_PROMPT_WAV", "acoustic/models/cosyvoice_prompt_zh.wav")
DEFAULT_INSTRUCT = "用平时稳重、清晰的语气说"


class CosyvoiceUnavailable(RuntimeError):
    pass


class CosyvoiceSynthesizer:
    """One resident model, serialized synthesis (single writer, one GPU)."""

    def __init__(self, *, model_dir: str = "", prompt_wav: str = "",
                 instruct: str = DEFAULT_INSTRUCT, fp16: bool = True,
                 load_trt: bool | None = None) -> None:
        self._model_dir = Path(model_dir) if model_dir else MODEL_DIR
        self._prompt_wav = Path(prompt_wav) if prompt_wav else DEFAULT_PROMPT_WAV
        self._instruct = instruct or DEFAULT_INSTRUCT
        self._lock = threading.Lock()
        self._model = None
        # TRT acceleration of the flow-estimator (DiT): enabled when the plan
        # exists (RAK_COSYVOICE_TRT=0 to force off). A plan is version-locked
        # to the installed TRT + CC 8.7; a stale plan fails at deserialize and
        # the code below falls back to the PyTorch estimator instead of dying.
        self.load_trt = (os.environ.get("RAK_COSYVOICE_TRT", "1") == "1"
                         if load_trt is None else load_trt)

        import torch  # noqa: F401 -- present or the cuda path is meaningless

        if not self._model_dir.is_dir():
            raise CosyvoiceUnavailable(f"model dir missing: {self._model_dir}")
        if str(COSYVOICE_ROOT) not in sys.path:
            sys.path.insert(0, str(COSYVOICE_ROOT))  # shim + upstream code first

        try:
            from cosyvoice.cli.cosyvoice import AutoModel
        except Exception as exc:
            raise CosyvoiceUnavailable(f"cosyvoice import failed: {exc}") from exc
        # AutoModel forwards **kwargs to the versioned class; CosyVoice3 does
        # not take load_jit, so pass only what every version accepts.
        plan = self._model_dir / "flow.decoder.estimator.fp16.mygpu.plan" if fp16 else \
            self._model_dir / "flow.decoder.estimator.fp32.mygpu.plan"
        want_trt = self.load_trt and plan.is_file()
        try:
            try:
                self._model = AutoModel(
                    model_dir=str(self._model_dir), load_trt=want_trt, fp16=fp16,
                )
            except Exception as exc:
                if not want_trt:
                    raise
                logging.warning("TRT plan unusable (%s); falling back to PyTorch estimator", exc)
                self._model = AutoModel(
                    model_dir=str(self._model_dir), load_trt=False, fp16=fp16,
                )
        except Exception as exc:
            raise CosyvoiceUnavailable(f"model load failed: {exc}") from exc

    @property
    def sample_rate(self) -> int:
        return int(getattr(self._model, "sample_rate", 24000))

    def synthesize(self, text: str, instruct: str = ""):
        """Returns (sample_rate, float32 ndarray mono)."""
        text = text.strip()
        if not text:
            raise ValueError("empty text")
        if not self._prompt_wav.is_file():
            raise CosyvoiceUnavailable(f"prompt wav missing: {self._prompt_wav}")
        import numpy as np

        # CosyVoice3 convention: the instruct string must end with the special
        # token (id 151646 in the Qwen vocab); the LLM asserts on it.
        instruct_text = (instruct or self._instruct).strip()
        if not instruct_text.endswith("<|endofprompt|>"):
            instruct_text += "<|endofprompt|>"
        with self._lock:
            chunks = []
            for chunk in self._model.inference_instruct2(
                text, instruct_text, str(self._prompt_wav), stream=False,
            ):
                chunks.append(chunk["tts_speech"].detach().cpu().numpy().reshape(-1))
        if not chunks:
            raise CosyvoiceUnavailable("no audio produced")
        return self.sample_rate, np.concatenate(chunks)
