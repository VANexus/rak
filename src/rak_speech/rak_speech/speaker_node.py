"""Real-time speaker identification by online clustering -> `/speech/speaker`.

No enrolment, no profile files (工程约定): the node discovers voices as they
appear. Pipeline (docs/microphone_array.md「声纹识别」):

    /audio (16 kHz mono S16) -> silero VAD gates speech
        -> every `eval_every_sec` while speech is detected, embed the trailing
           `window_sec` with 3D-Speaker CAM++ (sherpa-onnx, CPU, one thread)
        -> SpeakerClusterer: confident match votes for that cluster, a new voice
           opens a new label
        -> JSON on `/speech/speaker`: {"speaker": "S1", "score": 0.83,
                                      "new_cluster": false}

Labels are session-scoped (S1, S2, ...) and reset when the node restarts — there
is deliberately nothing on disk to keep in sync. Folding a weak match into the
closest cluster once `max_speakers` is reached keeps labels bounded.

Design constraints: one CPU extractor (~28 MB), ~1.2 Hz evaluation on ≤1.5 s
windows; identity shows after the first ~1 s of speech and keeps refreshing;
speech pipeline degrades gracefully when sherpa/model files are missing.
"""

from __future__ import annotations

import json

from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from audio_common_msgs.msg import AudioData
from rclpy.node import Node
from std_msgs.msg import String

from .speaker_core import DEFAULT_MAX_SPEAKERS, DEFAULT_THRESHOLD, SpeakerClusterer
from .speaker_models import SAMPLE_RATE, build_extractor, build_vad, embed, find_models

#: Int16 -> float32 divisor (the /audio contract is S16_LE mono).
_INT16_SCALE = 1.0 / 32768.0
#: Extra audio kept on top of the embedding window so window trimming never
#: races the evaluation timer under full-rate capture.
_BUFFER_MARGIN_SEC = 0.5


class SpeakerNode(Node):
    def __init__(self) -> None:
        super().__init__("speaker_id")
        self.declare_parameter("audio_topic", "/audio")
        self.declare_parameter("topic", "/speech/speaker")
        self.declare_parameter("model", "")  # empty: auto-search campplus*.onnx
        self.declare_parameter("vad_model", "")  # empty: auto-search silero_vad.onnx
        self.declare_parameter("threshold", DEFAULT_THRESHOLD)
        self.declare_parameter("max_speakers", DEFAULT_MAX_SPEAKERS)
        self.declare_parameter("eval_every_sec", 0.8)
        self.declare_parameter("window_sec", 1.5)
        self.declare_parameter("min_speech_sec", 1.0)

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self._window_samples = int(float(g("window_sec")) * SAMPLE_RATE)
        self._min_samples = int(float(g("min_speech_sec")) * SAMPLE_RATE)
        self._buf_cap = self._window_samples + int(_BUFFER_MARGIN_SEC * SAMPLE_RATE)

        self._clusterer = SpeakerClusterer(
            threshold=float(g("threshold")),
            max_speakers=int(g("max_speakers")),
        )
        self._ring: deque[np.ndarray] = deque()
        self._ring_len = 0
        self._last_identity = ("", 0.0)
        self._extractor = None
        self._vad = None
        try:
            model, vad_model = find_models()
            if str(g("model")):
                model = Path(str(g("model")))
            if str(g("vad_model")):
                vad_model = Path(str(g("vad_model")))
            if model is None or vad_model is None:
                raise FileNotFoundError(
                    "speaker model / silero vad not found; place them under "
                    "acoustic/models/ (see docs/microphone_array.md「声纹识别」)"
                )
            self._extractor = build_extractor(model)
            self._vad = build_vad(vad_model)
            self.get_logger().info(
                f"speaker id up (online clustering): model={model.name} "
                f"vad={vad_model.name} threshold={self._clusterer.threshold} "
                f"max_speakers={self._clusterer.max_speakers}"
            )
        except Exception as exc:  # 降级:采集/识别链路不能被我拖死
            self.get_logger().warning(f"speaker id disabled: {exc}")

        self.pub = self.create_publisher(String, str(g("topic")), 10)
        self.create_subscription(
            AudioData, str(g("audio_topic")), self._on_audio, 10
        )
        self.create_timer(float(g("eval_every_sec")), self._evaluate)

    # -- audio in -----------------------------------------------------------
    def _on_audio(self, msg: AudioData) -> None:
        if self._vad is None:
            return
        samples = np.frombuffer(bytes(msg.data), dtype=np.int16).astype(np.float32)
        samples *= _INT16_SCALE
        self._vad.accept_waveform(samples)
        self._ring.append(samples)
        self._ring_len += len(samples)
        while self._ring_len > self._buf_cap and len(self._ring) > 1:
            self._ring_len -= len(self._ring.popleft())

    # -- periodic identity --------------------------------------------------
    def _evaluate(self) -> None:
        if self._vad is None:
            return
        # 释放 VAD 内部整段缓存:不排空会随对话无限增长。
        while not self._vad.empty():
            self._vad.pop()

        if not self._vad.is_speech_detected():
            return  # nobody talking: let the last identity go stale upstream
        if self._ring_len < self._min_samples:
            return

        window = self._window()
        try:
            embedding = embed(self._extractor, window)
        except Exception as exc:  # 推理异常不致命,下一拍继续
            self.get_logger().warning(f"embedding failed: {exc}")
            return
        label, score, is_new = self._clusterer.assign(
            np.asarray(embedding, dtype=np.float32)
        )

        self.pub.publish(
            String(
                data=json.dumps(
                    {"speaker": label, "score": round(score, 3), "new_cluster": is_new}
                )
            )
        )
        if (label, is_new) != self._last_identity:
            self._last_identity = (label, is_new)
            what = "new voice" if is_new else "matched"
            self.get_logger().info(
                f"speaker: {label} ({what}, score {score:.3f}, "
                f"clusters {len(self._clusterer)})"
            )

    def _window(self) -> np.ndarray:
        parts: list[np.ndarray] = []
        need = self._window_samples
        for chunk in reversed(self._ring):
            parts.append(chunk)
            need -= len(chunk)
            if need <= 0:
                break
        return np.concatenate(list(reversed(parts)))[-self._window_samples:]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
