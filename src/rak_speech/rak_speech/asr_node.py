"""Offline speech recognition node.

Turns audio into text and, optionally, into the deterministic task intent that
`rak_task_executor` already consumes. It deliberately stops there: the executor
is the only owner of the navigation-task lifecycle (see CLAUDE.md), so this node
publishes a transcript and an intent, and never sends a goal of its own.

Backend is Vosk -- offline, no network at runtime, and an aarch64 wheel, which
matters on a Jetson that must not grow a PyTorch dependency for speech.

Audio in, three interchangeable sources that all yield 16 kHz mono S16 PCM:

- `topic` (standard): `audio_common_msgs/AudioData` on `audio_topic` (/audio),
  published by `audio_capture` reading the microphone array over GStreamer/ALSA.
- `arecord`: a capture device opened directly (fallback, no audio_common needed).
- `wav`: a file, for fixtures and regression tests.

    ros2 run rak_speech asr_node --ros-args -p source:=topic
    ros2 run rak_speech asr_node --ros-args -p source:=wav -p wav_path:=/tmp/x.wav
"""

from __future__ import annotations

import json
import threading
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rak_intent.task_intent import parse_control_command, parse_navigation_intent

from .audio import BYTES_PER_SECOND, DEFAULT_INBOX_SECONDS, SAMPLE_RATE, AudioInbox, build_source


class SpeechAsr(Node):
    def __init__(self) -> None:
        super().__init__("speech_asr")

        self.declare_parameter("source", "arecord")  # topic | arecord | wav
        self.declare_parameter("wav_path", "")
        self.declare_parameter("device", "default")
        self.declare_parameter("audio_topic", "/audio")  # source:=topic input
        # How far the hand-off may run ahead of the recogniser. vosk-small-cn
        # decodes at 0.84-1.01x real time on this board, so the stream must be
        # allowed to lead; past this budget the oldest audio is evicted.
        self.declare_parameter("audio_queue_seconds", DEFAULT_INBOX_SECONDS)
        self.declare_parameter("model_dir", "")
        self.declare_parameter("transcript_topic", "/speech/transcript")
        self.declare_parameter("intent_topic", "/speech/intent")
        self.declare_parameter("parse_intent", True)
        self.declare_parameter("once", False)  # exit when a file source ends
        self.declare_parameter(
            "engine", "zipformer"
        )  # zipformer (sherpa-onnx streaming 14M, fast) | vosk (legacy CPU)

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self._source_kind = str(g("source"))
        self._wav_path = str(g("wav_path"))
        self._device = str(g("device"))
        self._engine = str(g("engine"))
        self._model_dir = str(g("model_dir")) or self._find_model()
        self._parse_intent = bool(g("parse_intent"))
        self._once = bool(g("once"))

        self._transcript_pub = self.create_publisher(String, str(g("transcript_topic")), 10)
        self._intent_pub = self.create_publisher(String, str(g("intent_topic")), 10)

        self._inbox = AudioInbox(
            max_bytes=int(float(g("audio_queue_seconds")) * BYTES_PER_SECOND)
        )
        self._dropped_audio = 0
        self._audio_sub = None
        if self._source_kind == "topic":
            self._audio_sub = self._subscribe_audio(str(g("audio_topic")))

        self._stop = threading.Event()
        # Recognition blocks on the audio source, so it cannot run on the executor that
        # also serves subscriptions; a daemon thread keeps the node responsive and lets
        # the process exit without a join.
        self._worker = threading.Thread(target=self._run, name="asr", daemon=True)
        self._worker.start()

    def _subscribe_audio(self, topic: str):
        """Subscribe to the standard audio topic.

        The import stays inside this path so `wav` and `arecord` keep working on
        a machine where audio_common was never installed.
        """
        try:
            from audio_common_msgs.msg import AudioData
        except ImportError as exc:
            raise RuntimeError(
                "source:=topic needs audio_common_msgs (ros-jazzy-audio-capture)"
            ) from exc
        self.get_logger().info(f"audio in: {topic} (audio_common_msgs/AudioData)")
        return self.create_subscription(AudioData, topic, self._on_audio, 10)

    def _on_audio(self, msg: Any) -> None:
        """Executor-thread callback: it must never block on the recogniser.

        The inbox accepts the bytes without waiting and, over budget, evicts
        the oldest audio; the evictions come back here so a shortfall is said
        out loud rather than hidden as silence.
        """
        evicted = self._inbox.put(bytes(msg.data))
        if evicted:
            self._dropped_audio += evicted
            self.get_logger().warning(
                f"recogniser slower than the audio stream: "
                f"{self._dropped_audio} chunk(s) dropped",
                throttle_duration_sec=10.0,
            )

    def _find_model(self, starts: tuple[Path, ...] | None = None) -> str:
        """Locate the ASR model beside the other model assets, the same way the
        decider does -- an installed copy of this module cannot use `__file__`.

        Assets live under `data/models/` at the repo root since ADR-0013 moved
        `brain/`; the streaming zipformer (14M, sherpa-onnx) lives next to the
        kokoro TTS under `acoustic/models/`. `-p model_dir:=` overrides both.
        """
        if self._engine == "zipformer":
            for start in starts or (Path.cwd(),):
                for base in (start, *start.parents):
                    candidate = base / "acoustic" / "models"
                    if candidate.is_dir():
                        models = sorted(
                            p for p in candidate.iterdir()
                            if p.name.startswith("sherpa-onnx-streaming-zipformer")
                        )
                        if models:
                            return str(models[-1])
            return ""
        relative = Path("data") / "models"
        for start in starts or (Path(__file__).resolve().parent, Path.cwd()):
            for base in (start, *start.parents):
                candidate = base / relative
                if candidate.is_dir():
                    models = sorted(p for p in candidate.iterdir() if p.name.startswith("vosk-model"))
                    if models:
                        return str(models[-1])
        return ""

    # -- recognition -------------------------------------------------------------

    def _run(self) -> None:
        try:
            if self._engine == "zipformer":
                self._run_zipformer()
                return
            self._run_vosk()
        except Exception as exc:  # noqa: BLE001 -- a dead mic must not take the node down
            self.get_logger().error(f"speech recognition stopped: {exc}")
        finally:
            if self._once:
                rclpy.shutdown()

    def _run_vosk(self) -> None:
        import vosk

        vosk.SetLogLevel(-1)  # the Kaldi banner is noise on a robot's log
        if not self._model_dir:
            raise FileNotFoundError(
                "no Vosk model found; set -p model_dir:= or place one under "
                "data/models/vosk-model-*"
            )
        model = vosk.Model(self._model_dir)
        recognizer = vosk.KaldiRecognizer(model, SAMPLE_RATE)
        source = build_source(
            self._source_kind,
            wav_path=self._wav_path,
            device=self._device,
            inbox=self._inbox,
            stop=self._stop,
        )
        self.get_logger().info(
            f"speech up: source={self._source_kind} model={self._model_dir}"
        )
        for chunk in source.chunks():
            if self._stop.is_set():
                return
            if recognizer.AcceptWaveform(chunk):
                self._publish(json.loads(recognizer.Result()).get("text", ""))
        self._publish(json.loads(recognizer.FinalResult()).get("text", ""))

    def _run_zipformer(self) -> None:
        """Streaming Zipformer (14M, sherpa-onnx): ~5-10x realtime on CPU,
        unlike Vosk's measured 0.84-1.01x (docs/microphone_array.md)."""
        import numpy as np
        import sherpa_onnx

        if not self._model_dir:
            raise FileNotFoundError(
                "no zipformer model found; set -p model_dir:= or place one under "
                "acoustic/models/sherpa-onnx-streaming-zipformer-*"
            )
        model_dir = Path(self._model_dir)
        recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(
            tokens=str(model_dir / "tokens.txt"),
            encoder=str(model_dir / "encoder-epoch-99-avg-1.onnx"),
            decoder=str(model_dir / "decoder-epoch-99-avg-1.onnx"),
            joiner=str(model_dir / "joiner-epoch-99-avg-1.onnx"),
            num_threads=2,
            provider="cpu",  # a CUDA/TensorRT sherpa wheel (JP7.x) flips this
        )
        source = build_source(
            self._source_kind,
            wav_path=self._wav_path,
            device=self._device,
            inbox=self._inbox,
            stop=self._stop,
        )
        self.get_logger().info(
            f"speech up: source={self._source_kind} zipformer={model_dir.name}"
        )
        stream = recognizer.create_stream()
        for chunk in source.chunks():
            if self._stop.is_set():
                return
            samples = np.frombuffer(chunk, dtype=np.int16).astype(np.float32) / 32768.0
            stream.accept_waveform(SAMPLE_RATE, samples)
            while recognizer.is_ready(stream):
                recognizer.decode_stream(stream)
            if recognizer.is_endpoint(stream):
                text = recognizer.get_result(stream)
                recognizer.reset(stream)
                self._publish(text)
        self._publish(recognizer.get_result(stream))

    def _publish(self, text: str) -> None:
        text = text.strip()
        if not text:
            return
        self._transcript_pub.publish(String(data=text))
        self.get_logger().info(f"transcript: {text}")
        if not self._parse_intent:
            return
        intent = self._parse(text)
        if intent is not None:
            self._intent_pub.publish(String(data=json.dumps(intent, ensure_ascii=False)))
            self.get_logger().info(f"intent: {intent}")

    @staticmethod
    def _parse(text: str) -> dict[str, Any] | None:
        """Hand the transcript to the shared intent contract (rak_intent, ADR-0013).

        This is the same parser the executor uses, so a spoken command and a typed
        one cannot diverge. A transcript that matches nothing is a normal outcome,
        not an error -- people say things the robot cannot act on.
        """
        control = parse_control_command(text)
        if control is not None:
            return {"kind": "control", "command": control.value, "source_text": text}
        navigation = parse_navigation_intent(text)
        if navigation is not None:
            return {
                "kind": "navigate",
                "destination": navigation.destination,
                "source_text": text,
                "schema_version": navigation.schema_version,
            }
        return None

    def destroy_node(self) -> bool:
        self._stop.set()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeechAsr()
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
