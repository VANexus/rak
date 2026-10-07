"""Offline speech recognition node.

Turns audio into text and, optionally, into the deterministic task intent that
`rak_task_executor` already consumes. It deliberately stops there: the executor
is the only owner of the navigation-task lifecycle (see CLAUDE.md), so this node
publishes a transcript and an intent, and never sends a goal of its own.

Backend is Vosk -- offline, no network at runtime, and an aarch64 wheel, which
matters on a Jetson that must not grow a PyTorch dependency for speech.

    ros2 run rak_speech asr_node --ros-args -p source:=wav -p wav_path:=/tmp/x.wav
"""

from __future__ import annotations

import json
import threading
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .audio import SAMPLE_RATE, build_source


class SpeechAsr(Node):
    def __init__(self) -> None:
        super().__init__("speech_asr")

        self.declare_parameter("source", "arecord")  # wav | arecord
        self.declare_parameter("wav_path", "")
        self.declare_parameter("device", "default")
        self.declare_parameter("model_dir", "")
        self.declare_parameter("transcript_topic", "/speech/transcript")
        self.declare_parameter("intent_topic", "/speech/intent")
        self.declare_parameter("parse_intent", True)
        self.declare_parameter("once", False)  # exit when a file source ends

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self._source_kind = str(g("source"))
        self._wav_path = str(g("wav_path"))
        self._device = str(g("device"))
        self._model_dir = str(g("model_dir")) or self._find_model()
        self._parse_intent = bool(g("parse_intent"))
        self._once = bool(g("once"))

        self._transcript_pub = self.create_publisher(String, str(g("transcript_topic")), 10)
        self._intent_pub = self.create_publisher(String, str(g("intent_topic")), 10)

        self._stop = threading.Event()
        # Recognition blocks on the audio source, so it cannot run on the executor that
        # also serves subscriptions; a daemon thread keeps the node responsive and lets
        # the process exit without a join.
        self._worker = threading.Thread(target=self._run, name="asr", daemon=True)
        self._worker.start()

    def _find_model(self) -> str:
        """Locate the Vosk model beside the other model assets, the same way the
        TensorRT decider does -- an installed copy of this module cannot use `__file__`."""
        from pathlib import Path

        relative = Path("brain") / "data" / "models"
        for start in (Path(__file__).resolve().parent, Path.cwd()):
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
            import vosk

            vosk.SetLogLevel(-1)  # the Kaldi banner is noise on a robot's log
            if not self._model_dir:
                raise FileNotFoundError(
                    "no Vosk model found; set -p model_dir:= or place one under "
                    "brain/data/models/vosk-model-*"
                )
            model = vosk.Model(self._model_dir)
            recognizer = vosk.KaldiRecognizer(model, SAMPLE_RATE)
            source = build_source(
                self._source_kind, wav_path=self._wav_path, device=self._device
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
        except Exception as exc:  # noqa: BLE001 -- a dead mic must not take the node down
            self.get_logger().error(f"speech recognition stopped: {exc}")
        finally:
            if self._once:
                rclpy.shutdown()

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
        """Hand the transcript to the task layer's deterministic parser.

        This is the same `rak_ai.task_intent` the executor uses, so a spoken command
        and a typed one cannot diverge. A transcript that matches nothing is a normal
        outcome, not an error -- people say things the robot cannot act on.
        """
        try:
            from rak_ai.task_intent import parse_control_command, parse_navigation_intent
        except ImportError:
            return None
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
