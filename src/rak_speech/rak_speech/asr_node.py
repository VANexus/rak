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
import time
from pathlib import Path
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import String

from rak_intent.task_intent import parse_control_command, parse_navigation_intent
from rak_lifecycle.managed_node import ManagedNode

from .audio import BYTES_PER_SECOND, DEFAULT_INBOX_SECONDS, SAMPLE_RATE, AudioInbox, build_source


class SpeechAsr(ManagedNode):
    """Offline recognition node, split along ADR-0020 §5.

    `on_configure` pays for the model -- the only expensive resource here -- and opens
    the audio subscription; `on_activate` starts the recogniser thread; `on_cleanup`
    drops the model. A deactivate -> activate cycle therefore reloads nothing.

    A missing model fails `configure` instead of degrading silently: without one this
    node publishes no transcript at all, so "inactive with a reason" is the honest
    state, not "active and mute".
    """

    def __init__(self) -> None:
        super().__init__("speech_asr")

        # Only plain field initialisation here: no model, no device, no thread, no
        # publisher (ADR-0020 §5). These defaults are what "unconfigured" looks like.
        self._source_kind = "arecord"
        self._wav_path = ""
        self._device = "default"
        self._engine = "zipformer"
        self._model_dir = ""
        self._parse_intent = True
        self._once = False

        self._transcript_pub = None
        self._intent_pub = None
        self._inbox = None
        self._dropped_audio = 0
        self._audio_sub = None

        self._stop = threading.Event()
        self._worker = None
        self._model = None  # vosk.Model / sherpa-onnx OnlineRecognizer, opened in configure

        self._transcripts = 0
        self._started_at = 0.0

    # ---------------------------------------------------------------- configure
    def on_configure(self, state) -> TransitionCallbackReturn:
        super().on_configure(state)

        self._engine = str(
            self.declare("engine", "zipformer")
        )  # zipformer (sherpa-onnx streaming 14M, fast) | vosk (legacy CPU)
        self._source_kind = str(self.declare("source", "arecord"))  # topic | arecord | wav
        self._wav_path = str(self.declare("wav_path", ""))
        self._device = str(self.declare("device", "default"))
        audio_topic = str(self.declare("audio_topic", "/audio"))  # source:=topic input
        # How far the hand-off may run ahead of the recogniser. vosk-small-cn
        # decodes at 0.84-1.01x real time on this board, so the stream must be
        # allowed to lead; past this budget the oldest audio is evicted.
        queue_seconds = float(self.declare("audio_queue_seconds", DEFAULT_INBOX_SECONDS))
        self._model_dir = str(self.declare("model_dir", "")) or self._find_model()
        transcript_topic = str(self.declare("transcript_topic", "/speech/transcript"))
        intent_topic = str(self.declare("intent_topic", "/speech/intent"))
        self._parse_intent = bool(self.declare("parse_intent", True))
        self._once = bool(self.declare("once", False))  # exit when a file source ends

        # 未激活不发布：发布器一律走受管包装（ADR-0020 §5）。
        self._transcript_pub = self.lifecycle_publisher(String, transcript_topic, 10)
        self._intent_pub = self.lifecycle_publisher(String, intent_topic, 10)
        self._inbox = AudioInbox(max_bytes=int(queue_seconds * BYTES_PER_SECOND))

        try:
            if self._source_kind == "topic":
                self._audio_sub = self._subscribe_audio(audio_topic)
            # 模型加载是 configure 里唯一的重活（秒级 + 数百 MB）；采集在 activate 才开始。
            self._model = self._load_model()
        except Exception as exc:  # noqa: BLE001 -- configure 失败必须带可读原因，不能崩
            self._release()
            return self.fail(f"识别后端不可用: {type(exc).__name__}: {exc}")

        self.report(
            "engine",
            DiagnosticStatus.OK,
            f"{self._engine} {self._model_dir}",
            {
                "engine": self._engine,
                "model_dir": self._model_dir,
                "source": self._source_kind,
            },
        )
        self.get_logger().info(
            f"speech 已配置: engine={self._engine} source={self._source_kind} "
            f"model={self._model_dir}"
        )
        return TransitionCallbackReturn.SUCCESS

    # ----------------------------------------------------------------- activate
    def on_activate(self, state) -> TransitionCallbackReturn:
        ret = super().on_activate(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret
        self._stop.clear()
        self._dropped_audio = 0
        self._transcripts = 0
        self._started_at = time.monotonic()
        # Recognition blocks on the audio source, so it cannot run on the executor that
        # also serves subscriptions; a daemon thread keeps the node responsive and lets
        # the process exit without a join.
        self._worker = threading.Thread(target=self._run, name="asr", daemon=True)
        self._worker.start()
        self.add_timer(10.0, self._stats)
        return TransitionCallbackReturn.SUCCESS

    # --------------------------------------------------------------- deactivate
    def on_deactivate(self, state) -> TransitionCallbackReturn:
        # 只停 I/O：模型留在内存里，deactivate → activate 不重付一次加载。
        self._stop_worker()
        return super().on_deactivate(state)

    # ------------------------------------------------------------------ cleanup
    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self._release()
        return super().on_cleanup(state)

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        self._release()
        return super().on_shutdown(state)

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
        decider does -- an installed copy of this module cannot use `__file__`
        as the only anchor, so both the module path and cwd are walked (the
        zipformer branch once walked only cwd: launching the stack from $HOME
        made a perfectly good model look missing).

        Assets live under `data/models/` at the repo root since ADR-0013 moved
        `brain/`; the streaming zipformer (14M, sherpa-onnx) lives next to the
        kokoro TTS under `acoustic/models/`. `-p model_dir:=` overrides both.
        """
        if self._engine == "zipformer":
            for start in starts or (Path(__file__).resolve().parent, Path.cwd()):
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

    # -- model (configure) -------------------------------------------------------

    def _load_model(self) -> Any:
        """打开识别模型；缺失/装不上就抛，由 configure 转成带原因的 FAILURE。"""
        if self._engine == "zipformer":
            return self._load_zipformer()
        return self._load_vosk()

    def _load_vosk(self) -> Any:
        import vosk

        vosk.SetLogLevel(-1)  # the Kaldi banner is noise on a robot's log
        if not self._model_dir:
            raise FileNotFoundError(
                "no Vosk model found; set -p model_dir:= or place one under "
                "data/models/vosk-model-*"
            )
        return vosk.Model(self._model_dir)

    def _load_zipformer(self) -> Any:
        """Streaming Zipformer (14M, sherpa-onnx): ~5-10x realtime on CPU,
        unlike Vosk's measured 0.84-1.01x (docs/microphone_array.md)."""
        import sherpa_onnx

        if not self._model_dir:
            raise FileNotFoundError(
                "no zipformer model found; set -p model_dir:= or place one under "
                "acoustic/models/sherpa-onnx-streaming-zipformer-*"
            )
        model_dir = Path(self._model_dir)
        return sherpa_onnx.OnlineRecognizer.from_transducer(
            tokens=str(model_dir / "tokens.txt"),
            encoder=str(model_dir / "encoder-epoch-99-avg-1.onnx"),
            decoder=str(model_dir / "decoder-epoch-99-avg-1.onnx"),
            joiner=str(model_dir / "joiner-epoch-99-avg-1.onnx"),
            num_threads=2,
            provider="cpu",  # a CUDA/TensorRT sherpa wheel (JP7.x) flips this
        )

    # -- recognition (activate) --------------------------------------------------

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

        recognizer = vosk.KaldiRecognizer(self._model, SAMPLE_RATE)
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
        """Streaming Zipformer loop for the recognizer built in `_load_zipformer`."""
        import numpy as np

        recognizer = self._model
        model_dir = Path(self._model_dir)
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
        self.publish(String(data=text), publisher=self._transcript_pub)
        self._transcripts += 1
        self.get_logger().info(f"transcript: {text}")
        if not self._parse_intent:
            return
        intent = self._parse(text)
        if intent is not None:
            self.publish(
                String(data=json.dumps(intent, ensure_ascii=False)),
                publisher=self._intent_pub,
            )
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

    # -- health / teardown -------------------------------------------------------

    def _stats(self) -> None:
        """周期性统计：每分钟转写数就是这条链路的"还在干活"证据。"""
        elapsed = time.monotonic() - self._started_at
        per_min = (self._transcripts / elapsed * 60.0) if elapsed > 0 else 0.0
        level = DiagnosticStatus.OK
        if self._dropped_audio:
            level = DiagnosticStatus.WARN
        self.report(
            "asr",
            level,
            f"{per_min:.1f} transcripts/min",
            {
                "transcripts_per_min": f"{per_min:.1f}",
                "transcripts": self._transcripts,
                "dropped_chunks": self._dropped_audio,
                "engine": self._engine,
                "source": self._source_kind,
            },
        )

    def _stop_worker(self) -> None:
        self._stop.set()
        worker, self._worker = self._worker, None
        if worker is not None and worker.is_alive():
            # 识别循环每块音频（125ms）检查一次 stop，正常秒级退出。
            worker.join(timeout=2.0)

    def _release(self) -> None:
        """只有 cleanup/shutdown 走这里：模型与订阅是真资源，deactivate 不动它们。"""
        self._stop_worker()
        self._model = None
        if self._audio_sub is not None:
            try:
                self.destroy_subscription(self._audio_sub)
            except Exception:  # noqa: BLE001 -- 退出路径不打新错
                pass
            self._audio_sub = None
        self._inbox = None

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
