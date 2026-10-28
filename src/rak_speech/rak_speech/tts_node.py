"""离线 TTS 节点:/speech/say → sherpa-onnx VITS 合成 → /audio_out 裸 PCM。

播放交给标准 audio_play(audio_common):speech.launch.py 里
`/audio_out → alsasink → 喇叭`(C-Media USB 声卡,按 CARD id 不按编号)。

设计约束:

- 服务回调**只入队**:合成要几百毫秒到几秒,占住 executor 会把唤醒/识别
  回调一起堵死;工作线程顺序合成,队列超预算丢最旧(与 ASR 的 AudioInbox
  同一哲学:短句永远不丢,丢的是过时的旧句)。
- 模型与声学模型同放 `acoustic/models/`(声学 README 留档),缺失时节点
  降级:服务返回 accepted=false,进程不崩,采集/识别链路照常。
- sherpa-onnx 来自 pip(系统已装),不进 package.xml 的 rosdep 契约;
  numpy 由 python3-numpy 声明。
"""

from __future__ import annotations

import threading
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from audio_common_msgs.msg import AudioData

from rak_interfaces.srv import Say

#: 单条文本上限:机器人确认语很短;超长文本按拒绝处理并说清楚原因。
MAX_TEXT = 500


class SpeechTts(Node):
    def __init__(self) -> None:
        super().__init__("speech_tts")

        self.declare_parameter("say_service", "/speech/say")
        self.declare_parameter("audio_topic", "/audio_out")
        self.declare_parameter("model_dir", "")
        self.declare_parameter("model_file", "")  # 具体 .onnx;空=自动挑选
        self.declare_parameter(
            "backend", "sherpa"
        )  # sherpa(kokoro/vits,CPU) | cosyvoice(CosyVoice3-0.5B,GPU,情感 instruct)
        # kokoro 是 82M 参数的 24 kHz 模型,CPU 上约 0.75× 实时;留 2 个核给
        # ASR/驱动。要更低延迟可 model_dir 换 vits-aishell3(8 kHz,约 3× 实时)。
        self.declare_parameter("num_threads", 4)
        self.declare_parameter("lang", "en-us")  # kokoro 的 espeak 语种(zh 词典直查不受影响)
        self.declare_parameter("speed", 1.0)
        self.declare_parameter("sid", 0)
        self.declare_parameter("queue_max", 4)

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self._backend = str(g("backend"))
        self._model_dir = str(g("model_dir"))
        self._model_file = str(g("model_file"))
        self._num_threads = int(g("num_threads"))
        self._lang = str(g("lang"))
        self._speed = float(g("speed"))
        self._sid = int(g("sid"))
        self._queue_max = max(1, int(g("queue_max")))

        self._audio_pub = self.create_publisher(
            AudioData, str(g("audio_topic")), 10
        )

        self._engine = None
        self._engine_error = ""
        self._state = "loading"  # loading | ready | failed
        self._sample_rate = 0

        self._queue: deque[str] = deque()
        self._cond = threading.Condition()
        self._stop = threading.Event()
        self._dropped = 0

        self.create_service(Say, str(g("say_service")), self._on_say)
        self._worker = threading.Thread(target=self._run, name="tts", daemon=True)
        self._worker.start()

    # -- 服务(执行器线程:只入队,绝不合成) --------------------------------------

    def _on_say(self, request: Say.Request, response: Say.Response) -> Say.Response:
        text = (request.text or "").strip()
        if not text:
            response.accepted = False
            response.message = "空文本"
            return response
        if len(text) > MAX_TEXT:
            response.accepted = False
            response.message = f"文本超过 {MAX_TEXT} 字上限"
            return response
        if self._state == "failed":
            response.accepted = False
            response.message = f"TTS 引擎不可用: {self._engine_error}"
            return response
        with self._cond:
            if len(self._queue) >= self._queue_max:
                self._queue.popleft()
                self._dropped += 1
                self.get_logger().warning(
                    f"tts queue full: dropped oldest utterance ({self._dropped} total)"
                )
            self._queue.append((text, request.instruct or ""))
            self._cond.notify()
        response.accepted = True
        response.message = "queued" if self._state == "ready" else "queued (engine loading)"
        return response

    # -- 工作线程 -----------------------------------------------------------------

    def _run(self) -> None:
        try:
            engine, model_dir = self._load_engine()
        except Exception as exc:  # noqa: BLE001 -- 缺模型不能把节点弄崩
            self._state = "failed"
            self._engine_error = str(exc)
            self.get_logger().error(f"tts engine unavailable: {exc}")
            return
        self._engine = engine
        self._state = "ready"
        self.get_logger().info(
            f"tts up: model={model_dir} rate={self._sample_rate}Hz "
            f"threads={self._num_threads} speed={self._speed} sid={self._sid}"
        )

        while not self._stop.is_set():
            with self._cond:
                while not self._queue and not self._stop.is_set():
                    self._cond.wait(timeout=0.5)
                if self._stop.is_set():
                    return
                text, instruct = self._queue.popleft()
            try:
                self._speak(text, instruct)
            except Exception as exc:  # noqa: BLE001 -- 单句失败不能停播
                self.get_logger().error(f"synthesis failed for {text!r}: {exc}")

    def _speak(self, text: str, instruct: str = "") -> None:
        if self._backend == "cosyvoice":
            sample_rate, samples = self._engine.synthesize(text, instruct)
            pcm = (np.clip(samples, -1.0, 1.0) * 32767.0).astype(np.int16).tobytes()
            self._audio_pub.publish(AudioData(data=pcm))
            self.get_logger().info(
                f"say [cosyvoice] ({len(pcm) // (2 * sample_rate) + 1}s): {text}")
            return
        out = self._engine.generate(text, sid=self._sid, speed=self._speed)
        samples = np.asarray(out.samples, dtype=np.float32)
        if samples.size == 0:
            self.get_logger().warning(f"tts produced no audio for {text!r}")
            return
        pcm = (np.clip(samples, -1.0, 1.0) * 32767.0).astype(np.int16).tobytes()
        self._audio_pub.publish(AudioData(data=pcm))
        self.get_logger().info(f"say ({len(pcm) // (2 * self._sample_rate) + 1}s): {text}")

    # -- 模型定位与引擎构建 ----------------------------------------------------------

    def _load_engine(self):
        if self._backend == "cosyvoice":
            from .cosyvoice_backend import CosyvoiceSynthesizer

            synth = CosyvoiceSynthesizer(
                model_dir=self._model_dir,
                instruct="",  # 人格卡 neutral 默认
            )
            self._sample_rate = synth.sample_rate
            return synth, "cosyvoice"
        import sherpa_onnx  # 延迟导入:模型缺失/没装 sherpa 时服务仍能降级应答

        model_dir = Path(self._model_dir) if self._model_dir else self._find_model()
        if model_dir is None:
            raise FileNotFoundError(
                "no TTS model found; set -p model_dir:= or place one under "
                "acoustic/models/<name>/{*.onnx,tokens.txt}"
            )
        onnx = self._pick_onnx(model_dir)
        tokens = model_dir / "tokens.txt"
        if not tokens.is_file():
            raise FileNotFoundError(f"{tokens} missing")

        if "kokoro" in model_dir.name.lower():
            # Kokoro(24 kHz,中英双语):voices.bin 声库 + 逗号分隔的多 lexicon
            voices = model_dir / "voices.bin"
            if not voices.is_file():
                raise FileNotFoundError(f"{voices} missing (kokoro model)")
            lexicons = sorted(model_dir.glob("lexicon*.txt"))
            conf = sherpa_onnx.OfflineTtsKokoroModelConfig(
                model=str(onnx),
                voices=str(voices),
                tokens=str(tokens),
                lexicon=",".join(str(p) for p in lexicons),
                data_dir=str(model_dir),
                dict_dir=str(model_dir / "dict")
                if (model_dir / "dict").is_dir()
                else "",
                lang=self._lang,
            )
            model_conf = sherpa_onnx.OfflineTtsModelConfig(
                kokoro=conf, num_threads=self._num_threads, provider="cpu"
            )
        else:
            # VITS(如 aishell3,8 kHz):lexicon.txt 单词典,无声库
            lexicon = model_dir / "lexicon.txt"
            conf = sherpa_onnx.OfflineTtsVitsModelConfig(
                model=str(onnx),
                tokens=str(tokens),
                lexicon=str(lexicon) if lexicon.is_file() else "",
                data_dir=str(model_dir),
            )
            model_conf = sherpa_onnx.OfflineTtsModelConfig(
                vits=conf, num_threads=self._num_threads, provider="cpu"
            )
        config = sherpa_onnx.OfflineTtsConfig(model=model_conf)
        engine = sherpa_onnx.OfflineTts(config)
        self._sample_rate = int(engine.sample_rate)
        return engine, model_dir

    @staticmethod
    def _find_model() -> Path | None:
        """在声学模型目录里找 TTS 模型目录(kokoro 或 vits)。

        与 asr_node._find_model 同一惯例:从模块文件位置向上找仓库根,
        安装副本退化为从 cwd 找;-p model_dir:= 完全接管搜索。
        只认名字里带 kokoro/vits 的模型——acoustic/models 下还放着 ASR 的
        zipformer(encoder/joiner 也有 tokens.txt),不能被它冒名顶替。
        排序后 kokoro-* 先于 vits-* 被选中(默认优先 24 kHz 音质)。
        """
        for start in (Path(__file__).resolve().parent, Path.cwd()):
            for base in (start, *start.parents):
                for rel in ("acoustic/models", "data/models"):
                    candidate = base / rel
                    if candidate.is_dir():
                        for p in sorted(candidate.iterdir()):
                            if not (p.is_dir() and (p / "tokens.txt").is_file()):
                                continue
                            name = p.name.lower()
                            if "kokoro" in name or "vits" in name:
                                return p
        return None

    def _pick_onnx(self, model_dir: Path) -> Path:
        """指定 model_file 优先;否则偏好全精度(非 int8),再退任意 onnx。"""
        if self._model_file:
            chosen = model_dir / self._model_file
            if not chosen.is_file():
                raise FileNotFoundError(f"model_file not found: {chosen}")
            return chosen
        onnx = sorted(model_dir.glob("*.onnx"))
        full = [p for p in onnx if "int8" not in p.name]
        chosen = (full or onnx)[0]
        return chosen

    def destroy_node(self) -> bool:
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeechTts()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
