"""离线 TTS 节点:/speech/say → 合成 → /audio_out 裸 PCM(ADR-0018 三层)。

语音输出分三层(ADR-0018),节点按此组织:

  1. **即时反应层**:`/speech/say` 的文案若命中预合成素材集
     (`quick_phrases.toml` + `acoustic/models/quick/*.wav`),直接播 wav,
     **不经过任何推理**(<100ms);`/speech/quick`(String=素材 key)也走这层。
     这一层不依赖模型——内容层挂了它照常出声。
  2. **内容层**:其余句子交给 `backend`(默认 `qwentts`,qwentts.cpp +
     speaker=serena,本地常驻 server、PCM 流式);长内容按句拆开排队,
     一次只合成一句,句间留停顿。
  3. **响应预算**:即时层 <100ms;单句首帧由后端流式保证(~1s);单句合成
     超过 `sentence_budget` 丢弃并降级为即时层素材。

播放交给标准 audio_play(audio_common):speech.launch.py 里
`/audio_out → alsasink → 喇叭`(C-Media USB 声卡,按 CARD id 不按编号)。

设计约束:

- 服务回调**只入队**:合成要几百毫秒到几秒,占住 executor 会把唤醒/识别
  回调一起堵死;工作线程顺序合成,队列超预算丢最旧(与 ASR 的 AudioInbox
  同一哲学:短句永远不丢,丢的是过时的旧句)。即时层例外——它必须立刻出声,
  所以直接从回调线程播:发一个单调计数,worker 在流式过程中看见就让位,
  当前句作废、该条的后续句也不再垫进去(反射优先于内容),但**队列里别人
  的句子不受影响**。即时层与内容层共享一把发布锁,两个流不会交织成字节噪声。
- 模型与声学模型同放 `acoustic/models/`(声学 README 留档),缺失时节点
  降级:服务返回 accepted=false,进程不崩,采集/识别链路照常。
- sherpa-onnx 来自 pip(系统已装),不进 package.xml 的 rosdep 契约;
  numpy 由 python3-numpy 声明。
"""

from __future__ import annotations

import os
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import String

from audio_common_msgs.msg import AudioData

from rak_interfaces.srv import Say
from rak_lifecycle.managed_node import ManagedNode

from .tts_layers import QuickAssets, split_sentences

#: 单条文本上限:机器人确认语很短;超长文本按拒绝处理并说清楚原因。
MAX_TEXT = 500
#: 即时层匹配失败/内容层超时时的降级素材 key(见 quick_phrases.toml)。
DEGRADE_KEY = "hold"


class SpeechTts(ManagedNode):
    """离线 TTS,按 ADR-0020 §5 拆分。

    `on_configure` 建接口、载即时层素材、并**只在这里**加载内容层引擎(cosyvoice
    约 1.7 GB);`on_activate` 才开始合成线程;`on_deactivate` 停线程但**不卸载引擎**
    —— deactivate → activate 是秒级恢复,不该重付一次模型加载;`on_cleanup` 才 close。

    引擎缺失是**降级**而不是 configure 失败:即时层素材在磁盘上、不依赖模型,
    服务仍要能回 accepted=false(ADR-0018 的既定行为),所以这里报 ERROR 诊断后
    照常返回 SUCCESS。
    """

    def __init__(self) -> None:
        super().__init__("speech_tts")

        # 只做字段初始化:引擎、素材、发布器、服务、线程都归 configure/activate
        # (ADR-0020 §5)。下面这些默认值是"未配置"时的样子。
        self._backend = "qwentts"
        self._model_dir = ""
        self._model_file = ""
        self._num_threads = 4
        self._lang = "en-us"
        self._speed = 1.0
        self._sid = 0
        self._queue_max = 4
        self._speaker = "serena"
        self._instruct = ""
        self._sentence_max_chars = 40
        self._sentence_pause = 0.5
        self._sentence_budget = 30.0
        self._first_frame_budget = 1.5

        self._audio_pub = None
        self._quick = None
        self._say_service = None
        self._quick_sub = None

        self._engine = None
        self._engine_error = ""
        self._state = "loading"  # loading | ready | failed
        self._sample_rate = 0

        self._queue: deque[tuple[str, str]] = deque()
        self._cond = threading.Condition()
        self._stop = threading.Event()
        self._dropped = 0
        # 内容层正在往 /audio_out 流式写;即时层靠 _pub_lock 保证不交织,
        # 靠 _interrupt 单调计数打断当前这句(计数只增不减,worker 取快照对比,
        # 没有 set/clear 竞态)。
        self._pub_lock = threading.Lock()
        self._interrupt = 0

        self._worker = None
        self._utterances = 0
        self._errors = 0
        self._last_ttfa_ms = 0.0
        self._max_ttfa_ms = 0.0

    # ---------------------------------------------------------------- configure
    def on_configure(self, state) -> TransitionCallbackReturn:
        super().on_configure(state)

        say_service = str(self.declare("say_service", "/speech/say"))
        audio_topic = str(self.declare("audio_topic", "/audio_out"))
        quick_topic = str(self.declare("quick_topic", "/speech/quick"))
        self._model_dir = str(self.declare("model_dir", ""))
        self._model_file = str(self.declare("model_file", ""))  # 具体 .onnx;空=自动挑选
        self._backend = str(
            self.declare("backend", "qwentts")
        )  # qwentts(qwentts.cpp+serena,默认) | sherpa(kokoro/vits,CPU) | cosyvoice(GPU)
        # kokoro 是 82M 参数的 24 kHz 模型,CPU 上约 0.75× 实时;留 2 个核给
        # ASR/驱动。要更低延迟可 model_dir 换 vits-aishell3(8 kHz,约 3× 实时)。
        self._num_threads = int(self.declare("num_threads", 4))
        self._lang = str(self.declare("lang", "en-us"))  # kokoro 的 espeak 语种(zh 词典直查不受影响)
        self._speed = float(self.declare("speed", 1.0))
        self._sid = int(self.declare("sid", 0))
        self._queue_max = max(1, int(self.declare("queue_max", 4)))
        # -- qwentts 内容层(ADR-0018 定稿路线) --------------------------------
        self.declare("qwentts_server", "")  # 空=仓库内 tools/qwentts.cpp/build/tts-server
        self.declare("qwentts_model", "")   # 空=自动挑 qwen-talker-*-customvoice-*.gguf
        self.declare("qwentts_codec", "")   # 空=自动挑 qwen-tokenizer-*.gguf
        self.declare("qwentts_device", "CPU")  # GPU 路径待上游修复(qwentts.cpp#40)
        self.declare("qwentts_max_seq", 2048)  # KV cache 长度(=QT_MAX_SEQ);4096≈896MB
        self.declare("qwentts_warmup", True)   # 启动时用一句废话垫掉冷启动
        self._speaker = str(self.declare("speaker", "serena")) or "serena"  # 2026-10-17 定稿音色
        self._instruct = str(self.declare("instruct", ""))  # 空=后端默认语气档
        # -- 即时反应层 + 响应预算 --------------------------------------------
        quick_spec = str(self.declare("quick_spec", ""))    # 空=仓库内 src/rak_speech/config/quick_phrases.toml
        quick_wav_dir = str(self.declare("quick_wav_dir", ""))  # 空=仓库内 acoustic/models/quick
        self._sentence_max_chars = max(1, int(self.declare("sentence_max_chars", 40)))
        self._sentence_pause = max(0.0, float(self.declare("sentence_pause", 0.5)))
        self._sentence_budget = max(1.0, float(self.declare("sentence_budget", 30.0)))
        self._first_frame_budget = max(0.1, float(self.declare("first_frame_budget", 1.5)))

        self._audio_pub = self.lifecycle_publisher(AudioData, audio_topic, 10)

        try:
            # 即时反应层:先于模型加载——素材在磁盘上就能出声,不依赖内容层。
            spec = quick_spec or os.environ.get("RAK_TTS_QUICK_SPEC", "")
            wav_dir = quick_wav_dir or os.environ.get("RAK_TTS_QUICK_DIR", "")
            self._quick = QuickAssets.load(
                Path(spec) if spec else self._find_quick_spec(),
                Path(wav_dir) if wav_dir else self._find_quick_dir(),
            )
            self._say_service = self.create_service(Say, say_service, self._on_say)
            self._quick_sub = self.create_subscription(
                String, quick_topic, self._on_quick, 10
            )
        except Exception as exc:  # noqa: BLE001 -- configure 失败必须带可读原因
            self._release()
            return self.fail(f"即时层/接口创建失败: {type(exc).__name__}: {exc}")

        # 内容层引擎:configure 里唯一的重活(cosyvoice ≈1.7 GB,秒级到十秒级)。
        try:
            self._engine, label = self._load_engine()
        except Exception as exc:  # noqa: BLE001 -- 缺模型不能把节点弄崩
            self._state = "failed"
            self._engine_error = str(exc)
            self.get_logger().error(f"tts engine unavailable: {exc}")
            self.get_logger().error("即时反应层仍可用(预合成素材不依赖模型)")
            self.report(
                "engine",
                DiagnosticStatus.ERROR,
                f"内容层不可用: {exc}",
                {"backend": self._backend, "state": self._state},
            )
            return TransitionCallbackReturn.SUCCESS

        self._state = "ready"
        voice = (f" speaker={self._speaker}" if self._backend == "qwentts"
                 else f" sid={self._sid}")
        self.get_logger().info(
            f"tts up: backend={self._backend} model={label} "
            f"rate={self._sample_rate}Hz threads={self._num_threads} "
            f"speed={self._speed}{voice}"
        )
        # 即时层素材与内容层采样率必须一致:audio_play 只按一个 rate 播 /audio_out,
        # 不一致会让素材变调(且没人会在日志里发现)。
        odd = self._quick.sample_rates - {self._sample_rate}
        if odd:
            self.get_logger().warning(
                f"即时层素材采样率 {sorted(odd)} != 内容层 {self._sample_rate}Hz;"
                "素材会变调,请用同一个 rate 重新生成"
            )
        self.get_logger().info(
            f"即时层就绪: {len(self._quick.keys)} 条素材"
            + (f", 缺 wav: {self._quick.missing}" if self._quick.missing else "")
        )
        self.report(
            "engine",
            DiagnosticStatus.OK,
            f"{self._backend} {self._sample_rate}Hz",
            {
                "backend": self._backend,
                "sample_rate": self._sample_rate,
                "quick_keys": len(self._quick.keys),
            },
        )
        return TransitionCallbackReturn.SUCCESS

    # ----------------------------------------------------------------- activate
    def on_activate(self, state) -> TransitionCallbackReturn:
        ret = super().on_activate(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret
        self._stop.clear()
        self._dropped = 0
        self._utterances = 0
        self._errors = 0
        self._last_ttfa_ms = 0.0
        self._max_ttfa_ms = 0.0
        self._started_at = time.monotonic()
        self._worker = threading.Thread(target=self._run, name="tts", daemon=True)
        self._worker.start()
        self.add_timer(10.0, self._stats)
        return TransitionCallbackReturn.SUCCESS

    # --------------------------------------------------------------- deactivate
    def on_deactivate(self, state) -> TransitionCallbackReturn:
        # 停的是"在跑的东西",不是 1.7 GB 的引擎:deactivate → activate 不重付加载。
        self._stop_worker()
        return super().on_deactivate(state)

    # ------------------------------------------------------------------ cleanup
    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self._release()
        return super().on_cleanup(state)

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        self._release()
        return super().on_shutdown(state)

    def _stop_worker(self) -> None:
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        worker, self._worker = self._worker, None
        if worker is not None and worker.is_alive():
            worker.join(timeout=2.0)

    def _release(self) -> None:
        """只有 cleanup/shutdown 走这里:引擎与接口是真资源,deactivate 不动它们。"""
        self._stop_worker()
        engine, self._engine = self._engine, None
        if engine is not None and hasattr(engine, "close"):
            try:
                engine.close()
            except Exception:  # noqa: BLE001 -- 退出路径不打新错
                pass
        if self._say_service is not None:
            try:
                self.destroy_service(self._say_service)
            except Exception:  # noqa: BLE001
                pass
            self._say_service = None
        if self._quick_sub is not None:
            try:
                self.destroy_subscription(self._quick_sub)
            except Exception:  # noqa: BLE001
                pass
            self._quick_sub = None
        self._quick = None
        self._state = "loading"
        self._engine_error = ""
        self._sample_rate = 0

    def _stats(self) -> None:
        """周期性统计:首帧延迟就是 ADR-0018 的响应预算,必须是可观测的数字。"""
        level = DiagnosticStatus.OK
        if self._state == "failed":
            level = DiagnosticStatus.ERROR
        elif self._errors or self._last_ttfa_ms > self._first_frame_budget * 1000.0:
            level = DiagnosticStatus.WARN
        self.report(
            "tts",
            level,
            f"{self._state}, ttfa {self._last_ttfa_ms:.0f}ms",
            {
                "state": self._state,
                "backend": self._backend,
                "utterances": self._utterances,
                "errors": self._errors,
                "queue": len(self._queue),
                "dropped": self._dropped,
                "ttfa_ms": f"{self._last_ttfa_ms:.0f}",
                "ttfa_max_ms": f"{self._max_ttfa_ms:.0f}",
            },
        )

    def _note_ttfa(self, started: float) -> None:
        self._last_ttfa_ms = (time.monotonic() - started) * 1000.0
        if self._last_ttfa_ms > self._max_ttfa_ms:
            self._max_ttfa_ms = self._last_ttfa_ms

    # -- 服务/话题(执行器线程:即时层直出,内容层只入队) ------------------------

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
        # 即时层优先,且不依赖内容层是否可用。
        pcm = self._quick.pcm_for_text(text)
        if pcm is not None:
            self._play_now(pcm, f"instant[{self._quick.key_for_text(text)}]")
            response.accepted = True
            response.message = "instant"
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

    def _on_quick(self, msg: String) -> None:
        """`/speech/quick` = 素材 key,直接出声(ADR-0010 表达层的即时出口)。"""
        key = (msg.data or "").strip()
        pcm = self._quick.pcm_for_key(key)
        if pcm is None:
            self.get_logger().warning(
                f"quick: 未知素材 key {key!r}(有: {self._quick.keys})"
            )
            return
        self._play_now(pcm, f"quick[{key}]")

    def _play_now(self, pcm: bytes, tag: str) -> None:
        """即时层直出:打断内容层当前句,独占 /audio_out 写完这一条。

        先加计数(worker 下一块就会看见并让位),再拿锁整条写完——拿锁前 worker
        可能正握着锁写最后一块,所以顺序是「先喊停,再等锁」。
        """
        self._interrupt += 1
        with self._pub_lock:
            self.publish(AudioData(data=pcm), publisher=self._audio_pub)
        self.get_logger().info(f"say {tag}: {len(pcm) // 48}ms audio")

    # -- 工作线程 -----------------------------------------------------------------

    def _run(self) -> None:
        """内容层工作线程:引擎在 configure 已起好,这里只排队/合成(activate 才开跑)。

        即时层不经过这里——它在服务回调线程直出,所以引擎不可用时它照常出声。
        """
        if self._engine is None:
            self.get_logger().warning(
                "内容层引擎不可用,合成线程不启动(即时反应层照常)"
            )
            return
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
                self._errors += 1
                self.get_logger().error(f"synthesis failed for {text!r}: {exc}")

    def _speak(self, text: str, instruct: str = "") -> None:
        started = time.monotonic()
        self._utterances += 1
        if self._backend == "qwentts":
            self._speak_qwentts(text, instruct)
            return
        if self._backend == "cosyvoice":
            sample_rate, samples = self._engine.synthesize(text, instruct)
            pcm = (np.clip(samples, -1.0, 1.0) * 32767.0).astype(np.int16).tobytes()
            self._publish_chunk(pcm)
            self._note_ttfa(started)
            self.get_logger().info(
                f"say [cosyvoice] ({len(pcm) // (2 * sample_rate) + 1}s): {text}")
            return
        out = self._engine.generate(text, sid=self._sid, speed=self._speed)
        samples = np.asarray(out.samples, dtype=np.float32)
        if samples.size == 0:
            self.get_logger().warning(f"tts produced no audio for {text!r}")
            return
        pcm = (np.clip(samples, -1.0, 1.0) * 32767.0).astype(np.int16).tobytes()
        self._publish_chunk(pcm)
        self._note_ttfa(started)
        self.get_logger().info(f"say ({len(pcm) // (2 * self._sample_rate) + 1}s): {text}")

    # -- 内容层:分句 + 流式 + 预算(ADR-0018 层2/层3) ----------------------------

    def _speak_qwentts(self, text: str, instruct: str) -> None:
        sentences = split_sentences(text, self._sentence_max_chars)
        for idx, sentence in enumerate(sentences):
            if self._stop.is_set():
                return
            try:
                done = self._stream_sentence(sentence, instruct)
            except Exception as exc:  # noqa: BLE001 -- 超预算/后端失败都降级
                self._errors += 1
                self.get_logger().error(f"内容层丢句 {sentence!r}: {exc}")
                self._degrade(exc)
                return
            if not done:      # 即时层插话:这句作废,后面几句也不再垫进去
                return
            if idx < len(sentences) - 1:
                self._sleep_pause()

    def _stream_sentence(self, sentence: str, instruct: str) -> bool:
        """流式播一句;返回 True=播完,False=被即时层打断。"""
        started = time.monotonic()
        deadline = started + self._sentence_budget
        seq = self._interrupt
        first = True
        total = 0
        for chunk in self._engine.stream(sentence, instruct, self._speaker):
            if self._stop.is_set():
                return False
            if self._interrupt != seq:  # 即时层插话:让位,当前句作废
                self.get_logger().info(
                    f"内容句被即时层打断(已播 {total // (2 * self._sample_rate)}s): {sentence!r}"
                )
                return False
            if time.monotonic() > deadline:
                raise TimeoutError(f"单句超预算 {self._sentence_budget:.0f}s")
            now = time.monotonic()
            if first:
                first = False
                ttfa = now - started
                self._last_ttfa_ms = ttfa * 1000.0
                if self._last_ttfa_ms > self._max_ttfa_ms:
                    self._max_ttfa_ms = self._last_ttfa_ms
                # 注意:两个 severity 必须是两个调用点——rclpy 按调用点记 severity,
                # 同一行的 `level(...)` 在第二次换成另一种 severity 会直接抛
                # ValueError('Logger severity cannot be changed between calls')。
                if ttfa > self._first_frame_budget:
                    self.get_logger().warning(
                        f"首帧 {ttfa * 1000:.0f}ms 超预算"
                        f"(>{self._first_frame_budget * 1000:.0f}ms): {sentence!r}"
                    )
                else:
                    self.get_logger().info(
                        f"首帧 {ttfa * 1000:.0f}ms: {sentence!r}"
                    )
            self._publish_chunk(chunk)
            total += len(chunk)
        if total:
            self.get_logger().info(
                f"say [qwentts/{self._speaker}] {total // (2 * self._sample_rate)}s: "
                f"{sentence}"
            )
        return True

    def _sleep_pause(self) -> None:
        seq = self._interrupt
        deadline = time.monotonic() + self._sentence_pause
        while time.monotonic() < deadline:
            if self._stop.is_set() or self._interrupt != seq:
                return
            time.sleep(0.05)

    def _degrade(self, exc: Exception) -> None:
        """内容层失败 → 先保一句即时层素材,再决定后端是否彻底不可用。"""
        pcm = self._quick.pcm_for_key(DEGRADE_KEY)
        if pcm is not None:
            self._play_now(pcm, f"degrade[{DEGRADE_KEY}]")
        engine = self._engine
        if engine is not None and not getattr(engine, "alive", True):
            self._state = "failed"
            self._engine_error = f"内容层后端已退出: {exc}"
            self.get_logger().error(self._engine_error)

    def _publish_chunk(self, pcm: bytes) -> None:
        if not pcm:
            return
        with self._pub_lock:
            self.publish(AudioData(data=pcm), publisher=self._audio_pub)

    # -- 模型定位与引擎构建 ----------------------------------------------------------

    def _load_engine(self):
        if self._backend == "qwentts":
            from .qwentts_backend import DEFAULT_INSTRUCT, QwenttsSynthesizer

            server, model, codec = self._find_qwentts()
            warm = bool(self.get_parameter("qwentts_warmup").value)
            synth = QwenttsSynthesizer(
                server_bin=server, model=model, codec=codec,
                device=str(self.get_parameter("qwentts_device").value),
                speaker=self._speaker,
                instruct=self._instruct or DEFAULT_INSTRUCT,
                max_seq=int(self.get_parameter("qwentts_max_seq").value),
                warmup_text="好。" if warm else "",
            )
            try:
                self._sample_rate = synth.sample_rate
                label = f"{Path(model).name} ({synth.speaker})"
                if synth.warmup_ms is not None:
                    label += f", warmup {synth.warmup_ms / 1000:.1f}s"
            except Exception:  # 起完 server 再出错也要把子进程收掉,别留孤儿
                synth.close()
                raise
            return synth, label
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

    def _find_qwentts(self) -> tuple[str, str, str]:
        """定位 tts-server + talker/codec GGUF。

        参数(或环境变量)显式给出优先;否则从仓库根找 `tools/qwentts.cpp/`,
        等价于 `cosyvoice_backend` 的 `RAK_COSYVOICE_*` 约定。customvoice 的
        Q4_K_M 优先(整句 RTF 与内存的最小代价),没有就取排序第一个。
        """
        g = lambda n: str(self.get_parameter(n).value)  # noqa: E731
        server = g("qwentts_server") or os.environ.get("RAK_QWENTTS_SERVER", "")
        model = g("qwentts_model") or os.environ.get("RAK_QWENTTS_MODEL", "")
        codec = g("qwentts_codec") or os.environ.get("RAK_QWENTTS_CODEC", "")
        if server and model and codec:
            return server, model, codec

        root = self._resolve_repo_path("tools/qwentts.cpp")
        if root is None:
            raise FileNotFoundError(
                "找不到 tools/qwentts.cpp;用 -p qwentts_server:=/qwentts_model:=/qwentts_codec:= "
                "或环境变量 RAK_QWENTTS_{SERVER,MODEL,CODEC} 指定"
            )
        server = server or str(root / "build" / "tts-server")
        models_dir = root / "models"
        if not model:
            talkers = sorted(models_dir.glob("qwen-talker-*-customvoice-*.gguf"))
            talkers.sort(key=lambda p: ("Q4_K_M" not in p.name, p.name))
            if not talkers:
                raise FileNotFoundError(f"{models_dir} 下没有 qwen-talker-*-customvoice-*.gguf")
            model = str(talkers[0])
        if not codec:
            codecs = sorted(models_dir.glob("qwen-tokenizer-*.gguf"))
            if not codecs:
                raise FileNotFoundError(f"{models_dir} 下没有 qwen-tokenizer-*.gguf")
            codec = str(codecs[0])
        return server, model, codec

    def _find_quick_spec(self) -> Path:
        """素材清单:环境变量 > 已安装的 share 副本 > 仓库源(开发树)。"""
        env = os.environ.get("RAK_TTS_QUICK_SPEC")
        if env:
            return Path(env)
        try:  # 随包安装的那份(ament share),与 config/ 的安装约定一致
            from ament_index_python.packages import get_package_share_directory

            installed = Path(get_package_share_directory("rak_speech")) / "config" \
                / "quick_phrases.toml"
            if installed.is_file():
                return installed
        except Exception:  # noqa: BLE001 -- 没装 ament 索引也能跑开发树
            pass
        found = self._resolve_repo_path("src/rak_speech/config/quick_phrases.toml")
        return found if found else Path("quick_phrases.toml")

    def _find_quick_dir(self) -> Path:
        env = os.environ.get("RAK_TTS_QUICK_DIR")
        if env:
            return Path(env)
        found = self._resolve_repo_path("acoustic")
        return (found / "models" / "quick") if found else Path("quick")

    @staticmethod
    def _resolve_repo_path(rel: str) -> Path | None:
        """把「相对仓库根的路径」解析成绝对路径(找不到返回 None)。

        从模块位置向上找,再退化为从 cwd 找——开发树和安装副本(install/ 在仓库
        内)都能落回 gitignored 的 `tools/`、`acoustic/`。与 `_find_model` 同一惯例。
        """
        for start in (Path(__file__).resolve().parent, Path.cwd()):
            for base in (start, *start.parents):
                candidate = base / rel
                if candidate.exists():
                    return candidate
        return None

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
        self._release()
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
