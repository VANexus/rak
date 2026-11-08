"""qwentts.cpp 内容层后端(ADR-0018 层2):本地 tts-server,HTTP 流式 PCM。

选型定稿:`qwentts.cpp`(ServeurpersoCom fork)+ `--speaker serena` + instruct
语气档。落地用**同 fork 的 `tts-server`**(OpenAI 兼容本地服务)而不是 CLI 的
`--stream-by-line`,因为服务端给出两件 CLI 给不了的东西:

  * **按请求 instruct**:每句可按内容换情感档(CLI 的 `--instruct` 是进程级,
    只能靠重启进程换)。契合 persona 情感调色板。
  * **精确分帧**:CLI 的行模式在 stdout 上串流「每句一个 RIFF 头」的裸流,
    句边界没有带内标记(沉默期等于句尾),客户端只能靠猜;HTTP 的 chunked
    是可靠的帧边界。

模型常驻(一个子进程,Q4_K_M talker + 12Hz codec),请求之间不重载;`pcm`
响应边生成边下发,首字节(≈首帧出声)实测 ~1s,显著低于整句合成时间。

失败契约(与 cosyvoice_backend 一致):
  * 启动/加载失败 → `QwenttsUnavailable`,节点把内容层标 failed 并降级
    (即时反应层不依赖模型,照常可用);
  * 单句请求失败 → `QwenttsError`,worker 记录并丢掉这一句,不停播。

内存注意:`tools/qwentts.cpp` 的本地补丁给 KV cache 长度加了 `QT_MAX_SEQ`
环境变量(默认 4096 ≈ 896MB;短句场景 1024~2048 足够)。`max_seq` 参数即
`QT_MAX_SEQ`,默认 2048(≈448MB),在 3.4GB 统一内存的 Orin Nano 上留出余量。
"""

from __future__ import annotations

import http.client
import json
import os
import socket
import subprocess
import threading
import time
from collections import deque
from pathlib import Path
from typing import Iterator

#: qwentts 服务端固定 24 kHz 单声道输出(Pipeline hop 1920 @ 24 kHz)。
SAMPLE_RATE = 24000
#: 每句请求的流式读取粒度。保持偶数:下游按 int16 切,奇数字节会错位。
READ_CHUNK = 4096
#: 默认语气档(2026-10-17 用户拍板:清晰自然、亲切温和)。
DEFAULT_INSTRUCT = "清晰自然、亲切温和"


class QwenttsUnavailable(RuntimeError):
    """内容层后端起不来(二进制/模型缺失,或 server 启动失败)。"""


class QwenttsError(RuntimeError):
    """单个请求失败(server 中途死掉、超时、HTTP 非 200)。"""


class QwenttsSynthesizer:
    """一个常驻 tts-server 子进程 + 串行化的流式合成。"""

    def __init__(self, *, server_bin: str, model: str, codec: str,
                 device: str = "CPU", speaker: str = "serena",
                 instruct: str = DEFAULT_INSTRUCT, host: str = "127.0.0.1",
                 port: int = 0, max_seq: int = 2048,
                 startup_timeout: float = 180.0,
                 stall_timeout: float = 20.0,
                 warmup_text: str = "好。") -> None:
        self._bin = Path(server_bin)
        self._model = Path(model)
        self._codec = Path(codec)
        self._device = device or "CPU"
        self._speaker = speaker or "serena"
        self._instruct = instruct or DEFAULT_INSTRUCT
        self._host = host
        self._max_seq = int(max_seq)
        self._stall_timeout = float(stall_timeout)
        self._lock = threading.Lock()
        self._log: deque[str] = deque(maxlen=200)
        self._proc: subprocess.Popen | None = None

        if not self._bin.is_file():
            raise QwenttsUnavailable(f"tts-server 不存在: {self._bin}")
        if not self._model.is_file():
            raise QwenttsUnavailable(f"talker gguf 不存在: {self._model}")
        if not self._codec.is_file():
            raise QwenttsUnavailable(f"codec gguf 不存在: {self._codec}")

        self._port = int(port) or self._free_port()
        self._spawn()
        self._wait_ready(startup_timeout)
        #: warm-up 用时(ms);None = 没做或失败。首个请求要付一次图/内存池冷启动
        #: (实测首帧 ~4.6s vs 之后 ~1s),所以先拿一句废话把它垫掉,别让第一句
        #: 真话术吃掉预算。
        self.warmup_ms: float | None = None
        if warmup_text:
            try:
                started = time.monotonic()
                for _ in self.stream(warmup_text):
                    pass
                self.warmup_ms = (time.monotonic() - started) * 1000.0
            except QwenttsError:
                self.warmup_ms = None

    # -- 生命周期 ---------------------------------------------------------------

    def _free_port(self) -> int:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self._host, 0))
            return int(s.getsockname()[1])

    def _spawn(self) -> None:
        env = dict(os.environ)
        # 本地补丁的 KV cache 长度开关:短句场景别按 4096 分配 896MB。
        env["QT_MAX_SEQ"] = str(self._max_seq)
        self._proc = subprocess.Popen(
            [str(self._bin), "--model", str(self._model),
             "--codec", str(self._codec), "--device", self._device,
             "--host", self._host, "--port", str(self._port)],
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, env=env,
            text=True, bufsize=1,
        )
        threading.Thread(target=self._drain, name="qwentts-log",
                         daemon=True).start()

    def _drain(self) -> None:
        """常读 stderr:server 每句都打 [Perf],不读会填满管道把它堵死。"""
        proc = self._proc
        if proc is None or proc.stderr is None:
            return
        for line in proc.stderr:
            self._log.append(line.rstrip("\n"))

    def _wait_ready(self, timeout: float) -> None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            proc = self._proc
            if proc is not None and proc.poll() is not None:
                raise QwenttsUnavailable(
                    f"tts-server 退出(code={proc.returncode}): {self.tail()}")
            try:
                self._get("/v1/audio/voices", timeout=2.0)
                return
            except OSError:
                time.sleep(1.0)
        raise QwenttsUnavailable(f"tts-server {timeout:.0f}s 未就绪: {self.tail()}")

    def tail(self, n: int = 6) -> str:
        return " | ".join(list(self._log)[-n:])

    @property
    def alive(self) -> bool:
        proc = self._proc
        return proc is not None and proc.poll() is None

    def close(self) -> None:
        proc = self._proc
        if proc is None:
            return
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
        self._proc = None

    # -- 合成 -------------------------------------------------------------------

    @property
    def sample_rate(self) -> int:
        return SAMPLE_RATE

    @property
    def speaker(self) -> str:
        return self._speaker

    def stream(self, text: str, instruct: str = "", speaker: str = "") -> Iterator[bytes]:
        """POST 一句,边生成边 yield S16LE 24k 单声道块(长度恒为偶数)。

        调用方串行使用(节点里只有一个 worker 线程);`_lock` 只是把「一个
        请求占满连接」这件事说明白。
        """
        text = (text or "").strip()
        if not text:
            return
        body = json.dumps({
            "input": text,
            "voice": speaker or self._speaker,
            "instructions": instruct or self._instruct,
            "response_format": "pcm",
        }).encode("utf-8")
        with self._lock:
            yield from self._post_stream(body, text)

    def _post_stream(self, body: bytes, text: str) -> Iterator[bytes]:
        conn = http.client.HTTPConnection(self._host, self._port,
                                          timeout=self._stall_timeout)
        try:
            conn.request("POST", "/v1/audio/speech", body=body,
                         headers={"Content-Type": "application/json",
                                  "Content-Length": str(len(body))})
            resp = conn.getresponse()
            if resp.status != 200:
                detail = resp.read(2000).decode("utf-8", "replace")
                raise QwenttsError(f"HTTP {resp.status}: {detail[:300]}")
            pending = b""
            while True:
                chunk = resp.read(READ_CHUNK)
                if not chunk:
                    break
                pending += chunk
                even = len(pending) & ~1
                if even:                  # 半样本留到下一块,保持 int16 对齐
                    yield pending[:even]
                    pending = pending[even:]
        except (OSError, http.client.HTTPException) as exc:
            raise QwenttsError(f"请求失败({text[:12]!r}): {exc}") from exc
        finally:
            conn.close()

    def _get(self, path: str, timeout: float) -> str:
        conn = http.client.HTTPConnection(self._host, self._port, timeout=timeout)
        try:
            conn.request("GET", path)
            resp = conn.getresponse()
            return resp.read().decode("utf-8", "replace")
        finally:
            conn.close()
