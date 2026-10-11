"""Audio sources for the speech node.

Every source yields the same thing: 16 kHz, mono, signed 16-bit little-endian PCM
chunks, which is what the Vosk model expects. Keeping the contract at the source
means the recogniser does not care whether the bytes came from a file, a live
capture device, a ROS topic, or a test fixture.

The iFlytek XFM-DP six-microphone array on this robot does **not** stream its six
channels to the host: its UAC interface carries only the array's on-board
beamformed output -- 1 channel, 16 kHz, S16_LE, measured 2026-10-10. A second,
vendor-specific USB interface (class ff/42) is unbound to any driver; raw channel
data would need a private protocol, which is out of scope for the standard stack.
Capture through the UAC device works with plain ALSA today (no vendor init
needed). The measured detail and the verification commands live in
`docs/microphone_array.md`.

Standard input path is `source:=topic`: `audio_capture` (audio_common) reads the
array over GStreamer/ALSA and publishes `audio_common_msgs/AudioData` on `/audio`;
`MicrophoneSource` (an `arecord` subprocess) stays as the dependency-free
fallback for machines without audio_common.
"""

from __future__ import annotations

import subprocess
import threading
import wave
from collections import deque
from pathlib import Path
from queue import Empty
from typing import Iterator, Protocol

#: Bytes per chunk handed to the recogniser. 4000 bytes = 125 ms at 16 kHz mono S16.
CHUNK_BYTES = 4000

SAMPLE_RATE = 16000

#: Bytes for one second of the contract format (16 kHz mono S16 = 32000 B/s).
BYTES_PER_SECOND = SAMPLE_RATE * 2

#: Default hand-off budget: ten seconds of audio. The recogniser is *slower*
#: than real time on this board (measured 0.84-1.01x with vosk-small-cn), so a
#: live stream must be able to run ahead of it; ten seconds covers any voice
#: command while keeping the transcript at most a moment late instead of
#: eating words.
DEFAULT_INBOX_SECONDS = 10.0


class AudioInbox:
    """Byte-bounded hand-off: the subscription callback pushes, the recogniser drains.

    The budget is counted in bytes, not messages, because publishers choose
    their own buffer sizes (audio_capture emits ~10 ms frames here); a
    message-count bound would buy a different time budget from every publisher.

    Over budget, the **oldest** audio is evicted: an over-fast backlog can
    never be paid off, so a delayed transcript is the lesser loss compared to
    silently shredded words. Every eviction is counted for the caller to report.
    """

    def __init__(self, max_bytes: int = int(DEFAULT_INBOX_SECONDS * BYTES_PER_SECOND)) -> None:
        if max_bytes <= 0:
            raise ValueError("max_bytes must be positive")
        self.max_bytes = max_bytes
        self._chunks: deque[bytes] = deque()
        self._bytes = 0
        self._ready = threading.Condition()

    @property
    def depth_bytes(self) -> int:
        with self._ready:
            return self._bytes

    def put(self, data: bytes) -> int:
        """Append one message; return how many older chunks were evicted for budget."""
        with self._ready:
            self._chunks.append(data)
            self._bytes += len(data)
            evicted = 0
            while self._bytes > self.max_bytes and len(self._chunks) > 1:
                old = self._chunks.popleft()
                self._bytes -= len(old)
                evicted += 1
            self._ready.notify()
            return evicted

    def get(self, timeout: float = 0.25) -> bytes:
        """Block up to `timeout` for the next chunk; raise `Empty` on a quiet stream."""
        with self._ready:
            if not self._chunks:
                self._ready.wait_for(lambda: self._chunks, timeout=timeout)
            if not self._chunks:
                raise Empty
            data = self._chunks.popleft()
            self._bytes -= len(data)
            return data


class AudioSource(Protocol):
    def chunks(self) -> Iterator[bytes]:
        """Yield PCM chunks until the source is exhausted."""


class WavFileSource:
    """Reads a RIFF/WAVE file, rejecting anything that is not 16 kHz mono S16.

    Converting silently would be worse than failing: the recogniser would decode
    a resampled or downmixed signal as if it were the real thing and produce a
    confident wrong transcript.
    """

    def __init__(self, path: str | Path, chunk_bytes: int = CHUNK_BYTES) -> None:
        self.path = Path(path)
        self.chunk_bytes = chunk_bytes

    def chunks(self) -> Iterator[bytes]:
        with wave.open(str(self.path), "rb") as handle:
            if handle.getframerate() != SAMPLE_RATE:
                raise ValueError(
                    f"{self.path}: expected {SAMPLE_RATE} Hz, got {handle.getframerate()} Hz"
                )
            if handle.getnchannels() != 1:
                raise ValueError(
                    f"{self.path}: expected mono, got {handle.getnchannels()} channels"
                )
            if handle.getsampwidth() != 2:
                raise ValueError(
                    f"{self.path}: expected 16-bit samples, got {handle.getsampwidth() * 8}-bit"
                )
            while True:
                data = handle.readframes(self.chunk_bytes // 2)
                if not data:
                    return
                yield data


class MicrophoneSource:
    """Live capture through `arecord`, which speaks ALSA and needs no Python bindings."""

    def __init__(
        self,
        device: str = "default",
        sample_rate: int = SAMPLE_RATE,
        chunk_bytes: int = CHUNK_BYTES,
    ) -> None:
        self.device = device
        self.sample_rate = sample_rate
        self.chunk_bytes = chunk_bytes

    def chunks(self) -> Iterator[bytes]:
        command = [
            "arecord",
            "-q",
            "-D", self.device,
            "-f", "S16_LE",
            "-r", str(self.sample_rate),
            "-c", "1",
            "-t", "raw",
        ]
        # A capture device that never opens would otherwise look like silence.
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            assert process.stdout is not None
            while True:
                data = process.stdout.read(self.chunk_bytes)
                if not data:
                    error = (process.stderr.read() if process.stderr else b"").decode(
                        "utf-8", "replace"
                    )
                    raise RuntimeError(f"arecord on {self.device} ended: {error.strip()}")
                yield data
        finally:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()


class TopicSource:
    """Yields PCM pushed by an `audio_common_msgs/AudioData` subscription.

    The subscription callback runs on the executor thread while the recogniser
    runs on its own worker thread, so the hand-off is an `AudioInbox`.
    GStreamer decides the buffer sizes, so bytes accumulate here and are handed
    on only in whole chunks -- an S16 sample must never be split across two
    recogniser calls.
    """

    def __init__(
        self,
        inbox: AudioInbox,
        stop: threading.Event | None = None,
        chunk_bytes: int = CHUNK_BYTES,
    ) -> None:
        if chunk_bytes % 2:
            raise ValueError("chunk_bytes must be even: S16 samples are two bytes")
        self.inbox = inbox
        self.stop = stop
        self.chunk_bytes = chunk_bytes

    def chunks(self) -> Iterator[bytes]:
        pending = bytearray()
        # A live topic has no end of stream: only the node's stop event ends
        # iteration, so a silent microphone must not look like a finished file.
        while self.stop is None or not self.stop.is_set():
            try:
                data = self.inbox.get(timeout=0.25)
            except Empty:
                continue
            pending += data
            if len(pending) >= self.chunk_bytes:
                yield bytes(pending[: self.chunk_bytes])
                del pending[: self.chunk_bytes]


def build_source(
    kind: str,
    *,
    wav_path: str = "",
    device: str = "default",
    inbox: AudioInbox | None = None,
    stop: threading.Event | None = None,
) -> AudioSource:
    if kind == "wav":
        if not wav_path:
            raise ValueError("source:=wav needs wav_path")
        return WavFileSource(wav_path)
    if kind == "arecord":
        return MicrophoneSource(device=device)
    if kind == "topic":
        if inbox is None:
            raise ValueError("source:=topic needs an audio_inbox")
        return TopicSource(inbox, stop=stop)
    raise ValueError(f"unknown audio source {kind!r}; expected wav, arecord or topic")
