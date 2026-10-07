"""Audio sources for the speech node.

Every source yields the same thing: 16 kHz, mono, signed 16-bit little-endian PCM
chunks, which is what the Vosk model expects. Keeping the contract at the source
means the recogniser does not care whether the bytes came from a file, a live
capture device, or a test fixture.

The iFlytek XFM-DP microphone array on this robot exposes a **vendor-specific
USB interface** alongside its UAC streaming interface, and its capture endpoint
returns `EIO` on read until that vendor interface has been initialised -- so
`MicrophoneSource` cannot open it today. See `docs/` for the measured detail;
the file source is what the node is exercised against until that is solved.
"""

from __future__ import annotations

import subprocess
import wave
from pathlib import Path
from typing import Iterator, Protocol

#: Bytes per chunk handed to the recogniser. 4000 bytes = 125 ms at 16 kHz mono S16.
CHUNK_BYTES = 4000

SAMPLE_RATE = 16000


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


def build_source(kind: str, *, wav_path: str = "", device: str = "default") -> AudioSource:
    if kind == "wav":
        if not wav_path:
            raise ValueError("source:=wav needs wav_path")
        return WavFileSource(wav_path)
    if kind == "arecord":
        return MicrophoneSource(device=device)
    raise ValueError(f"unknown audio source {kind!r}; expected wav or arecord")
