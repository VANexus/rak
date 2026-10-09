#!/usr/bin/env python3
"""PTY-only UART v1 loopback; this tool cannot open a physical serial port."""

import argparse
import os
import pty
import select
import struct
import termios
import threading
import time
import tty

from rak_base_driver.protocol import (
    FEATURE_BASE_TELEMETRY,
    FEATURE_BASE_VELOCITY,
    MODE_DISARMED,
    PROTOCOL_VERSION,
    STATE_DISARMED,
    TYPE_BASE_STATE,
    TYPE_HELLO,
    TYPE_HELLO_ACK,
    TYPE_SET_BASE_VELOCITY,
    BaseState,
    Frame,
    StreamDecoder,
    encode_frame,
    pack_base_state,
    sequence_is_newer,
)

BAUD = 921600
REQUIRED_FEATURES = FEATURE_BASE_VELOCITY | FEATURE_BASE_TELEMETRY
HELLO_ACK = struct.Struct("<HIII")
VELOCITY = struct.Struct("<BHiii")


class PtyBoard:
    def __init__(self, master_fd):
        self.master_fd = master_fd
        self.decoder = StreamDecoder()
        self.rx_sequence = None
        self.tx_sequence = 0
        self.commands = 0
        self.errors = []
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def close(self):
        self.running = False
        self.thread.join(timeout=1)

    def _send(self, message_type, payload):
        frame = Frame(PROTOCOL_VERSION, message_type, self.tx_sequence, payload)
        self.tx_sequence = (self.tx_sequence + 1) & 0xFFFF
        os.write(self.master_fd, encode_frame(frame))

    def _run(self):
        while self.running:
            readable, _, _ = select.select([self.master_fd], [], [], 0.02)
            if not readable:
                continue
            try:
                data = os.read(self.master_fd, 512)
            except OSError:
                return
            for frame in self.decoder.feed(data):
                if self.rx_sequence is not None and not sequence_is_newer(
                    frame.sequence, self.rx_sequence
                ):
                    self.errors.append("host sequence did not advance")
                    continue
                self.rx_sequence = frame.sequence
                if frame.message_type == TYPE_HELLO:
                    if len(frame.payload) != 8:
                        self.errors.append("bad HELLO length")
                        continue
                    self._send(
                        TYPE_HELLO_ACK,
                        HELLO_ACK.pack(1, 0x010000, 0x1234, REQUIRED_FEATURES),
                    )
                elif frame.message_type == TYPE_SET_BASE_VELOCITY:
                    try:
                        mode, lease_ms, vx, vy, wz = VELOCITY.unpack(frame.payload)
                    except struct.error:
                        self.errors.append("bad velocity payload length")
                        continue
                    if (mode, lease_ms, vx, vy, wz) != (MODE_DISARMED, 50, 0, 0, 0):
                        self.errors.append("simulator received a non-DISARMED command")
                    self.commands += 1
                    state = BaseState(
                        frame.sequence, STATE_DISARMED, 0, 0, 0, 0, self.commands * 10
                    )
                    self._send(TYPE_BASE_STATE, pack_base_state(state))
                else:
                    self.errors.append(f"unexpected host message 0x{frame.message_type:02x}")
            if self.decoder.errors:
                self.errors.append("host emitted malformed frame")


def set_pty_baud(fd):
    attrs = termios.tcgetattr(fd)
    attrs[4] = termios.B921600
    attrs[5] = termios.B921600
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sec", type=float, default=2.0, help="simulation duration")
    parser.add_argument("--rate", type=int, default=100, help="command rate, maximum 100 Hz")
    args = parser.parse_args()
    if args.sec <= 0 or not 1 <= args.rate <= 100:
        parser.error("--sec must be positive and --rate must be in [1, 100]")

    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)
    set_pty_baud(slave_fd)
    slave_name = os.ttyname(slave_fd)
    board = PtyBoard(master_fd)
    board.start()
    host_decoder = StreamDecoder()
    rx_frames = 0
    hello_ack_seen = False
    host_sequence = 0
    started = time.monotonic()
    next_send = started
    period = 1.0 / args.rate
    try:
        host_fd = os.open(slave_name, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        while time.monotonic() - started < args.sec:
            now = time.monotonic()
            if now >= next_send:
                if not hello_ack_seen:
                    message_type, payload = TYPE_HELLO, struct.pack("<II", 1, REQUIRED_FEATURES)
                else:
                    message_type = TYPE_SET_BASE_VELOCITY
                    payload = VELOCITY.pack(MODE_DISARMED, 50, 0, 0, 0)
                os.write(
                    host_fd,
                    encode_frame(Frame(PROTOCOL_VERSION, message_type, host_sequence, payload)),
                )
                host_sequence = (host_sequence + 1) & 0xFFFF
                next_send = max(next_send + period, now + period if next_send < now else next_send)
            readable, _, _ = select.select([host_fd], [], [], min(0.01, period))
            if readable:
                try:
                    data = os.read(host_fd, 1024)
                except BlockingIOError:
                    data = b""
                for frame in host_decoder.feed(data):
                    rx_frames += 1
                    if frame.message_type == TYPE_HELLO_ACK:
                        hello_ack_seen = True
                    elif frame.message_type != TYPE_BASE_STATE:
                        board.errors.append(f"unexpected board message 0x{frame.message_type:02x}")
        elapsed = time.monotonic() - started
    finally:
        board.close()
        os.close(slave_fd)
        os.close(master_fd)
        if "host_fd" in locals():
            os.close(host_fd)

    if board.errors or not hello_ack_seen or board.commands < max(1, args.rate * args.sec * 0.7):
        raise SystemExit(
            f"PTY test failed: hello_ack={hello_ack_seen}, commands={board.commands}, "
            f"rx_frames={rx_frames}, errors={board.errors}"
        )
    print(
        f"PTY UART v1 PASS: simulated {BAUD} baud, {args.rate} Hz, "
        f"{board.commands} DISARMED commands and {rx_frames} replies in {elapsed:.2f}s"
    )
    print("No physical serial device was opened; no ARM or nonzero velocity was sent.")


if __name__ == "__main__":
    main()
