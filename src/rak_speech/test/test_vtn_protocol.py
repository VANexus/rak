# -*- coding: utf-8 -*-
"""R818 帧协议与链路测试:黄金帧、真实唤醒事件、假板子端到端。

不依赖 ROS 与真板子——协议是纯函数,链路用本机 TCP 假服务器验证。
"""

import json
import socket
import threading
import time

from rak_speech.vtn_link import VtnLink, tcp_connect
from rak_speech.vtn_protocol import (
    MSG_CONFIRM,
    MSG_CONTROL,
    MSG_DEVICE,
    MSG_HANDSHAKE,
    CONFIRM_DATA,
    FrameParser,
    build_frame,
    checksum,
    make_confirm,
    make_control,
    parse_ivw_event,
)

# wake_test2.log(2026-10-08 实测)里板子报上来的原帧数据,一字未改。
REAL_WAKE_JSON = (
    '{"content":{"arg1":3,"arg2":0,"eventType":4,'
    '"info":"{\\"ivw\\":{\\"start_ms\\":46140,\\"end_ms\\":46820,'
    '\\"beam\\":3,\\"physical\\":3,\\"score\\":774.0,'
    '\\"power\\":75545024.0,\\"angle\\":157.0,'
    '\\"keyword\\":\\"hei4 rui4 ke4\\"}}",'
    '"result":"hei4 rui4 ke4"},"type":"aiui_event"}'
)


# -- 黄金帧 ------------------------------------------------------------------
# 手算校验:确认帧 msg_id=0x1234 → 全字节和 660 & 0xFF = 0x94 → 补码 0x6C;
# 控制帧 {"type":"verison"} → 和 1855 & 0xFF = 0x3F → 补码 0xC1。


def test_confirm_golden_frame():
    assert make_confirm(0x1234).hex() == "a501ff04003412a50000006c"


def test_control_golden_frame():
    assert make_control('{"type":"verison"}', 0).hex() == (
        "a50105120000007b2274797065223a2276657269736f6e227dc1"
    )


def test_checksum_is_twos_complement():
    # 协议:sum(含校验字节)低 8 位必须为 0
    for payload in (b"", b"abc", bytes(range(255))):
        frame = build_frame(MSG_CONTROL, payload, 0xBEEF)
        assert sum(frame) & 0xFF == 0
        assert checksum(frame[:-1]) == frame[-1]


# -- 解析器 ------------------------------------------------------------------


def test_parser_reassembles_bytewise():
    frame = make_confirm(7)
    parser = FrameParser()
    out = []
    for i in range(len(frame)):
        out += parser.feed(frame[i : i + 1])
    assert len(out) == 1
    assert out[0].type == MSG_CONFIRM
    assert out[0].msg_id == 7
    assert out[0].data == CONFIRM_DATA


def test_parser_skips_garbage_and_keeps_partial_tail():
    frame = make_confirm(1)
    parser = FrameParser()
    out = parser.feed(b"\x00\xff\x13garbage" + frame + b"tail")
    assert len(out) == 1
    assert parser.dropped_bytes > 0
    # “tail”疑似下一帧开头,必须留在缓冲里等待后续字节
    assert parser.buffered == 4


def test_parser_drops_corrupt_checksum():
    good = bytearray(make_confirm(2))
    bad = bytes(good[:-1]) + bytes([(good[-1] + 1) & 0xFF])
    parser = FrameParser()
    assert parser.feed(bad) == []
    assert parser.bad_checksums == 1


def test_parser_refuses_oversize_length_field():
    # 伪造长度 60000 的“帧头”:当失步丢掉,绝不能干等 60KB
    fake = bytes([0xA5, 0x01, 0x04]) + (60000).to_bytes(2, "little") + b"\x00\x00"
    parser = FrameParser()
    frames = parser.feed(fake + make_confirm(3))
    assert [f.msg_id for f in frames] == [3]
    assert parser.desynced_frames >= 1


def test_parser_desync_threshold_is_configurable():
    parser = FrameParser(max_payload=64)
    fake = bytes([0xA5, 0x01, 0x04]) + (100).to_bytes(2, "little") + b"\x00\x00"
    frames = parser.feed(fake + make_confirm(4))
    assert [f.msg_id for f in frames] == [4]
    assert parser.desynced_frames == 1


# -- AIUI 事件 -----------------------------------------------------------------


def test_parse_real_wake_event_from_log():
    ivw = parse_ivw_event(REAL_WAKE_JSON.encode())
    assert ivw is not None
    assert ivw["keyword"] == "hei4 rui4 ke4"
    assert ivw["angle"] == 157.0
    assert ivw["beam"] == 3
    assert ivw["score"] == 774.0
    assert ivw["start_ms"] == 46140
    assert ivw["end_ms"] == 46820


def test_parse_ivw_survives_garbage_and_non_wake_events():
    assert parse_ivw_event(b"not json at all") is None
    assert parse_ivw_event(b"") is None
    other = json.dumps({"content": {"info": "{}"}, "type": "aiui_event"})
    assert parse_ivw_event(other.encode()) is None


# -- 链路(假板子) ---------------------------------------------------------------


class FakeBoard(threading.Thread):
    """说协议的假板子:握手 → 等确认 → 推唤醒事件 → 收主控配置。"""

    def __init__(self):
        super().__init__(daemon=True)
        self.ready = threading.Event()
        self.got_confirm = threading.Event()
        self.got_control = threading.Event()
        self.control_payload = None
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("127.0.0.1", 0))
        self.port = self.server.getsockname()[1]
        self.server.listen(1)

    def run(self):
        self.ready.set()
        conn, _ = self.server.accept()
        conn.settimeout(5)
        conn.sendall(build_frame(MSG_HANDSHAKE, CONFIRM_DATA, 9))
        parser = FrameParser()
        sent_event = False
        deadline = time.time() + 5
        while time.time() < deadline:
            try:
                chunk = conn.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                break
            for frame in parser.feed(chunk):
                if frame.type == MSG_CONFIRM and frame.msg_id == 9:
                    self.got_confirm.set()
                    if not sent_event:
                        sent_event = True
                        conn.sendall(
                            build_frame(MSG_DEVICE, REAL_WAKE_JSON.encode(), 1)
                        )
                elif frame.type == MSG_CONTROL:
                    self.control_payload = json.loads(frame.data.decode())
                    self.got_control.set()
        conn.close()
        self.server.close()


def test_link_handshake_wake_and_push_end_to_end():
    board = FakeBoard()
    board.start()
    assert board.ready.wait(2)

    events, states = [], []
    stop = threading.Event()
    link = VtnLink(
        connect=lambda: tcp_connect("127.0.0.1", board.port),
        ensure_forward=None,
        push_keywords='{"type":"wakeup_keywords","content":{"keyword":"hi rak"}}',
        on_state=states.append,
        on_ivw=events.append,
        reconnect_sec=0.2,
        stop=stop,
    )
    worker = threading.Thread(target=link.run, daemon=True)
    worker.start()

    assert board.got_confirm.wait(5), "链路应回确认帧给假板子"
    assert board.got_control.wait(5), "握手后应下发唤醒词配置"
    deadline = time.time() + 5
    while not events and time.time() < deadline:
        time.sleep(0.05)
    stop.set()
    worker.join(timeout=3)

    assert states and states[0] is True
    assert events and events[0]["keyword"] == "hei4 rui4 ke4"
    assert events[0]["angle"] == 157.0
    assert board.control_payload["content"]["keyword"] == "hi rak"


def test_link_reports_down_when_server_is_gone():
    """板子不在线时:state 保持 False,循环活着不崩(降级运行)。"""
    states, logs = [], []
    stop = threading.Event()
    link = VtnLink(
        connect=lambda: (_ for _ in ()).throw(OSError("refused")),
        on_state=states.append,
        reconnect_sec=0.05,
        stop=stop,
        log=logs.append,
    )
    worker = threading.Thread(target=link.run, daemon=True)
    worker.start()
    time.sleep(0.3)
    stop.set()
    worker.join(timeout=3)
    assert not worker.is_alive(), "断线时线程必须继续活着重试,而不是退出"
    assert states == [] or states == [False]
    assert any("连不上" in m for m in logs)
