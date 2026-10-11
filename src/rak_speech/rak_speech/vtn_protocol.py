"""R818 降噪板帧协议编解码(纯函数,无 ROS 依赖,可独立测试)。

协议出自《3_协议手册-R818降噪板.pdf》,与板上 demo(vtn)逐字节实测一致
(历史探测脚本见 git 历史中的 acoustic/scripts/vtn_probe.py):

    帧 = A5 | 用户ID(01) | 类型 | 长度(LE u16) | 消息ID(LE u16) | 数据 | 校验

    校验码 = (~(前面所有字节之和 & 0xFF) + 1) & 0xFF   (即补码)

类型:
    0x01  握手请求  板 → 主机;必须回 0xFF 确认,消息ID沿用对方的
    0x04  设备消息  板 → 主机;数据是 AIUI 事件 JSON,唤醒在 content.info.ivw
    0x05  主控消息  主机 → 板;数据是控制 JSON(wakeup_keywords/manual_wakeup/…)
    0xFF  确认消息  主机 → 板

板只在唤醒时上报一次方位角(固件行为),没有持续方位流。
"""

from __future__ import annotations

import json
from dataclasses import dataclass

SYNC = 0xA5
USER_ID = 0x01

MSG_HANDSHAKE = 0x01
MSG_DEVICE = 0x04
MSG_CONTROL = 0x05
MSG_CONFIRM = 0xFF

#: 确认消息的数据域(协议手册固定值,实测一致)。
CONFIRM_DATA = bytes([0xA5, 0x00, 0x00, 0x00])

#: 单帧数据域上限。事件 JSON 只有几百字节;失步时伪造的“长度”可能高达
#: u16 上限,若照单全收会把解析器饿死在一个永远凑不齐的帧上,所以宁可
#: 把超限帧当失步丢掉(板子目前没有任何接近该上限的消息类型)。
MAX_PAYLOAD = 8192

#: 接收缓冲上限 = 最大帧 + 余量;超过即强制失步恢复。
MAX_BUFFER = MAX_PAYLOAD + 8 + 4096


def checksum(body: bytes) -> int:
    """协议校验码:除校验字节外求和 → 取反 → 加 1。"""
    return (~(sum(body) & 0xFF) + 1) & 0xFF


def build_frame(msg_type: int, data: bytes, msg_id: int) -> bytes:
    """按协议拼一帧(主机侧发送方向:控制 / 确认)。"""
    body = (
        bytes([SYNC, USER_ID, msg_type])
        + len(data).to_bytes(2, "little")
        + (msg_id & 0xFFFF).to_bytes(2, "little")
        + data
    )
    return body + bytes([checksum(body)])


def make_confirm(msg_id: int) -> bytes:
    """回应板子握手请求的确认帧(消息ID必须与对方相同)。"""
    return build_frame(MSG_CONFIRM, CONFIRM_DATA, msg_id)


def make_control(payload: str | dict, msg_id: int) -> bytes:
    """主控消息:发给板子的 JSON 指令。"""
    if not isinstance(payload, str):
        payload = json.dumps(payload, ensure_ascii=False)
    return build_frame(MSG_CONTROL, payload.encode("utf-8"), msg_id)


@dataclass(frozen=True)
class Frame:
    """一帧解析结果。"""

    type: int
    msg_id: int
    data: bytes


class FrameParser:
    """流式解析:feed(字节块) → 完整帧列表。

    - 失步自愈:同步头/用户ID 对不上的字节直接丢弃并计数;
    - 超长“帧”按失步处理,不让解析器卡死;
    - 校验失败的帧丢弃并计数(坏帧是链路质量问题,不是异常路径)。
    """

    def __init__(self, max_payload: int = MAX_PAYLOAD) -> None:
        self._buf = bytearray()
        self._max_payload = max_payload
        self.dropped_bytes = 0  # 为找同步头丢掉的垃圾字节
        self.bad_checksums = 0
        self.desynced_frames = 0  # 头部合法但长度离谱的伪帧

    @property
    def buffered(self) -> int:
        return len(self._buf)

    def feed(self, chunk: bytes) -> list[Frame]:
        self._buf += chunk
        frames: list[Frame] = []
        while True:
            buf = self._buf
            if len(buf) < 8:  # 帧头都凑不齐
                break
            if buf[0] != SYNC or buf[1] != USER_ID:
                # 找下一个可能的同步头(同时校验用户ID,假同步概率 1/65536)
                idx = self._find_sync(1)
                drop = idx if idx != -1 else len(buf)
                self.dropped_bytes += drop
                del self._buf[:drop]
                continue
            length = int.from_bytes(buf[3:5], "little")
            if length > self._max_payload:
                self.desynced_frames += 1
                del self._buf[:1]  # 伪帧头,丢掉 A5 重新找同步
                continue
            total = 8 + length
            if len(buf) < total:
                if len(self._buf) <= self._max_payload + 8:
                    break  # 正常的“帧没收全”,等下一批字节
                # 缓冲超限还凑不齐 → 同步头是假的
                self.desynced_frames += 1
                del self._buf[:1]
                continue
            frame_bytes = bytes(buf[:total])
            del self._buf[:total]
            if checksum(frame_bytes[:-1]) != frame_bytes[-1]:
                self.bad_checksums += 1
                continue
            frames.append(
                Frame(
                    type=frame_bytes[2],
                    msg_id=int.from_bytes(frame_bytes[5:7], "little"),
                    data=frame_bytes[7:-1],
                )
            )
        return frames

    def _find_sync(self, start: int) -> int:
        buf = self._buf
        for i in range(start, len(buf) - 1):
            if buf[i] == SYNC and buf[i + 1] == USER_ID:
                return i
        # 末尾字节恰好是 A5 时留着,下一批字节能看到它的用户ID
        if buf and buf[-1] == SYNC:
            return len(buf) - 1
        return -1


def parse_ivw_event(data: bytes) -> dict | None:
    """设备消息(0x04)数据 → ivw 字段;非唤醒事件或损坏 → None。

    损坏的 JSON 是正常路径(链路上什么字节都可能出现),不是异常——
    调用方按“无唤醒”处理即可。
    """
    try:
        outer = json.loads(data.decode("utf-8", "ignore"))
        info = outer.get("content", {}).get("info")
        if isinstance(info, str):  # info 是转义过一层的 JSON 字符串
            info = json.loads(info)
        ivw = (info or {}).get("ivw")
    except (AttributeError, ValueError, TypeError):
        return None
    return dict(ivw) if isinstance(ivw, dict) else None
