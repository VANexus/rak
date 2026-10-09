#!/usr/bin/env python3
"""[AI] CAN v4 冻结版全链路冒烟测试（Jetson 侧，SocketCAN）。

协议真源：../vehical_rak_chassis/docs/PROTOCOL.md v4（FROZEN）+ protocol-signals.yaml。
验证项（全程不发非零 DRIVE_CMD，车轮不转，可带轮跑）：
  1. NODE_INFO @1Hz，PROTO=4（版本握手，v4 §10.4）
  2. DIAG @10Hz 解码：BAT_MV / FAULT / STATE / MODE / BUSOFF_CNT
  3. SAFETY_CMD e2e 下行落地：START→STATE.bit1+MODE=REMOTE
  4. EMERGENCY_STOP→STATE.bit0+MODE=ESTOP（ISR 快速路径语义）
  5. CRC 篡改拒收：坏 CRC 急停帧不得改变状态（e2e 负例）
  6. CLEAR_ESTOP→bit0 清除（R50/R52 门：电池+非 FAULT）
  7. STOP→MODE=IDLE 收尾
用法：python3 can_link_smoke.py [can0]
退出码：0=全部通过，1=存在失败项。
"""
import socket
import struct
import sys
import time

ID_SAFETY = 0x010
ID_NODEINFO = 0x030
ID_DIAG = 0x220
DID_SAFETY = 0x01

CMD_START = 0x1
CMD_STOP = 0x2
CMD_ESTOP = 0x4
CMD_CLEAR_ESTOP = 0x5

CAN_FRAME_FMT = "=IB3x8s"


def crc8_e2e(data7: bytes, data_id: int = DID_SAFETY) -> int:
    """PROTOCOL v4 §2：poly 0x07、无反射、无最终异或、初值=Data ID、覆盖 byte0..6。"""
    crc = data_id
    for b in data7:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class CanLink:
    def __init__(self, channel: str):
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((channel,))
        self.sock.settimeout(0.05)
        self.counter = 0

    def send_safety(self, cmd: int, corrupt_crc: bool = False) -> None:
        d = bytearray(8)
        d[0] = cmd
        d[6] = (self.counter & 0x0F) << 4  # [7:4] alive counter，flags=0
        self.counter = (self.counter + 1) & 0x0F
        crc = crc8_e2e(d[:7])
        d[7] = (crc ^ 0xFF) if corrupt_crc else crc  # 篡改：翻转全部位
        frame = struct.pack(CAN_FRAME_FMT, ID_SAFETY, 8, bytes(d))
        self.sock.send(frame)

    def collect(self, seconds: float) -> dict:
        seen = {}
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            try:
                cid, dlc, data = struct.unpack(CAN_FRAME_FMT, self.sock.recv(16))
            except socket.timeout:
                continue
            except BlockingIOError:
                continue
            seen[cid] = data[:dlc]
        return seen

    def latest_diag(self, wait: float = 0.3) -> tuple:
        """等待并返回最新 DIAG： (bat_mv, fault, state, mode, busoff_cnt) 或 None。"""
        end = time.monotonic() + wait
        diag = None
        while time.monotonic() < end:
            frames = self.collect(end - time.monotonic())
            if ID_DIAG in frames:
                d = frames[ID_DIAG]
                diag = (
                    d[0] | (d[1] << 8),
                    d[2] | (d[3] << 8),
                    d[4] | (d[5] << 8),
                    d[6],
                    d[7],
                )
        return diag


def main() -> int:
    channel = sys.argv[1] if len(sys.argv) > 1 else "can0"
    results = []

    def check(name: str, ok: bool, detail: str = "") -> None:
        results.append((name, ok, detail))
        print(f"[{'PASS' if ok else 'FAIL'}] {name}{('  ' + detail) if detail else ''}")

    link = CanLink(channel)
    print(f"== CAN v4 全链路冒烟：channel={channel}（不发非零 DRIVE，车轮不转）==")

    # 1+2. 上行遥测
    frames = link.collect(1.3)  # 覆盖至少一个 NODE_INFO 周期（1Hz）
    node = frames.get(ID_NODEINFO)
    check("NODE_INFO @1Hz 可达", node is not None,
          f"ids={sorted(hex(c) for c in frames)}" if node is None else "")
    if node is not None:
        proto = node[0]
        check("NODE_INFO.PROTO=4（v4 节点身份）", proto == 4, f"proto={proto}")

    diag = link.latest_diag()
    check("DIAG @10Hz 可达且可解码", diag is not None)
    if diag is not None:
        bat, fault, state, mode, busoff = diag
        print(f"    DIAG: BAT={bat}mV FAULT=0x{fault:04X} STATE=0x{state:04X} "
              f"MODE={mode} BUSOFF={busoff}")
        check("电池电压 ≥10500mV（CLEAR 门前提）", bat >= 10500, f"bat={bat}mV")
        check("基线无故障、IDLE、无 bus-off", fault == 0 and state & 0x3 == 0
              and mode == 0 and busoff == 0,
              f"state=0x{state:04X} mode={mode}")

    # 3. START
    link.send_safety(CMD_START)
    diag = link.latest_diag()
    ok = diag is not None and (diag[2] & 0x2) != 0 and diag[3] == 1
    check("START→STATE.bit1 + MODE=REMOTE", ok,
          f"STATE=0x{diag[2]:04X} MODE={diag[3]}" if diag else "no DIAG")

    # 4. EMERGENCY_STOP
    link.send_safety(CMD_ESTOP)
    diag = link.latest_diag()
    ok = diag is not None and (diag[2] & 0x1) != 0 and diag[3] == 2
    check("EMERGENCY_STOP→STATE.bit0 + MODE=ESTOP", ok,
          f"STATE=0x{diag[2]:04X} MODE={diag[3]}" if diag else "no DIAG")

    # 5. CRC 篡改拒收（e2e 负例）：坏帧必须是 no-op
    link.send_safety(CMD_CLEAR_ESTOP, corrupt_crc=True)
    diag = link.latest_diag()
    ok = diag is not None and (diag[2] & 0x1) != 0
    check("坏 CRC 帧被拒收（estop 仍锁存）", ok,
          f"STATE=0x{diag[2]:04X}" if diag else "no DIAG")

    # 6. CLEAR_ESTOP（合法 e2e）
    link.send_safety(CMD_CLEAR_ESTOP)
    diag = link.latest_diag()
    ok = diag is not None and (diag[2] & 0x1) == 0
    check("CLEAR_ESTOP→急停锁存清除", ok,
          f"STATE=0x{diag[2]:04X} MODE={diag[3]}" if diag else "no DIAG")

    # 7. STOP 收尾（急停清除后 can_remote 已被 ESTOP 清掉，应回 IDLE）
    link.send_safety(CMD_STOP)
    diag = link.latest_diag()
    ok = diag is not None and diag[3] == 0 and (diag[2] & 0x3) == 0
    check("STOP→MODE=IDLE 收尾", ok,
          f"STATE=0x{diag[2]:04X} MODE={diag[3]}" if diag else "no DIAG")

    failed = [r for r in results if not r[1]]
    print(f"== 结果：{len(results) - len(failed)}/{len(results)} 通过 ==")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
