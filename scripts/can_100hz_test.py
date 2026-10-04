#!/usr/bin/env python3
"""
CAN v3 100Hz 链路测试（上位机侧 · rak 工作区）

用法:
    python3 can_100hz_test.py [接口=can0] [秒数=8]

前置（需 sudo；本项目禁用 CAN FD，绝不要带 "fd on"）:
    sudo ip link set can0 type can bitrate 1000000
    sudo ip link set can0 up
    # 建议先自环验证控制器与驱动栈（不接总线）:
    #   sudo ip link set can0 down
    #   sudo ip link set can0 type can bitrate 1000000 loopback on
    #   sudo ip link set can0 up
    # 自环通过后再切回正常模式接真实总线

行为:
    - 以 100 Hz 发送 0x100 CHASSIS_VEL_CMD (vx=vy=wz=0, enable=1,
      计数器滚动, CRC8 poly 0x07 初值=DataID)
    - 统计接收: 0x200 CHASSIS_VEL_FB (目标 100Hz) / 0x030 互锁 (1kHz) /
      0x230 心跳 (1Hz) —— 速率、CRC 校验、到达间隔
协议真源: docs/PROTOCOL.md (CAN v3, 小端, DLC=8, byte6=计数|flags)
"""
import socket
import struct
import sys
import time

IFACE = sys.argv[1] if len(sys.argv) > 1 else "can0"
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
ENABLE = 1  # 链路级联调：enable=1 + 零速（安全，不产生运动）

# 接口状态前置检查：未 UP 时给出明确指引（bind 不会拦截 DOWN 接口）
try:
    _flags = int(open("/sys/class/net/%s/flags" % IFACE).read().strip(), 16)
except OSError:
    print("!! 接口 %s 不存在（检查名称/驱动）" % IFACE)
    sys.exit(2)
if not (_flags & 0x1):
    print("!! 接口 %s 处于 DOWN。需先执行（sudo；本项目禁 CAN FD，勿带 fd）:" % IFACE)
    print("   sudo ip link set %s type can bitrate 1000000" % IFACE)
    print("   sudo ip link set %s up" % IFACE)
    sys.exit(2)


def crc8(data: bytes, init: int) -> int:
    c = init
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def pack(can_id: int, payload: bytes) -> bytes:
    return struct.pack("=IB3x8s", can_id, len(payload), payload)


sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
try:
    sock.bind((IFACE,))
except OSError as e:
    print("!! 接口未就绪: bind(%s) 失败: %s" % (IFACE, e))
    print("   请先（需 sudo，禁带 fd）:")
    print("   sudo ip link set %s type can bitrate 1000000 && sudo ip link set %s up" % (IFACE, IFACE))
    sys.exit(2)
sock.setblocking(False)

rx200, rx200_badcrc, rx030, rx230, rx_other = [], 0, 0, 0, 0
sent = 0
cnt = 0
t0 = time.perf_counter()
next_t = t0

while True:
    now = time.perf_counter()
    if now - t0 >= SECS:
        break

    if now >= next_t:
        body = struct.pack("<hhh", 0, 0, 0) + bytes([((cnt & 0xF) << 4) | ENABLE])
        try:
            sock.send(pack(0x100, body + bytes([crc8(body, 0x06)])))
        except OSError as e:
            print("!! 发送失败: %s —— 接口可能刚被 down 或被占用" % e)
            sys.exit(2)
        sent += 1
        cnt += 1
        next_t += 0.010
        if next_t < now:
            next_t = now + 0.010  # 防追帧

    try:
        while True:
            cid, dlc, data = struct.unpack("=IB3x8s", sock.recv(16))
            cid &= 0x1FFFFFFF
            if cid == 0x200:
                rx200.append((time.perf_counter(), data))
                if crc8(data[:7], 0x09) != data[7]:
                    rx200_badcrc += 1
            elif cid == 0x030:
                rx030 += 1
            elif cid == 0x230:
                rx230 += 1
            else:
                rx_other += 1
    except BlockingIOError:
        pass
    time.sleep(0.0005)

el = time.perf_counter() - t0
print("== CAN v3 链路测试 @%s, %.2fs ==" % (IFACE, el))
print("发送 0x100 : %d 帧, %.1f Hz (目标 100)" % (sent, sent / el))
print("接收 0x200 : %d 帧, %.1f Hz (目标 100), CRC 错误 %d"
      % (len(rx200), len(rx200) / el, rx200_badcrc))
print("接收 0x030 : %d 帧 (%.0f Hz), 0x230: %d 帧, 其它 ID: %d"
      % (rx030, rx030 / el, rx230, rx_other))
if len(rx200) >= 2:
    ts = [t for t, _ in rx200]
    iv = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
    print("0x200 间隔 : min %.1f / avg %.1f / max %.1f ms"
          % (min(iv) * 1000, sum(iv) / len(iv) * 1000, max(iv) * 1000))
    d = rx200[-1][1]
    vx, vy, wz = struct.unpack("<hhh", d[:6])
    print("0x200 末帧 : vx=%d mm/s  vy=%d  wz=%d x0.01rad/s  flags=0x%X  counter=%d"
          % (vx, vy, wz, d[6] & 0xF, d[6] >> 4))