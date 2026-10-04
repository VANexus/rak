#!/usr/bin/env python3
"""上位机↔下位机 100Hz 串口链路测试（板载本地测试通道）

用法: python3 serial_100hz_test.py /dev/ttyUSBx [秒数=10]
下行: 8 字节命令帧 = 大端 int16 vx/vy/wz + byte6 enable + CRC8(poly7, 初值 0) @100Hz
上行: 8 字节反馈帧 = 小端 int16 vx/vy/wz + byte6 计数|flags + CRC8(初值 0x09) —— 即 0x200 布局
注意: 该通道为板载"本地测试"约定，与 CAN v3 的字节序不同（CAN 为小端）。
"""
import os
import struct
import sys
import termios
import time

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0


def crc8(p, init):
    c = init
    for b in p:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
attrs = termios.tcgetattr(fd)
cc = attrs[6]
cc[termios.VMIN] = 0
cc[termios.VTIME] = 0
cflag = termios.CS8 | termios.CREAD | termios.CLOCAL
termios.tcsetattr(fd, termios.TCSANOW,
                  [0, 0, cflag, 0, termios.B115200, termios.B115200, cc])

t0 = time.perf_counter()
next_t = t0
sent = 0
rxbuf = bytearray()
frames = []
slips = 0

while True:
    now = time.perf_counter()
    if now - t0 >= SECS:
        break
    if now >= next_t:
        body = struct.pack(">hhh", 0, 0, 0) + bytes([1])   # enable=1 + 零速（安全）
        os.write(fd, body + bytes([crc8(body, 0x00)]))
        sent += 1
        next_t += 0.010
        if next_t < now:
            next_t = now + 0.010
    try:
        d = os.read(fd, 512)
        if d:
            rxbuf += d
    except BlockingIOError:
        pass
    while len(rxbuf) >= 8:
        cand = bytes(rxbuf[:8])
        if crc8(cand[:7], 0x09) == cand[7]:
            frames.append((time.perf_counter(), cand))
            del rxbuf[:8]
        else:
            del rxbuf[:1]
            slips += 1
    time.sleep(0.0005)

el = time.perf_counter() - t0
print("== 串口 100Hz 测试 @%s, %.2fs ==" % (PORT, el))
print("下行 命令帧 : %d 帧, %.1f Hz (目标 100)" % (sent, sent / el))
print("上行 反馈帧 : %d 帧, %.1f Hz (目标 100), 失步滑字节 %d" % (len(frames), len(frames) / el, slips))
if len(frames) >= 2:
    ts = [t for t, _ in frames]
    iv = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
    print("反馈帧间隔 : min %.1f / avg %.1f / max %.1f ms"
          % (min(iv) * 1000, sum(iv) / len(iv) * 1000, max(iv) * 1000))
    d = frames[-1][1]
    vx, vy, wz = struct.unpack("<hhh", d[:6])
    print("末帧解码   : vx=%d mm/s vy=%d wz=%d x0.01rad/s flags=0x%X counter=%d"
          % (vx, vy, wz, d[6] & 0xF, d[6] >> 4))
os.close(fd)