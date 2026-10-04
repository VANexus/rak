#!/usr/bin/env python3
# [AI] A/B 板 USB 串口通信测试/驱动工具（R12 规格: 921600, 100Hz）
# 命令帧 8B: >hhh B CRC8  (vx/vy/wz 大端 int16, enable, CRC8 poly0x07 init0)
# 回读帧 8B: <hhh B CRC8  (0x200 布局: vx/vy/wz 小端 int16 + byte6=counter<<4|flags)
# 用法:
#   python serial_100hz.py COM5                    # 100Hz 空令牌(enable=0)链路测试 2s
#   python serial_100hz.py COM5 --sec 5            # 测 5 秒
#   python serial_100hz.py COM5 --vx 200 --enable 1  # 以 200mm/s 前进(确认架空/落地安全后再用!)
# Ctrl+C 退出, 退出前自动发 enable=0 刹停指令。
import argparse, sys, time
import serial
import serial.tools.list_ports

def list_ports():
    # [AI] 教训: Jetson 上 /dev/ttyUSB0 可能是激光雷达! 先 list 再动手
    for p in serial.tools.list_ports.comports():
        print(f'{p.device}  |  {p.description}  |  {p.hwid}')

def crc8(buf, init=0):
    crc = init
    for b in buf:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc

def cmd_frame(vx, vy, wz, enable):
    f = bytearray(8)
    f[0:2] = (vx & 0xFFFF).to_bytes(2, 'big')
    f[2:4] = (vy & 0xFFFF).to_bytes(2, 'big')
    f[4:6] = (wz & 0xFFFF).to_bytes(2, 'big')
    f[6] = enable & 1
    f[7] = crc8(f[:7])
    return bytes(f)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('port', nargs='?', help='如 /dev/ttyUSB1 或 COM5; CH9102 是"A 板串口1/3", 别选雷达口!')
    ap.add_argument('--list', action='store_true', help='列出所有串口及其身份')
    ap.add_argument('--baud', type=int, default=921600)
    ap.add_argument('--rate', type=int, default=100)
    ap.add_argument('--sec', type=float, default=2.0)
    ap.add_argument('--vx', type=int, default=0, help='mm/s')
    ap.add_argument('--vy', type=int, default=0, help='mm/s')
    ap.add_argument('--wz', type=int, default=0, help='0.01 rad/s')
    ap.add_argument('--enable', type=int, default=0, choices=(0, 1))
    a = ap.parse_args()
    if a.list or not a.port:
        list_ports()
        if not a.port:
            print('\n用法: serial_100hz.py <port> [--baud 921600] ...')
        return

    s = serial.Serial(a.port, a.baud, timeout=0.02)
    print(f'{a.port} @ {a.baud}  发 {a.rate}Hz  vx={a.vx} vy={a.vy} wz={a.wz} enable={a.enable}')
    print('---- 0x200 回读 (vx/vy/wz mm/s,0.01rad/s | flags) ----')

    frame = cmd_frame(a.vx, a.vy, a.wz, a.enable)
    period = 1.0 / a.rate
    rx_bytes = bytearray()
    n_rx = 0
    t0 = time.perf_counter()
    t_next = t0
    last_show = t0
    try:
        while True:
            now = time.perf_counter()
            if now - t0 >= a.sec:
                break
            if now >= t_next:
                s.write(frame)
                t_next += period
                if t_next < now:            # 落后了就重排, 不追帧
                    t_next = now + period
            rx_bytes.extend(s.read(64))
            while len(rx_bytes) >= 8:
                pkt = bytes(rx_bytes[:8]); del rx_bytes[:8]
                if crc8(pkt[:7]) != pkt[7]:
                    continue                # CRC 坏帧丢弃
                n_rx += 1
                if now - last_show >= 0.25:
                    vx, vy, wz = (int.from_bytes(pkt[i:i+2], 'little', signed=True) for i in (0, 2, 4))
                    cnt, flags = pkt[6] >> 4, pkt[6] & 0xF
                    print(f'  vx={vx:+5d} vy={vy:+5d} wz={wz:+5d}  '
                          f'cnt={cnt} flags={flags}  rx={n_rx}帧/{now-t0:.1f}s')
                    last_show = now
    except KeyboardInterrupt:
        pass
    dt = time.perf_counter() - t0
    s.write(cmd_frame(0, 0, 0, 0))          # 刹停令
    s.close()
    print(f'---- {n_rx} 帧 / {dt:.2f}s = {n_rx/dt:.1f}Hz 回读, 已发刹停 ----')

if __name__ == '__main__':
    main()
