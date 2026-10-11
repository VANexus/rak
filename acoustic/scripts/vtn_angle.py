#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VTN 降噪板 · 唤醒 / 声源角度 监听器
=====================================
作用：从板子的 /tmp/vtn.sock 收「唤醒事件」，抠出 angle（声源角度）。

前置（每次开机 / 重插 USB 之后都要做一次）：
    sudo chmod 666 /dev/bus/usb/001/097      # 设备号可能变，用 ls -l /dev/bus/usb/001/ 查
    adb forward tcp:9100 localfilesystem:/tmp/vtn.sock

用法：
    python3 vtn_angle.py
    看到「收到握手 → 已回确认」之后，对着板子大声说「小飞小飞」

协议（来自《3_协议手册-R818降噪板.pdf》，已逐字节实测验证）：
    帧 = 同步头 A5 | 用户ID 01 | 类型 | 长度(小端2B) | 消息ID(小端2B) | 数据 | 校验码
    类型：0x01 = 握手请求（设备发出，我们必须回 0xFF 确认）
          0x04 = 设备消息（AIUI 事件，唤醒 JSON 在这里）
          0x05 = 主控消息（我们发给板子的）
          0xFF = 确认消息
    校验码 = (~(前面所有字节之和 & 0xFF) + 1) & 0xFF
"""
import socket
import time
import json
import re

HOST, PORT = '127.0.0.1', 9100
RUN_SECONDS = 120        # 监听时长（秒）


def checksum(buf):
    """协议规定的校验码：除校验字节外全部求和 → 取反 → 加 1"""
    return (~(sum(buf) & 0xFF) + 1) & 0xFF


def make_confirm(msg_id):
    """构造「确认消息」，用于回应对方的握手请求（消息ID 必须与对方相同）"""
    body = bytes([0xA5, 0x01, 0xFF, 0x04, 0x00])
    body += msg_id.to_bytes(2, 'little')
    body += bytes([0xA5, 0x00, 0x00, 0x00])
    return body + bytes([checksum(body)])


def parse_frame(frame):
    """拆一帧 → (消息类型, 消息ID, 数据)"""
    msg_type = frame[2]
    msg_id = int.from_bytes(frame[5:7], 'little')
    length = int.from_bytes(frame[3:5], 'little')
    return msg_type, msg_id, frame[7:7 + length]


def handle_aiui(data):
    """板子报唤醒：从 JSON 里抠 angle / beam / keyword"""
    text = data.decode('utf-8', 'ignore')
    print('    原文: %s' % text)
    try:
        outer = json.loads(text)
        info = outer.get('content', {}).get('info')
        if isinstance(info, str):            # info 里是转义过的 JSON 字符串
            info = json.loads(info)
        ivw = (info or {}).get('ivw', {})
        angle = ivw.get('angle')
        if angle is not None:
            print('    ★★★ 声源角度 angle = %s°   beam = %s   keyword = %s'
                  % (angle, ivw.get('beam'), ivw.get('keyword')))
            return angle
    except Exception:
        pass
    m = re.search(r'angle\\?"?\s*:\s*([0-9.]+)', text)   # 兜底：正则硬抠
    if m:
        print('    ★★★ 声源角度 angle = %s°' % m.group(1))
    return None


def main():
    print('连接 %s:%d ...' % (HOST, PORT))
    try:
        sock = socket.create_connection((HOST, PORT), timeout=5)
    except OSError as e:
        print('连不上:', e)
        print('八成是忘了开隧道，先跑：')
        print('    adb forward tcp:9100 localfilesystem:/tmp/vtn.sock')
        return
    sock.settimeout(1.0)
    print('已连上。 ** 现在对着板子大声说「小飞小飞」 ** （%d 秒内）\n' % RUN_SECONDS)

    buf = b''
    start = time.time()
    while time.time() - start < RUN_SECONDS:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            continue
        if not chunk:
            print('连接被断开')
            break
        buf += chunk
        while len(buf) >= 8:
            if buf[0] != 0xA5:               # 丢垃圾字节，重新找同步头
                buf = buf[1:]
                continue
            length = int.from_bytes(buf[3:5], 'little')
            total = 8 + length
            if len(buf) < total:             # 帧还没收全，等下一次
                break
            frame, buf = buf[:total], buf[total:]
            msg_type, msg_id, data = parse_frame(frame)
            stamp = time.time() - start

            if msg_type == 0x01:             # 握手请求 → 回确认
                sock.sendall(make_confirm(msg_id))
                print('[%5.1fs] 收到握手 → 已回确认' % stamp)
            elif msg_type == 0x04:           # 设备消息（唤醒）
                print('[%5.1fs] ★ 唤醒事件：' % stamp)
                handle_aiui(data)
            else:
                print('[%5.1fs] 类型=0x%02X 数据=%r' % (stamp, msg_type, data))

    sock.close()
    print('\n监听结束。')


if __name__ == '__main__':
    main()
