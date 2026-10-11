#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VTN 降噪板 · 主动探测 / 唤醒监听
==================================
在 vtn_angle.py 基础上加了「我们主动发指令」的能力。

用法:
    python3 vtn_probe.py                    # 只监听（对着板子说唤醒词）
    python3 vtn_probe.py --auto             # 自动探针:先问版本,3 秒后"手动唤醒"
    python3 vtn_probe.py --send '{"type":"verison"}'   # 握手后自动发一条主控消息

前置:
    adb forward tcp:9100 localfilesystem:/tmp/vtn.sock

协议（《3_协议手册-R818降噪板.pdf》，已实测验证）:
    帧 = A5 | 用户ID 01 | 类型 | 长度(小端2B) | 消息ID(小端2B) | 数据 | 校验码
    类型: 0x01 握手请求(设备→我们)   0x04 设备消息(设备→我们)
          0x05 主控消息(我们→设备)   0xFF 确认消息(我们→设备)
    校验码 = (~(前面所有字节之和 & 0xFF) + 1) & 0xFF
"""
import socket
import time
import json
import re
import argparse

HOST, PORT = '127.0.0.1', 9100
RUN_SECONDS = 120

MSG_HANDSHAKE = 0x01
MSG_DEVICE = 0x04
MSG_CONTROL = 0x05
MSG_CONFIRM = 0xFF


# ---------- 打包 ----------

def checksum(buf):
    """协议规定的校验码：除校验字节外全部求和 → 取反 → 加 1"""
    return (~(sum(buf) & 0xFF) + 1) & 0xFF


def build(msg_type, data, msg_id):
    """按协议拼一帧"""
    body = (bytes([0xA5, 0x01, msg_type])
            + len(data).to_bytes(2, 'little')
            + msg_id.to_bytes(2, 'little')
            + data)
    return body + bytes([checksum(body)])


def make_confirm(msg_id):
    """确认消息：回应对方的握手请求"""
    return build(MSG_CONFIRM, bytes([0xA5, 0x00, 0x00, 0x00]), msg_id)


def make_control(json_str, msg_id):
    """主控消息：我们发给板子的指令（JSON）"""
    return build(MSG_CONTROL, json_str.encode('utf-8'), msg_id)


# ---------- 解析 ----------

def show_aiui(data):
    """板子报唤醒 / AIUI 事件：抠出 angle"""
    text = data.decode('utf-8', 'ignore')
    print('    原文: %s' % text)
    try:
        outer = json.loads(text)
        info = outer.get('content', {}).get('info')
        if isinstance(info, str):          # info 里是转义过的 JSON 字符串
            info = json.loads(info)
        ivw = (info or {}).get('ivw', {})
        if ivw:
            print('    ★★★ angle=%s°   beam=%s   keyword=%s   score=%s'
                  % (ivw.get('angle'), ivw.get('beam'),
                     ivw.get('keyword'), ivw.get('score')))
            return ivw.get('angle')
    except Exception:
        pass
    m = re.search(r'angle\\?"?\s*:\s*([0-9.]+)', text)   # 兜底：正则硬抠
    if m:
        print('    ★★★ angle = %s°' % m.group(1))
        return float(m.group(1))
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--send', help='握手成功后自动发这条主控消息(JSON 字符串)')
    ap.add_argument('--auto', action='store_true',
                    help='自动探针：先问版本,再"手动唤醒"')
    args = ap.parse_args()

    # (延迟秒数, JSON) —— 以"握手成功"为计时零点
    plan = []
    if args.auto:
        plan = [(2.0, '{"type":"verison"}'),
                (5.0, '{"type":"manual_wakeup","content":{"beam":1}}')]
    if args.send:
        plan = [(2.0, args.send)]

    print('连接 %s:%d ...' % (HOST, PORT))
    try:
        sock = socket.create_connection((HOST, PORT), timeout=5)
    except OSError as e:
        print('连不上:', e)
        print('先跑:  adb forward tcp:9100 localfilesystem:/tmp/vtn.sock')
        return
    sock.settimeout(1.0)
    print('已连上。')

    buf = b''
    start = time.time()
    link_at = None          # 握手成功那一刻
    fired = set()
    my_id = 0

    while time.time() - start < RUN_SECONDS:
        # 到了计划时间就发指令
        if link_at is not None:
            for i, (delay, payload) in enumerate(plan):
                if i not in fired and (time.time() - link_at) >= delay:
                    fired.add(i)
                    my_id = (my_id + 1) & 0xFFFF
                    sock.sendall(make_control(payload, my_id))
                    print('\n[%5.1fs] >>> 我们发出主控消息(id=%d): %s'
                          % (time.time() - start, my_id, payload))
                    print('         等待板子回话...\n')

        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            continue
        if not chunk:
            print('连接被断开')
            break
        buf += chunk

        while len(buf) >= 8:
            if buf[0] != 0xA5:                 # 丢垃圾字节，重新找同步头
                buf = buf[1:]
                continue
            length = int.from_bytes(buf[3:5], 'little')
            total = 8 + length
            if len(buf) < total:               # 帧没收全，等下一次
                break
            frame, buf = buf[:total], buf[total:]
            msg_type = frame[2]
            msg_id = int.from_bytes(frame[5:7], 'little')
            data = frame[7:7 + length]
            stamp = time.time() - start

            if msg_type == MSG_HANDSHAKE:      # 握手请求 → 回确认
                sock.sendall(make_confirm(msg_id))
                if link_at is None:
                    link_at = time.time()
                    print('[%5.1fs] 握手成功 → 已回确认。现在对着板子说「小飞小飞」'
                          % stamp)
            elif msg_type == MSG_DEVICE:       # 设备消息（唤醒 / 事件）
                print('[%5.1fs] ★ 设备消息:' % stamp)
                show_aiui(data)
            else:
                print('[%5.1fs] 类型=0x%02X id=%d 数据=%r'
                      % (stamp, msg_type, msg_id, data))
                print('        hex: %s' % frame.hex())

    sock.close()
    print('\n监听结束。')


if __name__ == '__main__':
    main()
