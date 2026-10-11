"""R818 降噪板链路:adb 隧道维护、socket 重连、帧分发(无 ROS 依赖)。

板上 demo 只监听 unix socket `/tmp/vtn.sock`,主机经
`adb forward tcp:<port> localfilesystem:/tmp/vtn.sock` 访问它(udev 规则
51-iflytek-adb.rules 保证免密)。链路任何一层断掉(板子 USB 掉线、adb
server 重启、demo 重启)都必须自己爬起来——这是车载节点,不是脚本。

回调都发生在 `run()` 所在的工作线程里;调用方(节点)负责线程安全。
"""

from __future__ import annotations

import shutil
import socket
import subprocess
import threading
from typing import Callable

from .vtn_protocol import (
    MSG_CONFIRM,
    MSG_CONTROL,
    MSG_DEVICE,
    MSG_HANDSHAKE,
    FrameParser,
    make_confirm,
    make_control,
    parse_ivw_event,
)

DEFAULT_LOCAL_PORT = 9100
DEFAULT_REMOTE_SOCKET = "/tmp/vtn.sock"


def ensure_adb_forward(
    serial: str = "",
    local_port: int = DEFAULT_LOCAL_PORT,
    remote_socket: str = DEFAULT_REMOTE_SOCKET,
    timeout: float = 5.0,
) -> tuple[bool, str]:
    """确保隧道存在,已存在同规格则复用。返回 (ok, 说明)。"""
    adb = shutil.which("adb")
    if not adb:
        return False, "adb 不在 PATH 里"
    base = [adb] + (["-s", serial] if serial else [])
    want_tcp = f"tcp:{local_port}"
    want_local = f"localfilesystem:{remote_socket}"
    try:
        listed = subprocess.run(
            base + ["forward", "--list"],
            capture_output=True, text=True, timeout=timeout,
        )
        if listed.returncode == 0:
            for line in listed.stdout.splitlines():
                parts = line.split()
                if len(parts) == 3 and parts[1] == want_tcp and parts[2] == want_local:
                    return True, "隧道已在"
        # 端口可能被别的规格占着:先撤再建(撤失败无所谓,看下一步)
        subprocess.run(
            base + ["forward", "--remove", want_tcp],
            capture_output=True, text=True, timeout=timeout,
        )
        added = subprocess.run(
            base + ["forward", want_tcp, want_local],
            capture_output=True, text=True, timeout=timeout,
        )
        if added.returncode == 0:
            return True, "隧道已建立"
        return False, (added.stderr or added.stdout).strip() or "adb forward 失败"
    except (OSError, subprocess.SubprocessError) as exc:
        return False, f"adb 调用失败: {exc}"


class VtnLink:
    """协议链路的宿主循环。

    connect(): 返回一个已连接 socket 的可调用(测试注入假连接);
    ensure_forward(): 每次连接前调用,返回 (ok, detail),None 表示不管隧道;
    on_state(up): 链路就绪/断开(就绪 = 收到板子握手帧);
    on_ivw(fields): 一条唤醒事件(已从 AIUI JSON 解出);
    on_other(type, data): 其它设备消息(原始记录用)。
    """

    def __init__(
        self,
        *,
        connect: Callable[[], socket.socket],
        ensure_forward: Callable[[], tuple[bool, str]] | None = None,
        push_keywords: str = "",
        on_state: Callable[[bool], None] | None = None,
        on_ivw: Callable[[dict], None] | None = None,
        on_other: Callable[[int, bytes], None] | None = None,
        reconnect_sec: float = 3.0,
        stop: threading.Event | None = None,
        log: Callable[[str], None] | None = None,
    ) -> None:
        self._connect = connect
        self._ensure_forward = ensure_forward
        self._push_keywords = push_keywords
        self._on_state = on_state or (lambda up: None)
        self._on_ivw = on_ivw or (lambda fields: None)
        self._on_other = on_other or (lambda t, d: None)
        self._reconnect_sec = reconnect_sec
        self._stop = stop or threading.Event()
        self._log = log or (lambda msg: None)

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        """一直跑到 stop();每次断线都重来,不把错误抛给调用方。"""
        while not self._stop.is_set():
            if self._ensure_forward is not None:
                ok, detail = self._ensure_forward()
                if not ok:
                    self._log(f"adb 隧道不可用: {detail}")
                    self._stop.wait(self._reconnect_sec)
                    continue
            try:
                sock = self._connect()
            except OSError as exc:
                self._log(f"连不上降噪板: {exc}")
                self._stop.wait(self._reconnect_sec)
                continue
            try:
                self._serve(sock)
            except OSError as exc:
                self._log(f"降噪板链路断开: {exc}")
            finally:
                try:
                    sock.close()
                except OSError:
                    pass
                self._on_state(False)
            self._stop.wait(self._reconnect_sec)

    # -- 单条连接内的事件循环 ------------------------------------------------

    def _serve(self, sock: socket.socket) -> None:
        sock.settimeout(1.0)
        parser = FrameParser()
        out_id = 0
        up = False
        pushed = False
        while not self._stop.is_set():
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                raise OSError("连接被对端关闭")
            for frame in parser.feed(chunk):
                if frame.type == MSG_HANDSHAKE:
                    sock.sendall(make_confirm(frame.msg_id))
                    if not up:
                        up = True
                        self._on_state(True)
                        self._log("与降噪板握手成功")
                    if self._push_keywords and not pushed:
                        out_id = (out_id + 1) & 0xFFFF
                        sock.sendall(make_control(self._push_keywords, out_id))
                        pushed = True
                        self._log(f"已下发唤醒词配置: {self._push_keywords}")
                elif frame.type == MSG_DEVICE:
                    ivw = parse_ivw_event(frame.data)
                    if ivw is not None:
                        self._on_ivw(ivw)
                    else:
                        self._on_other(frame.type, frame.data)
                else:
                    self._on_other(frame.type, frame.data)
        if parser.bad_checksums:
            self._log(f"坏帧(校验失败)累计 {parser.bad_checksums}")


def tcp_connect(
    host: str = "127.0.0.1",
    port: int = DEFAULT_LOCAL_PORT,
    timeout: float = 5.0,
) -> socket.socket:
    """默认连接方式:走本机 adb 隧道端口(或任何 TCP 到达的板子代理)。"""
    return socket.create_connection((host, port), timeout=timeout)
