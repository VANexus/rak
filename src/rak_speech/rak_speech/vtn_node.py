"""R818 降噪板桥接节点:唤醒事件 → /speech/wake,链路健康 → /speech/vtn_ready。

板子(RK3328,讯飞 VTN demo)只暴露 unix socket `/tmp/vtn.sock`,主机经
`adb forward` 隧道访问(协议见 vtn_protocol.py,链路与重连见 vtn_link.py)。
本节点把这两层包装成标准 ROS 2 接口:

- `/speech/wake`  rak_interfaces/WakeEvent —— 唤醒词 + 声源方位(板固件只在
  唤醒时报一次角度,没有持续方位流);
- `/speech/vtn_ready` std_msgs/Bool(latched)—— 链路是否握手成功。采集与
  识别链路不依赖它:板子掉线只是唤醒功能降级,不能把语音管线一起拖死。

参数 `push_keywords` 非空时,握手成功后向板子下发一次唤醒词配置(JSON,
如 {"type":"wakeup_keywords","content":{"keyword":"hi rak","threshold":"100"}});
默认空 = 不碰板上既有配置。
"""

from __future__ import annotations

import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from rak_interfaces.msg import WakeEvent

from .vtn_link import (
    DEFAULT_LOCAL_PORT,
    DEFAULT_REMOTE_SOCKET,
    VtnLink,
    ensure_adb_forward,
    tcp_connect,
)


def _f(value, default: float = 0.0) -> float:
    """板子 JSON 里什么标度都有;坏值按 0 处理,不让一条事件炸掉回调。"""
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _i(value, default: int = 0) -> int:
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


class VtnNode(Node):
    def __init__(self) -> None:
        super().__init__("speech_vtn")

        self.declare_parameter("wake_topic", "/speech/wake")
        self.declare_parameter("ready_topic", "/speech/vtn_ready")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("local_port", DEFAULT_LOCAL_PORT)
        self.declare_parameter("manage_forward", True)
        self.declare_parameter("remote_socket", DEFAULT_REMOTE_SOCKET)
        self.declare_parameter("adb_serial", "")
        self.declare_parameter("push_keywords", "")
        self.declare_parameter("reconnect_sec", 3.0)

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self._host = str(g("host"))
        self._port = int(g("local_port"))
        self._manage_forward = bool(g("manage_forward"))
        self._remote_socket = str(g("remote_socket"))
        self._adb_serial = str(g("adb_serial"))
        self._push_keywords = str(g("push_keywords"))
        self._reconnect_sec = float(g("reconnect_sec"))

        self._wake_pub = self.create_publisher(WakeEvent, str(g("wake_topic")), 10)
        # 链路状态是“最后已知事实”,新加入的订阅者应立刻拿到 → latched
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._ready_pub = self.create_publisher(Bool, str(g("ready_topic")), latched)
        self._ready_pub.publish(Bool(data=False))

        self._stop = threading.Event()
        self._link = VtnLink(
            connect=lambda: tcp_connect(self._host, self._port),
            ensure_forward=self._ensure_forward if self._manage_forward else None,
            push_keywords=self._push_keywords,
            on_state=self._on_state,
            on_ivw=self._on_ivw,
            on_other=self._on_other,
            reconnect_sec=self._reconnect_sec,
            stop=self._stop,
            log=self._log_throttled,
        )
        self._last_log = ("", 0.0)
        self._worker = threading.Thread(target=self._link.run, name="vtn", daemon=True)
        self._worker.start()

    # -- 链路回调(工作线程) ----------------------------------------------------

    def _ensure_forward(self) -> tuple[bool, str]:
        return ensure_adb_forward(
            serial=self._adb_serial,
            local_port=self._port,
            remote_socket=self._remote_socket,
        )

    def _on_state(self, up: bool) -> None:
        self._ready_pub.publish(Bool(data=up))
        if not up:
            self.get_logger().warning("vtn link down, wake unavailable")

    def _on_ivw(self, ivw: dict) -> None:
        msg = WakeEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.keyword = str(ivw.get("keyword", ""))
        msg.score = _f(ivw.get("score"))
        msg.angle = _f(ivw.get("angle"))
        msg.beam = _i(ivw.get("beam"))
        msg.physical = _i(ivw.get("physical"))
        msg.power = _f(ivw.get("power"))
        msg.start_ms = _i(ivw.get("start_ms"))
        msg.end_ms = _i(ivw.get("end_ms"))
        self._wake_pub.publish(msg)
        self.get_logger().info(
            f"wake: '{msg.keyword}' angle={msg.angle:.0f}° "
            f"beam={msg.beam} score={msg.score:.0f}"
        )

    def _on_other(self, msg_type: int, data: bytes) -> None:
        # 板子还会推非 ivw 的 AIUI 事件;先按原始帧留档,等有消费者再结构化
        self.get_logger().debug(f"vtn device msg type=0x{msg_type:02X} {data!r}")

    def _log_throttled(self, message: str) -> None:
        """断线重试每 reconnect_sec 一次,同内容 30 秒内只落一条日志。"""
        now = time.monotonic()
        last, at = self._last_log
        if message == last and now - at < 30.0:
            return
        self._last_log = (message, now)
        self.get_logger().warning(message)

    def destroy_node(self) -> bool:
        self._stop.set()
        if hasattr(self, "_link"):
            self._link.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VtnNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
