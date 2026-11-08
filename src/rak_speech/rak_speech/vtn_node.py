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
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from rak_interfaces.msg import WakeEvent
from rak_lifecycle.managed_node import ManagedNode

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


class VtnNode(ManagedNode):
    """唤醒桥接节点，按 ADR-0020 §5 拆分。

    链路宿主（参数 + 回调 + stop 事件）在 `on_configure` 建好，真正的 socket 在
    `on_activate` 才由工作线程打开、`on_deactivate` 立刻关掉——板子掉线只是唤醒
    降级，deactivate 之后一个字节都不该再进 `/speech/wake`。
    """

    def __init__(self) -> None:
        super().__init__("speech_vtn")

        # 只做字段初始化：发布器、链路、线程都归 configure/activate（ADR-0020 §5）。
        self._host = "127.0.0.1"
        self._port = DEFAULT_LOCAL_PORT
        self._manage_forward = True
        self._remote_socket = DEFAULT_REMOTE_SOCKET
        self._adb_serial = ""
        self._push_keywords = ""
        self._reconnect_sec = 3.0

        self._wake_pub = None
        self._ready_pub = None
        self._stop = threading.Event()
        self._link = None
        self._worker = None
        self._link_up = False
        self._last_log = ("", 0.0)
        self._wakes = 0
        self._started_at = 0.0

    # ---------------------------------------------------------------- configure
    def on_configure(self, state) -> TransitionCallbackReturn:
        super().on_configure(state)

        wake_topic = str(self.declare("wake_topic", "/speech/wake"))
        ready_topic = str(self.declare("ready_topic", "/speech/vtn_ready"))
        self._host = str(self.declare("host", "127.0.0.1"))
        self._port = int(self.declare("local_port", DEFAULT_LOCAL_PORT))
        self._manage_forward = bool(self.declare("manage_forward", True))
        self._remote_socket = str(self.declare("remote_socket", DEFAULT_REMOTE_SOCKET))
        self._adb_serial = str(self.declare("adb_serial", ""))
        self._push_keywords = str(self.declare("push_keywords", ""))
        self._reconnect_sec = float(self.declare("reconnect_sec", 3.0))

        self._wake_pub = self.lifecycle_publisher(WakeEvent, wake_topic, 10)
        # 链路状态是“最后已知事实”,新加入的订阅者应立刻拿到 → latched
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._ready_pub = self.lifecycle_publisher(Bool, ready_topic, latched)

        try:
            # 链路宿主在这里建好（端口/隧道/唤醒词配置都在构造期定下）;
            # socket 本身在 activate 的工作线程里才连——未激活不碰硬件。
            self._link = self._make_link()
        except Exception as exc:  # noqa: BLE001 -- configure 失败必须带可读原因
            self._link = None
            return self.fail(f"链路初始化失败: {exc}")

        self.report(
            "link",
            DiagnosticStatus.WARN,
            "configured, not activated",
            {"host": self._host, "port": self._port},
        )
        return TransitionCallbackReturn.SUCCESS

    # ----------------------------------------------------------------- activate
    def on_activate(self, state) -> TransitionCallbackReturn:
        ret = super().on_activate(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret
        if self._link is None:
            # deactivate 丢弃过链路宿主（它的 stop 事件绑在构造期），这里重建。
            self._link = self._make_link()
        self._wakes = 0
        self._started_at = time.monotonic()
        self._last_log = ("", 0.0)
        self.publish(Bool(data=self._link_up), publisher=self._ready_pub)
        self._worker = threading.Thread(target=self._link.run, name="vtn", daemon=True)
        self._worker.start()
        self.add_timer(10.0, self._stats)
        return TransitionCallbackReturn.SUCCESS

    # --------------------------------------------------------------- deactivate
    def on_deactivate(self, state) -> TransitionCallbackReturn:
        # 先关链路再交给基类停 timer / 停发布器：链路线程不该在禁用之后还发一帧。
        self._stop_link()
        return super().on_deactivate(state)

    # ------------------------------------------------------------------ cleanup
    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self._stop_link()
        return super().on_cleanup(state)

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        self._stop_link()
        return super().on_shutdown(state)

    def _make_link(self) -> VtnLink:
        return VtnLink(
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

    def _stop_link(self) -> None:
        if self._link is not None:
            self._link.stop()
        self._stop.set()
        worker, self._worker = self._worker, None
        if worker is not None and worker.is_alive():
            worker.join(timeout=2.0)
        self._link = None
        self._stop = threading.Event()
        self._link_up = False

    # -- 链路回调(工作线程) ----------------------------------------------------

    def _ensure_forward(self) -> tuple[bool, str]:
        return ensure_adb_forward(
            serial=self._adb_serial,
            local_port=self._port,
            remote_socket=self._remote_socket,
        )

    def _on_state(self, up: bool) -> None:
        self._link_up = up
        # 未激活时 `publish()` 返回 False（基类语义），这里不需要额外判状态。
        self.publish(Bool(data=up), publisher=self._ready_pub)
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
        self.publish(msg, publisher=self._wake_pub)
        self._wakes += 1
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

    def _stats(self) -> None:
        """周期性统计：链路上线状态 + 每分钟唤醒次数。"""
        elapsed = time.monotonic() - self._started_at
        per_min = (self._wakes / elapsed * 60.0) if elapsed > 0 else 0.0
        self.report(
            "link",
            DiagnosticStatus.OK if self._link_up else DiagnosticStatus.WARN,
            "up" if self._link_up else "down (wake unavailable)",
            {
                "up": str(self._link_up),
                "wakes_per_min": f"{per_min:.1f}",
                "wakes": self._wakes,
                "port": self._port,
            },
        )

    def destroy_node(self) -> bool:
        self._stop_link()
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
