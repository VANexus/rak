"""Exercise the actual ROS UART node against a pseudo-terminal F407 peer."""

import os
import pty
import select
import struct
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rak_interfaces.msg import BaseTelemetry
from rak_base_driver.node import BaseUartNode
from rak_base_driver.protocol import (
    FEATURE_BASE_TELEMETRY,
    FEATURE_BASE_VELOCITY,
    MODE_ARMED,
    STATE_ARMED,
    STATE_DISARMED,
    TYPE_BASE_STATE,
    TYPE_HELLO,
    TYPE_HELLO_ACK,
    TYPE_SET_BASE_VELOCITY,
    BaseState,
    Frame,
    StreamDecoder,
    VELOCITY,
    encode_frame,
    pack_base_state,
)
from std_srvs.srv import SetBool


class PtyF407Peer:
    def __init__(self, master_fd):
        self.master_fd = master_fd
        self.decoder = StreamDecoder()
        self.tx_sequence = 0
        self.commands = []
        self.errors = []
        self._lock = threading.Lock()
        self._running = True
        self._uptime_ms = 0
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def close(self):
        self._running = False
        self._thread.join(timeout=1.0)

    def _send(self, message_type, payload):
        packet = encode_frame(Frame(1, message_type, self.tx_sequence, payload))
        self.tx_sequence = (self.tx_sequence + 1) & 0xFFFF
        os.write(self.master_fd, packet)

    def _run(self):
        required = FEATURE_BASE_VELOCITY | FEATURE_BASE_TELEMETRY
        while self._running:
            readable, _, _ = select.select([self.master_fd], [], [], 0.02)
            if not readable:
                continue
            try:
                data = os.read(self.master_fd, 512)
            except OSError as exc:
                self.errors.append(str(exc))
                return
            for frame in self.decoder.feed(data):
                if frame.message_type == TYPE_HELLO:
                    if len(frame.payload) != 8:
                        self.errors.append("invalid HELLO length")
                        continue
                    self._send(
                        TYPE_HELLO_ACK,
                        struct.pack("<HIII", 1, 0x010000, 0x12345678, required),
                    )
                elif frame.message_type == TYPE_SET_BASE_VELOCITY:
                    try:
                        mode, lease_ms, vx, vy, wz = VELOCITY.unpack(frame.payload)
                    except struct.error:
                        self.errors.append("invalid velocity payload")
                        continue
                    with self._lock:
                        self.commands.append((mode, lease_ms, vx, vy, wz))
                    state = STATE_ARMED if mode == MODE_ARMED else STATE_DISARMED
                    self._uptime_ms += 10
                    self._send(
                        TYPE_BASE_STATE,
                        pack_base_state(
                            BaseState(frame.sequence, state, 0, vx, vy, wz, self._uptime_ms)
                        ),
                    )
                else:
                    self.errors.append(f"unexpected host type {frame.message_type}")

    def has_command(self, predicate):
        with self._lock:
            return any(predicate(command) for command in self.commands)

    def wait_command(self, predicate, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.has_command(predicate):
                return True
            time.sleep(0.01)
        return False


class UartProbe(Node):
    def __init__(self):
        super().__init__("base_uart_pty_probe")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.arm = self.create_client(SetBool, "/base_uart_driver/arm")
        self.telemetry = []
        self._condition = threading.Condition()
        self.create_subscription(BaseTelemetry, "/base/telemetry", self._on_telemetry, 10)

    def _on_telemetry(self, msg):
        with self._condition:
            self.telemetry.append(msg)
            self._condition.notify_all()

    def wait_telemetry(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        with self._condition:
            while time.monotonic() < deadline:
                match = next(
                    (msg for msg in reversed(self.telemetry) if predicate(msg)), None
                )
                if match is not None:
                    return match
                self._condition.wait(max(0.0, deadline - time.monotonic()))
        raise AssertionError("matching base telemetry was not received")


def test_uart_driver_defaults_to_disarmed_without_opening_port():
    rclpy.init()
    node = BaseUartNode()
    try:
        assert not node._allow_hardware
        assert not node._limits_valid
        assert node._ser is None
        assert not node._armed
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_uart_ros_node_handshake_command_and_stale_input_disarm():
    rclpy.init()
    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)
    port = os.ttyname(slave_fd)
    peer = PtyF407Peer(master_fd)
    peer.start()
    driver = BaseUartNode(parameter_overrides=[
        Parameter("port", value=port),
        Parameter("allow_hardware", value=True),
        Parameter("max_vx_m_s", value=0.5),
        Parameter("max_vy_m_s", value=0.5),
        Parameter("max_wz_rad_s", value=1.0),
        Parameter("command_timeout_sec", value=0.25),
        Parameter("telemetry_timeout_sec", value=0.3),
    ])
    probe = UartProbe()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(driver)
    executor.add_node(probe)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        assert probe.arm.wait_for_service(timeout_sec=3.0)
        healthy = probe.wait_telemetry(
            lambda msg: msg.connected and msg.handshake_ok and msg.valid
        )
        assert not healthy.armed
        assert peer.has_command(lambda item: item[0] == 0 and item[2:] == (0, 0, 0))

        twist = Twist()
        twist.linear.x = 0.1
        twist.linear.y = -0.05
        twist.angular.z = 0.2
        probe.cmd_pub.publish(twist)
        deadline = time.monotonic() + 1.0
        while driver._last_command_rx == 0.0 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert driver._last_command_rx > 0.0

        request = SetBool.Request()
        request.data = True
        response = probe.arm.call_async(request)
        arm_result = threading.Event()
        response.add_done_callback(lambda _: arm_result.set())
        assert arm_result.wait(2.0), "ARM service timed out"
        assert response.result().success
        assert peer.wait_command(
            lambda item: item[0] == MODE_ARMED
            and item[2:] == (100, -50, 200)
        )

        disarmed = probe.wait_telemetry(
            lambda msg: msg.handshake_ok and not msg.armed
            and "stale" in msg.status,
            timeout=2.0,
        )
        assert not disarmed.armed
        assert peer.wait_command(
            lambda item: item[0] == 0 and item[2:] == (0, 0, 0)
        )
        assert not peer.errors
    finally:
        executor.shutdown()
        for node in (probe, driver):
            executor.remove_node(node)
            node.destroy_node()
        peer.close()
        os.close(slave_fd)
        os.close(master_fd)
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
