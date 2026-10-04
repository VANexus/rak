"""ROS 2 host endpoint for the fail-closed F407 base UART protocol."""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rak_interfaces.msg import BaseTelemetry
from std_srvs.srv import SetBool

from .protocol import (
    MODE_ARMED,
    MODE_DISARMED,
    PROTOCOL_VERSION,
    REQUIRED_FEATURES,
    STATE_ARMED,
    STATE_DISARMED,
    TYPE_BASE_STATE,
    TYPE_FAULT_EVENT,
    TYPE_HELLO,
    TYPE_HELLO_ACK,
    TYPE_SET_BASE_VELOCITY,
    Frame,
    ProtocolError,
    StreamDecoder,
    encode_frame,
    fault_event,
    hello_ack_compatible,
    hello_payload,
    pack_velocity,
    sequence_is_newer,
    twist_to_wire,
    unpack_base_state,
)

try:
    import serial
except ImportError:  # Import remains optional for offline codec use.
    serial = None


class BaseUartNode(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__(
            "base_uart_driver", parameter_overrides=parameter_overrides
        )
        self.declare_parameter("port", "")
        self.declare_parameter("baud", 921600)
        self.declare_parameter("allow_hardware", False)
        self.declare_parameter("host_build_id", 1)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("telemetry_topic", "/base/telemetry")
        self.declare_parameter("max_vx_m_s", 0.0)
        self.declare_parameter("max_vy_m_s", 0.0)
        self.declare_parameter("max_wz_rad_s", 0.0)
        self.declare_parameter("command_timeout_sec", 0.1)
        self.declare_parameter("telemetry_timeout_sec", 0.1)

        self._port_path = str(self.get_parameter("port").value)
        self._baud = int(self.get_parameter("baud").value)
        self._allow_hardware = bool(self.get_parameter("allow_hardware").value)
        self._host_build_id = int(self.get_parameter("host_build_id").value)
        self._limits = (
            float(self.get_parameter("max_vx_m_s").value),
            float(self.get_parameter("max_vy_m_s").value),
            float(self.get_parameter("max_wz_rad_s").value),
        )
        self._limits_valid = all(
            value > 0.0 and value < float("inf") for value in self._limits
        )
        self._command_timeout = float(self.get_parameter("command_timeout_sec").value)
        self._telemetry_timeout = float(self.get_parameter("telemetry_timeout_sec").value)
        self._ser = None
        self._decoder = StreamDecoder()
        self._tx_sequence = 0
        self._rx_sequence = None
        self._last_echo = None
        self._last_echo_advance = 0.0
        self._last_state_rx = 0.0
        self._last_hello = 0.0
        self._last_command_rx = 0.0
        self._arm_requested_at = None
        self._last_twist = (0.0, 0.0, 0.0)
        self._last_command_sequence = 0
        self._board_id = 0
        self._firmware_version = 0
        self._boot_id = None
        self._state = None
        self._last_measured = (0.0, 0.0, 0.0)
        self._device_uptime_ms = 0
        self._fault_bitmap = 0
        self._armed = False
        self._connected = False
        self._handshake_ok = False
        self._valid = False
        self._status = "DISARMED: port is not configured"
        self._error_streak = 0
        self._last_tx = 0.0
        self._cmd_sub = self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self._on_twist,
            10,
        )
        self._telemetry_pub = self.create_publisher(
            BaseTelemetry, str(self.get_parameter("telemetry_topic").value), 10
        )
        self._arm_service = self.create_service(SetBool, "~/arm", self._on_arm)
        self._timer = self.create_timer(0.01, self._tick)

        if self._port_path:
            if serial is None:
                self._status = "pyserial is not installed"
                self.get_logger().error(self._status)
            elif not self._allow_hardware:
                self._status = "port configured but allow_hardware=false; device not opened"
                self.get_logger().warning(self._status)
            else:
                self._open_port()
        self.get_logger().info(
            f"UART driver ready; port={'unset' if not self._port_path else self._port_path}, "
            f"allow_hardware={self._allow_hardware}, mode=DISARMED"
        )

    def _open_port(self):
        try:
            self._ser = serial.Serial(
                self._port_path,
                self._baud,
                timeout=0,
                write_timeout=0.02,
                exclusive=True,
            )
            self._connected = True
            self._handshake_ok = False
            self._valid = False
            self._rx_sequence = None
            self._last_echo = None
            self._decoder = StreamDecoder()
            self._error_streak = 0
            self._status = "serial connected; awaiting HELLO_ACK"
            self._last_hello = 0.0
            self._armed = False
            self._arm_requested_at = None
        except Exception as exc:  # Serial errors vary by platform/driver.
            self._connected = False
            self._handshake_ok = False
            self._armed = False
            self._status = f"serial open failed: {exc}"
            self.get_logger().error(self._status)

    def _on_twist(self, msg: Twist):
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        try:
            wire = twist_to_wire(*values)
        except ProtocolError as exc:
            self._last_twist = (0.0, 0.0, 0.0)
            self._last_command_rx = 0.0
            self._armed = False
            self._status = f"invalid Twist rejected: {exc}"
            return
        if any(abs(value) > limit for value, limit in zip(values, self._limits)):
            self._last_twist = (0.0, 0.0, 0.0)
            self._last_command_rx = 0.0
            self._armed = False
            self._status = "Twist exceeds configured safety limits"
            return
        self._last_twist = tuple(float(value) for value in values)
        self._last_command_rx = time.monotonic()

    def _on_arm(self, request, response):
        if not request.data:
            self._armed = False
            self._arm_requested_at = None
            self._last_twist = (0.0, 0.0, 0.0)
            response.success = True
            response.message = "DISARMED requested"
            self._status = response.message
            return response
        now = time.monotonic()
        if not self._limits_valid:
            response.message = "ARM rejected: measured positive velocity limits are not configured"
        elif not self._allow_hardware:
            response.message = "ARM rejected: allow_hardware is false"
        elif not self._connected or self._ser is None:
            response.message = "ARM rejected: UART is disconnected"
        elif not self._handshake_ok:
            response.message = "ARM rejected: protocol handshake is not compatible"
        elif not self._valid or now - self._last_state_rx > self._telemetry_timeout:
            response.message = "ARM rejected: telemetry is stale or invalid"
        elif self._fault_bitmap != 0:
            response.message = f"ARM rejected: fault bitmap 0x{self._fault_bitmap:08x}"
        elif self._state != STATE_DISARMED:
            response.message = "ARM rejected: F407 is not confirmed DISARMED"
        else:
            self._armed = True
            self._arm_requested_at = now
            self._last_tx = 0.0
            response.success = True
            response.message = "ARMED by explicit operator request"
            self._status = response.message
            return response
        response.success = False
        self._status = response.message
        return response

    def _next_sequence(self):
        value = self._tx_sequence
        self._tx_sequence = (self._tx_sequence + 1) & 0xFFFF
        return value

    def _write_frame(self, message_type, payload):
        if self._ser is None:
            return
        frame = Frame(PROTOCOL_VERSION, message_type, self._next_sequence(), payload)
        self._ser.write(encode_frame(frame))
        self._last_tx = time.monotonic()

    def _send_hello(self, now):
        self._write_frame(TYPE_HELLO, hello_payload(self._host_build_id, REQUIRED_FEATURES))
        self._last_hello = now

    def _fail_closed(self, reason):
        was_armed = self._armed
        self._armed = False
        self._valid = False
        self._status = reason
        if was_armed and self._ser is not None and self._connected:
            try:
                self._send_velocity((0.0, 0.0, 0.0), MODE_DISARMED)
            except Exception as exc:
                self.get_logger().error(f"failed to write DISARMED command: {exc}")

    def _send_velocity(self, twist, mode):
        vx, vy, wz = twist_to_wire(*twist)
        sequence = self._next_sequence()
        payload = pack_velocity(mode, 50, vx, vy, wz)
        self._ser.write(
            encode_frame(Frame(PROTOCOL_VERSION, TYPE_SET_BASE_VELOCITY, sequence, payload))
        )
        self._last_command_sequence = sequence
        self._last_tx = time.monotonic()

    def _read_frames(self):
        if self._ser is None:
            return
        available = self._ser.in_waiting
        data = self._ser.read(available or 1)
        if data:
            errors_before = self._decoder.errors
            frames = self._decoder.feed(data)
            self._error_streak += self._decoder.errors - errors_before
            if frames:
                self._error_streak = 0
            for frame in frames:
                self._handle_frame(frame)
            if self._error_streak >= 3:
                self._fail_closed("three malformed UART packets")

    def _handle_frame(self, frame: Frame):
        now = time.monotonic()
        if frame.message_type == TYPE_HELLO_ACK:
            try:
                board_id, firmware, boot_id, _ = hello_ack_compatible(frame.payload)
            except ProtocolError as exc:
                self._fail_closed(f"incompatible HELLO_ACK: {exc}")
                return
            rebooted = self._boot_id is not None and boot_id != self._boot_id
            if rebooted:
                self._armed = False
                self._arm_requested_at = None
                self._valid = False
                self._rx_sequence = None
                self._last_echo = None
                self._state = None
                self._last_state_rx = 0.0
                self._status = "F407 reboot detected; re-handshaking DISARMED"
            if self._rx_sequence is not None and not sequence_is_newer(
                frame.sequence, self._rx_sequence
            ):
                self._fail_closed("stale HELLO_ACK sequence")
                return
            self._boot_id = boot_id
            self._board_id = board_id
            self._firmware_version = firmware
            self._rx_sequence = frame.sequence
            self._handshake_ok = True
            if not rebooted:
                self._status = "compatible F407 firmware; awaiting BASE_STATE"
            return
        if not self._handshake_ok:
            return
        if frame.message_type == TYPE_BASE_STATE:
            if self._rx_sequence is not None and not sequence_is_newer(
                frame.sequence, self._rx_sequence
            ):
                self._fail_closed("stale telemetry packet sequence")
                return
            try:
                state = unpack_base_state(frame.payload)
            except ProtocolError as exc:
                self._fail_closed(f"invalid BASE_STATE: {exc}")
                return
            if state.state not in (0, STATE_DISARMED, STATE_ARMED, 3):
                self._fail_closed(f"unknown F407 state {state.state}")
                return
            if state.echoed_command_sequence == self._last_echo:
                if now - self._last_echo_advance > self._telemetry_timeout:
                    self._fail_closed("command sequence echo stalled")
                    return
            elif self._last_echo is None or sequence_is_newer(
                state.echoed_command_sequence, self._last_echo
            ):
                self._last_echo = state.echoed_command_sequence
                self._last_echo_advance = now
            else:
                self._fail_closed("F407 echoed a stale command sequence")
                return
            if state.echoed_command_sequence != self._last_command_sequence and (
                now - self._last_tx > self._telemetry_timeout
            ):
                self._fail_closed("F407 command echo is behind")
                return
            self._rx_sequence = frame.sequence
            self._last_state_rx = now
            self._state = state.state
            self._fault_bitmap = state.fault_bitmap
            self._last_measured = (
                state.vx_mm_s / 1000.0,
                state.vy_mm_s / 1000.0,
                state.wz_mrad_s / 1000.0,
            )
            self._device_uptime_ms = state.device_uptime_ms
            self._valid = state.state in (STATE_DISARMED, STATE_ARMED) and state.fault_bitmap == 0
            if not self._valid:
                self._fail_closed(
                    f"invalid F407 state={state.state} fault=0x{state.fault_bitmap:08x}"
                )
            elif self._armed and state.state != STATE_ARMED:
                if self._arm_requested_at is None or now - self._arm_requested_at > self._telemetry_timeout:
                    self._fail_closed("F407 did not confirm ARMED")
            elif not self._armed and state.state == STATE_ARMED:
                self._fail_closed("F407 is ARMED without a host ARM request")
            else:
                self._status = "link healthy; ARMED" if self._armed else "link healthy; DISARMED"
            return
        if frame.message_type == TYPE_FAULT_EVENT:
            try:
                self._fault_bitmap, _ = fault_event(frame.payload)
            except ProtocolError as exc:
                self._fail_closed(f"invalid FAULT_EVENT: {exc}")
                return
            self._fail_closed(f"F407 fault event 0x{self._fault_bitmap:08x}")
            return
        self._fail_closed(f"unsupported F407 message type 0x{frame.message_type:02x}")

    def _tick(self):
        now = time.monotonic()
        if self._port_path and self._ser is None and self._allow_hardware and serial is not None:
            self._open_port()
        try:
            self._read_frames()
        except Exception as exc:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:
                    pass
            self._connected = False
            self._handshake_ok = False
            self._ser = None
            self._fail_closed(f"UART read failed: {exc}")
        if self._ser is not None and (
            not self._handshake_ok and now - self._last_hello >= 0.5
        ):
            try:
                self._send_hello(now)
            except Exception as exc:
                self._fail_closed(f"UART HELLO write failed: {exc}")
        elif self._ser is not None and self._handshake_ok and now - self._last_hello >= 1.0:
            try:
                self._send_hello(now)
            except Exception as exc:
                self._fail_closed(f"UART heartbeat HELLO failed: {exc}")
        if self._handshake_ok and now - self._last_state_rx > self._telemetry_timeout:
            self._fail_closed("BASE_STATE telemetry timeout")
        if self._armed and (
            now - self._last_command_rx > self._command_timeout or not self._valid
        ):
            self._fail_closed("command input stale or link unhealthy")
        if self._ser is not None and self._handshake_ok and now - self._last_tx >= 0.01:
            try:
                fresh = now - self._last_command_rx <= self._command_timeout
                twist = self._last_twist if self._armed and fresh and self._valid else (0.0, 0.0, 0.0)
                mode = MODE_ARMED if self._armed and fresh and self._valid else MODE_DISARMED
                self._send_velocity(twist, mode)
            except Exception as exc:
                self._fail_closed(f"UART command write failed: {exc}")
        self._publish_telemetry(now)

    def _publish_telemetry(self, now):
        msg = BaseTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.connected = self._connected and self._ser is not None
        msg.handshake_ok = self._handshake_ok
        msg.valid = self._valid and now - self._last_state_rx <= self._telemetry_timeout
        msg.armed = self._armed
        msg.mode = MODE_ARMED if self._armed else MODE_DISARMED
        msg.fault_bitmap = self._fault_bitmap
        msg.command_sequence = self._last_command_sequence
        msg.echoed_command_sequence = self._last_echo or 0
        if self._state is not None:
            # Measured velocities are populated from the latest BASE_STATE.
            msg.measured_twist.linear.x = self._last_measured[0]
            msg.measured_twist.linear.y = self._last_measured[1]
            msg.measured_twist.angular.z = self._last_measured[2]
        msg.device_uptime_ms = self._device_uptime_ms
        msg.age_sec = max(0.0, now - self._last_state_rx) if self._last_state_rx else 1.0e9
        msg.status = self._status
        self._telemetry_pub.publish(msg)

    def destroy_node(self):
        self._armed = False
        if self._ser is not None:
            try:
                if self._handshake_ok:
                    self._send_velocity((0.0, 0.0, 0.0), MODE_DISARMED)
            except Exception:
                pass
            try:
                self._ser.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BaseUartNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
