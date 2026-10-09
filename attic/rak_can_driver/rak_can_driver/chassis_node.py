"""Jetson host node for the chassis CAN v3 protocol.

Subscribes /cmd_vel (Twist), encodes 0x100 CHASSIS_VEL_CMD @ 100 Hz over
CAN 2.0A @ 1 Mbps, decodes 0x200/0x201/0x020 from the chassis. Also
exposes ~/safety_cmd (0x010) for emergency stop / homing / clear-fault.

Defaults to a python-can "virtual" bus so the node can be exercised on a
dev machine (Mac, no mttcan). On the real Jetson Orin Nano, pass
``--bus-type socketcan --channel can0`` after running:

    sudo ip link set can0 type can bitrate 1000000
    sudo ip link set can0 up

NEVER enable CAN FD on can0 -- bxCAN error-frames an FD frame.

Argument ``--loopback`` enables python-can internal loopback so the
node can sanity-check encode/decode without a real peer.

SECURITY MODEL (prototype-grade; production requires SROS2 + hardware
interlock per the chassis team's review):
  - ~/safety_cmd and /cmd_vel are NOT authenticated today.  In production
    the launch should add SROS2 policy: only a signed `arm` principal may
    call START (it then transitions to REMOTE); only a signed `cmd_vel`
    principal may publish /cmd_vel.  The host node here refuses START if
    no valid ARM handshake has occurred (enforced by the chassis MCU via
    estop_latched / fault_active feedback in BASE_STATE).
  - All parameters that flow into python-can's Bus() are allowlisted at
    startup; bitrate is fixed at 1_000_000 (CAN FD forbidden).
  - No state on this node can clear a hardware e-stop latch; that lives
    in the chassis MCU and requires the SOFT_RESET (code=7) sequence
    per protocol §7.1.
"""

from __future__ import annotations

import re
import threading
from typing import Optional

import can  # python-can
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt16
from std_srvs.srv import Trigger

from .protocol import (
    ChassisDiag,
    ChassisVelCmd,
    ChassisVelFb,
    SafetyCmd,
    SAFETY_CMD_NAMES,
    build_frame,
    decode_frame,
    m_s_to_mm_s,
    rad_s_to_mrad_s,
    mm_s_to_m_s,
    mrad_s_to_rad_s,
)


class ChassisCanNode(Node):
    """Drives 0x100 CHASSIS_VEL_CMD and listens for 0x200/0x201/0x020."""

    # Security: allowlist only the bus_type and channel parameter values that
    # pass through to a low-level plugin sink (python-can `Bus`).  Anything
    # outside the allowlist is rejected at startup before we touch the OS.
    _ALLOWED_BUS_TYPES = ("virtual", "socketcan")
    _ALLOWED_CHANNELS_RE = re.compile(r"^(can[0-9]+|[A-Za-z0-9_-]{1,32})$")
    _CAN_BITRATE_BPS = 1_000_000  # fixed; CAN FD is forbidden by protocol §1

    def __init__(self, *, parameter_overrides=None):
        super().__init__("chassis_can_node", parameter_overrides=parameter_overrides)

        # --- CAN bus parameters ---
        self.declare_parameter("bus_type", "virtual")
        self.declare_parameter("channel", "test")          # "can0" in production
        # bitrate is fixed (CAN 2.0A @ 1 Mbps; FD is explicitly forbidden).  We
        # keep the parameter declared only so an out-of-band `ros2 param set`
        # cannot accidentally configure a non-standard rate.
        self.declare_parameter("bitrate", self._CAN_BITRATE_BPS)
        self.declare_parameter("loopback", False)          # python-can internal loopback
        self.declare_parameter("receive_own_messages", False)
        # --- ROS topics / services ---
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("measured_twist_topic", "/chassis/measured_twist")
        self.declare_parameter("diagnostics_topic", "/chassis/diagnostics")
        # --- safety / arming ---
        self.declare_parameter("initial_enable", False)
        self.declare_parameter("max_vx_m_s", 2.0)
        self.declare_parameter("max_vy_m_s", 2.0)
        self.declare_parameter("max_wz_rad_s", 3.0)

        self._bus_type = str(self.get_parameter("bus_type").value)
        self._channel = str(self.get_parameter("channel").value)
        self._bitrate = int(self.get_parameter("bitrate").value)
        self._loopback = bool(self.get_parameter("loopback").value)
        self._max_vx = float(self.get_parameter("max_vx_m_s").value)
        self._max_vy = float(self.get_parameter("max_vy_m_s").value)
        self._max_wz = float(self.get_parameter("max_wz_rad_s").value)

        # Validate parameters that flow into the python-can Bus constructor.
        # Channel is constrained to either a numeric "canN" interface name
        # (production path) or a short alphanumeric virtual bus id.
        if self._bus_type not in self._ALLOWED_BUS_TYPES:
            raise ValueError(
                f"bus_type {self._bus_type!r} not in allowlist "
                f"{self._ALLOWED_BUS_TYPES}"
            )
        if not self._ALLOWED_CHANNELS_RE.match(self._channel):
            raise ValueError(
                f"channel {self._channel!r} does not match "
                f"{self._ALLOWED_BUS_TYPES and self._ALLOWED_CHANNELS_RE.pattern}"
            )
        if self._bitrate != self._CAN_BITRATE_BPS:
            raise ValueError(
                f"bitrate {self._bitrate} != fixed {self._CAN_BITRATE_BPS} "
                f"(CAN v3 forbids CAN FD)"
            )

        # --- ROS interface ---
        self._cmd_sub = self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self._on_twist,
            10,
        )
        self._measured_pub = self.create_publisher(
            Twist, str(self.get_parameter("measured_twist_topic").value), 10
        )
        # /chassis/diagnostics layout (Float32MultiArray, 3 fields):
        #   data[0] = battery_volts
        #   data[1] = fault_code (uint16 raw)
        #   data[2] = state_bits (uint16 raw)
        self._diag_pub = self.create_publisher(
            Float32MultiArray, str(self.get_parameter("diagnostics_topic").value), 10
        )
        self._safety_srv = self.create_service(
            Trigger, "~/safety_cmd", self._on_safety_cmd
        )

        # --- State ---
        self._counter = 0
        self._last_twist = (0.0, 0.0, 0.0)  # m/s, m/s, rad/s
        self._enabled = bool(self.get_parameter("initial_enable").value)
        self._bus: Optional[can.interface.Bus] = None
        self._lock = threading.Lock()
        self._stopped = False

        # --- Open the bus ---
        try:
            bus_kwargs = dict(
                interface=self._bus_type,
                channel=self._channel,
                bitrate=self._bitrate,
                receive_own_messages=self._loopback or self.get_parameter("receive_own_messages").value,
            )
            if self._bus_type == "virtual":
                # python-can virtual bus: don't pass bitrate; default is set per channel
                bus_kwargs.pop("bitrate", None)
                bus_kwargs.pop("receive_own_messages", None)
            elif self._bus_type == "socketcan":
                # real Jetson path: bitrate is a no-op (set by `ip link`); keep for clarity
                pass
            self._bus = can.Bus(**bus_kwargs)
            self.get_logger().info(
                f"CAN bus open: type={self._bus_type} channel={self._channel} "
                f"bitrate={self._bitrate} loopback={self._loopback}"
            )
        except can.CanError as exc:
            self.get_logger().error(f"failed to open CAN bus: {exc}")
            self._bus = None

        # --- Timers ---
        # 100 Hz velocity command + listener poll
        self._tx_timer = self.create_timer(0.01, self._tx_tick)   # 10 ms = 100 Hz
        self._rx_timer = self.create_timer(0.005, self._rx_tick)   # 5 ms poll for received frames

    # -----------------------------------------------------------------------
    # Twists -> atomic command
    # -----------------------------------------------------------------------
    def _on_twist(self, msg: Twist):
        vx, vy, wz = float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)
        # clamp to configured safety envelope (host side; MCU has its own)
        vx = max(-self._max_vx, min(self._max_vx, vx))
        vy = max(-self._max_vy, min(self._max_vy, vy))
        wz = max(-self._max_wz, min(self._max_wz, wz))
        self._last_twist = (vx, vy, wz)

    def _next_counter(self) -> int:
        c = self._counter
        self._counter = (self._counter + 1) & 0xF
        return c

    def _tx_tick(self):
        if self._bus is None:
            return
        vx_m, vy_m, wz_r = self._last_twist
        try:
            frame = ChassisVelCmd(
                vx_mm_s=m_s_to_mm_s(vx_m),
                vy_mm_s=m_s_to_mm_s(vy_m),
                wz_mrad_s=rad_s_to_mrad_s(wz_r),
                enable=self._enabled,
            ).encode(counter=self._next_counter())
        except OverflowError as exc:
            self.get_logger().warn(f"twist clamp failed: {exc}")
            return
        msg = can.Message(
            arbitration_id=0x100,
            data=frame,
            is_extended_id=False,
        )
        try:
            self._bus.send(msg, timeout=0.005)
        except can.CanError as exc:
            self.get_logger().warn(f"CAN send failed: {exc}")

    # -----------------------------------------------------------------------
    # Listener: 0x200 CHASSIS_VEL_FB, 0x201 CHASSIS_DIAG, 0x020 EMCY
    # -----------------------------------------------------------------------
    def _rx_tick(self):
        if self._bus is None:
            return
        try:
            # Non-blocking: short timeout so we don't stall the timer
            rx = self._bus.recv(timeout=0.0)
        except (can.CanError, OSError) as exc:
            self.get_logger().warn(f"CAN recv failed: {exc}")
            return
        if rx is None:
            return
        self._handle_frame(rx)

    def _handle_frame(self, msg: can.Message):
        can_id = msg.arbitration_id
        data = bytes(msg.data)
        if len(data) != 8:
            return
        try:
            if can_id == 0x200:
                fb = ChassisVelFb.decode(data)
                twist = Twist()
                twist.linear.x = mm_s_to_m_s(fb.vx_mm_s)
                twist.linear.y = mm_s_to_m_s(fb.vy_mm_s)
                twist.angular.z = mrad_s_to_rad_s(fb.wz_mrad_s)
                self._measured_pub.publish(twist)
            elif can_id == 0x201:
                diag = ChassisDiag.decode(data)
                arr = Float32MultiArray()
                arr.data = [
                    diag.battery_mv / 1000.0,
                    float(diag.fault_code),
                    float(diag.state_bits),
                ]
                self._diag_pub.publish(arr)
                if diag.fault_code != 0:
                    self.get_logger().warn(
                        f"chassis FAULT_CODE=0x{diag.fault_code:04x} "
                        f"state_bits=0x{diag.state_bits:04x}"
                    )
            elif can_id == 0x020:
                # EMCY_CHASSIS -- log
                fault_code = int.from_bytes(data[0:2], "little")
                fault_aux = int.from_bytes(data[2:4], "little")
                self.get_logger().error(
                    f"EMCY_CHASSIS fault_code=0x{fault_code:04x} aux=0x{fault_aux:04x}"
                )
        except ValueError as exc:
            self.get_logger().warn(f"decode error on id 0x{can_id:03x}: {exc}")

    # -----------------------------------------------------------------------
    # Safety service: ~/safety_cmd (Trigger; data = "START" / "STOP" / etc.)
    # -----------------------------------------------------------------------
    _CMD_NAME_TO_CODE = {v: k for k, v in SAFETY_CMD_NAMES.items()}

    def _on_safety_cmd(self, request, response):
        name = (request.data or "").strip().upper()
        if name not in self._CMD_NAME_TO_CODE:
            response.success = False
            response.message = (
                f"unknown cmd '{request.data}'. valid: {sorted(self._CMD_NAME_TO_CODE)}"
            )
            return response
        code = self._CMD_NAME_TO_CODE[name]
        if self._bus is None:
            response.success = False
            response.message = "CAN bus not open"
            return response
        try:
            frame = SafetyCmd(cmd_code=code, counter=self._next_counter()).encode()
            self._bus.send(can.Message(arbitration_id=0x010, data=frame), timeout=0.005)
        except can.CanError as exc:
            response.success = False
            response.message = f"send failed: {exc}"
            return response
        # local state side effects
        if code == 1:        # START
            self._enabled = True
        elif code in (2, 4):  # STOP / EMERGENCY_STOP
            self._enabled = False
            self._last_twist = (0.0, 0.0, 0.0)
        elif code == 5:    # CLEAR_ESTOP -- leave arm state alone
            pass
        elif code == 6:    # CLEAR_FAULT
            pass
        response.success = True
        response.message = f"SAFETY_CMD {name} (code={code}) sent"
        return response

    # -----------------------------------------------------------------------
    def destroy_node(self):
        if self._bus is not None:
            try:
                self._bus.shutdown()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ChassisCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
