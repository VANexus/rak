"""Pure serial link ROS node -- owns the wire, nothing else.

Subscribes nothing (no /cmd_vel, no nav2). Exposes:
  - service ~/send_frame (std_srvs/Trigger; data = "0xID hex") for ad-hoc TX
  - service ~/safety_cmd (Trigger; data = "START" / "STOP" / ...) for SAFETY_CMD
  - publisher ~/rx_frames (std_msgs/UInt8MultiArray) raw 8-byte frames from MCU
  - publisher ~/rx_diag (std_msgs/String) one-line summary "id=0x100 name=V_T0 cnt=N flags=F"

The node does not interpret motion -- upper layers (nav2, task_executor,
moveit) decide what to send; this layer only wires bytes.

Defaults: /dev/wheel_motor @ 921600 8N1, blocking-but-async via a reader
thread that pushes into a Lock; TX happens from rclpy thread via service.
"""
from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from serial import Serial, SerialException
from std_msgs.msg import String, UInt8MultiArray
from std_srvs.srv import Trigger

from .protocol import (
    FrameStreamDecoder,
    SAFETY_CMD_NAMES,
    build_chassis_vel_cmd,
    build_safety_cmd,
)


class SerialLinkNode(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__("serial_link_node", parameter_overrides=parameter_overrides)
        self.declare_parameter("port", "/dev/wheel_motor")
        self.declare_parameter("baud", 921600)
        self.declare_parameter("open_timeout_sec", 2.0)

        self._port = str(self.get_parameter("port").value)
        self._baud = int(self.get_parameter("baud").value)
        self._serial: Optional[Serial] = None
        self._tx_counter = 0
        self._decoder = FrameStreamDecoder()
        self._rx_thread: Optional[threading.Thread] = None
        self._rx_stop = threading.Event()
        self._lock = threading.Lock()

        self._rx_pub = self.create_publisher(UInt8MultiArray, "~/rx_frames", 10)
        self._diag_pub = self.create_publisher(String, "~/rx_diag", 10)
        self._send_srv = self.create_service(Trigger, "~/send_frame", self._on_send_frame)
        self._safety_srv = self.create_service(Trigger, "~/safety_cmd", self._on_safety_cmd)

        try:
            self._serial = Serial(self._port, self._baud, timeout=0.1)
            self.get_logger().info(f"opened {self._port} @ {self._baud} 8N1")
        except (SerialException, OSError) as exc:
            self.get_logger().error(f"failed to open {self._port}: {exc}")
            self._serial = None
            return

        self._rx_stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    # ------------------------------------------------------------------
    def _rx_loop(self) -> None:
        while not self._rx_stop.is_set() and rclpy.ok() and self._serial is not None:
            try:
                data = self._serial.read(64)
            except (SerialException, OSError) as exc:
                self.get_logger().warn(f"rx failed: {exc}")
                continue
            if not data:
                continue
            with self._lock:
                frames = self._decoder.feed(data)
            for f in frames:
                arr = UInt8MultiArray()
                arr.data = list(f.raw)
                self._rx_pub.publish(arr)
                diag = String()
                diag.data = (
                    f"id=0x{f.can_id:03x} name={f.name} cnt={f.counter} flags=0x{f.flags:x}"
                )
                self._diag_pub.publish(diag)

    # ------------------------------------------------------------------
    def _on_send_frame(self, request, response):
        """Trigger.data = "0xID hex[,cnt=hex][,flags=hex][,data=hex]" or free-form."""
        text = (request.data or "").strip()
        if not text:
            response.success = False
            response.message = "empty request.data"
            return response
        try:
            tokens = [t.strip() for t in text.split(",")]
            can_id = int(tokens[0], 16)
            if not 0 <= can_id <= 0x7FF:
                raise ValueError("can_id out of 11-bit range")
            counter = int(tokens[1].split("=", 1)[1], 16) if len(tokens) > 1 and "=" in tokens[1] else self._tx_counter
            flags = int(tokens[2].split("=", 1)[1], 16) if len(tokens) > 2 and "=" in tokens[2] else 0x0
            raw_override = bytes.fromhex(tokens[3].split("=", 1)[1].replace(" ", "")) if len(tokens) > 3 and tokens[3].startswith("data=") else None
        except (ValueError, IndexError) as exc:
            response.success = False
            response.message = f"parse error: {exc}"
            return response

        with self._lock:
            if self._serial is None:
                response.success = False
                response.message = "serial not open"
                return response
            try:
                if raw_override is not None and len(raw_override) == 8:
                    payload = raw_override
                elif can_id == 0x100:
                    payload = build_chassis_vel_cmd(0, 0, 0, counter=counter,
                                                    enable=bool(flags & 1))
                elif can_id == 0x010:
                    # for SAFETY_CMD we treat flags as cmd_code
                    payload = build_safety_cmd(cmd_code=flags & 0xF, counter=counter)
                else:
                    response.success = False
                    response.message = (
                        f"can_id 0x{can_id:03x} has no built-in builder; "
                        "send raw with data=HEX"
                    )
                    return response
                self._serial.write(payload)
                self._tx_counter = (self._tx_counter + 1) & 0xF
                response.success = True
                response.message = f"sent 8 bytes for 0x{can_id:03x}: {payload.hex().upper()}"
            except (SerialException, OSError) as exc:
                response.success = False
                response.message = f"write failed: {exc}"
        return response

    def _on_safety_cmd(self, request, response):
        name = (request.data or "").strip().upper()
        if name not in SAFETY_CMD_NAMES:
            response.success = False
            response.message = (
                f"unknown cmd '{request.data}'. valid: {sorted(SAFETY_CMD_NAMES.values())}"
            )
            return response
        code = {v: k for k, v in SAFETY_CMD_NAMES.items()}[name]
        with self._lock:
            if self._serial is None:
                response.success = False
                response.message = "serial not open"
                return response
            try:
                frame = build_safety_cmd(code, counter=self._tx_counter)
                self._serial.write(frame)
                self._tx_counter = (self._tx_counter + 1) & 0xF
                response.success = True
                response.message = f"SAFETY_CMD {name} (code={code}) sent"
            except (SerialException, OSError) as exc:
                response.success = False
                response.message = f"write failed: {exc}"
        return response

    def destroy_node(self):
        self._rx_stop.set()
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.0)
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialLinkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()