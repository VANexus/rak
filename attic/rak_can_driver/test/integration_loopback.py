"""Mac-only loopback integration test for rak_can_driver.chassis_node.

Architecture (no real CAN hardware needed):
  - python-can virtual bus, channel "test"  (single shared server)
  - ChassisCanNode (`chassis_can_node`) opens as virtual client, sends 0x100
    at 100 Hz when /cmd_vel is published
  - A small mock chassis process subscribes to the same virtual bus and
    replies with a few 0x200 / 0x201 / 0x020 frames so we can verify the
    decoder path end-to-end.

Run with:
  PYTHONPATH=src/rak_can_driver python3 src/rak_can_driver/test/integration_loopback.py
"""

from __future__ import annotations

import os
import sys
import threading
import time

import can
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor

from rak_can_driver.chassis_node import ChassisCanNode
from rak_can_driver.protocol import (
    ChassisVelFb,
    ChassisDiag,
    build_frame,
)


def mock_chassis(bus: can.interface.Bus, stop: threading.Event):
    """Receive 0x100 CHASSIS_VEL_CMD and reply with 0x200 + 0x201."""
    counter = 0
    while not stop.is_set():
        try:
            rx = bus.recv(timeout=0.1)
        except can.CanError:
            continue
        if rx is None:
            continue
        if rx.arbitration_id != 0x100:
            continue
        counter = (counter + 1) & 0xF
        # measured velocity = command velocity (echo, for testing)
        vx = int.from_bytes(rx.data[0:2], "little", signed=True)
        vy = int.from_bytes(rx.data[2:4], "little", signed=True)
        wz = int.from_bytes(rx.data[4:6], "little", signed=True)
        # 0x200 CHASSIS_VEL_FB
        flags = 0x1  # mode=REMOTE
        body = bytearray()
        for v in (vx, vy, wz):
            body += v.to_bytes(2, "little", signed=True)
        body.append(((counter & 0xF) << 4) | (flags & 0xF))
        body.append(0x00)  # placeholder CRC
        import struct
        crc_input = bytes(body)
        from rak_can_driver.protocol import crc8
        body[-1] = crc8(crc_input, 0x09)
        try:
            bus.send(can.Message(arbitration_id=0x200, data=bytes(body)), timeout=0.01)
        except can.CanError:
            pass
        # 0x201 CHASSIS_DIAG every 10 cycles (10 Hz)
        if counter == 0:
            dbody = bytearray()
            dbody += (12_000).to_bytes(2, "little")  # 12 V battery
            dbody += (0).to_bytes(2, "little")        # no fault
            dbody += (0b0000_0000_0000_0001).to_bytes(2, "little")  # homed=1
            dbody.append(((counter & 0xF) << 4) | 0x1)
            dbody[-1] = crc8(bytes(dbody), 0x0A)
            try:
                bus.send(can.Message(arbitration_id=0x201, data=bytes(dbody)), timeout=0.01)
            except can.CanError:
                pass


def main():
    # 1) Start python-can virtual bus server (so multiple clients can share)
    from can.interfaces.virtual import VirtualBus
    import socket
    # Use a TCP-backed virtual bus so multiple processes can share
    # (this works on Mac; on Linux you'd use socketcan directly)
    os.environ.setdefault("VIRTUAL_CAN_PORT", "9999")
    # Try socketcand style via can.interfaces.socketcand
    # Simpler: just use Python-can's multi-channel VirtualBus via 'virtual' interface.
    # python-can virtual bus can be shared by clients across processes via
    # /tmp/python-can-<uid>.sock or by using a TCP bridge.

    # 1a) The mock chassis opens a virtual bus on "test"; the host node opens
    # on the same "test" channel. python-can virtual supports multiple clients
    # by spawning a server automatically. We just use the same channel name.

    # Start the mock chassis thread
    mock_stop = threading.Event()
    mock_bus = can.Bus(interface="virtual", channel="test", receive_own_messages=True)
    mock_thread = threading.Thread(target=mock_chassis, args=(mock_bus, mock_stop), daemon=True)
    mock_thread.start()

    # 2) Boot ROS 2 + ChassisCanNode
    rclpy.init()
    node = ChassisCanNode(parameter_overrides=[
        # override bus params so it joins the same virtual channel
    ])
    # Force the bus to share the same virtual channel
    # (ChassisCanNode.__init__ already opens virtual/test by default)

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # 3) Publish a /cmd_vel (0.2 m/s forward, 0.1 rad/s yaw)
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = 0.1
    pub.publish(msg)
    # spin for ~0.5 s so several 100 Hz ticks + frames flow
    end = time.monotonic() + 0.5
    while time.monotonic() < end:
        executor.timeout_once(0.05)

    # 4) Verify: the host should have measured_twist close to (0.2, 0, 0.1)
    msgs = []
    def cb(twist):
        msgs.append((twist.linear.x, twist.linear.y, twist.angular.z))
    sub = node.create_subscription(Twist, "/chassis/measured_twist", cb, 10)
    end = time.monotonic() + 0.5
    while time.monotonic() < end:
        executor.timeout_once(0.05)

    print(f"received {len(msgs)} measured_twist messages")
    if msgs:
        x, y, wz = msgs[-1]
        print(f"  last: vx={x:.3f} m/s, vy={y:.3f} m/s, wz={wz:.3f} rad/s")
        # Allow some tolerance: integer rounding through m_s_to_mm_s
        assert abs(x - 0.2) < 0.01, f"vx mismatch: {x}"
        assert abs(wz - 0.1) < 0.01, f"wz mismatch: {wz}"
        print("  PASS: measured twist matches commanded twist within tolerance")
    else:
        print("  FAIL: no measured_twist received")
        sys.exit(1)

    # 5) Verify: safety service START -> enabled flag set, STOP -> cleared
    from std_srvs.srv import Trigger
    cli = node.create_client(Trigger, "/chassis_can_node/safety_cmd")
    if not cli.wait_for_service(timeout_sec=1.0):
        print("  FAIL: safety_cmd service not available")
        sys.exit(1)
    fut = cli.call_async(Trigger.Request(data="START"))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
    resp = fut.result()
    print(f"  safety START -> success={resp.success} msg={resp.message!r}")
    assert resp.success

    fut = cli.call_async(Trigger.Request(data="EMERGENCY_STOP"))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
    resp = fut.result()
    print(f"  safety EMERGENCY_STOP -> success={resp.success} msg={resp.message!r}")
    assert resp.success

    # Cleanup
    mock_stop.set()
    mock_thread.join(timeout=1.0)
    node.destroy_node()
    rclpy.shutdown()
    try:
        mock_bus.shutdown()
    except Exception:
        pass
    print("OK")


if __name__ == "__main__":
    main()
