"""rak_serial_link -- pure serial transport for chassis CAN v3 frames.

Independent of lidar/moveit/nav2. Owns the serial port, exposes raw TX/RX
plus the 0x010 safety service. Frame layout is the 8-byte CAN v3 record
from vehical_rak_chassis/docs/PROTOCOL.md -- this module implements the
wire format only, not what callers do with it.
"""