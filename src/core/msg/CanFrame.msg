# CAN bus frame, produced by the Rust nerve bridge and consumed by the system.
# Mirrors a classic / CAN FD frame with metadata.

# Arbitration ID (11-bit standard or 29-bit extended)
uint32 id
bool is_extended
bool is_error_frame
bool is_remote_request

# Payload (CAN: up to 8 bytes, CAN FD: up to 64 bytes)
uint8[] data
uint8 dlc

# Timestamp when the frame was received by the kernel
builtin_interfaces/Time stamp
