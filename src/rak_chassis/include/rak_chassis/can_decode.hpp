// Copyright 2026 RAK project
// [AI] CAN v4 帧解码（纯函数）：不依赖 socket / ROS / 线程，可脱离 SocketCAN 单测。
// 字节布局真源 = docs/protocol-signals.yaml v4（小端）。CanSession 的 process_rx 与
// test/can_frame_decode_test.cpp 共用本文件——布局只写一次。
#ifndef RAK_CHASSIS__CAN_DECODE_HPP_
#define RAK_CHASSIS__CAN_DECODE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace rak_chassis
{
/* [AI] STATE 位图里"坏消息"的位掩码（协议真源 PROTOCOL.md §5，v4 冻结）：
 * bit0 estop_latched / bit2 fault_active / bit3 cmd_stale / bit4 bat_uv / bit5 can_recovering。
 * 不进掩码的两段都必须排除，否则 ros2_control 会把健康态当故障：
 *   bit1 remote_running —— ARM(START) 后必然为 1；
 *   bit6-10 驱动门自诊断（dg_remote/dg_cal/dg_imu/dg_snapshot/dg_stab）—— 1 = 放行链正常。
 * 底盘板（0x220）与臂板（0x221）共用同一 state_bits 定义。 */
constexpr uint16_t kStateFaultMask = 0x003DU;

// ---- 小端读取（协议 byte_order: little）----
inline uint16_t read_u16(const std::array<uint8_t, 8> &d, size_t off)
{
  return static_cast<uint16_t>(
    static_cast<uint16_t>(d[off]) | (static_cast<uint16_t>(d[off + 1]) << 8U));
}

inline int16_t read_i16(const std::array<uint8_t, 8> &d, size_t off)
{
  return static_cast<int16_t>(read_u16(d, off));
}

inline int32_t read_i32(const std::array<uint8_t, 8> &d, size_t off)
{
  return static_cast<int32_t>(
    static_cast<uint32_t>(d[off]) | (static_cast<uint32_t>(d[off + 1]) << 8U) |
    (static_cast<uint32_t>(d[off + 2]) << 16U) | (static_cast<uint32_t>(d[off + 3]) << 24U));
}

// 0x030 / 0x031 NODE_INFO：PROTO_u8, HWREV_u8, BUILD_u16, FAULT_u16, STATE_u16
struct NodeInfoFrame
{
  uint8_t proto{};
  uint8_t hw_rev{};
  uint16_t build_tag{};   // 两字节 LE（"A2" -> 0x3241）
  uint16_t fault{};
  uint16_t state{};
  bool proto_ok{};        // == 4：旧固件拒绝发指令（v4 §10.4）
};
NodeInfoFrame decode_node_info(const std::array<uint8_t, 8> &d);

// 0x220 DIAG：BAT_u16, FAULT_u16, STATE_u16, MODE_u8, BUSOFFCNT_u8
struct DiagFrame
{
  uint16_t bat_mv{};
  uint16_t fault{};
  uint16_t state{};
  uint8_t mode{};
  uint8_t busoff_cnt{};
};
DiagFrame decode_diag(const std::array<uint8_t, 8> &d);

// 0x031 ARM_NODE_INFO：PROTO_u8, HWREV_u8, BUILD_u16, FAULT_u16, STATE_u16
using ArmNodeInfoFrame = NodeInfoFrame;   // 布局与底盘 NODE_INFO 完全相同

// 0x221 ARM_DIAG：FAULT_u16, STATE_u16, MODE_u8, SERVO_ERRCNT_u8, RSVD16
struct ArmDiagFrame
{
  uint16_t fault{};
  uint16_t state{};
  uint8_t mode{};
  uint8_t servo_errcnt{};
  // 臂板无电池通道，FAULT_u16 当前恒 0；J1/J2 位置环故障走 STATE.bit2。
  bool faulted{};   // STATE 坏消息位 || MODE∈{2,3}
};
ArmDiagFrame decode_arm_diag(const std::array<uint8_t, 8> &d);
}  // namespace rak_chassis

#endif  // RAK_CHASSIS__CAN_DECODE_HPP_
