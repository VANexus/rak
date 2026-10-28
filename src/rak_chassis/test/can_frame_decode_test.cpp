// Copyright 2026 RAK project
// [AI] can_decode 的字节布局回归：断言两端（固件 / 上位机）都复现 protocol-signals.yaml
// 的测试向量。这些帧决定"链条健康还是故障"的判据，布局错一位就是安全语义错。
#include "rak_chassis/can_decode.hpp"

#include <gtest/gtest.h>

#include <array>

namespace
{
using rak_chassis::ArmDiagFrame;
using rak_chassis::ArmNodeInfoFrame;
using rak_chassis::decode_arm_diag;
using rak_chassis::decode_diag;
using rak_chassis::decode_node_info;
using rak_chassis::DiagFrame;
using rak_chassis::kStateFaultMask;
using rak_chassis::NodeInfoFrame;
}  // namespace

// NODE_INFO V3：{PROTO 4, HWREV 1, BUILD 0xABCD, FAULT 0, STATE 0x0001}
TEST(CanDecode, NodeInfoVectorV3)
{
  const std::array<uint8_t, 8> d{0x04, 0x01, 0xCD, 0xAB, 0x00, 0x00, 0x01, 0x00};
  const NodeInfoFrame info = decode_node_info(d);
  EXPECT_TRUE(info.proto_ok);
  EXPECT_EQ(info.proto, 4U);
  EXPECT_EQ(info.hw_rev, 1U);
  EXPECT_EQ(info.build_tag, 0xABCDU);
  EXPECT_EQ(info.fault, 0U);
  EXPECT_EQ(info.state, 0x0001U);
}

// 旧固件（PROTO=3）不得被判为已握手
TEST(CanDecode, NodeInfoRejectsNonV4)
{
  const std::array<uint8_t, 8> d{0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  EXPECT_FALSE(decode_node_info(d).proto_ok);
}

// ARM_NODE_INFO VD：{PROTO 4, HWREV 1, BUILD 0x1234, FAULT 0, STATE 0x0000}
TEST(CanDecode, ArmNodeInfoVectorVD)
{
  const std::array<uint8_t, 8> d{0x04, 0x01, 0x34, 0x12, 0x00, 0x00, 0x00, 0x00};
  const ArmNodeInfoFrame info = decode_node_info(d);
  EXPECT_TRUE(info.proto_ok);
  EXPECT_EQ(info.build_tag, 0x1234U);
  EXPECT_EQ(info.fault, 0U);
  EXPECT_EQ(info.state, 0U);
}

// A2 固件实际发的 build_tag 两字节 "A","2" -> LE u16 = 0x3241
TEST(CanDecode, ArmNodeInfoBuildTagIsTwoCharLe)
{
  const std::array<uint8_t, 8> d{0x04, 0x01, 'A', '2', 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(decode_node_info(d).build_tag, 0x3241U);
}

// DIAG VC：{BAT 11100, FAULT 0, STATE 0x0003, MODE 1, BUSOFFCNT 0}
TEST(CanDecode, DiagVectorVC)
{
  const std::array<uint8_t, 8> d{0x5C, 0x2B, 0x00, 0x00, 0x03, 0x00, 0x01, 0x00};
  const DiagFrame diag = decode_diag(d);
  EXPECT_EQ(diag.bat_mv, 11100U);
  EXPECT_EQ(diag.fault, 0U);
  EXPECT_EQ(diag.state, 0x0003U);
  EXPECT_EQ(diag.mode, 1U);
  EXPECT_EQ(diag.busoff_cnt, 0U);
}

// ARM_DIAG VE：{FAULT 0, STATE 0x0003, MODE 1, SERVO_ERRCNT 0, RSVD16 0}
TEST(CanDecode, ArmDiagVectorVE)
{
  const std::array<uint8_t, 8> d{0x00, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00};
  const ArmDiagFrame diag = decode_arm_diag(d);
  EXPECT_EQ(diag.fault, 0U);
  EXPECT_EQ(diag.state, 0x0003U);
  EXPECT_EQ(diag.mode, 1U);
  EXPECT_EQ(diag.servo_errcnt, 0U);
  // 0x0003 含 bit0 estop_latched -> 判故障（与底盘 DIAG 同一 state_bits 口径）
  EXPECT_TRUE(diag.faulted);
}

// J1/J2 位置环故障：STATE.bit2（fault_active）置位即判故障
TEST(CanDecode, ArmDiagPositionLoopFaultViaStateBit2)
{
  const std::array<uint8_t, 8> d{0x00, 0x00, 0x04, 0x00, 0x01, 0x07, 0x00, 0x00};
  const ArmDiagFrame diag = decode_arm_diag(d);
  EXPECT_EQ(diag.servo_errcnt, 7U);
  EXPECT_TRUE(diag.faulted);
}

// 健康态：只有 bit1 remote_running -> 不判故障（掩码 0x003D 排除 bit1）
TEST(CanDecode, HealthyStateIsNotFault)
{
  const std::array<uint8_t, 8> d{0x00, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00};
  EXPECT_FALSE(decode_arm_diag(d).faulted);
}

// 驱动门自诊断位 bit6-10（实测 0x0180）也不得判故障
TEST(CanDecode, DriverGateDiagBitsAreNotFault)
{
  const std::array<uint8_t, 8> d{0x00, 0x00, 0x80, 0x01, 0x01, 0x00, 0x00, 0x00};
  EXPECT_FALSE(decode_arm_diag(d).faulted);
  EXPECT_EQ(static_cast<uint16_t>(0x0180U & kStateFaultMask), 0U);
}

// MODE=2(ESTOP)/3(FAULT) 即使 STATE 干净也判故障
TEST(CanDecode, EstopAndFaultModesAreFaults)
{
  const std::array<uint8_t, 8> estop{0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};
  const std::array<uint8_t, 8> fault{0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00};
  EXPECT_TRUE(decode_arm_diag(estop).faulted);
  EXPECT_TRUE(decode_arm_diag(fault).faulted);
}

// 掩码覆盖率：欠压 bit4 与 can_recovering bit5 都在坏消息集合内
TEST(CanDecode, BatteryUndervoltageAndBusRecoveryAreFaults)
{
  EXPECT_EQ(static_cast<uint16_t>(0x0010U & kStateFaultMask), 0x0010U);
  EXPECT_EQ(static_cast<uint16_t>(0x0020U & kStateFaultMask), 0x0020U);
  EXPECT_EQ(static_cast<uint16_t>(0x0002U & kStateFaultMask), 0U);
}
