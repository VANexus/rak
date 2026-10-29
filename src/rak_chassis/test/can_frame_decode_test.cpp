// Copyright 2026 RAK project
// [AI] can_decode 的字节布局回归：断言两端（固件 / 上位机）都复现 protocol-signals.yaml
// 的测试向量。这些帧决定"链条健康还是故障"的判据，布局错一位就是安全语义错。
#include "rak_chassis/can_decode.hpp"

#include <gtest/gtest.h>

#include <array>

namespace
{
using rak_chassis::ArmDiagFrame;
using rak_chassis::ArmImuFrame;
using rak_chassis::ArmNodeInfoFrame;
using rak_chassis::ChassisImuFrame;
using rak_chassis::decode_arm_diag;
using rak_chassis::decode_arm_imu;
using rak_chassis::decode_chassis_imu;
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

// ===== [AI] 2026-10-29：IMU 两帧（0x032 / 0x222）与 protocol-signals.yaml 对标 =====
// CHASSIS_IMU VF：{DYAW 123 mrad, PITCH -150 (0.01°), ROLL 250 (0.01°), EVT 0x03, AGE 4}
TEST(CanDecode, ChassisImuVectorVF)
{
  const std::array<uint8_t, 8> d{0x7B, 0x00, 0x6A, 0xFF, 0xFA, 0x00, 0x03, 0x04};
  const ChassisImuFrame f = decode_chassis_imu(d);
  EXPECT_EQ(f.dyaw_mrad, 123);
  EXPECT_EQ(f.pitch_cdeg, -150);
  EXPECT_EQ(f.roll_cdeg, 250);
  EXPECT_EQ(f.events, 0x03U);
  EXPECT_EQ(f.age_ms, 4U);
}

// 负偏航增量（左转）必须解成负数，不能被当成 u16 大正数
TEST(CanDecode, ChassisImuNegativeYawDeltaIsSigned)
{
  const std::array<uint8_t, 8> d{0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(decode_chassis_imu(d).dyaw_mrad, -32768);
}

// ARM_IMU V10：{GZ -1234 (0.01°/s), AMAG 1450 mg, EVT 0x01, AGE 9}
TEST(CanDecode, ArmImuVectorV10)
{
  const std::array<uint8_t, 8> d{0x2E, 0xFB, 0xAA, 0x05, 0x01, 0x09, 0x00, 0x00};
  const ArmImuFrame f = decode_arm_imu(d);
  EXPECT_EQ(f.gz_cdps, -1234);
  EXPECT_EQ(f.amag_mg, 1450U);   // 静态 ≈1000 mg，量程内
  EXPECT_EQ(f.events, 0x01U);
  EXPECT_EQ(f.age_ms, 9U);
}

// 事件位在两端必须同一套：撞车会让“轻微冲击”被读成“打滑”
TEST(CanDecode, ImuEventBitsMatchProtocol)
{
  EXPECT_EQ(rak_chassis::kImuEvtImpact, 0x01U);
  EXPECT_EQ(rak_chassis::kImuEvtSlip, 0x02U);
  EXPECT_EQ(rak_chassis::kImuEvtStill, 0x04U);
  EXPECT_EQ(rak_chassis::kImuEvtSaturated, 0x08U);
  EXPECT_EQ(rak_chassis::kImuEvtStale, 0x10U);
}
