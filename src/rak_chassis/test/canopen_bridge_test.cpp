// [AI] 2026-11-14 薄 CANopen 桥的参数契约与量化单测。
// 不初始化 rclcpp、不开 CAN、不连 proxy driver：只喂假的 HardwareInfo。
#include "rak_chassis/canopen_bridge.hpp"

#include <gtest/gtest.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
using Params = std::unordered_map<std::string, std::string>;

hardware_interface::ComponentInfo make_joint(
  const std::string & name, const std::string & command_kind,
  const std::string & state_kind, Params parameters, bool with_limits)
{
  hardware_interface::ComponentInfo joint{};
  joint.name = name;
  joint.type = "joint";

  hardware_interface::InterfaceInfo command{};
  command.name = command_kind;
  if (with_limits) {
    command.min = "-3.14159";
    command.max = "3.14159";
  }
  joint.command_interfaces.push_back(command);

  if (!state_kind.empty()) {
    hardware_interface::InterfaceInfo state{};
    state.name = state_kind;
    joint.state_interfaces.push_back(state);
  }
  joint.parameters = std::move(parameters);
  return joint;
}

hardware_interface::HardwareComponentInterfaceParams make_params(
  const std::string & name, Params hardware_parameters,
  std::vector<hardware_interface::ComponentInfo> joints)
{
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info.name = name;
  params.hardware_info.type = "system";
  params.hardware_info.hardware_plugin_name = "rak_chassis/CanopenChassis";
  params.hardware_info.hardware_parameters = std::move(hardware_parameters);
  params.hardware_info.joints = std::move(joints);
  return params;
}

Params liveness_params(bool allow_motion)
{
  Params params{
    {"driver_node", "chassis"},
    {"allow_motion", allow_motion ? "true" : "false"},
    {"feedback_timeout_ms", "200"},
    {"cmd_seq_object", "0x2040"},
    {"cmd_flags_object", "0x2041"},
    {"enable_bit", "0"},
  };
  return params;
}

Params wheel_params(
  const std::string & command_object, const std::string & state_object,
  const std::string & sign = "1")
{
  Params params{
    {"command_object", command_object},
    {"command_type", "int16"},
    {"command_scale", "100"},
    {"state_object", state_object},
    {"state_type", "int16"},
    {"state_scale", "0.01"},
    {"sign", sign},
  };
  return params;
}

std::vector<hardware_interface::ComponentInfo> wheel_joints()
{
  static const char * kNames[4] = {
    "front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"};
  std::vector<hardware_interface::ComponentInfo> joints;
  for (int i = 0; i < 4; ++i) {
    char command_object[8];
    char state_object[8];
    std::snprintf(command_object, sizeof(command_object), "0x%04X", 0x2000 + i);
    std::snprintf(state_object, sizeof(state_object), "0x%04X", 0x2010 + i);
    joints.push_back(
      make_joint(
        kNames[i], hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_VELOCITY, wheel_params(command_object, state_object), false));
  }
  return joints;
}

Params arm_joint_params(int index, bool with_state, const std::string & state_type)
{
  char command_object[8];
  char state_object[8];
  std::snprintf(command_object, sizeof(command_object), "0x%04X", 0x2100 + index);
  std::snprintf(state_object, sizeof(state_object), "0x%04X", 0x2110 + index);
  Params params{
    {"command_object", command_object},
    {"command_type", "uint16"},
    {"command_scale", "572.9577951"},
  };
  if (with_state) {
    params["state_object"] = state_object;
    params["state_type"] = state_type;
    params["state_scale"] = "0.0017453293";
  }
  return params;
}

std::vector<hardware_interface::ComponentInfo> arm_joints()
{
  std::vector<hardware_interface::ComponentInfo> joints;
  for (int i = 0; i < 7; ++i) {
    const bool is_gripper = (i == 6);
    // 契约：J1/J2 int16, J3..J6/夹爪 uint16；夹爪无反馈。
    const std::string state_type = (i < 2) ? "int16" : "uint16";
    joints.push_back(
      make_joint(
        "axis_" + std::to_string(i + 1), hardware_interface::HW_IF_POSITION,
        is_gripper ? "" : hardware_interface::HW_IF_POSITION,
        arm_joint_params(i, !is_gripper, state_type), true));
  }
  return joints;
}

Params arm_hardware_params(bool allow_motion)
{
  Params params{
    {"driver_node", "arm"},
    {"allow_motion", allow_motion ? "true" : "false"},
    {"feedback_timeout_ms", "200"},
    {"cmd_seq_object", "0x2140"},
    {"cmd_flags_object", "0x2141"},
    {"enable_bit", "0"},
    {"fault_object", "0x2131"},
  };
  return params;
}

int16_t as_int16(uint32_t data)
{
  return static_cast<int16_t>(data & 0xFFFFU);
}

rak_chassis::JointSpec wheel_spec(double sign = 1.0)
{
  rak_chassis::JointSpec spec;
  spec.name = "front_left_wheel";
  spec.command_object = 0x2000;
  spec.command_type = rak_chassis::ObjectType::kInt16;
  spec.command_scale = 100.0;
  spec.has_state = true;
  spec.state_object = 0x2010;
  spec.state_type = rak_chassis::ObjectType::kInt16;
  spec.state_scale = 0.01;
  spec.sign = sign;
  return spec;
}

rak_chassis::JointSpec arm_spec()
{
  rak_chassis::JointSpec spec;
  spec.name = "j1";
  spec.command_object = 0x2100;
  spec.command_type = rak_chassis::ObjectType::kUint16;
  spec.command_scale = 572.9577951;
  spec.has_state = true;
  spec.state_object = 0x2110;
  spec.state_type = rak_chassis::ObjectType::kInt16;
  spec.state_scale = 0.0017453293;
  spec.sign = 1.0;
  return spec;
}

rak_chassis::JointSpec gripper_spec()
{
  rak_chassis::JointSpec spec;
  spec.name = "gripper";
  spec.command_object = 0x2106;
  spec.command_type = rak_chassis::ObjectType::kUint16;
  spec.command_scale = 572.9577951;
  spec.has_state = false;
  spec.state_scale = 0.0;
  spec.sign = 1.0;
  return spec;
}
}  // namespace

/* ===================== 参数解析：底盘 ===================== */
TEST(CanopenBridgeParameters, ChassisParsesAllObjects)
{
  rak_chassis::CanopenChassis bridge;
  const auto params = make_params("chassis", liveness_params(false), wheel_joints());
  ASSERT_EQ(bridge.on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  EXPECT_FALSE(bridge.allow_motion());
  EXPECT_EQ(bridge.cmd_seq_object(), 0x2040);
  EXPECT_EQ(bridge.cmd_flags_object(), 0x2041);
  ASSERT_EQ(bridge.joints().size(), 4U);
  EXPECT_EQ(bridge.joints()[0].command_object, 0x2000);
  EXPECT_EQ(bridge.joints()[3].command_object, 0x2003);
  EXPECT_EQ(bridge.joints()[0].state_object, 0x2010);
  EXPECT_EQ(bridge.joints()[3].state_object, 0x2013);
  EXPECT_DOUBLE_EQ(bridge.joints()[0].command_scale, 100.0);
  EXPECT_DOUBLE_EQ(bridge.joints()[0].state_scale, 0.01);
  EXPECT_TRUE(bridge.joints()[0].has_state);
}

TEST(CanopenBridgeParameters, ChassisAcceptsAllowMotionTrue)
{
  rak_chassis::CanopenChassis bridge;
  const auto params = make_params("chassis", liveness_params(true), wheel_joints());
  ASSERT_EQ(bridge.on_init(params), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(bridge.allow_motion());
}

TEST(CanopenBridgeParameters, ChassisRejectsBadAllowMotion)
{
  rak_chassis::CanopenChassis bridge;
  Params params = liveness_params(false);
  params["allow_motion"] = "yes";
  EXPECT_EQ(
    bridge.on_init(make_params("chassis", params, wheel_joints())),
    hardware_interface::CallbackReturn::ERROR);
}

TEST(CanopenBridgeParameters, ChassisRejectsZeroCommandScale)
{
  rak_chassis::CanopenChassis bridge;
  auto joints = wheel_joints();
  joints[2].parameters["command_scale"] = "0";
  EXPECT_EQ(
    bridge.on_init(make_params("chassis", liveness_params(false), joints)),
    hardware_interface::CallbackReturn::ERROR);
}

TEST(CanopenBridgeParameters, ChassisRejectsZeroStateScale)
{
  rak_chassis::CanopenChassis bridge;
  auto joints = wheel_joints();
  joints[1].parameters["state_scale"] = "0.0";
  EXPECT_EQ(
    bridge.on_init(make_params("chassis", liveness_params(false), joints)),
    hardware_interface::CallbackReturn::ERROR);
}

TEST(CanopenBridgeParameters, ChassisRejectsMissingStateObject)
{
  rak_chassis::CanopenChassis bridge;
  auto joints = wheel_joints();
  joints[0].parameters.erase("state_object");
  EXPECT_EQ(
    bridge.on_init(make_params("chassis", liveness_params(false), joints)),
    hardware_interface::CallbackReturn::ERROR);
}

TEST(CanopenBridgeParameters, ChassisRejectsWrongJointCount)
{
  rak_chassis::CanopenChassis bridge;
  auto joints = wheel_joints();
  joints.pop_back();
  EXPECT_EQ(
    bridge.on_init(make_params("chassis", liveness_params(false), joints)),
    hardware_interface::CallbackReturn::ERROR);
}

TEST(CanopenBridgeParameters, ChassisRejectsMissingDriverNode)
{
  rak_chassis::CanopenChassis bridge;
  Params params = liveness_params(false);
  params.erase("driver_node");
  EXPECT_EQ(
    bridge.on_init(make_params("chassis", params, wheel_joints())),
    hardware_interface::CallbackReturn::ERROR);
}

TEST(CanopenBridgeParameters, ChassisRejectsBadObjectIndex)
{
  rak_chassis::CanopenChassis bridge;
  Params params = liveness_params(false);
  params["cmd_seq_object"] = "0xZZZZ";
  EXPECT_EQ(
    bridge.on_init(make_params("chassis", params, wheel_joints())),
    hardware_interface::CallbackReturn::ERROR);
}

TEST(CanopenBridgeParameters, ChassisRejectsWrongCommandInterface)
{
  rak_chassis::CanopenChassis bridge;
  auto joints = wheel_joints();
  joints[0].command_interfaces[0].name = hardware_interface::HW_IF_POSITION;
  EXPECT_EQ(
    bridge.on_init(make_params("chassis", liveness_params(false), joints)),
    hardware_interface::CallbackReturn::ERROR);
}

TEST(CanopenBridgeParameters, ChassisRejectsBadSign)
{
  rak_chassis::CanopenChassis bridge;
  auto joints = wheel_joints();
  joints[0].parameters["sign"] = "2";
  EXPECT_EQ(
    bridge.on_init(make_params("chassis", liveness_params(false), joints)),
    hardware_interface::CallbackReturn::ERROR);
}

/* ===================== 参数解析：机械臂 ===================== */
TEST(CanopenBridgeParameters, ArmParsesSixStatesAndGripperWithoutState)
{
  rak_chassis::CanopenArm bridge;
  ASSERT_EQ(
    bridge.on_init(make_params("arm", arm_hardware_params(false), arm_joints())),
    hardware_interface::CallbackReturn::SUCCESS);

  ASSERT_EQ(bridge.joints().size(), 7U);
  for (std::size_t i = 0; i < 6U; ++i) {
    EXPECT_TRUE(bridge.joints()[i].has_state) << i;
    EXPECT_EQ(bridge.joints()[i].state_object, static_cast<uint16_t>(0x2110U + i));
  }
  EXPECT_FALSE(bridge.joints()[6].has_state);
  EXPECT_EQ(bridge.joints()[6].command_object, 0x2106);
  EXPECT_EQ(bridge.joints()[0].state_type, rak_chassis::ObjectType::kInt16);
  EXPECT_EQ(bridge.joints()[2].state_type, rak_chassis::ObjectType::kUint16);
}

TEST(CanopenBridgeParameters, ArmRejectsStateObjectWithoutInterface)
{
  rak_chassis::CanopenArm bridge;
  auto joints = arm_joints();
  /* 夹爪声明了 state_object，但 URDF 没给状态接口 → 自相矛盾。 */
  joints[6].parameters["state_object"] = "0x2116";
  joints[6].parameters["state_type"] = "uint16";
  joints[6].parameters["state_scale"] = "0.0017453293";
  EXPECT_EQ(
    bridge.on_init(make_params("arm", arm_hardware_params(false), joints)),
    hardware_interface::CallbackReturn::ERROR);
}

TEST(CanopenBridgeParameters, ArmRejectsMissingLimits)
{
  rak_chassis::CanopenArm bridge;
  auto joints = arm_joints();
  joints[3].command_interfaces[0].min.clear();
  EXPECT_EQ(
    bridge.on_init(make_params("arm", arm_hardware_params(false), joints)),
    hardware_interface::CallbackReturn::ERROR);
}

TEST(CanopenBridgeParameters, ArmRejectsMissingFaultObjectValue)
{
  rak_chassis::CanopenArm bridge;
  Params params = arm_hardware_params(false);
  params["fault_object"] = "not-an-index";
  EXPECT_EQ(
    bridge.on_init(make_params("arm", params, arm_joints())),
    hardware_interface::CallbackReturn::ERROR);
}

/* ===================== 量化：安全门 ===================== */
TEST(CanopenQuantization, MotionDisabledYieldsZero)
{
  const auto spec = wheel_spec();
  const auto encoded = rak_chassis::encode_command(spec, 5.0, /*allow_motion=*/false);
  EXPECT_EQ(encoded.status, rak_chassis::CommandStatus::kMotionDisabled);
  EXPECT_EQ(encoded.data, 0U);

  const auto arm = rak_chassis::encode_command(arm_spec(), 1.5, /*allow_motion=*/false);
  EXPECT_EQ(arm.status, rak_chassis::CommandStatus::kMotionDisabled);
  EXPECT_EQ(arm.data, 0U);
}

/* ===================== 量化：轮速 ===================== */
TEST(CanopenQuantization, WheelVelocityCounts)
{
  const auto spec = wheel_spec();
  const auto forward = rak_chassis::encode_command(spec, 1.5, true);
  ASSERT_EQ(forward.status, rak_chassis::CommandStatus::kOk);
  EXPECT_EQ(as_int16(forward.data), 150);

  const auto backward = rak_chassis::encode_command(spec, -0.25, true);
  ASSERT_EQ(backward.status, rak_chassis::CommandStatus::kOk);
  EXPECT_EQ(as_int16(backward.data), -25);
}

TEST(CanopenQuantization, WheelSignFlipsCommandAndFeedback)
{
  const auto spec = wheel_spec(/*sign=*/-1.0);
  const auto encoded = rak_chassis::encode_command(spec, 1.5, true);
  ASSERT_EQ(encoded.status, rak_chassis::CommandStatus::kOk);
  EXPECT_EQ(as_int16(encoded.data), -150);

  /* 反馈同符号取反：100 count = 1.0 rad/s → -1.0 */
  EXPECT_DOUBLE_EQ(rak_chassis::decode_state(spec, 100U), -1.0);
}

TEST(CanopenQuantization, WheelFeedbackDecode)
{
  const auto spec = wheel_spec();
  EXPECT_DOUBLE_EQ(rak_chassis::decode_state(spec, 100U), 1.0);
  EXPECT_DOUBLE_EQ(
    rak_chassis::decode_state(spec, static_cast<uint32_t>(static_cast<uint16_t>(-250))),
    -2.5);
}

/* ===================== 量化：机械臂 ===================== */
TEST(CanopenQuantization, ArmRadiansToTenthDegree)
{
  /* 1 rad = 57.29578° = 572.957795 count */
  const auto one_rad = rak_chassis::encode_command(arm_spec(), 1.0, true);
  ASSERT_EQ(one_rad.status, rak_chassis::CommandStatus::kOk);
  EXPECT_EQ(one_rad.data, 573U);

  /* 0.1 rad ≈ 57.3 count */
  const auto small = rak_chassis::encode_command(arm_spec(), 0.1, true);
  ASSERT_EQ(small.status, rak_chassis::CommandStatus::kOk);
  EXPECT_EQ(small.data, 57U);
}

TEST(CanopenQuantization, ArmTenthDegreeToRadians)
{
  const auto spec = arm_spec();
  /* int16 反馈 -1 count = -0.1° ≈ -0.0017453293 rad */
  EXPECT_NEAR(
    rak_chassis::decode_state(spec, static_cast<uint32_t>(static_cast<uint16_t>(-1))),
    -0.0017453293, 1e-9);
  /* uint16 反馈 2250 count = 225.0° */
  auto unsigned_spec = spec;
  unsigned_spec.state_type = rak_chassis::ObjectType::kUint16;
  EXPECT_NEAR(rak_chassis::decode_state(unsigned_spec, 2250U), 3.92699118, 1e-6);
}

/* ===================== 量化：越界拒绝（不饱和） ===================== */
TEST(CanopenQuantization, Int16OverflowRejected)
{
  const auto spec = wheel_spec();
  /* 400 rad/s × 100 = 40000 > 32767 */
  const auto encoded = rak_chassis::encode_command(spec, 400.0, true);
  EXPECT_EQ(encoded.status, rak_chassis::CommandStatus::kOutOfRange);
  EXPECT_EQ(encoded.data, 0U);
}

TEST(CanopenQuantization, Uint16NegativeRejected)
{
  /* 臂的负角旧事故：uint16 对象收到负 count 必须显式拒绝。 */
  const auto encoded = rak_chassis::encode_command(arm_spec(), -0.1, true);
  EXPECT_EQ(encoded.status, rak_chassis::CommandStatus::kOutOfRange);
  EXPECT_EQ(encoded.data, 0U);

  const auto gripper = rak_chassis::encode_command(gripper_spec(), -0.05, true);
  EXPECT_EQ(gripper.status, rak_chassis::CommandStatus::kOutOfRange);
  EXPECT_EQ(gripper.data, 0U);
}

TEST(CanopenQuantization, NonFiniteRejected)
{
  const auto spec = wheel_spec();
  for (double value : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    const auto encoded = rak_chassis::encode_command(spec, value, true);
    EXPECT_EQ(encoded.status, rak_chassis::CommandStatus::kNonFinite);
    EXPECT_EQ(encoded.data, 0U);
  }
}
