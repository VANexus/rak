/* [AI] 2026-10-29 双下位机拆分后的契约测试。
 *
 * 以前一段 <ros2_control> 管 11 个关节；现在拆成两段 = 两个 plugin：
 *   RakChassisHardware -> 只认 4 轮
 *   RakArmHardware     -> 只认 6 定位轴 + 夹爪
 * 这里钉住的就是“每段只声明自己那块板”，以及导出接口的种类与数量。
 * 拆分的动机（故障域隔离）见 rak_system.hpp 顶部注释。
 */
#include "rak_chassis/rak_system.hpp"

#include <gtest/gtest.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <array>
#include <cstddef>
#include <string>

namespace
{
/* 名称/接口按 docs/arm_moveit_contract.md §1（FROZEN）。 */
constexpr const char * kArmJointNames[7] = {
  "j1_base", "j2_shoulder", "j3_elbow", "j4_forearm",
  "j5_wrist_pitch", "j6_wrist_roll", "gripper_joint"};

constexpr const char * kWheelJointNames[4] = {
  "front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"};

hardware_interface::HardwareComponentInterfaceParams make_wheel_hardware_params()
{
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info.name = "rak_chassis";
  params.hardware_info.hardware_parameters["host_build_id"] = "1380009522";
  params.hardware_info.hardware_parameters["can_channel"] = "can0";
  for (const auto &name : kWheelJointNames) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    hardware_interface::InterfaceInfo command;
    command.name = hardware_interface::HW_IF_VELOCITY;
    joint.command_interfaces.push_back(command);
    hardware_interface::InterfaceInfo position;
    position.name = hardware_interface::HW_IF_POSITION;
    joint.state_interfaces.push_back(position);
    hardware_interface::InterfaceInfo velocity;
    velocity.name = hardware_interface::HW_IF_VELOCITY;
    joint.state_interfaces.push_back(velocity);
    params.hardware_info.joints.push_back(joint);
  }
  return params;
}

hardware_interface::HardwareComponentInterfaceParams make_arm_hardware_params()
{
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info.name = "rak_arm";
  params.hardware_info.hardware_parameters["host_build_id"] = "1380009522";
  params.hardware_info.hardware_parameters["can_channel"] = "can0";
  for (size_t i = 0U; i < 7U; ++i) {
    hardware_interface::ComponentInfo joint;
    joint.name = kArmJointNames[i];
    hardware_interface::InterfaceInfo command;
    command.name = hardware_interface::HW_IF_POSITION;
    /* [AI] min/max 必填：CM 的 JointSaturationLimiter 靠它限幅，缺了会被
       on_init 拒绝（URDF wheels 的 NaN 教训同样适用于臂）。 */
    command.min = "0.0";
    command.max = (i == 1U) ? "4.45059" : "4.71239";  // J2 实测硬件限 255°
    joint.command_interfaces.push_back(command);
    if (i < 6U) {
      // 6 定位轴有回读；夹爪无物理反馈通道（契约 §2.3）→ 不声明 state
      hardware_interface::InterfaceInfo position;
      position.name = hardware_interface::HW_IF_POSITION;
      joint.state_interfaces.push_back(position);
    }
    params.hardware_info.joints.push_back(joint);
  }
  return params;
}

/* ===================== 底盘段 ===================== */

TEST(RakChassisHardware, AcceptsFourWheels)
{
  rak_chassis::RakChassisHardware hardware;
  EXPECT_EQ(hardware.on_init(make_wheel_hardware_params()),
    hardware_interface::CallbackReturn::SUCCESS);
}

// 拆分前那种“一段管 11 关节”的形状必须被拒绝：否则又会回到耦合状态
TEST(RakChassisHardware, RejectsArmJointsInChassisSection)
{
  auto params = make_wheel_hardware_params();
  const auto arm = make_arm_hardware_params();
  for (const auto &joint : arm.hardware_info.joints) {
    params.hardware_info.joints.push_back(joint);
  }
  rak_chassis::RakChassisHardware hardware;
  EXPECT_EQ(hardware.on_init(params), hardware_interface::CallbackReturn::ERROR);
}

TEST(RakChassisHardware, RejectsUnknownWheelName)
{
  auto params = make_wheel_hardware_params();
  params.hardware_info.joints[2U].name = "middle_left_wheel";
  rak_chassis::RakChassisHardware hardware;
  EXPECT_EQ(hardware.on_init(params), hardware_interface::CallbackReturn::ERROR);
}

TEST(RakChassisHardware, ExportsFourVelocityCommandsAndEightStates)
{
  rak_chassis::RakChassisHardware hardware;
  ASSERT_EQ(hardware.on_init(make_wheel_hardware_params()),
    hardware_interface::CallbackReturn::SUCCESS);
  const auto commands = hardware.export_command_interfaces();
  ASSERT_EQ(commands.size(), 4U);
  for (size_t i = 0U; i < 4U; ++i) {
    EXPECT_EQ(commands[i].get_interface_name(), hardware_interface::HW_IF_VELOCITY);
    EXPECT_EQ(commands[i].get_name(),
      std::string(kWheelJointNames[i]) + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  const auto states = hardware.export_state_interfaces();
  ASSERT_EQ(states.size(), 8U);
  for (size_t i = 0U; i < 4U; ++i) {
    EXPECT_EQ(states[i * 2U].get_name(),
      std::string(kWheelJointNames[i]) + "/" + hardware_interface::HW_IF_POSITION);
    EXPECT_EQ(states[i * 2U + 1U].get_name(),
      std::string(kWheelJointNames[i]) + "/" + hardware_interface::HW_IF_VELOCITY);
  }
}

// 底盘段不得导出任何臂接口（拆分的意义就在这）
TEST(RakChassisHardware, ExportsNoArmInterfaces)
{
  rak_chassis::RakChassisHardware hardware;
  ASSERT_EQ(hardware.on_init(make_wheel_hardware_params()),
    hardware_interface::CallbackReturn::SUCCESS);
  for (const auto &state : hardware.export_state_interfaces()) {
    const std::string name = state.get_name();
    EXPECT_EQ(name.rfind("j1_base", 0U), std::string::npos) << name;
    EXPECT_EQ(name.rfind("gripper_joint", 0U), std::string::npos) << name;
  }
  for (const auto &command : hardware.export_command_interfaces()) {
    EXPECT_NE(command.get_interface_name(), hardware_interface::HW_IF_POSITION);
  }
}

/* ===================== 机械臂段 ===================== */

TEST(RakArmHardware, AcceptsSevenAxes)
{
  rak_chassis::RakArmHardware hardware;
  EXPECT_EQ(hardware.on_init(make_arm_hardware_params()),
    hardware_interface::CallbackReturn::SUCCESS);
}

// 臂段里混进轮关节同样不允许
TEST(RakArmHardware, RejectsWheelsInArmSection)
{
  auto params = make_arm_hardware_params();
  const auto wheels = make_wheel_hardware_params();
  for (const auto &joint : wheels.hardware_info.joints) {
    params.hardware_info.joints.push_back(joint);
  }
  rak_chassis::RakArmHardware hardware;
  EXPECT_EQ(hardware.on_init(params), hardware_interface::CallbackReturn::ERROR);
}

TEST(RakArmHardware, RejectsUnknownArmJointName)
{
  auto params = make_arm_hardware_params();
  params.hardware_info.joints[2U].name = "elbow";  // 必须是 j3_elbow
  rak_chassis::RakArmHardware hardware;
  EXPECT_EQ(hardware.on_init(params), hardware_interface::CallbackReturn::ERROR);
}

TEST(RakArmHardware, RejectsArmJointWithoutCommandLimits)
{
  auto params = make_arm_hardware_params();
  params.hardware_info.joints[0U].command_interfaces[0].min = "";  // 缺限幅
  rak_chassis::RakArmHardware hardware;
  EXPECT_EQ(hardware.on_init(params), hardware_interface::CallbackReturn::ERROR);
}

TEST(RakArmHardware, ExportsSevenPositionCommandsAndSixStates)
{
  rak_chassis::RakArmHardware hardware;
  ASSERT_EQ(hardware.on_init(make_arm_hardware_params()),
    hardware_interface::CallbackReturn::SUCCESS);
  const auto commands = hardware.export_command_interfaces();
  ASSERT_EQ(commands.size(), 7U);
  for (size_t i = 0U; i < 7U; ++i) {
    EXPECT_EQ(commands[i].get_interface_name(), hardware_interface::HW_IF_POSITION);
    EXPECT_EQ(commands[i].get_name(),
      std::string(kArmJointNames[i]) + "/" + hardware_interface::HW_IF_POSITION);
  }
  // 4 轮 ×(position+velocity) + 6 个臂定位轴 position；夹爪无 state（回读恒 0）
  const auto states = hardware.export_state_interfaces();
  ASSERT_EQ(states.size(), 6U);
  for (size_t i = 0U; i < 6U; ++i) {
    EXPECT_EQ(states[i].get_interface_name(), hardware_interface::HW_IF_POSITION);
    EXPECT_EQ(states[i].get_name(),
      std::string(kArmJointNames[i]) + "/" + hardware_interface::HW_IF_POSITION);
  }
}

TEST(RakArmHardware, ExportsNoStateForGripper)
{
  rak_chassis::RakArmHardware hardware;
  ASSERT_EQ(hardware.on_init(make_arm_hardware_params()),
    hardware_interface::CallbackReturn::SUCCESS);
  for (const auto &state : hardware.export_state_interfaces()) {
    EXPECT_NE(state.get_name().rfind("gripper_joint/", 0U), 0U);
  }
}

// 臂段不得导出轮接口
TEST(RakArmHardware, ExportsNoWheelInterfaces)
{
  rak_chassis::RakArmHardware hardware;
  ASSERT_EQ(hardware.on_init(make_arm_hardware_params()),
    hardware_interface::CallbackReturn::SUCCESS);
  for (const auto &command : hardware.export_command_interfaces()) {
    EXPECT_EQ(command.get_interface_name(), hardware_interface::HW_IF_POSITION);
  }
  for (const auto &state : hardware.export_state_interfaces()) {
    EXPECT_EQ(state.get_interface_name(), hardware_interface::HW_IF_POSITION);
  }
}
}  // [AI] 匿名命名空间
