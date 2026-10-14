#include "rak_chassis/rak_system.hpp"

#include <gtest/gtest.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <array>
#include <string>

namespace
{
/* [AI] lyrical API: on_init 接受 HardwareComponentInterfaceParams（携带 HardwareInfo）。
   4 轮 + 7 臂共 11 关节（ADR-0012 待闭合项 #1），顺序与 URDF <ros2_control> 一致：
   轮在前、臂在后；名称/接口按 docs/arm_moveit_contract.md §1（FROZEN）。 */
constexpr const char * kArmJointNames[7] = {
  "j1_base", "j2_shoulder", "j3_elbow", "j4_forearm",
  "j5_wrist_pitch", "j6_wrist_roll", "gripper_joint"};

hardware_interface::HardwareComponentInterfaceParams make_wheel_hardware_params()
{
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info.name = "rak_wheel_base";
  params.hardware_info.hardware_parameters["host_build_id"] = "1380009522";
  const std::array<std::string, 4> wheel_names{
    "front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"};
  for (const auto &name : wheel_names) {
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

/* [AI] 4 轮 + 7 臂的完整 HardwareInfo（真实 URDF 展开后的形状） */
hardware_interface::HardwareComponentInterfaceParams make_full_hardware_params()
{
  auto params = make_wheel_hardware_params();
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

TEST(RakSystemHardware, RejectsWheelOnlyHardwareInfo)
{
  // [AI] 旧的"只导出四轮"形状已随 ADR-0012 待闭合项 #1 关闭
  rak_chassis::RakSystemHardware hardware;
  EXPECT_EQ(hardware.on_init(make_wheel_hardware_params()),
    hardware_interface::CallbackReturn::ERROR);
}

TEST(RakSystemHardware, RejectsUnknownArmJointName)
{
  auto params = make_full_hardware_params();
  params.hardware_info.joints[4U + 2U].name = "elbow";  // 必须是 j3_elbow
  rak_chassis::RakSystemHardware hardware;
  EXPECT_EQ(hardware.on_init(params), hardware_interface::CallbackReturn::ERROR);
}

TEST(RakSystemHardware, RejectsArmJointWithoutCommandLimits)
{
  auto params = make_full_hardware_params();
  params.hardware_info.joints[4U].command_interfaces[0].min = "";  // 缺限幅
  rak_chassis::RakSystemHardware hardware;
  EXPECT_EQ(hardware.on_init(params), hardware_interface::CallbackReturn::ERROR);
}

TEST(RakSystemHardware, ExportsFourWheelVelocityAndSevenArmPositionCommands)
{
  rak_chassis::RakSystemHardware hardware;
  ASSERT_EQ(hardware.on_init(make_full_hardware_params()),
    hardware_interface::CallbackReturn::SUCCESS);
  const auto commands = hardware.export_command_interfaces();
  ASSERT_EQ(commands.size(), 11U);
  for (size_t i = 0U; i < 4U; ++i) {
    EXPECT_EQ(commands[i].get_interface_name(), hardware_interface::HW_IF_VELOCITY);
  }
  for (size_t i = 4U; i < 11U; ++i) {
    EXPECT_EQ(commands[i].get_interface_name(), hardware_interface::HW_IF_POSITION);
    EXPECT_EQ(commands[i].get_name(),
      std::string(kArmJointNames[i - 4U]) + "/" + hardware_interface::HW_IF_POSITION);
  }
}

TEST(RakSystemHardware, ExportsEightWheelAndSixArmPositionStates)
{
  rak_chassis::RakSystemHardware hardware;
  ASSERT_EQ(hardware.on_init(make_full_hardware_params()),
    hardware_interface::CallbackReturn::SUCCESS);
  const auto states = hardware.export_state_interfaces();
  // 4 轮 ×(position+velocity) + 6 个臂定位轴 position；夹爪无 state（回读恒 0）
  ASSERT_EQ(states.size(), 14U);
  for (size_t i = 0U; i < 4U; ++i) {
    EXPECT_EQ(states[i * 2U].get_interface_name(), hardware_interface::HW_IF_POSITION);
    EXPECT_EQ(states[i * 2U + 1U].get_interface_name(), hardware_interface::HW_IF_VELOCITY);
  }
  for (size_t i = 8U; i < 14U; ++i) {
    EXPECT_EQ(states[i].get_interface_name(), hardware_interface::HW_IF_POSITION);
    EXPECT_EQ(states[i].get_name(),
      std::string(kArmJointNames[i - 8U]) + "/" + hardware_interface::HW_IF_POSITION);
  }
}

TEST(RakSystemHardware, ExportsNoStateForGripper)
{
  rak_chassis::RakSystemHardware hardware;
  ASSERT_EQ(hardware.on_init(make_full_hardware_params()),
    hardware_interface::CallbackReturn::SUCCESS);
  for (const auto &state : hardware.export_state_interfaces()) {
    EXPECT_NE(state.get_name().rfind("gripper_joint/", 0), 0U);
  }
}
}  // [AI] 匿名命名空间
