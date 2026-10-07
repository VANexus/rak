#include "rak_hardware/rak_system.hpp"

#include <gtest/gtest.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <array>
#include <string>

namespace
{
/* [AI] lyrical API: on_init 接受 HardwareComponentInterfaceParams（携带 HardwareInfo） */
hardware_interface::HardwareComponentInterfaceParams make_wheel_hardware_params()
{
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info.name = "rak_wheel_base";
  params.hardware_info.hardware_parameters["device"] = "/dev/serial/by-id/test-controller";
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

TEST(RakSystemHardware, ExportsOnlyFourWheelVelocityCommands)
{
  rak_hardware::RakSystemHardware hardware;
  ASSERT_EQ(hardware.on_init(make_wheel_hardware_params()),
    hardware_interface::CallbackReturn::SUCCESS);
  const auto commands = hardware.export_command_interfaces();
  ASSERT_EQ(commands.size(), 4U);
  for (const auto &command : commands) {
    EXPECT_EQ(command.get_interface_name(), hardware_interface::HW_IF_VELOCITY);
  }
}

TEST(RakSystemHardware, ExportsOnlyWheelPositionAndVelocityState)
{
  rak_hardware::RakSystemHardware hardware;
  ASSERT_EQ(hardware.on_init(make_wheel_hardware_params()),
    hardware_interface::CallbackReturn::SUCCESS);
  const auto states = hardware.export_state_interfaces();
  ASSERT_EQ(states.size(), 8U);
  for (size_t i = 0U; i < 4U; ++i) {
    EXPECT_EQ(states[i * 2U].get_interface_name(), hardware_interface::HW_IF_POSITION);
    EXPECT_EQ(states[i * 2U + 1U].get_interface_name(), hardware_interface::HW_IF_VELOCITY);
  }
}
}  // [AI] 匿名命名空间
