#ifndef RAK_HARDWARE__RAK_SYSTEM_HPP_
#define RAK_HARDWARE__RAK_SYSTEM_HPP_

#include "rak_hardware/serial_session.hpp"

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace rak_hardware
{
class RakSystemHardware final : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::return_type read(
    const rclcpp::Time &time, const rclcpp::Duration &period) override;
  hardware_interface::return_type write(
    const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  std::array<std::string, 4> wheel_names_{};
  std::array<double, 4> wheel_command_{};
  std::array<double, 4> wheel_position_{};
  std::array<double, 4> wheel_velocity_{};
  std::string device_path_;
  uint32_t host_build_id_{};
  bool allow_motion_{};  // [AI] 运动许可：默认 false，只有 URDF 显式配置 true 才允许 ARMED
  bool feedback_seen_{};  // [AI] 启动宽限标记：首次收到有效反馈前置 false
  std::unique_ptr<SerialSession> session_;
};
}  // [AI] 命名空间结束

#endif  // [AI] 头文件保护结束
