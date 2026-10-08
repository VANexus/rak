#include "rak_hardware/rak_system.hpp"

#include "rak_hardware/can_session.hpp"
#include "rak_hardware/serial_session.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace rak_hardware
{
hardware_interface::CallbackReturn RakSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams &params)
{
  /* [AI] lyrical API: HardwareInfo 改由 params 携带 */
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  const hardware_interface::HardwareInfo &info = params.hardware_info;

  static const std::array<std::string, 4> required_wheel_names{
    "front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"};
  if (info.joints.size() != required_wheel_names.size()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0U; i < required_wheel_names.size(); ++i) {
    const auto &joint = info.joints[i];
    if (joint.name != required_wheel_names[i] ||
      joint.command_interfaces.size() != 1U ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY ||
      joint.state_interfaces.size() != 2U ||
      joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
      joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    wheel_names_[i] = joint.name;
  }

  /* [AI] R49: transport=usb|can（默认 usb，行为与 R48 之前完全一致）；
     can 时 device 参数不要求串口 by-id 路径，改用 can_channel（默认 can0） */
  const auto transport = info.hardware_parameters.find("transport");
  transport_ =
    (transport != info.hardware_parameters.end() && transport->second == "can") ?
    Transport::kCan : Transport::kUsb;
  const auto device = info.hardware_parameters.find("device");
  const auto build_id = info.hardware_parameters.find("host_build_id");
  if (transport_ == Transport::kUsb) {
    if (device == info.hardware_parameters.end() ||
      device->second.rfind("/dev/serial/by-id/", 0U) != 0U ||
      build_id == info.hardware_parameters.end())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    if (build_id == info.hardware_parameters.end()) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto can_ch = info.hardware_parameters.find("can_channel");
    can_channel_ = (can_ch != info.hardware_parameters.end()) ? can_ch->second : "can0";
  }
  /* [AI] 运动许可默认关闭：allow_motion=false 时永远只发 DISARMED 零目标 */
  allow_motion_ = false;
  const auto motion = info.hardware_parameters.find("allow_motion");
  if (motion != info.hardware_parameters.end()) {
    allow_motion_ = motion->second == "true" || motion->second == "1";
  }
  try {
    size_t parsed = 0U;
    const unsigned long value = std::stoul(build_id->second, &parsed, 0);
    if (parsed != build_id->second.size() ||
      value > std::numeric_limits<uint32_t>::max())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    host_build_id_ = static_cast<uint32_t>(value);
  } catch (...) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (transport_ == Transport::kUsb) {
    device_path_ = device->second;
  }
  wheel_command_.fill(0.0);
  wheel_position_.fill(0.0);
  wheel_velocity_.fill(0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RakSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(8U);
  for (size_t i = 0U; i < wheel_names_.size(); ++i) {
    interfaces.emplace_back(
      wheel_names_[i], hardware_interface::HW_IF_POSITION, &wheel_position_[i]);
    interfaces.emplace_back(
      wheel_names_[i], hardware_interface::HW_IF_VELOCITY, &wheel_velocity_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
RakSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(4U);
  for (size_t i = 0U; i < wheel_names_.size(); ++i) {
    interfaces.emplace_back(
      wheel_names_[i], hardware_interface::HW_IF_VELOCITY, &wheel_command_[i]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn RakSystemHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  feedback_seen_ = false;
  if (session_) {
    session_->stop();
  }
  /* [AI] R49: 传输后端工厂——上层协议完全一致，仅换底层链路 */
  if (transport_ == Transport::kCan) {
    session_ = std::make_unique<CanSession>(can_channel_, host_build_id_);
  } else {
    session_ = std::make_unique<SerialSession>(device_path_, host_build_id_);
  }
  session_->start();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RakSystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  wheel_command_.fill(0.0);
  if (session_) {
    session_->set_wheel_command(wheel_command_);
    /* [AI] 仅当 on_init 时显式 allow_motion=true 才在激活时使能运动；
     * 断链后 session 内部会撤销使能，重新激活前不会自动恢复 */
    session_->set_motion_enabled(allow_motion_);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RakSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  wheel_command_.fill(0.0);
  if (session_) {
    session_->set_wheel_command(wheel_command_);
    session_->set_motion_enabled(false);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RakSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (session_) {
    session_->stop();
    session_.reset();
  }
  wheel_command_.fill(0.0);
  wheel_position_.fill(0.0);
  wheel_velocity_.fill(0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RakSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!session_) {
    wheel_position_.fill(0.0);
    wheel_velocity_.fill(0.0);
    return hardware_interface::return_type::ERROR;
  }
  const SessionSnapshot state = session_->snapshot();
  /* [AI] R37 联测对齐：不再要求 feedback_valid_mask==0x0F（固件 kick 学习
   * 上电前 mask 可为 0，学习本身依赖 ARMED 指令——按 mask 卡反馈会死锁）。
   * 反馈有效性只看 valid + 新鲜度；mask 保留为诊断信息。 */
  if (!state.connected || !state.handshaken || !state.feedback_valid ||
    state.faulted ||
    state.fault_bitmap != 0U || state.feedback_age > std::chrono::milliseconds(120))
  {
    wheel_position_.fill(0.0);
    wheel_velocity_.fill(0.0);
    if (feedback_seen_) {
      /* [AI] 曾有有效反馈后失效：撤销运动使能并上报错误（恢复后不自动重新使能） */
      session_->set_motion_enabled(false);
      return hardware_interface::return_type::ERROR;
    }
    /* [AI] 启动宽限期：握手未完成（F407 侧 ~100 ms）时返回零值，不让 CM 去激活硬件 */
    return hardware_interface::return_type::OK;
  }
  feedback_seen_ = true;
  wheel_position_ = state.wheel_position;
  wheel_velocity_ = state.wheel_velocity;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RakSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!session_) {
    wheel_command_.fill(0.0);
    return hardware_interface::return_type::ERROR;
  }
  if (!std::all_of(wheel_command_.begin(), wheel_command_.end(),
    [](double command) {return std::isfinite(command);}))
  {
    wheel_command_.fill(0.0);
    session_->set_wheel_command(wheel_command_);
    return hardware_interface::return_type::ERROR;
  }

  session_->set_wheel_command(wheel_command_);
  const SessionSnapshot state = session_->snapshot();
  if (state.connected && state.handshaken && !state.faulted) {
    return hardware_interface::return_type::OK;
  }
  /* [AI] 握手未完成时宽容（首次激活 ~100 ms）；已握手后断链/故障才报错 */
  return feedback_seen_ ? hardware_interface::return_type::ERROR
                        : hardware_interface::return_type::OK;
}
}  // [AI] 命名空间结束

PLUGINLIB_EXPORT_CLASS(rak_hardware::RakSystemHardware, hardware_interface::SystemInterface)
