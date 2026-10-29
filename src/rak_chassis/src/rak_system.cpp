#include "rak_chassis/rak_system.hpp"

#include "rak_chassis/can_session.hpp"
#include "rak_chassis/can_session_pool.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <thread>
#include <utility>

namespace rak_chassis
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
  /* [AI] ADR-0012 待闭合项 #1（2026-10-14 闭合）：6 定位轴 + 夹爪，顺序固定，
     名称/语义按 docs/arm_moveit_contract.md §1（FROZEN）。 */
  static const std::array<std::string, kArmAxes> required_arm_names{
    "j1_base", "j2_shoulder", "j3_elbow", "j4_forearm",
    "j5_wrist_pitch", "j6_wrist_roll", "gripper_joint"};
  /* [AI] 2026-10-29 拆分：**每段只声明自己那块板的关节**。
   * 以前一段管 11 个关节，导致一个 hardware 号兼两板 —— 故障域耦合成一个。 */
  if (role_ == Role::kChassis) {
    if (info.joints.size() != required_wheel_names.size()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rak_chassis_hw"),
        "chassis <ros2_control> must declare exactly %zu wheel joints, got %zu",
        required_wheel_names.size(), info.joints.size());
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
        RCLCPP_ERROR(
          rclcpp::get_logger("rak_chassis_hw"),
          "wheel joint %zu ('%s') interface layout mismatch", i, joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      wheel_names_[i] = joint.name;
    }
  } else {
    if (info.joints.size() != required_arm_names.size()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rak_arm_hw"),
        "arm <ros2_control> must declare exactly %zu joints, got %zu",
        required_arm_names.size(), info.joints.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    for (size_t i = 0U; i < required_arm_names.size(); ++i) {
      const auto &joint = info.joints[i];
      /* [AI] 6 定位轴: position 命令 + position 回读；夹爪无物理反馈通道
         （契约 §2.3 回读恒 0），故只声明 position 命令、不声明 state。 */
      const bool gripper = (i == kArmPoseJoints);
      const bool state_ok = gripper ?
        joint.state_interfaces.empty() :
        (joint.state_interfaces.size() == 1U &&
        joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION);
      /* [AI] min/max 必填：CM 的 JointSaturationLimiter 靠它限幅。 */
      const bool limits_ok = !joint.command_interfaces.empty() &&
        !joint.command_interfaces[0].min.empty() &&
        !joint.command_interfaces[0].max.empty();
      if (joint.name != required_arm_names[i] ||
        joint.command_interfaces.size() != 1U ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        !state_ok || !limits_ok)
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("rak_arm_hw"),
          "arm joint %zu ('%s') interface layout mismatch", i, joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      arm_names_[i] = joint.name;
    }
  }

  /* [AI] ADR-0016: 传输层只有 CAN v4 —— 无 transport 开关、无串口 device 参数。
     必需参数：host_build_id（会话握手）、can_channel（可选，默认 can0）。 */
  const auto build_id = info.hardware_parameters.find("host_build_id");
  if (build_id == info.hardware_parameters.end()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto can_ch = info.hardware_parameters.find("can_channel");
  can_channel_ = (can_ch != info.hardware_parameters.end()) ? can_ch->second : "can0";
  /* [AI] 运动许可默认关闭：allow_motion=false 时永远只发 DISARMED 零目标 */
  allow_motion_ = false;
  const auto motion = info.hardware_parameters.find("allow_motion");
  if (motion != info.hardware_parameters.end()) {
    allow_motion_ = motion->second == "true" || motion->second == "1";
  }
  /* [AI] 每轮极性（URDF wheel_sign，空格分隔 4 个数，顺序 [FL,FR,RL,RR]）。
     这是装配事实：某轮的电机/编码器装向与其余轮相反时，在本层把该轮的
     命令和反馈同时取反，使 ROS 侧 "正速度 = 向前滚" 成立。缺省 = 全 +1
     （中性兜底；本机实测值 -1 1 -1 1 由 URDF 默认值给出，不在本层硬编码）；
     非法值（数量不对/非有限/为 0）直接拒绝激活，不静默回退——极性错一半
     比不改更危险，宁可起不来。 */
  wheel_sign_.fill(1.0);
  const auto sign = info.hardware_parameters.find("wheel_sign");
  if (sign != info.hardware_parameters.end()) {
    std::istringstream ss(sign->second);
    std::string tok;
    size_t n = 0U;
    while (ss >> tok && n < wheel_sign_.size()) {
      try {
        size_t parsed = 0U;
        const double v = std::stod(tok, &parsed);
        if (parsed != tok.size() || !std::isfinite(v) || v == 0.0) {
          return hardware_interface::CallbackReturn::ERROR;
        }
        wheel_sign_[n++] = v;
      } catch (...) {
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    if (n != wheel_sign_.size() || (ss >> tok)) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(
      rclcpp::get_logger("rak_chassis_system"),
      "wheel_sign = [%g %g %g %g] (FL FR RL RR)",
      wheel_sign_[0], wheel_sign_[1], wheel_sign_[2], wheel_sign_[3]);
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
  wheel_command_.fill(0.0);
  wheel_position_.fill(0.0);
  wheel_velocity_.fill(0.0);
  arm_command_.fill(0.0);
  arm_sent_.fill(0.0);
  arm_position_.fill(0.0);
  arm_ready_.store(false, std::memory_order_relaxed);
  arm_node_ok_.store(true, std::memory_order_relaxed);
  arm_fault_reported_ = false;
  arm_online_reported_ = false;
  arm_nan_reported_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RakSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  if (role_ == Role::kChassis) {
    /* 4 轮：position + velocity（velocity 由固件 0x202 直接给）。 */
    interfaces.reserve(8U);
    for (size_t i = 0U; i < wheel_names_.size(); ++i) {
      interfaces.emplace_back(
        wheel_names_[i], hardware_interface::HW_IF_POSITION, &wheel_position_[i]);
      interfaces.emplace_back(
        wheel_names_[i], hardware_interface::HW_IF_VELOCITY, &wheel_velocity_[i]);
    }
    return interfaces;
  }
  /* [AI] 6 定位轴回读（= 固件 servo_get_pulse，非物理测量）；夹爪无反馈通道
     （契约 §2.3 回读恒 0），刻意不导出 state 接口——URDF 同样不声明。 */
  interfaces.reserve(kArmPoseJoints);
  for (size_t i = 0U; i < kArmPoseJoints; ++i) {
    interfaces.emplace_back(
      arm_names_[i], hardware_interface::HW_IF_POSITION, &arm_position_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
RakSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  if (role_ == Role::kChassis) {
    interfaces.reserve(wheel_names_.size());
    for (size_t i = 0U; i < wheel_names_.size(); ++i) {
      interfaces.emplace_back(
        wheel_names_[i], hardware_interface::HW_IF_VELOCITY, &wheel_command_[i]);
    }
    return interfaces;
  }
  /* [AI] 7 轴臂位置目标（指令角域 rad；Session 层饱和 + URDF min/max 限幅）。 */
  interfaces.reserve(kArmAxes);
  for (size_t i = 0U; i < kArmAxes; ++i) {
    interfaces.emplace_back(
      arm_names_[i], hardware_interface::HW_IF_POSITION, &arm_command_[i]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn RakSystemHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  feedback_seen_ = false;
  board_seen_ = false;
  if (session_) {
    release_session(can_channel_);
    session_.reset();
  }
  /* [AI] 2026-10-29：不再自己 new CanSession —— 底盘与臂两个组件**共享同一个
   * 会话**（can_session_pool）。各开一个 socket 会把每条帧解码两遍，
   * 而 Jetson 的 mttcan qlen 只有 10。 */
  SessionConfig config;
  config.channel = can_channel_;
  config.host_build_id = host_build_id_;
  config.allow_motion = allow_motion_;
  std::string error;
  session_ = acquire_session(config, error);
  if (!session_) {
    RCLCPP_ERROR(
      rclcpp::get_logger(role_ == Role::kChassis ? "rak_chassis_hw" : "rak_arm_hw"),
      "cannot acquire CAN session: %s", error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RakSystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!session_) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  /* [AI] 仅当 on_init 时显式 allow_motion=true 才在激活时使能运动；
   * 断链后 session 内部会撤销使能，重新激活前不会自动恢复。
   * 两段都调一次是幂等的（会话级开关，不是组件级）。 */
  session_->set_motion_enabled(allow_motion_);

  if (role_ == Role::kChassis) {
    wheel_command_.fill(0.0);
    session_->set_wheel_command(wheel_command_);
    RCLCPP_INFO(
      rclcpp::get_logger("rak_chassis_hw"),
      "chassis interfaces active (4 wheels, allow_motion=%d)",
      static_cast<int>(allow_motion_));
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  arm_ready_.store(false, std::memory_order_relaxed);
  /* [AI] 臂安全门（契约 §2.1 启动顺序 + PROTOCOL v4 §10.1 #7）：固件对舵机零门控，
   * 许可与发送时机全在上位机。激活时先等 SERVO_STATE 回读（@50Hz 自发，无需先发臂
   * 命令）并把 command/sent 基准同步成当前姿态——首帧因此恒等于当前位、不产生位移；
   * 也让"控制器拿陈旧零位当当前位"永远没有发送窗口（否则臂会被按到 0）。
   * 固件缺席时最多等 1.5s（臂板 NODE_INFO 是 1 Hz，留两帧），然后臂接口整轮保持静默。 */
  if (sync_arm_from_feedback(std::chrono::milliseconds(1500))) {
    arm_ready_.store(true, std::memory_order_relaxed);
    RCLCPP_INFO(
      rclcpp::get_logger("rak_arm_hw"),
      "arm feedback ready: 7 arm position interfaces armed%s",
      allow_motion_ ? "" : " (allow_motion=false -> arm frames stay suppressed)");
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("rak_arm_hw"),
      "arm feedback not ready within 1500ms: the 7 arm position interfaces "
      "will not send any command this activation (chassis unaffected)");
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool RakSystemHardware::sync_arm_from_feedback(std::chrono::milliseconds timeout)
{
  if (!session_) {
    return false;
  }
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const SessionSnapshot state = session_->snapshot();
    /* [AI] 2026-10-29 拆分：判据从“底盘握手 + 底盘反馈新鲜”改为**臂板自己在线**。
     * 以前这里借底盘的 handshaken/feedback_valid 作前提，臂板单独在线时反而等不到。 */
    if (state.connected && state.arm_online &&
      (state.servo_valid_mask & 0x3FU) == 0x3FU)
    {
      for (size_t i = 0U; i < kArmPoseJoints; ++i) {
        arm_position_[i] = state.joint_position[i];
        arm_command_[i] = state.joint_position[i];
        arm_sent_[i] = state.joint_position[i];
      }
      /* [AI] 夹爪无反馈通道：从 0 起步（协议域 0 = 张开位），与回读恒 0 一致 */
      arm_command_[kArmPoseJoints] = 0.0;
      arm_sent_[kArmPoseJoints] = 0.0;
      arm_nan_reported_ = false;
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

hardware_interface::CallbackReturn RakSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (role_ == Role::kChassis) {
    wheel_command_.fill(0.0);
    if (session_) {
      session_->set_wheel_command(wheel_command_);
      session_->set_motion_enabled(false);
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  /* [AI] 臂停用**不撤整机使能**：底盘导航可能正在跑，臂停用只意味着“不再允许
   * 下发臂目标”。arm_ready_=false 已阻断 write() 的臂分支，固件侧臂保持当前目标。 */
  arm_ready_.store(false, std::memory_order_relaxed);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RakSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (session_) {
    /* [AI] 只归还引用：另一个组件可能还在用同一条总线（pool 计数归零才 stop）。 */
    release_session(can_channel_);
    session_.reset();
  }
  wheel_command_.fill(0.0);
  wheel_position_.fill(0.0);
  wheel_velocity_.fill(0.0);
  arm_command_.fill(0.0);
  arm_sent_.fill(0.0);
  arm_position_.fill(0.0);
  arm_ready_.store(false, std::memory_order_relaxed);
  arm_node_ok_.store(true, std::memory_order_relaxed);
  arm_fault_reported_ = false;
  arm_online_reported_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RakSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!session_) {
    if (role_ == Role::kChassis) {
      wheel_position_.fill(0.0);
      wheel_velocity_.fill(0.0);
    }
    return hardware_interface::return_type::ERROR;
  }
  const SessionSnapshot state = session_->snapshot();

  /* ================= 底盘板（4 轮）= can0 上的 0x030/0x200-202/0x220/0x032 ============ */
  if (role_ == Role::kChassis) {
    /* [AI] R37 联测对齐：不再要求 feedback_valid_mask==0x0F（固件 kick 学习
     * 上电前 mask 可为 0，学习本身依赖 ARMED 指令——按 mask 卡反馈会死锁）。
     * 反馈有效性只看 valid + 新鲜度；mask 保留为诊断信息。 */
    if (!state.connected || !state.handshaken || !state.feedback_valid ||
      state.faulted ||
      state.chassis_fault_code != 0U ||
      /* [AI] ADR-0017: cmd_stale(STATE.bit3=0x0008) 不作反馈失效判据——它描述命令路径/
       * 固件门控状态(协议 §6.2: DRIVE_CMD 断流100ms→IDLE→置位), 不是"反馈可信吗"的证据:
       * 反馈真断了 feedback_age(120ms) 必然先超限。而且它是自锁环的一环——read() ERROR→
       * CM 停用硬件→on_deactivate 关运动使能→该位被本层动作保持→永远过不去, 只能 relaunch。
       * 保留 bat_uv/can_recovering(不由停用引起, 不自锁); estop/fault_active 由 state.faulted
       * 覆盖; kStateFaultMask/fault_bitmap 内容不变(诊断照常发布)。 */
      (state.fault_bitmap & ~0x0008U) != 0U ||
      state.feedback_age > std::chrono::milliseconds(120))
    {
      wheel_position_.fill(0.0);
      wheel_velocity_.fill(0.0);
      if (feedback_seen_) {
        /* [AI] 曾有有效反馈后失效：撤销运动使能并上报错误（恢复后不自动重新使能） */
        session_->set_motion_enabled(false);
        if (!feedback_loss_reported_) {
          feedback_loss_reported_ = true;
          RCLCPP_ERROR(
            rclcpp::get_logger("rak_chassis_hw"),
            "chassis read() lost feedback -> wheels deactivated: connected=%d "
            "handshaken=%d feedback_valid=%d faulted=%d fault_bitmap=0x%04X "
            "chassis_fault=0x%04X feedback_age=%ldms (limit 120ms)",
            static_cast<int>(state.connected), static_cast<int>(state.handshaken),
            static_cast<int>(state.feedback_valid), static_cast<int>(state.faulted),
            state.fault_bitmap, state.chassis_fault_code,
            static_cast<long>(state.feedback_age.count()));
        }
        return hardware_interface::return_type::ERROR;
      }
      /* [AI] 启动宽限期：握手未完成（F407 侧 ~100 ms）时返回零值，不让 CM 去激活硬件 */
      return hardware_interface::return_type::OK;
    }
    feedback_seen_ = true;
    feedback_loss_reported_ = false;
    /* [AI] 与 write() 同一符号约定：命令和反馈一起取反，回环自洽，
       ROS 侧正速度恒等于该轮向前滚。 */
    for (size_t i = 0U; i < wheel_position_.size(); ++i) {
      wheel_position_[i] = wheel_sign_[i] * state.wheel_position[i];
      wheel_velocity_[i] = wheel_sign_[i] * state.wheel_velocity[i];
    }
    return hardware_interface::return_type::OK;
  }

  /* ================= 机械臂板（7 轴）= 0x031/0x210/0x211/0x221/0x222 ============ */
  /* [AI] 臂板健康门（见 rak_system.hpp arm_node_ok_）：臂板 0x221 报故障
   * （STATE 坏消息位 / MODE∈{2,3}）即抑制臂命令；恢复后自动解锁。 */
  {
    const bool arm_ok = !state.arm_faulted;
    if (!arm_ok && !arm_fault_reported_) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rak_arm_hw"),
        "arm board fault -> arm commands suppressed: arm_state=0x%04X arm_fault=0x%04X "
        "arm_mode=%u arm_errcnt=%u (CLEAR_FAULT via SAFETY_CMD cmd=6 after the cause is gone)",
        state.arm_state, state.arm_fault, state.arm_mode, state.arm_servo_errcnt);
    }
    arm_fault_reported_ = !arm_ok;
    arm_node_ok_.store(arm_ok, std::memory_order_relaxed);
    if (state.arm_online && !arm_online_reported_) {
      RCLCPP_INFO(
        rclcpp::get_logger("rak_arm_hw"),
        "arm board online: build_tag='%c%c' (0x%04X) hw_rev=%u arm_state=0x%04X",
        static_cast<char>(state.arm_build_tag & 0xFFU),
        static_cast<char>((state.arm_build_tag >> 8U) & 0xFFU),
        state.arm_build_tag, state.arm_hw_rev, state.arm_state);
    }
    arm_online_reported_ = state.arm_online;
  }
  if (!state.connected) {
    return hardware_interface::return_type::ERROR;
  }
  if (!state.arm_online) {
    /* [AI] 臂板从未上线：启动宽限（activate 已等过 1.5 s），返回 OK 让 CM 别反复重启。
     * 上线过又掉线：返回 ERROR，**只停臂这一路控制器**（故障域隔离的收益就在这）。 */
    if (!board_seen_) {
      return hardware_interface::return_type::OK;
    }
    if (!feedback_loss_reported_) {
      feedback_loss_reported_ = true;
      RCLCPP_ERROR(
        rclcpp::get_logger("rak_arm_hw"),
        "arm board went offline: arm_feedback_age=%ldms (limit 1500ms); "
        "arm controller will be deactivated, chassis untouched",
        static_cast<long>(state.arm_feedback_age.count()));
    }
    return hardware_interface::return_type::ERROR;
  }
  board_seen_ = true;
  feedback_loss_reported_ = false;
  /* [AI] 6 轴臂回读（servo_valid_mask bit0-5；J1/J2 自 A2 起是真测量，J3..J6 是
   * 脉宽回声，契约 §2.2）。回读无效时保持上一拍：清零等于凭空造一个"臂在 0 位"的
   * 姿态，比陈旧值更坏（轮子清零是安全方向，臂不是）。夹爪无反馈通道，不导出 state。 */
  if ((state.servo_valid_mask & 0x3FU) == 0x3FU) {
    for (size_t i = 0U; i < kArmPoseJoints; ++i) {
      arm_position_[i] = state.joint_position[i];
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RakSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!session_) {
    wheel_command_.fill(0.0);
    return hardware_interface::return_type::ERROR;
  }

  /* ================= 底盘板：速度指令 ============ */
  if (role_ == Role::kChassis) {
    if (!std::all_of(wheel_command_.begin(), wheel_command_.end(),
      [](double command) {return std::isfinite(command);}))
    {
      wheel_command_.fill(0.0);
      session_->set_wheel_command(wheel_command_);
      return hardware_interface::return_type::ERROR;
    }
    /* [AI] 只在交线前取反：wheel_command_ 是导出给控制器的命令接口，
       改它会污染控制器看到的"自己的指令"。 */
    std::array<double, 4> wire{};
    for (size_t i = 0U; i < wire.size(); ++i) {
      wire[i] = wheel_sign_[i] * wheel_command_[i];
    }
    session_->set_wheel_command(wire);
    const SessionSnapshot state = session_->snapshot();
    if (state.connected && state.handshaken && !state.faulted) {
      return hardware_interface::return_type::OK;
    }
    /* [AI] 握手未完成时宽容（首次激活 ~100 ms）；已握手后断链/故障才报错 */
    return feedback_seen_ ? hardware_interface::return_type::ERROR
                          : hardware_interface::return_type::OK;
  }

  /* ================= 机械臂板：位置目标（事件驱动） ============ */
  /* [AI] 7 轴臂：事件驱动（契约 §2.1 / PROTOCOL §10.1 #7）——只有目标变化才交给
   * Session，是否真上总线由 Session 在 motion_enabled 下裁决（唯一安全门在底层，
   * 上层无从绕过；也绝不周期重放旧目标）。arm_ready_=false（激活时回读未就绪、
   * 或已停用）或 arm_node_ok_=false（臂板报故障）= 臂整轮静默。 */
  if (arm_ready_.load(std::memory_order_relaxed) &&
    arm_node_ok_.load(std::memory_order_relaxed))
  {
    const bool finite = std::all_of(arm_command_.begin(), arm_command_.end(),
      [](double command) {return std::isfinite(command);});
    if (!finite) {
      /* [AI] 非法臂指令只丢这一拍，不动其它轴。由好变坏时报一次留痕，恢复后复位。 */
      if (!arm_nan_reported_) {
        arm_nan_reported_ = true;
        RCLCPP_ERROR(
          rclcpp::get_logger("rak_arm_hw"),
          "non-finite arm command dropped; arm holds last sent target");
      }
    } else {
      arm_nan_reported_ = false;
      if (!std::equal(arm_command_.begin(), arm_command_.end(), arm_sent_.begin())) {
        session_->set_arm_command(arm_command_);
        arm_sent_ = arm_command_;
      }
    }
  }
  return hardware_interface::return_type::OK;
}
}  // [AI] 命名空间结束

/* [AI] 两个 plugin 名对应两个 <ros2_control> 段（见 rak_chassis_plugin.xml）。 */
PLUGINLIB_EXPORT_CLASS(rak_chassis::RakChassisHardware, hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(rak_chassis::RakArmHardware, hardware_interface::SystemInterface)
