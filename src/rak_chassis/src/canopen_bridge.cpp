#include "rak_chassis/canopen_bridge.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace rak_chassis
{
namespace
{
constexpr std::size_t kChassisJointCount = 4U;
constexpr std::size_t kArmJointCount = 7U;
constexpr long kInt16Min = -32768L;
constexpr long kInt16Max = 32767L;
constexpr long kUint16Min = 0L;
constexpr long kUint16Max = 65535L;
/* lround 超过 long 范围是 UB；超过这个量级的 SI 值必然越界，先拒再取整。 */
constexpr double kQuantizeSanityBound = 1e15;

std::string trimmed(const std::string & text)
{
  const auto first = text.find_first_not_of(" \t");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = text.find_last_not_of(" \t");
  return text.substr(first, last - first + 1);
}

bool parse_double(const std::string & text, double & value)
{
  try {
    std::size_t consumed = 0U;
    const double parsed = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(parsed)) {
      return false;
    }
    value = parsed;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool parse_unsigned(const std::string & text, unsigned long & value)
{
  try {
    std::size_t consumed = 0U;
    const unsigned long parsed = std::stoul(text, &consumed, 0);
    if (consumed != text.size()) {
      return false;
    }
    value = parsed;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}
}  // namespace

bool parse_object_index(const std::string & text, uint16_t & index)
{
  const std::string clean = trimmed(text);
  if (clean.empty()) {
    return false;
  }
  unsigned long value = 0U;
  if (!parse_unsigned(clean, value) || value > std::numeric_limits<uint16_t>::max()) {
    return false;
  }
  index = static_cast<uint16_t>(value);
  return true;
}

bool parse_object_type(const std::string & text, ObjectType & type)
{
  const std::string clean = trimmed(text);
  if (clean == "int16") {
    type = ObjectType::kInt16;
    return true;
  }
  if (clean == "uint16") {
    type = ObjectType::kUint16;
    return true;
  }
  if (clean == "int32") {
    /* [AI] 只用于状态方向（轮位置 0x2020..0x2023 是 int32 encoder counts）。 */
    type = ObjectType::kInt32;
    return true;
  }
  return false;
}

CommandValue encode_command(const JointSpec & spec, double value, bool allow_motion)
{
  if (!allow_motion) {
    return {CommandStatus::kMotionDisabled, 0U};
  }
  if (!std::isfinite(value)) {
    return {CommandStatus::kNonFinite, 0U};
  }
  const double scaled = value * spec.command_scale * spec.sign;
  if (!std::isfinite(scaled) || std::abs(scaled) > kQuantizeSanityBound) {
    return {CommandStatus::kOutOfRange, 0U};
  }
  const long rounded = std::lround(scaled);
  long minimum = kUint16Min;
  long maximum = kUint16Max;
  if (spec.command_type == ObjectType::kInt16) {
    minimum = kInt16Min;
    maximum = kInt16Max;
  } else if (spec.command_type == ObjectType::kInt32) {
    minimum = std::numeric_limits<int32_t>::min();
    maximum = std::numeric_limits<int32_t>::max();
  }
  if (rounded < minimum || rounded > maximum) {
    /* [AI] 越界显式失败：绝不静默饱和（夹爪/负角旧事故的教训）。 */
    return {CommandStatus::kOutOfRange, 0U};
  }
  const auto narrowed =
    spec.command_type == ObjectType::kInt16 ?
    static_cast<uint32_t>(static_cast<uint16_t>(static_cast<int16_t>(rounded))) :
    static_cast<uint32_t>(rounded);
  return {CommandStatus::kOk, static_cast<uint32_t>(narrowed)};
}

double decode_state_raw(ObjectType type, double scale, double sign, uint32_t raw)
{
  double counts = 0.0;
  switch (type) {
    case ObjectType::kInt16:
      counts = static_cast<double>(static_cast<int16_t>(raw & 0xFFFFU));
      break;
    case ObjectType::kUint16:
      counts = static_cast<double>(raw & 0xFFFFU);
      break;
    case ObjectType::kInt32:
      counts = static_cast<double>(static_cast<int32_t>(raw));
      break;
  }
  return counts * scale * sign;
}

double decode_state(const JointSpec & spec, uint32_t raw)
{
  return decode_state_raw(spec.state_type, spec.state_scale, spec.state_sign, raw);
}

std::string CanopenBridge::node_logger_name() const
{
  return kind_ == InterfaceKind::kVelocity ? "rak_chassis_hw" : "rak_arm_hw";
}

hardware_interface::CallbackReturn CanopenBridge::fail(const std::string & message) const
{
  RCLCPP_ERROR(rclcpp::get_logger(node_logger_name()), "%s", message.c_str());
  return hardware_interface::CallbackReturn::ERROR;
}

bool CanopenBridge::parse_hardware_parameters(const hardware_interface::HardwareInfo & info)
{
  const auto & params = info.hardware_parameters;

  const auto driver = params.find("driver_node");
  if (driver == params.end()) {
    (void)fail("<hardware> 参数 driver_node 缺失（chassis 或 arm）");
    return false;
  }
  driver_node_ = trimmed(driver->second);
  if (driver_node_.empty() || driver_node_.find('/') != std::string::npos) {
    (void)fail("driver_node 非法: '" + driver->second + "'（须为 proxy driver 节点名，无 '/'）");
    return false;
  }

  allow_motion_ = false;
  const auto motion = params.find("allow_motion");
  if (motion != params.end()) {
    const std::string value = trimmed(motion->second);
    if (value == "true") {
      allow_motion_ = true;
    } else if (value != "false") {
      (void)fail("allow_motion 只接受 \"true\"/\"false\", got '" + motion->second + "'");
      return false;
    }
  }

  feedback_timeout_ = std::chrono::milliseconds(200);
  const auto grace = params.find("startup_grace_ms");
  if (grace != params.end()) {
    try {
      const auto value = std::stol(grace->second);
      if (value <= 0) {
        (void)fail("startup_grace_ms 必须 > 0: '" + grace->second + "'");
      }
      startup_grace_ = std::chrono::milliseconds(value);
    } catch (const std::exception &) {
      (void)fail("startup_grace_ms 非法: '" + grace->second + "'");
    }
  }
  const auto timeout = params.find("feedback_timeout_ms");
  if (timeout != params.end()) {
    unsigned long parsed = 0U;
    if (!parse_unsigned(trimmed(timeout->second), parsed) || parsed == 0U ||
      parsed > std::numeric_limits<long>::max())
    {
      (void)fail("feedback_timeout_ms 非法: '" + timeout->second + "'");
      return false;
    }
    feedback_timeout_ = std::chrono::milliseconds(static_cast<long>(parsed));
  }

  /* [AI] 2026-11-17：稳定器开关 + 命令周期重发（两个都是硬件参数，默认保守）。 */
  stabilizer_ = false;
  const auto stab = params.find("stabilizer");
  if (stab != params.end()) {
    const std::string value = trimmed(stab->second);
    if (value == "true") {
      stabilizer_ = true;
    } else if (value != "false") {
      (void)fail("stabilizer 只接受 \"true\"/\"false\", got '" + stab->second + "'");
      return false;
    }
  }
  /* [AI] velocity_from_position —— 轮速反馈来源开关（文档齐全，别盲改）。
   *
   * true（当前 URDF rak_chassis.urdf.xacro <ros2_control> 里的字面量）：
   *   轮速 = 0x2020..0x2023（INT32 编码器计数）在 controller_manager 周期
   *   （100 Hz，dt=0.01 s）上的位置差分。为什么不用 0x2010 直读：
   *   板上 0x2010..0x2013（轮速反馈）空闲时会冻结在旧值（2026-11-18 实测），
   *   位置却一路正确 —— 里程计与速度闭环只信可靠的那一路。
   *   代价：差分附带 ±1 count 量化噪声。换算：1 count = 2π/61600 rad =
   *   1.0200e-4 rad，dt=0.01 s → 每轮 σ_ω ≈ 1.02e-2 rad/s；麦轮 FK
   *   vx = r/4·Σω → σ_vx ≈ 1.6e-4 m/s（量化底，真实误差以速度环残差与
   *   打滑为主，见 src/rak_bringup/config/ekf.yaml 的 odom1 协方差推导）。
   *   另一副作用：位置差分出的"轮速"推不出可信的麦轮 yaw —— 打滑本来就让
   *   轮系航向漂移（实测直行 -2.0°/m），EKF 里 yaw/vyaw 已明确关掉。
   *
   * 关掉它的前提（0x2010 根因修复后，另一名 teammate 在查）：
   *   1) 真机验证 0x2010..0x2013 连续 ≥60 s（含静止→匀速→静止）不冻结、
   *      数值与位置差分一致（差 ≤2%）；
   *   2) 把 URDF 里的字面量改成 false 并重编 rak_chassis 插件（改 URDF 后
   *      旧 .so 不认新参数，会静默沿用旧逻辑）；
   *   3) 用 `ros2 topic hz /mecanum_drive_controller/odometry` 与直行实测
   *      复核里程比仍是 1.007 量级，才准恢复默认直读。
   * 关掉后：速度直接读 0x2010（INT16 × state_scale），量化噪声消失，
   * 但 feedback_timeout_/jr 的 aux_state_* 位置导出路径不变（位置仍来自
   * 0x2020，joint_states 不会变 NaN）。 */
  velocity_from_position_ = false;
  const auto vfp = params.find("velocity_from_position");
  if (vfp != params.end()) {
    const std::string value = trimmed(vfp->second);
    if (value == "true") {
      velocity_from_position_ = true;
    } else if (value != "false") {
      (void)fail("velocity_from_position 只接受 \"true\"/\"false\", got '" + vfp->second + "'");
      return false;
    }
  }
  command_refresh_ = std::chrono::milliseconds(250);
  const auto refresh = params.find("command_refresh_ms");
  if (refresh != params.end()) {
    unsigned long parsed = 0U;
    if (!parse_unsigned(trimmed(refresh->second), parsed) || parsed > 10000U) {
      (void)fail("command_refresh_ms 非法（0..10000）: '" + refresh->second + "'");
      return false;
    }
    command_refresh_ = std::chrono::milliseconds(static_cast<long>(parsed));
  }

  for (const auto * key : {"cmd_seq_object", "cmd_flags_object"}) {
    if (params.find(key) == params.end()) {
      (void)fail(std::string("<hardware> 参数 ") + key + " 缺失");
      return false;
    }
  }
  if (!parse_object_index(params.at("cmd_seq_object"), cmd_seq_object_)) {
    (void)fail("cmd_seq_object 非法: '" + params.at("cmd_seq_object") + "'");
    return false;
  }
  if (!parse_object_index(params.at("cmd_flags_object"), cmd_flags_object_)) {
    (void)fail("cmd_flags_object 非法: '" + params.at("cmd_flags_object") + "'");
    return false;
  }

  enable_bit_ = 0U;
  const auto bit = params.find("enable_bit");
  if (bit != params.end()) {
    unsigned long parsed = 0U;
    if (!parse_unsigned(trimmed(bit->second), parsed) || parsed > 15U) {
      (void)fail("enable_bit 非法（0..15）: '" + bit->second + "'");
      return false;
    }
    enable_bit_ = parsed;
  }

  has_fault_object_ = false;
  const auto fault = params.find("fault_object");
  if (fault != params.end() && !trimmed(fault->second).empty()) {
    if (!parse_object_index(fault->second, fault_object_)) {
      (void)fail("fault_object 非法: '" + fault->second + "'");
      return false;
    }
    has_fault_object_ = true;
  }
  return true;
}

bool CanopenBridge::parse_joint(
  const hardware_interface::ComponentInfo & joint, bool state_optional, JointSpec & spec)
{
  const std::string tag = "joint '" + joint.name + "': ";
  spec = JointSpec{};
  spec.name = joint.name;

  const auto get = [&joint](const char * key, bool required, std::string & out) {
      const auto it = joint.parameters.find(key);
      if (it == joint.parameters.end() || trimmed(it->second).empty()) {
        return !required;
      }
      out = trimmed(it->second);
      return true;
    };

  std::string command_object;
  std::string command_type;
  std::string command_scale;
  std::string state_object;
  std::string state_type;
  std::string state_scale;
  std::string sign;

  if (!get("command_object", true, command_object) ||
    !get("command_type", true, command_type) ||
    !get("command_scale", true, command_scale) ||
    !get("state_object", false, state_object) ||
    !get("state_type", false, state_type) ||
    !get("state_scale", false, state_scale) ||
    !get("sign", false, sign))
  {
    (void)fail(tag + "缺少必填 <param>（command_object/command_type/command_scale）");
    return false;
  }

  if (!parse_object_index(command_object, spec.command_object)) {
    (void)fail(tag + "command_object 非法: '" + command_object + "'");
    return false;
  }
  if (!parse_object_type(command_type, spec.command_type)) {
    (void)fail(tag + "command_type 非法（int16|uint16）: '" + command_type + "'");
    return false;
  }
  if (!parse_double(command_scale, spec.command_scale) || spec.command_scale == 0.0) {
    (void)fail(tag + "command_scale 必须是非零有限数: '" + command_scale + "'");
    return false;
  }

  spec.has_state = !state_object.empty();
  if (spec.has_state) {
    if (!parse_object_index(state_object, spec.state_object)) {
      (void)fail(tag + "state_object 非法: '" + state_object + "'");
      return false;
    }
    if (!parse_object_type(state_type, spec.state_type)) {
      (void)fail(tag + "声明了 state_object 但 state_type 缺失或非法: '" + state_type + "'");
      return false;
    }
    if (!parse_double(state_scale, spec.state_scale) || spec.state_scale == 0.0) {
      (void)fail(tag + "state_scale 必须是非零有限数: '" + state_scale + "'");
      return false;
    }
  } else if (!state_optional) {
    (void)fail(tag + "本接口形态要求每个关节都有反馈（state_object 缺失）");
    return false;
  }

  /* [AI] 2026-11-17 第二路状态（轮位置）。三个参数要么都给要么都不给；
     不给时行为与旧版完全一致（只有速度状态）。 */
  spec.has_aux_state = false;
  std::string aux_object, aux_type, aux_scale, aux_iface;
  (void)get("aux_state_object", false, aux_object);
  (void)get("aux_state_type", false, aux_type);
  (void)get("aux_state_scale", false, aux_scale);
  (void)get("aux_state_interface", false, aux_iface);
  const bool aux_any = !aux_object.empty() || !aux_type.empty() ||
    !aux_scale.empty() || !aux_iface.empty();
  if (aux_any) {
    if (aux_object.empty() || aux_type.empty() || aux_scale.empty() || aux_iface.empty()) {
      (void)fail(tag + "aux_state_* 四个参数必须同时给出（object/type/scale/interface）");
      return false;
    }
    if (!parse_object_index(aux_object, spec.aux_state_object)) {
      (void)fail(tag + "aux_state_object 非法: '" + aux_object + "'");
      return false;
    }
    if (!parse_object_type(aux_type, spec.aux_state_type)) {
      (void)fail(tag + "aux_state_type 非法: '" + aux_type + "'");
      return false;
    }
    if (!parse_double(aux_scale, spec.aux_state_scale) || spec.aux_state_scale == 0.0) {
      (void)fail(tag + "aux_state_scale 必须是非零有限数: '" + aux_scale + "'");
      return false;
    }
    const std::string primary_iface =
      kind_ == InterfaceKind::kVelocity ? "velocity" : "position";
    if (aux_iface == primary_iface) {
      (void)fail(tag + "aux_state_interface 不能与主状态接口同名: '" + aux_iface + "'");
      return false;
    }
    spec.aux_state_interface = aux_iface;
    spec.has_aux_state = true;
  }

  spec.sign = 1.0;
  if (!sign.empty()) {
    double parsed = 0.0;
    if (!parse_double(sign, parsed) || (parsed != 1.0 && parsed != -1.0)) {
      (void)fail(tag + "sign 只接受 1 / -1: '" + sign + "'");
      return false;
    }
    spec.sign = parsed;
  }
  spec.state_sign = spec.sign;   // 默认跟随命令符号（旧语义）
  std::string state_sign;
  (void)get("state_sign", false, state_sign);
  if (!state_sign.empty()) {
    double parsed = 0.0;
    if (!parse_double(state_sign, parsed) || (parsed != 1.0 && parsed != -1.0)) {
      (void)fail(tag + "state_sign 只接受 1 / -1: '" + state_sign + "'");
      return false;
    }
    spec.state_sign = parsed;
  }
  return true;
}

hardware_interface::CallbackReturn CanopenBridge::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return fail("基类 on_init 失败");
  }
  const hardware_interface::HardwareInfo & info = params.hardware_info;

  const std::size_t expected =
    kind_ == InterfaceKind::kVelocity ? kChassisJointCount : kArmJointCount;
  if (info.joints.size() != expected) {
    return fail(
      "关节数量错误: 期望 " + std::to_string(expected) + ", 实际 " +
      std::to_string(info.joints.size()));
  }
  if (!parse_hardware_parameters(info)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  /* 两种形态的命令与状态同名：轮 velocity/velocity，臂 position/position。 */
  const std::string iface_kind =
    kind_ == InterfaceKind::kVelocity ? hardware_interface::HW_IF_VELOCITY :
    hardware_interface::HW_IF_POSITION;
  /* velocity 状态必配；position 状态允许缺省（无反馈的轴，如夹爪）。 */
  const bool state_required = kind_ == InterfaceKind::kVelocity;

  joints_.clear();
  joints_.reserve(info.joints.size());
  for (const auto & joint : info.joints) {
    if (joint.command_interfaces.size() != 1U ||
      joint.command_interfaces[0].name != iface_kind)
    {
      return fail(
        "joint '" + joint.name + "': 期望恰好 1 个 " + iface_kind +
        " 命令接口, 实际 " + std::to_string(joint.command_interfaces.size()));
    }
    if (!state_required &&
      (joint.command_interfaces[0].min.empty() || joint.command_interfaces[0].max.empty()))
    {
      /* CM 的 JointSaturationLimiter 依赖 min/max。 */
      return fail("joint '" + joint.name + "': position 命令接口必须声明 min/max");
    }

    JointSpec spec;
    if (!parse_joint(joint, !state_required, spec)) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    /* 状态接口必须与 state_object 声明一致：主状态（velocity/position）有反馈才算 1 个，
       第二路状态（轮 position）有 aux_state_object 才算 1 个。 */
    const std::size_t wanted_states =
      (spec.has_state ? 1U : 0U) + (spec.has_aux_state ? 1U : 0U);
    if (joint.state_interfaces.size() != wanted_states) {
      return fail(
        "joint '" + joint.name + "': 状态接口数量与 state_object 声明不符（期望 " +
        std::to_string(wanted_states) + ", 实际 " +
        std::to_string(joint.state_interfaces.size()) + "）");
    }
    for (const auto & state : joint.state_interfaces) {
      if (state.name != iface_kind &&
        !(spec.has_aux_state && state.name == spec.aux_state_interface))
      {
        return fail("joint '" + joint.name + "': 不支持的状态接口 '" + state.name + "'");
      }
    }
    joints_.push_back(spec);
  }

  command_.assign(joints_.size(), 0.0);
  state_.assign(joints_.size(), 0.0);
  aux_state_.assign(joints_.size(), 0.0);
  prev_aux_state_.assign(joints_.size(), 0.0);
  aux_prev_valid_ = false;
  last_sent_.assign(joints_.size(), std::nullopt);
  range_reported_.assign(joints_.size(), false);

  std::string summary = "joints:";
  for (const auto & spec : joints_) {
    summary += " " + spec.name;
  }
  for (const auto & spec : joints_) {
    RCLCPP_INFO(
      rclcpp::get_logger(node_logger_name()),
      "[dbg] joint=%s sign=%g state_sign=%g cmd_obj=0x%04X state_obj=0x%04X aux_obj=0x%04X",
      spec.name.c_str(), spec.sign, spec.state_sign,
      static_cast<unsigned>(spec.command_object),
      static_cast<unsigned>(spec.state_object),
      static_cast<unsigned>(spec.aux_state_object));
  }
  RCLCPP_INFO(
    rclcpp::get_logger(node_logger_name()),
    "canopen bridge ready: driver_node=%s allow_motion=%s feedback_timeout_ms=%ld %s",
    driver_node_.c_str(), allow_motion_ ? "true" : "false",
    static_cast<long>(feedback_timeout_.count()), summary.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CanopenBridge::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0U; i < joints_.size(); ++i) {
    if (joints_[i].has_state) {
      interfaces.emplace_back(
        joints_[i].name,
        kind_ == InterfaceKind::kVelocity ? hardware_interface::HW_IF_VELOCITY :
        hardware_interface::HW_IF_POSITION,
        &state_[i]);
    }
    /* [AI] 第二路状态（轮位置）：接口名来自 URDF 参数，joint_state_broadcaster
       因此能发布数字位置，robot_state_publisher 不再丢弃车轮 TF。 */
    if (joints_[i].has_aux_state) {
      interfaces.emplace_back(
        joints_[i].name, joints_[i].aux_state_interface, &aux_state_[i]);
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> CanopenBridge::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  const std::string kind_name =
    kind_ == InterfaceKind::kVelocity ? hardware_interface::HW_IF_VELOCITY :
    hardware_interface::HW_IF_POSITION;
  interfaces.reserve(joints_.size());
  for (std::size_t i = 0U; i < joints_.size(); ++i) {
    interfaces.emplace_back(joints_[i].name, kind_name, &command_[i]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn CanopenBridge::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (node_) {
    stop_node();
  }

  rclcpp::NodeOptions options;
  node_ = std::make_shared<rclcpp::Node>("rak_canopen_" + driver_node_, options);
  tpdo_publisher_ = node_->create_publisher<COData>("/" + driver_node_ + "/tpdo", 10);
  /* 反馈用 best-effort + keep-last-1：rpdo 是 100 Hz 的**状态流**，旧的被测值没有
     价值。默认 RELIABLE/深度 10 在控制环偶尔迟滞时会积压十几帧旧值，read() 于是
     读到过期状态（比丢帧更糟）。命令方向（tpdo publisher）保持 RELIABLE——丢一条
     命令就是丢一条命令。 */
  rpdo_subscription_ = node_->create_subscription<COData>(
    "/" + driver_node_ + "/rpdo", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
    [this](const COData::SharedPtr message) {rpdo_callback(message);});

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() {executor_->spin();});

  {
    std::lock_guard<std::mutex> lock(mutex_);
    rpdo_values_.clear();
    last_rpdo_ = Clock::now();
    stale_reported_ = false;
    feedback_seen_ = false;
  }
  cmd_seq_ = 0U;
  std::fill(last_sent_.begin(), last_sent_.end(), std::nullopt);
  std::fill(range_reported_.begin(), range_reported_.end(), false);
  nonfinite_reported_ = false;
  suppress_reported_ = false;

  RCLCPP_INFO(
    node_->get_logger(), "subscribed /%s/rpdo, publishing /%s/tpdo",
    driver_node_.c_str(), driver_node_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanopenBridge::on_activate(
  const rclcpp_lifecycle::State &)
{
  active_ = true;
  /* 重新计时启动宽限：首帧反馈到来前 read() 在超时内返回 OK。 */
  std::lock_guard<std::mutex> lock(mutex_);
  last_rpdo_ = Clock::now();
  feedback_seen_ = false;
  stale_reported_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanopenBridge::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  if (tpdo_publisher_) {
    /* 主动撤使能：flags bit0=0；底盘额外下发一次零轮速（臂固件无门控，
     * 命令帧一律不发，只靠 seq 停流 + bit0=0 让 MCU 进入 deadman）。 */
    publish_object(cmd_flags_object_, 0U);
    if (kind_ == InterfaceKind::kVelocity) {
      for (const auto & spec : joints_) {
        publish_object(spec.command_object, 0U);
      }
    }
    std::fill(last_sent_.begin(), last_sent_.end(), std::nullopt);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanopenBridge::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  stop_node();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    rpdo_values_.clear();
    stale_reported_ = false;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

void CanopenBridge::stop_node()
{
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  if (executor_ && node_) {
    executor_->remove_node(node_);
  }
  executor_.reset();
  rpdo_subscription_.reset();
  tpdo_publisher_.reset();
  node_.reset();
}

void CanopenBridge::rpdo_callback(const COData::SharedPtr message)
{
  if (message->subindex != 0U) {
    /* 契约对象全部在 subindex 0；其余（数组元素等）不属于本桥。 */
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  rpdo_values_[message->index] = message->data;
  last_rpdo_ = Clock::now();
  feedback_seen_ = true;
}

std::optional<uint32_t> CanopenBridge::lookup_locked(uint16_t index) const
{
  const auto it = rpdo_values_.find(index);
  if (it == rpdo_values_.end()) {
    return std::nullopt;
  }
  return it->second;
}

bool CanopenBridge::arm_suppressed() const
{
  if (!has_fault_object_) {
    /* 未配置 fault_object 时保持健康：不给不发故障对象的固件制造假阴性。 */
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto value = lookup_locked(fault_object_);
  return value.has_value() && *value != 0U;
}

hardware_interface::return_type CanopenBridge::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!node_) {
    return hardware_interface::return_type::ERROR;
  }
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  const auto now = Clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    /* 启动宽限：proxy driver 还没 boot 完、从站还没发第一帧 TPDO 时不算故障。
       收到过反馈之后，才按 feedback_timeout_ms 判超时（故障域仍然按板隔离）。 */
    const auto window = feedback_seen_ ? feedback_timeout_ : startup_grace_;
    if (feedback_seen_ && now - last_rpdo_ > feedback_timeout_) {
      /* 故障域按板隔离：本板超时只影响本 hardware 组件。 */
      if (!stale_reported_) {
        stale_reported_ = true;
        RCLCPP_ERROR(
          node_->get_logger(),
          "feedback stale (no /%s/rpdo within %ld ms) -> read() ERROR",
          driver_node_.c_str(), static_cast<long>(feedback_timeout_.count()));
      }
      return hardware_interface::return_type::ERROR;
    }
    if (!feedback_seen_ && now - last_rpdo_ > window) {
      if (!stale_reported_) {
        stale_reported_ = true;
        RCLCPP_ERROR(
          node_->get_logger(),
          "no /%s/rpdo within startup_grace_ms=%ld -> read() ERROR",
          driver_node_.c_str(), static_cast<long>(window.count()));
      }
      return hardware_interface::return_type::ERROR;
    }
    stale_reported_ = false;
    const double dt = aux_prev_valid_ ?
      std::chrono::duration<double>(now - last_aux_time_).count() : 0.0;
    for (std::size_t i = 0U; i < joints_.size(); ++i) {
      if (!joints_[i].has_state) {
        continue;
      }
      const auto value = lookup_locked(joints_[i].state_object);
      if (value.has_value()) {
        state_[i] = decode_state(joints_[i], *value);
      }
      if (joints_[i].has_aux_state) {
        const auto aux = lookup_locked(joints_[i].aux_state_object);
        if (aux.has_value()) {
          aux_state_[i] = decode_state_raw(joints_[i].aux_state_type,
              joints_[i].aux_state_scale, joints_[i].state_sign, *aux);
        }
      }
    }
    /* 速度状态 = 位置差分。dt 不合理（首拍、卡顿、回绕）时不更新，保留上一拍。
       [AI] 2026-11-18 P0-1：此路径的来历、量化噪声量级（±1 count/拍 ≈
       0.01 rad/s @100 Hz）与"0x2010 修好后如何关掉"的完整前提，见上方
       on_init 里 velocity_from_position 参数的注释块。0x2010 直读修好后，
       这里整段按开关自动失效，不需要动代码。 */
    if (velocity_from_position_ && aux_prev_valid_ && dt > 1e-4 && dt < 0.5) {
      for (std::size_t i = 0U; i < joints_.size(); ++i) {
        if (!(joints_[i].has_state && joints_[i].has_aux_state)) {
          continue;
        }
        const double derived = (aux_state_[i] - prev_aux_state_[i]) / dt;
        /* 量化噪声 1 count/拍 = 0.01 rad/s（100 Hz），超过 50 rad/s 只可能是解码
           错位或位置回绕 —— 丢掉这一拍，绝不把尖峰喂给控制器。 */
        if (std::isfinite(derived) && std::abs(derived) < 50.0) {
          state_[i] = derived;
        }
      }
    }
    for (std::size_t i = 0U; i < joints_.size(); ++i) {
      prev_aux_state_[i] = aux_state_[i];
    }
    last_aux_time_ = now;
    aux_prev_valid_ = true;
  }
  return hardware_interface::return_type::OK;
}

void CanopenBridge::publish_object(uint16_t index, uint32_t data)
{
  COData message;
  message.index = index;
  message.subindex = 0U;
  message.data = data;
  tpdo_publisher_->publish(message);
}

hardware_interface::return_type CanopenBridge::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_ || !tpdo_publisher_) {
    return hardware_interface::return_type::OK;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    /* proxy driver 还没 boot 完时 tpdo_transmit 只会刷 "driver not activated" 错误；
       首帧 rpdo 之前不发命令（安全：此时也没有运动许可生效）。 */
    if (!feedback_seen_) {
      return hardware_interface::return_type::OK;
    }
  }

  const bool motion_ok = allow_motion_ && !(kind_ == InterfaceKind::kPosition && arm_suppressed());

  /* liveness 每周期都发：cmd_seq 自增喂 MCU 的 100 ms deadman，cmd_flags 承载
   * 使能位（allow_motion=false 时 bit0=0）。 */
  ++cmd_seq_;
  publish_object(cmd_seq_object_, cmd_seq_);
  uint32_t flags = motion_ok ? (1U << enable_bit_) : 0U;
  /* [AI] bit1 = 轮域陀螺稳定器（只有底盘固件认；臂板无门控不看这个字节）。 */
  if (motion_ok && stabilizer_ && kind_ == InterfaceKind::kVelocity) {
    flags |= 0x0002U;
  }
  publish_object(cmd_flags_object_, flags);

  /* [AI] 命令周期重发：事件驱动只在目标变化时发帧，一旦某一帧 RPDO 丢了
     （主站/驱动重连、总线仲裁输掉），板子的 OD 会停在旧值而主机毫不知情。
     按 command_refresh_ms 把 last_sent_ 清空一次，让本周期无条件重发一遍。 */
  if (command_refresh_.count() > 0) {
    const auto now = Clock::now();
    if (now - last_cmd_publish_ >= command_refresh_) {
      std::fill(last_sent_.begin(), last_sent_.end(), std::nullopt);
      last_cmd_publish_ = now;
    }
  }

  const bool suppressed_now = kind_ == InterfaceKind::kPosition && arm_suppressed();
  if (suppressed_now && !suppress_reported_) {
    suppress_reported_ = true;
    RCLCPP_ERROR(node_->get_logger(), "arm board fault (fault_object != 0) -> commands suppressed");
  } else if (!suppressed_now && suppress_reported_) {
    suppress_reported_ = false;
    RCLCPP_INFO(node_->get_logger(), "arm board fault cleared -> commands allowed");
    /* 故障期间的 command_ 可能已变而 last_sent_ 还是旧值，恢复后强制重发一遍。 */
    std::fill(last_sent_.begin(), last_sent_.end(), std::nullopt);
  }

  for (std::size_t i = 0U; i < joints_.size(); ++i) {
    const CommandValue encoded = encode_command(joints_[i], command_[i], motion_ok);
    switch (encoded.status) {
      case CommandStatus::kMotionDisabled:
        if (kind_ == InterfaceKind::kPosition) {
          /* 臂固件无门控：本层一帧都不发。 */
          continue;
        }
        /* 底盘：下发一次量化后的 0（事件驱动去重后不会重放）。 */
        break;
      case CommandStatus::kNonFinite:
        if (!nonfinite_reported_) {
          nonfinite_reported_ = true;
          RCLCPP_ERROR(
            node_->get_logger(), "non-finite command on '%s' dropped",
            joints_[i].name.c_str());
        }
        continue;
      case CommandStatus::kOutOfRange:
        if (!range_reported_[i]) {
          range_reported_[i] = true;
          RCLCPP_ERROR(
            node_->get_logger(),
            "command on '%s' out of range for object 0x%04X (%s, scale %g) -> frame dropped",
            joints_[i].name.c_str(), joints_[i].command_object,
            joints_[i].command_type == ObjectType::kInt16 ? "int16" : "uint16",
            joints_[i].command_scale);
        }
        continue;
      case CommandStatus::kOk:
        range_reported_[i] = false;
        nonfinite_reported_ = false;
        break;
    }
    if (!last_sent_[i] || *last_sent_[i] != encoded.data) {
      /* 事件驱动：只在目标变化时发（臂禁止周期重放旧目标）。 */
      publish_object(joints_[i].command_object, encoded.data);
      last_sent_[i] = encoded.data;
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace rak_chassis

PLUGINLIB_EXPORT_CLASS(rak_chassis::CanopenChassis, hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(rak_chassis::CanopenArm, hardware_interface::SystemInterface)
