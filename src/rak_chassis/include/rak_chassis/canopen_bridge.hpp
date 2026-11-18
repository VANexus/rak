#ifndef RAK_CHASSIS__CANOPEN_BRIDGE_HPP_
#define RAK_CHASSIS__CANOPEN_BRIDGE_HPP_

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <canopen_interfaces/msg/co_data.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace rak_chassis
{

/* [AI] 2026-11-14 重写：本包不再拥有协议。板子侧的 CAN 帧、PDO 映射、NMT 全部由
 * ros2_canopen（canopen_core + canopen_proxy_driver）承担；本桥只做三件事：
 *   1. OD 对象 ↔ ros2_control 接口的映射（对象索引**只**来自 URDF 参数，代码不写死）；
 *   2. 量化（SI ↔ count）与符号；
 *   3. 安全门（allow_motion / 故障对象 / 越界拒绝 / 反馈超时按板隔离）。
 * 命令走 <driver_node>/tpdo（proxy driver 转成 RPDO 发给板子），反馈走
 * <driver_node>/rpdo（板子 TPDO 上来的对象）。 */

/// OD 数据类型。kInt32 只用于状态方向（轮位置 0x2020..0x2023 是 int32 counts），
/// 命令方向仍是 16 位（板上速度指令对象就是 INT16）。
enum class ObjectType
{
  kInt16,
  kUint16,
  kInt32,
};

/// 解析 "0x2040" / "2040"（十进制）/ "0X2040" 到 16 位对象索引。
bool parse_object_index(const std::string & text, uint16_t & index);

/// 解析 "int16" / "uint16" / "int32"。
bool parse_object_type(const std::string & text, ObjectType & type);

/// 一个关节（= 一个 OD 对象对）的完整绑定，全部来自 URDF <param>。
struct JointSpec
{
  std::string name;
  uint16_t command_object = 0U;
  ObjectType command_type = ObjectType::kInt16;
  /// 每 1 个 SI 单位对应多少 count（如 100 = 0.01 rad/s 分辨率）。
  double command_scale = 0.0;
  /// 无 state_object 的关节没有反馈（如夹爪）：不导出 state 接口。
  bool has_state = false;
  uint16_t state_object = 0U;
  ObjectType state_type = ObjectType::kInt16;
  /// 每 1 count 对应多少 SI 单位（如 0.01 = 0.01 rad/s 分辨率）。
  double state_scale = 0.0;
  /// 命令取反（线上下发值 × sign）。
  double sign = 1.0;
  /// 反馈取反。**默认跟随 sign**（旧 wheel_sign 语义：命令与反馈同时取反，
  /// 闭环自洽）；单独给出时可以把"命令方向"和"编码器读数方向"分开 ——
  /// 2026-11-17 装车遇到的就是这种：固件指令表与实际接线不一致，左侧两轮
  /// 纯前进命令下反转（车斜着走）。那时只能改命令侧，反馈侧保持原始真值。 */
  double state_sign = 1.0;
  /* [AI] 2026-11-17 第二路状态（轮位置）：四个轮子除了速度反馈，板上还发布
     0x2020..0x2023 的 int32 编码器计数。有它 joint_state_broadcaster 才发布数字
     而不是 NaN（NaN 位置会让 robot_state_publisher 丢弃车轮 TF），
     mecanum_drive_controller 也才能按"位置增量"算里程计而不是积分速度。 */
  bool has_aux_state = false;
  uint16_t aux_state_object = 0U;
  ObjectType aux_state_type = ObjectType::kInt16;
  double aux_state_scale = 0.0;   ///< 每 1 count 对应多少 SI 单位（rad/count）
  std::string aux_state_interface;  ///< 接口名（轮 = "position"）
};

/// 命令编码结果。非 kOk 时 data 恒 0 且**不得下发**。
enum class CommandStatus
{
  kOk,
  kMotionDisabled,  // allow_motion=false：本层拒绝运动
  kNonFinite,       // NaN/Inf
  kOutOfRange,      // 量化后超出对象类型范围（显式拒绝，不饱和）
};

struct CommandValue
{
  CommandStatus status = CommandStatus::kOk;
  uint32_t data = 0U;
};

/// SI 值 → 线上 count。越界/非有限值显式失败，绝不静默饱和。
CommandValue encode_command(const JointSpec & spec, double value, bool allow_motion);

/// 线上 count → SI 值。raw 只取低 16 位，按声明类型做有/无符号解释。
double decode_state(const JointSpec & spec, uint32_t raw);

/// 桥的两种形态：轮速（velocity）与臂角（position）。
enum class InterfaceKind
{
  kVelocity,
  kPosition,
};

class CanopenBridge : public hardware_interface::SystemInterface
{
public:
  CanopenBridge() = default;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// 测试与日志用：解析后的绑定（on_init 之后有效）。
  const std::vector<JointSpec> & joints() const {return joints_;}
  bool allow_motion() const {return allow_motion_;}
  uint16_t cmd_seq_object() const {return cmd_seq_object_;}
  uint16_t cmd_flags_object() const {return cmd_flags_object_;}

protected:
  explicit CanopenBridge(InterfaceKind kind)
  : kind_(kind) {}

private:
  using COData = canopen_interfaces::msg::COData;
  using Clock = std::chrono::steady_clock;

  std::string node_logger_name() const;
  hardware_interface::CallbackReturn fail(const std::string & message) const;
  bool parse_hardware_parameters(const hardware_interface::HardwareInfo & info);
  bool parse_joint(
    const hardware_interface::ComponentInfo & joint, bool state_optional,
    JointSpec & spec);
  void rpdo_callback(const COData::SharedPtr message);
  void publish_object(uint16_t index, uint32_t data);
  void stop_node();
  bool arm_suppressed() const;
  /// 已收到的最新对象值（无则 nullopt）。调用方持锁。
  std::optional<uint32_t> lookup_locked(uint16_t index) const;

  InterfaceKind kind_ = InterfaceKind::kVelocity;
  std::vector<JointSpec> joints_;
  bool allow_motion_ = false;
  std::string driver_node_;
  std::chrono::milliseconds feedback_timeout_{200};
  /* 2026-11-14 台架实测：controller_manager 在 proxy driver 完成 boot（约 0.7 s）之前
     就 activate 硬件，首帧 rpdo 必然晚于 200 ms —— 旧逻辑会让 CM 立刻停用硬件，
     控制器永远起不来。故"首帧之前"用独立的启动宽限；收到过反馈后才是故障判据。 */
  std::chrono::milliseconds startup_grace_{5000};
  bool feedback_seen_ = false;
  uint16_t cmd_seq_object_ = 0U;
  uint16_t cmd_flags_object_ = 0U;
  unsigned enable_bit_ = 0U;
  bool has_fault_object_ = false;
  uint16_t fault_object_ = 0U;

  /* ros2_control 句柄指向的数据。命令由 CM 线程写入，read/write 也在同一线程，
   * 故不加锁；rpdo 状态由订阅回调（spin 线程）写入，用 mutex_ 保护。 */
  std::vector<double> command_;
  std::vector<double> state_;
  /// 第二路状态（轮位置，rad）。无 aux 绑定的关节恒 0 且不导出。
  std::vector<double> aux_state_;
  /// cmd_flags.bit1 = 轮域陀螺稳定器（固件 R74，默认关）。开=用板上 IMU 的
  /// yaw rate 闭环修四轮目标，直走抗偏的直接手段；风险见 docs/CANOPEN-V5.md §5。
  bool stabilizer_ = false;
  /// 速度反馈改用轮位置差分（需要 aux_state）。2026-11-17 装车实测：板上
  /// 0x2010..0x2013 的速度对象会**冻结在旧值**（位置在动、速度仍是上一段的数），
  /// 而 0x2020..0x2023 的编码器位置一路正确。里程计和闭环都只信可靠的那路。
  bool velocity_from_position_ = false;
  std::vector<double> prev_aux_state_;
  Clock::time_point last_aux_time_{};
  bool aux_prev_valid_ = false;
  /// 命令周期性重发周期（0=只用事件驱动）。驱动/主站丢一帧 RPDO 时，
  /// 下一个周期会把目标重新写下去，不必重启 bringup。
  std::chrono::milliseconds command_refresh_{250};
  Clock::time_point last_cmd_publish_{};

  mutable std::mutex mutex_;
  std::unordered_map<uint16_t, uint32_t> rpdo_values_;
  Clock::time_point last_rpdo_{};
  bool stale_reported_ = false;

  uint32_t cmd_seq_ = 0U;
  /// 事件驱动基准：每关节最后一次真正下发的 count（未发过 = nullopt）。
  std::vector<std::optional<uint32_t>> last_sent_;
  std::vector<bool> range_reported_;
  bool nonfinite_reported_ = false;
  bool suppress_reported_ = false;

  bool active_ = false;

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::Publisher<COData>::SharedPtr tpdo_publisher_;
  rclcpp::Subscription<COData>::SharedPtr rpdo_subscription_;
  std::thread spin_thread_;
};

/// 4 轮底盘：velocity 命令 + velocity 状态。
class CanopenChassis : public CanopenBridge
{
public:
  CanopenChassis()
  : CanopenBridge(InterfaceKind::kVelocity) {}
};

/// 7 轴臂：position 命令 + position 状态（无反馈的轴不导出状态）。
class CanopenArm : public CanopenBridge
{
public:
  CanopenArm()
  : CanopenBridge(InterfaceKind::kPosition) {}
};

}  // namespace rak_chassis

#endif  // RAK_CHASSIS__CANOPEN_BRIDGE_HPP_
