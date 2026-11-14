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

/// OD 数据类型：只支持板上实际用到的两种 16 位类型。
enum class ObjectType
{
  kInt16,
  kUint16,
};

/// 解析 "0x2040" / "2040"（十进制）/ "0X2040" 到 16 位对象索引。
bool parse_object_index(const std::string & text, uint16_t & index);

/// 解析 "int16" / "uint16"。
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
  /// 命令与反馈同时取反（旧 wheel_sign 语义）。
  double sign = 1.0;
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
