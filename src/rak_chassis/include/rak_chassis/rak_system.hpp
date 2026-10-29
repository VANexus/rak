#ifndef RAK_CHASSIS__RAK_SYSTEM_HPP_
#define RAK_CHASSIS__RAK_SYSTEM_HPP_

#include "rak_chassis/session.hpp"

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace rak_chassis
{
/* [AI] 2026-10-29 双下位机拆分：同一块 C30D 板上的两个角色已经分成**两块板**
 * （底盘板 / 机械臂板），Jetson 侧却在一个 SystemInterface 里同时管 4 轮 + 7 轴。
 * 后果是故障域耦合：底盘板反馈超时 -> read() ERROR -> CM 停用整个 hardware ->
 * 臂控制器一起被 deactivate（哪怕臂板还在正常上行）。
 *
 * 现在按 role 拆成两个 <ros2_control> 段：
 *   kChassis -> 4 轮（0x030/0x200-202/0x220/0x032）
 *   kArm     -> 7 轴（0x031/0x210/0x211/0x221/0x222）
 * 逻辑仍在同一份实现里（靠 role_ 分派），但**导出的接口、on_init 校验、read/write
 * 的失败判据都按板隔离**。两个组件共享同一个 CanSession（can_session_pool.hpp）。 */
enum class Role
{
  kChassis,
  kArm,
};

class RakSystemHardware : public hardware_interface::SystemInterface
{
public:
  RakSystemHardware() = default;
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

protected:
  /* 只有两个薄子类会走这里（见文件末尾），外部一律用默认构造 = 底盘角色。 */
  explicit RakSystemHardware(Role role)
  : role_(role) {}

private:
  /* [AI] 激活时等舵机回读（SERVO_STATE_A/B @50Hz，无需先发臂命令），把 6 轴
     回读同步进 command/sent 基准——首帧因此恒等于当前姿态，不产生位移。
     超时返回 false 且 arm_ready_ 保持 false：臂接口静默（轮子不受影响）。 */
  bool sync_arm_from_feedback(std::chrono::milliseconds timeout);

  /* [AI] ADR-0016: 链路唯一实现是 CAN v4 —— 无 transport 开关、无串口 device 参数。
     下位机适配器 = CanSession，上层经 ros2_control 接口说话，无第二条链路。 */
  std::string can_channel_{"can0"};
  std::array<std::string, 4> wheel_names_{};
  std::array<double, 4> wheel_command_{};
  std::array<double, 4> wheel_position_{};
  std::array<double, 4> wheel_velocity_{};
  /* [AI] 每轮极性（接线/编码器安装方向的装配事实，不是运动学参数）。
     +1 = 与固件线上的约定一致；-1 = 该轮的命令与反馈同时取反，使 ROS 的
     "正速度 = 向前滚" 与该轮的物理方向一致。URDF wheel_sign 参数配置，
     代码兜底全 +1（中性：本层不写死某台车的接线）。
     2026-10-14 实车标定本机为 -1 1 -1 1（左侧 FL/RL 反接），真源在
     src/rak_description/urdf/rak_chassis.urdf.xacro 的 wheel_sign 默认值。
     顺序 = [FL, FR, RL, RR]，与 wheel_names_ 一致。 */
  std::array<double, 4> wheel_sign_{1.0, 1.0, 1.0, 1.0};
  /* [AI] ADR-0012 待闭合项 #1：7 轴臂位置接口（6 定位轴 + 夹爪）。
     名称与行程按 docs/arm_moveit_contract.md §1（FROZEN）：命令域是舵机指令角
     （rad，0..270°，Home=0/225/270/0/0/0），**不是** rak_arm.urdf.xacro 的 q_urdf。
     arm_command_: 控制器写入的目标；arm_sent_: 已交给 Session 的目标（事件驱动基准，
     禁止周期重放旧目标）。
     arm_position_: 固件回读。A2（2026-10-28）起 J1/J2 是增量编码器电机——回读是
     **真测量**；J3..J6 仍是舵机脉宽回声（±1°，非物理测量），夹爪无反馈。 */
  static constexpr std::size_t kArmAxes = 7U;
  static constexpr std::size_t kArmPoseJoints = 6U;  // 夹爪无反馈，不进 state
  std::array<std::string, kArmAxes> arm_names_{};
  std::array<double, kArmAxes> arm_command_{};
  std::array<double, kArmAxes> arm_sent_{};
  std::array<double, kArmPoseJoints> arm_position_{};
  /* [AI] 安全门（PROTOCOL v4 §10.1 #7：固件对舵机不做任何门控，许可全在本层）：
     激活时舵机回读未就绪则整个生命周期都不发臂命令——避免控制器拿陈旧零位当
     "当前位"后把臂按到 0。atomic：激活线程与 CM 更新线程都会碰它。 */
  std::atomic<bool> arm_ready_{false};
  /* [AI] 臂板（0x031/0x221）健康门：收到臂板故障（STATE 坏消息位 / MODE∈{2,3}）后
     抑制臂命令（舵机 J3..J6 无门控，只有本层能拦住）。未收到过臂板诊断时保持 true
     —— 不给不发 0x221 的旧固件制造假阴性。 */
  std::atomic<bool> arm_node_ok_{true};
  bool arm_fault_reported_{false};   // [AI] 臂板故障只在进入时报一次
  bool arm_online_reported_{false};  // [AI] 臂板上线只在进入时报一次
  bool arm_nan_reported_{false};  // [AI] 非法臂指令只在由好变坏时报一次
  uint32_t host_build_id_{};
  bool allow_motion_{};  // [AI] 运动许可：默认 false，只有 URDF 显式配置 true 才允许 ARMED
  bool feedback_seen_{};  // [AI] 启动宽限标记：首次收到有效反馈前置 false
  // read() 报错会让 CM 直接停用硬件，但报错本身必须留痕；该位保证每个失效
  // 片段只记一条，反馈恢复后复位，便于反复出现时逐次记录。
  bool feedback_loss_reported_{};
  /* [AI] 共享会话，所有权在 can_session_pool（引用计数归零才真正 stop）。 */
  std::shared_ptr<Session> session_;
  Role role_{Role::kChassis};
  /* [AI] 本板是否已经“在线过一次”：两板各自用自己的首帧时刻作启动宽限，
   * 不再互相借道（以前臂板宽限要看底盘的 feedback_seen_）。 */
  bool board_seen_{false};
};

/* ---- 两个 plugin 名 = 两个 <ros2_control> 段 = 两个故障域 ---- */
class RakChassisHardware final : public RakSystemHardware
{
public:
  RakChassisHardware()
  : RakSystemHardware(Role::kChassis) {}
};

class RakArmHardware final : public RakSystemHardware
{
public:
  RakArmHardware()
  : RakSystemHardware(Role::kArm) {}
};
}  // [AI] 命名空间结束

#endif  // [AI] 头文件保护结束
