// Copyright 2026 RAK project
// [AI] 传输层抽象——rak_chassis 与下位机的会话接口。
// [AI] ADR-0016: Session 接口的唯一实现是 CAN v4（CanSession）。上层
// （RakSystemHardware / ros2_control）只经 ros2_control 接口与本 API 说话，不感知链路
// （协议事实源：../vehical_rak_chassis/docs/PROTOCOL.md v4）。
#ifndef RAK_CHASSIS__SESSION_HPP_
#define RAK_CHASSIS__SESSION_HPP_

#include <array>
#include <chrono>
#include <cstdint>

namespace rak_chassis
{

// [AI] 会话快照：position/velocity 单位恒为 rad（接口契约，见 ADR-0016：链路唯一实现
// 是 CanSession，上层零改动）。
struct SessionSnapshot
{
  bool connected{};
  bool handshaken{};
  bool feedback_valid{};
  bool faulted{true};
  uint8_t feedback_valid_mask{};
  uint32_t accepted_features{};
  uint32_t boot_id{};
  // [AI] 底盘板（0x220 DIAG）的 STATE 坏消息位掩码；0 = 健康。
  uint32_t fault_bitmap{};
  // [AI] 底盘板（0x030 NODE_INFO）的 FAULT_u16（0=无故障；当前固件只在欠压时给 0x30）。
  // 历史上该值曾被写进 fault_bitmap（与 0x220 互相覆盖，1 Hz vs 10 Hz 竞态）——
  // 2026-10-28 拆成独立字段，两者都为空才算健康（消费端不得只看其中一个）。
  uint16_t chassis_fault_code{};
  uint32_t encoder_epoch{};
  uint64_t sample_time_us{};
  std::array<double, 4> wheel_position{};
  std::array<double, 4> wheel_velocity{};
  // [AI] 冻结版补齐：舵机实际角读回（rad，J1..J6+夹爪）——CAN=SERVO_STATE_A/B @50Hz。
  // servo_valid_mask bit0..5=J1..J6 读回有效；bit6 夹爪无反馈
  // 通道恒 0（协议保留位，固件遥测恒发 0，消费端不得当真实反馈用）。
  std::array<double, 7> joint_position{};
  // [AI] A2（2026-10-28）起 arm 板 J1/J2 换成增量编码器电机：bit0/bit1 是**真测量**
  // （编码器回读，0.0059°/count）；bit2..bit5（J3..J6）仍是舵机脉宽回声（±1°，
  // 非物理测量）。bit6 夹爪无反馈通道，恒无效。消费端对 bit2..5 不得做碰撞/卡死检测。
  uint8_t servo_valid_mask{};
  std::chrono::milliseconds feedback_age{std::chrono::milliseconds::max()};
  // ---- [AI] 机械臂板上行（双板架构，2026-10-28）：0x031 ARM_NODE_INFO / 0x221 ARM_DIAG ----
  // 这两帧只描述机械臂板，与底盘板的 fault_bitmap/chassis_fault_code 互不覆盖。
  bool arm_online{};        // 收到过 PROTO=4 的 ARM_NODE_INFO，且臂帧未超时
  uint8_t arm_hw_rev{};     // ARM_NODE_INFO.HWREV_u8
  uint16_t arm_build_tag{}; // ARM_NODE_INFO.BUILD_u16（build_tag 两字节 LE，"A2"→0x3241）
  uint16_t arm_state{};     // 臂板 STATE_u16 位图（state_bits 语义同底盘板）
  uint16_t arm_fault{};     // 臂板 FAULT_u16（A2 恒 0；J1/J2 位置环故障走 STATE.bit2）
  uint8_t arm_mode{};       // 臂板 mode_enum（0 IDLE/1 REMOTE/2 ESTOP/3 FAULT）
  uint8_t arm_servo_errcnt{};  // ARM_DIAG.SERVO_ERRCNT_u8（饱和 255）
  bool arm_faulted{};       // 臂板 STATE 坏消息位或 MODE∈{2,3}（最后一次 ARM_DIAG/INFO）
  std::chrono::milliseconds arm_feedback_age{std::chrono::milliseconds::max()};

  // ---- [AI] 2026-10-29 IMU 两帧（0x032 底盘 / 0x222 机械臂板，纯加法）----
  // 前提：三颗 IMU 全在车体上。刚体角速度与安装位置无关 →
  //   arm_gz_rad_s 与 (imu_dyaw_rad / 帧周期) 是**同源冗余**，消费端可互校。
  // 加速度受安装位置影响，只给模值/冲击。权威姿态仍是 /imu（LSM6DSR）。
  bool imu_valid{};                        // 最近 100 ms 内收到过 0x032
  double imu_dyaw_rad{};                   // 上一帧区间的偏航增量（非累计）
  uint32_t imu_dyaw_frames{};              // 增量帧计数（消费端用它自己的 dt 积分）
  double imu_pitch_rad{};
  double imu_roll_rad{};
  uint8_t imu_events{};                    // kImuEvt*（impact/slip/still/saturated/stale）
  std::chrono::milliseconds imu_age{std::chrono::milliseconds::max()};
  bool arm_imu_valid{};                    // 最近 200 ms 内收到过 0x222
  double arm_gz_rad_s{};
  uint16_t arm_amag_mg{};
  uint8_t arm_imu_events{};
  std::chrono::milliseconds arm_imu_age{std::chrono::milliseconds::max()};
};

// [AI] 传输无关的会话接口。实现必须线程安全：控制线程调 set_*，
// IO 线程收发，snapshot() 任意线程可调。
class Session
{
public:
  virtual ~Session() = default;

  Session(const Session &) = delete;
  Session &operator=(const Session &) = delete;

  virtual void start() = 0;
  virtual void stop() = 0;
  // [AI] 轮速目标（rad/s，四轮 FL/FR/RL/RR）；实现内部按协议量化
  virtual void set_wheel_command(const std::array<double, 4> &command) = 0;
  // [AI] 运动许可：true→START/使能指令流，false→STOP/零目标
  virtual void set_motion_enabled(bool enabled) = 0;
  // [AI] 冻结版补齐：臂 7 轴位置目标（rad，J1..J6+夹爪，行程 0..4.712 rad）。
  // 实现内部按协议量化（CAN=0.1° u16 @0x110/111）。安全契约（PROTOCOL
  // v4 §10.1 #7）：固件对舵机命令不做远程门/急停门控，许可与发送时机由本层负责——
  // 实现必须仅在 motion_enabled 下发送，事件驱动（有新目标才发），禁止周期重放旧目标。
  virtual void set_arm_command(const std::array<double, 7> &command)
  {
    (void)command;
  }
  virtual SessionSnapshot snapshot() const = 0;

protected:
  Session() = default;
};

}  // namespace rak_chassis

#endif  // RAK_CHASSIS__SESSION_HPP_
