// Copyright 2026 RAK project
// [AI] R49: 传输层抽象——rak_chassis 与下位机的会话接口。
// USB v2 串口（SerialSession）与 CAN v4（CanSession）实现同一公开 API，
// 上层（RakSystemHardware / ros2_control）完全无感，按 hardware_parameters
// 的 transport 字段选择后端（协议事实源：../vehical_rak_chassis/docs/PROTOCOL.md v4 §10）。
#ifndef RAK_CHASSIS__SESSION_HPP_
#define RAK_CHASSIS__SESSION_HPP_

#include "rak_chassis/protocol.hpp"

#include <array>
#include <chrono>
#include <cstdint>

namespace rak_chassis
{

// [AI] 会话快照：单位约定与 SerialSession 保持一致（position/velocity = rad），
// 使上层在两种传输之间零改动切换。
struct SessionSnapshot
{
  bool connected{};
  bool handshaken{};
  bool feedback_valid{};
  bool faulted{true};
  uint8_t feedback_valid_mask{};
  uint32_t accepted_features{};
  uint32_t boot_id{};
  uint32_t fault_bitmap{};
  uint32_t encoder_epoch{};
  uint64_t sample_time_us{};
  std::array<double, 4> wheel_position{};
  std::array<double, 4> wheel_velocity{};
  // [AI] 冻结版补齐：舵机实际角读回（rad，J1..J6+夹爪）——CAN=SERVO_STATE_A/B @50Hz，
  // USB v2=ARM_STATE。servo_valid_mask bit0..5=J1..J6 读回有效；bit6 夹爪无反馈
  // 通道恒 0（协议保留位，固件遥测恒发 0，消费端不得当真实反馈用）。
  std::array<double, 7> joint_position{};
  uint8_t servo_valid_mask{};
  std::chrono::milliseconds feedback_age{std::chrono::milliseconds::max()};
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
  // 实现内部按协议量化（CAN=0.1° u16 @0x110/111；v2=µrad i32）。安全契约（PROTOCOL
  // v4 §10.1 #7）：固件对舵机命令不做远程门/急停门控，许可与发送时机由本层负责——
  // 实现必须仅在 motion_enabled 下发送，事件驱动（有新目标才发），禁止周期重放旧目标。
  // 默认空实现：备用传输可不支持（当前 SerialSession 走 v2 0x11 已覆盖，可覆盖）。
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
