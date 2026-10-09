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
  virtual SessionSnapshot snapshot() const = 0;

protected:
  Session() = default;
};

}  // namespace rak_chassis

#endif  // RAK_CHASSIS__SESSION_HPP_
