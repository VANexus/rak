// Copyright 2026 RAK project
// [AI] R49: CAN v4 传输后端（SocketCAN）—— ADR-0016 后为唯一实现，Session 接口的唯一落点。
// 协议：../vehical_rak_chassis/docs/PROTOCOL.md v4 + protocol-signals.yaml。经典帧 1 Mbps，
// 仅 SAFETY_CMD 为 e2e 帧（counter+CRC8/DID 种子），其余 raw8。
#ifndef RAK_CHASSIS__CAN_SESSION_HPP_
#define RAK_CHASSIS__CAN_SESSION_HPP_

#include "rak_chassis/session.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace rak_chassis
{

class CanSession final : public Session
{
public:
  // [AI] channel = SocketCAN 接口名（生产为 "can0"）。bitrate 固定 1 Mbps
  // （由 ip link 配置，本类不负责链路层启停）。
  CanSession(
    std::string channel,
    uint32_t host_build_id,
    std::chrono::milliseconds feedback_timeout = std::chrono::milliseconds(120));
  ~CanSession() override;

  void start() override;
  void stop() override;
  void set_wheel_command(const std::array<double, 4> &command) override;
  void set_arm_command(const std::array<double, 7> &command) override;
  void set_motion_enabled(bool enabled) override;
  SessionSnapshot snapshot() const override;

private:
  void run();
  bool try_open();
  void close_socket();
  bool send_frame(uint32_t can_id, const std::array<uint8_t, 8> &data);
  void send_safety(uint8_t cmd_code);
  void process_rx(const std::array<uint8_t, 8> &data, uint32_t can_id);
  void publish_link_state(bool connected, bool handshaken, bool faulted);

  const std::string channel_;
  const uint32_t host_build_id_;
  const std::chrono::milliseconds feedback_timeout_;
  std::atomic<bool> stopping_{false};
  std::atomic<bool> started_{false};
  std::atomic<bool> fault_latched_{false};
  std::atomic<bool> motion_enabled_{false};
  std::atomic<bool> motion_request_dirty_{true};  // [AI] 使能沿变化待发 START/STOP
  std::array<std::atomic<uint64_t>, 4> latest_command_bits_{};  // [AI] double 位模式原子交换
  // [AI] 冻结版补齐：臂 7 轴目标（rad 的 double 位模式）+ 有新目标待发标志。
  // 发送策略见 session.hpp set_arm_command 注释：仅 motion_enabled 且 dirty，
  // 事件驱动，防"舵机命令周期重放"（固件侧舵机无远程门/急停门控）。
  std::array<std::atomic<uint64_t>, 7> latest_arm_bits_{};
  std::atomic<bool> arm_dirty_{false};
  mutable std::mutex snapshot_mutex_;
  SessionSnapshot snapshot_;
  std::chrono::steady_clock::time_point last_feedback_at_{};
  std::chrono::steady_clock::time_point last_node_info_at_{};
  // [AI] 机械臂板最近一次上行（0x031/0x221）时间戳：arm_online 新鲜度判据用
  std::chrono::steady_clock::time_point last_arm_info_at_{};
  uint8_t safety_counter_{};        // [AI] e2e alive counter 0..15
  int socket_fd_{-1};
  std::thread io_thread_;
};

}  // [AI] 命名空间结束

#endif  // RAK_CHASSIS__CAN_SESSION_HPP_
