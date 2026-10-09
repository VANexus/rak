#ifndef RAK_CHASSIS__SERIAL_SESSION_HPP_
#define RAK_CHASSIS__SERIAL_SESSION_HPP_

#include "rak_chassis/protocol.hpp"
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
class SerialSession final : public Session
{
public:
  explicit SerialSession(
    std::string device_path,
    uint32_t host_build_id,
    std::chrono::milliseconds feedback_timeout = std::chrono::milliseconds(120));
  ~SerialSession() override;

  SerialSession(const SerialSession &) = delete;
  SerialSession &operator=(const SerialSession &) = delete;

  void start() override;
  void stop() override;
  void set_wheel_command(const std::array<double, 4> &command) override;
  void set_arm_command(const std::array<double, 7> &command) override;
  void set_motion_enabled(bool enabled) override;
  SessionSnapshot snapshot() const override;

private:
  void run();
  bool open_device(int &fd);
  void close_device(int fd);
  bool receive_packet(
    int fd, std::vector<uint8_t> &packet, std::vector<uint8_t> &stream_buffer);
  bool process_packet(const std::vector<uint8_t> &packet);
  bool process_frame(const Frame &frame);
  void publish_link_state(bool connected, bool handshaken, bool faulted);
  void revoke_feedback(bool faulted);

  const std::string device_path_;
  const uint32_t host_build_id_;
  const std::chrono::milliseconds feedback_timeout_;
  std::atomic<bool> stopping_{false};
  std::atomic<bool> started_{false};
  std::atomic<bool> fault_latched_{false};
  std::array<std::atomic<uint64_t>, 4> latest_command_bits_{};
  // [AI] 冻结版补齐：臂目标（rad 的 double 位模式）+ 待发标志。v2 备用链路走
  // kTypeArmCommand(0x11)，线格式自 R42 冻结未变；axis_count 固定 6（J1..J6），
  // 夹爪第 7 轴需 GRIPPER_SERVO feature 协商且无硬件通道，v2 备用路不发。
  std::array<std::atomic<uint64_t>, 7> latest_arm_bits_{};
  std::atomic<bool> arm_dirty_{false};
  std::atomic<bool> motion_enabled_{false};
  mutable std::mutex snapshot_mutex_;
  SessionSnapshot snapshot_;
  std::chrono::steady_clock::time_point last_feedback_at_{};
  bool have_rx_sequence_{};
  bool have_sample_time_{};
  uint16_t last_rx_sequence_{};
  uint16_t tx_sequence_{};
  uint32_t current_boot_id_{};
  std::array<uint16_t, 32> sent_drive_sequences_{};
  size_t sent_drive_sequence_count_{};
  size_t sent_drive_sequence_index_{};
  std::thread io_thread_;
};
}  // [AI] 命名空间结束

#endif  // [AI] 头文件保护结束
