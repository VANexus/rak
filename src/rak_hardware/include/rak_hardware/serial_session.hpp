#ifndef RAK_HARDWARE__SERIAL_SESSION_HPP_
#define RAK_HARDWARE__SERIAL_SESSION_HPP_

#include "rak_hardware/protocol.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace rak_hardware
{
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

class SerialSession
{
public:
  explicit SerialSession(
    std::string device_path,
    uint32_t host_build_id,
    std::chrono::milliseconds feedback_timeout = std::chrono::milliseconds(120));
  ~SerialSession();

  SerialSession(const SerialSession &) = delete;
  SerialSession &operator=(const SerialSession &) = delete;

  void start();
  void stop();
  void set_wheel_command(const std::array<double, 4> &command);
  void set_motion_enabled(bool enabled);
  SessionSnapshot snapshot() const;

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
