#include "rak_hardware/serial_session.hpp"

#include <array>
#include <atomic>
#include <cassert>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <mutex>
#include <poll.h>
#include <pty.h>
#include <string>
#include <sys/stat.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace
{
using namespace std::chrono_literals;

struct Frame
{
  uint8_t type{};
  uint16_t sequence{};
  std::vector<uint8_t> payload;
};

uint16_t crc16(const uint8_t *data, size_t size)
{
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8U;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0U ?
        static_cast<uint16_t>((crc << 1U) ^ 0x1021U) :
        static_cast<uint16_t>(crc << 1U);
    }
  }
  return crc;
}

std::vector<uint8_t> encode_frame(uint8_t type, uint16_t sequence,
  const std::vector<uint8_t> &payload)
{
  std::vector<uint8_t> raw{2U, type,
    static_cast<uint8_t>(sequence), static_cast<uint8_t>(sequence >> 8U),
    static_cast<uint8_t>(payload.size()), static_cast<uint8_t>(payload.size() >> 8U)};
  raw.insert(raw.end(), payload.begin(), payload.end());
  const uint16_t crc = crc16(raw.data(), raw.size());
  raw.push_back(static_cast<uint8_t>(crc));
  raw.push_back(static_cast<uint8_t>(crc >> 8U));

  std::vector<uint8_t> wire(1U, 0U);
  size_t code_index = 0U;
  uint8_t code = 1U;
  for (uint8_t byte : raw) {
    if (byte == 0U) {
      wire[code_index] = code;
      code_index = wire.size();
      wire.push_back(0U);
      code = 1U;
    } else {
      wire.push_back(byte);
      ++code;
      if (code == 0xffU) {
        wire[code_index] = code;
        code_index = wire.size();
        wire.push_back(0U);
        code = 1U;
      }
    }
  }
  wire[code_index] = code;
  wire.push_back(0U);
  return wire;
}

bool decode_frame(const std::vector<uint8_t> &wire, Frame &frame)
{
  if (wire.size() < 2U || wire.back() != 0U) {
    return false;
  }
  std::vector<uint8_t> raw;
  size_t index = 0U;
  while (index + 1U < wire.size()) {
    const uint8_t code = wire[index++];
    if (code == 0U || index + code - 1U > wire.size() - 1U) {
      return false;
    }
    for (uint8_t i = 1U; i < code; ++i) {
      raw.push_back(wire[index++]);
    }
    if (code != 0xffU && index < wire.size() - 1U) {
      raw.push_back(0U);
    }
  }
  if (raw.size() < 8U || raw[0] != 2U) {
    return false;
  }
  const size_t payload_size = static_cast<size_t>(raw[4]) |
    (static_cast<size_t>(raw[5]) << 8U);
  if (raw.size() != payload_size + 8U) {
    return false;
  }
  const uint16_t expected = static_cast<uint16_t>(raw[raw.size() - 2U]) |
    static_cast<uint16_t>(raw.back()) << 8U;
  if (crc16(raw.data(), raw.size() - 2U) != expected) {
    return false;
  }
  frame.type = raw[1];
  frame.sequence = static_cast<uint16_t>(raw[2]) |
    static_cast<uint16_t>(raw[3]) << 8U;
  frame.payload.assign(raw.begin() + 6, raw.end() - 2);
  return true;
}

void put_u16(std::vector<uint8_t> &bytes, uint16_t value)
{
  bytes.push_back(static_cast<uint8_t>(value));
  bytes.push_back(static_cast<uint8_t>(value >> 8U));
}

void put_u32(std::vector<uint8_t> &bytes, uint32_t value)
{
  for (unsigned shift = 0; shift < 32; shift += 8) {
    bytes.push_back(static_cast<uint8_t>(value >> shift));
  }
}

void put_u64(std::vector<uint8_t> &bytes, uint64_t value)
{
  for (unsigned shift = 0; shift < 64; shift += 8) {
    bytes.push_back(static_cast<uint8_t>(value >> shift));
  }
}

void put_i64(std::vector<uint8_t> &bytes, int64_t value)
{
  put_u64(bytes, static_cast<uint64_t>(value));
}

void put_i32(std::vector<uint8_t> &bytes, int32_t value)
{
  put_u32(bytes, static_cast<uint32_t>(value));
}

bool wait_for(const std::function<bool()> &predicate, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(2ms);
  }
  return predicate();
}

struct Pty
{
  int master{-1};
  int slave{-1};
  std::string directory;
  std::string stable_path;

  Pty()
  {
    char dir_template[] = "/tmp/rak-hardware-pty-XXXXXX";
    char *dir = mkdtemp(dir_template);
    assert(dir != nullptr);
    directory = dir;
    stable_path = directory + "/wheel-controller";
    assert(openpty(&master, &slave, nullptr, nullptr, nullptr) == 0);
    termios settings{};
    assert(tcgetattr(slave, &settings) == 0);
    cfmakeraw(&settings);
    assert(tcsetattr(slave, TCSANOW, &settings) == 0);
    assert(symlink(tty_name().c_str(), stable_path.c_str()) == 0);
    assert(fcntl(master, F_SETFL, O_NONBLOCK) == 0);
  }

  std::string tty_name() const
  {
    char name[256]{};
    assert(ttyname_r(slave, name, sizeof(name)) == 0);
    return name;
  }

  ~Pty()
  {
    if (master >= 0) {
      close(master);
    }
    if (slave >= 0) {
      close(slave);
    }
    unlink(stable_path.c_str());
    rmdir(directory.c_str());
  }
};

class F407Peer
{
public:
  explicit F407Peer(int master, uint32_t features = 0x07U, uint32_t boot_id = 0x12345678U)
  : master_(master), features_(features), boot_id_(boot_id)
  {
    thread_ = std::thread([this]() { run(); });
  }

  ~F407Peer()
  {
    stopping_.store(true);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  void pause_feedback(bool value)
  {
    pause_feedback_.store(value);
  }

  void corrupt_next_state()
  {
    corrupt_next_state_.store(true);
  }

  void inject_malformed_frame()
  {
    inject_malformed_.store(true);
  }

  uint32_t requested_features() const
  {
    return requested_features_.load();
  }

  uint32_t host_build_id() const
  {
    return host_build_id_.load();
  }

  unsigned handshake_count() const
  {
    return handshake_count_.load();
  }

  unsigned drive_commands() const
  {
    return drive_commands_.load();
  }

  unsigned arm_commands() const
  {
    return arm_commands_.load();
  }

  bool only_disarmed_zero_commands() const
  {
    return only_disarmed_zero_commands_.load();
  }

private:
  void send(const std::vector<uint8_t> &bytes)
  {
    std::lock_guard<std::mutex> lock(send_mutex_);
    size_t offset = 0U;
    while (offset < bytes.size() && !stopping_.load()) {
      const ssize_t count = write(master_, bytes.data() + offset, bytes.size() - offset);
      if (count > 0) {
        offset += static_cast<size_t>(count);
      } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
        pollfd descriptor{master_, POLLOUT, 0};
        poll(&descriptor, 1, 10);
      } else {
        return;
      }
    }
  }

  void send_ack()
  {
    std::vector<uint8_t> payload;
    put_u16(payload, 0x0407U);
    put_u32(payload, 0x00010100U);  /* [AI] 规格语义：固件自身版本 v1.1.0（高16位非协议号） */
    put_u32(payload, boot_id_);
    put_u32(payload, features_);
    send(encode_frame(0x02U, outgoing_sequence_++, payload));
  }

  void send_state(uint16_t echoed_sequence)
  {
    std::vector<uint8_t> payload;
    put_u16(payload, echoed_sequence);
    payload.push_back(0U);
    payload.push_back(0x0fU);
    put_u32(payload, 0U);
    put_u32(payload, 7U);
    put_u64(payload, sample_time_us_);
    put_i64(payload, 1000000);
    put_i64(payload, -2000000);
    put_i64(payload, 3000000);
    put_i64(payload, -4000000);
    put_i32(payload, 500000);
    put_i32(payload, -600000);
    put_i32(payload, 700000);
    put_i32(payload, -800000);
    sample_time_us_ += 10000U;
    auto wire = encode_frame(0x20U, outgoing_sequence_++, payload);
    if (corrupt_next_state_.exchange(false)) {
      wire[wire.size() / 2U] ^= 0x40U;
    }
    send(wire);
  }

  void process(const std::vector<uint8_t> &wire)
  {
    Frame frame;
    if (!decode_frame(wire, frame)) {
      return;
    }
    if (frame.type == 0x01U && frame.payload.size() == 8U) {
      host_build_id_.store(static_cast<uint32_t>(frame.payload[0]) |
        static_cast<uint32_t>(frame.payload[1]) << 8U |
        static_cast<uint32_t>(frame.payload[2]) << 16U |
        static_cast<uint32_t>(frame.payload[3]) << 24U);
      requested_features_.store(static_cast<uint32_t>(frame.payload[4]) |
        static_cast<uint32_t>(frame.payload[5]) << 8U |
        static_cast<uint32_t>(frame.payload[6]) << 16U |
        static_cast<uint32_t>(frame.payload[7]) << 24U);
      handshake_count_.fetch_add(1U);
      send_ack();
    } else if (frame.type == 0x10U && frame.payload.size() == 20U) {
      drive_commands_.fetch_add(1U);
      bool disarmed_zero = frame.payload[0] == 0U &&
        frame.payload[3] == 0U;
      for (size_t i = 4U; i < frame.payload.size(); ++i) {
        disarmed_zero = disarmed_zero && frame.payload[i] == 0U;
      }
      only_disarmed_zero_commands_.store(
        only_disarmed_zero_commands_.load() && disarmed_zero);
      if (!pause_feedback_.load()) {
        send_state(frame.sequence);
      }
    } else if (frame.type == 0x11U) {
      arm_commands_.fetch_add(1U);
    }
  }

  void run()
  {
    std::vector<uint8_t> packet;
    uint8_t buffer[256];
    while (!stopping_.load()) {
      if (inject_malformed_.exchange(false)) {
        auto malformed = encode_frame(0x20U, outgoing_sequence_++, std::vector<uint8_t>(68U, 0U));
        malformed[malformed.size() / 2U] ^= 0x40U;
        send(malformed);
      }
      pollfd descriptor{master_, POLLIN, 0};
      const int ready = poll(&descriptor, 1, 5);
      if (ready <= 0 || (descriptor.revents & POLLIN) == 0) {
        continue;
      }
      const ssize_t count = read(master_, buffer, sizeof(buffer));
      if (count <= 0) {
        continue;
      }
      for (ssize_t i = 0; i < count; ++i) {
        packet.push_back(buffer[i]);
        if (buffer[i] == 0U) {
          process(packet);
          packet.clear();
        } else if (packet.size() > 140U) {
          packet.clear();
        }
      }
    }
  }

  int master_;
  uint32_t features_;
  uint32_t boot_id_;
  uint64_t sample_time_us_{1000000U};
  uint16_t outgoing_sequence_{};
  std::mutex send_mutex_;
  std::atomic<bool> stopping_{false};
  std::atomic<bool> pause_feedback_{false};
  std::atomic<bool> corrupt_next_state_{false};
  std::atomic<bool> inject_malformed_{false};
  std::atomic<uint32_t> requested_features_{0U};
  std::atomic<uint32_t> host_build_id_{0U};
  std::atomic<unsigned> handshake_count_{0U};
  std::atomic<unsigned> drive_commands_{0U};
  std::atomic<unsigned> arm_commands_{0U};
  std::atomic<bool> only_disarmed_zero_commands_{true};
  std::thread thread_;
};

void test_handshake_feedback_and_safe_output()
{
  Pty pty;
  F407Peer peer(pty.master);
  rak_hardware::SerialSession session(pty.stable_path, 0x52414b32U);
  session.start();
  session.set_wheel_command({1.0, -2.0, 3.0, -4.0});

  const bool received_feedback =
    wait_for([&]() { return session.snapshot().feedback_valid; }, 1500ms);
  if (!received_feedback) {
    const auto failed_state = session.snapshot();
    std::fprintf(stderr, "串口连接=%d 握手=%d 故障锁存=%d 特性=%u 轮指令=%u\n",
      failed_state.connected, failed_state.handshaken, failed_state.faulted,
      peer.requested_features(), peer.drive_commands());
  }
  assert(received_feedback);
  const auto state = session.snapshot();
  assert(state.handshaken);
  assert(state.accepted_features == 0x07U);
  assert(state.boot_id == 0x12345678U);
  assert(state.feedback_valid_mask == 0x0fU);
  assert(std::abs(state.wheel_position[0] - 1.0) < 1e-9);
  assert(std::abs(state.wheel_position[1] + 2.0) < 1e-9);
  assert(std::abs(state.wheel_velocity[0] - 0.5) < 1e-9);
  assert(std::abs(state.wheel_velocity[3] + 0.8) < 1e-9);
  assert(wait_for([&]() { return peer.drive_commands() >= 2U; }, 500ms));
  assert(peer.requested_features() == 0x07U);
  assert(peer.host_build_id() == 0x52414b32U);
  assert(peer.only_disarmed_zero_commands());
  assert(peer.arm_commands() == 0U);
  session.stop();
}

void test_missing_required_feature_never_starts_drive()
{
  Pty pty;
  F407Peer peer(pty.master, 0x03U);
  rak_hardware::SerialSession session(pty.stable_path, 0x52414b32U);
  session.start();
  std::this_thread::sleep_for(250ms);
  assert(!session.snapshot().handshaken);
  assert(!session.snapshot().feedback_valid);
  assert(peer.drive_commands() == 0U);
  session.stop();
}

void test_malformed_feedback_and_timeout_revoke_validity()
{
  Pty pty;
  F407Peer peer(pty.master);
  rak_hardware::SerialSession session(pty.stable_path, 0x52414b32U);
  session.start();
  assert(wait_for([&]() { return session.snapshot().feedback_valid; }, 1500ms));
  const unsigned initial_handshakes = peer.handshake_count();

  peer.pause_feedback(true);
  peer.inject_malformed_frame();
  assert(wait_for([&]() { return !session.snapshot().feedback_valid; }, 500ms));
  std::this_thread::sleep_for(200ms);
  assert(!session.snapshot().feedback_valid);

  peer.pause_feedback(false);
  assert(wait_for([&]() { return session.snapshot().feedback_valid; }, 1500ms));
  assert(session.snapshot().faulted);
  assert(peer.handshake_count() > initial_handshakes);
  assert(peer.only_disarmed_zero_commands());
  assert(peer.arm_commands() == 0U);
  session.stop();
}

void test_second_session_cannot_own_the_same_device()
{
  Pty pty;
  F407Peer peer(pty.master);
  rak_hardware::SerialSession first(pty.stable_path, 0x52414b32U);
  rak_hardware::SerialSession second(pty.stable_path, 0x52414b32U);
  first.start();
  assert(wait_for([&]() { return first.snapshot().handshaken; }, 1500ms));
  second.start();
  std::this_thread::sleep_for(200ms);
  assert(!second.snapshot().connected);
  assert(peer.arm_commands() == 0U);
  second.stop();
  first.stop();
}
}  // [AI] 匿名命名空间

int main()
{
  test_handshake_feedback_and_safe_output();
  test_missing_required_feature_never_starts_drive();
  test_malformed_feedback_and_timeout_revoke_validity();
  test_second_session_cannot_own_the_same_device();
  return 0;
}
