#define _DEFAULT_SOURCE

#include "rak_chassis/serial_session.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <cstdio>  // [AI] 临时诊断日志，定位联调问题后删除

namespace rak_chassis
{
namespace
{
constexpr auto kIoPollInterval = std::chrono::milliseconds(1);
constexpr auto kReconnectInterval = std::chrono::milliseconds(50);
constexpr auto kCommandPeriod = std::chrono::milliseconds(10);
constexpr auto kHelloPeriod = std::chrono::seconds(1);
constexpr auto kWriteTimeout = std::chrono::milliseconds(20);
/* [AI] ARMED 命令的轮速上限（urad/s），防止单帧失控；实际限幅仍以 MCU 等比饱和为准 */
constexpr int64_t kMaxWheelUradS = 3000000;  // [AI] R37 联测放宽：0.6 rad/s 钳位把
// 0.05 m/s 点动(1.538 rad/s)截成 0.6，四轮"同步变慢"假象。3 rad/s 仍远低于
// 固件 V_CMD_LIMIT(≈35 rad/s) 与 URDF joint limit(10 rad/s)，等比饱和兜底在 MCU。
constexpr uint16_t kLeaseMs = 100U;

double command_from_bits(const std::atomic<uint64_t> &bits_cell)
{
  const uint64_t bits = bits_cell.load(std::memory_order_relaxed);
  double value = 0.0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

bool write_all(int fd, const std::vector<uint8_t> &bytes, const std::atomic<bool> &stopping)
{
  size_t offset = 0U;
  const auto deadline = std::chrono::steady_clock::now() + kWriteTimeout;
  while (offset < bytes.size() && !stopping.load()) {
    const ssize_t count = write(fd, bytes.data() + offset, bytes.size() - offset);
    if (count > 0) {
      offset += static_cast<size_t>(count);
      continue;
    }
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      return false;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    pollfd descriptor{fd, POLLOUT, 0};
    poll(&descriptor, 1, 2);
  }
  return offset == bytes.size();
}

bool valid_arm_state(const Frame &frame)
{
  if (frame.payload.size() < 6U) {
    return false;
  }
  const uint8_t mode = frame.payload[2];
  const uint8_t axis_count = frame.payload[3];
  const size_t expected_size = axis_count == 6U ? 66U : axis_count == 7U ? 74U : 0U;
  if (expected_size == 0U || frame.payload.size() != expected_size || mode != 0U) {
    return false;
  }
  return read_u32(&frame.payload[6]) == 0U;
}
}  // [AI] 私有实现命名空间结束

SerialSession::SerialSession(
  std::string device_path,
  uint32_t host_build_id,
  std::chrono::milliseconds feedback_timeout)
: device_path_(std::move(device_path)),
  host_build_id_(host_build_id),
  feedback_timeout_(feedback_timeout)
{
}

SerialSession::~SerialSession()
{
  stop();
}

void SerialSession::start()
{
  bool expected = false;
  if (started_.compare_exchange_strong(expected, true)) {
    stopping_.store(false);
    fault_latched_.store(false);
    io_thread_ = std::thread(&SerialSession::run, this);
  }
}

void SerialSession::stop()
{
  if (!started_.exchange(false)) {
    return;
  }
  stopping_.store(true);
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
  publish_link_state(false, false, true);
  revoke_feedback(true);
}

void SerialSession::set_wheel_command(const std::array<double, 4> &command)
{
  for (size_t i = 0U; i < command.size(); ++i) {
    double finite_command = std::isfinite(command[i]) ? command[i] : 0.0;
    uint64_t bits;
    std::memcpy(&bits, &finite_command, sizeof(bits));
    latest_command_bits_[i].store(bits, std::memory_order_relaxed);
  }
}

void SerialSession::set_motion_enabled(bool enabled)
{
  motion_enabled_.store(enabled, std::memory_order_relaxed);
}

void SerialSession::set_arm_command(const std::array<double, 7> &command)
{
  for (size_t i = 0U; i < 7U; ++i) {
    uint64_t bits = 0U;
    std::memcpy(&bits, &command[i], sizeof(bits));
    latest_arm_bits_[i].store(bits, std::memory_order_relaxed);
  }
  arm_dirty_.store(true, std::memory_order_relaxed);
}

SessionSnapshot SerialSession::snapshot() const
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  SessionSnapshot result = snapshot_;
  if (result.handshaken) {
    result.feedback_age = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_feedback_at_);
    if (result.feedback_age > feedback_timeout_) {
      result.feedback_valid = false;
      result.feedback_valid_mask = 0U;
      result.wheel_position.fill(0.0);
      result.wheel_velocity.fill(0.0);
      result.faulted = true;
    }
  }
  return result;
}

bool SerialSession::open_device(int &fd)
{
  fd = open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    return false;
  }
  if (flock(fd, LOCK_EX | LOCK_NB) != 0 || ioctl(fd, TIOCEXCL) != 0) {
    close(fd);
    fd = -1;
    return false;
  }

  termios settings{};
  if (tcgetattr(fd, &settings) != 0) {
    close_device(fd);
    fd = -1;
    return false;
  }
  cfmakeraw(&settings);
  settings.c_cflag &= static_cast<tcflag_t>(~(PARENB | CSTOPB | CSIZE));
  settings.c_cflag |= static_cast<tcflag_t>(CS8 | CLOCAL | CREAD);
#ifdef CRTSCTS
  settings.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
  settings.c_cc[VMIN] = 0;
  settings.c_cc[VTIME] = 0;
  if (cfsetispeed(&settings, B921600) != 0 ||
    cfsetospeed(&settings, B921600) != 0 ||
    tcsetattr(fd, TCSANOW, &settings) != 0)
  {
    close_device(fd);
    fd = -1;
    return false;
  }
  tcflush(fd, TCIOFLUSH);
  return true;
}

void SerialSession::close_device(int fd)
{
  if (fd >= 0) {
    ioctl(fd, TIOCNXCL);
    close(fd);
  }
}

bool SerialSession::receive_packet(
  int fd, std::vector<uint8_t> &packet, std::vector<uint8_t> &stream_buffer)
{
  const auto delimiter = std::find(stream_buffer.begin(), stream_buffer.end(), 0U);
  if (delimiter != stream_buffer.end()) {
    const size_t packet_size = static_cast<size_t>(delimiter - stream_buffer.begin()) + 1U;
    packet.assign(stream_buffer.begin(), stream_buffer.begin() + packet_size);
    stream_buffer.erase(stream_buffer.begin(), stream_buffer.begin() + packet_size);
    return true;
  }

  pollfd descriptor{fd, POLLIN, 0};
  const int ready = poll(&descriptor, 1, static_cast<int>(kIoPollInterval.count()));
  if (ready <= 0 || (descriptor.revents & POLLIN) == 0) {
    return false;
  }
  uint8_t incoming[256];
  const ssize_t count = read(fd, incoming, sizeof(incoming));
  if (count <= 0) {
    return false;
  }
  if (stream_buffer.size() + static_cast<size_t>(count) > 512U) {
    stream_buffer.clear();
    packet.clear();
    return true;
  }
  stream_buffer.insert(stream_buffer.end(), incoming, incoming + count);
  return receive_packet(fd, packet, stream_buffer);
}

bool SerialSession::process_packet(const std::vector<uint8_t> &packet)
{
  Frame frame;
  if (!decode_frame(packet.data(), packet.size(), frame)) {
    return false;
  }
  return process_frame(frame);
}

bool SerialSession::process_frame(const Frame &frame)
{
  uint32_t packet_boot_id = current_boot_id_;
  /* [AI] 临时诊断（限幅：只打前 400 条） */
  {
    static int dbg_lines = 0;
    if (dbg_lines < 400) {
      ++dbg_lines;
      fprintf(stderr, "[rak_dbg] frame type=0x%02x seq=%u len=%zu\n",
        frame.type, frame.sequence, frame.payload.size());
    }
  }
  if (frame.type == kTypeHelloAck) {
    /* [AI] 按 usb_serial_v2.md：firmware_version=(major<<16)|(minor<<8)|patch，
     * 是固件自身版本号，与协议版本无关（勿再校验高 16 位==2，R30 联测修正） */
    if (frame.payload.size() != 14U || read_u16(frame.payload.data()) != 0x0407U ||
      read_u32(&frame.payload[2]) == 0U)
    {
      fprintf(stderr, "[rak_dbg] hello_ack header reject\n");
      return false;
    }
    packet_boot_id = read_u32(&frame.payload[6]);
    if (packet_boot_id == 0U ||
      (read_u32(&frame.payload[10]) & kRequiredFeatures) != kRequiredFeatures)
    {
      fprintf(stderr, "[rak_dbg] hello_ack features reject\n");
      revoke_feedback(true);
      return false;
    }
    if (packet_boot_id != current_boot_id_) {
      current_boot_id_ = packet_boot_id;
      have_rx_sequence_ = false;
      sent_drive_sequence_count_ = 0U;
      sent_drive_sequence_index_ = 0U;
      have_sample_time_ = false;
      /* [AI] R30：固件新 boot 会话即全新链路，清除历史故障锁 */
      fault_latched_.store(false);
      revoke_feedback(false);
    }
  }

  if (have_rx_sequence_ && !sequence_is_newer(frame.sequence, last_rx_sequence_)) {
    fprintf(stderr, "[rak_dbg] stale seq type=0x%02x seq=%u last=%u\n",
      frame.type, frame.sequence, last_rx_sequence_);
    return false;
  }

  if (frame.type == kTypeHelloAck) {
    const uint32_t accepted = read_u32(&frame.payload[10]);
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot_.connected = true;
    snapshot_.handshaken = true;
    snapshot_.faulted = fault_latched_.load();
    snapshot_.accepted_features = accepted;
    snapshot_.boot_id = packet_boot_id;
    last_feedback_at_ = std::chrono::steady_clock::now();
  } else if (frame.type == kTypeDriveState) {
    /* [AI] R37 联测对齐：固件侧反馈契约=有 sample_time 即可信，fb_mask 反映
     * 编码器学习进度（上电后 kick 学习前可为 0），ARMED 时 mode=1。
     * 旧条件（mode==0 且 mask==0x0F）会与 kick 学习形成死锁，已放宽。 */
    if (frame.payload.size() != 68U || !snapshot().handshaken ||
      frame.payload[2] > 1U ||
      read_u64(&frame.payload[12]) == 0U)
    {
      /* [AI] R30 联测修正：固件上电即流式发送状态帧（先于握手/命令回显），
       * 此类帧属"当前不可用"而非协议错误——丢弃、不刷新 lease，不触发断链 */
      return true;
    }
    const uint16_t echoed_sequence = read_u16(frame.payload.data());
    bool echo_found = false;
    for (size_t i = 0U; i < sent_drive_sequence_count_; ++i) {
      if (sent_drive_sequences_[i] == echoed_sequence) {
        echo_found = true;
        break;
      }
    }
    if (!echo_found) {
      return true;  // [AI] 回显不匹配：丢弃而非断链
    }
    const uint32_t epoch = read_u32(&frame.payload[8]);
    const uint64_t sample_time = read_u64(&frame.payload[12]);
    {
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      if (have_sample_time_ && sample_time <= snapshot_.sample_time_us) {
        return false;
      }
      snapshot_.encoder_epoch = epoch;
      snapshot_.sample_time_us = sample_time;
      snapshot_.fault_bitmap = read_u32(&frame.payload[4]);
      snapshot_.feedback_valid_mask = frame.payload[3];
      for (size_t i = 0U; i < 4U; ++i) {
        snapshot_.wheel_position[i] =
          static_cast<double>(read_i64(&frame.payload[20U + i * 8U])) / 1000000.0;
        snapshot_.wheel_velocity[i] =
          static_cast<double>(read_i32(&frame.payload[52U + i * 4U])) / 1000000.0;
      }
      snapshot_.feedback_valid = true;
      snapshot_.faulted = fault_latched_.load();
      last_feedback_at_ = std::chrono::steady_clock::now();
      have_sample_time_ = true;
    }
  } else if (frame.type == kTypeArmState) {
    if (!valid_arm_state(frame)) {
      /* [AI] R30 联测修正：状态帧异常丢弃（含 DISARMED 校验不过），不断链 */
      return true;
    }
    /* [AI] 冻结版补齐：解析 measured_position 进 SessionSnapshot.joint_position。
       v2 ARM_STATE 布局（app_protocol_v2.h）：echoed_seq u16 + mode u8 +
       axis_count u8 + measured_valid_mask u8 + measured_source u8 + fault u32 +
       sample_time_us u64（=18B 头）+ axis_count×command i32 + axis_count×measured i32。
       measured 为固件回读（舵机无编码器，来源=指令回显），mask 直接透传给消费端。 */
    {
      const uint8_t axis_count = frame.payload[3];
      const uint8_t measured_mask = frame.payload[4];
      const size_t meas_base = 18U + static_cast<size_t>(axis_count) * 4U;
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      for (size_t i = 0U; i < axis_count && i < 7U; ++i) {
        snapshot_.joint_position[i] =
          static_cast<double>(read_i32(&frame.payload[meas_base + i * 4U])) /
          1000000.0;
      }
      snapshot_.servo_valid_mask = static_cast<uint8_t>(measured_mask & 0x7FU);
    }
  } else if (frame.type == kTypeDiagnostics) {
    if (frame.payload.size() != 36U) {
      return false;
    }
  } else if (frame.type == kTypeFaultEvent) {
    if (frame.payload.size() != 7U || frame.payload[6] < 1U || frame.payload[6] > 3U) {
      return false;
    }
    const uint32_t faults = read_u32(frame.payload.data());
    if (faults != 0U) {
      {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        snapshot_.fault_bitmap = faults;
      }
      revoke_feedback(true);
      return false;
    }
  } else {
    return false;
  }

  last_rx_sequence_ = frame.sequence;
  have_rx_sequence_ = true;
  return true;
}

void SerialSession::publish_link_state(bool connected, bool handshaken, bool faulted)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  snapshot_.connected = connected;
  snapshot_.handshaken = handshaken;
  snapshot_.faulted = faulted || fault_latched_.load();
  if (!connected) {
    snapshot_.accepted_features = 0U;
    snapshot_.boot_id = 0U;
  }
}

void SerialSession::revoke_feedback(bool faulted)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  snapshot_.feedback_valid = false;
  snapshot_.feedback_valid_mask = 0U;
  snapshot_.wheel_position.fill(0.0);
  snapshot_.wheel_velocity.fill(0.0);
  /* [AI] 冻结版补齐：舵机读回随反馈一并撤销，防陈旧关节角流入上层 */
  snapshot_.joint_position.fill(0.0);
  snapshot_.servo_valid_mask = 0U;
  snapshot_.faulted = faulted || fault_latched_.load();
  snapshot_.feedback_age = std::chrono::milliseconds::max();
}

void SerialSession::run()
{
  while (!stopping_.load()) {
    int fd = -1;
    if (!open_device(fd)) {
      publish_link_state(false, false, true);
      revoke_feedback(true);
      std::this_thread::sleep_for(kReconnectInterval);
      continue;
    }

    have_rx_sequence_ = false;
    have_sample_time_ = false;
    last_rx_sequence_ = 0U;
    tx_sequence_ = 0U;
    /* [AI] R37 联测修正：current_boot_id_ 跨重连保留——重连≠固件重启，
     * 只有 HELLO_ACK 报告的 boot_id 真变化才清故障锁（pty 测试暴露） */
    sent_drive_sequence_count_ = 0U;
    sent_drive_sequence_index_ = 0U;
    publish_link_state(true, false, true);
    revoke_feedback(true);

    std::vector<uint8_t> stream_buffer;
    std::vector<uint8_t> packet;
    const auto connected_at = std::chrono::steady_clock::now();
    auto last_hello = connected_at - kHelloPeriod;
    auto last_drive = connected_at;
    bool failed = false;

    while (!stopping_.load() && !failed) {
      if (receive_packet(fd, packet, stream_buffer)) {
        if (!process_packet(packet)) {
          /* [AI] R30 联测修正：串口流中偶发非协议字节（如孤立 0x00）曾导致整链
           * 反复断开。改为丢弃坏包并清缓冲：不刷新 lease（固件 lease 看门狗仍
           * 兜底），但不再因单个坏帧断链。 */
          static int fail_lines = 0;
          if (fail_lines < 50) {
            ++fail_lines;
            fprintf(stderr, "[rak_dbg] bad packet dropped size=%zu\n", packet.size());
          }
          stream_buffer.clear();
          packet.clear();
          continue;
        }
        packet.clear();
      }

      const auto now = std::chrono::steady_clock::now();
      const SessionSnapshot state = snapshot();
      if (state.handshaken && state.feedback_age > feedback_timeout_) {
        fault_latched_.store(true);
        revoke_feedback(true);
        failed = true;
        continue;
      }

      if (now - last_hello >= kHelloPeriod) {
        Frame hello;
        hello.type = kTypeHello;
        hello.sequence = tx_sequence_++;
        append_u32(hello.payload, host_build_id_);
        append_u32(hello.payload, kRequiredFeatures);
        std::vector<uint8_t> wire;
        if (!encode_frame(hello, wire) || !write_all(fd, wire, stopping_)) {
          failed = true;
          continue;
        }
        last_hello = now;
      }

      if (state.handshaken && now - last_drive >= kCommandPeriod) {
        Frame command;
        command.type = kTypeDriveCommand;
        command.sequence = tx_sequence_++;
        command.payload.reserve(20U);
        /* [AI] 仅当 motion_enabled 且四路目标均有限时发 ARMED；其余一律 DISARMED 零目标。
         * 断链/超时/协议错误路径会在循环外撤销 motion_enabled_，恢复通信不自动重新使能。 */
        bool armed = motion_enabled_.load(std::memory_order_relaxed);
        double wheel_speeds[4U] = {0.0, 0.0, 0.0, 0.0};
        if (armed) {
          for (size_t wheel = 0U; wheel < 4U; ++wheel) {
            wheel_speeds[wheel] = command_from_bits(latest_command_bits_[wheel]);
            armed = armed && std::isfinite(wheel_speeds[wheel]);
          }
        }
        command.payload.push_back(armed ? 1U : 0U);
        append_u16(command.payload, kLeaseMs);
        command.payload.push_back(armed ? 0x0FU : 0U);
        for (size_t wheel = 0U; wheel < 4U; ++wheel) {
          int64_t urad_s = 0;
          if (armed) {
            urad_s = static_cast<int64_t>(std::lround(wheel_speeds[wheel] * 1e6));
            urad_s = std::clamp<int64_t>(urad_s, -kMaxWheelUradS, kMaxWheelUradS);
          }
          append_i32(command.payload, static_cast<int32_t>(urad_s));
        }
        std::vector<uint8_t> wire;
        if (!encode_frame(command, wire) || !write_all(fd, wire, stopping_)) {
          failed = true;
          continue;
        }
        sent_drive_sequences_[sent_drive_sequence_index_] = command.sequence;
        sent_drive_sequence_index_ = (sent_drive_sequence_index_ + 1U) %
          sent_drive_sequences_.size();
        sent_drive_sequence_count_ = std::min(
          sent_drive_sequence_count_ + 1U, sent_drive_sequences_.size());
        last_drive = now;
      }

      /* [AI] 冻结版补齐：臂命令 0x11，事件驱动——仅 handshaken + motion_enabled +
         有新目标时发送（与 CanSession 同一安全契约：舵机许可在本层）。v2 线格式
         自 R42 冻结未变：mode u8 + lease u16 + axis_count u8 + valid_mask u8 +
         N×i32 µrad；axis_count 固定 6（J1..J6），夹爪不协商不发。固件断流语义 =
         舵机保持当前位置，禁止周期重放旧目标。 */
      if (state.handshaken && motion_enabled_.load(std::memory_order_relaxed) &&
        arm_dirty_.exchange(false))
      {
        Frame arm;
        arm.type = kTypeArmCommand;
        arm.sequence = tx_sequence_++;
        arm.payload.reserve(5U + 6U * 4U);
        arm.payload.push_back(1U);                      // [AI] mode=ARMED
        append_u16(arm.payload, kLeaseMs);
        arm.payload.push_back(6U);                      // [AI] axis_count=6（J1..J6）
        arm.payload.push_back(0x3FU);                   // [AI] valid_mask
        for (size_t axis = 0U; axis < 6U; ++axis) {
          const double rad = command_from_bits(latest_arm_bits_[axis]);
          int64_t urad = 0;
          if (std::isfinite(rad) && rad > 0.0) {
            constexpr double kMaxArmRad = 4.71238898038469;  // 270°
            const double clamped = std::min(rad, kMaxArmRad);
            urad = static_cast<int64_t>(std::lround(clamped * 1e6));
          }
          append_i32(arm.payload, static_cast<int32_t>(urad));
        }
        std::vector<uint8_t> wire;
        if (!encode_frame(arm, wire) || !write_all(fd, wire, stopping_)) {
          failed = true;
          continue;
        }
      }
    }

    if (failed && snapshot().handshaken) {
      fault_latched_.store(true);
    }
    fprintf(stderr, "[rak_dbg] link exit failed=%d\n", failed ? 1 : 0);
    /* [AI] 任何链路退出（断链/超时/协议错误/停止）都撤销运动使能，恢复后不自动重新使能 */
    motion_enabled_.store(false, std::memory_order_relaxed);
    close_device(fd);
    publish_link_state(false, false, true);
    revoke_feedback(true);
  }
}
}  // [AI] 命名空间结束
