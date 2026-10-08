// Copyright 2026 RAK project
// [AI] R49: CAN v4 传输后端实现（SocketCAN）。协议真源 docs/protocol-signals.yaml v4：
//   0x010 SAFETY_CMD(e2e) / 0x030 NODE_INFO / 0x100 DRIVE_CMD / 0x200+201 WHEEL_POS
//   / 0x202 WHEEL_SPD / 0x220 DIAG。内核负责 CAN CRC/ACK/重传与时间戳，
//   本类只做编解码与节拍；SocketCAN 收发不占用户态 CPU（用户要求的最丝滑路径）。
#include "rak_hardware/can_session.hpp"

#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <utility>
#include <vector>

namespace rak_hardware
{
namespace
{
constexpr uint32_t kIdSafety = 0x010U;
constexpr uint32_t kIdNodeInfo = 0x030U;
constexpr uint32_t kIdDriveCmd = 0x100U;
constexpr uint32_t kIdWheelPosA = 0x200U;
constexpr uint32_t kIdWheelPosB = 0x201U;
constexpr uint32_t kIdWheelSpd = 0x202U;
constexpr uint32_t kIdDiag = 0x220U;
constexpr uint8_t kDidSafety = 0x01U;

constexpr int kDriveHz = 100;                     // [AI] DRIVE_CMD 周期（v4 §3）
constexpr double kDriveLsbRadS = 0.01;            // [AI] DRIVE_CMD 量化 0.01 rad/s
constexpr double kSpdLsbRadS = 0.01;              // [AI] WHEEL_SPD 量化
constexpr double kWheelCprNominal = 61600.0;      // [AI] 编码器 counts/转（手测名义值，
// docs/PROTOCOL.md v4 §5；四轮 CPR 未逐一标定，里程计精度与 TUI 航位推算同级）
constexpr double kTwoPi = 6.283185307179586476925286766559;

uint8_t crc8_e2e(const std::array<uint8_t, 8> &d, uint8_t data_id)
{
  // [AI] poly 0x07，无反射，无最终异或，初值 = Data ID，覆盖 byte0..6
  uint8_t crc = data_id;
  for (int i = 0; i < 7; ++i) {
    crc ^= d[static_cast<size_t>(i)];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x80U) ? static_cast<uint8_t>((crc << 1) ^ 0x07U) :
        static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

int16_t to_drive_lsb(double rad_s)
{
  const double v = std::lround(rad_s / kDriveLsbRadS);
  if (v > 32767.0) { return 32767; }
  if (v < -32768.0) { return -32768; }
  return static_cast<int16_t>(v);
}

int32_t read_i32(const std::array<uint8_t, 8> &d, size_t off)
{
  return static_cast<int32_t>(
    static_cast<uint32_t>(d[off]) | (static_cast<uint32_t>(d[off + 1]) << 8U) |
    (static_cast<uint32_t>(d[off + 2]) << 16U) | (static_cast<uint32_t>(d[off + 3]) << 24U));
}

int16_t read_i16(const std::array<uint8_t, 8> &d, size_t off)
{
  return static_cast<int16_t>(
    static_cast<uint16_t>(d[off]) | (static_cast<uint16_t>(d[off + 1]) << 8U));
}

uint16_t read_u16(const std::array<uint8_t, 8> &d, size_t off)
{
  return static_cast<uint16_t>(static_cast<uint16_t>(d[off]) |
    (static_cast<uint16_t>(d[off + 1]) << 8U));
}
}  // namespace

CanSession::CanSession(
  std::string channel,
  uint32_t host_build_id,
  std::chrono::milliseconds feedback_timeout)
: channel_(std::move(channel)),
  host_build_id_(host_build_id),
  feedback_timeout_(feedback_timeout)
{
  for (auto &cmd : latest_command_bits_) {
    cmd.store(0.0, std::memory_order_relaxed);  // [AI] 零目标的 double 位模式
  }
}

CanSession::~CanSession()
{
  stop();
}

void CanSession::start()
{
  if (started_.exchange(true)) {
    return;
  }
  stopping_.store(false);
  try_open();
  io_thread_ = std::thread(&CanSession::run, this);
}

bool CanSession::try_open()
{
  socket_fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
  bool opened = false;
  if (socket_fd_ >= 0) {
    ifreq ifr {};
    std::strncpy(ifr.ifr_name, channel_.c_str(), IFNAMSIZ - 1U);
    if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) == 0) {
      sockaddr_can addr {};
      addr.can_family = AF_CAN;
      addr.can_ifindex = ifr.ifr_ifindex;
      // [AI] 只收这些 ID，减少内核→用户态拷贝（过滤器在内核态生效）
      can_filter filters[7];
      const uint32_t ids[7] = {kIdSafety, kIdNodeInfo, kIdDriveCmd, kIdWheelPosA,
        kIdWheelPosB, kIdWheelSpd, kIdDiag};
      for (size_t i = 0U; i < 7U; ++i) {
        filters[i].can_id = ids[i];
        filters[i].can_mask = CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK;
      }
      ::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters, sizeof(filters));
      opened = ::bind(socket_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0;
    }
  }
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot_.connected = opened;
    snapshot_.handshaken = false;
    snapshot_.faulted = true;
    snapshot_.feedback_valid = false;
    snapshot_.feedback_valid_mask = 0U;
  }
  if (!opened) {
    // [AI] 接口没起（ip link down / 不存在）：保持 connected=false，线程仍轮询
    // 重试 try_open，ip link set up 后自动入网——与 restart-ms 语义对齐。
    if (socket_fd_ >= 0) {
      ::close(socket_fd_);
      socket_fd_ = -1;
    }
  }
  return opened;
}

void CanSession::stop()
{
  if (!started_.exchange(false)) {
    return;
  }
  stopping_.store(true);
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
  close_socket();
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  snapshot_.connected = false;
  snapshot_.handshaken = false;
  snapshot_.feedback_valid = false;
}

void CanSession::close_socket()
{
  const int fd = socket_fd_;
  socket_fd_ = -1;
  if (fd >= 0) {
    ::close(fd);
  }
}

void CanSession::set_wheel_command(const std::array<double, 4> &command)
{
  for (size_t i = 0U; i < 4U; ++i) {
    uint64_t bits = 0U;
    std::memcpy(&bits, &command[i], sizeof(bits));
    latest_command_bits_[i].store(bits, std::memory_order_relaxed);
  }
}

void CanSession::set_motion_enabled(bool enabled)
{
  const bool prev = motion_enabled_.exchange(enabled);
  if (prev != enabled) {
    motion_request_dirty_.store(true);
  }
}

SessionSnapshot CanSession::snapshot() const
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  SessionSnapshot result = snapshot_;
  if (result.feedback_valid) {
    result.feedback_age =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_feedback_at_);
    if (result.feedback_age > feedback_timeout_) {
      result.feedback_valid = false;
      result.wheel_position.fill(0.0);
      result.wheel_velocity.fill(0.0);
    }
  }
  return result;
}

void CanSession::publish_link_state(bool connected, bool handshaken, bool faulted)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  snapshot_.connected = connected;
  snapshot_.handshaken = handshaken;
  snapshot_.faulted = faulted;
}

bool CanSession::send_frame(uint32_t can_id, const std::array<uint8_t, 8> &data)
{
  const int fd = socket_fd_;
  if (fd < 0) {
    return false;
  }
  can_frame frame {};
  frame.can_id = can_id;
  frame.can_dlc = 8U;
  std::memcpy(frame.data, data.data(), 8U);
  return ::write(fd, &frame, sizeof(frame)) == sizeof(frame);
}

void CanSession::send_safety(uint8_t cmd_code)
{
  std::array<uint8_t, 8> d {};
  d[0] = cmd_code;                              // [AI] CMD 低 4 位有效
  d[6] = static_cast<uint8_t>((safety_counter_ & 0x0FU) << 4U);  // counter 高 4 位
  safety_counter_ = static_cast<uint8_t>((safety_counter_ + 1U) & 0x0FU);
  d[7] = crc8_e2e(d, kDidSafety);               // [AI] CRC 种子 = DID 0x01（非 CAN ID！）
  (void)send_frame(kIdSafety, d);
}

void CanSession::process_rx(const std::array<uint8_t, 8> &d, uint32_t can_id)
{
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  switch (can_id) {
    case kIdNodeInfo:
      // [AI] PROTO=4 才认（v4 §10.4：旧固件拒绝发指令）
      if (d[0] == 4U) {
        snapshot_.handshaken = true;
        last_node_info_at_ = now;
      }
      snapshot_.fault_bitmap = read_u16(d, 4U);
      break;
    case kIdWheelPosA:
      // [AI] W1/W2 原始 counts → rad（名义 CPR，见 kWheelCprNominal 注释）
      snapshot_.wheel_position[0] =
        static_cast<double>(read_i32(d, 0U)) / kWheelCprNominal * kTwoPi;
      snapshot_.wheel_position[1] =
        static_cast<double>(read_i32(d, 4U)) / kWheelCprNominal * kTwoPi;
      snapshot_.feedback_valid_mask |= 0x03U;
      snapshot_.feedback_valid = true;
      snapshot_.faulted = fault_latched_.load();
      last_feedback_at_ = now;
      break;
    case kIdWheelPosB:
      snapshot_.wheel_position[2] =
        static_cast<double>(read_i32(d, 0U)) / kWheelCprNominal * kTwoPi;
      snapshot_.wheel_position[3] =
        static_cast<double>(read_i32(d, 4U)) / kWheelCprNominal * kTwoPi;
      snapshot_.feedback_valid_mask |= 0x0CU;
      snapshot_.feedback_valid = true;
      snapshot_.faulted = fault_latched_.load();
      last_feedback_at_ = now;
      break;
    case kIdWheelSpd:
      for (size_t i = 0U; i < 4U; ++i) {
        snapshot_.wheel_velocity[i] =
          static_cast<double>(read_i16(d, i * 2U)) * kSpdLsbRadS;
      }
      break;
    case kIdDiag:
      // [AI] STATE 位图 bit0=estop bit2=fault；MODE: 2=ESTOP 3=FAULT
      {
        const uint16_t state = read_u16(d, 4U);
        const uint8_t mode = d[6];
        const bool fault =
          (state & 0x0005U) != 0U || mode == 2U || mode == 3U;
        fault_latched_.store(fault);
        snapshot_.faulted = fault;
        snapshot_.fault_bitmap = state;
        snapshot_.encoder_epoch = 0U;   // [AI] CAN v4 无 epoch 概念，恒 0
        snapshot_.sample_time_us =
          static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
              now.time_since_epoch()).count());
      }
      break;
    default:
      break;
  }
}

void CanSession::run()
{
  auto last_tx = std::chrono::steady_clock::now() - std::chrono::milliseconds(100);
  bool last_motion = false;
  while (!stopping_.load()) {
    // ---- RX ----
    if (socket_fd_ >= 0) {
      can_frame frame {};
      while (::read(socket_fd_, &frame, sizeof(frame)) == sizeof(frame)) {
        if ((frame.can_id & CAN_EFF_FLAG) != 0U || (frame.can_id & CAN_RTR_FLAG) != 0U) {
          continue;
        }
        std::array<uint8_t, 8> data {};
        std::memcpy(data.data(), frame.data, 8U);
        process_rx(data, frame.can_id);
      }
    } else if (started_.load()) {
      // [AI] 链路未开：周期性重试 try_open（等 ip link set can0 up）
      if (!try_open()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        continue;
      }
    }

    // ---- TX @100Hz ----
    const auto now = std::chrono::steady_clock::now();
    if (now - last_tx >= std::chrono::milliseconds(1000 / kDriveHz)) {
      last_tx = now;
      const bool motion = motion_enabled_.load();
      if (motion_request_dirty_.exchange(false) || motion != last_motion) {
        // [AI] 使能沿：START/STOP 各发一帧（MCU 锁存远程门，无需周期重发）
        send_safety(motion ? 1U : 2U);
        last_motion = motion;
      }
      std::array<uint8_t, 8> d {};
      if (motion) {
        for (size_t i = 0U; i < 4U; ++i) {
          uint64_t bits = latest_command_bits_[i].load(std::memory_order_relaxed);
          double cmd = 0.0;
          std::memcpy(&cmd, &bits, sizeof(cmd));
          const int16_t raw = to_drive_lsb(cmd);
          d[i * 2U] = static_cast<uint8_t>(static_cast<uint16_t>(raw) & 0xFFU);
          d[i * 2U + 1U] = static_cast<uint8_t>((static_cast<uint16_t>(raw) >> 8U) & 0xFFU);
        }
      }
      // [AI] DISARMED 也周期发零目标：MCU 端远程门关着本就拒绝，双保险
      (void)send_frame(kIdDriveCmd, d);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  // [AI] 退出前显式 STOP，防止下位机远程门悬空
  if (socket_fd_ >= 0) {
    send_safety(2U);
  }
}

}  // namespace rak_hardware
