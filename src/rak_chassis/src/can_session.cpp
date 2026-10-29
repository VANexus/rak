// Copyright 2026 RAK project
// [AI] R49: CAN v4 传输后端实现（SocketCAN）。协议真源 docs/protocol-signals.yaml v4：
//   0x010 SAFETY_CMD(e2e) / 0x030 NODE_INFO / 0x100 DRIVE_CMD / 0x200+201 WHEEL_POS
//   / 0x202 WHEEL_SPD / 0x220 DIAG。内核负责 CAN CRC/ACK/重传与时间戳，
//   本类只做编解码与节拍；SocketCAN 收发不占用户态 CPU（用户要求的最丝滑路径）。
#include "rak_chassis/can_session.hpp"

#include "rak_chassis/can_decode.hpp"

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

namespace rak_chassis
{
namespace
{
constexpr uint32_t kIdSafety = 0x010U;
constexpr uint32_t kIdNodeInfo = 0x030U;
constexpr uint32_t kIdDriveCmd = 0x100U;
constexpr uint32_t kIdArmCmdA = 0x110U;      // [AI] 冻结版补齐：J1-J4 目标（0.1° u16）
constexpr uint32_t kIdArmCmdB = 0x111U;      // [AI] J5,J6,夹爪 目标
constexpr uint32_t kIdWheelPosA = 0x200U;
constexpr uint32_t kIdWheelPosB = 0x201U;
constexpr uint32_t kIdWheelSpd = 0x202U;
constexpr uint32_t kIdServoA = 0x210U;       // [AI] 冻结版补齐：J1-J4 实际角遥测
constexpr uint32_t kIdServoB = 0x211U;       // [AI] J5,J6,夹爪 实际角遥测
constexpr uint32_t kIdDiag = 0x220U;
/* [AI] 机械臂板上行（双板架构 2026-10-28）。底盘板与臂板各自一套 NODE_INFO/DIAG，
 * 同总线 ID 全局唯一，故臂板另起 0x031/0x221，避免与 0x030/0x220 撞车。 */
constexpr uint32_t kIdArmNodeInfo = 0x031U;  // ARM_NODE_INFO: PROTO/HWREV/BUILD/FAULT/STATE
constexpr uint32_t kIdArmDiag = 0x221U;      // ARM_DIAG: FAULT/STATE/MODE/SERVO_ERRCNT/RSVD
/* [AI] 2026-10-29：IMU 两帧（纯加法）—— 底盘板 0x032、机械臂板 0x222。
 * 两板都装在车体上，陀螺可互校；加速度只当事件强度。 */
constexpr uint32_t kIdChassisImu = 0x032U;   // CHASSIS_IMU: DYAW/PITCH/ROLL/EVT/AGE
constexpr uint32_t kIdArmImu = 0x222U;       // ARM_IMU: GZ/AMAG/EVT/AGE/RSVD
constexpr uint8_t kDidSafety = 0x01U;
/* [AI] kStateFaultMask 与 LE 读取器统一搬到 can_decode.hpp（单点定义 + 可单测）。 */

constexpr int kDriveHz = 100;                     // [AI] DRIVE_CMD 周期（v4 §3）
/* [AI] 臂板上行新鲜度窗口：ARM_NODE_INFO @1 Hz + ARM_DIAG @10 Hz，取 1.5 s
 * （覆盖 1 Hz NODE_INFO 抖动；两帧都断流才判 arm_online=false）。 */
constexpr int kArmInfoTimeoutMs = 1500;
/* [AI] 2026-10-29 IMU 两帧的新鲜度窗：底盘 100 Hz → 100 ms（丢 10 帧即判失效）；
 * 臂板同样 100 Hz，但给 200 ms（与 arm_imu_valid 注释一致，容忍调度抖动）。 */
constexpr int kImuTimeoutMs = 100;
constexpr int kArmImuTimeoutMs = 200;
/* [AI] 底盘板握手新鲜度：NODE_INFO @1 Hz，给 1.5 s（丢两帧才判失效）。
 * handshaken 以前只在 open/close 时复位——socket 还在而板子掉电/断线时
 * 它会一直保持 true，只能靠 feedback_age(120 ms) 兜底。语义上“握手”同样
 * 是一条有寿命的断言，必须带窗。 */
constexpr int kNodeInfoTimeoutMs = 1500;
constexpr int kArmHz = 50;                        // [AI] ARM_CMD 上限节拍（v4 §3，实际事件驱动）
constexpr double kDriveLsbRadS = 0.01;            // [AI] DRIVE_CMD 量化 0.01 rad/s
constexpr double kSpdLsbRadS = 0.01;              // [AI] WHEEL_SPD 量化
constexpr double kDegToRad = 0.017453292519943295;  // [AI] 2026-10-29: IMU 帧（度 → rad）
// [AI] 舵机角量化 0.1°（v4 §5，0..2700 ↔ 0..270°）；遥测读回同系数
constexpr double kServoLsbRad = 0.0017453292519943295;  // 0.1° in rad
constexpr double kServoMaxRad = 4.71238898038469;       // 270° in rad
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

// [AI] rad → 0.1° u16（饱和截断到 [0,2700]，负值截到 0——协议 §2：不取模不回绕）
uint16_t to_servo_lsb(double rad)
{
  if (!(rad > 0.0)) { return 0U; }                 // [AI] 同时处理 NaN
  if (rad > kServoMaxRad) { return 2700U; }
  const double v = std::lround(rad / kServoLsbRad);
  if (v > 2700.0) { return 2700U; }
  return static_cast<uint16_t>(v);
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
    can_filter filters[11];
    const uint32_t ids[11] = {kIdSafety, kIdNodeInfo, kIdArmNodeInfo, kIdDriveCmd,
      kIdWheelPosA, kIdWheelPosB, kIdWheelSpd, kIdServoA, kIdServoB, kIdDiag, kIdArmDiag};
    for (size_t i = 0U; i < 11U; ++i) {
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

void CanSession::set_arm_command(const std::array<double, 7> &command)
{
  for (size_t i = 0U; i < 7U; ++i) {
    uint64_t bits = 0U;
    std::memcpy(&bits, &command[i], sizeof(bits));
    latest_arm_bits_[i].store(bits, std::memory_order_relaxed);
  }
  arm_dirty_.store(true, std::memory_order_relaxed);
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
      // [AI] 冻结版补齐：轮反馈超时视为链路异常，舵机读回一并撤销防陈旧值
      result.joint_position.fill(0.0);
      result.servo_valid_mask = 0U;
    }
  }
  /* [AI] 臂板新鲜度：只有收到过 0x031/0x221 才计算；断流超窗 -> arm_online=false
   * （arm_state/arm_fault 保留最后一次快照供诊断；消费端以 arm_online 判定在用性）。 */
  if (result.arm_online) {
    result.arm_feedback_age = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_arm_info_at_);
    if (result.arm_feedback_age > std::chrono::milliseconds(kArmInfoTimeoutMs)) {
      result.arm_online = false;
    }
  }
  /* [AI] 底盘板握手新鲜度（同 arm_online 律）：断流超窗 -> handshaken=false。
   * 不比 feedback_age(120 ms) 更早触发，只是让“握手”这个断言不再永久为真。 */
  if (result.handshaken) {
    const auto info_age = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_node_info_at_);
    if (info_age > std::chrono::milliseconds(kNodeInfoTimeoutMs)) {
      result.handshaken = false;
    }
  }
  /* [AI] 2026-10-29：IMU 两帧同律新鲜度。断流后只清 valid（保留最后一次数值给诊断），
   * 否则消费端会把很久以前的偏航增量当成当前增量继续积分。 */
  if (result.imu_valid) {
    const auto imu_age = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_imu_at_);
    if (imu_age > std::chrono::milliseconds(kImuTimeoutMs)) {
      result.imu_valid = false;
      result.imu_dyaw_rad = 0.0;
    }
  }
  if (result.arm_imu_valid) {
    const auto arm_imu_age = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_arm_imu_at_);
    if (arm_imu_age > std::chrono::milliseconds(kArmImuTimeoutMs)) {
      result.arm_imu_valid = false;
      result.arm_gz_rad_s = 0.0;
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
    {
      const NodeInfoFrame info = decode_node_info(d);
      // [AI] PROTO=4 才认（v4 §10.4：旧固件拒绝发指令）
      if (info.proto_ok) {
        snapshot_.handshaken = true;
        last_node_info_at_ = now;
      }
      /* [AI] 2026-10-28：0x030.FAULT_u16 走独立字段，不再覆盖 0x220 的 fault_bitmap
       * （两者语义不同、周期不同，过去互相冲刷造成 1 Hz vs 10 Hz 竞态）。 */
      snapshot_.chassis_fault_code = info.fault;
    }
      break;
  case kIdArmNodeInfo:
    {
      const ArmNodeInfoFrame info = decode_node_info(d);
      // [AI] 臂板握手：PROTO=4 才认作"机械臂板在线"（v4 加法帧，旧底盘固件不发）
      if (info.proto_ok) {
        snapshot_.arm_online = true;
        last_arm_info_at_ = now;
      }
      snapshot_.arm_hw_rev = info.hw_rev;
      snapshot_.arm_build_tag = info.build_tag;   // build_tag[0],[1] LE：'A','2' -> 0x3241
      snapshot_.arm_fault = info.fault;
      snapshot_.arm_state = info.state;
    }
    break;
  case kIdArmDiag:
    /* [AI] 臂板诊断（decode_arm_diag：FAULT/STATE/MODE/SERVO_ERRCNT）。臂板无电池
     * 通道，FAULT_u16 当前恒 0；J1/J2 位置环故障经 STATE.bit2（fault_active）上报。 */
    {
      const ArmDiagFrame diag = decode_arm_diag(d);
      snapshot_.arm_fault = diag.fault;
      snapshot_.arm_state = diag.state;
      snapshot_.arm_mode = diag.mode;
      snapshot_.arm_servo_errcnt = diag.servo_errcnt;
      snapshot_.arm_faulted = diag.faulted;
      last_arm_info_at_ = now;   // 臂板仍在上行（不据此置 arm_online，见 0x031）
    }
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
    case kIdServoA:
    /* [AI] J1-J4 实际角（0.1° u16 -> rad）。A2（2026-10-28）起 J1/J2 是增量编码器
     * 电机：bit0/bit1 为**真测量**（0.0059°/count）；J3/J4 仍是舵机脉宽回声
     * （servo_get_pulse，±1°，非物理测量）。夹爪无反馈通道（见 kIdServoB）。 */
      for (size_t i = 0U; i < 4U; ++i) {
        snapshot_.joint_position[i] =
          static_cast<double>(read_u16(d, i * 2U)) * kServoLsbRad;
      }
      snapshot_.servo_valid_mask |= 0x0FU;
      break;
    case kIdServoB:
      // [AI] J5,J6 实际角；byte4-5 = 夹爪回显恒 0（无硬件通道），bit6 不置有效
      for (size_t i = 0U; i < 2U; ++i) {
        snapshot_.joint_position[4U + i] =
          static_cast<double>(read_u16(d, i * 2U)) * kServoLsbRad;
      }
      snapshot_.servo_valid_mask |= 0x30U;
      break;
    case kIdChassisImu:
      /* [AI] 2026-10-29：增量/事件，不是姿态本体。dyaw 是“上一帧区间”的增量，
       * 消费端必须用自己的 dt 积分（不要拿它与绝对角速度混用）。 */
      {
        const ChassisImuFrame f = decode_chassis_imu(d);
        snapshot_.imu_dyaw_rad = static_cast<double>(f.dyaw_mrad) * 1e-3;
        snapshot_.imu_dyaw_frames++;
        snapshot_.imu_pitch_rad = static_cast<double>(f.pitch_cdeg) * 1e-2 * kDegToRad;
        snapshot_.imu_roll_rad = static_cast<double>(f.roll_cdeg) * 1e-2 * kDegToRad;
        snapshot_.imu_events = f.events;
        snapshot_.imu_age = std::chrono::milliseconds(f.age_ms);
        snapshot_.imu_valid = true;
        last_imu_at_ = now;
      }
      break;
    case kIdArmImu:
      /* [AI] 2026-10-29：GZ 与底盘板同源冗余，可用于“谁坏了”的判别。 */
      {
        const ArmImuFrame f = decode_arm_imu(d);
        snapshot_.arm_gz_rad_s = static_cast<double>(f.gz_cdps) * 1e-2 * kDegToRad;
        snapshot_.arm_amag_mg = f.amag_mg;
        snapshot_.arm_imu_events = f.events;
        snapshot_.arm_imu_age = std::chrono::milliseconds(f.age_ms);
        snapshot_.arm_imu_valid = true;
        last_arm_imu_at_ = now;
      }
      break;
    case kIdDiag:
      // [AI] STATE 位图只把坏消息位写进 fault_bitmap（见 kStateFaultMask 注释）；
      // MODE: 2=ESTOP 3=FAULT
      {
      const DiagFrame diag = decode_diag(d);
        const bool fault =
        (diag.state & 0x0005U) != 0U || diag.mode == 2U || diag.mode == 3U;
        fault_latched_.store(fault);
        snapshot_.faulted = fault;
      snapshot_.fault_bitmap = static_cast<uint16_t>(diag.state & kStateFaultMask);
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
  auto last_arm_tx = last_tx;              // [AI] ARM 50Hz 节拍（协议 §3 周期上限）
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

      // [AI] 冻结版补齐：ARM_CMD A/B @≤50Hz，事件驱动——仅 motion_enabled 且有
      // 新目标时发送（协议 §10.1 #7：固件对舵机命令无门控，许可在本层；
      // firmware 断流语义=舵机保持当前位置，禁止周期重放旧目标）。
      // 夹爪（axis 6）无硬件通道：固件只回显不落地，照发保持契约一致。
      if (now - last_arm_tx >= std::chrono::milliseconds(1000 / kArmHz) &&
        motion_enabled_.load() && arm_dirty_.exchange(false))
      {
        last_arm_tx = now;
        std::array<uint8_t, 8> a {};
        std::array<uint8_t, 8> b {};
        for (size_t i = 0U; i < 4U; ++i) {
          uint64_t bits = latest_arm_bits_[i].load(std::memory_order_relaxed);
          double cmd = 0.0;
          std::memcpy(&cmd, &bits, sizeof(cmd));
          const uint16_t raw = to_servo_lsb(cmd);
          a[i * 2U] = static_cast<uint8_t>(raw & 0xFFU);
          a[i * 2U + 1U] = static_cast<uint8_t>((raw >> 8U) & 0xFFU);
        }
        for (size_t i = 0U; i < 3U; ++i) {
          uint64_t bits = latest_arm_bits_[4U + i].load(std::memory_order_relaxed);
          double cmd = 0.0;
          std::memcpy(&cmd, &bits, sizeof(cmd));
          const uint16_t raw = to_servo_lsb(cmd);
          b[i * 2U] = static_cast<uint8_t>(raw & 0xFFU);
          b[i * 2U + 1U] = static_cast<uint8_t>((raw >> 8U) & 0xFFU);
        }
        b[6] = 0U;                       // [AI] RSVD16（协议要求保留位发 0）
        b[7] = 0U;
        (void)send_frame(kIdArmCmdA, a);
        (void)send_frame(kIdArmCmdB, b);
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  // [AI] 退出前显式 STOP，防止下位机远程门悬空
  if (socket_fd_ >= 0) {
    send_safety(2U);
  }
}

}  // namespace rak_chassis
