// [AI] CAN v4 冻结版生产路径端到端测试——直接驱动 rak_chassis::CanSession
//（与 ros2_control/SystemInterface 同一条代码路径）。零物理动作：
// 轮驱仅发零目标（闭环零速），ARM 仅重申 Home 位（舵机已在位，≤1 LSB 微动）。
// 退出码 0=全部通过。
#include "rak_chassis/can_session.hpp"

#include <cmath>
#include <cstdio>
#include <thread>

using namespace std::chrono_literals;

namespace
{
int g_fail = 0;

void check(const char *name, bool ok, const char *detail = "")
{
  std::printf("[%s] %s%s%s\n", ok ? "PASS" : "FAIL", name,
              detail[0] ? "  " : "", detail);
  if (!ok) { ++g_fail; }
}

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }
}  // namespace

int main()
{
  rak_chassis::CanSession can("can0", 0x52414B31U);  // "RAK1"
  can.start();

  // 1. 协议握手：NODE_INFO PROTO=4 -> handshaken
  bool handshaken = false;
  for (int i = 0; i < 30 && !handshaken; ++i) {
    handshaken = can.snapshot().handshaken;
    std::this_thread::sleep_for(100ms);
  }
  check("生产路径握手（NODE_INFO PROTO=4）", handshaken);

  auto s = can.snapshot();
  check("链路已连接且无故障", s.connected && !s.faulted);

  // 2. 轮反馈：WHEEL_POS_A/B -> feedback_valid + 4 轮位置
  bool fb = false;
  for (int i = 0; i < 20 && !fb; ++i) {
    fb = can.snapshot().feedback_valid;
    std::this_thread::sleep_for(50ms);
  }
  s = can.snapshot();
  check("轮位置反馈 100Hz 落地", fb && (s.feedback_valid_mask & 0x0FU) == 0x0FU);
  if (fb) {
    char d[128];
    std::snprintf(d, sizeof(d), "pos[rad]: %.3f %.3f %.3f %.3f",
                  s.wheel_position[0], s.wheel_position[1],
                  s.wheel_position[2], s.wheel_position[3]);
    check("四轮位置快照可读", true, d);
  }

  // 3. 远程门 + 零目标轮驱 1s（闭合生产写路径，轮子不转）
  const std::array<double, 4U> zero{0.0, 0.0, 0.0, 0.0};
  can.set_wheel_command(zero);
  can.set_motion_enabled(true);
  std::this_thread::sleep_for(1s);
  s = can.snapshot();
  check("START+DRIVE 100Hz 生产路径无故障", s.handshaken && !s.faulted);

  // 4. ARM 命令 Home 位（0/225/270/0/0/0° + 夹爪 0）——舵机已在位，≤1 LSB
  constexpr double kD2R = 3.14159265358979323846 / 180.0;
  const std::array<double, 7U> home{
    0.0, 225.0 * kD2R, 270.0 * kD2R, 0.0, 0.0, 0.0, 0.0};
  can.set_arm_command(home);
  std::this_thread::sleep_for(300ms);   // [AI] 50Hz 节拍发出 A/B 组
  std::this_thread::sleep_for(300ms);   // [AI] 等 SERVO_STATE 遥测回读
  s = can.snapshot();
  check("舵机读回 J1-J6 有效（servo_valid_mask bit0-5）",
        (s.servo_valid_mask & 0x3FU) == 0x3FU);
  char d[160];
  std::snprintf(d, sizeof(d), "J1..J6[deg]: %.1f %.1f %.1f %.1f %.1f %.1f",
    s.joint_position[0] / kD2R, s.joint_position[1] / kD2R,
    s.joint_position[2] / kD2R, s.joint_position[3] / kD2R,
    s.joint_position[4] / kD2R, s.joint_position[5] / kD2R);
  std::printf("      %s\n", d);
  check("ARM_CMD Home 位与 SERVO_STATE 回读一致（±1°）",
    near(s.joint_position[0], 0.0, kD2R) &&
    near(s.joint_position[1], 225.0 * kD2R, 1.0 * kD2R) &&
    near(s.joint_position[2], 270.0 * kD2R, 1.0 * kD2R) &&
    near(s.joint_position[3], 0.0, kD2R) &&
    near(s.joint_position[4], 0.0, kD2R) &&
    near(s.joint_position[5], 0.0, kD2R));
  check("夹爪反馈位按协议置无效（bit6=0，无硬件通道）",
    (s.servo_valid_mask & 0x40U) == 0U);

  // 5. STOP 收尾
  can.set_motion_enabled(false);
  std::this_thread::sleep_for(300ms);
  s = can.snapshot();
  check("STOP 后链路保持无故障", !s.faulted);
  can.stop();

  std::printf("== 生产路径 e2e: %s ==\n", g_fail == 0 ? "PASS" : "FAIL");
  return g_fail == 0 ? 0 : 1;
}
