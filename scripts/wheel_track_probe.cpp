// [AI] 轮速跟踪表征：区分「协议命令是否正确落地」与「闭环调参质量」。
// 1 rad/s 定点采样 6×500ms 看收敛/振荡，再 3 rad/s 高速验证。
#include "rak_chassis/can_session.hpp"

#include <cstdio>
#include <thread>

using namespace std::chrono_literals;

int main()
{
  rak_chassis::CanSession can("can0", 0x52414B31U);
  can.start();
  for (int i = 0; i < 30; ++i) {
    auto s = can.snapshot();
    if (s.handshaken && s.feedback_valid) { break; }
    std::this_thread::sleep_for(100ms);
  }
  const std::array<double, 4U> one{1.0, 1.0, 1.0, 1.0};
  can.set_wheel_command(one);
  can.set_motion_enabled(true);
  std::printf("1 rad/s 定点采样（每 500ms，FL FR RL RR）：\n");
  for (int k = 0; k < 6; ++k) {
    std::this_thread::sleep_for(500ms);
    auto s = can.snapshot();
    std::printf("  t+%d.%ds: %+.2f %+.2f %+.2f %+.2f\n", (k + 1) / 2,
      (k % 2) ? 5 : 0, s.wheel_velocity[0], s.wheel_velocity[1],
      s.wheel_velocity[2], s.wheel_velocity[3]);
  }
  const std::array<double, 4U> three{3.0, 3.0, 3.0, 3.0};
  can.set_wheel_command(three);
  std::this_thread::sleep_for(1500ms);
  auto s = can.snapshot();
  std::printf("3 rad/s: %+.2f %+.2f %+.2f %+.2f\n", s.wheel_velocity[0],
    s.wheel_velocity[1], s.wheel_velocity[2], s.wheel_velocity[3]);
  const std::array<double, 4U> zero{0.0, 0.0, 0.0, 0.0};
  can.set_wheel_command(zero);
  std::this_thread::sleep_for(700ms);
  can.set_motion_enabled(false);
  std::this_thread::sleep_for(300ms);
  can.stop();
  return 0;
}
