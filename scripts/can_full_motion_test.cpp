// [AI] CAN v4 冻结版全动作域实测——臂 7 轴全行程扫描 + 麦轮四类轮速组合。
// 生产路径（rak_chassis::CanSession，与 ros2_control 同代码路径）。
// 前提（用户确认）：四轮架空、臂周清空。急停兜底：CanSession 退出自动 STOP。
// 退出码 0=全部通过。
#include "rak_chassis/can_session.hpp"

#include <algorithm>
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

constexpr double kD2R = 3.14159265358979323846 / 180.0;
constexpr double kR2D = 180.0 / 3.14159265358979323846;

std::array<double, 7U> with_axis(const std::array<double, 7U> &base,
  size_t axis, double rad)
{
  auto out = base;
  out[axis] = rad;
  return out;
}
}  // namespace

int main()
{
  rak_chassis::CanSession can("can0", 0x52414B31U);
  can.start();

  // ---- 握手 + 反馈就绪 ----
  bool ready = false;
  for (int i = 0; i < 30 && !ready; ++i) {
    auto s = can.snapshot();
    ready = s.handshaken && s.feedback_valid;
    if (!ready) { std::this_thread::sleep_for(100ms); }
  }
  check("握手 + 轮反馈就绪", ready);
  if (!ready) { can.stop(); return 1; }

  // ---- 回 Home 起点 ----
  const std::array<double, 7U> home{
    0.0, 225.0 * kD2R, 270.0 * kD2R, 0.0, 0.0, 0.0, 0.0};
  can.set_motion_enabled(true);
  can.set_arm_command(home);
  std::this_thread::sleep_for(700ms);
  auto s = can.snapshot();
  std::printf("起点读回: J1..J6[deg] = %.1f %.1f %.1f %.1f %.1f %.1f\n",
    s.joint_position[0] * kR2D, s.joint_position[1] * kR2D,
    s.joint_position[2] * kR2D, s.joint_position[3] * kR2D,
    s.joint_position[4] * kR2D, s.joint_position[5] * kR2D);

  // ---- 臂逐轴全行程扫描：0→90→180→270→90→0，其余轴保持 ----
  const double steps_deg[] = {0.0, 90.0, 180.0, 270.0, 90.0, 0.0};
  for (size_t axis = 0U; axis < 6U; ++axis) {
    char name[64];
    std::snprintf(name, sizeof(name), "J%zu 全行程扫描回读一致(±2°)", axis + 1U);
    bool axis_ok = true;
    char detail[160] = "";
    for (const double t : steps_deg) {
      auto cmd = with_axis(home, axis, t * kD2R);
      can.set_arm_command(cmd);
      std::this_thread::sleep_for(350ms);
      auto cur = can.snapshot();
      const double got_deg = cur.joint_position[axis] * kR2D;
      if (std::fabs(got_deg - t) > 2.0) {
        axis_ok = false;
        std::snprintf(detail, sizeof(detail),
          "axis%zu 目标 %.0f° 读回 %.1f°", axis + 1U, t, got_deg);
        break;
      }
    }
    check(name, axis_ok, detail);
  }

  // ---- 夹爪（第 7 轴）：命令全行程，回读按协议恒 0（无硬件通道）----
  bool grip_ok = true;
  for (const double t : {0.0, 135.0, 270.0, 0.0}) {
    auto cmd = with_axis(home, 6U, t * kD2R);
    can.set_arm_command(cmd);
    std::this_thread::sleep_for(300ms);
    if (can.snapshot().joint_position[6U] != 0.0) { grip_ok = false; }
  }
  check("夹爪命令全行程、回读恒 0（协议契约）", grip_ok);

  // ---- 收臂回 Home ----
  can.set_arm_command(home);
  std::this_thread::sleep_for(600ms);
  s = can.snapshot();
  const bool home_ok =
    std::fabs(s.joint_position[1] - 225.0 * kD2R) < 2.0 * kD2R &&
    std::fabs(s.joint_position[2] - 270.0 * kD2R) < 2.0 * kD2R &&
    std::fabs(s.joint_position[0]) < 2.0 * kD2R;
  check("收臂回 Home（0/225/270/0/0/0）", home_ok);

  // ---- 麦轮四类轮速组合（IK 四轮域），验证 WHEEL_SPD 遥测跟踪 ----
  struct Pat { const char *name; std::array<double, 4U> w; };
  const Pat patterns[] = {
    {"前进 1 rad/s",        {1.0, 1.0, 1.0, 1.0}},
    {"横移 (FL-,FR+,RL+,RR-)", {-1.0, 1.0, 1.0, -1.0}},
    {"原地旋转 (FL+,FR-,RL+,RR-)", {1.0, -1.0, 1.0, -1.0}},
    {"斜航 (FL+,RR+)",      {1.0, 0.0, 0.0, 1.0}},
    {"快速倒退 2 rad/s",    {-2.0, -2.0, -2.0, -2.0}},
  };
  for (const auto &p : patterns) {
    can.set_wheel_command(p.w);
    std::this_thread::sleep_for(1500ms);   // [AI] 闭环稳定窗口
    auto cur = can.snapshot();
    char detail[160];
    bool ok = true;
    for (size_t i = 0U; i < 4U; ++i) {
      const double tol = std::max(0.12, 0.25 * std::fabs(p.w[i]));
      if (std::fabs(cur.wheel_velocity[i] - p.w[i]) > tol) { ok = false; }
    }
    std::snprintf(detail, sizeof(detail),
      "实测[rad/s]: %+.2f %+.2f %+.2f %+.2f", cur.wheel_velocity[0],
      cur.wheel_velocity[1], cur.wheel_velocity[2], cur.wheel_velocity[3]);
    check(p.name, ok, detail);
  }

  // ---- 停车收尾 ----
  const std::array<double, 4U> zero{0.0, 0.0, 0.0, 0.0};
  can.set_wheel_command(zero);
  std::this_thread::sleep_for(800ms);
  can.set_motion_enabled(false);
  std::this_thread::sleep_for(400ms);
  s = can.snapshot();
  bool stopped = true;
  for (const double v : s.wheel_velocity) {
    if (std::fabs(v) > 0.05) { stopped = false; }
  }
  check("零目标受控停车 + STOP 回 IDLE", stopped && !s.faulted);
  can.stop();

  std::printf("== 全动作域实测: %s ==\n", g_fail == 0 ? "PASS" : "FAIL");
  return g_fail == 0 ? 0 : 1;
}
