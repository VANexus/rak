"""底盘速度上限的单一数值来源（stdlib-only，任何人不得再抄一份常数）。

为什么放在 `rak_tool_executor`：它是运动命令的唯一写入者，工具表的 schema 上限、
执行器侧的 clamp、台架 TUI 的手动限幅都从这里取，三份常数才不会漂移。
MCP 层不持有任何速度数字，只读工具表。

单位与坐标系（既有事实，本模块只给数值，不重新定义语义）：
- vx 前进为正、vy 左为正，单位 m/s；wz 逆时针为正，单位 rad/s；
- 坐标系 = base_link；
- 命令入口 = `/mecanum_drive_controller/reference`（TwistStamped），
  见 `src/rak_bringup/config/chassis_controllers.yaml`（reference_timeout 0.5 s
  无新鲜输入自动置零 → 这是第二道「不会一直跑」的闸，但本层不依赖它）；
- 轮序 FL/FR/RL/RR 的事实源同为该 yaml，本模块不重复。

上位机限速只是第一道闸：MCU 侧对轮速指令还有硬饱和
（`docs/reference/firmware_capability_spec.md` N4）。轮几何与限速在实测替换前
不得提速（`docs/tutorials/construction.md` / `docs/status/project_status.md`）。
"""

# 未标定前的保守上限（与台架键盘 TUI 同一套值，2026-10-14 起未变）。
MAX_LINEAR_MPS = 0.30
MAX_ANGULAR_RPS = 1.20

# 命令流刷新率：低于控制器 reference_timeout 的一半，保证不会误触发置零。
CMD_RATE_HZ = 20.0

# 单次运动类工具的最长时长 —— 「调用后一直跑」在参数层面就不成立。
MIN_DURATION_S = 0.05
MAX_DURATION_S = 3.0
