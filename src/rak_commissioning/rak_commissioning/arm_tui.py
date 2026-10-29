#!/usr/bin/env python3
"""[AI] 机械臂示教 TUI —— 只经 ROS 接口对话，绝不直连 CAN/串口（ADR-0016 决策 2）。

命令链（ADR-0007 统一 ros2_control）：
    arm_tui ─FollowJointTrajectory(action)─► arm_controller(JTC)
                └─position 接口─► rak_chassis(RakArmHardware) ─Session─► can0 ─► 机械臂板
    arm_tui ─Float64MultiArray─► gripper_controller ─► 同上第 7 轴（夹爪）

反馈 /joint_states（joint_state_broadcaster）。⚠️ 回读 = 固件 servo_get_pulse，
舵机无编码器（契约 §2.2）：只能看"指令到没到"，不能做碰撞/卡死检测。
夹爪无物理反馈通道（回读恒 0，契约 §2.3），屏显只给命令值。

单位：舵机指令角 degree，0..270°（docs/arm_moveit_contract.md §1，FROZEN）。
Home = J1..J6 0/225/270/0/0/0。注意这不是 rak_arm.urdf.xacro 的 q_urdf。

键位：
    1..6      选择 J1..J6        G         选择夹爪
    + / -     目标加/减一个步长   [ / ]     步长 1° / 5° / 15° 循环
    H         发 Home 位（只动 J1..J6，不动夹爪）
    空格       目标同步到当前回读（= 让臂停在原地）
    X         退出（不发任何收尾命令，控制器保持最后目标 = 原地保持）

安全：
  - 启动不发任何目标，收到 /joint_states 反馈后才接收按键指令；
  - 目标变化按 150ms 合并后才发一个 goal（连按不会 goal 洪泛）；
  - 实驱必须 chassis_bringup allow_motion:=true —— 唯一安全门在 Session 层，
    关着时命令会被静默抑制，goal 因回读不动而 abort（屏上给出提示）；
  - 每个 goal 都是短时单点轨迹，JTC 无新目标时保持最后指令位。

必须用 `ros2 run rak_commissioning arm_tui` 启动——需要独占终端读键盘，
经 launch 后台启动会拿不到 TTY 而退化。
"""
import curses
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# [AI] 契约 §1：名称/顺序/行程与 URDF <ros2_control> 一一对应。
ARM_JOINTS = [
    "j1_base", "j2_shoulder", "j3_elbow",
    "j4_forearm", "j5_wrist_pitch", "j6_wrist_roll",
]
ARM_LABELS = ["J1 腰", "J2 大臂", "J3 肘", "J4 小臂", "J5 腕俯", "J6 腕旋"]
GRIPPER_JOINT = "gripper_joint"
ALL_JOINTS = ARM_JOINTS + [GRIPPER_JOINT]
HOME_DEG = [0.0, 225.0, 270.0, 0.0, 0.0, 0.0]   # 固件上电默认位（R44）
LIMIT_DEG = [270.0, 255.0, 270.0, 270.0, 270.0, 270.0]  # J2 实测硬件限（properties_arm）
GRIP_LIMIT_DEG = 270.0
STEP_CHOICES = (1.0, 5.0, 15.0)
GOAL_DURATION = 0.35          # 单点 goal 时长（舵机 PWM 50Hz，物理响应更慢）
SEND_DEBOUNCE = 0.15          # 目标变化合并窗口，防连按 goal 洪泛

DEG = 180.0 / 3.14159265358979323846
RAD = 1.0 / DEG


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def bar(value_deg, limit_deg, width=24):
    """0..limit 的条形图（指令角域恒非负）。"""
    if limit_deg <= 0:
        return " " * width
    n = int(round(clamp(value_deg, 0.0, limit_deg) / limit_deg * width))
    return "#" * n + "-" * (width - n)


class ArmTui(Node):

    def __init__(self):
        super().__init__("arm_tui")
        self.declare_parameter("action", "/arm_controller/follow_joint_trajectory")
        self.declare_parameter("gripper_topic", "/gripper_controller/commands")
        self.action_name = str(self.get_parameter("action").value)
        self.gripper_topic = str(self.get_parameter("gripper_topic").value)

        self.action = ActionClient(self, FollowJointTrajectory, self.action_name)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, self.gripper_topic, QoSProfile(depth=10))
        self.create_subscription(JointState, "/joint_states", self._on_joints,
                                 QoSProfile(depth=10))

        # 目标（指令角域 rad）；收到首个反馈才初始化，之前拒绝任何指令
        self.targets = [0.0] * len(ALL_JOINTS)
        self.fb = [None] * len(ALL_JOINTS)
        self.fb_time = 0.0
        self.have_feedback = False
        self.sel = 0                     # 0..5 臂轴，6 = 夹爪
        self.step_deg = STEP_CHOICES[1]
        self.server_ok = False
        self.last_status = "尚未发送"
        self.last_code = 0
        self.dirty = False               # [AI] 目标已变、待防抖发送
        self.gripper_dirty = False
        self.sending = False             # [AI] goal 在途（屏显用）

        self._debounce = self.create_timer(SEND_DEBOUNCE, self._flush)
        self._probe = self.create_timer(1.0, self._probe_server)
        self._probe_server()

        self.get_logger().info(
            f"臂示教 TUI: action={self.action_name} 夹爪={self.gripper_topic}。"
            "键位 1-6选轴 G夹爪 +/-步长 []步长档 H回Home 空格原地保持 X退出")

    # ---- 反馈 ----
    def _on_joints(self, msg):
        got = False
        for i, name in enumerate(ALL_JOINTS):
            if name not in msg.name:
                continue
            j = msg.name.index(name)
            if j < len(msg.position):
                self.fb[i] = msg.position[j]
                got = True
        if not got:
            return
        self.fb_time = time.monotonic()
        if not self.have_feedback:
            # [AI] 首个反馈即当前姿态：目标从这里起步，启动零位移
            for i in range(len(ARM_JOINTS)):
                if self.fb[i] is not None:
                    self.targets[i] = self.fb[i]
            self.have_feedback = True
            self.get_logger().info("收到 /joint_states 反馈，目标已同步到当前姿态")

    def _probe_server(self):
        self.server_ok = self.action.wait_for_server(timeout_sec=0.05)

    # ---- 指令 ----
    def _flush(self):
        if not self.have_feedback:
            return
        if self.gripper_dirty:
            self.gripper_dirty = False
            msg = Float64MultiArray()
            msg.data = [self.targets[6]]
            self.gripper_pub.publish(msg)
        if self.dirty:
            self.dirty = False
            self._send_arm_goal()

    def _send_arm_goal(self):
        if not self.server_ok:
            self.last_status = "action server 不可用"
            return
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [self.targets[i] for i in range(len(ARM_JOINTS))]
        # [AI] 单点 goal 时长；Jazzy JTC 不做起点校验，轨迹从当前回读插值过来
        # （见 chassis_controllers.yaml 注释）。字段直写，免依赖 rclpy.duration。
        point.time_from_start.sec = int(GOAL_DURATION)
        point.time_from_start.nanosec = int(
            (GOAL_DURATION - int(GOAL_DURATION)) * 1e9)
        traj.points.append(point)
        goal.trajectory = traj
        self.sending = True
        self.last_status = "goal 在途"
        future = self.action.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:                      # noqa: BLE001
            self.sending = False
            self.last_status = f"发送失败: {exc}"
            return
        if not handle.accepted:
            self.sending = False
            self.last_status = "goal 被拒绝（容差/起点校验）"
            return
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self.sending = False
        try:
            result = future.result()
            self.last_code = result.status
            # [AI] action_msgs/GoalStatus: 4=SUCCEEDED 5=CANCELED 6=ABORTED。
            # 实动被 allow_motion 压帧时，JTC 等不到回读跟上 → 容差超时 abort(6)。
            self.last_status = {
                4: "完成 ✓",
                5: "已取消",
                6: "中止 ✗（回读没跟上？检查 allow_motion:=true）",
            }.get(result.status, f"状态 {result.status}")
        except Exception as exc:                      # noqa: BLE001
            self.last_status = f"结果异常: {exc}"

    def _mark(self):
        """目标变化：臂合并 150ms 后发一个 goal，夹爪同步一帧。"""
        if not self.have_feedback:
            self.last_status = "等待 /joint_states 反馈…"
            return
        if self.sel == 6:
            self.gripper_dirty = True
        else:
            self.dirty = True

    # ---- 键位 ----
    def on_key(self, ch):
        if ch in ("1", "2", "3", "4", "5", "6"):
            self.sel = int(ch) - 1
        elif ch in ("g", "G"):
            self.sel = 6
        elif ch in ("+", "="):
            self._step(+1.0)
        elif ch in ("-", "_"):
            self._step(-1.0)
        elif ch in ("]", "}"):
            self.step_deg = STEP_CHOICES[
                (STEP_CHOICES.index(self.step_deg) + 1) % len(STEP_CHOICES)]
        elif ch in ("[", "{"):
            self.step_deg = STEP_CHOICES[
                (STEP_CHOICES.index(self.step_deg) - 1) % len(STEP_CHOICES)]
        elif ch in ("h", "H"):
            # [AI] 只动 6 个定位轴，夹爪保持
            for i, deg in enumerate(HOME_DEG):
                self.targets[i] = deg * RAD
            self._mark()
        elif ch == " ":
            # [AI] 急停语义 = 停在原地：目标同步回当前回读（臂本来就在那儿）
            if self.have_feedback:
                for i in range(len(ARM_JOINTS)):
                    if self.fb[i] is not None:
                        self.targets[i] = self.fb[i]
                self.dirty = False
                self.last_status = "目标已同步到当前回读（保持原地）"
        elif ch in ("x", "X"):
            return False
        return True

    def _step(self, sign):
        limit = GRIP_LIMIT_DEG if self.sel == 6 else LIMIT_DEG[self.sel]
        cur = self.targets[self.sel] * DEG
        cur = clamp(cur + sign * self.step_deg, 0.0, limit)
        self.targets[self.sel] = cur * RAD
        self._mark()

    def shutdown(self):
        # [AI] 退出不发任何收尾命令：JTC 保持最后指令位，臂原地不动。
        pass


def draw(stdscr, node):
    stdscr.erase()
    maxy, maxx = stdscr.getmaxyx()
    lines = []

    age = time.monotonic() - node.fb_time if node.fb_time else None
    js_ok = age is not None and age < 0.5
    lines.append(" 机械臂示教 TUI".ljust(max(1, maxx - 46)) +
                 f" 反馈: {'OK' if js_ok else '无数据'}   "
                 f"action: {'OK' if node.server_ok else '未就绪'}"
                 f"   {'发送中…' if node.sending else ''}")
    lines.append(" " + "─" * max(1, maxx - 2))
    lines.append("  关节        目标°     回读°   "
                 "0" + " " * 20 + "270   步长")
    for i, label in enumerate(ARM_LABELS):
        tgt = node.targets[i] * DEG
        f = node.fb[i]
        ftxt = "--" if f is None else f"{f * DEG:6.1f}"
        mark = "▶" if node.sel == i else " "
        step = f"{node.step_deg:.0f}°" if node.sel == i else ""
        lines.append(f" {mark}{label:<7} {tgt:7.1f} {ftxt:>8}   "
                     f"[{bar(tgt, LIMIT_DEG[i])}] {step}")
    gsel = "▶" if node.sel == 6 else " "
    lines.append(f" {gsel}夹爪    {node.targets[6] * DEG:7.1f}  {'无回读':>6}   "
                 f"[{bar(node.targets[6] * DEG, GRIP_LIMIT_DEG)}] "
                 f"{'(仅命令)' if node.sel == 6 else ''}")

    lines.append(" " + "─" * max(1, maxx - 2))
    lines.append(f"  {node.last_status}")
    if not node.have_feedback:
        lines.append("  ⚠ 等待 /joint_states —— 需先起 chassis_bringup"
                     "（controller_manager + joint_state_broadcaster）")
    lines.append("  ⚠ 实动需 chassis_bringup allow_motion:=true；回读是指令回声，"
                 "非实测位置")
    lines.append(" " + "─" * max(1, maxx - 2))
    lines.append("  [1-6] 选轴  [G] 夹爪  [+/-] 步长  [[]/[]] 步长档 1/5/15°")
    lines.append("  [H] 回 Home(0/225/270/0/0/0)   [空格] 停在原地   [X] 退出")

    for i, text in enumerate(lines):
        if 0 <= i < maxy - 1:
            stdscr.addnstr(i, 0, text, maxx - 1, curses.A_NORMAL)
    stdscr.refresh()


def run_curses(stdscr, node):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)
    rclpy.spin_once(node, timeout_sec=0.0)
    while rclpy.ok():
        try:
            ch = stdscr.getch()
        except curses.error:
            ch = -1
        if ch != -1:
            key = curses.keyname(ch).decode("utf-8", "replace")
            if ch in (27,):
                key = " "
            elif ch < 0 or key.startswith("^"):
                key = ""
            if key and not node.on_key(key):
                break
        rclpy.spin_once(node, timeout_sec=0.02)
        draw(stdscr, node)
    return True


def main(args=None):
    rclpy.init(args=args)
    node = ArmTui()
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        print("arm_tui 需要真实终端（TTY）。请用 ros2 run 直接运行，"
              "不要经 launch 后台启动。", file=sys.stderr)
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    try:
        curses.wrapper(run_curses, node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
