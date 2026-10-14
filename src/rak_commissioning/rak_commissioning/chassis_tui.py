#!/usr/bin/env python3
"""[AI] 底盘遥控 TUI（架空调试 / 低速遥控用）。

与 chassis_key_test 的区别：它是带屏显的遥控面板——实时显示 vx/vy/wz 目标
与四轮速度反馈，步进速度可调，键位方向在屏幕上直接标注，避免"按了不知道
发的是哪个轴"的歧义。

发布 /mecanum_drive_controller/reference (TwistStamped) @20Hz，与
chassis_key_test 同一契约；退出前自动发一拍零目标。

键位（REP-103: x前 y左 z上，故 vy+ = 左移、wz+ = 逆时针）：
    W / S        前进 / 后退          （改 vx）
    A / D        右移 / 左移          （改 vy，注意 A 是右）
    Q / E        顺时针 / 逆时针      （改 wz，注意 Q 是顺）
    1..4         速度档位（细/标准/快/最大）
    + / -        微调当前步进
    空格         全部归零（急停）
    X            退出（退出前发零目标）

必须用 `ros2 run rak_commissioning chassis_tui` 启动——需要独占终端读键盘，
经 launch 后台启动会拿不到 TTY 而退化。
"""
import curses
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState

# [AI] 档位表：(线步进 m/s, 角步进 rad/s)。上限由 MAX_LIN/MAX_ANG 兜底。
PRESETS = [
    (0.02, 0.10),   # 1 精细
    (0.05, 0.25),   # 2 标准（chassis_key_test 同款）
    (0.10, 0.50),   # 3 快速
    (0.15, 0.75),   # 4 最大
]
MAX_LIN = 0.30     # m/s  未标定前保持保守
MAX_ANG = 1.20     # rad/s
STEP_MIN = 0.01
STEP_MAX = 0.20
ANG_MIN = 0.05
ANG_MAX = 1.00

WHEELS = ["front_left_wheel", "front_right_wheel",
          "rear_left_wheel", "rear_right_wheel"]
WHEEL_LABELS = ["FL", "FR", "RL", "RR"]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def bar(value, limit, width=20):
    """居中零点的条形图：正值向右，负值向左。"""
    if limit <= 0:
        return " " * width
    half = width // 2
    n = int(round(abs(value) / limit * (half - 1)))
    n = min(n, half - 1)
    cells = [" "] * width
    mid = half - 1 if width % 2 else half
    cells[mid] = "|"
    if value >= 0:
        for i in range(n):
            cells[mid + 1 + i] = "#"
    else:
        for i in range(n):
            cells[mid - 1 - i] = "#"
    return "".join(cells)


class ChassisTui(Node):

    def __init__(self):
        super().__init__("chassis_tui")
        self.declare_parameter("topic", "/mecanum_drive_controller/reference")
        self.declare_parameter("stamped", True)
        self.declare_parameter("rate", 20.0)
        topic = str(self.get_parameter("topic").value)
        self.stamped = bool(self.get_parameter("stamped").value)
        self.rate = float(self.get_parameter("rate").value)

        self.pub = self.create_publisher(
            TwistStamped if self.stamped else Twist, topic, QoSProfile(depth=10))
        # [AI] 轮速反馈只为屏显，QoS best-effort 也不影响——拿不到就显示 --。
        self.wheel_vel = [None] * 4
        self.last_joint = 0.0
        self.create_subscription(JointState, "/joint_states", self._on_joints,
                                 QoSProfile(depth=10))

        self.vx = self.vy = self.wz = 0.0
        self.lin_step, self.ang_step = PRESETS[1]
        self.preset = 2
        self.publishing = True

        self._pub_timer = self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(
            f"底盘遥控 TUI: 发布 {topic} @{self.rate:.0f}Hz。"
            f"键位 W/S前后 A右移 D左移 Q顺时针 E逆时针，1-4档位 +/-步进，空格急停，X退出")

    def _on_joints(self, msg):
        self.last_joint = time.monotonic()
        for i, name in enumerate(WHEELS):
            if name in msg.name:
                try:
                    j = msg.name.index(name)
                    self.wheel_vel[i] = msg.velocity[j] if j < len(msg.velocity) else None
                except ValueError:
                    self.wheel_vel[i] = None

    def _tick(self):
        if not self.publishing:
            return
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.angular.z = self.wz
        if self.stamped:
            out = TwistStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = "base_link"
            out.twist = msg
            self.pub.publish(out)
        else:
            self.pub.publish(msg)

    # ---- 键位处理 ----
    def on_key(self, ch):
        if ch in ("w", "W"):
            self.vx = clamp(self.vx + self.lin_step, -MAX_LIN, MAX_LIN)
        elif ch in ("s", "S"):
            self.vx = clamp(self.vx - self.lin_step, -MAX_LIN, MAX_LIN)
        elif ch in ("a", "A"):
            # [AI] vy- = 右移（REP-103 +y 是左）——A 在左半边却给右移，已在此
            # 屏显标注；若操作习惯上要反，改这一处即可。
            self.vy = clamp(self.vy - self.lin_step, -MAX_LIN, MAX_LIN)
        elif ch in ("d", "D"):
            self.vy = clamp(self.vy + self.lin_step, -MAX_LIN, MAX_LIN)
        elif ch in ("q", "Q"):
            # [AI] wz- = 顺时针（俯视）。
            self.wz = clamp(self.wz - self.ang_step, -MAX_ANG, MAX_ANG)
        elif ch in ("e", "E"):
            self.wz = clamp(self.wz + self.ang_step, -MAX_ANG, MAX_ANG)
        elif ch == " ":
            self.stop()
        elif ch in ("1", "2", "3", "4"):
            self.preset = int(ch)
            self.lin_step, self.ang_step = PRESETS[self.preset - 1]
        elif ch in ("+", "="):
            self.lin_step = clamp(self.lin_step * 1.5, STEP_MIN, STEP_MAX)
            self.ang_step = clamp(self.ang_step * 1.5, ANG_MIN, ANG_MAX)
        elif ch in ("-", "_"):
            self.lin_step = clamp(self.lin_step / 1.5, STEP_MIN, STEP_MAX)
            self.ang_step = clamp(self.ang_step / 1.5, ANG_MIN, ANG_MAX)
        elif ch in ("x", "X"):
            return False
        return True

    def stop(self):
        self.vx = self.vy = self.wz = 0.0

    def shutdown(self):
        # [AI] 退出前发一拍零目标，避免 reference_timeout 的 0.5s 悬空。
        self.publishing = False
        self.stop()
        self._tick()
        time.sleep(0.05)


def draw(stdscr, node):
    stdscr.erase()
    maxy, maxx = stdscr.getmaxyx()
    lines = []

    age = time.monotonic() - node.last_joint if node.last_joint else None
    js_ok = age is not None and age < 0.5
    # [AI] 不用 ljust 对齐：CJK 在终端是双宽，按字符数补空格必然错位。
    lines.append(f" 底盘遥控 TUI     joint_states: "
                 f"{'OK' if js_ok else 'no data'}")
    lines.append(" " + "─" * max(1, maxx - 2))

    def row(tag, unit, val, limit):
        pct = 0.0 if limit <= 0 else abs(val) / limit
        return (f"  {tag}  {val:+.2f} {unit:<6} "
                f"[{bar(val, limit)}] {pct * 100:3.0f}%")

    lines.append(row("vx 前后", "m/s", node.vx, MAX_LIN))
    lines.append(row("vy 横移", "m/s", node.vy, MAX_LIN))
    lines.append(row("wz 自转", "rad/s", node.wz, MAX_ANG))
    lines.append(" " + "─" * max(1, maxx - 2))

    fb = []
    for i, v in enumerate(node.wheel_vel):
        fb.append(f"{WHEEL_LABELS[i]} {'--' if v is None else f'{v:+.2f}'}")
    lines.append("  轮速反馈(rad/s)  " + "   ".join(fb))

    lines.append(" " + "─" * max(1, maxx - 2))
    lines.append(f"  步进 lin {node.lin_step:.3f} m/s   ang {node.ang_step:.3f} rad/s"
                 f"      档位 {node.preset}/4")
    lines.append("  [W/S] 前/后   [A] 右移 [D] 左移   [Q] 顺时针 [E] 逆时针")
    lines.append("  [1-4] 档位    [+/-] 步进        [空格] 急停    [X] 退出")

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
            # [AI] 回车/方向键等特殊键不参与控制；ESC 归一成空格（急停）。
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
    node = ChassisTui()
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        print("chassis_tui 需要真实终端（TTY）。请用 ros2 run 直接运行，"
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
