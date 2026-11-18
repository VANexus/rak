#!/usr/bin/env python3
"""can0 跑前守栏：一屏打分表说清"现在能不能跑真机"。

无 ROS 依赖，stdlib only，**默认只读**（不发 CAN 帧、不 kill 任何东西）。
只有显式 `--fix` 才会清孤儿进程，且**先打印将杀的 PID 与命令行**再动手。

三段判据：

[1] 链路 + 板子（原有行为，语义不变）
  - interface 缺失 / down          -> FAIL，提示 can0-up.service（含 Orin pinmux）
  - 非 1 Mbps / 无 bitrate          -> WARN（下位机按 1 Mbps 配，不匹配整条哑掉）
  - 2 s 被动收帧 0 且 TX 错误增长   -> FAIL，板子没上线（供电/接线/终端电阻）
  - 收帧 > 0                        -> PASS，进 G1（chassis_bringup）

[2] 进程单例（本轮读数全废的直接原因）
  device_container（NMT 主站）必须 1 个；controller_manager / i2c_imu /
  robot_state_publisher / rf2o_laser_odometry / slam_toolbox ≤ 1。
  多于上限 -> FAIL，列出 PID + 命令行前 120 字符 + 可复制清理命令。

[3] 静态不变量（改过没生效的那类坑）
  - `src/rak_canopen/config/bus.yaml` 的 `chassis.polling` 必须 false
    （true 时 SDO 轮询覆盖 TPDO 真值 -> `0x2020` 恒定不变）
  - URDF 轮关节 `state_sign` 必须全 1、且 4 个都在
  - `install/` 与 `install/architecture/` 并存 -> WARN，并打印 AMENT_PREFIX_PATH
    顺序与**实际解析到**的 share 目录（并存会静默喂旧 URDF）
  - `slam.launch.py` 的 rf2o `init_pose_from_topic` 必须为空（非空时 rf2o 一帧不处理）
  - `librak_chassis_plugin.so` 的 mtime 早于 URDF -> FAIL（改了 URDF 没重编插件，
    旧 .so 继续按老 state_sign 翻转）。FAIL 行直接给出重编命令（两棵树）与复验方法

用法::

    python3 scripts/can0_preflight.py                  # 全量守栏
    python3 scripts/can0_preflight.py --link-only      # 老行为：只要链路 + 板子
    python3 scripts/can0_preflight.py --fix            # 先打印将 kill 的 PID，再清孤儿
                                                       # 白名单：绝不杀 device_container（NMT
                                                       # 主站）与任何 `ros2 launch` 父进程
    python3 scripts/can0_preflight.py --root /tmp/fake # 静态判据指向别处的树（测试用）

退出码：
  0 = 无 FAIL（WARN 允许，已在表里点名）
  1 = 有 FAIL：链路/板子/进程单例/静态不变量任一
  2 = 参数或权限问题（--iface 为空、--window <= 0 等）
"""
import argparse
import os
import re
import socket
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SYSFS = "/sys/class/net/{iface}/statistics/{name}"
CMD_WIDTH = 120

# name -> (pgrep 模式, 上限, 判据说明)
PROC_RULES = [
    ("device_container", "device_container", 1,
     "NMT 主站；>1 = 双主站，从站会被踢出 Operational"),
    ("controller_manager", "controller_manager", 1,
     ">1 = 双 controller_manager，/joint_states 与命令互相打架"),
    ("i2c_imu", "i2c_imu", 1,
     ">1 = 多个 i2c_imu，频率统计与 /imu/data_raw 全废（本轮叠到 3 个）"),
    ("robot_state_publisher", "robot_state_publisher", 1,
     ">1 = 多个 TF 发布者（本轮 7 个），TF 树随机跳变"),
    ("rf2o_laser_odometry", "rf2o_laser_odometry", 1,
     ">1 = 多个激光里程计抢 /odom 与 odom->base_footprint"),
    ("slam_toolbox", "slam_toolbox", 1,
     ">1 = 多张 /map 互相覆盖"),
]


class Row:
    """一条判据。level: PASS / WARN / FAIL / INFO。"""

    __slots__ = ("name", "level", "detail", "hint")

    def __init__(self, name, level, detail="", hint=""):
        self.name, self.level, self.detail, self.hint = name, level, detail, hint

    def render(self):
        line = f"[{self.level:4s}] {self.name:22s} {self.detail}"
        return line.rstrip()


# ---------------------------------------------------------------- [1] 链路 + 板子

def read_stat(iface, name, sysfs=SYSFS):
    try:
        with open(sysfs.format(iface=iface, name=name)) as fh:
            return int(fh.read().strip())
    except OSError:
        return None


def link_dump(iface):
    r = subprocess.run(["ip", "-details", "link", "show", iface],
                       capture_output=True, text=True)
    return r.stdout if r.returncode == 0 else ""


def link_state(dump):
    # "state UP" / "state ERROR-PASSIVE" 出现在 can state 行；两处都找
    m = re.search(r"state (\S+)", dump)
    state = m.group(1) if m else "?"
    berr = re.search(r"berr-counter tx (\d+) rx (\d+)", dump)
    return state, ("bitrate" in dump), (
        (int(berr.group(1)), int(berr.group(2))) if berr else None)


def count_frames(iface, window):
    """被动收帧计数。socket 绑定失败返回 None（无权限/接口不在）。"""
    try:
        s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        s.bind((iface,))
    except (OSError, PermissionError) as e:
        print(f"[!!] 抓帧失败（{e}）；rx 计数改用 sysfs 增量")
        return None
    frames = 0
    deadline = time.monotonic() + window
    while time.monotonic() < deadline:
        s.settimeout(max(0.05, deadline - time.monotonic()))
        try:
            s.recvfrom(16)
        except socket.timeout:
            break
        frames += 1
    s.close()
    return frames


def check_link(iface, window):
    """原有 G0 判据，原样保留语义，只改成返回 Row。"""
    rows = []
    dump = link_dump(iface)
    if not dump:
        return [Row(iface, "FAIL", "接口不存在",
                    "安装并启用 can0-up.service（含 Orin AON pinmux，漏写时 can0 照常 "
                    "UP 但一帧都收不到）")]
    state, has_bitrate, berr = link_state(dump)
    up = "UP" in dump.split("\n")[0]
    rows.append(Row(iface, "PASS" if up else "FAIL",
                    f"state={state} up={up} bitrate={'有' if has_bitrate else '缺'}"
                    + (f" berr(tx,rx)=({berr[0]},{berr[1]})" if berr else ""),
                    "" if up else "sudo systemctl start can0-up.service"))
    if not up:
        return rows
    if not has_bitrate or "1000000" not in dump:
        rows.append(Row("bitrate", "WARN", "未确认 1 Mbps",
                        "下位机按 1 Mbps 配置，不匹配会整条哑掉"))

    rx0 = read_stat(iface, "rx_packets")
    terr0 = read_stat(iface, "tx_errors")
    frames = count_frames(iface, window)
    if frames is None:
        rx1 = read_stat(iface, "rx_packets")
        frames = (rx1 - rx0) if (rx1 is not None and rx0 is not None) else 0
    terr1 = read_stat(iface, "tx_errors")
    dtx = None if (terr0 is None or terr1 is None) else terr1 - terr0

    if frames and frames > 0:
        rows.append(Row("板子在总线", "PASS",
                        f"{window:.1f}s 被动收帧 {frames}；进 G1（chassis_bringup）"))
        return rows
    rows.append(Row("板子在总线", "FAIL", f"{window:.1f}s 被动收帧 0"
                    f"（tx_errors Δ{dtx}）"))
    if dtx is not None and dtx > 0:
        rows.append(Row("诊断", "INFO", "零入帧 + TX 错误增长 = 我发帧无人 ACK",
                        "排查：板供电 -> CAN_H/CAN_L 接线 -> 终端电阻 -> 板上固件是否起来"))
    elif berr and berr[0] > 0 and berr[1] == 0:
        rows.append(Row("诊断", "INFO", f"berr tx={berr[0]} rx=0（历史 TX 从未被 ACK）",
                        "同上：供电/接线/终端电阻"))
    else:
        rows.append(Row("诊断", "INFO", "零入帧且无 TX 错误 = 窗口内没有发送者",
                        "先跑一次 bringup（它会发 NMT），或检查 pinmux/driver"))
    return rows


# ---------------------------------------------------------------- [2] 进程单例

def list_procs(pattern):
    """pgrep -af <pattern> -> [(pid, cmdline)]。排除本脚本自己与 pgrep 自身。"""
    try:
        r = subprocess.run(["pgrep", "-af", pattern], capture_output=True, text=True)
    except OSError:
        return []
    out = []
    me = os.getpid()
    for line in r.stdout.splitlines():
        pid_s, _, cmd = line.partition(" ")
        if not pid_s.isdigit():
            continue
        pid = int(pid_s)
        if pid == me or "can0_preflight" in cmd or "pgrep -af" in cmd:
            continue
        out.append((pid, cmd.strip()))
    return out


def check_singletons(lister=None):
    """单例断言。lister(pattern) -> [(pid, cmd)]，测试可注入。"""
    lister = lister or list_procs
    rows = []
    for name, pattern, cap, why in PROC_RULES:
        found = lister(pattern)
        n = len(found)
        if n > cap:
            detail = f"{n} 个（上限 {cap}）：" + "；".join(
                f"pid {p} {c[:CMD_WIDTH]}" for p, c in found)
            if name == "device_container":
                hint = (f"为什么必须唯一：{why}。**别用 kill**：先 `candump can0,080:7FF` "
                        "看是否两条 SYNC 节奏，再人工确认哪一个是真主站（本工具 --fix 也不杀它）")
            else:
                hint = (f"为什么必须唯一：{why}。清孤儿：kill "
                        + " ".join(str(p) for p, _ in found[1:])
                        + "   # 保留最早的那个；或重跑 --fix（不杀主站与 ros2 launch 父进程）")
            rows.append(Row(name, "FAIL", detail, hint))
        elif n == 0 and name == "device_container":
            rows.append(Row(name, "WARN", "0 个：NMT 主站没起来",
                            "尚未 bringup 时正常；若已 launch 则是主站挂了"))
        else:
            rows.append(Row(name, "PASS", f"{n} 个"))
    bad = "; ".join(r.name for r in rows if r.level == "FAIL")
    rows.insert(0, Row("进程单例", "FAIL" if bad else "PASS",
                       bad and f"超限：{bad}" or "全部 ≤ 上限且唯一"))
    return rows


def kill_veto(name, cmd):
    """--fix 的白名单：返回非空 = 绝不杀，理由原样打印给操作员。

    两条硬边界：
      - `device_container` 是 NMT 主站。双主站时"哪个是真的"只能人工判断，
        本工具杀错一个就把从站踢出 Operational，比留着更坏。
      - 命令行含 `ros2 launch` 的是 launch 父进程，杀它连带整棵子进程树
        （包括正在跑的主站与 controller_manager），等于随手拆掉现场。
    """
    if name == "device_container":
        return "NMT 主站，本工具不杀（双主站要人工确认哪一个是真主站）"
    if re.search(r"\bros2\s+launch\b", cmd):
        return "ros2 launch 父进程，杀它会连带整棵子进程树"
    return ""


def fix_singletons(rows, lister=None, killer=None):
    """--fix：只杀超出上限的多余子进程。先打印再杀，白名单内的一律跳过。"""
    lister = lister or list_procs
    to_kill, skipped = [], []
    for name, pattern, cap, _why in PROC_RULES:
        found = lister(pattern)
        if len(found) <= cap:
            continue
        for pid, cmd in found[cap:]:
            veto = kill_veto(name, cmd)
            (skipped if veto else to_kill).append((name, pid, cmd, veto))

    if skipped:
        print(f"[--fix] 跳过 {len(skipped)} 个受保护进程（不杀）：")
        for name, pid, cmd, veto in skipped:
            print(f"  {name:22s} pid {pid:7d} {cmd[:CMD_WIDTH]}")
            print(f"  {'':22s}        理由：{veto}")
    if not to_kill:
        print("[--fix] 没有可清理的进程。"
              + ("（受保护的那些需人工处理）" if skipped else ""))
        return 0
    print(f"[--fix] 将 kill {len(to_kill)} 个进程（保留每个同名组的最早 PID）：")
    for name, pid, cmd, _veto in to_kill:
        print(f"  {name:22s} pid {pid:7d} {cmd[:CMD_WIDTH]}")
    killer = killer or (lambda pid: os.kill(pid, 15))
    killed = 0
    for _name, pid, _cmd, _veto in to_kill:
        try:
            killer(pid)
            killed += 1
        except OSError as e:
            print(f"  [!!] pid {pid} 取消失败：{e}")
    print(f"[--fix] 已发 SIGTERM {killed} 个；3 s 后复查还在的再用 kill -9。")
    return killed


# ---------------------------------------------------------------- [3] 静态不变量

def yaml_scalar(text, section, key):
    """在一层缩进的顶层 section 里取 key 的标量值（够读 bus.yaml，不引 yaml）。"""
    inside = False
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].rstrip() if raw.lstrip().startswith("#") else raw.rstrip()
        if not line.strip():
            continue
        if re.match(rf"^{re.escape(section)}\s*:", line):
            inside = True
            continue
        if inside:
            if re.match(r"^\S", line):
                break
            m = re.match(rf"^\s+{re.escape(key)}\s*:\s*(\S+)", line)
            if m:
                return m.group(1).strip().strip('"\'')
    return None


def check_polling(bus_path):
    try:
        text = open(bus_path, encoding="utf-8").read()
    except OSError as e:
        return Row("bus.yaml polling", "WARN", f"读不到 {bus_path}（{e}）")
    val = yaml_scalar(text, "chassis", "polling")
    if val is None:
        return Row("bus.yaml polling", "WARN", "chassis 段没有 polling 键",
                   "上游默认 true；显式写 false")
    ok = val.lower() == "false"
    return Row("bus.yaml polling", "PASS" if ok else "FAIL",
               f"chassis.polling = {val}",
               "" if ok else "true 时 SDO 轮询覆盖 TPDO 真值 -> 0x2020 恒定不变；改 false")


def check_state_sign(urdf_path):
    try:
        text = open(urdf_path, encoding="utf-8").read()
    except OSError as e:
        return Row("URDF state_sign", "WARN", f"读不到 {urdf_path}（{e}）")
    vals = re.findall(r'<param\s+name="state_sign"\s*>\s*([^<]*?)\s*</param>', text)
    if not vals:
        return Row("URDF state_sign", "WARN", "URDF 里没有 state_sign 参数（版本太旧或已换名）")
    bad = [v for v in vals if v.strip() not in ("1", "1.0", "+1")]
    detail = f"共 {len(vals)} 个：{', '.join(vals)}"
    if bad:
        return Row("URDF state_sign", "FAIL", detail + f"；非 1 的 {len(bad)} 个",
                   "必须全 1（固件已乘 encoder_sign）；改完还要重编 rak_chassis 插件")
    if len(vals) != 4:
        return Row("URDF state_sign", "WARN", detail + "（期望 4 个轮关节）")
    return Row("URDF state_sign", "PASS", detail)


def check_install_trees(root, env=None):
    """两套安装树是否并存 + 打印实际解析到的 share 目录。"""
    env = env if env is not None else os.environ
    plain, arch = os.path.join(root, "install"), os.path.join(root, "install", "architecture")
    both = os.path.isdir(plain) and os.path.isdir(arch)
    rows = []
    prefix = [p for p in env.get("AMENT_PREFIX_PATH", "").split(":") if p]
    order = " > ".join(prefix[:4]) if prefix else "(空：尚未 source)"
    resolved = resolve_share(root, "rak_description", "urdf/rak_chassis.urdf.xacro", env)
    rows.append(Row("安装树", "WARN" if both else "PASS",
                    ("install/ 与 install/architecture/ 并存" if both else "只有一套安装树"),
                    "并存会静默喂旧 URDF：改 URDF 要么两棵树一起 build，"
                    "要么确认前缀顺序" if both else ""))
    rows.append(Row("AMENT 顺序", "INFO", order[:SUM_WIDTH],
                    "get_package_share_directory 按这个顺序解析；顺序错就取到旧树"))
    rows.append(Row("实际 URDF", "INFO" if resolved else "WARN",
                    resolved or "未解析到 rak_chassis.urdf.xacro（环境没 source？）"))
    return rows


SUM_WIDTH = 200


def resolve_share(root, pkg, rel, env=None):
    """按 AMENT_PREFIX_PATH 顺序（缺省回落到仓库两棵树）找第一个存在的文件。"""
    env = env if env is not None else os.environ
    cands = []
    prefix = [p for p in env.get("AMENT_PREFIX_PATH", "").split(":") if p]
    if not prefix:
        prefix = [os.path.join(root, "install", "architecture"), os.path.join(root, "install")]
    for p in prefix:
        # ament 标准前缀是 <install-base>/<pkg>，但本仓也用 install-base 本身当前缀，
        # 所以两种拼法都试（<prefix>/share/<pkg>/... 与 <prefix>/<pkg>/share/<pkg>/...）。
        cands.append(os.path.join(p, "share", pkg, rel))
        cands.append(os.path.join(p, pkg, "share", pkg, rel))
    for c in cands:
        if os.path.isfile(c):
            return c
    return ""


def check_rf2o_init(launch_path):
    try:
        text = open(launch_path, encoding="utf-8").read()
    except OSError as e:
        return Row("rf2o init_pose", "WARN", f"读不到 {launch_path}（{e}）")
    m = re.search(r'["\']init_pose_from_topic["\']\s*:\s*(?:ParameterValue\(\s*)?["\']([^"\']*)["\']',
                  text)
    if not m:
        return Row("rf2o init_pose", "WARN", "slam.launch.py 里没写 init_pose_from_topic",
                   "显式置空字符串；留上游默认 /base_pose_ground_truth 时 rf2o 一帧不处理")
    val = m.group(1)
    return Row("rf2o init_pose", "PASS" if val == "" else "FAIL",
               f'init_pose_from_topic = "{val}"',
               "" if val == "" else "必须为空（上游默认指向 Gazebo 话题，非空/默认时一帧不处理）")


def _tree(so_path):
    """install/lib/... -> "install"；install/architecture/... -> "architecture"。"""
    parts = so_path.split(os.sep)
    return "architecture" if "architecture" in parts else (parts[-5] if len(parts) > 5 else "install")


def check_plugin_mtime(root, env=None):
    """插件 .so 必须不早于 URDF：早于 = 改了 URDF 没重编，旧 .so 按老 state_sign 翻转。"""
    env = env if env is not None else os.environ
    urdf = resolve_share(root, "rak_description", "urdf/rak_chassis.urdf.xacro", env)
    src_urdf = os.path.join(root, "src", "rak_description", "urdf", "rak_chassis.urdf.xacro")
    newest_urdf = max([p for p in (urdf, src_urdf) if p and os.path.isfile(p)],
                      key=os.path.getmtime, default="")
    if not newest_urdf:
        return Row("插件 mtime", "WARN", "找不到 URDF，无法比较 mtime")
    sos = [os.path.join(root, "install", "rak_chassis", "lib", "librak_chassis_plugin.so"),
           os.path.join(root, "install", "architecture", "rak_chassis", "lib",
                        "librak_chassis_plugin.so")]
    sos = [s for s in sos if os.path.isfile(s)]
    if not sos:
        return Row("插件 mtime", "WARN", "找不到 librak_chassis_plugin.so（没 build？）")
    t_urdf = os.path.getmtime(newest_urdf)
    stale = [s for s in sos if os.path.getmtime(s) < t_urdf]
    fmt = lambda t: time.strftime("%m-%d %H:%M", time.localtime(t))  # noqa: E731
    detail = (f"URDF {fmt(t_urdf)}；" + "；".join(
        f"{_tree(s)} {fmt(os.path.getmtime(s))}" for s in sos))
    if stale:
        return Row("插件 mtime", "FAIL", detail + f"；{len(stale)} 个 .so 早于 URDF",
                   "旧 .so 不认新的 state_sign，会继续按老逻辑翻转。重编（两棵树都要）：\n"
                   "colcon build --packages-select rak_chassis "
                   "--build-base build/architecture --install-base install/architecture\n"
                   "colcon build --packages-select rak_chassis"
                   "   # 默认 build/ + install/\n"
                   "复验：重跑 python3 scripts/can0_preflight.py，"
                   "本行必须变 [PASS] 插件 mtime 才能动真机")
    return Row("插件 mtime", "PASS", detail)


def check_static(root, env=None):
    root = root or ROOT
    rows = [
        check_polling(os.path.join(root, "src", "rak_canopen", "config", "bus.yaml")),
        check_state_sign(os.path.join(root, "src", "rak_description", "urdf",
                                      "rak_chassis.urdf.xacro")),
        check_rf2o_init(os.path.join(root, "src", "rak_bringup", "launch", "slam.launch.py")),
        check_plugin_mtime(root, env),
    ]
    rows.extend(check_install_trees(root, env))
    bad = [r.name for r in rows if r.level == "FAIL"]
    rows.insert(0, Row("静态不变量", "FAIL" if bad else "PASS",
                       f"不满足：{', '.join(bad)}" if bad else "全部满足"))
    return rows


# ---------------------------------------------------------------- main

def print_rows(rows):
    for r in rows:
        print(r.render())
        for line in (r.hint.splitlines() if r.hint else []):
            print(f"       -> {line}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--window", type=float, default=2.0)
    ap.add_argument("--link-only", action="store_true",
                    help="只要老行为（链路 + 板子），跳过单例与静态判据")
    ap.add_argument("--fix", action="store_true",
                    help="清孤儿进程（先打印将杀的 PID 与命令行；"
                         "不杀 device_container 与 ros2 launch 父进程）")
    ap.add_argument("--root", default=ROOT, help="仓库根（静态判据指向它）")
    args = ap.parse_args(argv)

    if not args.iface or args.window <= 0:
        print("[!!] 参数错误：--iface 不能为空，--window 必须 > 0", file=sys.stderr)
        return 2

    print(f"== 跑前守栏（{args.iface}，窗口 {args.window:.1f}s，"
          f"{'只读' if not args.fix else '--fix 会清进程'}）==")
    rows = check_link(args.iface, args.window)
    link_fail = any(r.level == "FAIL" for r in rows)
    print_rows(rows)

    if not args.link_only:
        print()
        proc_rows = check_singletons()
        print_rows(proc_rows)
        if args.fix:
            fix_singletons(proc_rows)
        print()
        static_rows = check_static(args.root)
        print_rows(static_rows)
        rows.extend(proc_rows)
        rows.extend(static_rows)

    fails = [r for r in rows if r.level == "FAIL"]
    warns = [r for r in rows if r.level == "WARN"]
    print()
    if fails:
        print(f"== 退出码 1：{len(fails)} 项 FAIL -> {', '.join(r.name for r in fails)}==")
        print("   下一步：先清上面点名的项（进程孤儿用 --fix），再重跑本脚本。")
        return 1
    print(f"== 退出码 0：无 FAIL，{len(warns)} 项 WARN"
          + (f"（{', '.join(r.name for r in warns)}）" if warns else "") + " ==")
    if link_fail:
        return 1
    print("   下一步：可进 G1 `ros2 launch rak_bringup chassis_bringup.launch.py`。")
    return 0


if __name__ == "__main__":
    sys.exit(main())