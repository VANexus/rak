#!/usr/bin/env python3
"""断言 `rak_chassis.urdf.xacro` 仍然满足两个 hardware 组件的初始化契约。

[AI] 2026-10-15 新建，起因：把机械臂的真实几何并入整车 URDF（替换掉原来的
零长度控制桩链）—— 那一步会大改运动学树，而 `RakSystemHardware::on_init`
对 `<ros2_control>` 的内容有**一整套硬性要求**：

    src/rak_system.cpp:37   info.joints.size() 必须恰好 11（4 轮 + 7 臂）
    src/rak_system.cpp:53-75 按固定顺序校验名字、接口种类、min/max 非空
    src/rak_system.cpp:57-62 gripper_joint 必须**没有** state_interface
    Jazzy component_parser  <ros2_control> 里每个 joint 都必须在 URDF 树里存在
    import_joint_limiters() 读 URDF <limit> 作为位置限幅依据

⚠️ 违反时 `on_init` 是**静默返回 ERROR**（没有日志），现象只是 controller_manager
反复 "Waiting for data on 'robot_description'" —— 真机上极难定位。所以离线把它钉住。

跑法：colcon test --packages-select rak_chassis
      或直接 python3 src/rak_chassis/test/validate_chassis_urdf.py

⚠️ 只用 `xacro` CLI，**不要 `import xacro`**：本机 PATH 上的 python3 常是 miniconda，
缺 PyYAML，会在 `$(arg …)` 处抛 "substitution args not supported: No module named 'yaml'"。
同理本文件不 import yaml。
"""
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

URDF = (Path(__file__).resolve().parents[2]
        / "rak_description" / "urdf" / "rak_chassis.urdf.xacro")

WHEELS = ["front_left_wheel", "front_right_wheel",
          "rear_left_wheel", "rear_right_wheel"]
ARM = ["j1_base", "j2_shoulder", "j3_elbow",
       "j4_forearm", "j5_wrist_pitch", "j6_wrist_roll"]
GRIPPER = "gripper_joint"

# 契约 §1（FROZEN）：全轴 0..270°，仅 J2 是 0..255°（实测硬件约束）
LIMIT_DEG = {"j1_base": 270.0, "j2_shoulder": 255.0, "j3_elbow": 270.0,
             "j4_forearm": 270.0, "j5_wrist_pitch": 270.0, "j6_wrist_roll": 270.0}
AXIS = {"j1_base": "0 0 1", "j2_shoulder": "0 1 0", "j3_elbow": "0 -1 0",
        "j4_forearm": "0 0 1", "j5_wrist_pitch": "0 1 0", "j6_wrist_roll": "0 0 1"}

# 臂链上每个旋转关节的 (parent, child)
CHAIN = [("j1_base", "arm_base_link", "shoulder_link"),
         ("j2_shoulder", "shoulder_link", "upper_arm_link"),
         ("j3_elbow", "upper_arm_link", "forearm_link"),
         ("j4_forearm", "forearm_link", "forearm_roll_link"),
         ("j5_wrist_pitch", "forearm_roll_link", "wrist_pitch_link"),
         ("j6_wrist_roll", "wrist_pitch_link", "tool0")]

# 重构证据：这些 link 必须有**长度非零**的圆柱 collision（原来的桩链是空的）
CYL_COLLISION = {"upper_arm_link": 0.10, "forearm_link": 0.11,
                 "forearm_roll_link": 0.05, "wrist_pitch_link": 0.09}

_fail = []


def check(ok, msg):
    print(("  ok   " if ok else "  FAIL ") + msg)
    if not ok:
        _fail.append(msg)
    return ok


def _close(a, b, tol=1e-9):
    """两个字符串形式的浮点数是否相等（None/空/非法一律算不等）。"""
    try:
        return abs(float(a) - float(b)) < tol
    except (TypeError, ValueError):
        return False


def expand():
    try:
        out = subprocess.run(["xacro", str(URDF)],
                             check=True, capture_output=True, text=True)
    except FileNotFoundError:
        out = subprocess.run(["ros2", "run", "xacro", "xacro", str(URDF)],
                             check=True, capture_output=True, text=True)
    return out.stdout


def main():
    if not URDF.exists():
        print(f"找不到 {URDF}")
        return 1
    text = expand()
    root = ET.fromstring(text)

    # ---- A：两段 <ros2_control>（底盘板 / 机械臂板，各自独立故障域） ----
    print("A. robot / <ros2_control>（双下位机：两段）")
    check(root.get("name") == "rak_chassis", 'robot name == "rak_chassis"')
    rcs = root.findall("ros2_control")
    check(len(rcs) == 2, f"<ros2_control> 元素恰好 2 个（底盘 + 臂，实为 {len(rcs)}）")
    if len(rcs) != 2:
        return 1
    rc_by_name = {rc.get("name"): rc for rc in rcs}
    check(set(rc_by_name) == {"rak_chassis", "rak_arm"},
          f"两段 name 必须是 rak_chassis / rak_arm（实为 {sorted(rc_by_name)}）")
    if set(rc_by_name) != {"rak_chassis", "rak_arm"}:
        return 1
    expect_plugin = {
        "rak_chassis": "rak_chassis/CanopenChassis",
        "rak_arm": "rak_chassis/CanopenArm",
    }
    for seg, want in expect_plugin.items():
        plugin = rc_by_name[seg].find("hardware/plugin")
        check(plugin is not None and plugin.text == want,
              f"{seg}: hardware plugin == {want}")
    # CANopen 薄桥（ADR-0024）：两段各自绑一个 proxy driver 节点，安全门同源。
    seg_params = {
        seg: {x.get("name"): (x.text or "").strip()
              for x in rc.findall("hardware/param")}
        for seg, rc in rc_by_name.items()
    }
    check(seg_params["rak_chassis"].get("driver_node") == "chassis"
          and seg_params["rak_arm"].get("driver_node") == "arm",
          f"两段的 driver_node 必须是 chassis / arm（实为 "
          f"{seg_params['rak_chassis'].get('driver_node')!r} / "
          f"{seg_params['rak_arm'].get('driver_node')!r}）")
    check(seg_params["rak_chassis"].get("allow_motion")
          == seg_params["rak_arm"].get("allow_motion"),
          f"两段的 allow_motion 必须一致（运动许可只有一个来源，实为 "
          f"{seg_params['rak_chassis'].get('allow_motion')!r} vs "
          f"{seg_params['rak_arm'].get('allow_motion')!r}）")
    for seg in ("rak_chassis", "rak_arm"):
        for key in ("cmd_seq_object", "cmd_flags_object"):
            check(bool(seg_params[seg].get(key)),
                  f"{seg}: {key} 非空（[V5] §5 命令活性对象）")
        for dead in ("host_build_id", "can_channel"):
            check(dead not in seg_params[seg],
                  f"{seg}: 不再有 v4 参数 {dead}")

    # ---- A2：OD 对象映射（CANopen 薄桥的唯一职责）----
    print("A2. 关节 → OD 对象映射")
    seen_objects = {}
    for j in rc_by_name["rak_chassis"].findall("joint") + rc_by_name["rak_arm"].findall("joint"):
        n = j.get("name")
        p = {x.get("name"): (x.text or "").strip() for x in j.findall("param")}
        obj = p.get("command_object", "")
        check(bool(obj) and p.get("command_type") in ("int16", "uint16"),
              f"{n}: command_object/command_type 非空（{obj!r}/{p.get('command_type')!r}）")
        check(_close(p.get("command_scale"), p.get("command_scale")) and
              float(p.get("command_scale") or 0) != 0.0,
              f"{n}: command_scale 非零（实为 {p.get('command_scale')!r}）")
        if obj:
            check(obj not in seen_objects,
                  f"{n}: command_object {obj} 唯一（已用于 {seen_objects.get(obj)}）")
            seen_objects[obj] = n
        if p.get("state_object"):
            check(bool(p.get("state_type")),
                  f"{n}: 有 state_object 就必须有 state_type")
            check(_close(p.get("state_scale"), p.get("state_scale")) and
                  float(p.get("state_scale") or 0) != 0.0,
                  f"{n}: state_scale 非零（实为 {p.get('state_scale')!r}）")
    gripper = rc_by_name["rak_arm"].find("joint[@name=\"gripper_joint\"]")
    check(gripper is not None and gripper.find("param[@name=\"state_object\"]") is None,
          "gripper_joint: 无 state_object（无物理反馈通道，契约 §2.3）")
    check("mock_components" not in text,
          "全文不含 mock_components（仿真插件不得混进真机树）")

    # ---- B：关节按段归属（底盘段只有轮、臂段只有臂），合并顺序不变 ----
    print("B. <ros2_control> 关节清单与顺序")
    rc_chassis = rc_by_name["rak_chassis"]
    rc_arm = rc_by_name["rak_arm"]
    # 运动学树里的 <joint>：F 的存在性（component_parser 的 getJoint 规则）与
    # D2 的限位比对都要用，所以在这里一次建好。
    tree_joints = {j.get("name"): j for j in root.findall("joint")}
    wheel_names = [j.get("name") for j in rc_chassis.findall("joint")]
    arm_names = [j.get("name") for j in rc_arm.findall("joint")]
    check(wheel_names == WHEELS, f"底盘段只声明 4 轮（实为 {wheel_names}）")
    check(arm_names == ARM + [GRIPPER],
          f"臂段只声明 j1..j6 + gripper（实为 {arm_names}）")
    # 后续 C/D/E/D2/F 沿用合并视图，断言本身一条都不放松
    rc_joints = rc_chassis.findall("joint") + rc_arm.findall("joint")
    names = [j.get("name") for j in rc_joints]
    expect = WHEELS + ARM + [GRIPPER]
    check(names == expect,
          f"两段合并顺序 == 4 轮 + j1..j6 + gripper（实为 {names}）")

    # ---- C/D/E：每个关节的接口种类 ----
    print("C/D/E. 接口种类（轮 / 臂 / 夹爪）")
    for j in rc_joints:
        n = j.get("name")
        ci = j.findall("command_interface")
        si = j.findall("state_interface")
        if n in WHEELS:
            check([c.get("name") for c in ci] == ["velocity"]
                  and [s.get("name") for s in si] == ["velocity"],
                  f"{n}: 1 个 velocity 命令 + velocity 状态"
                  "（轮 position counts 未标定，不导出）")
        elif n in ARM:
            ok = (len(ci) == 1 and ci[0].get("name") == "position"
                  and len(si) == 1 and si[0].get("name") == "position")
            check(ok, f"{n}: 1 个 position 命令 + 恰好 1 个 position 状态")
        elif n == GRIPPER:
            check(len(ci) == 1 and ci[0].get("name") == "position" and len(si) == 0,
                  f"{n}: 1 个 position 命令 + **0 个状态**（无反馈通道，契约 §2.3）")

    # ---- D2：命令 min/max 必须**等于**该关节的 <limit>，不是「非空就行」----
    # [AI] 原来只查非空是纸老虎：把 J2 的命令 max 写回 270°（超实测硬件限 255°）
    # 它照样全绿 —— 而这个文件存在的唯一理由就是钉住这份契约。
    # CM 的有效限幅 = <limit> 与命令 min/max 的**交集**（import_joint_limiters
    # 读 <limit>，JointSaturationLimiter 用命令界），两层必须同值，
    # 否则注释里说的「双保险」就变成互相打架。臂与夹爪用**同一套**断言。
    print("D2. 命令 min/max == <limit> lower/upper")
    for n in ARM + [GRIPPER]:
        j = next((x for x in rc_joints if x.get("name") == n), None)
        lim = tree_joints.get(n)
        if j is None or lim is None or lim.find("limit") is None:
            check(False, f"{n}: 找得到 ros2_control joint 与 <limit>")
            continue
        p = {x.get("name"): (x.text or "").strip()
             for x in j.findall("command_interface/param")}
        lo = lim.find("limit").get("lower")
        hi = lim.find("limit").get("upper")
        check(_close(p.get("min"), lo) and _close(p.get("max"), hi),
              f"{n}: 命令 [{p.get('min')}, {p.get('max')}] == <limit> [{lo}, {hi}]")

    # ---- F：11 个名字都必须存在于 URDF 树 ----
    print("F. 运动学树里存在性（component_parser getJoint 规则）")
    missing = [n for n in expect if n not in tree_joints]
    check(not missing, f"11 个关节全部在树里（缺失：{missing}）")

    # ---- G：臂链结构 / 轴向 / 限位 ----
    print("G. 臂链结构 / 轴向 / 限位")
    for jn, p, c in CHAIN:
        j = tree_joints.get(jn)
        if j is None:
            check(False, f"{jn} 在树里")
            continue
        gp = j.find("parent").get("link")
        gc = j.find("child").get("link")
        check(gp == p and gc == c, f"{jn}: {gp} -> {gc}")
        ax = j.find("axis")
        check(ax is not None and ax.get("xyz") == AXIS[jn],
              f"{jn}: axis == {AXIS[jn]}")
        lim = j.find("limit")
        want = LIMIT_DEG[jn] * math.pi / 180.0
        ok = (lim is not None and abs(float(lim.get("lower")) - 0.0) < 1e-6
              and abs(float(lim.get("upper")) - want) < 1e-6)
        check(ok, f"{jn}: <limit> == 0..{LIMIT_DEG[jn]}°")
    gj = tree_joints.get(GRIPPER)
    check(gj is not None and gj.find("parent").get("link") == "tool0"
          and gj.find("child").get("link") == "gripper_link",
          "gripper_joint: tool0 -> gripper_link")

    # ---- H：重构真的发生了（桩链是空 link，真实几何有非零 collision）----
    print("H. 真实几何确实并进来了（不是零长桩）")
    for link, L in CYL_COLLISION.items():
        el = next((l for l in root.findall("link") if l.get("name") == link), None)
        ok = False
        if el is not None:
            for col in el.findall("collision"):
                cyl = col.find("geometry/cylinder")
                if cyl is not None and abs(float(cyl.get("length")) - L) < 1e-6:
                    ok = True
        check(ok, f"{link}: 圆柱 collision 长度 == {L} m")
    tool = next((l for l in root.findall("link") if l.get("name") == "tool0"), None)
    check(tool is not None and len(tool.findall("visual")) > 0, "tool0 有 visual")
    arm_base = next((l for l in root.findall("link")
                     if l.get("name") == "arm_base_link"), None)
    check(arm_base is not None
          and arm_base.find("collision") is not None
          and arm_base.find("inertial") is not None,
          "arm_base_link 有 collision + inertial")

    # ---- I：无重名 / 无残留 ----
    print("I. 无重名定义 / 无旧命名残留")
    for tag in ("link", "joint"):
        ns = [e.get("name") for e in root.findall(tag)]
        dup = sorted({n for n in ns if ns.count(n) > 1})
        check(not dup, f"{tag} 无重名（重复：{dup}）")
    defs = {}
    for m in root.iter("material"):
        if m.find("color") is not None:
            defs[m.get("name")] = defs.get(m.get("name"), 0) + 1
    dup = {k: v for k, v in defs.items() if v > 1}
    check(not dup, f"material 定义无重名（重复：{dup}）")
    check(text.count('link name="arm_base_link"') == 1, "arm_base_link 只定义一次")
    check("arm_ctrl_" not in text, "无 arm_ctrl_* 旧桩链命名残留")

    print()
    if _fail:
        print(f"❌ {len(_fail)} 项失败")
        return 1
    print("ALL OK: chassis URDF 满足 CanopenChassis + CanopenArm 薄桥契约（CANopen v5）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
