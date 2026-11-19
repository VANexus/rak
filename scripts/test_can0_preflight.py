#!/usr/bin/env python3
"""`scripts/can0_preflight.py` 的桌面测试：不碰真硬件、不依赖 ROS。

跑法::

    python3 -m pytest scripts/test_can0_preflight.py -q

覆盖：单主站判定、polling 判定、state_sign 判定、两套安装树并存告警、
      以及插件 mtime 与 rf2o init_pose 两条（同属"改了没生效"判据）。
sysfs 与 `ip` 输出用假数据注入，进程用假 lister 注入。
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import can0_preflight as pf  # noqa: E402

URDF_SNIPPET = """\
<ros2_control name="rak_chassis" type="system">
  <joint name="front_left_wheel">
      <param name="state_sign">{fl}</param>
  </joint>
  <joint name="front_right_wheel">
      <param name="state_sign">{fr}</param>
  </joint>
  <joint name="rear_left_wheel">
      <param name="state_sign">{rl}</param>
  </joint>
  <joint name="rear_right_wheel">
      <param name="state_sign">{rr}</param>
  </joint>
</ros2_control>
"""

BUS_YAML = """\
options:
  dcf_path: "/tmp"

master:
  node_id: 127

chassis:
  node_id: 1
  polling: {polling}
  period: 10

arm:
  node_id: 2
  polling: true
"""


def write(path, text, mtime=None):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    if mtime is not None:
        os.utime(path, (mtime, mtime))
    return path


def fake_tree(tmp, signs=(1, 1, 1, 1), polling="false", both_trees=False,
              so_mtime=1000.0, urdf_mtime=2000.0):
    """造一棵最小仓库树：bus.yaml + URDF + slam.launch.py + 可选安装树。"""
    root = tmp
    write(root / "src/rak_canopen/config/bus.yaml", BUS_YAML.format(polling=polling))
    urdf = write(root / "src/rak_description/urdf/rak_chassis.urdf.xacro",
                 URDF_SNIPPET.format(fl=signs[0], fr=signs[1], rl=signs[2], rr=signs[3]),
                 mtime=urdf_mtime)
    write(root / "src/rak_bringup/launch/slam.launch.py",
          'p = {"init_pose_from_topic": ""}\n')
    trees = ["install"] + (["install/architecture"] if both_trees else [])
    for tree in trees:
        write(root / tree / "rak_chassis/lib/librak_chassis_plugin.so", "ELF", mtime=so_mtime)
        write(root / tree / "rak_description/share/rak_description/urdf/rak_chassis.urdf.xacro",
              urdf.read_text(encoding="utf-8"), mtime=urdf_mtime)
    return root


# ------------------------------------------------------------------ 单主站判定

def test_singleton_pass_when_one_each():
    lister = lambda pat: [(100, f"/opt/ros/jazzy/bin/{pat} --ros-args")]  # noqa: E731
    rows = pf.check_singletons(lister)
    assert all(r.level != "FAIL" for r in rows), [r.render() for r in rows]
    assert rows[0].name == "进程单例" and rows[0].level == "PASS"


def test_two_masters_fail_and_name_pids():
    def lister(pat):
        if pat == "device_container":
            return [(111, "/opt/ros/jazzy/lib/canopen_core/device_container_node --ros-args"),
                    (222, "/opt/ros/jazzy/lib/canopen_core/device_container_node --ros-args")]
        return []

    rows = pf.check_singletons(lister)
    first = rows[0]
    assert first.name == "进程单例" and first.level == "FAIL"
    dc = [r for r in rows if r.name == "device_container"][0]
    assert dc.level == "FAIL"
    assert "111" in dc.detail and "222" in dc.detail     # PID 列出来了
    assert "别用 kill" in dc.hint and "人工确认" in dc.hint  # 主站不能随手杀


def test_three_i2c_imu_fail():
    """本轮真实故障：i2c_imu 叠到 3 个，频率统计全废。"""
    lister = lambda pat: [(10 + i, "python3 i2c_imu") for i in range(3)] \
        if pat == "i2c_imu" else []
    rows = pf.check_singletons(lister)
    dc = [r for r in rows if r.name == "i2c_imu"][0]
    assert dc.level == "FAIL" and "3 个" in dc.detail


def test_ros2_run_wrapper_is_not_counted_as_a_second_instance(monkeypatch):
    """`ros2 run pkg exe` 会 fork 出真正的节点进程，两个命令行都含 exe 名。

    本轮真机实测：只有一个 i2c_imu，却被判成 "2 个（上限 1）"。只数真正的节点。
    """
    def fake_pgrep(cmd, capture_output=True, text=True):
        class R:
            stdout = (
                "500 /usr/bin/python3 /opt/ros/jazzy/bin/ros2 run rak_imu_driver i2c_imu\n"
                "501 /home/xrak/rak/install/architecture/rak_imu_driver/lib/"
                "rak_imu_driver/i2c_imu --ros-args -p topic:=/imu/data_raw\n"
            )
        return R()

    monkeypatch.setattr(pf.subprocess, "run", fake_pgrep)
    found = pf.list_procs("i2c_imu")
    assert [pid for pid, _ in found] == [501], found


def test_query_shell_is_not_counted_as_a_process(monkeypatch):
    """本轮真机实测：3 个 robot_state_publisher 里 1 个是脚本自己的 shell。

    `python3 scripts/can0_preflight.py` 的父 shell 命令行里含被查询的关键词，
    `pgrep -af` 会把它列出来；只接受"以 / 开头的可执行路径"作为节点进程。
    """
    def fake_pgrep(cmd, capture_output=True, text=True):
        class R:
            stdout = (
                "300 /opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher --ros-args\n"
                "301 bash -c cd /home/xrak/rak && grep robot_state_publisher | head\n"
                "302 /usr/bin/python3 /opt/ros/jazzy/bin/ros2 run rak_imu_driver i2c_imu\n"
            )
        return R()

    monkeypatch.setattr(pf.subprocess, "run", fake_pgrep)
    found = pf.list_procs("robot_state_publisher")
    assert [pid for pid, _ in found] == [300], found


def test_missing_device_container_is_warn_not_fail():
    rows = pf.check_singletons(lambda pat: [])
    dc = [r for r in rows if r.name == "device_container"][0]
    assert dc.level == "WARN"


def test_fix_kills_only_the_excess(capsys):
    """--fix 保留每组最早 PID；可杀的那类（i2c_imu）才真杀。"""
    killed = []

    def lister(pat):
        if pat == "i2c_imu":
            return [(111, "python3 i2c_imu"), (222, "python3 i2c_imu"),
                    (333, "python3 i2c_imu")]
        return []

    rows = pf.check_singletons(lister)
    assert pf.fix_singletons(rows, lister, killer=killed.append) == 2   # 绝不真杀
    out = capsys.readouterr().out
    assert killed == [222, 333], killed
    assert "将 kill 2 个进程" in out and "pid     222" in out


def test_veto_never_kills_nmt_master():
    """主站是硬白名单：双主站必须人工判断，工具不替人挑。"""
    veto = pf.kill_veto("device_container",
                        "/opt/ros/jazzy/lib/canopen_core/device_container_node --ros-args")
    assert veto and "主站" in veto
    assert pf.kill_veto("i2c_imu", "python3 /x/i2c_imu") == ""


def test_veto_never_kills_ros2_launch_parent():
    cmd = ("/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch rak_bringup "
           "chassis_bringup.launch.py allow_motion:=true")
    veto = pf.kill_veto("controller_manager", cmd)
    assert veto and "launch" in veto
    # 真正的 controller_manager 子进程（不经 ros2 launch）才允许杀
    assert pf.kill_veto("controller_manager",
                        "/opt/ros/jazzy/lib/controller_manager/ros2_control_node") == ""


def test_fix_never_kills_protected_even_when_excess(capsys):
    """3 个 device_container + 2 个 ros2 launch 里多出来的：全都不能杀。"""
    procs = {
        "device_container": [(100, "device_container_node"),
                             (101, "device_container_node"),
                             (102, "device_container_node")],
        "controller_manager": [(200, "ros2 launch a.launch.py"),
                               (201, "ros2 launch b.launch.py")],
        "i2c_imu": [(300, "python3 i2c_imu"), (301, "python3 i2c_imu")],
    }
    killed = []
    lister = lambda pat: procs.get(pat, [])          # noqa: E731
    rows = pf.check_singletons(lister)
    n = pf.fix_singletons(rows, lister, killer=killed.append)
    assert killed == [301], killed                    # 只有那个多余的 i2c_imu
    assert n == 1
    out = capsys.readouterr().out
    assert "跳过 3 个受保护进程" in out
    assert "NMT 主站" in out and "ros2 launch 父进程" in out


def test_fix_noop_when_clean(capsys):
    rows = pf.check_singletons(lambda pat: [(1, "one")])
    assert pf.fix_singletons(rows, lambda pat: [(1, "one")], killer=lambda p: None) == 0
    assert "没有可清理的进程" in capsys.readouterr().out


# ------------------------------------------------------------------ polling 判定

def test_polling_false_passes(tmp_path):
    root = fake_tree(tmp_path, polling="false")
    row = pf.check_polling(str(root / "src/rak_canopen/config/bus.yaml"))
    assert row.level == "PASS", row.render()


def test_polling_true_fails(tmp_path):
    """chassis.polling: true 会用 SDO 轮询覆盖 TPDO 真值（0x2020 恒定不变）。"""
    root = fake_tree(tmp_path, polling="true")
    row = pf.check_polling(str(root / "src/rak_canopen/config/bus.yaml"))
    assert row.level == "FAIL"
    assert "0x2020" in row.hint


def test_polling_key_absent_is_warn(tmp_path):
    bus = tmp_path / "bus.yaml"
    bus.write_text("chassis:\n  node_id: 1\n", encoding="utf-8")
    assert pf.check_polling(str(bus)).level == "WARN"


def test_polling_ignores_arm_section_and_comments(tmp_path):
    """arm 段仍是 true，不能把底盘判成 FAIL（真实 bus.yaml 就是这样）。"""
    root = fake_tree(tmp_path, polling="false")
    row = pf.check_polling(str(root / "src/rak_canopen/config/bus.yaml"))
    assert row.level == "PASS"


# ------------------------------------------------------------------ state_sign 判定

def test_state_sign_all_one_passes(tmp_path):
    root = fake_tree(tmp_path)
    row = pf.check_state_sign(str(root / "src/rak_description/urdf/rak_chassis.urdf.xacro"))
    assert row.level == "PASS" and "共 4 个" in row.detail


def test_state_sign_minus_one_fails(tmp_path):
    """旧值 -1 会把固件已乘过 encoder_sign 的计数再翻一次。"""
    root = fake_tree(tmp_path, signs=(1, -1, 1, 1))
    row = pf.check_state_sign(str(root / "src/rak_description/urdf/rak_chassis.urdf.xacro"))
    assert row.level == "FAIL"
    assert "非 1 的 1 个" in row.detail
    assert "重编" in row.hint


def test_state_sign_fewer_than_four_warns(tmp_path):
    urdf = tmp_path / "rak_chassis.urdf.xacro"
    urdf.write_text(URDF_SNIPPET.format(fl=1, fr=1, rl=1, rr=1).split("rear_right")[0]
                    + "</ros2_control>\n", encoding="utf-8")
    row = pf.check_state_sign(str(urdf))
    assert row.level == "WARN" and "期望 4 个" in row.detail


# ------------------------------------------------------------------ 两套安装树并存

def test_two_install_trees_warn(tmp_path):
    root = fake_tree(tmp_path, both_trees=True)
    rows = pf.check_install_trees(str(root), env={"AMENT_PREFIX_PATH": ""})
    tree_row = [r for r in rows if r.name == "安装树"][0]
    assert tree_row.level == "WARN"
    assert "并存" in tree_row.detail
    assert "静默喂旧 URDF" in tree_row.hint


def test_single_install_tree_passes(tmp_path):
    root = fake_tree(tmp_path, both_trees=False)
    rows = pf.check_install_trees(str(root), env={"AMENT_PREFIX_PATH": ""})
    assert [r for r in rows if r.name == "安装树"][0].level == "PASS"


def test_resolve_share_follows_prefix_order(tmp_path):
    """AMENT_PREFIX_PATH 顺序决定解析到哪份 URDF —— 并存时的真正风险点。"""
    root = fake_tree(tmp_path, both_trees=True)
    old = root / "install/rak_description/share/rak_description/urdf/rak_chassis.urdf.xacro"
    new = (root / "install/architecture/rak_description/share/rak_description/urdf"
           / "rak_chassis.urdf.xacro")
    env = {"AMENT_PREFIX_PATH": f"{root}/install:{root}/install/architecture"}
    assert pf.resolve_share(str(root), "rak_description",
                            "urdf/rak_chassis.urdf.xacro", env) == str(old)
    env_rev = {"AMENT_PREFIX_PATH": f"{root}/install/architecture:{root}/install"}
    assert pf.resolve_share(str(root), "rak_description",
                            "urdf/rak_chassis.urdf.xacro", env_rev) == str(new)


# ------------------------------------------------------------------ 插件 mtime / rf2o

def test_stale_plugin_fails(tmp_path):
    """改了 URDF 没重编：旧 .so 继续按老 state_sign 翻转。"""
    root = fake_tree(tmp_path, so_mtime=1000.0, urdf_mtime=2000.0)
    row = pf.check_plugin_mtime(str(root), env={"AMENT_PREFIX_PATH": ""})
    assert row.level == "FAIL"
    # FAIL 行必须给出两棵树的 colcon 命令与复验方法，操作员照着敲就行
    assert "--packages-select rak_chassis" in row.hint
    assert "install-base install/architecture" in row.hint
    assert "复验" in row.hint


def test_fresh_plugin_passes(tmp_path):
    root = fake_tree(tmp_path, so_mtime=3000.0, urdf_mtime=2000.0)
    row = pf.check_plugin_mtime(str(root), env={"AMENT_PREFIX_PATH": ""})
    assert row.level == "PASS", row.render()


def test_rf2o_empty_passes(tmp_path):
    launch = tmp_path / "slam.launch.py"
    launch.write_text('{"init_pose_from_topic": ""}\n', encoding="utf-8")
    assert pf.check_rf2o_init(str(launch)).level == "PASS"


def test_rf2o_nonempty_fails(tmp_path):
    launch = tmp_path / "slam.launch.py"
    launch.write_text('{"init_pose_from_topic": "/base_pose_ground_truth"}\n',
                      encoding="utf-8")
    row = pf.check_rf2o_init(str(launch))
    assert row.level == "FAIL" and "一帧不处理" in row.hint


# ------------------------------------------------------------------ 链路 + 退出码

IP_DUMP_OK = """\
2: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT
    link/can  promiscuity 0
    can state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 100
      bitrate 1000000 sample-point 0.800
"""


def test_link_pass_with_frames(monkeypatch, tmp_path):
    monkeypatch.setattr(pf, "link_dump", lambda iface: IP_DUMP_OK)
    monkeypatch.setattr(pf, "count_frames", lambda iface, window: 42)
    monkeypatch.setattr(pf, "read_stat", lambda iface, name, sysfs=pf.SYSFS: 0)
    rows = pf.check_link("can0", 0.1)
    assert all(r.level != "FAIL" for r in rows)
    assert any("被动收帧 42" in r.detail for r in rows)


def test_link_zero_frames_with_tx_errors_fails(monkeypatch):
    """零入帧 + TX 错误增长 = 我发帧无人 ACK（板子没上线）。"""
    monkeypatch.setattr(pf, "link_dump", lambda iface: IP_DUMP_OK)
    monkeypatch.setattr(pf, "count_frames", lambda iface, window: 0)
    seq = {"tx_errors": iter([0, 5]), "rx_packets": iter([0, 0])}
    monkeypatch.setattr(pf, "read_stat",
                        lambda iface, name, sysfs=pf.SYSFS: next(
                            seq.get(name, iter([0, 0]))))
    rows = pf.check_link("can0", 0.1)
    assert [r for r in rows if r.name == "板子在总线"][0].level == "FAIL"
    assert any("无人 ACK" in r.detail for r in rows)


def test_link_missing_interface_fails(monkeypatch):
    monkeypatch.setattr(pf, "link_dump", lambda iface: "")
    rows = pf.check_link("can0", 0.1)
    assert rows[0].level == "FAIL" and "can0-up.service" in rows[0].hint


def test_main_link_only_exit_zero(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(pf, "link_dump", lambda iface: IP_DUMP_OK)
    monkeypatch.setattr(pf, "count_frames", lambda iface, window: 7)
    monkeypatch.setattr(pf, "read_stat", lambda iface, name, sysfs=pf.SYSFS: 0)
    assert pf.main(["--link-only", "--window", "0.1"]) == 0


def test_main_exit_one_on_static_fail(monkeypatch, tmp_path, capsys):
    root = fake_tree(tmp_path, polling="true")
    monkeypatch.setattr(pf, "link_dump", lambda iface: IP_DUMP_OK)
    monkeypatch.setattr(pf, "count_frames", lambda iface, window: 7)
    monkeypatch.setattr(pf, "read_stat", lambda iface, name, sysfs=pf.SYSFS: 0)
    monkeypatch.setattr(pf, "check_singletons", lambda lister=None: [
        pf.Row("进程单例", "PASS", "ok")])
    assert pf.main(["--window", "0.1", "--root", str(root)]) == 1
    out = capsys.readouterr().out
    assert "bus.yaml polling" in out and "退出码 1" in out


def test_main_exit_two_on_bad_args():
    assert pf.main(["--window", "0"]) == 2


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))