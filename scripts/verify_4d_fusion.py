#!/usr/bin/env python3
"""4D 融合导航的可运行验证（ADR-0027 / docs/design/vision_nav_v2.md §7）。

三个门，全部本机可跑，不需要传感器与底盘：
  G1 配置契约   ekf_4d.yaml / nav2_4d.yaml 自洽（纯文件检查）
  G2 EKF 融合   起 ekf_node，喂轮式+激光+VIO+IMU，要求 /odometry/filtered 出现且
                odom->base_footprint TF 由它发布（ADR-0002 唯一发布者）
  G3 4D 代价层  起 controller_server（v1 参数 + 4d overlay），喂合成 /scan 与
                /camera/depth/points，要求 /local_costmap/costmap 上真的出现障碍格
                —— 证明 STVL 体素层接线正确（不是"起了个节点"）
  G4 规划器     起 planner_server，要求日志出现 "Using motion model: State Lattice"
                且 lattice 文件就是 omni 原语（1.3.13 唯一原生全向路径）
  G5 视觉里程计 起 rtabmap_odom rgbd_odometry，喂合成 RGB-D，要求 /odometry/vio 出消息
                （VIO 的视觉半环真的在跑，而不是只写了个 launch）
  G6 真出一条路 同 G4 的 planner + 全 Free /map，发 ComputePathToPose，要求 SUCCEEDED
                且路径点数 > 1（"能规划"而不是"节点起来了"）

用法：python3 scripts/verify_4d_fusion.py [--skip-runtime]
退出码 0 = 全绿；非 0 = 至少一道门红。
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
CONFIG = REPO / "src" / "rak_bringup" / "config"
EKFS = CONFIG / "ekf_4d.yaml"
NAV2_BASE = CONFIG / "nav2_params.yaml"
NAV2_4D = CONFIG / "nav2_4d.yaml"
DOMAIN = os.environ.get("ROS_DOMAIN_ID", "93")

failures: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""))
    if not ok:
        failures.append(name)


def _yaml(path: Path):
    import yaml
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def gate_config() -> None:
    ekf = _yaml(EKFS)["ekf_filter_node"]["ros__parameters"]
    nav2 = _yaml(NAV2_4D)
    slots = sorted({k.split("_")[0] for k in ekf if k.endswith("_config")})
    check("G1 EKF 四路输入", slots == ["imu0", "odom0", "odom1", "odom2"], str(slots))
    check("G1 每路都有协方差", all(f"{s}_covariance" in ekf for s in slots))
    local = nav2["local_costmap"]["local_costmap"]["ros__parameters"]
    glob = nav2["global_costmap"]["global_costmap"]["ros__parameters"]
    layer = local["voxel_3d_layer"]
    check("G1 局部层是体素层且带时间衰减",
          layer["plugin"] == "rtabmap_costmap_plugins::VoxelLayer" and layer["decay"] > 0,
          f"{layer['plugin']} decay={layer['decay']}")
    check("G1 全局层保留 static_layer", glob["plugins"][0] == "static_layer", str(glob["plugins"]))


def _env() -> dict:
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = DOMAIN
    env.setdefault("ROS_LOCALHOST_ONLY", "1")
    return env


def _lifecycle(node: str, transition: str, timeout: float = 40.0) -> bool:
    """Retry: a lifecycle service is only reachable after the node finished creating."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        proc = subprocess.run(
            ["ros2", "lifecycle", "set", node, transition],
            capture_output=True, text=True, timeout=15.0, env=_env(),
        )
        text = proc.stdout + proc.stderr
        if "successful" in text:
            return True
        if "already" in text.lower() or "no transition" in text.lower():
            return True
        time.sleep(1.0)
    return False


def _spawn(cmd: list[str], log_path: str) -> subprocess.Popen:
    """Own process group: `ros2 run` forks the real node, terminate on the group kills both."""
    log = open(log_path, "w")
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, env=_env(),
                            start_new_session=True)


def _kill(proc: subprocess.Popen) -> None:
    import signal
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


def gate_ekf() -> None:
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    import tf2_ros

    node = _spawn(
        ["ros2", "run", "robot_localization", "ekf_node", "--ros-args",
         "--params-file", str(EKFS), "-r", "__node:=ekf_filter_node"],
        "/tmp/verify_4d_ekf.log",
    )
    rclpy.init()
    helper = rclpy.create_node("verify_4d_ekf_probe")
    wheel_pub = helper.create_publisher(Odometry, "/odometry/wheel", 10)
    laser_pub = helper.create_publisher(Odometry, "/odom", 10)
    vio_pub = helper.create_publisher(Odometry, "/odometry/vio", 10)
    imu_pub = helper.create_publisher(Imu, "/imu/data_raw", 20)
    seen = {"filtered": None}

    def _on_filtered(msg):
        seen["filtered"] = msg

    helper.create_subscription(Odometry, "/odometry/filtered", _on_filtered, 10)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, helper)

    def _odom(pub, x):
        msg = Odometry()
        msg.header.stamp = helper.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"
        msg.pose.pose.position.x = x
        msg.twist.twist.linear.x = 0.1
        pub.publish(msg)

    deadline = time.time() + 25.0
    tf_ok = False
    steps = 0
    while time.time() < deadline:
        steps += 1
        _odom(wheel_pub, 0.01 * steps)
        _odom(laser_pub, 0.01 * steps)
        _odom(vio_pub, 0.01 * steps)
        imu = Imu()
        imu.header.stamp = helper.get_clock().now().to_msg()
        imu.header.frame_id = "imu_link"
        imu.angular_velocity.z = 0.1
        imu_pub.publish(imu)
        rclpy.spin_once(helper, timeout_sec=0.1)
        try:
            if buf.lookup_transform("odom", "base_footprint", rclpy.time.Time()).transform:
                tf_ok = True
        except Exception:
            pass
        if seen["filtered"] is not None and tf_ok:
            break

    check("G2 /odometry/filtered 有输出", seen["filtered"] is not None)
    check("G2 EKF 发布 odom->base_footprint", tf_ok)
    helper.destroy_node()
    rclpy.shutdown()
    _kill(node)


def gate_costmap() -> None:
    """Prove the 4D layer marks: synthetic scan + depth cloud -> /local_costmap/costmap.

    Measured through the CLI (`ros2 topic echo --once --field data`) rather than an
    rclpy subscription: on this box a same-process rclpy OccupancyGrid subscriber saw
    zero frames while `ros2 topic echo` received them (2026-11-15). Costmap data is
    int8, so "marked" is 100 (lethal) vs 0 (free).
    """
    import struct

    node = _spawn(
        ["ros2", "run", "nav2_controller", "controller_server", "--ros-args",
         "--params-file", str(NAV2_BASE), "--params-file", str(NAV2_4D),
         "-p", "use_sim_time:=false"],
        "/tmp/verify_4d_costmap.log",
    )
    # odom->base_link plus the lidar mount: the scan source declares sensor_frame: laser_link.
    tf_odom = _spawn(
        ["ros2", "run", "tf2_ros", "static_transform_publisher", "--frame-id", "odom",
         "--child-frame-id", "base_link"], "/tmp/verify_4d_tf_odom.log")
    tf_laser = _spawn(
        ["ros2", "run", "tf2_ros", "static_transform_publisher", "--frame-id", "base_link",
         "--child-frame-id", "laser_link", "--z", "0.1525"], "/tmp/verify_4d_tf_laser.log")

    # Drivers first, then Nav2 — the real bringup order. Starting the sensors *after*
    # the costmap activated produced zero marks every time on this box (measured
    # 2026-11-15), so the gate runs the order that also matches deployment.
    sensors = _spawn([sys.executable, str(REPO / "scripts" / "fake_4d_sensors.py"),
                      "--seconds", "60", "--range", "1.5"],
                     "/tmp/verify_4d_sensors.log")
    time.sleep(5.0)

    configured = _lifecycle("/controller_server", "configure")
    activated = _lifecycle("/controller_server", "activate")
    check("G3 controller_server 进 active", configured and activated,
          f"configure={configured} activate={activated}")
    time.sleep(8.0)
    for topic in ("/scan", "/camera/depth/points", "/local_costmap/costmap"):
        info = subprocess.run(["ros2", "topic", "info", topic],
                              capture_output=True, text=True, env=_env(), timeout=15)
        counts = [line.strip() for line in info.stdout.splitlines()
                  if "Publisher count" in line or "Subscription count" in line]
        print(f"    {topic}: {counts}")
    dump = "/tmp/verify_4d_costmap_dump.txt"
    with open(dump, "w") as handle:
        subprocess.run(["ros2", "topic", "echo", "--once", "/local_costmap/costmap",
                        "--field", "data"], stdout=handle, stderr=subprocess.DEVNULL,
                       timeout=20, env=_env())
    import re
    numbers = [int(x) for x in re.findall(r"-?\d+", Path(dump).read_text(errors="replace"))]
    numbers = numbers[-6400:] if numbers else []
    marked = sum(1 for v in numbers if v > 0)
    check("G3 4D 体素层把合成障碍标进 /local_costmap/costmap", marked > 0,
          f"marked_cells={marked} max={max(numbers) if numbers else None}")

    for proc in (node, tf_odom, tf_laser, sensors):
        _kill(proc)


def gate_planner() -> None:
    lattice = os.path.join(
        os.environ.get("NAV2_SMAC_SHARE", "/opt/ros/jazzy/share/nav2_smac_planner"),
        "sample_primitives", "5cm_resolution", "0.5m_turning_radius", "omni", "output.json",
    )
    if not os.path.exists(lattice):
        check("G4 omni 状态格存在", False, lattice)
        return
    check("G4 omni 状态格存在", True, lattice)
    log_path = "/tmp/verify_4d_planner.log"
    node = _spawn(
        ["ros2", "run", "nav2_planner", "planner_server", "--ros-args",
         "--params-file", str(NAV2_BASE), "--params-file", str(NAV2_4D),
         "-p", "GridBased.lattice_filepath:=" + lattice],
        log_path,
    )
    ok = _lifecycle("/planner_server", "configure", timeout=40.0)
    _kill(node)
    text = Path(log_path).read_text(encoding="utf-8", errors="replace")
    check("G4 planner_server configure 成功", ok)
    check("G4 用的是 omni 状态格",
          "State Lattice" in text and "omni/output.json" in text,
          "" if "State Lattice" in text else "日志里没有 State Lattice")



def gate_vio() -> None:
    """rtabmap_odom rgbd_odometry -> /odometry/vio, fed by synthetic RGB-D."""
    tf_color = _spawn(
        ["ros2", "run", "tf2_ros", "static_transform_publisher", "--frame-id",
         "base_footprint", "--child-frame-id", "camera_color_optical_frame"],
        "/tmp/verify_4d_tf_color.log")
    tf_depth = _spawn(
        ["ros2", "run", "tf2_ros", "static_transform_publisher", "--frame-id",
         "base_footprint", "--child-frame-id", "camera_depth_optical_frame", "--z", "0.02"],
        "/tmp/verify_4d_tf_depth.log")
    sensors = _spawn([sys.executable, str(REPO / "scripts" / "fake_vio_sensors.py"),
                      "--seconds", "50"], "/tmp/verify_4d_vio_sensors.log")
    node = _spawn(
        ["ros2", "run", "rtabmap_odom", "rgbd_odometry", "--ros-args",
         "--params-file", str(CONFIG / "vio_rtabmap.yaml"),
         "-r", "rgb/image:=/camera/color/image_raw",
         "-r", "rgb/camera_info:=/camera/color/camera_info",
         "-r", "depth/image:=/camera/depth/image_raw",
         "-r", "depth/camera_info:=/camera/depth/camera_info",
         "-r", "odom:=/odometry/vio"], "/tmp/verify_4d_vio.log")
    time.sleep(15.0)
    dump = "/tmp/verify_4d_vio_dump.txt"
    with open(dump, "w") as handle:
        subprocess.run(["ros2", "topic", "echo", "--once", "/odometry/vio"],
                       stdout=handle, stderr=subprocess.DEVNULL, timeout=20, env=_env())
    text = Path(dump).read_text(errors="replace")
    ok = "child_frame_id: base_footprint" in text and "frame_id: odom" in text
    check("G5 rtabmap_odom 发布 /odometry/vio（odom -> base_footprint）", ok,
          "" if ok else text.splitlines()[:3])
    for proc in (node, sensors, tf_color, tf_depth):
        _kill(proc)


def gate_plan_path() -> None:
    """A real path out of SmacPlannerLattice, on an all-free synthetic /map."""
    lattice = os.path.join(
        os.environ.get("NAV2_SMAC_SHARE", "/opt/ros/jazzy/share/nav2_smac_planner"),
        "sample_primitives", "5cm_resolution", "0.5m_turning_radius", "omni", "output.json",
    )
    tf_map = _spawn(
        ["ros2", "run", "tf2_ros", "static_transform_publisher", "--frame-id", "map",
         "--child-frame-id", "odom"], "/tmp/verify_4d_tf_map.log")
    tf_odom = _spawn(
        ["ros2", "run", "tf2_ros", "static_transform_publisher", "--frame-id", "odom",
         "--child-frame-id", "base_link"], "/tmp/verify_4d_tf_odom2.log")
    tf_laser = _spawn(
        ["ros2", "run", "tf2_ros", "static_transform_publisher", "--frame-id", "base_link",
         "--child-frame-id", "laser_link", "--z", "0.1525"], "/tmp/verify_4d_tf_laser2.log")
    sensors = _spawn([sys.executable, str(REPO / "scripts" / "fake_4d_sensors.py"),
                      "--seconds", "70", "--map"], "/tmp/verify_4d_map_sensors.log")
    node = _spawn(
        ["ros2", "run", "nav2_planner", "planner_server", "--ros-args",
         "--params-file", str(NAV2_BASE), "--params-file", str(NAV2_4D),
         "-p", "GridBased.lattice_filepath:=" + lattice], "/tmp/verify_4d_plan.log")
    time.sleep(6.0)
    configured = _lifecycle("/planner_server", "configure")
    activated = _lifecycle("/planner_server", "activate")
    check("G6 planner_server 进 active", configured and activated,
          f"configure={configured} activate={activated}")
    time.sleep(2.0)
    goal = ("{use_start: true, planner_id: 'GridBased', "
            "start: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0}, "
            "orientation: {w: 1.0}}}, "
            "goal: {header: {frame_id: 'map'}, pose: {position: {x: 4.0, y: 1.0}, "
            "orientation: {w: 1.0}}}}")
    dump = "/tmp/verify_4d_plan_dump.txt"
    with open(dump, "w") as handle:
        subprocess.run(["ros2", "action", "send_goal", "/compute_path_to_pose",
                        "nav2_msgs/action/ComputePathToPose", goal],
                       stdout=handle, stderr=subprocess.STDOUT, timeout=40, env=_env())
    text = Path(dump).read_text(errors="replace")
    poses = sum(1 for line in text.splitlines() if line.strip().startswith("x:"))
    ok = "SUCCEEDED" in text and poses > 1
    check("G6 SmacPlannerLattice 真的返回路径", ok, f"poses={poses}")
    for proc in (node, sensors, tf_map, tf_odom, tf_laser):
        _kill(proc)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--skip-runtime", action="store_true",
                        help="only run the offline G1 config gate")
    parser.add_argument("--only",
                        choices=["config", "ekf", "costmap", "planner", "vio", "path"],
                        help="run a single gate")
    args = parser.parse_args()

    only = args.only
    if only in (None, "config"):
        gate_config()
    if not args.skip_runtime:
        if only in (None, "ekf"):
            gate_ekf()
        if only in (None, "costmap"):
            gate_costmap()
        if only in (None, "planner"):
            gate_planner()
        if only in (None, "vio"):
            gate_vio()
        if only in (None, "path"):
            gate_plan_path()

    print()
    if failures:
        print(f"FAILED: {len(failures)} gate(s): {', '.join(failures)}")
        return 1
    print("ALL GATES PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
