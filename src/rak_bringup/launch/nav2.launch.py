"""SLAM + omnidirectional Nav2 with optional local motion-test adapters.

Velocity path: MPPI / behavior -> velocity smoother -> collision monitor ->
the selected final-Twist adapter on /cmd_vel. ``none`` leaves the endpoint to
an outer bringup layer, which can compose a hardware driver without reversing
the package dependency direction:

  - sim        cmd_vel_sim      (sim-only /base/telemetry mirror)
  - stub       cmd_vel_stub     (Twist relay -> /cmd_vel_raw)
  - none       no adapter in this launch

抗打滑接线（firmware-capability-spec §7.2，P1-1）：
  L1  slip_watch：残差检测（默认随本 launch 启动，生命周期拥有者 = 本 launch，
      managed 自启；system_manifest.yaml 里 /slip_watch 条目必须保持 enabled:false，
      否则出现第二个生命周期拥有者——移交 system_manager 时须先摘下面的自启）。
  L2  odom_cov_guard：/mecanum_drive_controller/odometry --原样转发(时间戳不动)-->
      /odom_wheel（EKF odom1 消费）；slip_watch 的 /slip_watch/slip_suspected
      为 true 期间把 pose(x,y)/twist(vx,vy) 协方差 ×cov_factor(默认10)，恢复后
      斜坡回落。信号缺失按无打滑 fail-open，诊断报 missing。
  L3  默认关闭（slip_gate:=false）。slip_gate:=true 才换线：
      collision_monitor --/cmd_vel_pre_slip--> slip_watch gate(平移×0.5,恢复斜坡)
      --/cmd_vel--> 后端适配器/真底盘。打开条件：真打滑阈值标定完成后
      （slip_sigma 基线工具回填 sigma_baseline 再置 sigma_baseline>0）。
      关闭时链路与改造前完全一致（collision_monitor 直接发 /cmd_vel）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from rak_orchestration.launch_helpers import managed

#: The servers `navigate_to_pose_w_replanning_and_recovery.xml` actually resolves.
#: `nav2_bringup`'s navigation_launch.py always starts ten; the four dropped here were
#: measured at ~19% of a core on an otherwise idle stack, doing nothing this robot asks
#: for -- `smoother_server` (the default BT never calls it), `route_server` and
#: `docking_server` (no route or dock use case), `waypoint_follower` (the task executor
#: drives NavigateToPose, not NavigateThroughPoses).
_NAV2_SERVERS = (
    ("nav2_controller", "controller_server", "controller_server", True),
    ("nav2_planner", "planner_server", "planner_server", False),
    ("nav2_behaviors", "behavior_server", "behavior_server", True),
    ("nav2_bt_navigator", "bt_navigator", "bt_navigator", False),
    # [AI] 2026-10-14: smoother 输入必须 remap 到 cmd_vel_nav（与上游
    # navigation_launch.py 一致）。不 remap 时它订阅的是 collision_monitor 的
    # 输出 /cmd_vel、自己的 /cmd_vel_smoothed 又喂回 collision_monitor——自环，
    # 而 MPPI 的 /cmd_vel_nav 无人消费。速度链端到端验证之前从未暴露。
    ("nav2_velocity_smoother", "velocity_smoother", "velocity_smoother", True),
    ("nav2_collision_monitor", "collision_monitor", "collision_monitor", False),
)


def _navigation_servers(params_file, use_sim_time, camera_mode=False, planner_lattice="",
                        slip_gate=False):
    """Start the lifecycle servers and their manager directly, minus the dead weight.

    ``camera_mode`` drops the lidar observation sources: with no /scan there is nothing
    for collision_monitor to watch, and a source that never arrives would hold the
    command path at its timeout. Road-mask slowdown is v1.1 (`vision_nav_v1.md` §4.1).
    """
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    nodes = []
    for package, executable, name, remap_cmd_vel in _NAV2_SERVERS:
        if name == "collision_monitor" and camera_mode:
            continue
        base = list(params_file) if isinstance(params_file, (list, tuple)) else [params_file]
        parameters = base + [{"use_sim_time": use_sim_time}]
        # 4D 档：omni 状态格文件由 launch 解析（参数文件里不写死 distro 路径）。
        if name == "planner_server" and planner_lattice:
            parameters.append({"GridBased": {"lattice_filepath": planner_lattice}})
        # L3 gate（slip_gate:=true 才换线）：collision_monitor 的输出从 /cmd_vel
        # 让位给 slip_watch，链路变成 /cmd_vel_pre_slip → slip_watch gate → /cmd_vel。
        # 其余 server 的 cmd_vel 一律不动（它们的 cmd_vel 在链路上游，归 remap_cmd_vel 管）。
        extra_remappings = []
        if name == "collision_monitor" and slip_gate:
            extra_remappings = [("cmd_vel", "cmd_vel_pre_slip")]
        nodes.append(
            Node(
                package=package,
                executable=executable,
                name=name,
                output="screen",
                parameters=parameters,
                remappings=remappings + ([("cmd_vel", "cmd_vel_nav")] if remap_cmd_vel else [])
                + extra_remappings,
            )
        )
    nodes.append(
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"autostart": True},
                {"use_sim_time": use_sim_time},
                {"node_names": [name for _, _, name, _ in _NAV2_SERVERS
                                if not (camera_mode and name == "collision_monitor")]},
            ],
        )
    )
    return nodes


DESCRIPTION_URDF = os.path.join(
    get_package_share_directory("rak_description"), "urdf", "rak_chassis.urdf.xacro"
)


def _compose(context, nav_share, nav2_params):
    """Resolve `sensors` before building nodes: lidar runs slam.launch.py, camera does not."""
    sensors = LaunchConfiguration("sensors").perform(context)
    publish_robot_tf = LaunchConfiguration("publish_robot_tf").perform(context)
    lidar_z = LaunchConfiguration("lidar_z").perform(context)
    map_yaml = LaunchConfiguration("map_yaml").perform(context)
    odom_source = LaunchConfiguration("odom_source").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    backend = LaunchConfiguration("motion_backend")
    camera_mode = sensors == "camera"
    # 4D 融合档：EKF 换成多输入配置（轮式 + rf2o + VIO + IMU），默认沿用 ekf.yaml。
    ekf_params = LaunchConfiguration("ekf_params").perform(context).strip()
    slam_extra = {"ekf_params": ekf_params} if ekf_params else {}

    entities = []
    if camera_mode:
        # The lidar branch gets the body tree from slam.launch.py. Without this the camera
        # branch has no base_footprint -> base_link and Nav2's costmaps never activate
        # ("transform from base_link to odom did not become available") -- measured.
        if publish_robot_tf == "true":
            entities.append(Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": ParameterValue(
                    Command(["xacro ", DESCRIPTION_URDF, " lidar_z:=", lidar_z]),
                    value_type=str)}],
            ))
        if odom_source == "rf2o":
            entities.append(LogInfo(msg=(
                "sensors:=camera has no /scan, so rf2o cannot publish odometry. "
                "Pass odom_source:=ekf once the EKF owns odom->base_footprint."
            )))
        if map_yaml:
            entities.append(Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[{"yaml_filename": map_yaml}, {"use_sim_time": use_sim_time}],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ))
            entities.append(Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output="screen",
                parameters=[{"autostart": True}, {"node_names": ["map_server"]}],
            ))
        else:
            entities.append(LogInfo(msg=(
                "sensors:=camera without map_yaml: no static map, the global costmap "
                "stays empty. Pass map_yaml:=<path> once the map is built."
            )))
    else:
        entities.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_share, "launch", "slam.launch.py")),
            launch_arguments={
                "lidar_z": LaunchConfiguration("lidar_z"),
                "start_lidar": LaunchConfiguration("start_lidar"),
                "odom_source": LaunchConfiguration("odom_source"),
                "slip_cov_guard": LaunchConfiguration("slip_cov_guard"),
                "publish_robot_tf": LaunchConfiguration("publish_robot_tf"),
            }.items() | slam_extra.items(),
        ))

    entities.append(Node(
        package="rak_motion_stub",
        executable="cmd_vel_sim",
        name="cmd_vel_sim",
        output="screen",
        parameters=[{
            "cmd_vel_topic": "/cmd_vel",
            "telemetry_topic": "/base/telemetry",
        }],
        condition=IfCondition(EqualsSubstitution(backend, "sim")),
    ))
    entities.append(Node(
        package="rak_motion_stub",
        executable="cmd_vel_stub",
        name="cmd_vel_stub",
        output="screen",
        parameters=[{
            "cmd_vel_topic": "/cmd_vel",
            "raw_topic": "/cmd_vel_raw",
            "log_every_n": 5,
        }],
        condition=IfCondition(EqualsSubstitution(backend, "stub")),
    ))
    params_files = [nav2_params]
    overlay = LaunchConfiguration("params_overlay").perform(context).strip()
    if overlay:
        params_files.append(overlay)
    slip_gate = LaunchConfiguration("slip_gate").perform(context) == "true"
    # ---- 抗打滑链（§7.2）：L1 检测 + L2 协方差中继常驻；L3 gate 仅 slip_gate:=true 换线。
    # 生命周期拥有者 = 本 launch（managed autostart=True 自启，与 supervised 无关）：
    # system_manifest.yaml 的 /slip_watch 条目保持 enabled:false，勿造第二个拥有者。
    if LaunchConfiguration("slip_watch").perform(context) == "true":
        slip_remappings = (
            [("cmd_vel_in", "cmd_vel_pre_slip"), ("cmd_vel_gated", "cmd_vel")]
            if slip_gate else [])
        entities.extend(managed(
            package="rak_metrics",
            executable="slip_watch",
            name="slip_watch",
            output="screen",
            autostart=True,
            parameters=[{"gate_enabled": slip_gate}],
            remappings=slip_remappings,
        ))
    # odom_cov_guard 不在这里起：它由 slam.launch.py 的 ekf 档拥有（单一生命周期
    # 拥有者，ADR-0020 §3）。本文件只把 slip_cov_guard 透传下去，双起会得到两个
    # 同名节点，后者的 /odom_wheel 覆盖前者、打滑膨胀被随机绕过。
    entities.extend(_navigation_servers(
        params_files, use_sim_time, camera_mode,
        LaunchConfiguration("lattice_filepath").perform(context).strip(),
        slip_gate=slip_gate,
    ))
    return entities


def generate_launch_description():
    nav_share = get_package_share_directory("rak_bringup")
    nav2_params = os.path.join(nav_share, "config", "nav2_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "motion_backend",
            default_value="sim",
            choices=["sim", "stub", "none"],
            description="Final /cmd_vel adapter; none leaves it to an outer bringup.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        # [AI] 实测高度（2026-10-14：激光扫描面距地 0.18m，减 base 高 0.0275）。
        # 旧默认 0.10 是占位值；URDF 默认已同步，这里保持一致。
        DeclareLaunchArgument("lidar_z", default_value="0.1525"),
        DeclareLaunchArgument(
            "start_lidar", default_value="true",
            description="start the lidar inside slam.launch.py; false when sensors already did",
        ),
        DeclareLaunchArgument("odom_source", default_value="rf2o", choices=["rf2o", "ekf", "none"]),
        # [AI] 抗打滑链开关（§7.2 / P1-1）：slip_watch/odom_cov_guard 常驻，
        # slip_gate 默认 false——真打滑阈值标定完成后才允许置 true（接线图见文件头注释）。
        DeclareLaunchArgument(
            "slip_watch", default_value="true", choices=["true", "false"],
            description="L1 残差检测 + slip_suspected 信号源（本 launch 拥有其生命周期）",
        ),
        DeclareLaunchArgument(
            "slip_cov_guard", default_value="true", choices=["true", "false"],
            description="L2 轮式 odom 协方差中继（透传给 slam.launch.py 的 ekf 档；"
                        "/mecanum_drive_controller/odometry -> /odom_wheel）",
        ),
        DeclareLaunchArgument(
            "slip_gate", default_value="false", choices=["true", "false"],
            description="L3 /cmd_vel 限幅门换线；默认关闭，阈值标定后才能开",
        ),
        # [AI] 与 chassis_bringup 同跑时必须传 false——否则两个
        # robot_state_publisher 都发 base_footprint->base_link（ADR-0015 一帧一发布者）。
        DeclareLaunchArgument(
            "publish_robot_tf",
            default_value="true",
            description="start robot_state_publisher for the body tree; false if chassis_bringup already does",
        ),
        # [AI] 2026-11-15：v1.0 视觉寻路运行时无雷达。camera 档不启动 slam.launch.py，
        # 改用静态地图（离线建好一次），并将 collision_monitor 的雷达观察源清空。
        # 详见 docs/design/vision_nav_v1.md §4.1。
        DeclareLaunchArgument(
            "sensors",
            default_value="lidar",
            choices=["lidar", "camera"],
            description="lidar: slam_toolbox + rf2o (current). camera: static map, no runtime lidar.",
        ),
        # [AI] 2026-11-15（ADR-0026 / vision_nav_v2.md）：4D 融合档的两个口子。
        # ekf_params 换 EKF 配置（多输入融合），params_overlay 追加 Nav2 参数文件
        # （按节点名合并，后者覆盖），这样 v1 的 nav2_params.yaml 保持不动。
        DeclareLaunchArgument(
            "ekf_params",
            default_value="",
            description="Path to a robot_localization params file; empty reuses slam.launch.py's ekf.yaml.",
        ),
        DeclareLaunchArgument(
            "lattice_filepath",
            default_value="",
            description="Omni state-lattice primitives for SmacPlannerLattice; empty keeps the plugin default.",
        ),
        DeclareLaunchArgument(
            "params_overlay",
            default_value="",
            description="Extra Nav2 params file merged after nav2_params.yaml (e.g. config/nav2_4d.yaml).",
        ),
        DeclareLaunchArgument(
            "map_yaml",
            default_value="",
            description="Static map for sensors:=camera (map_server). Empty means no map.",
        ),
        OpaqueFunction(function=_compose, kwargs={"nav_share": nav_share, "nav2_params": nav2_params}),
    ])
