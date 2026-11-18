"""Combined text-task executor, SLAM/Nav2 and the selected motion backend.

Wires rak_task_executor (the task lifecycle layer) on top of the full Nav2
chain. motion_backend picks the final /cmd_vel adapter:
  - sim|stub   non-hardware adapters
  - hardware   removed (ADR-0016): the chassis is reached through
               chassis_bringup.launch.py -> mecanum_drive_controller over CAN,
               never through a second /cmd_vel adapter here.

任务层本身在 `task.launch.py`（ADR-0020 之后它需要能被单独组合进 `robot.launch.py`
的 full 档，而不是连带再起一份导航链）。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    navigation_share = get_package_share_directory("rak_bringup")
    launch_dir = os.path.join(navigation_share, "launch")
    return LaunchDescription([
        DeclareLaunchArgument(
            "motion_backend",
            default_value="sim",
            choices=["sim", "stub"],
            description="Final /cmd_vel adapter; the real chassis is reached via "
                        "chassis_bringup.launch.py (CAN), not this argument.",
        ),
        DeclareLaunchArgument("slow_planner", default_value="local",
                              choices=["local", "rai", "harness"]),
        DeclareLaunchArgument("llm_config_path", default_value=""),
        DeclareLaunchArgument("llm_vendor", default_value=""),
        DeclareLaunchArgument("harness", default_value="true",
                              choices=["true", "false"],
                              description="Start harness_node next to the executor "
                                          "(ADR-0010 arbitration seam)."),
        DeclareLaunchArgument("waypoints_file", default_value=""),
        DeclareLaunchArgument("lidar_z", default_value="0.1525"),  # 2026-10-14 实测
        DeclareLaunchArgument("odom_source", default_value="rf2o", choices=["rf2o", "ekf", "none"]),
        # [AI] L3 /cmd_vel 限幅门（§7.2）：透传给 nav2.launch.py。默认关闭，
        # 真打滑阈值标定完成后才允许 slip_gate:=true（接线图见 nav2.launch.py 头注释）。
        DeclareLaunchArgument(
            "slip_gate", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument(
            "supervised", default_value="false",
            description="true = 任务层由 system_manager 驱动（ADR-0020）",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "nav2.launch.py")),
            launch_arguments={
                "motion_backend": LaunchConfiguration("motion_backend"),
                "lidar_z": LaunchConfiguration("lidar_z"),
                "odom_source": LaunchConfiguration("odom_source"),
                "slip_gate": LaunchConfiguration("slip_gate"),
                "use_sim_time": "false",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "task.launch.py")),
            launch_arguments={
                "slow_planner": LaunchConfiguration("slow_planner"),
                "llm_config_path": LaunchConfiguration("llm_config_path"),
                "llm_vendor": LaunchConfiguration("llm_vendor"),
                "harness": LaunchConfiguration("harness"),
                "waypoints_file": LaunchConfiguration("waypoints_file"),
                "supervised": LaunchConfiguration("supervised"),
            }.items(),
        ),
    ])
