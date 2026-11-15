"""4D fusion navigation (vision_nav_v2.md, ADR-0027) — one command, all sensors.

    ros2 launch rak_bringup 4d_nav.launch.py                 # lidar+depth+IMU+EKF+VIO, sim motion
    ros2 launch rak_bringup 4d_nav.launch.py vio:=none        # no visual odometry
    ros2 launch rak_bringup 4d_nav.launch.py map_yaml:=/abs/map.yaml sensors_mode:=map
    ros2 launch rak_bringup 4d_nav.launch.py motion_backend:=none   # outer bringup owns /cmd_vel

What it composes (three existing entries, no new stacking rules):
  sensors.launch.py   depth camera (+ /camera/depth/points) + IMU + body-less sensors;
                      lidar is left to slam.launch.py so it is started exactly once.
  vio.launch.py       rtabmap_odom -> /odometry/vio (VIO's vision half).
  nav2.launch.py      lidar + rf2o + slam_toolbox + robot_localization EKF + Nav2,
                      with ekf_params:=ekf_4d.yaml and params_overlay:=nav2_4d.yaml.

Ownership unchanged: EKF owns odom->base_footprint, slam_toolbox owns map->odom,
robot_state_publisher owns the body tree, `allow_motion` still gates the actuators.
Launching this moves nothing.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("rak_bringup")
    # 上游自带的 omni 状态格：5 cm 分辨率 / 0.5 m 转弯半径，正好覆盖 4 m 滚动窗。
    # 路径在这里解析，参数文件（nav2_4d.yaml）只声明插件。
    lattice = os.path.join(
        get_package_share_directory("nav2_smac_planner"),
        "sample_primitives", "5cm_resolution", "0.5m_turning_radius", "omni", "output.json",
    )

    def _include(name, args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", name)),
            launch_arguments=args.items(),
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera", default_value="gemini335l", choices=["gemini335l", "astra"],
            description="Depth-camera backend (depth cloud source for the 4D layer)."),
        DeclareLaunchArgument(
            "vio", default_value="rgbd", choices=["rgbd", "stereo", "none"],
            description="rtabmap_odom flavour feeding /odometry/vio."),
        DeclareLaunchArgument(
            "motion_backend", default_value="sim", choices=["sim", "stub", "none"],
            description="Final /cmd_vel adapter (same meaning as nav2.launch.py)."),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument(
            "map_yaml", default_value="",
            description="Static map for the global layer; empty = SLAM fills it."),
        DeclareLaunchArgument("lidar_z", default_value="0.1525"),

        # 1) sensors we need here; lidar comes from slam.launch.py, mic/TTS not needed.
        _include("sensors.launch.py", {
            "lidar": "false",
            "camera": LaunchConfiguration("camera"),
            "camera_on": "true",
            "imu": "true",
            "mic": "false",
            "chassis": "false",
            "slam": "false",
            "publish_robot_tf": "false",
            "rviz": LaunchConfiguration("rviz"),
        }),
        # 2) VIO input (EKF odom2). source:=none makes this a no-op.
        _include("vio.launch.py", {
            "source": LaunchConfiguration("vio"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }),
        # 3) SLAM + EKF(4 inputs) + Nav2 with the 4D costmap overlay.
        _include("nav2.launch.py", {
            "sensors": "lidar",
            "odom_source": "ekf",
            "ekf_params": os.path.join(share, "config", "ekf_4d.yaml"),
            "params_overlay": os.path.join(share, "config", "nav2_4d.yaml"),
            "lattice_filepath": lattice,
            "motion_backend": LaunchConfiguration("motion_backend"),
            "map_yaml": LaunchConfiguration("map_yaml"),
            "lidar_z": LaunchConfiguration("lidar_z"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "publish_robot_tf": "true",
        }),
    ])
