"""Full nav2 chain, layered on top of slam.launch.py.

    ros2 launch rak_navigation nav2.launch.py

Pipeline: lidar -> rf2o -> slam_toolbox -> nav2 navigation -> /cmd_vel
                                                         -> cmd_vel_stub -> /cmd_vel_raw (for MCU)

SLAM mode: slam_toolbox provides /map + map->odom TF, so no AMCL/map_server.
The 2D N10P lidar is the only sensor. cmd_vel_stub forwards the unicycle
Twist verbatim; 4-wheel kinematics live on the MCU.

Lifecycle ordering matters: slam_toolbox must be active before nav2 costmaps
start, otherwise the global costmap StaticLayer gets nothing on /map.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("rak_navigation")
    nav2_share = get_package_share_directory("nav2_bringup")
    nav2_params = os.path.join(share, "config", "nav2_params.yaml")

    return LaunchDescription([
        # Reuse the whole SLAM chain (driver + filter + static_tf + rf2o + slam_toolbox).
        # It already wires base_link -> laser_link, /scan, /odom, /map and the
        # slam_toolbox lifecycle dance.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "slam.launch.py")),
        ),

        # Test relay only; this is not a hardware or emergency-stop interface.
        # Existing relay behavior is preserved during the package migration.
        #   ros2 service call /cmd_vel_stub/enable std_srvs/srv/SetBool "{data: false}"
        Node(
            package="rak_motion_stub",
            executable="cmd_vel_stub",
            name="cmd_vel_stub",
            output="screen",
            parameters=[{
                "cmd_vel_topic": "/cmd_vel",
                "raw_topic": "/cmd_vel_raw",
                "log_every_n": 5,    # 20 Hz controller * 5 = ~4 Hz log; readable, not noisy
            }],
        ),

        # nav2 navigation stack: planner / controller / bt / behavior / smoother
        # / collision_monitor / waypoint_follower / etc. Lifecycle manager
        # autostarts them. This launch reads nav2_params.yaml above.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "params_file": nav2_params,
                "use_sim_time": "False",
                "autostart": "True",
            }.items(),
        ),
    ])
