"""Single-entry lightweight local costmap: rf2o + one controller_server.

This launch is deliberately small: it reuses /scan and /camera/depth/points and
starts only one controller_server/local_costmap. The lifecycle manager keeps the
node active and prevents the duplicate-instance failure seen when an old and a
new controller_server coexist under the same name.

Assumptions:
  - /scan and /camera/depth/points are already running (lidar + depth driver).
  - Exactly one body-tree publisher exists. Set publish_robot_tf:=true only when
    chassis_bringup is not running its own robot_state_publisher.
  - No global costmap, planner, slam_toolbox or bt_navigator is started here.

    ros2 launch slam_prep/launch/local_costmap_2d.launch.py publish_robot_tf:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OVERLAY = os.path.join(HERE, "..", "config", "local_costmap_2d.yaml")


def generate_launch_description():
    base_params = os.path.join(
        get_package_share_directory("rak_bringup"), "config", "nav2_params.yaml"
    )
    overlay = LaunchConfiguration("overlay")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_robot_tf = LaunchConfiguration("publish_robot_tf")
    start_rf2o = LaunchConfiguration("start_rf2o")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "overlay", default_value=DEFAULT_OVERLAY,
                description="2D local-costmap overlay YAML.",
            ),
            DeclareLaunchArgument(
                "publish_robot_tf", default_value="false",
                description="true only when no chassis_bringup robot_state_publisher is running.",
            ),
            DeclareLaunchArgument(
                "start_rf2o", default_value="true",
                description="start the standalone laser odometry source.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(HERE, "body_tf.launch.py")),
                condition=IfCondition(publish_robot_tf),
            ),
            Node(
                package="rf2o_laser_odometry",
                executable="rf2o_laser_odometry_node",
                name="rf2o_laser_odometry",
                output="screen",
                parameters=[
                    {
                        "laser_scan_topic": "/scan",
                        "odom_topic": "/odom",
                        "base_frame_id": "base_footprint",
                        "odom_frame_id": "odom",
                        "publish_tf": True,
                        "freq": 10.0,
                        "init_pose_from_topic": "",
                        "use_sim_time": use_sim_time,
                    }
                ],
                condition=IfCondition(start_rf2o),
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[
                    base_params,
                    overlay,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_local",
                output="screen",
                parameters=[
                    {
                        "autostart": True,
                        "bond_timeout": 0.0,
                        "node_names": ["controller_server"],
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        ]
    )
