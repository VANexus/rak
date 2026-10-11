"""Read-only planning + kinematic fixture, strictly on ROS domain 77."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    if os.environ.get("ROS_DOMAIN_ID") != "77":
        raise RuntimeError("set ROS_DOMAIN_ID=77 before launching isolated grasp simulation")
    share = get_package_share_directory("rak_moveit_config")
    perception = get_package_share_directory("rak_grasp_perception")
    description = get_package_share_directory("rak_description")
    robot_description = ParameterValue(Command(["xacro ", os.path.join(
        description, "urdf", "six_axis_sim.urdf.xacro")]), value_type=str)
    semantic = Path(share, "config", "six_axis_sim.srdf").read_text()
    return LaunchDescription([
        DeclareLaunchArgument("snapshot"), DeclareLaunchArgument("roi"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(perception, "config", "grasp.rviz")),
        DeclareLaunchArgument("simulation_mount", default_value=os.path.join(perception, "config", "virtual_mount.yaml")),
        GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, "launch", "six_axis_sim.launch.py")),
            launch_arguments={"rviz": "false"}.items())]),
        Node(package="rak_grasp_perception", executable="plan_grasp_sim", output="screen",
             parameters=[{k: LaunchConfiguration(k) for k in ("snapshot", "roi", "simulation_mount")}]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", LaunchConfiguration("rviz_config")],
             parameters=[{"robot_description": robot_description, "robot_description_semantic": semantic}],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
