import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    pkg_name = "icp_registration"
    share_dir = get_package_share_directory(pkg_name)
    livox_ros_driver2_share = get_package_share_directory("livox_ros_driver2")
    config_file = os.path.join(share_dir, "config", "icp.yaml")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        livox_ros_driver2_share, "launch", "msg_MID360_launch.py"
                    )
                )
            ),
            Node(
                package=pkg_name,
                executable="icp_registration_node",
                name="icp_node",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="fake_odom_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "odom", "body"],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--ros-args", "--log-level", "info"],
            ),
        ]
    )
