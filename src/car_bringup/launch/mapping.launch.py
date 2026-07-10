import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    bring_up_share = get_package_share_directory("car_bringup")
    chassis_share = get_package_share_directory("chassis_driver")

    map_config_file = os.path.join(bring_up_share, "config", "mapping_config.yaml")
    mid360_config_file = os.path.join(bring_up_share, "config", "MID360_config.json")
    rviz__config_file = os.path.join(bring_up_share, "rviz", "maping.rviz")
    urdf_file = os.path.join(bring_up_share, "urdf", "chassis_rviz2.urdf")

    rviz_use = LaunchConfiguration("rviz")
    declare_rviz_cmd = DeclareLaunchArgument(
        "rviz", default_value="false", description="是否启用RViz进行可视化"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz__config_file],
        condition=IfCondition(rviz_use),
    )

    livox_driver_node = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[map_config_file, {"user_config_path": mid360_config_file}],
    )

    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fast_lio_node",
        parameters=[map_config_file],
        output="screen",
    )

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan_node",
        remappings=[("cloud_in", ["/livox/lidar/pointcloud"]), ("scan", ["/scan"])],
        parameters=[map_config_file],
    )

    start_async_slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox_async_node",
        parameters=[map_config_file],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(["xacro ", urdf_file]),
            }
        ],
    )

    chassis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(chassis_share, "launch", "joy_car.launch.py")
        )
    )

    ld = LaunchDescription()

    ld.add_action(declare_rviz_cmd)
    ld.add_action(livox_driver_node)
    ld.add_action(fast_lio_node)
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(rviz_node)
    # ld.add_action(chassis_launch)

    return ld
