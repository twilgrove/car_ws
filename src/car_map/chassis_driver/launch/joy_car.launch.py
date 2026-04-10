import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("chassis_driver")
    joy_config = os.path.join(pkg_share, "config", "joy_params.yaml")
    chassis_config = os.path.join(pkg_share, "config", "chassis_params.yaml")

    use_joy = LaunchConfiguration("use_joy")
    declare_use_joy_cmd = DeclareLaunchArgument(
        "use_joy",
        default_value="true",
        description="是否启用遥控器控制底盘",
    )

    port = LaunchConfiguration("port")
    declare_port_cmd = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyUSB0",
        description="底盘串口设备路径",
    )

    log_level = LaunchConfiguration("log_level")
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="INFO",
        description="日志级别",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="是否启用仿真时间（启用后不启动底盘节点）",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[
            {
                "deadzone": 0.05,
                "autorepeat_rate": 100.0,
            }
        ],
        condition=IfCondition(use_joy),
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy",
        output="screen",
        parameters=[joy_config],
        condition=IfCondition(use_joy),
    )

    chassis_node = Node(
        package="chassis_driver",
        executable="chassis_node",
        name="chassis_node",
        output="screen",
        parameters=[
            chassis_config,
            {"port": port},
        ],
        arguments=["--ros-args", "--log-level", log_level],
        condition=UnlessCondition(use_sim_time),
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_joy_cmd)
    ld.add_action(declare_port_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(joy_node)
    ld.add_action(teleop_node)
    ld.add_action(chassis_node)

    return ld
