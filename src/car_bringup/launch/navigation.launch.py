import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    bring_up_share = get_package_share_directory("car_bringup")

    nav_config_file = os.path.join(bring_up_share, "config", "nav_params.yaml")
    mid360_config_file = os.path.join(bring_up_share, "config", "MID360_config.json")
    rviz__config_file = os.path.join(bring_up_share, "rviz", "nav.rviz")
    urdf_file = os.path.join(bring_up_share, "urdf", "chassis_rviz2.urdf")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    rviz_use = LaunchConfiguration("rviz")
    declare_rviz_cmd = DeclareLaunchArgument(
        "rviz", default_value="true", description="是否启用RViz进行可视化"
    )

    use_respawn = LaunchConfiguration("use_respawn")
    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn", default_value="false", description="是否启用节点自动重启"
    )

    log_level = LaunchConfiguration("log_level")
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="日志输出级别"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz__config_file, "--ros-args", "--log-level", log_level],
        condition=IfCondition(rviz_use),
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    chassis_node = Node(
        package="chassis_driver",
        executable="chassis_node",
        name="chassis_node",
        output="screen",
        parameters=[nav_config_file],
        arguments=["--ros-args", "--log-level", log_level],
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    livox_driver_node = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[nav_config_file, {"user_config_path": mid360_config_file}],
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
    )

    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fast_lio_node",
        parameters=[nav_config_file],
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
    )

    ground_segmentation_node = Node(
        package="linefit_ground_segmentation_ros",
        executable="ground_segmentation_node",
        name="ground_segmentation_node",
        output="screen",
        parameters=[nav_config_file],
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
    )

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan_node",
        remappings=[("cloud_in", ["/livox/lidar/pointcloud"]), ("scan", ["/scan"])],
        parameters=[nav_config_file],
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
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
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
    )

    icp_node = Node(
        package="icp_registration",
        executable="icp_registration_node",
        name="icp_node",
        output="screen",
        parameters=[nav_config_file],
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
    )

    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="nav2_map_server_node",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[nav_config_file],
        arguments=["--ros-args", "--log-level", log_level],
    )

    nav2_bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="nav2_bt_navigator_node",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[nav_config_file],
        arguments=["--ros-args", "--log-level", log_level],
    )

    nav2_behavior_server_node = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="nav2_behavior_server_node",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[nav_config_file],
        arguments=["--ros-args", "--log-level", log_level],
    )

    nav2_planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="nav2_planner_server_node",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[nav_config_file],
        arguments=["--ros-args", "--log-level", log_level],
    )

    nav2_smoother_server_node = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="nav2_smoother_server_node",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[nav_config_file],
        arguments=["--ros-args", "--log-level", log_level],
    )

    nav2_controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="nav2_controller_server_node",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[nav_config_file],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )

    nav2_velocity_smoother_node = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="nav2_velocity_smoother_node",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[nav_config_file],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
    )

    lifecycle_nodes = [
        "nav2_map_server_node",
        "nav2_bt_navigator_node",
        "nav2_behavior_server_node",
        "nav2_planner_server_node",
        "nav2_smoother_server_node",
        "nav2_controller_server_node",
        "nav2_velocity_smoother_node",
    ]
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"autostart": True},
            {"node_names": lifecycle_nodes},
        ],
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(chassis_node)
    ld.add_action(livox_driver_node)
    ld.add_action(fast_lio_node)
    # ld.add_action(ground_segmentation_node) # 斜放雷达时启用地面分割节点（修改配置文件）
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(icp_node)
    ld.add_action(nav2_map_server)
    ld.add_action(nav2_bt_navigator_node)
    ld.add_action(nav2_behavior_server_node)
    ld.add_action(nav2_planner_server_node)
    ld.add_action(nav2_smoother_server_node)
    ld.add_action(nav2_controller_server_node)
    ld.add_action(nav2_velocity_smoother_node)
    ld.add_action(TimerAction(period=1.0, actions=[nav2_lifecycle_manager]))
    ld.add_action(rviz_node)

    return ld
