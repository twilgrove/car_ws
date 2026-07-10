"""
前后端建图 launch: FAST-LIO2 (前端) + PGO (后端回环) + octomap (2D/3D 占据) + chassis_driver

tf 链:
    map        --[pgo, 回环校正, 动态]--------> odom
    odom       --[fastlio2, body_frame=lidar_link]--> lidar_link
    lidar_link --[URDF, robot_state_publisher]-> base_link
    (map->odom 由 pgo 动态发; body_frame 直接命名 lidar_link, 省掉静态 tf)

节点:
    robot_state_publisher  机器人模型 + URDF tf
    livox_ros_driver2      MID360 驱动 (CustomMsg)
    fastlio2/lio_node      前端 LIO
    pgo/pgo_node           后端: 回环 + map->odom + 存图服务 /pgo/save_maps
    octomap_server         点云投占据图 -> /projected_map (2D) + octomap (3D)
    chassis_driver         joy_car.launch.py (默认参数, 手柄控车)
    rviz2
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("car_bringup")
    chassis_share = get_package_share_directory("chassis_driver")

    urdf_file = os.path.join(bringup_share, "urdf", "chassis_rviz2.urdf")
    mid360_config = os.path.join(bringup_share, "config", "MID360_config.json")
    rviz_config = os.path.join(bringup_share, "rviz", "mapping.rviz")
    lio_config = os.path.join(bringup_share, "config", "lio.yaml")
    pgo_config = os.path.join(bringup_share, "config", "pgo.yaml")
    octomap_config = os.path.join(bringup_share, "config", "octomap.yaml")

    with open(urdf_file, "r") as f:
        robot_description = f.read()

    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="是否启动 RViz"
    )
    use_rviz = LaunchConfiguration("rviz")

    # 1. 机器人模型 + URDF tf
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # 2. Livox MID360
    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[
            {"xfer_format": 1},                               # 1=只发 Livox CustomMsg
            {"multi_topic": 0},                               # 0=所有雷达共用一个话题
            {"data_src": 0},                                  # 0=实时雷达 (非 lvx 回放)
            {"publish_freq": 10.0},                           # 点云发布频率 (Hz)
            {"output_data_type": 0},                          # 输出数据类型 (默认 0)
            {"frame_id": "lidar_link"},                       # 点云/IMU 的 frame_id
            {"lvx_file_path": "/home/livox/livox_test.lvx"},  # 回放文件路径 (实时时不用)
            {"user_config_path": mid360_config},              # MID360 配置 (雷达/主机 IP)
            {"cmdline_input_bd_code": "livox0000000001"},     # 广播码 (用 JSON 配置时忽略)
        ],
    )

    # 3. FAST-LIO2 前端
    lio_node = Node(
        package="fastlio2",
        executable="lio_node",
        name="lio_node",
        namespace="fastlio2",
        output="screen",
        parameters=[{"config_path": lio_config}],
    )

    # 4. PGO 后端 
    pgo_node = Node(
        package="pgo",
        executable="pgo_node",
        name="pgo_node",
        namespace="pgo",
        output="screen",
        parameters=[{"config_path": pgo_config}],
    )

    # 5. octomap
    octomap_node = Node(
        package="octomap_server2",
        executable="octomap_server",
        name="octomap_server",
        output="screen",
        remappings=[("cloud_in", "/fastlio2/body_cloud")],
        parameters=[octomap_config],
    )

    # 6. chassis_driver 
    chassis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(chassis_share, "launch", "joy_car.launch.py")
        )
    )

    # 7. RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_rviz,
        robot_state_publisher,
        livox_driver,
        lio_node,
        pgo_node,
        octomap_node,
        chassis_launch,
        rviz_node,
    ])
