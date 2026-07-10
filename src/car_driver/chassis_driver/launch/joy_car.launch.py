import os
import shlex
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _ensure_rtt_ready(context, *args, **kwargs):
    auto_load = LaunchConfiguration("auto_load_rtt").perform(context)
    rpmsg_path = LaunchConfiguration("rpmsg_path").perform(context)
    start_command = LaunchConfiguration("rtt_start_command").perform(context)
    timeout_s = float(LaunchConfiguration("rtt_ready_timeout").perform(context))

    if os.path.exists(rpmsg_path):
        print(f"[launch] RT-Thread endpoint already exists: {rpmsg_path}")
        return []

    if not _as_bool(auto_load):
        print(
            f"[launch] RT-Thread endpoint is missing and auto_load_rtt=false: {rpmsg_path}"
        )
        return []

    cmd = shlex.split(start_command)
    if not cmd:
        raise RuntimeError("rtt_start_command is empty")

    if os.geteuid() != 0 and cmd[0] == "/usr/local/bin/rtt-start.sh":
        cmd = ["sudo", "-n"] + cmd

    print(f"[launch] RT-Thread endpoint missing, loading firmware: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if os.path.exists(rpmsg_path):
            print(f"[launch] RT-Thread endpoint is ready: {rpmsg_path}")
            return []
        time.sleep(0.2)

    raise RuntimeError(
        f"RT-Thread endpoint not ready after {timeout_s:.1f}s: {rpmsg_path}"
    )


def generate_launch_description():
    pkg_share = get_package_share_directory("chassis_driver")
    joy_config = os.path.join(pkg_share, "config", "joy_params.yaml")
    chassis_config = os.path.join(pkg_share, "config", "chassis_params.yaml")

    use_joy = LaunchConfiguration("use_joy")
    declare_use_joy_cmd = DeclareLaunchArgument(
        "use_joy",
        default_value="true",
        description="Enable joystick teleop.",
    )

    rpmsg_path = LaunchConfiguration("rpmsg_path")
    declare_rpmsg_path_cmd = DeclareLaunchArgument(
        "rpmsg_path",
        default_value="/proc/rpmsg_rtt",
        description="RT-Thread RPMsg procfs control endpoint.",
    )

    auto_load_rtt = LaunchConfiguration("auto_load_rtt")
    declare_auto_load_rtt_cmd = DeclareLaunchArgument(
        "auto_load_rtt",
        default_value="false",
        description="Load RT-Thread firmware before starting the chassis node if the RPMsg endpoint is missing.",
    )

    rtt_start_command = LaunchConfiguration("rtt_start_command")
    declare_rtt_start_command_cmd = DeclareLaunchArgument(
        "rtt_start_command",
        default_value="/usr/local/bin/rtt-start.sh",
        description="Command used to load RT-Thread when auto_load_rtt is true.",
    )

    rtt_ready_timeout = LaunchConfiguration("rtt_ready_timeout")
    declare_rtt_ready_timeout_cmd = DeclareLaunchArgument(
        "rtt_ready_timeout",
        default_value="45",
        description="Seconds to wait for the RT-Thread RPMsg endpoint after loading firmware.",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Do not start chassis node when simulation time is enabled.",
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
        parameters=[chassis_config],
        condition=UnlessCondition(use_sim_time),
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_joy_cmd)
    ld.add_action(declare_rpmsg_path_cmd)
    ld.add_action(declare_auto_load_rtt_cmd)
    ld.add_action(declare_rtt_start_command_cmd)
    ld.add_action(declare_rtt_ready_timeout_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(OpaqueFunction(function=_ensure_rtt_ready))
    ld.add_action(joy_node)
    ld.add_action(teleop_node)
    ld.add_action(chassis_node)
    return ld
