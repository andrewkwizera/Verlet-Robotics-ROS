"""Launch the YAM operator dashboard on its own.

This launch intentionally starts **only** the dashboard process.  It does not
launch the simulation, RViz or any controller, and it does not modify the
existing simulation launch files.

Terminal 1: ros2 launch yam_sim_display full_sim.launch.py
Terminal 2: ros2 launch yam_sim_display dashboard.launch.py
Browser  : http://localhost:8080
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("yam_sim_display")
    config_file = os.path.join(share, "config", "dashboard.yaml")

    parameters = [config_file]

    # Empty defaults mean "use the value from dashboard.yaml".  A non-empty
    # launch argument takes precedence over the YAML without editing the file.
    overrides = {}

    port = LaunchConfiguration("port").perform(context).strip()
    if port:
        overrides["port"] = int(port)

    bind_address = LaunchConfiguration("bind_address").perform(context).strip()
    if bind_address:
        overrides["bind_address"] = bind_address

    refresh_ms = LaunchConfiguration("refresh_ms").perform(context).strip()
    if refresh_ms:
        overrides["refresh_ms"] = int(refresh_ms)

    if overrides:
        parameters.append(overrides)

    dashboard = Node(
        package="yam_sim_display",
        executable="dashboard_node",
        name="yam_dashboard",
        output="screen",
        emulate_tty=True,
        parameters=parameters,
    )

    return [dashboard]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "port",
            default_value="",
            description="HTTP port (default: value in config/dashboard.yaml, 8080)",
        ),
        DeclareLaunchArgument(
            "bind_address",
            default_value="",
            description=(
                "HTTP bind address (default: value in config/dashboard.yaml, "
                "127.0.0.1). Use 0.0.0.0 only on a trusted network."
            ),
        ),
        DeclareLaunchArgument(
            "refresh_ms",
            default_value="",
            description="Browser poll interval in ms (default: 100)",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
