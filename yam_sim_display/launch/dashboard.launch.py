"""Launch the YAM operator dashboard on its own.

This launch intentionally starts **only** the dashboard process.  It does not
launch the simulation, RViz or any controller, and it does not modify the
existing simulation launch files.

Terminal 1: ros2 launch yam_sim_display full_sim.launch.py
Terminal 2: ros2 launch yam_sim_display dashboard.launch.py
Browser  : http://localhost:8080

Physical WebRTC example:
  ros2 launch yam_sim_display dashboard.launch.py \
    bind_address:=<LEADER_IP> webrtc_enabled:=true
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
    supervision_file = os.path.join(share, "config", "supervision.yaml")

    parameters = [config_file, supervision_file]

    # Empty defaults mean "use the value from dashboard.yaml".  A non-empty
    # launch argument takes precedence over the YAML without editing the file.
    overrides = {}

    port = LaunchConfiguration("port").perform(context).strip()
    if port:
        overrides["port"] = int(port)

    bind_address = LaunchConfiguration("bind_address").perform(context).strip()
    if bind_address:
        overrides["bind_address"] = bind_address

    network_rtt_host = LaunchConfiguration("network_rtt_host").perform(context).strip()
    if network_rtt_host:
        overrides["network_rtt_host"] = network_rtt_host

    webrtc_enabled = LaunchConfiguration("webrtc_enabled").perform(context).strip()
    if webrtc_enabled:
        normalized = webrtc_enabled.lower()
        if normalized not in ("true", "false"):
            raise ValueError("webrtc_enabled must be true or false")
        overrides["webrtc.enabled"] = normalized == "true"

    refresh_ms = LaunchConfiguration("refresh_ms").perform(context).strip()
    if refresh_ms:
        overrides["refresh_ms"] = int(refresh_ms)

    startup_grace = LaunchConfiguration("startup_grace_s").perform(context).strip()
    if startup_grace:
        overrides["supervision.startup_grace_s"] = float(startup_grace)

    ready_hold = LaunchConfiguration("ready_hold_s").perform(context).strip()
    if ready_hold:
        overrides["supervision.ready_hold_s"] = float(ready_hold)

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
        DeclareLaunchArgument(
            "network_rtt_host",
            default_value="",
            description="Optional peer IP for the read-only ICMP RTT display",
        ),
        DeclareLaunchArgument(
            "webrtc_enabled",
            default_value="",
            description="Enable WebRTC signaling (true/false; default from YAML)",
        ),
        DeclareLaunchArgument(
            "startup_grace_s",
            default_value="",
            description=(
                "Readiness startup grace in seconds (default: value in "
                "config/supervision.yaml, 5.0)"
            ),
        ),
        DeclareLaunchArgument(
            "ready_hold_s",
            default_value="",
            description=(
                "How long all readiness prerequisites must hold before Ready "
                "(default: value in config/supervision.yaml, 1.0)"
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
