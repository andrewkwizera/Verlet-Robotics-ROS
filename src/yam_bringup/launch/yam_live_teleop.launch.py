"""Physical YAM leader (can0) + follower (can1), with explicit actuation gates."""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def assemble(context):
    desc = get_package_share_directory("yam_description")
    bringup = get_package_share_directory("yam_bringup")
    xacro = os.path.join(desc, "urdf", "yam_live.urdf.xacro")
    controllers = os.path.join(bringup, "config", "yam_live_controllers.yaml")
    allow = LaunchConfiguration("allow_actuation").perform(context)
    calibration = LaunchConfiguration("calibration_verified").perform(context)
    estop = LaunchConfiguration("emergency_stop_verified").perform(context)
    auto_track = LaunchConfiguration("auto_track").perform(context)
    follower_speed = LaunchConfiguration("follower_speed").perform(context)
    # Do not silently permit an unlimited motor speed. Value is also checked
    # independently by each hardware plugin and the ROS tracking bridge.
    speed = float(follower_speed)
    if not (0.05 <= speed <= 0.50):
        raise ValueError("follower_speed must be 0.05..0.50 rad/s")
    nodes = []
    for role, channel in [("leader", "can0"), ("follower", "can1")]:
        args = ["xacro", xacro, f"role:={role}", f"can_interface:={channel}",
                f"allow_actuation:={allow}", f"calibration_verified:={calibration}",
                f"emergency_stop_verified:={estop}",
                f"follower_speed:={follower_speed}"]
        for n in range(1, 7):
            args.append(f"offset_{n}:={LaunchConfiguration(f'{role}_offset_{n}').perform(context)}")
        urdf = subprocess.check_output(args, text=True)
        nodes.extend([
            Node(package="robot_state_publisher", executable="robot_state_publisher",
                 namespace=role, name="robot_state_publisher", output="screen",
                 parameters=[{"robot_description": urdf, "frame_prefix": role + "_"}]),
            Node(package="controller_manager", executable="ros2_control_node",
                 namespace=role, name="controller_manager", output="screen",
                 parameters=[controllers],
                 remappings=[("/robot_description", f"/{role}/robot_description")]),
            Node(package="controller_manager", executable="spawner",
                 arguments=["joint_state_broadcaster", "--controller-manager", f"/{role}/controller_manager"],
                 output="screen"),
        ])
    nodes.append(Node(package="controller_manager", executable="spawner",
                      arguments=["yam_position_controller", "--controller-manager", "/follower/controller_manager"],
                      output="screen"))
    # The bridge may auto-arm only after BOTH hardware interfaces publish fresh states,
    # the follower controller subscribes to commands, and the initial poses align.
    authorized = all(v.lower() in ("true", "1") for v in (allow, calibration, estop))
    nodes.append(Node(package="yam_teleop_cpp", executable="yam_live_bridge",
                      name="yam_live_bridge", output="screen",
                      parameters=[{"actuation_authorized": authorized,
                                   "auto_track": auto_track.lower() in ("true", "1"),
                                   "follower_speed": speed}]))
    return nodes


def generate_launch_description():
    args = [
        DeclareLaunchArgument("allow_actuation", default_value="false",
                              description="false = CAN-read-only; true = physical motor activation"),
        DeclareLaunchArgument("calibration_verified", default_value="false",
                              description="Only set true after verifying joint offsets and axis conventions"),
        DeclareLaunchArgument("emergency_stop_verified", default_value="false",
                              description="Only set true after verifying independent emergency-stop behavior"),
        DeclareLaunchArgument("follower_speed", default_value="0.25",
                              description="Follower joint speed limit rad/s (0.05..0.50); increase ONLY after verifying physical tracking and load"),
        DeclareLaunchArgument("auto_track", default_value="false",
                              description="Automatically arm tracking ONCE after 2 seconds if aligned and safe"),
    ]
    for role in ("leader", "follower"):
        for n in range(1, 7):
            args.append(DeclareLaunchArgument(f"{role}_offset_{n}", default_value="0.0"))
    return LaunchDescription(args + [OpaqueFunction(function=assemble)])
