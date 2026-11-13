"""Run physical YAM follower and live teleop bridge on the FOLLOWER NUC.
Leader joint states are subscribed across DDS (/leader/joint_states).
Follower motor commands never have to cross the network.
"""
import os
import subprocess
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    desc = get_package_share_directory('yam_description')
    bringup = get_package_share_directory('yam_bringup')
    xacro = os.path.join(desc, 'urdf', 'yam_live.urdf.xacro')
    controllers = os.path.join(bringup, 'config', 'yam_live_controllers.yaml')
    camera_launch = os.path.join(
        get_package_share_directory('yam_camera'),
        'launch',
        'follower_camera_sender.launch.py',
    )

    speed_string = LaunchConfiguration('follower_speed').perform(context)
    speed = float(speed_string)
    if not math.isfinite(speed) or not 0.05 <= speed <= 0.50:
        raise ValueError('follower_speed must be finite in 0.05..0.50 rad/s')

    allow = LaunchConfiguration('allow_actuation').perform(context)
    calibrated = LaunchConfiguration('calibration_verified').perform(context)
    estop = LaunchConfiguration('emergency_stop_verified').perform(context)
    auto_track = LaunchConfiguration('auto_track').perform(context)
    authorized = all(v.strip().lower() in ('true','1')
                     for v in (allow, calibrated, estop))

    camera_arguments = {
        'enable_d405': LaunchConfiguration('enable_d405'),
        'enable_d455f': LaunchConfiguration('enable_d455f'),
    }
    leader_ip = LaunchConfiguration('leader_ip').perform(context).strip()
    if leader_ip:
        camera_arguments['leader_ip'] = leader_ip

    args = [
        'xacro', xacro,
        'role:=follower',
        'can_interface:=' + LaunchConfiguration('can_interface').perform(context),
        'allow_actuation:=' + allow,
        'calibration_verified:=' + calibrated,
        'emergency_stop_verified:=' + estop,
        'follower_speed:=' + speed_string,
    ]
    for j in range(1, 7):
        args.append('offset_' + str(j) + ':=' +
                    LaunchConfiguration('follower_offset_' + str(j)).perform(context))
    urdf = subprocess.check_output(args, text=True)

    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='follower', name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': urdf, 'frame_prefix': 'follower_'}]),
        Node(package='controller_manager', executable='ros2_control_node',
             namespace='follower', name='controller_manager', output='screen',
             parameters=[controllers],
             remappings=[('/robot_description', '/follower/robot_description')]),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster',
                        '--controller-manager', '/follower/controller_manager'],
             output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['yam_position_controller',
                        '--controller-manager', '/follower/controller_manager'],
             output='screen'),
        Node(package='yam_teleop_cpp', executable='yam_live_bridge',
             name='yam_live_bridge', output='screen',
             parameters=[{'actuation_authorized': authorized,
                          'auto_track': auto_track.strip().lower() in ('true','1'),
                          'follower_speed': speed}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            condition=IfCondition(LaunchConfiguration('enable_cameras')),
            launch_arguments=camera_arguments.items(),
        ),
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('can_interface', default_value='can0',
                              description='CAN adapter attached to FOLLOWER on this NUC'),
        DeclareLaunchArgument('allow_actuation', default_value='false',
                              description='Motor commands disabled unless physical verification completed'),
        DeclareLaunchArgument('calibration_verified', default_value='false',
                              description='Affirm verified offsets, axes and torque direction'),
        DeclareLaunchArgument('emergency_stop_verified', default_value='false',
                              description='Affirm verified independent emergency stop'),
        DeclareLaunchArgument('auto_track', default_value='false',
                              description='Auto-arm bridge once leader and follower align'),
        DeclareLaunchArgument('follower_speed', default_value='0.25',
                              description='rad/s, requires load-specific physical commissioning'),
        DeclareLaunchArgument('enable_cameras', default_value='false',
                              description='Start both follower camera sender processes'),
        DeclareLaunchArgument('enable_d405', default_value='true',
                              description='Start the D405 sender when cameras are enabled'),
        DeclareLaunchArgument('enable_d455f', default_value='true',
                              description='Start the D455f sender when cameras are enabled'),
        DeclareLaunchArgument(
            'leader_ip', default_value='',
            description='Optional leader IP override; empty uses cameras.local.yaml'),
    ]
    args.extend(DeclareLaunchArgument('follower_offset_' + str(j), default_value='0.0')
                for j in range(1, 7))
    return LaunchDescription(args + [OpaqueFunction(function=setup)])
