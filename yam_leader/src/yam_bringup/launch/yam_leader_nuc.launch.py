"""Run only physical YAM leader on this host; remote follower is another NUC.
Leader's own USB-CAN device defaults to can0. No follower controllers or bridge.
This is an additional role-specific launch, leaving single-host launch unchanged.
"""
import os
import subprocess

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
        'leader_camera_receiver.launch.py',
    )

    args = [
        'xacro', xacro,
        'role:=leader',
        'can_interface:=' + LaunchConfiguration('can_interface').perform(context),
        'allow_actuation:=' + LaunchConfiguration('allow_actuation').perform(context),
        'calibration_verified:=' + LaunchConfiguration('calibration_verified').perform(context),
        'emergency_stop_verified:=' + LaunchConfiguration('emergency_stop_verified').perform(context),
    ]
    for j in range(1, 7):
        args.append('offset_' + str(j) + ':=' +
                    LaunchConfiguration('leader_offset_' + str(j)).perform(context))
    urdf = subprocess.check_output(args, text=True)

    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='leader', name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': urdf, 'frame_prefix': 'leader_'}]),
        Node(package='controller_manager', executable='ros2_control_node',
             namespace='leader', name='controller_manager', output='screen',
             parameters=[controllers],
             remappings=[('/robot_description', '/leader/robot_description')]),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster',
                        '--controller-manager', '/leader/controller_manager'],
             output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            condition=IfCondition(LaunchConfiguration('enable_cameras')),
            launch_arguments={
                'enable_d405': LaunchConfiguration('enable_d405'),
                'enable_d455f': LaunchConfiguration('enable_d455f'),
            }.items(),
        ),
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('can_interface', default_value='can0',
                              description='CAN adapter attached to LEADER on this NUC'),
        DeclareLaunchArgument('allow_actuation', default_value='false',
                              description='Motor commands disabled unless physical verification completed'),
        DeclareLaunchArgument('calibration_verified', default_value='false',
                              description='Affirm verified offsets, axes and torque direction'),
        DeclareLaunchArgument('emergency_stop_verified', default_value='false',
                              description='Affirm verified independent emergency stop'),
        DeclareLaunchArgument('enable_cameras', default_value='false',
                              description='Start both leader camera receiver processes'),
        DeclareLaunchArgument('enable_d405', default_value='true',
                              description='Start the D405 receiver when cameras are enabled'),
        DeclareLaunchArgument('enable_d455f', default_value='true',
                              description='Start the D455f receiver when cameras are enabled'),
    ]
    args.extend(DeclareLaunchArgument('leader_offset_' + str(j), default_value='0.0')
                for j in range(1, 7))
    return LaunchDescription(args + [OpaqueFunction(function=setup)])
