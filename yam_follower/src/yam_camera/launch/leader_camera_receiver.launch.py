"""Receive and display the D405 and D455f streams on the leader NUC."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def load_settings():
    """Load the tested defaults. Launch arguments can override each value."""
    package_dir = get_package_share_directory('yam_camera')
    config_path = os.path.join(package_dir, 'config', 'cameras.yaml')
    with open(config_path, encoding='utf-8') as config_file:
        return yaml.safe_load(config_file)


def receiver_process(name, enabled, port):
    """Build one low-latency RTP/H.264 receiver process."""
    rtp_caps = [
        'application/x-rtp,media=video,encoding-name=H264,payload=',
        LaunchConfiguration('rtp_payload_type'), ',clock-rate=90000',
    ]

    return ExecuteProcess(
        name=name,
        condition=IfCondition(LaunchConfiguration(enabled)),
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        respawn_max_retries=5,
        cmd=[
            'gst-launch-1.0', '-e', '-v',
            'udpsrc', ['port=', LaunchConfiguration(port)],
            'caps=', rtp_caps,
            '!', 'rtpjitterbuffer',
            ['latency=', LaunchConfiguration('jitter_buffer_ms')],
            'drop-on-latency=true',
            '!', 'rtph264depay',
            '!', 'h264parse',
            '!', 'avdec_h264',
            '!', 'queue', 'max-size-buffers=1', 'leaky=downstream',
            '!', 'videoconvert',
            '!', 'autovideosink', 'sync=false', 'async=false',
        ],
    )


def generate_launch_description():
    settings = load_settings()
    cameras = settings['cameras']
    network = settings['network']

    arguments = [
        DeclareLaunchArgument('enable_d405', default_value='true'),
        DeclareLaunchArgument('enable_d455f', default_value='true'),
        DeclareLaunchArgument('d405_port', default_value=str(cameras['d405']['port'])),
        DeclareLaunchArgument('d455f_port', default_value=str(cameras['d455f']['port'])),
        DeclareLaunchArgument(
            'rtp_payload_type', default_value=str(network['rtp_payload_type'])),
        DeclareLaunchArgument(
            'jitter_buffer_ms', default_value=str(network['jitter_buffer_ms'])),
    ]

    processes = [
        receiver_process('d405_camera_receiver', 'enable_d405', 'd405_port'),
        receiver_process('d455f_camera_receiver', 'enable_d455f', 'd455f_port'),
    ]
    return LaunchDescription(arguments + processes)
