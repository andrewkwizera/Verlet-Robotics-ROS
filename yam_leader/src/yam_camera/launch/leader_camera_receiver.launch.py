"""Receive and display the D405 and D455f streams on the leader NUC."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def load_settings():
    """Load public defaults, then apply optional machine-local settings."""
    package_dir = get_package_share_directory('yam_camera')
    config_dir = os.path.join(package_dir, 'config')
    example_path = os.path.join(config_dir, 'cameras.example.yaml')
    local_path = os.path.join(config_dir, 'cameras.local.yaml')

    with open(example_path, encoding='utf-8') as config_file:
        settings = yaml.safe_load(config_file)

    if os.path.exists(local_path):
        with open(local_path, encoding='utf-8') as config_file:
            local_settings = yaml.safe_load(config_file) or {}
        for section, values in local_settings.items():
            if isinstance(values, dict) and isinstance(settings.get(section), dict):
                for key, value in values.items():
                    if isinstance(value, dict) and isinstance(
                            settings[section].get(key), dict):
                        settings[section][key].update(value)
                    else:
                        settings[section][key] = value
            else:
                settings[section] = values

    return settings


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
