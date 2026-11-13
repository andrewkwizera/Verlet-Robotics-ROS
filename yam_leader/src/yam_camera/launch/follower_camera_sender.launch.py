"""Capture and send the D405 and D455f streams from the follower NUC."""

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


def sender_process(name, enabled, device, port):
    """Build one low-latency GStreamer sender process."""
    caps = [
        'video/x-raw,format=YUY2,width=', LaunchConfiguration('width'),
        ',height=', LaunchConfiguration('height'),
        ',framerate=', LaunchConfiguration('fps'), '/1',
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
            'v4l2src', ['device=', LaunchConfiguration(device)],
            'do-timestamp=true',
            '!', caps,
            '!', 'queue', 'max-size-buffers=1', 'leaky=downstream',
            '!', 'videoconvert',
            '!', 'video/x-raw,format=I420',
            '!', 'x264enc', 'tune=zerolatency', 'speed-preset=ultrafast',
            ['bitrate=', LaunchConfiguration('bitrate_kbps')],
            ['key-int-max=', LaunchConfiguration('keyframe_interval')],
            'bframes=0',
            '!', 'video/x-h264,profile=baseline',
            '!', 'h264parse',
            '!', 'rtph264pay',
            ['pt=', LaunchConfiguration('rtp_payload_type')],
            'config-interval=-1',
            ['mtu=', LaunchConfiguration('mtu')],
            '!', 'udpsink',
            ['host=', LaunchConfiguration('leader_ip')],
            ['port=', LaunchConfiguration(port)],
            'sync=false', 'async=false',
        ],
    )


def generate_launch_description():
    settings = load_settings()
    cameras = settings['cameras']
    video = settings['video']
    network = settings['network']

    arguments = [
        DeclareLaunchArgument('enable_d405', default_value='true'),
        DeclareLaunchArgument('enable_d455f', default_value='true'),
        DeclareLaunchArgument('d405_device', default_value=cameras['d405']['device']),
        DeclareLaunchArgument('d455f_device', default_value=cameras['d455f']['device']),
        DeclareLaunchArgument('d405_port', default_value=str(cameras['d405']['port'])),
        DeclareLaunchArgument('d455f_port', default_value=str(cameras['d455f']['port'])),
        DeclareLaunchArgument('leader_ip', default_value=network['leader_ip']),
        DeclareLaunchArgument('width', default_value=str(video['width'])),
        DeclareLaunchArgument('height', default_value=str(video['height'])),
        DeclareLaunchArgument('fps', default_value=str(video['fps'])),
        DeclareLaunchArgument('bitrate_kbps', default_value=str(video['bitrate_kbps'])),
        DeclareLaunchArgument(
            'keyframe_interval', default_value=str(video['keyframe_interval'])),
        DeclareLaunchArgument(
            'rtp_payload_type', default_value=str(network['rtp_payload_type'])),
        DeclareLaunchArgument('mtu', default_value=str(network['mtu'])),
    ]

    processes = [
        sender_process('d405_camera_sender', 'enable_d405', 'd405_device', 'd405_port'),
        sender_process('d455f_camera_sender', 'enable_d455f', 'd455f_device', 'd455f_port'),
    ]
    return LaunchDescription(arguments + processes)
