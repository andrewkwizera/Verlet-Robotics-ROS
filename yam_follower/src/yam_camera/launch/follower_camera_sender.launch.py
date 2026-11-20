"""Capture and send the D405 and D455f streams from the follower NUC."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def gstreamer_env():
    """Keep tracing/debug logging out of the GStreamer subprocesses."""
    blocked = {'GST_TRACERS', 'GST_DEBUG', 'GST_DEBUG_FILE'}
    return {key: value for key, value in os.environ.items()
            if key not in blocked}


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


def sender_process(name, enabled, device, leader_port, gateway_port):
    """Send one encoded RTP stream to the leader and local WebRTC gateway."""
    caps = [
        'video/x-raw,format=YUY2,width=', LaunchConfiguration('width'),
        ',height=', LaunchConfiguration('height'),
        ',framerate=', LaunchConfiguration('fps'), '/1',
    ]

    return ExecuteProcess(
        name=name,
        condition=IfCondition(LaunchConfiguration(enabled)),
        output='screen',
        env=gstreamer_env(),
        respawn=True,
        respawn_delay=2.0,
        respawn_max_retries=5,
        cmd=[
            'gst-launch-1.0', '-e',
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
            '!', 'tee', 'name=rtp_tee',

            # Keep the existing native stream independent from WebRTC.
            'rtp_tee.', '!', 'queue',
            'max-size-buffers=16', 'max-size-bytes=0', 'max-size-time=0',
            'leaky=downstream',
            '!', 'udpsink',
            ['host=', LaunchConfiguration('leader_ip')],
            ['port=', LaunchConfiguration(leader_port)],
            'sync=false', 'async=false',

            # The follower-side gateway receives this local RTP copy.
            'rtp_tee.', '!', 'queue',
            'max-size-buffers=16', 'max-size-bytes=0', 'max-size-time=0',
            'leaky=downstream',
            '!', 'udpsink',
            ['host=', LaunchConfiguration('gateway_host')],
            ['port=', LaunchConfiguration(gateway_port)],
            'sync=false', 'async=false',
        ],
    )


def generate_launch_description():
    settings = load_settings()
    cameras = settings['cameras']
    video = settings['video']
    network = settings['network']

    # Force the new 60 FPS defaults even when an old cameras.local.yaml is present.
    # Launch arguments can still explicitly override these values.
    arguments = [
        DeclareLaunchArgument('enable_d405', default_value='true'),
        DeclareLaunchArgument('enable_d455f', default_value='true'),
        DeclareLaunchArgument('d405_device', default_value=cameras['d405']['device']),
        DeclareLaunchArgument('d455f_device', default_value=cameras['d455f']['device']),
        DeclareLaunchArgument('d405_port', default_value=str(cameras['d405']['port'])),
        DeclareLaunchArgument('d455f_port', default_value=str(cameras['d455f']['port'])),
        DeclareLaunchArgument(
            'd405_gateway_port',
            default_value=str(cameras['d405']['gateway_port'])),
        DeclareLaunchArgument(
            'd455f_gateway_port',
            default_value=str(cameras['d455f']['gateway_port'])),
        DeclareLaunchArgument('leader_ip', default_value=network['leader_ip']),
        DeclareLaunchArgument(
            'gateway_host', default_value=network['gateway_host']),
        DeclareLaunchArgument('width', default_value='848'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='60'),
        DeclareLaunchArgument('bitrate_kbps', default_value=str(video['bitrate_kbps'])),
        DeclareLaunchArgument(
            'keyframe_interval', default_value=str(video['keyframe_interval'])),
        DeclareLaunchArgument(
            'rtp_payload_type', default_value=str(network['rtp_payload_type'])),
        DeclareLaunchArgument('mtu', default_value=str(network['mtu'])),
    ]

    processes = [
        sender_process(
            'd405_camera_sender', 'enable_d405', 'd405_device',
            'd405_port', 'd405_gateway_port'),
        sender_process(
            'd455f_camera_sender', 'enable_d455f', 'd455f_device',
            'd455f_port', 'd455f_gateway_port'),
    ]
    return LaunchDescription(arguments + processes)
