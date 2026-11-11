
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    leader = Node(
        package="yam_teleop_cpp",
        executable="yam_leader",
        name="yam_leader",
        output="screen"
    )

    follower = Node(
        package="yam_teleop_cpp",
        executable="yam_follower",
        name="yam_follower",
        output="screen"
    )

    return LaunchDescription([
        leader,
        follower
    ])