
import os
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # --------------------------------------------------
    # Locate ROS 2 packages
    # --------------------------------------------------

    description_dir = get_package_share_directory(
        "yam_description"
    )

    bringup_dir = get_package_share_directory(
        "yam_bringup"
    )

    # --------------------------------------------------
    # Locate actual YAM robot description
    # --------------------------------------------------

    xacro_file = os.path.join(
        description_dir,
        "urdf",
        "yam.urdf.xacro"
    )

    controllers_file = os.path.join(
        bringup_dir,
        "config",
        "yam_physical_controllers.yaml"
    )

    # --------------------------------------------------
    # Generate URDF from Xacro
    # --------------------------------------------------

    robot_description = subprocess.check_output(
        [
            "xacro",
            xacro_file,
            "role:=follower",
            "can_interface:=can1",
        ],
        text=True
    )

    # --------------------------------------------------
    # Publish YAM robot description
    # --------------------------------------------------

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description
            }
        ]
    )

    # --------------------------------------------------
    # Start physical ros2_control Controller Manager
    # --------------------------------------------------

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            controllers_file
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager
    ])