from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    bringup_dir = Path(
        get_package_share_directory("yam_bringup")
    )

    urdf_file = bringup_dir / "urdf" / "yam_mock.urdf"

    config_file = (
        bringup_dir / "config" / "yam_controllers.yaml"
    )

    robot_description = urdf_file.read_text()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description}
        ],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            str(config_file)
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
    ])