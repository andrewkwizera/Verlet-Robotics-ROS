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

    # Publishes the URDF on /robot_description and the robot TF tree.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description}
        ],
    )

    # Runs the control loop and loads our hardware plugin.
    # On Jazzy it reads the URDF from ~/robot_description,
    # so we connect that to /robot_description.
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            str(config_file)
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Starts the controller that publishes /joint_states.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Starts the controller that listens on
    # /yam_position_controller/commands (sent by yam_follower).
    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["yam_position_controller"],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        position_controller_spawner,
    ])
