"""Independent YAM leader simulation.

Runs the existing yam_leader executable (sine-wave joint source) and publishes
the real YAM description so the leader arm can be inspected in RViz.

No CAN device, no controller_manager and no physics are involved: the leader's
role in the real stack is to publish /leader/joint_states, which yam_leader
already emulates.
"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Origin (xyz, rpy) of joint7/joint8 in yam_description/urdf/yam_joints.xacro,
# i.e. the finger transform at q = 0.
FINGER_TRANSFORMS = [
    ("tip_left",
     ("0.0445119", "-0.0239931", "0.0566829"),
     ("-3.141592653589783", "0", "1.570796326794895")),
    ("tip_right",
     ("-0.0445119", "0.0239931", "0.0566829"),
     ("-3.141592653589784", "0", "1.5707963267949003")),
]


def generate_launch_description():
    sim_dir = get_package_share_directory("yam_sim_display")

    xacro_file = os.path.join(sim_dir, "urdf", "yam_sim.urdf.xacro")
    rviz_file = os.path.join(sim_dir, "rviz", "leader_sim.rviz")

    robot_description = subprocess.check_output(
        ["xacro", xacro_file], text=True)

    use_rviz = LaunchConfiguration("use_rviz")

    # yam_leader publishes /yam/leader/joint_states; remap to the live name.
    leader_source = Node(
        package="yam_teleop_cpp",
        executable="yam_leader",
        namespace="leader",
        name="yam_leader",
        output="screen",
        remappings=[("/yam/leader/joint_states", "/leader/joint_states")],
    )

    # Namespaced description -> /leader/robot_description.
    # frame_prefix uses the 'namespace/' form so frames are leader/base,
    # leader/link1, ... This is required because RViz2's RobotModel "TF Prefix"
    # resolves frames as tf_prefix + "/" + link_name.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="leader",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "frame_prefix": "leader/",
        }],
        remappings=[("joint_states", "/leader/joint_states")],
    )

    # joint7/joint8 (gripper fingers) are not part of the 6-joint mock hardware,
    # so no joint state ever arrives for them and robot_state_publisher emits no
    # TF for tip_left/tip_right. Publish them as static transforms at q=0 so the
    # RobotModel display has a complete, error-free frame tree.
    finger_transforms = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"leader_{finger}_offset",
            output="screen",
            arguments=[
                "--x", xyz[0], "--y", xyz[1], "--z", xyz[2],
                "--roll", rpy[0], "--pitch", rpy[1], "--yaw", rpy[2],
                "--frame-id", "leader/gripper",
                "--child-frame-id", f"leader/{finger}",
            ],
        )
        for finger, xyz, rpy in FINGER_TRANSFORMS
    ]

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2 with the leader simulation view",
        ),
        leader_source,
        robot_state_publisher,
        *finger_transforms,
        rviz,
    ])
