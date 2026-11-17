"""Independent YAM follower simulation.

Runs the mock ros2_control stack (no CAN), forwards leader joint states to the
follower position controller and publishes the real YAM description.

Topics match the live two-NUC structure:
  in   /leader/joint_states
  out  /follower/joint_states
       /follower/yam_position_controller/commands

Args:
  use_rviz     start RViz2 with the follower view (default true)
  fake_leader  also start the local yam_leader sine source so this launch can
               be demonstrated on its own (default false; full_sim leaves it
               false to avoid duplicate /leader/joint_states publishers)
  use_bridge   use the production yam_live_bridge instead of yam_follower
"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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
    controllers_file = os.path.join(
        sim_dir, "config", "follower_controllers.yaml")
    rviz_file = os.path.join(sim_dir, "rviz", "follower_sim.rviz")

    robot_description = subprocess.check_output(
        ["xacro", xacro_file], text=True)

    use_rviz = LaunchConfiguration("use_rviz")
    fake_leader = LaunchConfiguration("fake_leader")
    use_bridge = LaunchConfiguration("use_bridge")

    # Namespaced description -> /follower/robot_description.
    # frame_prefix uses the 'namespace/' form so frames are follower/base,
    # follower/link1, ... This is required because RViz2's RobotModel "TF Prefix"
    # resolves frames as tf_prefix + "/" + link_name.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="follower",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "frame_prefix": "follower/",
        }],
        remappings=[("joint_states", "/follower/joint_states")],
    )

    # joint7/joint8 (gripper fingers) are not part of the 6-joint mock hardware,
    # so no joint state ever arrives for them and robot_state_publisher emits no
    # TF for tip_left/tip_right. Publish them as static transforms at q=0 so the
    # RobotModel display has a complete, error-free frame tree.
    finger_transforms = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"follower_{finger}_offset",
            output="screen",
            arguments=[
                "--x", xyz[0], "--y", xyz[1], "--z", xyz[2],
                "--roll", rpy[0], "--pitch", rpy[1], "--yaw", rpy[2],
                "--frame-id", "follower/gripper",
                "--child-frame-id", f"follower/{finger}",
            ],
        )
        for finger, xyz, rpy in FINGER_TRANSFORMS
    ]

    # Controller configuration is keyed by /follower/controller_manager, so the
    # node must live in the 'follower' namespace. The description comes from
    # the namespaced robot_state_publisher topic.
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="follower",
        name="controller_manager",
        output="screen",
        parameters=[controllers_file],
        remappings=[("/robot_description", "/follower/robot_description")],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/follower/controller_manager",
        ],
        output="screen",
    )

    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "yam_position_controller",
            "--controller-manager", "/follower/controller_manager",
        ],
        output="screen",
    )

    # yam_follower subscribes /yam/leader/joint_states and publishes
    # /yam_position_controller/commands; remap both to the live names.
    forwarder = Node(
        package="yam_teleop_cpp",
        executable="yam_follower",
        name="yam_follower",
        output="screen",
        condition=UnlessCondition(use_bridge),
        remappings=[
            ("/yam/leader/joint_states", "/leader/joint_states"),
            ("/yam_position_controller/commands",
             "/follower/yam_position_controller/commands"),
        ],
    )

    # Optional: exercise the production bridge (rate limiting, freshness
    # checks, /yam_teleop/status) without any CAN hardware.
    live_bridge = Node(
        package="yam_teleop_cpp",
        executable="yam_live_bridge",
        name="yam_live_bridge",
        output="screen",
        condition=IfCondition(use_bridge),
        parameters=[{
            "actuation_authorized": True,
            "auto_track": True,
            "follower_speed": 0.25,
        }],
    )

    # Optional local motion source for standalone demos.
    local_leader = Node(
        package="yam_teleop_cpp",
        executable="yam_leader",
        namespace="leader",
        name="yam_leader",
        output="screen",
        condition=IfCondition(fake_leader),
        remappings=[("/yam/leader/joint_states", "/leader/joint_states")],
    )

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
            description="Start RViz2 with the follower simulation view",
        ),
        DeclareLaunchArgument(
            "fake_leader",
            default_value="false",
            description="Also run the local yam_leader sine source",
        ),
        DeclareLaunchArgument(
            "use_bridge",
            default_value="false",
            description="Use yam_live_bridge instead of yam_follower",
        ),
        robot_state_publisher,
        *finger_transforms,
        controller_manager,
        joint_state_broadcaster,
        position_controller,
        forwarder,
        live_bridge,
        local_leader,
        rviz,
    ])
