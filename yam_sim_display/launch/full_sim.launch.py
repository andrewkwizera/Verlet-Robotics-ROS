"""Combined YAM simulation: leader + follower in one RViz view.

Includes leader_sim and follower_sim with their own RViz disabled, then starts
a single RViz instance showing both robots. The two robots are placed side by
side with static transforms so they do not overlap in the view.

Topic wiring is unchanged from the individual launches, so exactly one
/leader/joint_states publisher and one RViz instance exist.

Args:
  use_rviz    start the combined RViz2 view (default true)
  use_bridge  use the production yam_live_bridge instead of yam_follower
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_dir = get_package_share_directory("yam_sim_display")

    launch_dir = os.path.join(sim_dir, "launch")
    rviz_file = os.path.join(sim_dir, "rviz", "full_sim.rviz")

    use_rviz = LaunchConfiguration("use_rviz")
    use_bridge = LaunchConfiguration("use_bridge")

    # The included launches are scoped: passing use_rviz:="false" to them would
    # otherwise overwrite this launch's own use_rviz configuration (launch
    # configurations are shared with included launch descriptions), which
    # silently prevented the combined RViz from ever starting.
    included_sims = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "leader_sim.launch.py")),
                launch_arguments={"use_rviz": "false"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "follower_sim.launch.py")),
                launch_arguments={
                    "use_rviz": "false",
                    "fake_leader": "false",
                    "use_bridge": use_bridge,
                }.items(),
            ),
        ],
        scoped=True,
    )

    # Side-by-side placement. Each robot publishes its own root frame
    # (leader/base, follower/base), so a single static parent per robot keeps
    # the TF tree valid.
    leader_offset = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="leader_world_offset",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0.7", "--z", "0",
            "--yaw", "0", "--pitch", "0", "--roll", "0",
            "--frame-id", "world", "--child-frame-id", "leader/base",
        ],
    )

    follower_offset = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="follower_world_offset",
        output="screen",
        arguments=[
            "--x", "0", "--y", "-0.7", "--z", "0",
            "--yaw", "0", "--pitch", "0", "--roll", "0",
            "--frame-id", "world", "--child-frame-id", "follower/base",
        ],
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
            description="Start the combined RViz2 view",
        ),
        DeclareLaunchArgument(
            "use_bridge",
            default_value="false",
            description="Use yam_live_bridge instead of yam_follower",
        ),
        included_sims,
        leader_offset,
        follower_offset,
        rviz,
    ])
