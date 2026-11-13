# YAM teleoperation across two NUCs (ROS 2 Jazzy)

These archives split the **working single-NUC source** into role-specific ROS 2 launch files:

* **Leader NUC** (the existing NUC): `ros2 launch yam_bringup yam_leader_nuc.launch.py`
* **Follower NUC** (the new NUC): `ros2 launch yam_bringup yam_follower_nuc.launch.py`

The namespaced topics are identical to the working single-NUC setup:
`/leader/joint_states` (published on leader NUC), `/follower/joint_states`
(published on follower NUC), and
`/follower/yam_position_controller/commands` (published by bridge on follower NUC).

Both new launch files are included in **both archives** so you can maintain **one Git repository**.
Only one robot is opened by each launch. Each NUC's local USB-CAN adapter defaults to `can0`.
Keep the original `yam_live_teleop.launch.py` for the older one-NUC setup.

## Build

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep can-utils \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-kdl-parser ros-jazzy-orocos-kdl \
  ros-jazzy-rmw-fastrtps-cpp python3-yaml \
  gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly gstreamer1.0-libav
source /opt/ros/jazzy/setup.bash
cd ~/yam_ros2_ws
colcon build --packages-select yam_description yam_ros2_control yam_teleop_cpp yam_camera yam_bringup --symlink-install
source install/setup.bash
```

## Network configuration

Both machines must use the same `ROS_DOMAIN_ID`, the same `RMW_IMPLEMENTATION`,
and discovery beyond localhost. On **both** NUCs, in every terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/yam_ros2_ws/install/setup.bash
source ~/yam_ros2_ws/scripts/nuc_network_env.sh
```

Use wired Ethernet on the same reachable network; ensure UDP DDS discovery/data
is permitted between hosts. Confirm leader joint states are visible from follower
with `ros2 topic echo /leader/joint_states --once` before commissioning motion.

## CAN on each local NUC

```bash
ip -br link show can0
sudo ip link set can0 down  # only if interface is already UP; otherwise skip
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

If the USB-CAN adapter enumerates as `can1`, use `can_interface:=can1` on
that NUC's launch. The two host interface names are independent.

## Launch

Read-only for networking/integration:

On existing NUC:
`ros2 launch yam_bringup yam_leader_nuc.launch.py`

On new NUC:
`ros2 launch yam_bringup yam_follower_nuc.launch.py`

Camera streaming is optional and disabled by default. To run the robot and
camera subsystem together, start the leader receiver first:

```bash
# Leader NUC
./scripts/start_leader.sh enable_cameras:=true

# Follower NUC
./scripts/start_follower.sh enable_cameras:=true
```

The D405 uses UDP port 5000 and the D455f uses UDP port 5002. See
`src/yam_camera/README.md` for camera-only testing and the ignored local
configuration that contains the real camera paths and leader IP address.

For live motor activation, specify `allow_actuation:=true`,
`calibration_verified:=true`, and `emergency_stop_verified:=true` on **each**
launch **only after verifying** physical calibration, gravity torque direction,
support and independent emergency stop on both machines. Only follower launch
accepts `auto_track:=true` and `follower_speed:=0.25`.

When tracking is disabled, follower bridge holds the last target, which is
not an independent safety-rated stop. If network drops, the bridge detects a
stale leader stream (~250 ms configured) and disarms tracking; **the follower
motors may remain enabled in their hold mode**. The 8000 ms motor firmware
CAN timeout is not a safety-rated emergency stop.

## Integration caveats

* No network transport/protocol changes to the bridge: ROS 2 DDS transports
  `/leader/joint_states` across the Ethernet connection.
* The bridge tracks at 50 Hz and uses a 250 ms state freshness bound.
  Do **not** raise error thresholds just to tolerate network latency.
* The 6-joint URDF motor zero/direction assumptions must still be commissioned
  for the hardware on the new NUC. Flag `true` is only an assertion.
* This source was validated for launch syntax and internal topic wiring in an
  offline environment, **not physically on the newly-provisioned NUC**.
