# Physical YAM leader–follower teleoperation — C++ ROS 2 Jazzy

This patch **modifies the existing Git repository**. It does not replace the repository,
run I2RT at runtime, or operate synthetic leader joint states.

## Architecture

```
physical YAM leader, can0 (6 DM MIT motors)
   -> yam_ros2_control/YAMLiveHardware (KDL gravity compensation)
   -> /leader/joint_states
   -> C++ yam_live_bridge (alignment gate, capped target rate, lost-stream disarm)
   -> /follower/yam_position_controller/commands
   -> ROS 2 JointGroupPositionController
   -> yam_ros2_control/YAMLiveHardware (position hold + KDL gravity feedforward)
   -> physical follower YAM, can1 (6 DM MIT motors)
```

Both robots have independent `ros2_control` controller managers. Their joint state
broadcasters are loaded automatically. The follower position controller is loaded
automatically. The bridge automatically starts in **tracking disabled** mode.

### What changed

* Physical plugin computes gravity torques for **both** arms from the installed YAM
  URDF before enabling any motor. The leader floats using zero MIT position gain,
  factory gravity damping and gravity feedforward. The follower holds its **measured**
  initial joint positions with MIT PD and gravity feedforward.
* MIT feedback and the read-only motor `xout` register are reconciled at startup;
  25-rad position wrapping is handled during operation; encoder disagreement is
  faulted rather than silently accepted.
* Follower tracking is constrained to 0.20 rad/s at the ROS bridge and 0.25 rad/s
  at the motor interface; tracking errors or large separation disarm the bridge.
* Uncommanded follower drift during idle is avoided by **latching** the hold target
  instead of repeatedly commanding the changing measured pose.
* Added `auto_track` launch argument, which tries to arm tracking **once** after
  both state streams are fresh, the follower controller subscribes, and every
  leader/follower joint is aligned to within 0.10 rad. It does **not** automatically
  re-arm after a fault or manual disarm.
* Joint-state topics: `/leader/joint_states`, `/follower/joint_states`.
  Status: `/yam_teleop/status`. Manual tracking service:
  `/yam_teleop/enable_tracking` (`std_srvs/srv/SetBool`).
* Fixed `yam_description/CMakeLists.txt` to stop installing a nonexistent `config`
  directory.

## Integrate with your existing Git workflow

On your development computer, **apply `YAM-ROS2-Integrated-Changes.patch` at the
root of the existing Git checkout**. This patch uses the `src/...` paths already in
your repository. No NUC ZIP download or replacement of `~/yam_ros2_ws` is needed.

```bash
git status --short
git apply --check YAM-ROS2-Integrated-Changes.patch
git apply YAM-ROS2-Integrated-Changes.patch
git diff --check
git add src/yam_ros2_control/src/yam_live_hardware.cpp \
        src/yam_ros2_control/include/yam_ros2_control/yam_live_hardware.hpp \
        src/yam_teleop_cpp/src/yam_live_bridge.cpp \
        src/yam_bringup/launch/yam_live_teleop.launch.py \
        src/yam_description/CMakeLists.txt README_LIVE_TELEOP.md tests/test_can_codec.cpp
git commit -m "Integrate real ROS2 YAM leader follower actuation"
git push
```

Alternatively, unpack `YAM-ROS2-Integrated-Overlay.zip` directly onto your **existing
Git checkout** (it contains only the modified/additional files, rooted at `src/`,
`tests/` and this README), then `git add`, `git commit`, `git push`.

On the NUC (after pushing):

```bash
cd ~/yam_ros2_ws
# Old src backup inside the workspace prevents colcon from discovering unique packages:
if [ -d src.backup.20261104-141941 ]; then
  mv src.backup.20261104-141941 ~/yam_ros2_src_backup_20261104-141941
fi
git pull --ff-only
source /opt/ros/jazzy/setup.bash
colcon build --packages-select yam_description yam_ros2_control yam_teleop_cpp yam_bringup --symlink-install
source install/setup.bash
```

### Safe read-only launch (no motor enable)

```bash
ros2 launch yam_bringup yam_live_teleop.launch.py
```

### Commissioned live operation — one launch, both arms

**The operator must first verify the physical motor-zero positions, the mapping of
URDF joint coordinates to real motors, the gravity torque directions, independent
emergency stop, and mechanical supports/fixtures.** A `true` launch flag is NOT a
calibration or an emergency-stop test. No file in the submitted repository proves
those physical conditions; therefore they cannot be automatically certified.

Once they have been verified in the physical installation, start the full live
controller with **automatic ROS 2 tracking**:

```bash
ros2 launch yam_bringup yam_live_teleop.launch.py \
  allow_actuation:=true \
  calibration_verified:=true \
  emergency_stop_verified:=true \
  auto_track:=true
```

These commands rely on the default **numeric** offsets `0.0` for all twelve
motors, which are consistent with the default factory I2RT software configuration
but **not proof** of physical zero calibration. If your physically verified
motor offsets differ, pass `leader_offset_1:=NUMBER` through
`leader_offset_6:=NUMBER` and likewise `follower_offset_1` through
`follower_offset_6`, or edit the launch argument defaults in your Git checkout.
Never paste angle-bracket placeholders into Bash.

When both arms start in corresponding poses within **0.10 rad at each joint**, the
bridge waits for controller readiness and automatically enters `TRACKING ARMED`
after at least 2 seconds. Move the leader **slowly** and the follower controller
will command the matching pose. If the initial poses are farther apart,
auto-tracking remains off until they are manually aligned; it never commands a
large pose jump to force alignment.

If `auto_track:=false`, the live system may instead be armed manually:

```bash
ros2 service call /yam_teleop/enable_tracking std_srvs/srv/SetBool '{data: true}'
```

Stop tracking while retaining follower position hold:

```bash
ros2 service call /yam_teleop/enable_tracking std_srvs/srv/SetBool '{data: false}'
```

This service is **not an emergency stop**, and `Ctrl+C` does not safely arrest an
unsupported gravity-loaded arm. The CAN firmware's 8000 ms timeout is not an
adequate personnel safety response. Loss of electrical power or motor disable
can make an unsupported arm drop. Keep every arm supported and the workspace
clear until the real actuator startup and stop responses have been commissioned.

For current bridge state:

```bash
ros2 topic echo /yam_teleop/status --once
```

## Offline verification and constraints

A standalone C++ CAN encoding test (does not activate any motor) can be compiled:

```bash
g++ -std=c++17 -Isrc/yam_ros2_control/include \
  tests/test_can_codec.cpp src/yam_ros2_control/src/yam_can_driver.cpp \
  -o /tmp/yam_test_can_codec
/tmp/yam_test_can_codec
```

The source and configuration were prepared without a ROS 2 Jazzy installation or
physical YAM motors in the authoring environment. A successful NUC `colcon build`
and actual gravity compensation/tracking on both physical arms are **not yet
verified**. These are hardware-commissioning requirements, not grounds to bypass
software interlocks. In particular, the source URDF gripper/end-effector inertia
must match the installed leader hardware for correct gravity torque, and the
physical mount must correspond to model gravity (world -Z).

Tracking design constraints: bridge max 0.20 rad/s, hardware max 0.25 rad/s,
bridge initial alignment 0.10 rad, tracking gap cutoff 0.35 rad, follower error
limit 0.20 rad, stale joint-state cutoff 250 ms, servo watchdog 150 ms, and
per-joint gravity torque caps `[12,12,12,5,5,5]` Nm. These are conservative
initial software parameters, **not empirically validated safety limits**.
