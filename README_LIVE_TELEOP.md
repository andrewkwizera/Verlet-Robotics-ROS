# Physical YAM → YAM teleoperation with ROS 2 Jazzy (C++)

This patch implements a **separate native-C++ `ros2_control` path** for two physical six-axis YAM Standard arms:

- **Leader**, `can0`: MIT commands, KDL-based gravity feedforward from the supplied YAM V1 URDF; operator manually moves the arm.
- **Follower**, `can1`: MIT position control using the existing YAM PD gains; target comes from a ROS 2 position controller.
- **ROS 2 connection**: `/leader/joint_states` → `yam_live_bridge` → `/follower/yam_position_controller/commands`.
- **Enable tracking**: `/yam_teleop/enable_tracking` (`std_srvs/srv/SetBool`). Tracking starts **disabled**.
- **Hardware power-on interlocks**: `allow_actuation`, `calibration_verified`, and `emergency_stop_verified` launch arguments, each default `false`.

The existing `yam_physical_follower.launch.py` and `YAMPhysicalHardware` read-only path are **preserved**. The old `yam_teleop.launch.py` still launches the synthetic leader; **do not use it for physical teleoperation**.

## Important: status and limitations

The code was integrated from the supplied 2026-11-03 project archive and its CAN register protocol was checked against the motor results already obtained. **ROS 2 Jazzy is not installed in the environment where the patch was prepared, so the new package has NOT been compiled or run against either physical arm.** Source/XML/Python parsing checks do not replace that validation.

This is an **implementation to commission**, not a certified safety controller. In particular:

- Motors and arm links can cause injury. **Both arms require mechanical support**, accessible and independently verified emergency-stop hardware, a clear operating envelope, and a second person during first physical actuation.
- The firmware motor timeout observed on both buses was **8,000 ms**. Loss of the ROS 2 process or USB-CAN can therefore leave torque applied for seconds; a motor disable can instead cause the arm to drop. This is **not an adequate independent safety function**.
- The bridge emits a measured-pose hold if it detects stale ROS messages. If the **bridge process itself crashes**, the standard position controller may retain its last command. An external safety relay or validated hardware watchdog is required for unattended motion.
- **Gripper motor ID 7 is not controlled.** The six shoulder-to-wrist joints are in scope.
- **Joint offsets are not established by the files in this archive.** Xacro defaults of 0.0 are placeholders and must be verified against actual physical joint coordinates. Do not treat the prior raw `xout` telemetry as proof of zero calibration.
- KDL gravity compensation uses the supplied YAM URDF chain `base` → `gripper` and factors `[1,1.1,1.1,1.2,1,1]`. It is not a numerical match test against I2RT's MuJoCo gravity calculation. End-effector hardware, arm mounting, axis direction and torque signs must be verified under mechanical support.
- Control is at 50 Hz for initial bring-up; no real-time scheduling or latency performance is guaranteed. Motor limits, 0.25 rad/s command ramp, 0.20 rad tracking error, and 5 N·m torque cap are conservative *engineering choices*, **not verified safe operating limits**.

## 1. Install changed source files

Copy the patch ZIP to the NUC. In the workspace, stop **all** ROS 2 controller and I2RT processes using `can0` or `can1` first.

```bash
cd ~/yam_ros2_ws
cp -a src "src.backup.$(date +%Y%m%d-%H%M%S)"
unzip -o ~/Downloads/YAM-ROS2-Live-Teleop-Patch.zip
```

If the patch ZIP is in a different directory, change only the ZIP path. `unzip` overlays **complete replacement files** under `src/`; it does not delete your other packages or meshes.

## 2. Build

Use a fresh terminal that has not already sourced an old copy of the workspace:

```bash
cd ~/yam_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select yam_description yam_ros2_control yam_teleop_cpp yam_bringup --symlink-install
source install/setup.bash
```

If the build says `kdl_parser` or `orocos_kdl` was not found:

```bash
sudo apt update
sudo apt install ros-jazzy-kdl-parser ros-jazzy-orocos-kdl
```

If `colcon` shows only its underlay override warning (not a failed build), use a fresh terminal and source `/opt/ros/jazzy/setup.bash` before building, rather than sourcing your own workspace ahead of the build.

## 3. Non-actuating integration startup (default)

This confirms that the *new complete two-namespace launch* and controller plugin registrations load without energizing motor drives:

```bash
source /opt/ros/jazzy/setup.bash
source ~/yam_ros2_ws/install/setup.bash
ros2 launch yam_bringup yam_live_teleop.launch.py
```

In separate terminals:

```bash
ros2 control list_hardware_components -c /leader/controller_manager
ros2 control list_hardware_components -c /follower/controller_manager
ros2 topic echo /leader/joint_states --once
ros2 topic echo /follower/joint_states --once
ros2 control list_controllers -c /follower/controller_manager
```

Both hardware components should be active **in read-only mode**, and all 6 raw position values should be published on each namespace. The follower position controller is harmless in this mode because the hardware plugin does not send MIT commands unless actuation is explicitly authorized.

## 4. Physical operation — only after on-site commissioning

**Do not execute this as an initial blind startup.** Confirm, with the arm mechanically supported and emergency stop independently tested, all joint offsets, axis signs and the KDL gravity torque directions; verify safe torque loss and power-off behavior. Check that `can0` really connects to the intended leader arm and `can1` to the follower. Ensure no other process owns those interfaces. The following command is the supported launch interface **after** those verifications (replace the offset values with the commissioned offsets):

```bash
ros2 launch yam_bringup yam_live_teleop.launch.py \
  allow_actuation:=true \
  calibration_verified:=true \
  emergency_stop_verified:=true \
  leader_offset_1:=0.0 leader_offset_2:=0.0 leader_offset_3:=0.0 \
  leader_offset_4:=0.0 leader_offset_5:=0.0 leader_offset_6:=0.0 \
  follower_offset_1:=0.0 follower_offset_2:=0.0 follower_offset_3:=0.0 \
  follower_offset_4:=0.0 follower_offset_5:=0.0 follower_offset_6:=0.0
```

The `0.0` entries above are **format examples, not approved calibrations**. Replace every value before actuation. Hardware activation will verify motor gear ratios (`Gr`), `CTRL_MODE=1`, `PMAX/VMAX/TMAX`, and nonzero firmware timeout; no Flash writes occur. It will enable MIT mode for each bus, initialize follower targets to measured pose, and run leader gravity compensation. Controller Manager automatically activates hardware components during launch; there is **no extra enable command** once `allow_actuation=true` is passed. Both arms must be supported **before** launching.

When both `joint_states` streams are live and the physical arms have been carefully brought within 0.10 rad on every joint, explicitly arm tracking:

```bash
ros2 service call /yam_teleop/enable_tracking std_srvs/srv/SetBool '{data: true}'
```

Then move the leader **slowly**. Tracking is disabled automatically for messages older than 200 ms, >0.35 rad separation, or a motor fault. To **stop tracking but continue holding the follower**:

```bash
ros2 service call /yam_teleop/enable_tracking std_srvs/srv/SetBool '{data: false}'
```

Do **not** kill the ROS process or press a motor-disabling E-stop while the arm is unsupported; the arm may fall. An independent emergency-stop function must be available for imminent hazards, with the falling-arm risk addressed mechanically.

## 5. Layout

```text
yam_ros2_control/
  include/yam_ros2_control/yam_live_hardware.hpp   physical ros2_control plugin
  src/yam_live_hardware.cpp                       leader/follower modes and safety interlock
  include/yam_ros2_control/yam_can_register_reader.hpp   read-only register verification
  include/yam_ros2_control/yam_can_driver.hpp     MIT codec / enable / disable
  src/yam_can_driver.cpp                          native SocketCAN motor I/O

yam_teleop_cpp/src/yam_live_bridge.cpp            physical joint-state mirror and enable service

yam_description/urdf/yam_live.urdf.xacro          actual YAM V1 links/joints + live hardware parameters

yam_bringup/config/yam_live_controllers.yaml     separate namespaced controllers at 50 Hz
yam_bringup/launch/yam_live_teleop.launch.py       leader and follower controller_managers
```

## 6. Completion criteria

A successful `colcon` build, both Controller Managers loaded, six real joints per bus updating, live MIT mode without faults, measured follower tracking of deliberate leader movements under mechanical support, and successful tracking-disarm and hardware emergency-stop tests are all required before calling the implementation **physically validated**. None of the above is asserted solely by generating this patch.
