# Verlet Robotics – YAM Teleoperation (ROS 2)

A ROS 2 workspace for **leader–follower teleoperation** of the I2RT **YAM** 6-DOF arm.
You move a *leader* arm by hand, and a *follower* arm copies it.
The main goal of the project is to keep the delay between the two as **low as possible**.

> **Current status:** the full pipeline runs in **mock mode** (no motors).
> The leader is simulated, and the follower's motors are faked by the hardware plugin.
> Code for driving real motors over CAN exists but has **not been tested on hardware yet**.

---

## 1. How it works

```
 ┌──────────────┐  /yam/leader/joint_states   ┌───────────────┐
 │  yam_leader  │ ──────────────────────────► │ yam_follower  │
 │ (leader arm) │       (JointState)          │ checks + sends│
 └──────────────┘                             └───────┬───────┘
                                                      │ /yam_position_controller/commands
                                                      ▼   (Float64MultiArray)
                                   ┌─────────────────────────────────────┐
                                   │ controller_manager (ros2_control)   │
                                   │  ├─ yam_position_controller         │
                                   │  └─ joint_state_broadcaster ──► /joint_states
                                   └──────────────┬──────────────────────┘
                                                  │ read() / write() every cycle
                                                  ▼
                                   ┌─────────────────────────────────────┐
                                   │ YAMHardwareInterface (plugin)       │
                                   │  mock: state = command              │
                                   │  real: YAMCanDriver ──► CAN ──► motors
                                   └─────────────────────────────────────┘
```

Step by step:

1. **`yam_leader`** publishes the leader arm's 6 joint positions 50 times a second.
2. **`yam_follower`** checks the message (correct joint names, 6 values, no NaN/inf)
   and forwards the positions to the position controller.
3. **`controller_manager`** runs the control loop. On each cycle the
   `yam_position_controller` passes the target positions to the hardware plugin.
4. **`YAMHardwareInterface`** either fakes the motion (mock mode) or sends the
   positions to the real Damiao motors through **`YAMCanDriver`** (real mode).
5. **`joint_state_broadcaster`** publishes the follower's joint states on `/joint_states`, and
   `robot_state_publisher` turns them into TF frames.

---

## 2. Repository layout

```
src/
├── yam_teleop_cpp/        Leader and follower nodes (C++)
│   └── src/
│       ├── leader_node.cpp       publishes leader joint positions (simulated for now)
│       └── follower_node.cpp     validates and forwards them to the controller
│
├── yam_ros2_control/      Hardware layer (C++)
│   ├── src/yam_hardware_interface.cpp   ros2_control plugin (mock or real)
│   ├── src/yam_can_driver.cpp           SocketCAN driver for DM4340 / DM4310 motors
│   └── yam_ros2_control.xml             registers the plugin with pluginlib
│
├── yam_bringup/           Configuration and launch files
│   ├── launch/yam_mock_control.launch.py   starts ros2_control + controllers
│   ├── launch/yam_teleop.launch.py         starts leader + follower nodes
│   ├── urdf/yam_mock.urdf                  robot model + hardware settings
│   └── config/yam_controllers.yaml         controller list and loop rate
│
└── yam_hardware/          Python tools
    └── yam_hardware/leader_driver.py       reads the REAL leader arm (standalone script)
```

---

## 3. Requirements

- **Ubuntu 24.04**
- **ROS 2 Jazzy** ([install guide](https://docs.ros.org/en/jazzy/Installation.html))
- ros2_control packages:

```bash
sudo apt update
sudo apt install \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-robot-state-publisher \
  python3-colcon-common-extensions
```

For real hardware only:
- A USB-to-CAN adapter that works with SocketCAN (shows up as `can0`)
- The I2RT `i2rt` Python SDK (needed only for `leader_driver.py`)

---

## 4. Build

Always build from the **workspace root**, not from inside a package folder.

```bash
cd ~/Verlet-Robotics-ROS
source /opt/ros/jazzy/setup.bash

# Install any missing dependencies listed in package.xml files
rosdep install --from-paths src --ignore-src -y

colcon build --symlink-install
source install/setup.bash
```

Run `source install/setup.bash` again in **every new terminal**.

---

## 5. Run in mock mode (no robot needed)

**Terminal 1: control system**
```bash
ros2 launch yam_bringup yam_mock_control.launch.py
```
This starts `robot_state_publisher` and `controller_manager`, and launches both controllers.

**Terminal 2: teleop nodes**
```bash
ros2 launch yam_bringup yam_teleop.launch.py
```
This starts `yam_leader` (simulated sine-wave motion) and `yam_follower`.

**Terminal 3: check that it works**
```bash
ros2 control list_controllers          # both controllers should be "active"
ros2 topic hz /yam/leader/joint_states # about 50 Hz
ros2 topic echo /joint_states          # follower positions follow the leader
```

You can also open `rviz2`, set the fixed frame to `base_link`, and add a **RobotModel** display
to watch the arm move.

---

## 6. Running on real hardware

> ⚠️ **Safety first.** Real mode moves physical motors. Support the arm, keep the
> workspace clear, and have someone ready to cut power the first time.

### 6.1 Bring up the CAN bus
```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
candump can0        # optional: watch raw CAN traffic
```

### 6.2 Configure the follower in the URDF
Edit `src/yam_bringup/urdf/yam_mock.urdf`:

```xml
<hardware>
  <plugin>yam_ros2_control/YAMHardwareInterface</plugin>
  <param name="use_mock_hardware">false</param>   <!-- was true -->
  <param name="can_interface">can0</param>
</hardware>
```

Each joint has its own motor settings:

| Param         | Meaning                                         |
|---------------|-------------------------------------------------|
| `motor_id`    | CAN ID the motor listens on (e.g. `0x01`)       |
| `response_id` | CAN ID the motor replies with (its "master ID") |
| `motor_type`  | `DM4340` (joints 1–3) or `DM4310` (joints 4–6)  |
| `kp`, `kd`    | Position gain and damping gain                  |

**Check before first use:**
- [ ] `response_id` values match how your motors are configured (`0x11`–`0x16` is a guess).
- [ ] Motor types match your arm.
- [ ] `kp` / `kd` start low; raise them slowly.
- [ ] If the leader and follower are on separate CAN buses, the follower should use its own
      interface (for example `can1`).

Rebuild after editing (`colcon build --symlink-install`), then launch as in section 5.

When the plugin activates, it enables each motor and **holds its current position**,
so the arm should not jump. If any motor times out or reports a fault, the control
loop stops with an error.

### 6.3 Reading the real leader arm (test script)
```bash
python3 src/yam_hardware/yam_hardware/leader_driver.py
```
This puts the leader arm in zero-gravity mode on `can0` and prints its joint positions.
It is **not a ROS node yet**, so nothing is published. Press `Ctrl+C` to stop, then support
the arm before pressing Enter.

---

## 7. Topics and settings

| Topic                                | Type                          | From → To                         |
|--------------------------------------|-------------------------------|-----------------------------------|
| `/yam/leader/joint_states`           | `sensor_msgs/JointState`      | `yam_leader` → `yam_follower`     |
| `/yam_position_controller/commands`  | `std_msgs/Float64MultiArray`  | `yam_follower` → controller       |
| `/joint_states`                      | `sensor_msgs/JointState`      | broadcaster → RViz / TF           |
| `/robot_description`                 | `std_msgs/String`             | `robot_state_publisher` → all     |

| Setting               | Where                       | Current value |
|-----------------------|-----------------------------|---------------|
| Leader publish rate   | `leader_node.cpp` (timer)   | 50 Hz         |
| Control loop rate     | `yam_controllers.yaml`      | 50 Hz         |
| CAN reply timeout     | `yam_can_driver.hpp`        | 10 ms         |

---

## 8. Roadmap / known gaps

- [ ] Turn `leader_driver.py` into a ROS node that publishes the **real** leader arm positions.
- [ ] Test real mode on the follower arm and confirm the CAN IDs and gains.
- [ ] **Measure latency:** compare the leader's `header.stamp` with the time each command reaches the motors.
- [ ] **Reduce latency:**
  - raise the leader and control loop rates (500 Hz–1 kHz);
  - use best-effort QoS with depth 1 so old messages never queue up;
  - send all 6 CAN commands first, then collect the replies (instead of one motor at a time).
- [ ] Add gripper support.
- [ ] Fill in package descriptions and licenses in each `package.xml`.

---

## 9. Troubleshooting

| Problem | Likely cause / fix |
|---------|--------------------|
| `Package 'yam_bringup' not found` | Run `source install/setup.bash` in this terminal. |
| Controllers stay `inactive` / not listed | The spawners failed. Check terminal 1 for plugin load errors, then rebuild. |
| Follower logs `Invalid joint names or position count` | The leader must send exactly `joint_1` … `joint_6`, in order. |
| `CAN interface not found: can0` | The CAN adapter isn't connected or isn't up (see 6.1). |
| `CAN motor feedback timeout` | Wrong `motor_id` / `response_id`, the motor has no power, or the bitrate is wrong. |
| `Motor reported fault code N` | The motor is in an error state; power-cycle it and check the Damiao error code list. |
| Strange build errors after moving folders | Delete `build/ install/ log/` and rebuild from the workspace root. |
