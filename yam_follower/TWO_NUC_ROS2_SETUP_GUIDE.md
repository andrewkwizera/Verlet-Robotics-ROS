# Verlet Robotics YAM teleoperation — complete two-NUC deployment guide

**Target:** Ubuntu 24.04 / ROS 2 Jazzy; two physical 6-DoF YAM arms; each NUC has a separate USB-CAN adapter; C++ `ros2_control` on both NUCs; ROS 2 DDS over Ethernet.

This guide accompanies `YAM-ROS2-Leader-NUC.zip` and `YAM-ROS2-Follower-NUC.zip`. Both ZIPs contain the **same common ROS 2 source repository**, with both launch files, so they can be maintained in **one Git repository**. The **program started on each NUC is different**.

## A. What runs where

| | NUC 1: LEADER (existing) | NUC 2: FOLLOWER (new) |
| --- | --- | --- |
| Attached arm | Physical leader YAM | Physical follower YAM |
| Local CAN | Usually `can0` | Usually `can0` (names independent per host) |
| ROS 2 manager | `/leader/controller_manager` | `/follower/controller_manager` |
| Robot telemetry | Publishes `/leader/joint_states` | Publishes `/follower/joint_states` |
| Motor control | Local gravity compensation | Local position control + gravity feedforward |
| Tracking bridge | **NOT running here** | `yam_live_bridge` runs **here** |
| Launch file | `yam_leader_nuc.launch.py` | `yam_follower_nuc.launch.py` |

**Data path:** physical leader -> local SocketCAN -> `/leader/joint_states` -> Ethernet DDS -> `yam_live_bridge` (follower NUC) -> `/follower/yam_position_controller/commands` -> local `ros2_control` -> local SocketCAN -> physical follower.

**Key design choice:** Only the leader's 6-DoF joint states cross the network. Follower CAN traffic and servo commands stay on the follower computer. Do **not** start `yam_live_teleop.launch.py` on either NUC for two-computer operation; that old launch tries to claim both `can0` and `can1` on the same computer.

## B. Prepare both NUCs

1. Connect each arm's USB-CAN adapter only to its assigned NUC. Make sure mechanical supports and the independent emergency stop are present on **both** systems. Do not run the Python I2RT controller at the same time as ROS 2; two processes must not transmit commands to one CAN arm.
2. Connect NUCs with wired Ethernet to the same LAN/switch; an ordinary router with DHCP is easiest. The lab teammate should obtain or reserve two stable addresses. **Documentation-only TEST-NET addresses**: leader `192.0.2.11`, follower `192.0.2.12`. Substitute the real addresses throughout.
3. On each NUC:

   ```bash
   hostname
   ip -br addr
   ip route
   nmcli device status
   ```

4. Test IP connectivity **both directions**:

   ```bash
   # On LEADER, using follower's REAL address:
   ping -c 4 192.0.2.12
   # On FOLLOWER, using leader's REAL address:
   ping -c 4 192.0.2.11
   ```

   If either fails, fix Ethernet addressing/switch/VLAN/firewall first. On a direct cable with no DHCP, the network teammate must configure two addresses in one subnet (e.g., `/24`) on the actual Ethernet interfaces. Do **not** blindly overwrite the SSH network connection you are currently using.

5. On the new NUC install any missing ROS 2 dependencies (Jazzy/`ros2_control` are already installed, but the controller **plugins** are separate packages):

   ```bash
   sudo apt update
   sudo apt install -y \
     python3-colcon-common-extensions python3-rosdep can-utils \
     ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
     ros-jazzy-xacro ros-jazzy-robot-state-publisher \
     ros-jazzy-kdl-parser ros-jazzy-orocos-kdl \
     ros-jazzy-rmw-fastrtps-cpp
   ```

   Installing the same dependencies on the existing NUC is optional if it already builds the source. For repeatable builds, run (on each system with source in place):

   ```bash
   source /opt/ros/jazzy/setup.bash
   sudo rosdep init 2>/dev/null || true
   rosdep update
   cd ~/yam_ros2_ws
   rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
   ```

## C. Put the code on both machines (recommended: the same Git repository)

**Use a single Git repo, not two diverging repos.** Both ZIPs contain a complete `src/` directory and both launch files. On your development PC, overlay the extracted `src/` from **one** ZIP into the existing Git checkout; do not delete your existing `src/` folder. Add the two new launch files and any modified source, commit, and push:

```bash
git status
git add src scripts README_TWO_NUCS.md
git commit -m "Run YAM leader and follower on separate ROS2 NUCs"
git push
```

For the existing leader NUC:

```bash
cd ~/yam_ros2_ws
git pull --ff-only
```

For the new follower NUC, if the repository is not yet cloned:

```bash
cd ~
git clone <YOUR_REAL_GIT_REPOSITORY_URL> yam_ros2_ws
cd ~/yam_ros2_ws
```

Use the actual Git URL in place of the example above. If you cannot access the Git repo on NUC 2, transfer `YAM-ROS2-Follower-NUC.zip` and extract into `~/yam_ros2_ws/` **so it creates** `~/yam_ros2_ws/src/...`. Do not extract an entire nested project folder into `src/`.

**Important:** Do not create backups such as `src.backup.*` **inside** `~/yam_ros2_ws/`: `colcon` discovers duplicate package names there. Put backups outside the workspace.

## D. Build on BOTH machines

Run independently on each NUC:

```bash
cd ~/yam_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon list --base-paths src
colcon build \
  --packages-select \
  yam_description yam_ros2_control yam_teleop_cpp yam_bringup \
  --symlink-install
source install/setup.bash
ros2 launch yam_bringup yam_leader_nuc.launch.py --show-args
ros2 launch yam_bringup yam_follower_nuc.launch.py --show-args
```

The expected package names are `yam_description`, `yam_ros2_control`, `yam_teleop_cpp`, and `yam_bringup`. The old one-NUC launch remains installed for local rollback, but must not be used in the new two-NUC setup.

## E. Set CAN on both machines (local USB-CAN, not Ethernet)

With motor-control programs stopped, on each machine:

```bash
ip -br link | grep can
sudo ip link set can0 down  # if already UP
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

Expected `can0`, `bitrate 1000000`, interface state `UP`, and no BUS-OFF. If local adapter instead shows `can1`, the corresponding role launch accepts `can_interface:=can1`. The interface name used on the first NUC has no relation to the name used on the second.

For a persistent configuration, the teammate may add a systemd/NetworkManager CAN setup on each host; for the first deployment, the manual commands are sufficient.

## F. Configure ROS 2 networking (BOTH NUCs, EVERY session)

ROS 2 DDS discovers participants only when domain, middleware, network reachability and discovery settings are compatible. The included script sets a consistent domain and Fast DDS on both:

```bash
cd ~/yam_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source scripts/nuc_network_env.sh
printf 'ROS_DOMAIN_ID=%s RMW_IMPLEMENTATION=%s ROS_AUTOMATIC_DISCOVERY_RANGE=%s\n' \
  "$ROS_DOMAIN_ID" "$RMW_IMPLEMENTATION" "$ROS_AUTOMATIC_DISCOVERY_RANGE"
```

The script defaults to:

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
```

**Both NUCs must use the SAME `ROS_DOMAIN_ID` (42 here), not different IDs.** Use another number if 42 conflicts with another robot test in the lab. Both must have `rmw_fastrtps_cpp` installed. Only one of the NUCs may publish `/leader/joint_states`, and only one may publish `/follower/joint_states`.

For a network that blocks multicast discovery, add an optional unicast static-peer configuration **on both NUCs**, with **real** host IP addresses:

```bash
# On LEADER (replace follower IP):
export ROS_STATIC_PEERS=192.0.2.12
# On FOLLOWER (replace leader IP):
export ROS_STATIC_PEERS=192.0.2.11
```

Run the role launch **from the same terminal** with these variables set. Do not assume setting them in one terminal automatically changes another. Static peers do not override firewall/VLAN restrictions or an incompatible RMW. Avoid `ROS_LOCALHOST_ONLY=1`.

If `ufw` is enabled, permit ROS 2 DDS UDP discovery/data **between these two trusted lab IPs**. DDS uses several UDP ports and multicast; the exact allowed ports must match the DDS vendor and domain. Have your network teammate validate firewall rules on the isolated lab interface rather than exposing a broad range to public networks.

### Verify the two machines can exchange ROS 2 messages BEFORE live actuation

1. On LEADER in a terminal with the environment configured:

   ```bash
   ros2 run demo_nodes_cpp talker
   ```

2. On FOLLOWER with the same ROS 2 environment:

   ```bash
   ros2 run demo_nodes_cpp listener
   ```

   The follower must print received messages. Stop both with `Ctrl+C`.

3. **Reverse direction:** start the talker on FOLLOWER and listener on LEADER. Verify traffic flows both ways. If not, fix DDS configuration before starting the arms.

4. Optional: check the environment that runs on each machine:

   ```bash
   echo "$ROS_DOMAIN_ID $RMW_IMPLEMENTATION $ROS_AUTOMATIC_DISCOVERY_RANGE"
   ros2 daemon stop
   ros2 topic list
   ```

## G. Start the actual split ROS 2 program — first in READ-ONLY mode

On the **existing LEADER NUC**, after CAN and ROS environment setup:

```bash
cd ~/yam_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source scripts/nuc_network_env.sh
ros2 launch yam_bringup yam_leader_nuc.launch.py can_interface:=can0
```

On the **new FOLLOWER NUC** in a separate terminal/session:

```bash
cd ~/yam_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source scripts/nuc_network_env.sh
ros2 launch yam_bringup yam_follower_nuc.launch.py can_interface:=can0
```

The default `allow_actuation:=false` avoids motor commands while checking network integration. Do not start the original combined `yam_live_teleop.launch.py` on either system.

With both running, on the follower NUC in another correctly sourced terminal:

```bash
ros2 node list
ros2 topic list -t | grep -E 'leader/joint_states|follower/joint_states|yam_teleop'
ros2 topic echo /leader/joint_states --once
ros2 topic echo /follower/joint_states --once
ros2 topic hz /leader/joint_states
ros2 topic hz /follower/joint_states
ros2 topic echo /yam_teleop/status --once
```

Expected cross-host `/leader/joint_states` and local `/follower/joint_states` (nominal update rate 50 Hz). The bridge status should show fresh leader and follower feedback. At this point, motor actuation remains disabled.

## H. Live physical teleoperation (commissioned operation only)

This section is for the engineering team **after** validating the follower arm's 6-joint zero/direction mapping and gravity/motor torque direction on its new host, confirming supported startup, and confirming independent emergency stops. The `*_verified:=true` launch parameters are *declarations* and do not perform calibration or safety checks. The previous one-NUC working result does not guarantee safe behavior after moving hardware to a different machine. Do not guess joint offsets or increase control limits to mask errors.

1. Stop each read-only launch cleanly (`Ctrl+C`) while the arms remain mechanically supported.
2. On LEADER, using verified offset values if nonzero:

   ```bash
   cd ~/yam_ros2_ws
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   source scripts/nuc_network_env.sh
   ros2 launch yam_bringup yam_leader_nuc.launch.py \
     can_interface:=can0 \
     allow_actuation:=true \
     calibration_verified:=true \
     emergency_stop_verified:=true
   ```

3. On FOLLOWER, after leader is running and fresh feedback is visible, with its **own** commissioned offsets:

   ```bash
   cd ~/yam_ros2_ws
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   source scripts/nuc_network_env.sh
   ros2 launch yam_bringup yam_follower_nuc.launch.py \
     can_interface:=can0 \
     allow_actuation:=true \
     calibration_verified:=true \
     emergency_stop_verified:=true \
     auto_track:=true \
     follower_speed:=0.25
   ```

The follower bridge auto-arms only when both streams are fresh, the position-controller subscriber exists, and all initial joint-position differences are below 0.10 rad. The current bridge stops tracking above 0.35 rad separation, and the follower hardware has additional command-error limits. These limits should **not** be removed simply to make faster gestures follow.

4. In a second terminal on FOLLOWER:

   ```bash
   cd ~/yam_ros2_ws
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   source scripts/nuc_network_env.sh
   ros2 topic echo /yam_teleop/status --once
   ```

   Look for `tracking=ENABLED`, `leader=FRESH`, `follower=FRESH`. If auto-tracking did not arm due to initial alignment, the service can request it **once the arms are in safe matching poses**:

   ```bash
   ros2 service call /yam_teleop/enable_tracking \
     std_srvs/srv/SetBool '{data: true}'
   ```

5. Move the leader smoothly, starting with small motions. The follower responds via the new NUC's local CAN interface. **Do not rapidly swing the leader to validate the network**; quantify latency before increasing speed, as the bridge intentionally disarms when separation grows.
6. Operator-controlled tracking disarm:

   ```bash
   ros2 service call /yam_teleop/enable_tracking \
     std_srvs/srv/SetBool '{data: false}'
   ```

   This disarms tracking but is **not an independent E-stop**, and does not necessarily remove follower motor torque.

## I. What happens if Ethernet communication fails?

The existing C++ bridge checks the arrival of leader and follower joint-state messages, using a 250 ms freshness bound. If `/leader/joint_states` stops arriving, it **disarms tracking**; the follower-side hardware still has its own safety controls and may **hold its last setpoint**, not become passive. The current motor firmware timeout of 8000 ms is *not* a safety-rated stop. That is why an independent safety stop, mechanical support and failure-mode commissioning are required.

Monitor from FOLLOWER:

```bash
ros2 topic echo /yam_teleop/status
```

The status includes tracking state, leader/follower freshness, max joint separation, worst joint, command tracking gap, and configured follower speed.

## J. Troubleshooting for the network teammate

| Symptom | Likely cause / check |
| --- | --- |
| `ping` between NUCs fails | Wrong IP, subnet, cable, switch/VLAN, or firewall. Fix before ROS. |
| Ping works; ROS `talker/listener` fails | Check SAME `ROS_DOMAIN_ID`, SAME `RMW_IMPLEMENTATION`, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, no localhost-only, multicast/UDP firewall. Try `ROS_STATIC_PEERS` with real peer IPs. |
| Both talker/listener directions work; leader joint states absent on follower | Verify leader's role-specific launch, `/leader/joint_state_broadcaster`, and same DDS environment in its launch shell. |
| Leader state visible, follower controller topics absent | Verify only follower role launch running on follower NUC; `ros2 control list_controllers -c /follower/controller_manager`. |
| Hardware says CAN interface missing | Check local `ip -details link show can0`; local adapter might be `can1`, change `can_interface` launch argument. |
| `TRACKING ARMED` then disarmed as `Leader stream stale` | DDS loss, Wi-Fi/jitter, firewall, CPU overrun, or network disconnection. Prefer wired LAN, measure message rate. |
| `TRACKING ARMED` then separation >0.35 rad | Actual follower lag or command speed limiting; **not necessarily network failure**. Inspect `/yam_teleop/status`, controller loop overruns, motor load, physical gravity calibration. |
| ROS 2 controller `LIVE follower CONTROL FAULT` | Hardware-side motor control error. Do not simply raise thresholds. Record exact joint/error; inspect motor direction, torque, load and control. |
| Duplicate colcon package names | Remove/move `src.backup.*` or duplicate package folders outside workspace. |
| `lib...` / CMake dependency missing | Install missing `ros-jazzy-*` dependency and rebuild. |
| Topic is visible, but never publishes | Broadcaster inactive, CAN read failed, or controller hardware deactivated; inspect role launch output. |

## K. Network performance recommendations

* Start with a dedicated gigabit Ethernet LAN or switch, same subnet and stable IPs; avoid Wi-Fi for initial control.
* The current ROS 2 controller/bridge runs at 50 Hz (~20 ms scheduling interval), but that alone is **not** an end-to-end latency guarantee.
* Source-state freshness is 250 ms and follower-speed default is 0.25 rad/s, with joint-error disarm thresholds. Do not make a remote arm chase an unlimited-speed leader.
* Ensure both NUCs have NTP/chrony synchronized clocks before attempting one-way timestamp-based latency analysis. The existing bridge's freshness check uses local steady-clock arrival time and does not need synchronized clocks simply to operate.
* ROS 2 DDS topic transport is not a safety-rated communications channel. The joint controllers and independent E-stop retain responsibility for safe operation.
* For measured latency, log leader header timestamps at receive and follower feedback with clock alignment, or instrument dedicated steady-clock RTT ping/echo. `ping` alone is not a robot-control latency benchmark.

## L. What changed from the successful single-NUC software?

New files:

* `src/yam_bringup/launch/yam_leader_nuc.launch.py`
* `src/yam_bringup/launch/yam_follower_nuc.launch.py`
* `scripts/nuc_network_env.sh`, `scripts/start_leader.sh`, `scripts/start_follower.sh`
* `README_TWO_NUCS.md`
* `src/yam_ros2_control/src/yam_live_hardware.cpp` (role-independent CAN interface validation only)

The last working hardware plugin is preserved except for **one required host-split adjustment** in `src/yam_ros2_control/src/yam_live_hardware.cpp`: the hard-coded `leader=can0`, `follower=can1` mapping has been replaced with validation of the role and local adapter **independently**. Either physical YAM can now use `can0` or `can1` on its own Linux host. Gravity feedforward, persistent target, gripper mass, position gains, velocity limits, bridge topics, and safety thresholds remain unchanged. No new network control protocol is invented. Existing launch and packages are preserved.

**Verification limits:** Both new launch files passed local Python syntax parsing and their XML source descriptions passed offline parsing. ZIP integrity checked. This environment does not have ROS 2 Jazzy binaries or the actual physical NUCs; neither cross-host DDS discovery nor physical control has been demonstrated with these new launches in this environment.
