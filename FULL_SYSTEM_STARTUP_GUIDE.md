# YAM full-system startup guide

This guide starts the complete physical teleoperation pipeline:

```text
Leader arm
  -> leader NUC joint states
  -> ROS 2 DDS over the private/Tailscale network
  -> follower bridge and controller
  -> follower arm

Follower D455f camera
  -> YUYV capture
  -> H.264 encode
  -> local RTP/UDP port 5102
  -> follower WebRTC gateway
  -> leader dashboard and browser
```

The dashboard displays both robot arms, tracking state, joint error, network
RTT, the D455f feed, WebRTC statistics and estimated camera-to-screen latency.

## Safety requirements

Before enabling physical actuation:

- Support both arms and clear the workspace.
- Verify joint calibration and motor direction.
- Verify the independent emergency stop.
- Start with the leader and follower in safe, closely aligned positions.
- Keep `auto_track:=false` until both ROS streams and robot positions have been
  checked.

Software follower speed and acceleration ramps have been removed. Do not assume
that a large or sudden position command will be made safe by software ramping.
The automatic reconnect feature is not a safety-rated stop.

## Machines and placeholders

Replace these placeholders locally:

| Placeholder | Meaning |
|---|---|
| `<LEADER_IP>` | Leader NUC's private or Tailscale IP |
| `<FOLLOWER_IP>` | Follower NUC's private or Tailscale IP |
| `<D455F_BY_ID_PATH>` | Full D455f colour path ending in `video-index0` |

Do not commit real IP addresses, camera serial numbers or local credentials.

The commands assume the repository is at `~/yam_ros2_ws`. Adjust that path if
your clone is elsewhere.

## Terminal layout

Use these terminals during commissioning:

| Terminal | Machine | Purpose |
|---|---|---|
| L1 | Leader NUC | Leader arm and joint-state publisher |
| F1 | Follower NUC | Follower arm and teleoperation bridge |
| L2 | Leader NUC | Dashboard, signaling and robot visualization |
| F2 | Follower NUC | D455f capture and H.264 sender |
| F3 | Follower NUC | WebRTC gateway |
| D1 | Either NUC | ROS and network diagnostics |

## Part 1: One-time preparation

### 1. Update the repository

Run on both NUCs:

```bash
cd ~/yam_ros2_ws
git fetch origin
```

Switch to the shared `main` branch and update it:

```bash
git switch main
git pull --ff-only origin main
```

### 2. Install follower video dependencies

Run on the follower NUC:

```bash
sudo apt update
sudo apt install -y \
  python3-gi \
  gir1.2-gst-plugins-bad-1.0 \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gstreamer1.0-nice
```

Confirm the required elements:

```bash
for plugin in \
  v4l2src videoconvert x264enc h264parse rtph264pay \
  rtph264depay rtpjitterbuffer webrtcbin nicesrc
do
  gst-inspect-1.0 "$plugin" >/dev/null 2>&1 \
    && echo "$plugin: FOUND" \
    || echo "$plugin: MISSING"
done
```

Every element must report `FOUND`.

### 3. Configure the D455f locally

Find its stable colour interface on the follower:

```bash
for device in /dev/v4l/by-id/*455*; do
  echo "===== $device ====="
  v4l2-ctl --device="$device" --list-formats 2>/dev/null
done
```

Use the path that reports `YUYV`. It normally ends in `video-index0`.

Create this ignored file:

```text
yam_follower/src/yam_camera/config/cameras.local.yaml
```

Contents:

```yaml
cameras:
  d455f:
    device: <D455F_BY_ID_PATH>

network:
  leader_ip: <LEADER_IP>
  gateway_host: 127.0.0.1
```

Confirm `848x480` at `60 FPS`:

```bash
v4l2-ctl \
  --device="<D455F_BY_ID_PATH>" \
  --list-formats-ext | grep -A 8 "848x480"
```

Confirm the camera is on USB 3. The RealSense entry should show `5000M` or
higher:

```bash
lsusb -t
```

### 4. Permanently disable Wi-Fi power saving

Run on both NUCs, replacing the interface if it is not `wlp172s0`:

```bash
YAM_WIFI_PROFILE="$(nmcli -g GENERAL.CONNECTION device show wlp172s0)"

sudo nmcli connection modify \
  "$YAM_WIFI_PROFILE" \
  802-11-wireless.powersave 2

sudo iw dev wlp172s0 set power_save off

printf '[connection]\nwifi.powersave=2\n' |
  sudo tee /etc/NetworkManager/conf.d/99-yam-wifi-powersave-off.conf

sudo nmcli general reload
iw dev wlp172s0 get power_save
```

Expected result:

```text
Power save: off
```

### 5. Build the robot workspaces

Leader NUC:

```bash
cd ~/yam_ros2_ws/yam_leader
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Follower NUC:

```bash
cd ~/yam_ros2_ws/yam_follower
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 6. Build the dashboard and WebRTC gateway

Run from the repository root on both NUCs. On the leader, source
`yam_leader/install/setup.bash`. On the follower, source
`yam_follower/install/setup.bash`.

Leader NUC:

```bash
cd ~/yam_ros2_ws
source /opt/ros/jazzy/setup.bash
source yam_leader/install/setup.bash

colcon --log-base log_dashboard build \
  --base-paths yam_sim_display \
  --build-base build_dashboard \
  --install-base install_dashboard \
  --symlink-install

source install_dashboard/setup.bash
```

Follower NUC:

```bash
cd ~/yam_ros2_ws
source /opt/ros/jazzy/setup.bash
source yam_follower/install/setup.bash

colcon --log-base log_dashboard build \
  --base-paths yam_sim_display \
  --build-base build_dashboard \
  --install-base install_dashboard \
  --symlink-install

source install_dashboard/setup.bash
```

## Part 2: Start the complete system

### Step 1: Check the NUC-to-NUC network

Leader NUC:

```bash
tailscale status
tailscale ping <FOLLOWER_IP>
ping -c 20 <FOLLOWER_IP>
```

Follower NUC:

```bash
tailscale status
tailscale ping <LEADER_IP>
ping -c 20 <LEADER_IP>
```

Both directions should respond. Resolve packet loss or large latency spikes
before enabling the robots.

### Step 2: Start the leader arm — terminal L1

For a communication-only startup, keep actuation disabled:

```bash
cd ~/yam_ros2_ws

export ROS_DOMAIN_ID=43
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS=<FOLLOWER_IP>

bash yam_leader/scripts/start_leader.sh \
  can_interface:=can0 \
  allow_actuation:=false \
  calibration_verified:=false \
  emergency_stop_verified:=false \
  enable_cameras:=false
```

Only after the physical safety checks have genuinely passed, the three
authorization values may be changed to `true`:

```bash
bash yam_leader/scripts/start_leader.sh \
  can_interface:=can0 \
  allow_actuation:=true \
  calibration_verified:=true \
  emergency_stop_verified:=true \
  enable_cameras:=false
```

Leave terminal L1 running.

### Step 3: Start the follower arm — terminal F1

Communication-only startup:

```bash
cd ~/yam_ros2_ws

export ROS_DOMAIN_ID=43
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS=<LEADER_IP>

bash yam_follower/scripts/start_follower.sh \
  can_interface:=can0 \
  allow_actuation:=false \
  calibration_verified:=false \
  emergency_stop_verified:=false \
  auto_track:=false \
  auto_reconnect:=true \
  enable_cameras:=false
```

Physical startup, only after the safety checks pass:

```bash
bash yam_follower/scripts/start_follower.sh \
  can_interface:=can0 \
  allow_actuation:=true \
  calibration_verified:=true \
  emergency_stop_verified:=true \
  auto_track:=false \
  auto_reconnect:=true \
  enable_cameras:=false
```

`auto_track:=false` keeps initial tracking under deliberate operator control.
Leave terminal F1 running.

### Step 4: Start the dashboard on the leader — terminal L2

```bash
cd ~/yam_ros2_ws

source /opt/ros/jazzy/setup.bash
source yam_leader/install/setup.bash
source install_dashboard/setup.bash
source yam_leader/scripts/nuc_network_env.sh

export ROS_DOMAIN_ID=43
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS=<FOLLOWER_IP>

ros2 launch yam_sim_display dashboard.launch.py \
  bind_address:=<LEADER_IP> \
  webrtc_enabled:=true \
  network_rtt_host:=<FOLLOWER_IP>
```

WebRTC signaling currently has no authentication or TLS. Bind it only to a
trusted private or Tailscale address, never a public interface.

Open this URL in the leader browser:

```text
http://<LEADER_IP>:8080/
```

The robot panels should begin displaying joint data once ROS discovery is
working. The D455f panel will remain waiting until steps 5 and 6 are started.

### Step 5: Start D455f capture — terminal F2

```bash
cd ~/yam_ros2_ws

source /opt/ros/jazzy/setup.bash
source yam_follower/install/setup.bash

ros2 launch yam_camera follower_camera_sender.launch.py \
  enable_d405:=false \
  enable_d455f:=true
```

The camera is captured and encoded once. The encoded D455f RTP is split into:

- Port `5002` toward the optional native leader receiver.
- Local port `5102` for the follower WebRTC gateway.

Optional proof that local RTP is present:

```bash
sudo tcpdump -ni lo udp port 5102 -c 20
```

### Step 6: Start the WebRTC gateway — terminal F3

```bash
cd ~/yam_ros2_ws

source /opt/ros/jazzy/setup.bash
source yam_follower/install/setup.bash
source install_dashboard/setup.bash

ros2 run yam_sim_display webrtc_gateway \
  --signaling http://<LEADER_IP>:8080 \
  --only-camera d455f \
  --camera d455f:127.0.0.1:5102
```

Expected gateway behavior:

- It discovers the browser session.
- It posts a D455f WebRTC answer.
- D455f ingress packet counts increase.
- Its media path contains no decoder, converter or encoder.

### Step 7: Verify ROS 2 communication — terminal D1

Run on the follower:

```bash
cd ~/yam_ros2_ws

source /opt/ros/jazzy/setup.bash
source yam_follower/install/setup.bash
source yam_follower/scripts/nuc_network_env.sh

export ROS_DOMAIN_ID=43
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS=<LEADER_IP>

ros2 daemon stop
ros2 topic list
```

Check both joint streams:

```bash
ros2 topic hz /leader/joint_states
```

Press `Ctrl+C`, then:

```bash
ros2 topic hz /follower/joint_states
```

Press `Ctrl+C`, then inspect the bridge:

```bash
ros2 topic echo /yam_teleop/status --once
```

Both streams must be fresh before tracking is enabled. If Tailscale ping works
but the ROS topics are missing, stop here and inspect DDS interface selection,
the static peer addresses, `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION`.

### Step 8: Verify the dashboard and camera

The dashboard should show:

- Live leader and follower arm visualization.
- Fresh joint-state streams and tracking status.
- Joint error values.
- ICMP network RTT.
- A live D455f image at approximately `848x480` and `60 FPS`.
- WebRTC bitrate, jitter, RTT, packet loss, dropped frames and freeze count.
- Estimated camera-to-screen latest, median and P95 latency.

The browser timing value is an estimate. Use
`yam_sim_display/docs/CAMERA_LATENCY_VALIDATION.md` for the authoritative
physical glass-to-glass test.

### Step 9: Enable initial tracking

Before enabling tracking:

- Confirm both robot streams are fresh.
- Confirm leader and follower joint positions are safely aligned.
- Confirm the follower workspace is clear.
- Keep access to the independent emergency stop.

On the follower diagnostic terminal:

```bash
ros2 service call \
  /yam_teleop/enable_tracking \
  std_srvs/srv/SetBool \
  "{data: true}"
```

Expected response:

```text
success: true
```

If the request is rejected, read the returned reason and correct it. Do not
repeatedly force the request.

After a recoverable leader-network interruption, `auto_reconnect:=true` allows
tracking to resume only after every leader joint is brought within 5 degrees of
the corresponding follower joint and remains stable for 750 ms.

## Part 3: Normal monitoring

During operation, watch for:

- Tracking becoming disabled or stale.
- Increasing leader/follower joint separation.
- Packet loss, RTT or jitter spikes.
- Camera FPS falling below the configured rate.
- Increasing dropped-frame or freeze counters.
- Camera-to-screen latency growing over time.
- USB/UVC errors in terminal F2.
- Gateway negotiation or `no-rtp` errors in terminal F3.
- Excessive NUC CPU use or temperature.

For a commissioning soak test, run the complete system for at least 30 minutes
while recording these values.

## Part 4: Safe shutdown

### 1. Disable tracking first

On the follower diagnostic terminal:

```bash
ros2 service call \
  /yam_teleop/enable_tracking \
  std_srvs/srv/SetBool \
  "{data: false}"
```

Confirm the dashboard reports tracking disabled. Keep both arms supported.

### 2. Stop processes in this order

Press `Ctrl+C` in:

1. F3 — WebRTC gateway.
2. F2 — D455f camera sender.
3. L2 — Dashboard.
4. F1 — Follower robot process.
5. L1 — Leader robot process.

The follower may remain in a motor hold mode when tracking is disabled. This is
not an independent safety-rated stop; use the hardware emergency stop whenever
the physical situation requires it.

## Troubleshooting

### `auto_reconnect` is an unknown launch argument

The follower bringup package is stale. Pull the latest branch, rebuild
`yam_follower`, and source `yam_follower/install/setup.bash`.

### The D455f device path is missing

```bash
ls -l /dev/v4l/by-id/ | grep -i 455
```

Use the `video-index0` path that reports `YUYV`. Do not store `/dev/videoN` in
the local configuration because that number can change after reconnection.

### The camera sender starts but the gateway reports no RTP

```bash
sudo tcpdump -ni lo udp port 5102 -c 20
```

- No packets: inspect terminal F2 and the camera device configuration.
- Packets present: inspect terminal F3, gateway configuration and GStreamer
  dependencies.

### Signaling works but no video appears

- Confirm `webrtcbin` and `nicesrc` are installed on the follower.
- Confirm the leader browser created a WebRTC session.
- Check firewall rules between the NUCs. WebRTC media uses dynamic ICE UDP
  ports in addition to dashboard TCP port `8080`.
- Confirm both NUCs have a direct Tailscale path when possible.

### Tailscale works but ROS topics are absent

Confirm on both NUCs:

```bash
echo "$ROS_DOMAIN_ID"
echo "$ROS_AUTOMATIC_DISCOVERY_RANGE"
echo "$ROS_STATIC_PEERS"
echo "$RMW_IMPLEMENTATION"
```

Both hosts must use the same ROS domain and RMW implementation. Each static
peer must point to the other NUC. DDS may still fail if it advertises an
unreachable interface; inspect Fast DDS interface configuration in that case.

### Camera latency estimate is unavailable

Use a current Chrome/Chromium browser. The dashboard requires WebRTC rendered
frame timing metadata for the live estimate. The physical high-speed recording
procedure remains valid even when browser timing metadata is unavailable.

## Related guides

- `yam_sim_display/docs/CAMERA_DASHBOARD_RUNBOOK.md`
- `yam_sim_display/docs/CAMERA_LATENCY_VALIDATION.md`
- `yam_follower/TWO_NUC_ROS2_SETUP_GUIDE.md`
