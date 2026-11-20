# Physical camera dashboard runbook

For the complete robot, ROS 2, dashboard and camera startup sequence, see
`FULL_SYSTEM_STARTUP_GUIDE.md` at the repository root.

This guide starts one D455f camera on the follower NUC and displays it in the
operator dashboard through WebRTC. Replace values inside angle brackets with
the addresses used by your NUCs. Do not commit those addresses.

## Video path

```text
D455f -> follower H.264 encoder -> local UDP 5102 -> WebRTC gateway
       -> WebRTC network connection -> leader dashboard browser
```

The same encoder also keeps the existing UDP output to leader port `5002`.
The native receiver does not need to run during the dashboard test.

## 1. Install follower dependencies

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
  gstreamer1.0-nice
```

## 2. Create the follower's local camera configuration

Create `yam_follower/src/yam_camera/config/cameras.local.yaml`:

```yaml
cameras:
  d455f:
    device: /dev/v4l/by-id/<D455F_COLOR_VIDEO_INDEX0>

network:
  leader_ip: <LEADER_NUC_IP>
  gateway_host: 127.0.0.1
```

This file is ignored by Git. Use the full stable `/dev/v4l/by-id/` path, not a
changing path such as `/dev/video4`.

## 3. Build on both NUCs

Run from the repository root. Source the existing robot workspace first if it
is built separately.

Follower:

```bash
source /opt/ros/jazzy/setup.bash
colcon build \
  --base-paths yam_follower/src yam_sim_display \
  --packages-select yam_camera yam_sim_display \
  --symlink-install
source install/setup.bash
```

Leader:

```bash
source /opt/ros/jazzy/setup.bash
colcon build \
  --base-paths yam_leader/src yam_sim_display \
  --packages-select yam_sim_display \
  --symlink-install
source install/setup.bash
```

## 4. Start the leader dashboard

Use the leader's private or Tailscale address. Do not bind this prototype to a
public interface because WebRTC signaling currently has no authentication or
TLS.

```bash
ros2 launch yam_sim_display dashboard.launch.py \
  bind_address:=<LEADER_NUC_IP> \
  webrtc_enabled:=true \
  webrtc_cameras:=d455f \
  network_rtt_host:=<FOLLOWER_NUC_IP>
```

Use `webrtc_cameras:=d405,d455f` when both camera gateway workers are running.
The dashboard does not create offers for cameras omitted from this list.

## Two-camera mode

Configure both stable color-device paths on the follower:

```yaml
cameras:
  d405:
    device: /dev/v4l/by-id/<D405_COLOR_VIDEO_INDEX4>
  d455f:
    device: /dev/v4l/by-id/<D455F_COLOR_VIDEO_INDEX0>

network:
  leader_ip: <LEADER_NUC_IP>
  gateway_host: 127.0.0.1
```

Start the leader dashboard with both cameras enabled:

```bash
ros2 launch yam_sim_display dashboard.launch.py \
  bind_address:=<LEADER_NUC_IP> \
  webrtc_enabled:=true \
  webrtc_cameras:=d405,d455f \
  network_rtt_host:=<FOLLOWER_NUC_IP>
```

Start both camera senders on the follower:

```bash
ros2 launch yam_camera follower_camera_sender.launch.py \
  enable_d405:=true \
  enable_d455f:=true
```

Confirm both local RTP inputs before starting the gateway:

```bash
sudo tcpdump -ni lo '(udp port 5100 or udp port 5102)' -c 40
```

Start one independent gateway worker for each camera:

```bash
ros2 run yam_sim_display webrtc_gateway \
  --signaling http://<LEADER_NUC_IP>:8080 \
  --camera d405:127.0.0.1:5100 \
  --camera d455f:127.0.0.1:5102
```

Do not use `--only-camera` in two-camera mode. D405 and D455f negotiate
separate WebRTC peer connections, so one camera can be retried without
restarting the other.

## 5. Start the D455f sender on the follower

```bash
ros2 launch yam_camera follower_camera_sender.launch.py \
  enable_d405:=false \
  enable_d455f:=true
```

Optional local proof that RTP is reaching the gateway port:

```bash
sudo tcpdump -ni lo udp port 5102 -c 20
```

## 6. Start the follower WebRTC gateway

```bash
ros2 run yam_sim_display webrtc_gateway \
  --signaling http://<LEADER_NUC_IP>:8080 \
  --only-camera d455f \
  --camera d455f:127.0.0.1:5102
```

The gateway should report increasing D455f ingress packet counts and should not
contain a decoder, converter or encoder.

## 7. Open and verify the dashboard

Open this address in the leader browser:

```text
http://<LEADER_NUC_IP>:8080/
```

Expected D455f results:

- State changes to `live`.
- Resolution becomes `848x480`.
- FPS approaches `60` after the first statistics sample.
- Bitrate, jitter and transport RTT update.
- Gateway ingress packets continuously increase.
- Dropped-frame and freeze counters do not continuously increase.

RTT and jitter are network transport measurements. Camera-to-screen latency is
measured separately in the physical latency test.

Follow `yam_sim_display/docs/CAMERA_LATENCY_VALIDATION.md` to validate the live
browser estimate against a physical high-speed recording.

## 8. Stop the test

Press `Ctrl+C` in this order:

1. WebRTC gateway.
2. Camera sender.
3. Leader dashboard.
