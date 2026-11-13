# YAM camera streaming

This package keeps the camera video path separate from ROS 2 arm control.
The follower NUC captures and encodes both cameras. The leader NUC receives,
decodes and displays both streams.

## Required GStreamer elements

Confirm every required element is installed on the appropriate NUC:

```bash
for element in v4l2src x264enc rtph264pay udpsink; do
  gst-inspect-1.0 "$element" >/dev/null || exit 1
done

for element in udpsrc rtpjitterbuffer rtph264depay avdec_h264 autovideosink; do
  gst-inspect-1.0 "$element" >/dev/null || exit 1
done
```

## Run only the camera subsystem

On the leader NUC, start the receivers first:

```bash
ros2 launch yam_camera leader_camera_receiver.launch.py
```

On the follower NUC:

```bash
ros2 launch yam_camera follower_camera_sender.launch.py
```

Disable one camera when troubleshooting with `enable_d405:=false` or
`enable_d455f:=false`.

## Run cameras with the robot launch

Camera startup is disabled by default. Enable it explicitly:

```bash
# Leader NUC
./scripts/start_leader.sh enable_cameras:=true

# Follower NUC
./scripts/start_follower.sh enable_cameras:=true leader_ip:=192.168.50.11
```

The stable camera paths and tested video settings are in
`config/cameras.yaml`. Launch arguments can override them without editing code.
