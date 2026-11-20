# M3 Phase 1c — standalone WebRTC gateway

A ROS-free Follower-side process that receives two already-encoded H.264 RTP
streams and forwards them through two independent `webrtcbin` peer connections
to a browser. Signaling uses the Phase 1b API on the Leader dashboard.

This is a research prototype. The physical camera sender now provides local
RTP copies for this gateway, but physical validation is still required. The
gateway makes no glass-to-glass latency claim.

## Architecture

```text
D405 RTP  :5100 ─┐
                 ├─ udpsrc -> rtpjitterbuffer -> rtph264depay -> h264parse
D455f RTP :5102 ─┘     -> rtph264pay -> queue -> webrtcbin -> browser
                              (no decode / encode / convert)
```

- One `CameraPipeline` and one signaling worker per camera (`d405`, `d455f`).
- Browser creates a signaling session and posts one offer per camera.
- Gateway discovers the newest active session via `GET /api/webrtc/sessions`,
  answers each camera offer, and publishes gateway-side stats/health.
- Non-trickle ICE only; no audio; H.264 only.
- Session IDs are kept in memory and never logged. If the selected session
  expires, the gateway rediscovers another active session and rebuilds **only**
  the camera that receives a new offer.

## Run (development)

1. Start the dashboard with signaling enabled (loopback by default):

   ```bash
   ros2 run yam_sim_display dashboard_node --ros-args \
     -p webrtc.enabled:=true -p port:=8080
   ```

2. Start each synthetic sender from the unchanged Phase 0 spike:

   ```bash
   python3 yam_sim_display/spike/webrtc_poc/sender.py --id d405 --port 5100
   python3 yam_sim_display/spike/webrtc_poc/sender.py --id d455f --port 5102
   ```

3. Start the gateway from source (no ROS setup required):

   ```bash
   python3 yam_sim_display/gateway/run.py \
     --signaling http://127.0.0.1:8080 --run-dir /tmp/yam_webrtc_gateway
   ```

   After `colcon build` + `source install/setup.bash`:

   ```bash
   ros2 run yam_sim_display webrtc_gateway --config \
     install/yam_sim_display/share/yam_sim_display/config/webrtc_gateway.json
   ```

4. Open the diagnostic page:

   ```
   http://127.0.0.1:8080/webrtc_diagnostic.html
   ```

   The page is a test harness, not the production operator UI. It creates its
   own session, keeps it alive, shows per-camera browser stats separately from
   gateway stats, and offers per-camera retry buttons.

5. Open the production operator dashboard:

   ```
   http://127.0.0.1:8080/
   ```

   With `webrtc.enabled:=false` (the default), the existing M2 camera fallback
   is unchanged. With signaling enabled and the gateway running, `#card-camera`
   shows two independent D405/D455f video panels backed by the same shared
   browser client. Browser and gateway counters are rendered in separate,
   camera-scoped rows. WebRTC health is presentation-only: it never enters
   `/api/state`, supervision, readiness, safety or the operator session.

   Each panel has a retry button for that camera only. Browser-side ICE
   renegotiation is best effort; sender restart remains the verified recovery
   path.

## Configuration

Defaults live in `config/webrtc_gateway.json` and `gateway/config.py`:

```json
{
  "signaling_url": "http://127.0.0.1:8080",
  "cameras": {
    "d405":  {"host": "127.0.0.1", "port": 5100},
    "d455f": {"host": "127.0.0.1", "port": 5102}
  },
  "poll_timeout_s": 10.0,
  "keepalive_interval_s": 60.0,
  "stats_interval_s": 1.0,
  "jitter_buffer_ms": 30,
  "queue_max_packets": 32
}
```

CLI overrides:

- `--config PATH`
- `--signaling URL`
- `--camera ID:HOST:PORT` (repeatable)
- `--only-camera ID` (repeatable; useful when only one camera is connected)
- `--run-dir PATH`
- `--poll-timeout`, `--keepalive-interval`, `--stats-interval`
- `--jitter-buffer-ms`, `--queue-max-packets`

The jitter buffer drops packets that arrive outside its configured latency.
The queue is downstream-leaky, so a slow browser drops old RTP packets instead
of causing video delay to grow over time.

Use `--camera d405:<follower-ip>:5100` and the same form for `d455f` when the
RTP senders are not on the same host as the gateway. No camera launch files,
`cameras.local.yaml`, udev rules, network, VPN or firewall configuration are
changed by this package.

## Stats and health

Gateway-side stats are posted per camera to `POST /api/webrtc/stats` and include
ingress packet/byte counters, negotiated payload type, ICE/connection states,
and `webrtcbin` outbound RTP statistics. They are labeled `source: "gateway"`.

The diagnostic page separately collects browser `RTCPeerConnection.getStats()`
(`source: "browser"`). Connection state, health, and both stat views remain
per-camera; no supervision dimension or safety behavior is modified.

## Tests

```bash
wsl -d Ubuntu-24.04 -- bash -lc 'cd ~/verlet && python3 -m pytest yam_sim_display/test -q'
python3 yam_sim_display/test/e2e_webrtc_gateway.py --page diagnostic
python3 yam_sim_display/test/e2e_webrtc_gateway.py --page dashboard
```

The E2E harness starts both synthetic senders, the gateway, a headless Chrome,
checks two independent video connections, stops one sender to verify the other
continues, restarts only the stopped sender to verify recovery, and verifies the
no-transcode element dumps. It requires GStreamer and Google Chrome and is not
part of the default pytest run. The optional `--check-retry` flag probes the
selected camera's browser-side renegotiation on either page; that probe is best
effort and is recorded separately without affecting the acceptance result.

## Known limitations

- Real D405/D455f RTP inputs are implemented but still require physical
  validation before `PV-RTP-01` can be marked complete.
- One active browser session is served (newest wins); multi-tab concurrency
  is out of scope.
- The per-camera retry buttons on both pages are best effort in headless
  Chrome: in-receiver ICE renegotiation was not reliable under WSL, and the
  verified independent recovery path is synthetic sender restart.
- No authentication, TLS, STUN/TURN or NAT traversal. Host candidates only;
  keep signaling bound to loopback/VPN and restrict network access externally.
- No audio, no transcoding, no glass-to-glass latency measurement.
