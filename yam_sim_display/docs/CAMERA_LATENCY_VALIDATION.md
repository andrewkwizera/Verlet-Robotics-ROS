# Camera-to-screen latency validation

The dashboard shows an **estimated camera-to-screen latency** for each WebRTC
feed. This estimate is useful for live monitoring, but it is not accepted as a
physical glass-to-glass result until it has been validated with an external
recording.

## What the dashboard estimate measures

For every rendered WebRTC frame, the browser may provide:

- `captureTime`: estimated time at which the remote frame was captured.
- `receiveTime`: time at which the final encoded packet reached the browser.
- `expectedDisplayTime`: time at which the browser expects to display it.

The dashboard calculates:

```text
estimated camera-to-screen = expectedDisplayTime - captureTime
```

It displays the latest value, median and P95 over the latest 120 rendered
frames. The estimate is unavailable when the browser does not expose WebRTC
capture timing.

The browser capture time is derived through RTP/RTCP clock synchronization.
The callback targets browser composition and may not include the monitor's full
physical scan-out. The gateway also repacketizes RTP, so the estimate must be
compared with a physical measurement on this specific pipeline.

## Equipment

- D455f and follower NUC running the physical camera sender.
- Leader dashboard showing the D455f feed.
- A millisecond timer or an LED that can change state clearly.
- A phone or camera capable of recording at 120 FPS or preferably 240 FPS.

At 240 FPS, each recording frame represents approximately `4.17 ms`.

## Physical measurement

1. Put the timer or LED inside the D455f view.
2. Position the high-speed recording camera so the physical timer/LED and the
   leader monitor are visible in the same recording.
3. Start the WebRTC dashboard and wait until its timing sample count reaches
   at least 120.
4. Record at least 100 clear timer changes or LED transitions.
5. For each transition, count the high-speed recording frames between the
   physical event and the same event appearing on the dashboard.
6. Calculate each latency:

```text
latency_ms = frame_difference * 1000 / recording_fps
```

7. Report minimum, median, P95 and maximum.
8. Record the dashboard's estimated median and P95 during the same test.

## Result template

| Measurement | Physical recording | Dashboard estimate |
|---|---:|---:|
| Samples |  |  |
| Minimum |  ms |  ms |
| Median |  ms |  ms |
| P95 |  ms |  ms |
| Maximum |  ms |  ms |

Also record:

- Camera mode and FPS.
- Recording FPS.
- Browser and display refresh rate.
- Network type and RTT.
- WebRTC jitter, dropped frames and freezes.
- Follower and leader CPU use.

## Interpreting the result

Use the physical recording as the authoritative glass-to-glass measurement.
Treat the dashboard value as an operational estimate. If the two differ
significantly, keep the dashboard label as estimated and investigate RTP
timestamp mapping, browser support and monitor refresh behavior before using it
for acceptance testing.
