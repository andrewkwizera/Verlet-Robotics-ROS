#!/usr/bin/env python3
"""Executable entry point for the standalone WebRTC gateway.

Runs as an ordinary process: no ROS import, no rclpy, no parameter server.
The GLib main loop dispatches GStreamer promises and ICE state callbacks.
"""

from __future__ import annotations

import os
import signal
import sys


def _package_root() -> str:
    """Find the directory containing the ``gateway`` package."""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.dirname(here),
        os.path.join(
            os.path.dirname(os.path.dirname(here)), "share", "yam_sim_display"
        ),
    ]
    for candidate in candidates:
        if os.path.isfile(os.path.join(candidate, "gateway", "config.py")):
            return candidate
    return candidates[0]


_ROOT = _package_root()
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from gateway.config import from_args  # noqa: E402
from gateway.gateway import Gateway  # noqa: E402

from gi.repository import GLib  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    try:
        config = from_args(argv)
    except (ValueError, OSError) as exc:
        sys.stderr.write(f"webrtc_gateway: invalid configuration: {exc}\n")
        return 2

    os.makedirs(config.run_dir, exist_ok=True)
    gateway = Gateway(config)
    gateway.start()
    loop = GLib.MainLoop()

    def shutdown() -> bool:
        gateway.stop()
        loop.quit()
        return False

    for signum in (signal.SIGINT, signal.SIGTERM):
        try:
            GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, signum, shutdown)
        except Exception:  # noqa: BLE001 - platform without unix signals
            pass

    print(
        f"[gateway] signaling={config.signaling_url} run_dir={config.run_dir}",
        flush=True,
    )
    try:
        loop.run()
    except KeyboardInterrupt:
        shutdown()
    finally:
        gateway.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
