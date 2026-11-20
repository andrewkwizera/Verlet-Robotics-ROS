"""Read-only, background ICMP RTT probe for the dashboard.

RTT is network ping timing, NOT leader-to-follower motion response latency.
No commands or ROS messages are published by this module.
"""
from __future__ import annotations

from collections import deque
import ipaddress
import math
import re
import statistics
import subprocess
import threading
import time


class NetworkRTTProbe:
    def __init__(self, target: str = "") -> None:
        self.target = str(ipaddress.ip_address(target)) if target else ""
        self._samples = deque(maxlen=60)
        self._last_ok = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        if self.target:
            self._thread = threading.Thread(
                target=self._loop, name="yam-rtt-probe", daemon=True
            )
            self._thread.start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                result = subprocess.run(
                    ["ping", "-n", "-c", "1", "-W", "1", self.target],
                    capture_output=True, text=True, timeout=2.0, check=False,
                )
                match = re.search(r"\btime[=<]\s*([0-9]+(?:\.[0-9]+)?)\s*ms", result.stdout)
                if result.returncode == 0 and match:
                    rtt = float(match.group(1))
                    if math.isfinite(rtt) and rtt >= 0:
                        with self._lock:
                            self._samples.append(rtt)
                            self._last_ok = time.monotonic()
            except (OSError, subprocess.TimeoutExpired, ValueError):
                pass
            self._stop.wait(1.0)

    def snapshot(self, now: float | None = None) -> dict:
        if now is None:
            now = time.monotonic()
        if not self.target:
            return {
                "available": False,
                "display": "Network RTT not configured",
                "reason": "Set network_rtt_host to enable the ICMP RTT probe.",
            }
        with self._lock:
            values = list(self._samples)
            age = None if self._last_ok is None else now - self._last_ok
        if not values or age is None or age > 3.0:
            return {
                "available": False,
                "display": "Network RTT unavailable",
                "reason": (f"ICMP probe to {self.target} has no fresh response. "
                           "This panel does not measure robot-motion lag."),
            }
        ordered = sorted(values)
        p95 = ordered[max(0, math.ceil(len(ordered) * 0.95) - 1)]
        latest = values[-1]
        return {
            "available": True,
            "display": f"{latest:.1f} ms RTT",
            "reason": (f"ICMP to leader {self.target} (last {len(values)} samples): "
                       f"median {statistics.median(values):.1f} ms, "
                       f"P95 {p95:.1f} ms. "
                       "This is network round-trip time, not robot response latency."),
        }

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.5)
