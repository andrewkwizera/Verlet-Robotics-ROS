#!/usr/bin/env python3
"""One-time PC-side patch: add an active ICMP RTT monitor to YAM dashboard."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] if Path(__file__).resolve().parent.name == 'scripts' else Path.cwd() / 'yam_sim_display'
NODE = ROOT / 'dashboard' / 'node.py'
HTML = ROOT / 'dashboard' / 'web' / 'index.html'
PROBE = ROOT / 'dashboard' / 'network_probe.py'

PROBE_CODE = '''"""Read-only, background ICMP RTT probe for the dashboard.

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
    def __init__(self, target: str = "100.73.207.42") -> None:
        self.target = str(ipaddress.ip_address(target))
        self._samples = deque(maxlen=60)
        self._last_ok = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="yam-rtt-probe", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                result = subprocess.run(
                    ["ping", "-n", "-c", "1", "-W", "1", self.target],
                    capture_output=True, text=True, timeout=2.0, check=False,
                )
                match = re.search(r"\\btime[=<]\\s*([0-9]+(?:\\.[0-9]+)?)\\s*ms", result.stdout)
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
        self._thread.join(timeout=2.5)
'''

def replace_once(text: str, old: str, new: str, filename: Path) -> str:
    if new in text:
        return text
    if text.count(old) != 1:
        raise RuntimeError(f"Unexpected source content: {filename} (anchor count {text.count(old)})")
    return text.replace(old, new, 1)

if not NODE.exists() or not HTML.exists():
    raise SystemExit(f"Run this script from the repository root or save it under yam_sim_display/scripts. Expected {NODE}")

node = NODE.read_text(encoding='utf-8')
node = replace_once(node,
    'from .bridge_status import BridgeStatusMonitor',
    'from .bridge_status import BridgeStatusMonitor\nfrom .network_probe import NetworkRTTProbe', NODE)
node = replace_once(node,
    '        self.bridge_status = BridgeStatusMonitor()',
    '        self.bridge_status = BridgeStatusMonitor()\n        self.network_probe = NetworkRTTProbe("100.73.207.42")', NODE)
node = replace_once(node,
    '        merged["robot_control"] = self.bridge_status.snapshot(now)',
    '        merged["robot_control"] = self.bridge_status.snapshot(now)\n        merged["latency"] = self.network_probe.snapshot(now)', NODE)
node = replace_once(node,
    '        self._server.stop()',
    '        self.network_probe.stop()\n        self._server.stop()', NODE)

html = HTML.read_text(encoding='utf-8')
html = replace_once(html,
    '<div class="latency-title">Network latency</div>',
    '<div class="latency-title">Network RTT (ICMP)</div>', HTML)

# Write only after all anchor checks succeed.
PROBE.write_text(PROBE_CODE, encoding='utf-8')
NODE.write_text(node, encoding='utf-8')
HTML.write_text(html, encoding='utf-8')
print('Updated yam_sim_display/dashboard/node.py and dashboard/web/index.html')
print('Added yam_sim_display/dashboard/network_probe.py')
print('Live ICMP RTT to leader: 100.73.207.42. Robot commands and ROS topics unchanged.')
