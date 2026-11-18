"""Read-only snapshot of the YAM C++ teleoperation bridge status.

The bridge publishes a ``std_msgs/String`` key=value line once per second.
This module deliberately does *not* interpret joint streams as proof that
physical tracking is armed. It never produces motor commands or safety votes.
"""

from __future__ import annotations

import math
import re
import threading
import time
from typing import Optional

_KEY = re.compile(r"(?:^|\s)([a-z][a-z0-9_]*)=")
_FLOAT_FIELDS = ("max_separation_rad", "command_gap_rad", "leader_age_ms", "follower_age_ms")
_FRESH_FOR_S = 2.5  # Bridge status publication interval is 1 second.


def parse_bridge_status(message: str) -> dict:
    """Parse the bridge status without assuming the disarm reason is one word."""
    markers = list(_KEY.finditer(message))
    fields: dict[str, str] = {}
    for index, marker in enumerate(markers):
        end = markers[index + 1].start() if index + 1 < len(markers) else len(message)
        fields[marker.group(1)] = message[marker.end():end].strip()[:256]
    tracking = fields.get("tracking")
    if tracking not in ("ENABLED", "DISABLED"):
        raise ValueError("Bridge status did not include a valid tracking field")
    result: dict = {
        "tracking": tracking,
        "leader": fields.get("leader", "UNKNOWN"),
        "follower": fields.get("follower", "UNKNOWN"),
        "actuation_authorized": fields.get("actuation_authorized") == "true",
        "last_disarm_reason": fields.get("last_disarm_reason", "none"),
    }
    for field in _FLOAT_FIELDS:
        try:
            value = float(fields.get(field, "nan"))
            result[field] = value if math.isfinite(value) else None
        except (TypeError, ValueError):
            result[field] = None
    result["worst_joint"] = fields.get("worst_joint", "unknown")
    return result


class BridgeStatusMonitor:
    """Monotonic-age status snapshot; independent of safety supervision."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._received: Optional[float] = None
        self._last: Optional[dict] = None

    def update(self, message: str, now: Optional[float] = None) -> bool:
        try:
            parsed = parse_bridge_status(message)
        except ValueError:
            return False
        if now is None:
            now = time.monotonic()
        with self._lock:
            self._last = parsed
            self._received = now
        return True

    def snapshot(self, now: Optional[float] = None) -> dict:
        if now is None:
            now = time.monotonic()
        with self._lock:
            last = dict(self._last) if self._last is not None else None
            received = self._received
        if last is None or received is None:
            return {"available": False, "fresh": False, "tracking": "UNKNOWN",
                    "age_ms": None, "reason": "No bridge status received"}
        age_ms = max(0.0, (now - received) * 1000.0)
        if age_ms > _FRESH_FOR_S * 1000.0:
            # Never display an old ENABLED state as a live, armed state.
            return {"available": True, "fresh": False, "tracking": "UNKNOWN",
                    "age_ms": age_ms, "reason": "Bridge status stale",
                    "last_disarm_reason": last.get("last_disarm_reason", "none")}
        return {"available": True, "fresh": True, "age_ms": age_ms, **last}
