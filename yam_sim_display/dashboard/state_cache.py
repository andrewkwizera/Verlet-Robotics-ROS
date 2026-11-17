"""Thread-safe holder for the merged telemetry + supervision snapshot.

The HTTP server reads this object; the ROS timer writes it.  Keeping the merge
outside both ``telemetry.py`` and ``http_server.py`` means neither layer needs
to know about the other's concerns.
"""

from __future__ import annotations

import threading
from typing import Optional


class StateCache:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._snapshot: Optional[dict] = None

    def publish(self, snapshot: dict) -> None:
        with self._lock:
            self._snapshot = snapshot

    def snapshot(self) -> Optional[dict]:
        with self._lock:
            return self._snapshot
