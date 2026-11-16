"""Operator-facing reasons attached to supervision dimensions.

A reason is never only a log line: it is part of the state payload so the UI
can explain *why* a dimension has the value it has.  Logging is a side effect
and may be throttled; the reason list is authoritative.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional

from .states import SEVERITY_RANK


@dataclass(frozen=True)
class Reason:
    """One machine-readable cause of a dimension state."""

    code: str
    severity: str
    message: str
    source: str = "telemetry"
    evidence: Optional[dict] = None


@dataclass
class _TrackedReason:
    code: str
    severity: str
    message: str
    source: str
    evidence: Optional[dict]
    first_seen: str
    first_seen_uptime_s: float
    last_seen: str
    episode: int

    def as_dict(self) -> dict:
        return {
            "code": self.code,
            "severity": self.severity,
            "message": self.message,
            "source": self.source,
            "evidence": dict(self.evidence) if self.evidence else {},
            "first_seen": self.first_seen,
            "first_seen_uptime_s": round(self.first_seen_uptime_s, 3),
            "last_seen": self.last_seen,
            # Number of separate episodes this code has been present, not the
            # number of evaluation ticks: the supervisor runs at 20 Hz.
            "count": self.episode,
        }


class ReasonTracker:
    """Deduplicates reasons by code and keeps first/last-seen metadata.

    ``count`` is the number of episodes (an episode starts when a code appears
    after being absent).  It deliberately does not increment per evaluation.
    """

    def __init__(self) -> None:
        self._active: Dict[str, _TrackedReason] = {}
        self._episodes: Dict[str, int] = {}

    def update(
        self,
        reasons: Iterable[Reason],
        wall_iso: str,
        now: float,
    ) -> List[dict]:
        incoming = {reason.code: reason for reason in reasons}
        for code in list(self._active):
            if code not in incoming:
                del self._active[code]

        for code, reason in incoming.items():
            entry = self._active.get(code)
            if entry is None:
                episode = self._episodes.get(code, 0) + 1
                self._episodes[code] = episode
                entry = _TrackedReason(
                    code=code,
                    severity=reason.severity,
                    message=reason.message,
                    source=reason.source,
                    evidence=reason.evidence,
                    first_seen=wall_iso,
                    first_seen_uptime_s=now,
                    last_seen=wall_iso,
                    episode=episode,
                )
                self._active[code] = entry
            else:
                entry.severity = reason.severity
                entry.message = reason.message
                entry.source = reason.source
                entry.evidence = reason.evidence
                entry.last_seen = wall_iso

        return [
            entry.as_dict()
            for entry in sorted(
                self._active.values(),
                key=lambda item: (-SEVERITY_RANK.get(item.severity, 0), item.code),
            )
        ]

    def episode_counts(self) -> Dict[str, int]:
        return {
            code: entry.episode for code, entry in self._active.items()
        }

    def clear(self) -> None:
        self._active.clear()
