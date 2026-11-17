"""System-health dimension.

Health is derived only from the telemetry the dashboard already receives.  It
answers "is the observation path healthy?", not "is the robot safe?".  Safety
is a separate, input-driven dimension.
"""

from __future__ import annotations

from typing import Dict, Optional

from .monitor_view import MonitorView, StreamView
from .reasons import Reason
from .results import DimensionResult
from .states import (
    HEALTH_DEGRADED,
    HEALTH_DISCONNECTED,
    HEALTH_HEALTHY,
    SEVERITY_ERROR,
    SEVERITY_WARNING,
)


class HealthEvaluator:
    def __init__(self, config, telemetry_config) -> None:
        self.config = config
        self.telemetry = telemetry_config
        self._last_rejected: Dict[str, int] = {}
        self._rejected_until: Dict[str, float] = {}

    def evaluate(self, view: MonitorView, now: float) -> DimensionResult:
        reasons = []
        disconnected = False
        evidence = {}

        for role in ("leader", "follower"):
            stream = view.stream(role)
            evidence[f"{role}_state"] = stream.state
            evidence[f"{role}_age_ms"] = _round(stream.age_ms)
            evidence[f"{role}_rate_hz"] = _round(stream.rate_hz, 2)
            evidence[f"{role}_rejected"] = stream.rejected

            if stream.state == "disconnected":
                disconnected = True
                reasons.append(Reason(
                    code=f"health.{role}.disconnected",
                    severity=SEVERITY_ERROR,
                    message=(
                        f"{role.capitalize()} telemetry disconnected"
                        + (
                            f" (last message {stream.age_ms:.0f} ms ago)"
                            if stream.age_ms is not None
                            else " (no message received)"
                        )
                    ),
                    source=role,
                    evidence={
                        "state": stream.state,
                        "age_ms": _round(stream.age_ms),
                        "count": stream.count,
                    },
                ))
                continue

            if stream.state in ("warning", "stale"):
                reasons.append(Reason(
                    code=f"health.{role}.{stream.state}",
                    severity=SEVERITY_WARNING,
                    message=(
                        f"{role.capitalize()} telemetry is {stream.state}"
                        f" (message age {stream.age_ms:.0f} ms)"
                    ),
                    source=role,
                    evidence={
                        "state": stream.state,
                        "age_ms": _round(stream.age_ms),
                    },
                ))

            reasons.extend(self._quality_reasons(role, stream, now))

        state = (
            HEALTH_DISCONNECTED if disconnected
            else HEALTH_DEGRADED if reasons
            else HEALTH_HEALTHY
        )
        return DimensionResult(
            state=state,
            reasons=reasons,
            evidence=evidence,
        )

    # -- internals ----------------------------------------------------------

    def _quality_reasons(
        self, role: str, stream: StreamView, now: float
    ) -> list[Reason]:
        reasons = []
        cfg = self.telemetry

        rate = stream.rate_hz
        if rate is not None and (rate < cfg.rate_min_hz or rate > cfg.rate_max_hz):
            reasons.append(Reason(
                code=f"health.{role}.rate_out_of_band",
                severity=SEVERITY_WARNING,
                message=(
                    f"{role.capitalize()} publish rate {rate:.1f} Hz outside "
                    f"{cfg.rate_min_hz:.0f}-{cfg.rate_max_hz:.0f} Hz band"
                ),
                source=role,
                evidence={
                    "rate_hz": round(rate, 3),
                    "min_hz": cfg.rate_min_hz,
                    "max_hz": cfg.rate_max_hz,
                },
            ))

        previous = self._last_rejected.get(role)
        if previous is not None and stream.rejected > previous:
            self._rejected_until[role] = now + self.config.reject_window_s
        self._last_rejected[role] = stream.rejected
        if now < self._rejected_until.get(role, -1.0):
            reasons.append(Reason(
                code=f"health.{role}.rejected_frames",
                severity=SEVERITY_WARNING,
                message=(
                    f"{role.capitalize()} rejected {stream.rejected} malformed "
                    "joint-state frame(s) recently"
                ),
                source=role,
                evidence={
                    "rejected": stream.rejected,
                    "window_s": self.config.reject_window_s,
                },
            ))

        delta = stream.source_dt_ms
        if delta is not None and (
            delta <= 0.0 or delta > self.config.source_dt_warn_ms
        ):
            reasons.append(Reason(
                code=f"health.{role}.source_stamp_gap",
                severity=SEVERITY_WARNING,
                message=(
                    f"{role.capitalize()} source stamps advance by {delta:.1f} ms "
                    f"(limit {self.config.source_dt_warn_ms:.0f} ms)"
                ),
                source=role,
                evidence={
                    "source_dt_ms": round(delta, 3),
                    "limit_ms": self.config.source_dt_warn_ms,
                },
            ))

        return reasons


def _round(value: Optional[float], digits: int = 1) -> Optional[float]:
    if value is None:
        return None
    return round(float(value), digits)
