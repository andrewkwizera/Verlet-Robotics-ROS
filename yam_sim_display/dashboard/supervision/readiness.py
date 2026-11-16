"""Operational readiness.

Readiness is a documented conjunction of explicit prerequisites.  It is not a
safety certification and it never overrides the safety dimension: by default
``safety_normal`` is one of the required prerequisites, so a missing safety
source makes readiness *Not Ready* even when every telemetry check passes.
"""

from __future__ import annotations

from typing import Dict, List, Optional

from .monitor_view import MonitorView
from .reasons import Reason
from .results import DimensionResult
from .states import (
    READINESS_INITIALIZING,
    READINESS_NOT_READY,
    READINESS_READY,
    SAFETY_FAULT,
    SAFETY_NORMAL,
    SEVERITY_BLOCKING,
)

# Published verbatim in the API and rendered in the drawer.  ``alignment`` is
# deliberately a diagnostic, not a prerequisite: it is useful to the operator
# but is not required for the documented ready state in Milestone 2.
PREREQUISITE_DEFINITIONS = (
    {
        "id": "leader_fresh",
        "description": "Leader joint states are connected",
        "source": "telemetry",
        "required": True,
    },
    {
        "id": "follower_fresh",
        "description": "Follower joint states are connected",
        "source": "telemetry",
        "required": True,
    },
    {
        "id": "rates_in_band",
        "description": "Both publish rates are within the configured band",
        "source": "telemetry",
        "required": True,
    },
    {
        "id": "joint_set_complete",
        "description": "The expected joint set is present in both streams",
        "source": "telemetry",
        "required": True,
    },
    {
        "id": "tracking_available",
        "description": "Leader and follower poses are fresh and comparable",
        "source": "telemetry",
        "required": True,
    },
    {
        "id": "safety_normal",
        "description": "An explicit safety input reports Normal",
        "source": "safety",
        "required": True,
    },
)

READINESS_NOTE = (
    "Ready means the documented prerequisites hold. It is not a safety "
    "certification, and it does not authorise motion."
)


class ReadinessEvaluator:
    def __init__(self, config) -> None:
        self.config = config
        self.state = READINESS_INITIALIZING
        self._start_uptime: Optional[float] = None
        self._all_pass_since: Optional[float] = None

    def evaluate(
        self,
        view: MonitorView,
        safety_state: str,
        now: float,
        telemetry_config,
    ) -> DimensionResult:
        if self._start_uptime is None:
            self._start_uptime = now

        prerequisites = self._evaluate_prerequisites(
            view, safety_state, telemetry_config
        )
        diagnostics = self._evaluate_diagnostics(view)
        failing = [
            entry for entry in prerequisites
            if entry["required"] and not entry["passed"]
        ]
        all_pass = not failing

        previous = self.state
        if previous == READINESS_INITIALIZING:
            if all_pass:
                self._all_pass_since = (
                    now if self._all_pass_since is None else self._all_pass_since
                )
                if now - self._all_pass_since >= self.config.ready_hold_s:
                    self.state = READINESS_READY
            else:
                self._all_pass_since = None
                grace_elapsed = (
                    now - self._start_uptime
                ) >= self.config.startup_grace_s
                if grace_elapsed or safety_state == SAFETY_FAULT:
                    self.state = READINESS_NOT_READY
        elif previous == READINESS_READY:
            if not all_pass:
                self.state = READINESS_NOT_READY
                self._all_pass_since = None
        else:  # not_ready
            if all_pass:
                self._all_pass_since = (
                    now if self._all_pass_since is None else self._all_pass_since
                )
                if now - self._all_pass_since >= self.config.ready_hold_s:
                    self.state = READINESS_READY
            else:
                self._all_pass_since = None

        reasons = [
            Reason(
                code=f"readiness.{entry['id']}",
                severity=SEVERITY_BLOCKING,
                message=(
                    f"{entry['description']} — {entry['evidence']}"
                ),
                source=entry["source"],
                evidence={
                    "prerequisite": entry["id"],
                    "detail": entry["evidence"],
                },
            )
            for entry in failing
        ]

        extra = {
            "prerequisites": prerequisites,
            "diagnostics": diagnostics,
            "gated_by_safety": True,
            "startup_grace_s": self.config.startup_grace_s,
            "ready_hold_s": self.config.ready_hold_s,
            "note": READINESS_NOTE,
        }
        if self.state == READINESS_INITIALIZING and failing:
            extra["initializing_reason"] = (
                "Waiting for startup grace or for all prerequisites to hold."
            )

        return DimensionResult(
            state=self.state,
            reasons=reasons,
            evidence={
                "failing": [entry["id"] for entry in failing],
                "all_pass": all_pass,
                "since_pass_s": (
                    round(now - self._all_pass_since, 3)
                    if self._all_pass_since is not None else None
                ),
            },
            extra=extra,
        )

    # -- prerequisites ------------------------------------------------------

    def _evaluate_prerequisites(
        self,
        view: MonitorView,
        safety_state: str,
        telemetry_config,
    ) -> List[dict]:
        checks: Dict[str, tuple[bool, str]] = {
            "leader_fresh": _fresh_check(view.stream("leader")),
            "follower_fresh": _fresh_check(view.stream("follower")),
            "rates_in_band": _rate_check(view, telemetry_config),
            "joint_set_complete": _joint_set_check(
                view, self.config.expected_joint_count
            ),
            "tracking_available": _tracking_check(view),
            "safety_normal": (
                safety_state == SAFETY_NORMAL,
                (
                    "safety input reports Normal"
                    if safety_state == SAFETY_NORMAL
                    else f"safety status is {safety_state}"
                ),
            ),
        }

        result = []
        for definition in PREREQUISITE_DEFINITIONS:
            passed, evidence = checks[definition["id"]]
            result.append({
                **definition,
                "passed": passed,
                "evidence": evidence,
            })
        return result

    def _evaluate_diagnostics(self, view: MonitorView) -> List[dict]:
        tracking = view.tracking
        limit = self.config.alignment_diagnostic_rad
        if not tracking.available or tracking.max_abs_error_rad is None:
            return [{
                "id": "alignment",
                "description": (
                    f"Max |leader − follower| is within {limit:.3f} rad"
                ),
                "source": "telemetry",
                "passed": False,
                "diagnostic": True,
                "evidence": "tracking error unavailable",
            }]
        error = tracking.max_abs_error_rad
        return [{
            "id": "alignment",
            "description": f"Max |leader − follower| is within {limit:.3f} rad",
            "source": "telemetry",
            "passed": error <= limit,
            "diagnostic": True,
            "evidence": (
                f"{error * 1000.0:.1f} mrad"
                + (f" on {tracking.max_abs_error_joint}"
                   if tracking.max_abs_error_joint else "")
            ),
        }]


# -- individual checks ------------------------------------------------------


def _fresh_check(stream) -> tuple[bool, str]:
    if stream.state == "connected":
        age = "no age" if stream.age_ms is None else f"{stream.age_ms:.0f} ms old"
        return True, f"connected ({age})"
    return False, f"{stream.state} ({stream.count} messages)"


def _rate_check(view: MonitorView, telemetry_config) -> tuple[bool, str]:
    parts = []
    passed = True
    for role in ("leader", "follower"):
        rate = view.stream(role).rate_hz
        if rate is None:
            passed = False
            parts.append(f"{role} rate unavailable")
        elif not (telemetry_config.rate_min_hz <= rate <= telemetry_config.rate_max_hz):
            passed = False
            parts.append(f"{role} {rate:.1f} Hz")
        else:
            parts.append(f"{role} {rate:.1f} Hz")
    return passed, ", ".join(parts)


def _joint_set_check(view: MonitorView, expected_count: int) -> tuple[bool, str]:
    leader = set(view.stream("leader").joints)
    follower = set(view.stream("follower").joints)
    if not leader or not follower:
        return False, "waiting for joint names"
    if leader != follower:
        return False, (
            f"leader has {len(leader)}, follower has {len(follower)}; sets differ"
        )
    if len(leader) != expected_count:
        return False, f"{len(leader)} joints present, expected {expected_count}"
    return True, f"{expected_count} joints present in both streams"


def _tracking_check(view: MonitorView) -> tuple[bool, str]:
    if not view.tracking.available:
        return False, "tracking error unavailable"
    error = view.tracking.max_abs_error_rad
    if error is None:
        return False, "tracking error unavailable"
    return True, f"available (max {error * 1000.0:.1f} mrad)"
