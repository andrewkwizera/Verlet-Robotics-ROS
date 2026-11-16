"""Configuration for the supervision layer.

Kept separate from ``dashboard/config.py`` so Milestone 1's parameter surface
is unchanged.  ``config/supervision.yaml`` overrides these defaults and is
passed as a second parameter file by ``dashboard.launch.py``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

# Parameter name -> default value.  Dotted names map to nested YAML sections.
SUPERVISION_DEFAULTS: dict[str, Any] = {
    "supervision.startup_grace_s": 5.0,
    "supervision.ready_hold_s": 1.0,
    "supervision.reason_log_min_interval_s": 5.0,
    "supervision.health.reject_window_s": 5.0,
    "supervision.health.source_dt_warn_ms": 250.0,
    "supervision.safety.source": "none",
    "supervision.safety.timeout_ms": 2000.0,
    "supervision.safety.fault_clear_hold_s": 1.0,
    "supervision.readiness.expected_joint_count": 6,
    "supervision.readiness.alignment_diagnostic_rad": 0.10,
    "supervision.session.inactivity_s": 30.0,
}

# Only the honest default is accepted in Milestone 2.  A future hardware
# safety source must be added as an explicit adapter and reviewed; the
# dashboard must never invent Normal from silence or from the bridge string.
_SUPPORTED_SAFETY_SOURCES = {"none"}


@dataclass(frozen=True)
class SupervisionConfig:
    startup_grace_s: float = SUPERVISION_DEFAULTS["supervision.startup_grace_s"]
    ready_hold_s: float = SUPERVISION_DEFAULTS["supervision.ready_hold_s"]
    reason_log_min_interval_s: float = SUPERVISION_DEFAULTS[
        "supervision.reason_log_min_interval_s"
    ]
    reject_window_s: float = SUPERVISION_DEFAULTS[
        "supervision.health.reject_window_s"
    ]
    source_dt_warn_ms: float = SUPERVISION_DEFAULTS[
        "supervision.health.source_dt_warn_ms"
    ]
    safety_source: str = SUPERVISION_DEFAULTS["supervision.safety.source"]
    safety_timeout_s: float = (
        SUPERVISION_DEFAULTS["supervision.safety.timeout_ms"] / 1000.0
    )
    fault_clear_hold_s: float = SUPERVISION_DEFAULTS[
        "supervision.safety.fault_clear_hold_s"
    ]
    expected_joint_count: int = SUPERVISION_DEFAULTS[
        "supervision.readiness.expected_joint_count"
    ]
    alignment_diagnostic_rad: float = SUPERVISION_DEFAULTS[
        "supervision.readiness.alignment_diagnostic_rad"
    ]
    session_inactivity_s: float = SUPERVISION_DEFAULTS[
        "supervision.session.inactivity_s"
    ]

    @classmethod
    def from_parameter_dict(cls, values: Mapping[str, Any]) -> "SupervisionConfig":
        def get(name: str) -> Any:
            return values.get(name, SUPERVISION_DEFAULTS[name])

        startup_grace_s = float(get("supervision.startup_grace_s"))
        if startup_grace_s < 0.0:
            raise ValueError("startup_grace_s must be >= 0")

        ready_hold_s = float(get("supervision.ready_hold_s"))
        if not 0.0 < ready_hold_s <= 60.0:
            raise ValueError("ready_hold_s must satisfy 0 < ready_hold_s <= 60")

        reject_window_s = float(get("supervision.health.reject_window_s"))
        if reject_window_s < 0.0:
            raise ValueError("reject_window_s must be >= 0")

        source_dt_warn_ms = float(get("supervision.health.source_dt_warn_ms"))
        if source_dt_warn_ms <= 0.0:
            raise ValueError("source_dt_warn_ms must be > 0")

        source = str(get("supervision.safety.source")).strip().lower()
        if source not in _SUPPORTED_SAFETY_SOURCES:
            raise ValueError(
                "supervision.safety.source only supports 'none' in Milestone 2; "
                f"got {source!r}. Safety is never inferred from another topic."
            )

        safety_timeout_s = float(get("supervision.safety.timeout_ms")) / 1000.0
        if safety_timeout_s <= 0.0:
            raise ValueError("safety.timeout_ms must be > 0")

        fault_clear_hold_s = float(get("supervision.safety.fault_clear_hold_s"))
        if fault_clear_hold_s < 0.0:
            raise ValueError("fault_clear_hold_s must be >= 0")

        expected_joint_count = int(get("supervision.readiness.expected_joint_count"))
        if not 1 <= expected_joint_count <= 64:
            raise ValueError("expected_joint_count must be between 1 and 64")

        alignment = float(get("supervision.readiness.alignment_diagnostic_rad"))
        if alignment <= 0.0:
            raise ValueError("alignment_diagnostic_rad must be > 0")

        inactivity = float(get("supervision.session.inactivity_s"))
        if inactivity <= 0.0:
            raise ValueError("session.inactivity_s must be > 0")

        reason_log = float(get("supervision.reason_log_min_interval_s"))
        if reason_log < 0.0:
            raise ValueError("reason_log_min_interval_s must be >= 0")

        return cls(
            startup_grace_s=startup_grace_s,
            ready_hold_s=ready_hold_s,
            reason_log_min_interval_s=reason_log,
            reject_window_s=reject_window_s,
            source_dt_warn_ms=source_dt_warn_ms,
            safety_source=source,
            safety_timeout_s=safety_timeout_s,
            fault_clear_hold_s=fault_clear_hold_s,
            expected_joint_count=expected_joint_count,
            alignment_diagnostic_rad=alignment,
            session_inactivity_s=inactivity,
        )
