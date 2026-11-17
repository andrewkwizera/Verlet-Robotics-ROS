"""Dashboard configuration.

``DEFAULTS`` is the single source of truth: the ROS node declares exactly these
parameter names, and ``config/dashboard.yaml`` overrides a subset of them.  The
launch file may override ``port`` / ``bind_address`` on top of the YAML.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

# Parameter name -> default value.  Dotted names map to nested YAML sections.
#
# The Python type here is authoritative: rclpy rejects a YAML override whose
# type differs (e.g. ``250`` for a DOUBLE parameter).  Keep config/dashboard.yaml
# in sync; ``test_telemetry.py`` asserts that it is.
DEFAULTS: dict[str, Any] = {
    "leader_topic": "/leader/joint_states",
    "follower_topic": "/follower/joint_states",
    "port": 8080,
    "bind_address": "127.0.0.1",
    "refresh_ms": 100,
    "freshness.warning_ms": 250.0,
    "freshness.stale_ms": 1000.0,
    "freshness.disconnect_ms": 3000.0,
    "diagnostics.max_error_warn_rad": 0.15,
    "diagnostics.rms_error_warn_rad": 0.10,
    "diagnostics.rate_min_hz": 30.0,
    "diagnostics.rate_max_hz": 70.0,
    "diagnostics.rate_window_s": 2.0,
    "diagnostics.static_joint_epsilon_rad": 0.001,
    "diagnostics.static_joint_min_samples": 50,
    "events.max_entries": 200,
    "webrtc.enabled": False,
}


@dataclass(frozen=True)
class DashboardConfig:
    leader_topic: str = DEFAULTS["leader_topic"]
    follower_topic: str = DEFAULTS["follower_topic"]
    port: int = DEFAULTS["port"]
    bind_address: str = DEFAULTS["bind_address"]
    refresh_ms: int = DEFAULTS["refresh_ms"]
    warning_s: float = DEFAULTS["freshness.warning_ms"] / 1000.0
    stale_s: float = DEFAULTS["freshness.stale_ms"] / 1000.0
    disconnect_s: float = DEFAULTS["freshness.disconnect_ms"] / 1000.0
    max_error_warn_rad: float = DEFAULTS["diagnostics.max_error_warn_rad"]
    rms_error_warn_rad: float = DEFAULTS["diagnostics.rms_error_warn_rad"]
    rate_min_hz: float = DEFAULTS["diagnostics.rate_min_hz"]
    rate_max_hz: float = DEFAULTS["diagnostics.rate_max_hz"]
    rate_window_s: float = DEFAULTS["diagnostics.rate_window_s"]
    static_joint_epsilon_rad: float = DEFAULTS["diagnostics.static_joint_epsilon_rad"]
    static_joint_min_samples: int = DEFAULTS["diagnostics.static_joint_min_samples"]
    max_events: int = DEFAULTS["events.max_entries"]
    webrtc_enabled: bool = DEFAULTS["webrtc.enabled"]

    @classmethod
    def from_parameter_dict(cls, values: Mapping[str, Any]) -> "DashboardConfig":
        """Build a validated config from flat ``dotted.name -> value`` pairs."""
        def get(name: str) -> Any:
            return values.get(name, DEFAULTS[name])

        refresh_ms = int(round(float(get("refresh_ms"))))
        if not 20 <= refresh_ms <= 1000:
            raise ValueError(
                f"refresh_ms must be between 20 and 1000, got {refresh_ms}"
            )

        port = int(get("port"))
        if not 1 <= port <= 65535:
            raise ValueError(f"port must be between 1 and 65535, got {port}")

        bind = str(get("bind_address")).strip()
        if not bind:
            raise ValueError("bind_address must not be empty")

        warning_s = float(get("freshness.warning_ms")) / 1000.0
        stale_s = float(get("freshness.stale_ms")) / 1000.0
        disconnect_s = float(get("freshness.disconnect_ms")) / 1000.0
        if not 0.0 < warning_s < stale_s < disconnect_s:
            raise ValueError(
                "freshness thresholds must satisfy 0 < warning_ms < stale_ms "
                f"< disconnect_ms, got {warning_s * 1000:.0f}/"
                f"{stale_s * 1000:.0f}/{disconnect_s * 1000:.0f} ms"
            )

        rate_min = float(get("diagnostics.rate_min_hz"))
        rate_max = float(get("diagnostics.rate_max_hz"))
        if rate_min <= 0.0 or rate_max <= rate_min:
            raise ValueError(
                "rate band must satisfy 0 < rate_min_hz < rate_max_hz, got "
                f"{rate_min}/{rate_max}"
            )

        return cls(
            leader_topic=str(get("leader_topic")),
            follower_topic=str(get("follower_topic")),
            port=port,
            bind_address=bind,
            refresh_ms=refresh_ms,
            warning_s=warning_s,
            stale_s=stale_s,
            disconnect_s=disconnect_s,
            max_error_warn_rad=float(get("diagnostics.max_error_warn_rad")),
            rms_error_warn_rad=float(get("diagnostics.rms_error_warn_rad")),
            rate_min_hz=rate_min,
            rate_max_hz=rate_max,
            rate_window_s=float(get("diagnostics.rate_window_s")),
            static_joint_epsilon_rad=float(
                get("diagnostics.static_joint_epsilon_rad")
            ),
            static_joint_min_samples=int(
                get("diagnostics.static_joint_min_samples")
            ),
            max_events=int(get("events.max_entries")),
            webrtc_enabled=bool(get("webrtc.enabled")),
        )
