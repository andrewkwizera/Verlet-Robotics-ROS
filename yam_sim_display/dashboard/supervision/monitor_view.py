"""Typed read-only adapter over the Milestone 1 telemetry snapshot.

The supervision layer must not reach into the raw telemetry dictionary in a
dozen places, and must not import ROS.  ``MonitorView`` is the only bridge
between the two layers; ``telemetry.py`` itself stays untouched.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Mapping, Optional, Sequence, Tuple

from ..telemetry import STATE_CONNECTED, STATE_DISCONNECTED


def _number(value) -> Optional[float]:
    try:
        if value is None:
            return None
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result


@dataclass(frozen=True)
class StreamView:
    role: str
    state: str = STATE_DISCONNECTED
    state_label: str = "Disconnected"
    age_ms: Optional[float] = None
    rate_hz: Optional[float] = None
    count: int = 0
    rejected: int = 0
    source_dt_ms: Optional[float] = None
    joint_order: Tuple[str, ...] = ()
    joints: Mapping[str, float] = field(default_factory=dict)

    @property
    def connected(self) -> bool:
        return self.state == STATE_CONNECTED

    @property
    def fresh_enough(self) -> bool:
        # Mirror telemetry.STATE_SEVERITY_OK without importing private state.
        return self.state in (STATE_CONNECTED, "warning")

    @classmethod
    def from_mapping(cls, role: str, data: Optional[Mapping]) -> "StreamView":
        if not isinstance(data, Mapping):
            return cls(role=role)
        joints = data.get("joints")
        order = data.get("joint_order")
        return cls(
            role=role,
            state=str(data.get("state") or STATE_DISCONNECTED),
            state_label=str(data.get("state_label") or "Disconnected"),
            age_ms=_number(data.get("age_ms")),
            rate_hz=_number(data.get("rate_hz")),
            count=int(data.get("count") or 0),
            rejected=int(data.get("rejected") or 0),
            source_dt_ms=_number(data.get("source_dt_ms")),
            joint_order=tuple(order) if isinstance(order, (list, tuple)) else (),
            joints=dict(joints) if isinstance(joints, Mapping) else {},
        )


@dataclass(frozen=True)
class TrackingView:
    available: bool = False
    max_abs_error_rad: Optional[float] = None
    max_abs_error_joint: Optional[str] = None
    rms_error_rad: Optional[float] = None
    static_joints: Tuple[str, ...] = ()

    @classmethod
    def from_mapping(cls, data: Optional[Mapping]) -> "TrackingView":
        if not isinstance(data, Mapping):
            return cls()
        static = data.get("static_joints")
        return cls(
            available=bool(data.get("available")),
            max_abs_error_rad=_number(data.get("max_abs_error_rad")),
            max_abs_error_joint=data.get("max_abs_error_joint"),
            rms_error_rad=_number(data.get("rms_error_rad")),
            static_joints=tuple(static) if isinstance(static, (list, tuple)) else (),
        )


@dataclass(frozen=True)
class MonitorView:
    available: bool = False
    streams: Mapping[str, StreamView] = field(default_factory=dict)
    tracking: TrackingView = field(default_factory=TrackingView)
    config: Mapping = field(default_factory=dict)
    raw_config: Mapping = field(default_factory=dict)

    def stream(self, role: str) -> StreamView:
        return self.streams.get(role) or StreamView(role=role)

    @classmethod
    def empty(cls) -> "MonitorView":
        return cls(
            available=False,
            streams={
                "leader": StreamView("leader"),
                "follower": StreamView("follower"),
            },
        )

    @classmethod
    def from_snapshot(cls, snapshot: Optional[Mapping]) -> "MonitorView":
        if not isinstance(snapshot, Mapping):
            return cls.empty()
        streams = snapshot.get("streams")
        streams = streams if isinstance(streams, Mapping) else {}
        config = snapshot.get("config")
        return cls(
            available=True,
            streams={
                "leader": StreamView.from_mapping("leader", streams.get("leader")),
                "follower": StreamView.from_mapping(
                    "follower", streams.get("follower")
                ),
            },
            tracking=TrackingView.from_mapping(snapshot.get("tracking")),
            config=dict(config) if isinstance(config, Mapping) else {},
            raw_config=dict(config) if isinstance(config, Mapping) else {},
        )


def joint_set(mapping: Mapping[str, float]) -> Sequence[str]:
    return tuple(sorted(mapping.keys()))
