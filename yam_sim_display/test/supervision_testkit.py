"""Shared deterministic helpers for the supervision tests (no ROS)."""

from __future__ import annotations

import os
import sys
from typing import Optional

_PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, _PACKAGE_ROOT)

from dashboard.config import DEFAULTS, DashboardConfig  # noqa: E402
from dashboard.supervision import (  # noqa: E402
    SUPERVISION_DEFAULTS,
    SupervisionConfig,
    Supervisor,
)
from dashboard.supervision.safety import SafetyReading  # noqa: E402
from dashboard.telemetry import EventLog  # noqa: E402

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
WALL = "2026-01-01T00:00:00+00:00"


class FakeClock:
    def __init__(self, start: float = 1000.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class FakeSafetyInput:
    configured = True
    source = "test-safety"

    def __init__(self, reading: Optional[SafetyReading] = None) -> None:
        self.reading = reading

    def set(self, reading: Optional[SafetyReading]) -> None:
        self.reading = reading

    def read(self) -> Optional[SafetyReading]:
        return self.reading


def normal_reading(clock: FakeClock) -> SafetyReading:
    return SafetyReading(
        state="normal", detail="test input normal",
        source="test-safety", received_at=clock.now,
    )


def make_snapshot(
    *,
    leader_state: str = "connected",
    follower_state: str = "connected",
    leader_rate: Optional[float] = 50.0,
    follower_rate: Optional[float] = 50.0,
    leader_count: int = 100,
    follower_count: int = 100,
    leader_rejected: int = 0,
    follower_rejected: int = 0,
    leader_dt: Optional[float] = 20.0,
    follower_dt: Optional[float] = 20.0,
    leader_age: Optional[float] = 10.0,
    follower_age: Optional[float] = 10.0,
    tracking: bool = True,
    max_error: float = 0.02,
    joint_count: int = 6,
    leader_joint_count: Optional[int] = None,
    follower_joint_count: Optional[int] = None,
) -> dict:
    leader_joint_count = (
        joint_count if leader_joint_count is None else leader_joint_count
    )
    follower_joint_count = (
        joint_count if follower_joint_count is None else follower_joint_count
    )
    leader_names = JOINTS[:leader_joint_count]
    follower_names = JOINTS[:follower_joint_count]
    leader_joints = {name: 0.0 for name in leader_names}
    follower_joints = {name: 0.0 for name in follower_names}

    def stream(role, state, rate, count, rejected, dt, age, names, joints):
        return {
            "state": state,
            "state_label": state.capitalize(),
            "age_ms": age,
            "rate_hz": rate,
            "count": count,
            "rejected": rejected,
            "source_stamp_ns": 1,
            "source_dt_ms": dt,
            "joint_order": names,
            "joints": joints,
        }

    tracking_block = {
        "available": tracking,
        "per_joint": [],
        "joint_count": joint_count if tracking else 0,
        "max_abs_error_rad": max_error if tracking else None,
        "max_abs_error_joint": "joint_1" if tracking else None,
        "rms_error_rad": max_error if tracking else None,
        "static_joints": [],
        "note": "",
    }

    return {
        "schema": 1,
        "generated_at": WALL,
        "uptime_s": 1.0,
        "config": {
            "refresh_ms": 100,
            "leader_topic": "/leader/joint_states",
            "follower_topic": "/follower/joint_states",
            "warning_ms": 250.0,
            "stale_ms": 1000.0,
            "disconnect_ms": 3000.0,
            "max_error_warn_rad": 0.15,
            "rms_error_warn_rad": 0.10,
            "rate_min_hz": 30.0,
            "rate_max_hz": 70.0,
        },
        "system": {
            "state": "connected",
            "state_label": "Connected",
            "label": "Teleoperation link nominal",
            "detail": "Both joint-state streams reporting",
        },
        "streams": {
            "leader": stream(
                "leader", leader_state, leader_rate, leader_count,
                leader_rejected, leader_dt, leader_age,
                leader_names, leader_joints,
            ),
            "follower": stream(
                "follower", follower_state, follower_rate, follower_count,
                follower_rejected, follower_dt, follower_age,
                follower_names, follower_joints,
            ),
        },
        "tracking": tracking_block,
        "latency": {"available": False, "display": "Unavailable", "reason": ""},
        "camera": {"state": "pending", "title": "", "reason": "", "stream_url": None},
        "events": [],
    }


def make_engine(
    clock: Optional[FakeClock] = None,
    *,
    safety_input=None,
    overrides: Optional[dict] = None,
):
    clock = clock or FakeClock()
    values = dict(SUPERVISION_DEFAULTS)
    values.update(overrides or {})
    config = SupervisionConfig.from_parameter_dict(values)
    telemetry = DashboardConfig.from_parameter_dict(dict(DEFAULTS))
    events = EventLog(200)
    supervisor = Supervisor(
        config,
        telemetry,
        events,
        safety_input=safety_input,
        clock=clock,
        wall=lambda: WALL,
    )
    return supervisor, events, clock
