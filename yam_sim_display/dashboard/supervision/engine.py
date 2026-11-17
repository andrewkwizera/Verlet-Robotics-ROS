"""Supervision engine: combines the four independent dimensions.

The engine is pure Python.  It is fed the Milestone 1 snapshot and an optional
safety input, and returns a JSON-ready ``supervision`` block.  It never
publishes ROS messages, never subscribes to command topics and never touches
the telemetry store's internals.
"""

from __future__ import annotations

import time
from datetime import datetime, timezone
from typing import Dict, Optional, Tuple

from .health import HealthEvaluator
from .monitor_view import MonitorView
from .readiness import ReadinessEvaluator
from .reasons import ReasonTracker
from .safety import NullSafetyInput, SafetyEvaluator, SafetyInput
from .states import (
    DIMENSIONS,
    EVENT_LEVEL,
    LABELS,
    SESSION_ACTIONS,
    SESSION_STATES,
    SESSION_TRANSITIONS,
    SEVERITY_RANK,
)


def _wall_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


class Supervisor:
    def __init__(
        self,
        config,
        telemetry_config,
        event_log,
        safety_input: Optional[SafetyInput] = None,
        clock=time.monotonic,
        wall=_wall_iso,
    ) -> None:
        self.config = config
        self.telemetry_config = telemetry_config
        self.events = event_log
        self.clock = clock
        self.wall = wall

        self.health = HealthEvaluator(config, telemetry_config)
        self.readiness = ReadinessEvaluator(config)
        self.safety = SafetyEvaluator(config, safety_input or NullSafetyInput())

        self._trackers = {name: ReasonTracker() for name in DIMENSIONS}
        self._state: Dict[str, Optional[str]] = {name: None for name in DIMENSIONS}
        self._since: Dict[str, str] = {}
        self._since_uptime: Dict[str, float] = {}
        self._logged_episode: Dict[Tuple[str, str], int] = {}
        self._last_reason_log: Dict[Tuple[str, str], float] = {}

    # -- public API ---------------------------------------------------------

    def evaluate(self, snapshot, now: Optional[float] = None) -> dict:
        moment = self.clock() if now is None else now
        wall_iso = self.wall()
        view = MonitorView.from_snapshot(snapshot)

        safety_result = self.safety.evaluate(moment)
        health_result = self.health.evaluate(view, moment)
        readiness_result = self.readiness.evaluate(
            view, safety_result.state, moment, self.telemetry_config
        )

        results = {
            "health": health_result,
            "readiness": readiness_result,
            "safety": safety_result,
        }

        payload = {
            "version": 1,
            "generated_at": wall_iso,
            "health": None,
            "readiness": None,
            "safety": None,
            "session": self._session_block(),
            "config": self._config_block(),
            "available": view.available,
        }

        for name in ("health", "readiness", "safety"):
            result = results[name]
            tracked = self._trackers[name].update(
                result.reasons, wall_iso, moment
            )
            self._record_state(name, result.state, wall_iso, moment, tracked)
            payload[name] = {
                "state": result.state,
                "label": LABELS.get(result.state, result.state),
                "since": self._since[name],
                "since_uptime_s": round(moment - self._since_uptime[name], 3),
                "reasons": tracked,
                "evidence": result.evidence,
                **result.extra,
            }

        # The session dimension is owned by the browser.  Publish its
        # vocabulary and canonical transition table, but never a fabricated
        # server-side session state.
        return payload

    # -- state bookkeeping --------------------------------------------------

    def _record_state(
        self,
        dimension: str,
        state: str,
        wall_iso: str,
        now: float,
        reasons: list,
    ) -> None:
        previous = self._state[dimension]
        self._state[dimension] = state
        if previous is None:
            self._since[dimension] = wall_iso
            self._since_uptime[dimension] = now
            # Initial state is not a transition; keep startup quiet like M1.
            self._mark_all_logged(dimension, reasons)
            return

        if previous == state:
            self._log_new_reasons(dimension, reasons, now)
            return

        self._since[dimension] = wall_iso
        self._since_uptime[dimension] = now

        top = reasons[0] if reasons else None
        detail = f" — {top['code']}: {top['message']}" if top else ""
        level = _worst_level(reasons)
        self.events.add(
            level,
            f"supervision.{dimension}",
            f"{dimension.capitalize()}: "
            f"{LABELS.get(previous, previous)} → {LABELS.get(state, state)}{detail}",
        )
        self._mark_all_logged(dimension, reasons)

    def _log_new_reasons(self, dimension: str, reasons: list, now: float) -> None:
        for reason in reasons:
            key = (dimension, reason["code"])
            episode = int(reason.get("count") or 0)
            if episode <= self._logged_episode.get(key, 0):
                continue
            last = self._last_reason_log.get(key)
            if (
                last is not None
                and now - last < self.config.reason_log_min_interval_s
            ):
                continue
            self._logged_episode[key] = episode
            self._last_reason_log[key] = now
            self.events.add(
                _worst_level([reason]),
                f"supervision.{dimension}",
                f"{dimension.capitalize()} reason {reason['code']}: "
                f"{reason['message']}",
            )

    def _mark_all_logged(self, dimension: str, reasons: list) -> None:
        for reason in reasons:
            self._logged_episode[(dimension, reason["code"])] = int(
                reason.get("count") or 0
            )

    # -- static blocks ------------------------------------------------------

    def _session_block(self) -> dict:
        return {
            "managed_by": "client",
            "states": list(SESSION_STATES),
            "transitions": {
                state: list(targets)
                for state, targets in SESSION_TRANSITIONS.items()
            },
            "actions": {
                state: list(actions)
                for state, actions in SESSION_ACTIONS.items()
            },
            "inactivity_s": self.config.session_inactivity_s,
            "note": (
                "Session state is owned by this browser and never affects the "
                "robot. No POST endpoint exists."
            ),
        }

    def _config_block(self) -> dict:
        return {
            "startup_grace_s": self.config.startup_grace_s,
            "ready_hold_s": self.config.ready_hold_s,
            "safety_source": self.config.safety_source,
            "safety_timeout_s": self.config.safety_timeout_s,
            "fault_clear_hold_s": self.config.fault_clear_hold_s,
            "expected_joint_count": self.config.expected_joint_count,
            "alignment_diagnostic_rad": self.config.alignment_diagnostic_rad,
            "session_inactivity_s": self.config.session_inactivity_s,
        }


def _worst_level(reasons: list) -> str:
    if not reasons:
        return "info"
    worst = max(
        reasons,
        key=lambda item: SEVERITY_RANK.get(item.get("severity", "info"), 0),
    )
    return EVENT_LEVEL.get(worst.get("severity", "info"), "info")
