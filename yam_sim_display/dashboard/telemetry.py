"""Pure telemetry state for the YAM operator dashboard.

This module deliberately has **no ROS imports**.  Every timestamp is a plain
monotonic float supplied by the caller, which makes the whole state machine
deterministic and unit-testable without a running ROS graph.

Responsibilities kept here:
  * per-stream freshness / rate / counters
  * leader-follower tracking error
  * bounded, transition-only event log

Responsibilities deliberately kept OUT of here:
  * ROS subscriptions (see ``node.py``)
  * HTTP transport (see ``http_server.py``)
  * presentation / projection maths (see ``web/app.js``)
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from datetime import datetime, timezone
from typing import Deque, Dict, List, Mapping, Optional, Sequence

# --- freshness states -------------------------------------------------------

STATE_CONNECTED = "connected"
STATE_WARNING = "warning"
STATE_STALE = "stale"
STATE_DISCONNECTED = "disconnected"

SEVERITY = {
    STATE_CONNECTED: 0,
    STATE_WARNING: 1,
    STATE_STALE: 2,
    STATE_DISCONNECTED: 3,
}

STATE_LABELS = {
    STATE_CONNECTED: "Connected",
    STATE_WARNING: "Warning",
    STATE_STALE: "Stale",
    STATE_DISCONNECTED: "Disconnected",
}

# A warning clears at 80% of the threshold that raised it, so a signal sitting
# on the boundary cannot generate an event on every evaluation tick.
HYSTERESIS = 0.8


def worst_state(states: Sequence[str]) -> str:
    """Return the most severe freshness state in ``states``."""
    worst = STATE_CONNECTED
    for state in states:
        if SEVERITY.get(state, 0) > SEVERITY[worst]:
            worst = state
    return worst


def compute_rate(times: Sequence[float], now: float, window_s: float) -> Optional[float]:
    """Messages per second over a trailing window, or ``None`` if unknown.

    Fewer than two samples cannot define a rate; the caller must render that as
    "unavailable" rather than inventing a value.
    """
    if len(times) < 2 or window_s <= 0.0:
        return None

    cutoff = now - window_s
    recent = [t for t in times if t >= cutoff]
    if len(recent) < 2:
        recent = list(times)[-2:]

    span = recent[-1] - recent[0]
    if span <= 0.0:
        return None
    return (len(recent) - 1) / span


class JointStream:
    """Tracks one ``sensor_msgs/JointState`` stream.

    Not internally synchronised: ``TelemetryStore`` owns the lock.
    """

    def __init__(self, role: str, history: int = 800) -> None:
        self.role = role
        self._times: Deque[float] = deque(maxlen=history)
        self._positions: Dict[str, float] = {}
        self._order: List[str] = []
        self._min: Dict[str, float] = {}
        self._max: Dict[str, float] = {}

        self.count = 0
        self.rejected = 0
        self.last_arrival: Optional[float] = None
        self.first_arrival: Optional[float] = None
        self.source_stamp_ns: Optional[int] = None
        self.source_dt_ms: Optional[float] = None
        self.state = STATE_DISCONNECTED
        # True once this stream has been observed in the connected state.  A
        # DDS backlog can deliver several messages at once, so "count > 1" is
        # not a valid proxy for "was connected before".
        self.ever_connected = False

    # -- ingestion ----------------------------------------------------------

    def update(
        self,
        positions: Mapping[str, float],
        source_stamp_ns: Optional[int],
        now: float,
    ) -> bool:
        """Ingest one message. Returns ``False`` if the message was rejected."""
        if not positions:
            self.rejected += 1
            return False

        if not self._order:
            self._order = list(positions.keys())
        else:
            # Reject malformed frames once a canonical joint set is known;
            # silently mixing two joint layouts would corrupt the error math.
            if len(positions) != len(self._order) or any(
                name not in positions for name in self._order
            ):
                self.rejected += 1
                return False

        for name in self._order:
            value = positions[name]
            if not math.isfinite(value):
                self.rejected += 1
                return False
            self._positions[name] = value
            if name in self._min:
                if value < self._min[name]:
                    self._min[name] = value
                if value > self._max[name]:
                    self._max[name] = value
            else:
                self._min[name] = value
                self._max[name] = value

        self._times.append(now)
        if self.first_arrival is None:
            self.first_arrival = now
        self.last_arrival = now
        self.count += 1

        if source_stamp_ns is not None:
            # Guard against non-monotonic stamps: keep the newest observed
            # stamp and only derive a delta when time actually advances.
            if self.source_stamp_ns is None or source_stamp_ns > self.source_stamp_ns:
                if self.source_stamp_ns is not None:
                    self.source_dt_ms = (
                        source_stamp_ns - self.source_stamp_ns
                    ) / 1e6
                self.source_stamp_ns = source_stamp_ns

        return True

    # -- derived state ------------------------------------------------------

    def update_state(self, now: float, warning_s: float, stale_s: float,
                     disconnect_s: float) -> str:
        if self.count == 0 or self.last_arrival is None:
            self.state = STATE_DISCONNECTED
            return self.state

        age = max(0.0, now - self.last_arrival)
        if age >= disconnect_s:
            self.state = STATE_DISCONNECTED
        elif age >= stale_s:
            self.state = STATE_STALE
        elif age >= warning_s:
            self.state = STATE_WARNING
        else:
            self.state = STATE_CONNECTED
        return self.state

    def age_ms(self, now: float) -> Optional[float]:
        if self.last_arrival is None:
            return None
        return max(0.0, (now - self.last_arrival) * 1000.0)

    def static_joints(self, epsilon_rad: float, min_samples: int) -> List[str]:
        """Joints whose commanded value has not moved during this session.

        Used to flag joints that are structurally constant in the current
        simulation, so their near-zero tracking error is not misread as
        evidence of tracking quality.
        """
        if self.count < min_samples:
            return []
        return [
            name for name in self._order
            if (self._max[name] - self._min[name]) <= epsilon_rad
        ]

    def positions(self) -> Dict[str, float]:
        return dict(self._positions)

    def snapshot(self, now: float, rate_window_s: float) -> dict:
        return {
            "state": self.state,
            "state_label": STATE_LABELS.get(self.state, self.state),
            "age_ms": self.age_ms(now),
            "rate_hz": compute_rate(self._times, now, rate_window_s),
            "count": self.count,
            "rejected": self.rejected,
            "source_stamp_ns": self.source_stamp_ns,
            "source_dt_ms": self.source_dt_ms,
            "joint_order": list(self._order),
            "joints": dict(self._positions),
        }


def compute_tracking(
    leader: Mapping[str, float],
    follower: Mapping[str, float],
    static_joints: Sequence[str] = (),
) -> Optional[dict]:
    """Per-joint leader/follower error plus aggregate statistics."""
    names = [name for name in leader if name in follower]
    if not names:
        return None

    static = set(static_joints)
    rows: List[dict] = []
    squared = 0.0
    max_abs = -1.0
    worst: Optional[str] = None

    for name in names:
        error = leader[name] - follower[name]
        squared += error * error
        if abs(error) > max_abs:
            max_abs = abs(error)
            worst = name
        rows.append({
            "name": name,
            "leader": leader[name],
            "follower": follower[name],
            "error_rad": error,
            "error_mrad": error * 1000.0,
            "static": name in static,
        })

    return {
        "available": True,
        "per_joint": rows,
        "joint_count": len(names),
        "max_abs_error_rad": max_abs,
        "max_abs_error_joint": worst,
        "rms_error_rad": math.sqrt(squared / len(names)),
        "static_joints": sorted(static),
    }


class EventLog:
    """Bounded ring buffer of operator-facing events."""

    def __init__(self, max_entries: int = 200) -> None:
        self._events: Deque[dict] = deque(maxlen=max(1, int(max_entries)))
        self._next_id = 1

    def add(self, level: str, source: str, message: str,
            wall_iso: Optional[str] = None) -> None:
        self._events.append({
            "id": self._next_id,
            "time": wall_iso or _wall_iso(),
            "level": level,
            "source": source,
            "message": message,
        })
        self._next_id += 1

    def snapshot(self) -> List[dict]:
        return list(self._events)

    def __len__(self) -> int:
        return len(self._events)


def _wall_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


class TelemetryStore:
    """Thread-safe aggregate of all dashboard telemetry.

    ROS callbacks call :meth:`on_joint_state`; a node timer calls
    :meth:`evaluate`; HTTP request threads call :meth:`snapshot`.  Only
    ``evaluate`` mutates derived state, so event emission is independent of
    browser polling frequency and cannot be multiplied by extra clients.
    """

    def __init__(self, config, clock=time.monotonic) -> None:
        self.config = config
        self.leader = JointStream("leader")
        self.follower = JointStream("follower")
        self.events = EventLog(config.max_events)
        self._clock = clock
        self._lock = threading.RLock()
        self._snapshot: Optional[dict] = None
        self._started = clock()

        self._tracking_alert = False
        self._rate_alert = {"leader": False, "follower": False}
        self._last_evaluated_state = {
            "leader": STATE_DISCONNECTED,
            "follower": STATE_DISCONNECTED,
        }

    # -- ingestion ----------------------------------------------------------

    def on_joint_state(
        self,
        role: str,
        names: Sequence[str],
        positions: Sequence[float],
        source_stamp_ns: Optional[int] = None,
        now: Optional[float] = None,
    ) -> bool:
        payload = dict(zip(names, positions)) if names else {}
        moment = self._clock() if now is None else now
        with self._lock:
            stream = self.leader if role == "leader" else self.follower
            return stream.update(payload, source_stamp_ns, moment)

    # -- evaluation ---------------------------------------------------------

    def evaluate(self, now: Optional[float] = None) -> dict:
        moment = self._clock() if now is None else now
        with self._lock:
            self._refresh_states(moment)
            self._emit_threshold_events(moment)
            self._snapshot = self._build_snapshot(moment)
            return self._snapshot

    def snapshot(self) -> Optional[dict]:
        with self._lock:
            return self._snapshot

    # -- internals ----------------------------------------------------------

    def _refresh_states(self, now: float) -> None:
        cfg = self.config
        for stream in (self.leader, self.follower):
            previous = stream.state
            current = stream.update_state(
                now, cfg.warning_s, cfg.stale_s, cfg.disconnect_s
            )
            if current != previous:
                self._log_state_change(stream, previous, current)

    def _log_state_change(self, stream: JointStream, previous: str, current: str) -> None:
        age = stream.age_ms(self._clock())
        age_text = "no message received yet" if age is None else f"age {age:.0f} ms"

        if current == STATE_CONNECTED:
            if previous == STATE_DISCONNECTED:
                if not stream.ever_connected:
                    # First contact.  A DDS backlog can make count > 1 before
                    # this is ever logged, so the flag decides, not the count.
                    self.events.add(
                        "info", stream.role,
                        f"{stream.role.capitalize()} telemetry connected ({age_text})",
                    )
                else:
                    self.events.add(
                        "info", stream.role,
                        f"{stream.role.capitalize()} telemetry recovered after "
                        f"disconnect ({age_text})",
                    )
            else:
                self.events.add(
                    "info", stream.role,
                    f"{stream.role.capitalize()} telemetry recovered from "
                    f"{STATE_LABELS.get(previous, previous).lower()} ({age_text})",
                )
            stream.ever_connected = True
            return

        # Before the first successful connection the stream simply has no data;
        # an age-band transition there is startup noise, not an operator event.
        if not stream.ever_connected:
            return

        if current == STATE_WARNING:
            # Deliberately "message age", never "latency": no latency probe
            # exists, so the two must not be conflated.
            self.events.add(
                "warning", stream.role,
                f"{stream.role.capitalize()} telemetry message age above warning "
                f"threshold ({age_text}, limit {self.config.warning_s * 1000:.0f} ms)",
            )
        elif current == STATE_STALE:
            self.events.add(
                "warning", stream.role,
                f"{stream.role.capitalize()} telemetry is stale ({age_text}, "
                f"limit {self.config.stale_s * 1000:.0f} ms)",
            )
        elif current == STATE_DISCONNECTED:
            self.events.add(
                "error", stream.role,
                f"{stream.role.capitalize()} telemetry disconnected ({age_text}, "
                f"limit {self.config.disconnect_s * 1000:.0f} ms)",
            )

    def _emit_threshold_events(self, now: float) -> None:
        cfg = self.config

        # Tracking error: hysteresis prevents chatter around the threshold.
        # Only comparable while both streams are fresh enough, so a frozen
        # pose can never raise or clear a tracking event.
        tracking = None
        if (
            self.leader.count > 0
            and self.follower.count > 0
            and STATE_SEVERITY_OK(self.leader.state)
            and STATE_SEVERITY_OK(self.follower.state)
        ):
            tracking = self._tracking_snapshot()

        if tracking is None:
            # Re-arm silently: if the error is still out of bounds when fresh
            # telemetry returns, it is reported as a new crossing.
            self._tracking_alert = False
        else:
            exceeded = tracking["max_abs_error_rad"] >= cfg.max_error_warn_rad
            if exceeded and not self._tracking_alert:
                self.events.add(
                    "warning", "tracking",
                    f"Tracking error {tracking['max_abs_error_rad'] * 1000:.1f} mrad "
                    f"on {tracking['max_abs_error_joint']} exceeds "
                    f"{cfg.max_error_warn_rad * 1000:.0f} mrad",
                )
                self._tracking_alert = True
            elif (
                not exceeded
                and self._tracking_alert
                and tracking["max_abs_error_rad"] < cfg.max_error_warn_rad * HYSTERESIS
            ):
                self.events.add(
                    "info", "tracking",
                    f"Tracking error back within limits "
                    f"({tracking['max_abs_error_rad'] * 1000:.1f} mrad)",
                )
                self._tracking_alert = False
            # Between the clear threshold and the raise threshold the alert
            # state is held, which is what makes the hysteresis effective.

        # Stream rate band.  Only meaningful for a live, fresh stream and once
        # a full observation window has elapsed: a DDS backlog can replay its
        # history in a few milliseconds, which would otherwise look like a huge
        # rate spike at startup.  A stalled stream is already reported by the
        # freshness events, so it must not also raise a rate event.
        for stream in (self.leader, self.follower):
            if stream.state != STATE_CONNECTED:
                continue
            if stream.count < 10 or stream.first_arrival is None:
                continue
            if now - stream.first_arrival < cfg.rate_window_s:
                continue
            rate = compute_rate(stream._times, now, cfg.rate_window_s)  # noqa: SLF001
            if rate is None:
                continue
            out_of_band = rate < cfg.rate_min_hz or rate > cfg.rate_max_hz
            previous = self._rate_alert[stream.role]
            if out_of_band and not previous:
                self.events.add(
                    "warning", stream.role,
                    f"{stream.role.capitalize()} publish rate {rate:.1f} Hz outside "
                    f"configured {cfg.rate_min_hz:.0f}-{cfg.rate_max_hz:.0f} Hz band",
                )
                self._rate_alert[stream.role] = True
            elif not out_of_band and previous:
                self.events.add(
                    "info", stream.role,
                    f"{stream.role.capitalize()} publish rate back at {rate:.1f} Hz",
                )
                self._rate_alert[stream.role] = False

    def _tracking_snapshot(self) -> Optional[dict]:
        cfg = self.config
        static = self.leader.static_joints(
            cfg.static_joint_epsilon_rad, cfg.static_joint_min_samples
        )
        return compute_tracking(
            self.leader.positions(), self.follower.positions(), static
        )

    def _build_snapshot(self, now: float) -> dict:
        cfg = self.config
        leader = self.leader.snapshot(now, cfg.rate_window_s)
        follower = self.follower.snapshot(now, cfg.rate_window_s)

        # Only report tracking while both sides have data; a one-sided
        # difference against an empty pose would be meaningless.
        fresh_enough = (
            self.leader.count > 0
            and self.follower.count > 0
            and STATE_SEVERITY_OK(self.leader.state)
            and STATE_SEVERITY_OK(self.follower.state)
        )
        tracking = self._tracking_snapshot() if fresh_enough else None
        if tracking is not None:
            tracking["note"] = self._static_note(tracking["static_joints"])
        else:
            tracking = {
                "available": False,
                "per_joint": [],
                "static_joints": [],
                "note": "Tracking error requires fresh leader and follower telemetry.",
            }

        system_state = worst_state([self.leader.state, self.follower.state])
        return {
            "schema": 1,
            "generated_at": _wall_iso(),
            "uptime_s": max(0.0, now - self._started),
            "config": {
                "refresh_ms": cfg.refresh_ms,
                "leader_topic": cfg.leader_topic,
                "follower_topic": cfg.follower_topic,
                "warning_ms": cfg.warning_s * 1000.0,
                "stale_ms": cfg.stale_s * 1000.0,
                "disconnect_ms": cfg.disconnect_s * 1000.0,
                "max_error_warn_rad": cfg.max_error_warn_rad,
                "rms_error_warn_rad": cfg.rms_error_warn_rad,
                "rate_min_hz": cfg.rate_min_hz,
                "rate_max_hz": cfg.rate_max_hz,
            },
            "system": {
                "state": system_state,
                "state_label": STATE_LABELS[system_state],
                "label": _system_label(system_state),
                "detail": _system_detail(self.leader, self.follower),
            },
            "streams": {"leader": leader, "follower": follower},
            "tracking": tracking,
            "latency": {
                "available": False,
                "display": "Unavailable \u2014 no valid latency measurement source",
                "reason": (
                    "No round-trip latency probe or cross-host clock "
                    "synchronisation is present. Message age and publish rate "
                    "are shown instead and must not be read as network latency."
                ),
            },
            "camera": {
                "state": "pending",
                "title": "Camera feed pending",
                "reason": (
                    "The GStreamer camera sender provides native RTP on leader ports "
                    "5000/5002 and local WebRTC gateway RTP on follower ports "
                    "5100/5102. Enable WebRTC signaling and start the gateway "
                    "to show video here."
                ),
                "stream_url": None,
            },
            "events": self.events.snapshot(),
        }

    @staticmethod
    def _static_note(static_joints: Sequence[str]) -> str:
        if not static_joints:
            return ""
        names = ", ".join(j.replace("_", " ") for j in static_joints)
        return (
            f"{names.capitalize()} are static in the current simulation. "
            "Their near-zero error does not demonstrate tracking performance."
        )


def STATE_SEVERITY_OK(state: str) -> bool:
    """True when a stream is fresh enough to be compared."""
    return SEVERITY.get(state, 3) <= SEVERITY[STATE_WARNING]


def _system_label(state: str) -> str:
    return {
        STATE_CONNECTED: "Teleoperation link nominal",
        STATE_WARNING: "Telemetry delayed",
        STATE_STALE: "Telemetry stale",
        STATE_DISCONNECTED: "Telemetry unavailable",
    }.get(state, state)


def _system_detail(leader: JointStream, follower: JointStream) -> str:
    missing = [
        stream.role for stream in (leader, follower) if stream.count == 0
    ]
    if missing:
        return "Waiting for: " + ", ".join(missing)
    return "Both joint-state streams reporting"
