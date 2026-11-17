"""Safety-status dimension.

Safety is input-driven and fail-safe by construction:

* no configured source            -> Unknown
* configured source, no reading   -> Unknown
* reading older than the timeout  -> Unknown (or Fault if a fault was latched)
* only a fresh explicit Normal    -> Normal

Telemetry health, readiness and ``/yam_teleop/status`` are never used as proof
of safety.  In Milestone 2 the shipped source is ``none``, so the state is
Unknown for the entire simulation by design.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol, runtime_checkable

from .reasons import Reason
from .results import DimensionResult
from .states import (
    SAFETY_FAULT,
    SAFETY_NORMAL,
    SAFETY_UNKNOWN,
    SAFETY_WARNING,
    SEVERITY_ERROR,
    SEVERITY_WARNING,
)

SAFETY_EXPLANATION = (
    "Safety status is supplied by an independent safety system. The dashboard "
    "does not infer safety from telemetry, silence or the teleoperation bridge."
)
REQUIRED_INPUT = (
    "A structured reading from an independent safety source (for example a "
    "safety PLC, relay state or drive fault bits). None is configured."
)


@dataclass(frozen=True)
class SafetyReading:
    state: str
    detail: str = ""
    source: str = "external"
    received_at: Optional[float] = None


@runtime_checkable
class SafetyInput(Protocol):
    configured: bool
    source: str

    def read(self) -> Optional[SafetyReading]:
        ...


class NullSafetyInput:
    """The only input shipped in Milestone 2: deliberately no data."""

    configured = False
    source = "none"

    def read(self) -> Optional[SafetyReading]:
        return None


class SafetyEvaluator:
    def __init__(self, config, safety_input: Optional[SafetyInput] = None) -> None:
        self.config = config
        self.input = safety_input or NullSafetyInput()
        self._fault_latched = False
        self._fault_detail = ""
        self._ok_since: Optional[float] = None

    def evaluate(self, now: float) -> DimensionResult:
        result = self._evaluate(now)
        result.extra.update({
            "source": self.input.source,
            "latch": self._fault_latched,
            "explanation": SAFETY_EXPLANATION,
            "required_input": REQUIRED_INPUT,
        })
        return result

    # -- internals ----------------------------------------------------------

    def _evaluate(self, now: float) -> DimensionResult:
        if not self.input.configured:
            return self._unknown("safety.no_source", "No safety input is configured")

        reading = self.input.read()
        if reading is None:
            if self._fault_latched:
                return DimensionResult(
                    state=SAFETY_FAULT,
                    reasons=[Reason(
                        code="safety.fault_latched",
                        severity=SEVERITY_ERROR,
                        message=(
                            "Fault latched; safety input is absent, so only a "
                            "fresh explicit Normal can clear it"
                        ),
                        source=self.input.source,
                    )],
                    evidence={"latch": True},
                )
            return self._unknown(
                "safety.awaiting_input",
                f"Safety source '{self.input.source}' has not produced a reading",
            )

        age = None
        if reading.received_at is not None:
            age = max(0.0, now - reading.received_at)
        if age is not None and age > self.config.safety_timeout_s:
            if self._fault_latched:
                return DimensionResult(
                    state=SAFETY_FAULT,
                    reasons=[Reason(
                        code="safety.input_stale_after_fault",
                        severity=SEVERITY_ERROR,
                        message=(
                            f"Safety input stale for {age:.2f} s after a fault; "
                            "fault remains latched"
                        ),
                        source=self.input.source,
                        evidence={
                            "age_s": round(age, 3),
                            "timeout_s": self.config.safety_timeout_s,
                        },
                    )],
                    evidence={"age_s": round(age, 3), "latch": True},
                )
            return self._unknown(
                "safety.input_stale",
                f"Safety input is {age:.2f} s old (limit "
                f"{self.config.safety_timeout_s:.2f} s)",
            )

        state = reading.state
        detail = reading.detail or ""

        if state == SAFETY_FAULT:
            self._fault_latched = True
            self._fault_detail = detail or "safety source reported a fault"
            self._ok_since = None
            return DimensionResult(
                state=SAFETY_FAULT,
                reasons=[Reason(
                    code="safety.fault",
                    severity=SEVERITY_ERROR,
                    message=self._fault_detail,
                    source=self.input.source,
                )],
                evidence={"latch": True},
            )

        if self._fault_latched:
            if state == SAFETY_NORMAL:
                self._ok_since = now if self._ok_since is None else self._ok_since
                held = now - self._ok_since
                if held >= self.config.fault_clear_hold_s:
                    self._fault_latched = False
                    self._fault_detail = ""
                    self._ok_since = None
                    return DimensionResult(
                        state=SAFETY_NORMAL,
                        reasons=[],
                        evidence={"age_s": _round(age), "latch": False},
                    )
                return DimensionResult(
                    state=SAFETY_FAULT,
                    reasons=[Reason(
                        code="safety.fault_latched",
                        severity=SEVERITY_ERROR,
                        message=(
                            "Fault latched; clearing requires fresh Normal for "
                            f"{self.config.fault_clear_hold_s:.1f} s "
                            f"({held:.2f} s elapsed)"
                        ),
                        source=self.input.source,
                        evidence={
                            "clear_hold_s": self.config.fault_clear_hold_s,
                            "held_s": round(held, 3),
                        },
                    )],
                    evidence={"latch": True},
                )
            self._ok_since = None
            return DimensionResult(
                state=SAFETY_FAULT,
                reasons=[Reason(
                    code="safety.fault_latched",
                    severity=SEVERITY_ERROR,
                    message=(
                        f"Fault latched; input reports {state} "
                        "(only sustained Normal clears it)"
                    ),
                    source=self.input.source,
                )],
                evidence={"latch": True},
            )

        if state == SAFETY_NORMAL:
            return DimensionResult(
                state=SAFETY_NORMAL,
                reasons=[],
                evidence={"age_s": _round(age), "latch": False},
            )

        if state == SAFETY_WARNING:
            return DimensionResult(
                state=SAFETY_WARNING,
                reasons=[Reason(
                    code="safety.warning",
                    severity=SEVERITY_WARNING,
                    message=detail or "Safety source reports a warning",
                    source=self.input.source,
                )],
                evidence={"age_s": _round(age)},
            )

        # Any explicit unknown (or an unrecognised value) is Unknown, never
        # promoted to Normal.
        return self._unknown(
            "safety.input_unknown",
            detail or f"Safety source reports {state}",
        )

    def _unknown(self, code: str, message: str) -> DimensionResult:
        return DimensionResult(
            state=SAFETY_UNKNOWN,
            reasons=[Reason(
                code=code,
                severity=SEVERITY_WARNING,
                message=message,
                source=self.input.source,
            )],
            evidence={"latch": self._fault_latched},
        )


def _round(value: Optional[float]) -> Optional[float]:
    return None if value is None else round(float(value), 3)
