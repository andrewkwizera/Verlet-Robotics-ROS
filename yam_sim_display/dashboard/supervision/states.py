"""State vocabulary for the supervision layer.

Pure Python (no ROS imports) so every rule is unit-testable without a graph.
The four dimensions are intentionally independent: no combined state is
computed or stored here.
"""

from __future__ import annotations

# --- system health ---------------------------------------------------------

HEALTH_HEALTHY = "healthy"
HEALTH_DEGRADED = "degraded"
HEALTH_DISCONNECTED = "disconnected"

# --- operational readiness -------------------------------------------------

READINESS_READY = "ready"
READINESS_NOT_READY = "not_ready"
READINESS_INITIALIZING = "initializing"

# --- safety ----------------------------------------------------------------

SAFETY_NORMAL = "normal"
SAFETY_WARNING = "warning"
SAFETY_FAULT = "fault"
SAFETY_UNKNOWN = "unknown"

# --- operator session (owned by the browser; vocabulary shared) ------------

SESSION_IDLE = "idle"
SESSION_ACTIVE = "active"
SESSION_PAUSED = "paused"
SESSION_ENDED = "ended"

SESSION_STATES = (
    SESSION_IDLE,
    SESSION_ACTIVE,
    SESSION_PAUSED,
    SESSION_ENDED,
)

# Canonical session transition table.  The browser drives the state machine
# and reads this table from the API; the Python copy is what the API publishes
# and is also asserted against the JavaScript fallback by the tests.
SESSION_TRANSITIONS = {
    SESSION_IDLE: (SESSION_ACTIVE,),
    SESSION_ACTIVE: (SESSION_PAUSED, SESSION_ENDED),
    SESSION_PAUSED: (SESSION_ACTIVE, SESSION_ENDED),
    SESSION_ENDED: (SESSION_ACTIVE,),
}

# Operator actions are constrained per state.  The target table alone cannot
# distinguish "start" from "resume" because both lead to active.
SESSION_ACTION_TARGET = {
    "start": SESSION_ACTIVE,
    "pause": SESSION_PAUSED,
    "resume": SESSION_ACTIVE,
    "end": SESSION_ENDED,
}

SESSION_ACTIONS = {
    SESSION_IDLE: ("start",),
    SESSION_ACTIVE: ("pause", "end"),
    SESSION_PAUSED: ("resume", "end"),
    SESSION_ENDED: ("start",),
}

LABELS = {
    HEALTH_HEALTHY: "Healthy",
    HEALTH_DEGRADED: "Degraded",
    HEALTH_DISCONNECTED: "Disconnected",
    READINESS_READY: "Ready",
    READINESS_NOT_READY: "Not Ready",
    READINESS_INITIALIZING: "Initializing",
    SAFETY_NORMAL: "Normal",
    SAFETY_WARNING: "Warning",
    SAFETY_FAULT: "Fault",
    SAFETY_UNKNOWN: "Unknown",
    SESSION_IDLE: "Idle",
    SESSION_ACTIVE: "Active",
    SESSION_PAUSED: "Paused",
    SESSION_ENDED: "Ended",
}

# Severity used for reason ordering and drawer presentation.
SEVERITY_INFO = "info"
SEVERITY_WARNING = "warning"
SEVERITY_ERROR = "error"
SEVERITY_BLOCKING = "blocking"

SEVERITY_RANK = {
    SEVERITY_INFO: 0,
    SEVERITY_WARNING: 1,
    SEVERITY_ERROR: 2,
    SEVERITY_BLOCKING: 3,
}

# A blocking prerequisite is still an operator-facing warning, never a fault.
EVENT_LEVEL = {
    SEVERITY_INFO: "info",
    SEVERITY_WARNING: "warning",
    SEVERITY_ERROR: "error",
    SEVERITY_BLOCKING: "warning",
}

DIMENSIONS = ("health", "readiness", "safety", "session")
