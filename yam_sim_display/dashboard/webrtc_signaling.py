"""Minimal store-and-forward signaling for the M3 research prototype.

This module is deliberately small and has no ROS, authentication or TLS layer.
It is media-only: it stores offer/answer/stats blobs in per-tab, per-camera
mailboxes and can never address a ROS topic, service or parameter.

State lives on the HTTP server threads only.  It is never merged into the
Milestone 1/2 telemetry snapshot, so the read-only safety boundary is intact.
"""

from __future__ import annotations

import threading
import time
import uuid
from typing import Dict, Optional, Tuple

CAMERA_IDS = ("d405", "d455f")
DEFAULT_SESSION_TTL_S = 300.0
DEFAULT_POLL_TIMEOUT_S = 10.0


class SignalingError(Exception):
    """Base class for signaling validation failures."""


class UnknownSession(SignalingError):
    """A session id was absent or expired."""


class UnknownCamera(SignalingError):
    """A camera id was not in the configured allowlist."""


class SignalingStore:
    """Thread-safe, non-trickle signaling mailboxes.

    Each mailbox is keyed by ``(session_id, camera_id)``.  A revision counter
    per field lets a long-poll caller wait only for newer data, which also
    makes a restarted gateway or browser resilient to duplicate polls.
    """

    def __init__(
        self,
        camera_ids=CAMERA_IDS,
        session_ttl_s: float = DEFAULT_SESSION_TTL_S,
        poll_timeout_s: float = DEFAULT_POLL_TIMEOUT_S,
        clock=time.monotonic,
    ) -> None:
        self._camera_ids = tuple(camera_ids)
        self._ttl = float(session_ttl_s)
        self._poll_timeout = float(poll_timeout_s)
        self._clock = clock
        self._lock = threading.Lock()
        self._changed = threading.Condition(self._lock)
        self._sessions: Dict[str, float] = {}
        self._mailboxes: Dict[Tuple[str, str], dict] = {}

    @property
    def camera_ids(self) -> tuple[str, ...]:
        """Return the cameras enabled for this dashboard deployment."""
        return self._camera_ids

    # -- sessions -----------------------------------------------------------

    def create_session(self) -> str:
        with self._changed:
            self._prune_locked()
            session_id = uuid.uuid4().hex
            self._sessions[session_id] = self._clock() + self._ttl
            return session_id

    def touch_session(self, session_id: str) -> None:
        with self._changed:
            self._prune_locked()
            if session_id not in self._sessions:
                raise UnknownSession(session_id)
            self._sessions[session_id] = self._clock() + self._ttl

    def delete_session(self, session_id: str) -> None:
        with self._changed:
            if session_id not in self._sessions:
                raise UnknownSession(session_id)
            del self._sessions[session_id]
            for key in [k for k in self._mailboxes if k[0] == session_id]:
                del self._mailboxes[key]
            self._changed.notify_all()

    def active_sessions(self) -> list[dict]:
        """Return unexpired sessions, newest first, with offer revisions only.

        Session ids are sensitive while signaling has no authentication: the
        response is intentionally minimal and is never written to any log or
        to the M1/M2 state snapshot.
        """
        with self._changed:
            self._prune_locked()
            sessions = []
            for session_id in reversed(list(self._sessions)):
                revisions = {}
                for camera_id in self._camera_ids:
                    mailbox = self._mailboxes.get((session_id, camera_id))
                    revisions[camera_id] = (
                        mailbox["offer_rev"] if mailbox is not None else 0
                    )
                sessions.append({
                    "session_id": session_id,
                    "offer_rev": revisions,
                })
            return sessions

    # -- media-only mailboxes -----------------------------------------------

    def put_offer(self, session_id: str, camera_id: str, sdp: str) -> int:
        with self._changed:
            self._prune_locked()
            mailbox = self._mailbox_locked(session_id, camera_id)
            mailbox["offer"] = str(sdp)
            mailbox["offer_rev"] += 1
            mailbox["updated"] = self._clock()
            self._changed.notify_all()
            return mailbox["offer_rev"]

    def get_offer(
        self,
        session_id: str,
        camera_id: str,
        after_rev: int = 0,
        timeout: Optional[float] = None,
    ) -> Optional[dict]:
        return self._get(
            session_id, camera_id, "offer", "sdp", after_rev, timeout
        )

    def put_answer(self, session_id: str, camera_id: str, sdp: str) -> int:
        with self._changed:
            self._prune_locked()
            mailbox = self._mailbox_locked(session_id, camera_id)
            mailbox["answer"] = str(sdp)
            mailbox["answer_rev"] += 1
            mailbox["updated"] = self._clock()
            self._changed.notify_all()
            return mailbox["answer_rev"]

    def get_answer(
        self,
        session_id: str,
        camera_id: str,
        after_rev: int = 0,
        timeout: Optional[float] = None,
    ) -> Optional[dict]:
        return self._get(
            session_id, camera_id, "answer", "sdp", after_rev, timeout
        )

    def put_stats(
        self,
        session_id: str,
        camera_id: str,
        stats: dict,
        health: str = "unknown",
    ) -> int:
        with self._changed:
            self._prune_locked()
            mailbox = self._mailbox_locked(session_id, camera_id)
            mailbox["stats"] = dict(stats)
            mailbox["health"] = str(health)
            mailbox["stats_rev"] += 1
            mailbox["updated"] = self._clock()
            self._changed.notify_all()
            return mailbox["stats_rev"]

    def get_stats(
        self,
        session_id: str,
        camera_id: str,
        after_rev: int = 0,
        timeout: Optional[float] = None,
    ) -> Optional[dict]:
        result = self._get(
            session_id, camera_id, "stats", "stats", after_rev, timeout
        )
        if result is None:
            return None
        with self._lock:
            mailbox = self._mailbox_locked(session_id, camera_id)
            result["health"] = mailbox["health"]
        return result

    # -- internals ----------------------------------------------------------

    def _blank_mailbox(self) -> dict:
        return {
            "offer": None,
            "offer_rev": 0,
            "answer": None,
            "answer_rev": 0,
            "stats": None,
            "stats_rev": 0,
            "health": "unknown",
            "updated": 0.0,
        }

    def _prune_locked(self) -> None:
        now = self._clock()
        expired = [sid for sid, expiry in self._sessions.items() if expiry <= now]
        for session_id in expired:
            del self._sessions[session_id]
            for key in [k for k in self._mailboxes if k[0] == session_id]:
                del self._mailboxes[key]

    def _mailbox_locked(self, session_id: str, camera_id: str) -> dict:
        if camera_id not in self._camera_ids:
            raise UnknownCamera(camera_id)
        if session_id not in self._sessions:
            raise UnknownSession(session_id)
        key = (session_id, camera_id)
        if key not in self._mailboxes:
            self._mailboxes[key] = self._blank_mailbox()
        return self._mailboxes[key]

    def _get(
        self,
        session_id: str,
        camera_id: str,
        field: str,
        payload_key: str,
        after_rev: int,
        timeout: Optional[float],
    ) -> Optional[dict]:
        seconds = self._poll_timeout if timeout is None else float(timeout)
        deadline = self._clock() + max(0.0, seconds)
        rev_key = f"{field}_rev"
        with self._changed:
            while True:
                self._prune_locked()
                mailbox = self._mailbox_locked(session_id, camera_id)
                if mailbox[rev_key] > after_rev:
                    return {
                        payload_key: mailbox[field],
                        "revision": mailbox[rev_key],
                    }
                remaining = deadline - self._clock()
                if remaining <= 0.0:
                    return None
                self._changed.wait(remaining)
