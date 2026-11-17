"""Stdlib HTTP client for the Phase 1b signaling API.

The gateway never imports ROS.  Session ids are opaque secrets here: they are
kept in memory only and are never included in log messages or exceptions.
"""

from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from typing import Any, Mapping, Optional


@dataclass(frozen=True)
class SessionInfo:
    session_id: str
    offer_rev: Mapping[str, int]


class SignalingError(RuntimeError):
    """Base class for signaling failures."""


class SessionExpired(SignalingError):
    """The selected session was absent or expired (HTTP 404)."""


class SignalingUnavailable(SignalingError):
    """The signaling service could not be reached or returned a server error."""


class SignalingClient:
    """Small blocking client for the media-only Phase 1b routes."""

    def __init__(self, base_url: str, user_agent: str = "yam-webrtc-gateway/1.0"):
        self.base_url = base_url.rstrip("/")
        self.user_agent = user_agent

    # -- public API -------------------------------------------------------

    def active_sessions(self) -> list[SessionInfo]:
        """Return active sessions, newest first, with minimal fields only."""
        document = self._request("GET", "/api/webrtc/sessions")
        if not isinstance(document, dict):
            raise SignalingError("invalid sessions response")
        raw_sessions = document.get("sessions")
        if not isinstance(raw_sessions, list):
            raise SignalingError("sessions response has no sessions list")

        sessions: list[SessionInfo] = []
        for entry in raw_sessions:
            if not isinstance(entry, dict):
                continue
            session_id = entry.get("session_id")
            if not isinstance(session_id, str) or not session_id:
                continue
            raw_revisions = entry.get("offer_rev", {})
            revisions: dict[str, int] = {}
            if isinstance(raw_revisions, dict):
                for camera_id, value in raw_revisions.items():
                    try:
                        revisions[str(camera_id)] = int(value)
                    except (TypeError, ValueError):
                        revisions[str(camera_id)] = 0
            sessions.append(SessionInfo(session_id, revisions))
        return sessions

    def get_offer(
        self,
        session_id: str,
        camera_id: str,
        after_rev: int = 0,
        timeout_s: float = 10.0,
    ) -> Optional[tuple[str, int]]:
        """Long-poll one camera mailbox; return ``(sdp, revision)`` or None."""
        document = self._request(
            "GET",
            "/api/webrtc/offer",
            params={
                "session_id": session_id,
                "camera_id": camera_id,
                "after_rev": int(after_rev),
                "timeout": f"{float(timeout_s):.3f}",
            },
            http_timeout=float(timeout_s) + 10.0,
        )
        if document is None:
            return None
        sdp = document.get("sdp")
        revision = document.get("revision")
        if not isinstance(sdp, str) or not sdp:
            raise SignalingError("offer response has no SDP")
        try:
            return sdp, int(revision)
        except (TypeError, ValueError) as exc:
            raise SignalingError("offer response has an invalid revision") from exc

    def post_answer(
        self, session_id: str, camera_id: str, sdp: str
    ) -> int:
        """Publish a gateway answer for one camera mailbox."""
        document = self._request(
            "POST",
            "/api/webrtc/answer",
            params={"session_id": session_id, "camera_id": camera_id},
            payload={"sdp": sdp},
        )
        return _revision_from(document, "answer")

    def post_stats(
        self,
        session_id: str,
        camera_id: str,
        stats: Mapping[str, Any],
        health: str,
    ) -> int:
        """Publish gateway-side stats for one camera mailbox."""
        document = self._request(
            "POST",
            "/api/webrtc/stats",
            params={"session_id": session_id, "camera_id": camera_id},
            payload={"stats": dict(stats), "health": str(health)},
        )
        return _revision_from(document, "stats")

    def get_stats(
        self,
        session_id: str,
        camera_id: str,
        after_rev: int = 0,
        timeout_s: float = 10.0,
    ) -> Optional[dict]:
        """Read one gateway stats mailbox (not browser statistics)."""
        document = self._request(
            "GET",
            "/api/webrtc/stats",
            params={
                "session_id": session_id,
                "camera_id": camera_id,
                "after_rev": int(after_rev),
                "timeout": f"{float(timeout_s):.3f}",
            },
            http_timeout=float(timeout_s) + 10.0,
        )
        if document is None:
            return None
        try:
            revision = int(document.get("revision"))
        except (TypeError, ValueError) as exc:
            raise SignalingError("stats response has an invalid revision") from exc
        return {
            "stats": document.get("stats", {}),
            "health": document.get("health", "unknown"),
            "revision": revision,
        }

    def keepalive(self, session_id: str) -> None:
        """Touch the selected session without exposing its id in errors."""
        self._request(
            "POST",
            "/api/webrtc/keepalive",
            params={"session_id": session_id},
        )

    # -- internals --------------------------------------------------------

    def _request(
        self,
        method: str,
        route: str,
        params: Optional[Mapping[str, Any]] = None,
        payload: Optional[Mapping[str, Any]] = None,
        http_timeout: float = 15.0,
    ) -> Optional[dict]:
        url = self.base_url + route
        if params:
            url += "?" + urllib.parse.urlencode(params)

        data = None
        headers = {"User-Agent": self.user_agent}
        if payload is not None:
            data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            headers["Content-Type"] = "application/json"

        request = urllib.request.Request(url, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(request, timeout=http_timeout) as response:
                if response.status == 204:
                    return None
                raw = response.read()
                return json.loads(raw.decode("utf-8")) if raw else None
        except urllib.error.HTTPError as exc:
            if exc.code == 404:
                raise SessionExpired(f"{route}: session is gone") from None
            if exc.code == 405 and route == "/api/webrtc/sessions":
                raise SignalingError("signaling routes are disabled") from None
            if exc.code >= 500:
                raise SignalingUnavailable(f"{route}: HTTP {exc.code}") from None
            raise SignalingError(f"{route}: HTTP {exc.code}") from None
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise SignalingUnavailable(f"{route}: signaling unreachable") from exc
        except (UnicodeDecodeError, ValueError) as exc:
            raise SignalingError(f"{route}: invalid JSON response") from exc


def _revision_from(document: Optional[dict], field: str) -> int:
    if not isinstance(document, dict):
        raise SignalingError(f"{field} response was empty")
    try:
        return int(document.get("revision"))
    except (TypeError, ValueError) as exc:
        raise SignalingError(f"{field} response has an invalid revision") from exc


class SessionSelector:
    """Choose the newest active session and recover when it disappears.

    The selector is shared by all camera workers and the keepalive thread.  A
    worker only resets its own offer revision when the session id actually
    changes, so one expired session never forces the whole gateway to restart.
    """

    def __init__(
        self,
        client: SignalingClient,
        refresh_interval_s: float = 1.0,
        clock=time.monotonic,
    ) -> None:
        self._client = client
        self._refresh_interval = float(refresh_interval_s)
        self._clock = clock
        self._lock = threading.Lock()
        self._current: Optional[SessionInfo] = None
        self._last_refresh = 0.0
        self._rejected: set[str] = set()
        self.generation = 0

    def current(self, force: bool = False) -> Optional[SessionInfo]:
        """Return the newest known active session, refreshing if due."""
        with self._lock:
            now = self._clock()
            if (
                not force
                and self._current is not None
                and (now - self._last_refresh) < self._refresh_interval
            ):
                return self._current

            try:
                sessions = self._client.active_sessions()
            except SignalingError:
                # Keep the last known session; transient failures must not
                # tear down independent camera pipelines.
                return self._current

            previous_id = self._current.session_id if self._current else None
            sessions = [
                session for session in sessions
                if session.session_id not in self._rejected
            ]
            self._current = sessions[0] if sessions else None
            self._last_refresh = now
            new_id = self._current.session_id if self._current else None
            if new_id != previous_id:
                self.generation += 1
            if sessions:
                # A different active session was found; allow future arrivals
                # with these ids to be considered again.
                self._rejected.clear()
            return self._current

    def invalidate(self, session_id: str) -> None:
        """Forget a session after 404 so the next call rediscovers another."""
        with self._lock:
            self._rejected.add(session_id)
            if self._current is not None and self._current.session_id == session_id:
                self._current = None
                self._last_refresh = 0.0
                self.generation += 1
