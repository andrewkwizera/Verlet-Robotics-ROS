"""Minimal HTTP transport for the dashboard.

Serves the static frontend plus ``GET /api/state`` (read-only) and, when
explicitly enabled, the media-only ``/api/webrtc/*`` signaling endpoints.
Deliberately built on the Python standard library so the dashboard adds no
runtime dependency beyond ``rclpy``.

The handler never queries ROS: ``/api/state`` returns the snapshot that the node
already cached.  Signaling state lives in a separate in-memory store and is
never merged into that snapshot, so an extra browser tab cannot slow down the
control path or influence the read-only safety boundary.
"""

from __future__ import annotations

import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional
from urllib.parse import parse_qs, urlsplit

from .webrtc_signaling import SignalingError, SignalingStore

_CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".svg": "image/svg+xml",
    ".ico": "image/x-icon",
}

_MAX_SIGNALING_BODY = 1_048_576
_WEBRTC_POST_ROUTES = {
    "/api/webrtc/session",
    "/api/webrtc/keepalive",
    "/api/webrtc/offer",
    "/api/webrtc/answer",
    "/api/webrtc/stats",
}


class _DashboardHandler(BaseHTTPRequestHandler):
    server_version = "yam-dashboard/1.0"
    sys_version = ""
    protocol_version = "HTTP/1.1"

    # -- routing ------------------------------------------------------------

    def do_GET(self) -> None:  # noqa: N802 (stdlib naming)
        parts = urlsplit(self.path)
        route = parts.path

        if route.startswith("/api/webrtc/"):
            self._webrtc_get(parts)
            return
        if route in ("/", "/index.html"):
            self._serve_static("index.html")
            return
        if route == "/api/state":
            self._serve_state()
            return
        if route == "/favicon.ico":
            self._send_bytes(204, b"", "image/x-icon")
            return

        self._serve_static(route.lstrip("/"))

    def do_HEAD(self) -> None:  # noqa: N802
        self._send_bytes(405, b"", "text/plain; charset=utf-8")

    def do_POST(self) -> None:  # noqa: N802
        parts = urlsplit(self.path)
        if parts.path.startswith("/api/webrtc/"):
            self._webrtc_post(parts)
            return
        self._method_not_allowed()

    def do_DELETE(self) -> None:  # noqa: N802
        parts = urlsplit(self.path)
        if parts.path == "/api/webrtc/session":
            self._webrtc_delete_session(parts)
            return
        self._method_not_allowed()

    # -- signaling ----------------------------------------------------------

    @property
    def _signaling(self) -> Optional[SignalingStore]:
        return getattr(self.server, "signaling", None)

    @staticmethod
    def _query(parts, name: str) -> Optional[str]:
        values = parse_qs(parts.query).get(name)
        return values[0] if values else None

    def _read_json(self) -> Optional[dict]:
        """Return a JSON object body, or None when it is absent or invalid."""
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except (TypeError, ValueError):
            self.close_connection = True
            return None
        if length < 0 or length > _MAX_SIGNALING_BODY:
            self.close_connection = True
            return None
        raw = self.rfile.read(length) if length else b""
        if not raw:
            return {}
        try:
            value = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, ValueError):
            return None
        return value if isinstance(value, dict) else None

    def _send_json(self, status: int, document: dict) -> None:
        body = json.dumps(document, separators=(",", ":")).encode("utf-8")
        self._send_bytes(status, body, "application/json; charset=utf-8")

    def _method_not_allowed(self) -> None:
        # The request body has not been read; close rather than desynchronise
        # a persistent connection.
        self.close_connection = True
        self._send_bytes(405, b"", "text/plain; charset=utf-8")

    def _webrtc_session_ids(self, parts):
        session_id = self._query(parts, "session_id")
        camera_id = self._query(parts, "camera_id")
        if not session_id or not camera_id:
            self._send_bytes(
                400, b"session_id and camera_id are required\n",
                "text/plain; charset=utf-8",
            )
            return None
        return session_id, camera_id

    def _webrtc_revision(self, parts) -> Optional[int]:
        raw = self._query(parts, "after_rev")
        if raw is None:
            return 0
        try:
            value = int(raw)
        except ValueError:
            self._send_bytes(
                400, b"after_rev must be an integer\n",
                "text/plain; charset=utf-8",
            )
            return None
        return value if value >= 0 else 0

    def _webrtc_timeout(self, parts) -> Optional[float]:
        raw = self._query(parts, "timeout")
        if raw is None:
            # Negative means "use the store default"; None means invalid.
            return -1.0
        try:
            value = float(raw)
        except ValueError:
            self._send_bytes(
                400, b"timeout must be a number\n",
                "text/plain; charset=utf-8",
            )
            return None
        return min(max(value, 0.0), 30.0)

    def _webrtc_get(self, parts) -> None:
        store = self._signaling
        route = parts.path
        if store is None or route not in (
            "/api/webrtc/sessions",
            "/api/webrtc/offer",
            "/api/webrtc/answer",
            "/api/webrtc/stats",
        ):
            self._send_bytes(404, b"Not found\n", "text/plain; charset=utf-8")
            return

        if route == "/api/webrtc/sessions":
            self._send_json(200, {"sessions": store.active_sessions()})
            return

        ids = self._webrtc_session_ids(parts)
        if ids is None:
            return
        session_id, camera_id = ids
        after_rev = self._webrtc_revision(parts)
        if after_rev is None:
            return
        timeout = self._webrtc_timeout(parts)
        if timeout is None:
            return
        if timeout < 0.0:
            timeout = None

        try:
            if route == "/api/webrtc/offer":
                result = store.get_offer(
                    session_id, camera_id, after_rev, timeout
                )
            elif route == "/api/webrtc/answer":
                result = store.get_answer(
                    session_id, camera_id, after_rev, timeout
                )
            else:
                result = store.get_stats(
                    session_id, camera_id, after_rev, timeout
                )
        except SignalingError:
            self._send_bytes(404, b"Unknown session or camera\n",
                             "text/plain; charset=utf-8")
            return

        if result is None:
            self._send_bytes(204, b"", "text/plain; charset=utf-8")
        else:
            self._send_json(200, result)

    def _webrtc_post(self, parts) -> None:
        store = self._signaling
        route = parts.path
        if store is None:
            self._send_bytes(404, b"Not found\n", "text/plain; charset=utf-8")
            return
        if route not in _WEBRTC_POST_ROUTES:
            self._method_not_allowed()
            return

        if route == "/api/webrtc/session":
            self._send_json(201, {"session_id": store.create_session()})
            return

        session_id = self._query(parts, "session_id")
        if not session_id:
            self._send_bytes(
                400, b"session_id is required\n", "text/plain; charset=utf-8"
            )
            return

        if route == "/api/webrtc/keepalive":
            try:
                store.touch_session(session_id)
            except SignalingError:
                self._send_bytes(404, b"Unknown session\n",
                                 "text/plain; charset=utf-8")
                return
            self._send_bytes(204, b"", "text/plain; charset=utf-8")
            return

        ids = self._webrtc_session_ids(parts)
        if ids is None:
            return
        session_id, camera_id = ids
        document = self._read_json()
        if document is None:
            self._send_bytes(400, b"Invalid JSON body\n",
                             "text/plain; charset=utf-8")
            return

        try:
            if route == "/api/webrtc/stats":
                stats = document.get("stats")
                if not isinstance(stats, dict):
                    raise ValueError("stats must be an object")
                revision = store.put_stats(
                    session_id, camera_id, stats,
                    str(document.get("health", "unknown")),
                )
            else:
                sdp = document.get("sdp")
                if not isinstance(sdp, str) or not sdp:
                    raise ValueError("sdp must be a non-empty string")
                if route == "/api/webrtc/offer":
                    revision = store.put_offer(session_id, camera_id, sdp)
                else:
                    revision = store.put_answer(session_id, camera_id, sdp)
        except SignalingError:
            self._send_bytes(404, b"Unknown session or camera\n",
                             "text/plain; charset=utf-8")
            return
        except ValueError as exc:
            self._send_bytes(400, str(exc).encode("utf-8"),
                             "text/plain; charset=utf-8")
            return

        self._send_json(202, {"revision": revision})

    def _webrtc_delete_session(self, parts) -> None:
        store = self._signaling
        if store is None:
            self._send_bytes(404, b"Not found\n", "text/plain; charset=utf-8")
            return
        session_id = self._query(parts, "session_id")
        if not session_id:
            self._send_bytes(
                400, b"session_id is required\n", "text/plain; charset=utf-8"
            )
            return
        try:
            store.delete_session(session_id)
        except SignalingError:
            self._send_bytes(404, b"Unknown session\n",
                             "text/plain; charset=utf-8")
            return
        self._send_bytes(204, b"", "text/plain; charset=utf-8")

    # -- responders ---------------------------------------------------------

    def _serve_state(self) -> None:
        source = self.server.snapshot_source  # type: ignore[attr-defined]
        snapshot = source.snapshot()
        if snapshot is None:
            snapshot = {
                "schema": 2,
                "ready": False,
                "message": "Dashboard is starting; no telemetry evaluated yet.",
            }
        body = json.dumps(snapshot, separators=(",", ":")).encode("utf-8")
        self._send_bytes(200, body, "application/json; charset=utf-8")

    def _serve_static(self, relative: str) -> None:
        base = os.path.realpath(self.server.web_dir)  # type: ignore[attr-defined]
        candidate = os.path.realpath(os.path.join(base, relative))

        try:
            inside = os.path.commonpath([base, candidate]) == base
        except ValueError:
            inside = False

        if not inside or not os.path.isfile(candidate):
            self._send_bytes(
                404,
                b"Not found\n",
                "text/plain; charset=utf-8",
            )
            return

        try:
            with open(candidate, "rb") as handle:
                payload = handle.read()
        except OSError:
            self._send_bytes(404, b"Not found\n", "text/plain; charset=utf-8")
            return

        extension = os.path.splitext(candidate)[1].lower()
        self._send_bytes(
            200,
            payload,
            _CONTENT_TYPES.get(extension, "application/octet-stream"),
        )

    def _send_bytes(self, status: int, payload: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        if payload:
            self.wfile.write(payload)

    # Access-log spam is unhelpful for a 10 Hz poller.
    def log_message(self, fmt: str, *args) -> None:  # noqa: A003
        return


class DashboardHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        address: tuple[str, int],
        snapshot_source,
        web_dir: str,
        signaling: Optional[SignalingStore] = None,
    ) -> None:
        self.snapshot_source = snapshot_source
        self.web_dir = web_dir
        self.signaling = signaling
        super().__init__(address, _DashboardHandler)


class ServerThread:
    """Owns the HTTP server and its serving thread."""

    def __init__(
        self,
        snapshot_source,
        bind_address: str,
        port: int,
        web_dir: str,
        signaling: Optional[SignalingStore] = None,
    ) -> None:
        self._server = DashboardHTTPServer(
            (bind_address, port), snapshot_source, web_dir, signaling
        )
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name="yam-dashboard-http",
            daemon=True,
        )
        self.address = self._server.server_address

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2.0)
