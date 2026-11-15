"""Minimal read-only HTTP transport for the dashboard.

Serves the static frontend plus ``GET /api/state``.  Deliberately built on the
Python standard library so the dashboard adds no runtime dependency beyond
``rclpy``.

The handler never queries ROS: it returns the snapshot that the node already
cached.  That keeps HTTP latency independent of the ROS executor and means an
extra browser tab cannot slow down the control path.
"""

from __future__ import annotations

import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional
from urllib.parse import urlsplit

_CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".svg": "image/svg+xml",
    ".ico": "image/x-icon",
}


class _DashboardHandler(BaseHTTPRequestHandler):
    server_version = "yam-dashboard/1.0"
    sys_version = ""
    protocol_version = "HTTP/1.1"

    # -- routing ------------------------------------------------------------

    def do_GET(self) -> None:  # noqa: N802 (stdlib naming)
        route = urlsplit(self.path).path

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

    # -- responders ---------------------------------------------------------

    def _serve_state(self) -> None:
        snapshot = self.server.store.snapshot()  # type: ignore[attr-defined]
        if snapshot is None:
            snapshot = {
                "schema": 1,
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

    def __init__(self, address: tuple[str, int], store, web_dir: str) -> None:
        self.store = store
        self.web_dir = web_dir
        super().__init__(address, _DashboardHandler)


class ServerThread:
    """Owns the HTTP server and its serving thread."""

    def __init__(self, store, bind_address: str, port: int, web_dir: str) -> None:
        self._server = DashboardHTTPServer((bind_address, port), store, web_dir)
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
