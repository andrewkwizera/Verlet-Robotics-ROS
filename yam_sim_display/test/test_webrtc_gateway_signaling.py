"""Gateway signaling client tests against a real Phase 1b loopback server."""

from __future__ import annotations

import http.client
import os
import sys
import threading
import unittest

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_PACKAGE_ROOT = os.path.dirname(_TEST_DIR)
for path in (_TEST_DIR, _PACKAGE_ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)

from dashboard.http_server import DashboardHTTPServer  # noqa: E402
from dashboard.state_cache import StateCache  # noqa: E402
from dashboard.webrtc_signaling import SignalingStore  # noqa: E402
from gateway.signaling_client import (  # noqa: E402
    SessionExpired,
    SessionSelector,
    SignalingClient,
    SignalingError,
)

WEB_DIR = os.path.join(_PACKAGE_ROOT, "dashboard", "web")


class SignalingClientTests(unittest.TestCase):
    def setUp(self):
        self.store = SignalingStore(session_ttl_s=30.0, poll_timeout_s=0.05)
        self.cache = StateCache()
        self.server = DashboardHTTPServer(
            ("127.0.0.1", 0), self.cache, WEB_DIR, self.store
        )
        self.thread = threading.Thread(
            target=self.server.serve_forever, daemon=True
        )
        self.thread.start()
        self.base_url = f"http://127.0.0.1:{self.server.server_address[1]}"
        self.client = SignalingClient(self.base_url)

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2.0)

    def raw_post(self, path, body=b"{}"):
        connection = http.client.HTTPConnection(
            "127.0.0.1", self.server.server_address[1], timeout=5
        )
        try:
            connection.request("POST", path, body=body)
            response = connection.getresponse()
            response.read()
            return response.status
        finally:
            connection.close()

    def test_active_sessions_are_newest_first(self):
        first = self.store.create_session()
        second = self.store.create_session()
        self.store.put_offer(first, "d405", "old")

        sessions = self.client.active_sessions()
        self.assertEqual(
            [entry.session_id for entry in sessions], [second, first]
        )
        self.assertEqual(sessions[1].offer_rev["d405"], 1)
        self.assertEqual(sessions[1].offer_rev["d455f"], 0)

    def test_offer_answer_and_stats_round_trip(self):
        session_id = self.store.create_session()
        self.store.put_offer(session_id, "d455f", "offer-sdp")

        self.assertEqual(
            self.client.get_offer(session_id, "d455f"), ("offer-sdp", 1)
        )
        self.assertIsNone(
            self.client.get_offer(session_id, "d405", timeout_s=0.05)
        )
        answer_revision = self.client.post_answer(
            session_id, "d455f", "answer-sdp"
        )
        self.assertEqual(answer_revision, 1)
        self.assertEqual(
            self.store.get_answer(session_id, "d455f")["sdp"], "answer-sdp"
        )

        stats_revision = self.client.post_stats(
            session_id, "d455f", {"ingress_packets": 12}, "connected"
        )
        self.assertEqual(stats_revision, 1)
        stored = self.store.get_stats(session_id, "d455f")
        self.assertEqual(stored["stats"]["ingress_packets"], 12)
        self.assertEqual(stored["health"], "connected")

        gateway_stats = self.client.get_stats(session_id, "d455f")
        self.assertEqual(gateway_stats["stats"]["ingress_packets"], 12)
        self.assertEqual(gateway_stats["health"], "connected")

    def test_keepalive_touches_session(self):
        session_id = self.store.create_session()
        self.client.keepalive(session_id)
        self.assertEqual(
            self.raw_post("/api/webrtc/session", b"{}"), 201
        )

    def test_expired_session_raises_without_leaking_id(self):
        session_id = self.store.create_session()
        self.store.delete_session(session_id)
        with self.assertRaises(SessionExpired) as context:
            self.client.get_offer(session_id, "d405", timeout_s=0.05)
        self.assertNotIn(session_id, str(context.exception))

        with self.assertRaises(SessionExpired):
            self.client.post_answer(session_id, "d405", "answer")
        with self.assertRaises(SessionExpired):
            self.client.post_stats(session_id, "d405", {}, "error")
        with self.assertRaises(SessionExpired):
            self.client.keepalive(session_id)


class DisabledSignalingTests(unittest.TestCase):
    def test_disabled_endpoint_is_reported_cleanly(self):
        cache = StateCache()
        server = DashboardHTTPServer(("127.0.0.1", 0), cache, WEB_DIR)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            client = SignalingClient(
                f"http://127.0.0.1:{server.server_address[1]}"
            )
            with self.assertRaises(SignalingError):
                client.active_sessions()
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2.0)


class SessionSelectorTests(unittest.TestCase):
    def test_selector_discovers_newest_and_recovers_after_invalidation(self):
        store = SignalingStore()
        cache = StateCache()
        server = DashboardHTTPServer(
            ("127.0.0.1", 0), cache, WEB_DIR, store
        )
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            first = store.create_session()
            second = store.create_session()
            client = SignalingClient(
                f"http://127.0.0.1:{server.server_address[1]}"
            )
            selector = SessionSelector(client, refresh_interval_s=0.0)

            self.assertEqual(selector.current().session_id, second)
            selector.invalidate(second)
            self.assertEqual(selector.current().session_id, first)

            store.delete_session(second)
            store.delete_session(first)
            selector.invalidate(first)
            self.assertIsNone(selector.current())
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2.0)

    def test_selector_keeps_last_session_on_transient_error(self):
        class FailingClient:
            calls = 0

            def active_sessions(self):
                FailingClient.calls += 1
                raise SignalingError("temporary outage")

        from gateway.signaling_client import SessionInfo

        selector = SessionSelector(FailingClient(), refresh_interval_s=0.0)
        selector._current = SessionInfo("cached-id", {})
        self.assertIsNotNone(selector.current())
        self.assertEqual(selector.current().session_id, "cached-id")


if __name__ == "__main__":
    unittest.main(verbosity=2)
