"""Functional tests for the M3 Phase 1b signaling prototype.

Pure Python: no ROS, no authentication, no TLS.  The HTTP tests run a real
loopback server so the same routes the gateway will use are exercised.
"""

from __future__ import annotations

import http.client
import json
import os
import sys
import threading
import time
import unittest

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_PACKAGE_ROOT = os.path.dirname(_TEST_DIR)
if _TEST_DIR not in sys.path:
    sys.path.insert(0, _TEST_DIR)

from dashboard.http_server import DashboardHTTPServer  # noqa: E402
from dashboard.state_cache import StateCache  # noqa: E402
from dashboard.webrtc_signaling import (  # noqa: E402
    CAMERA_IDS,
    SignalingStore,
    UnknownCamera,
    UnknownSession,
)

WEB_DIR = os.path.join(_PACKAGE_ROOT, "dashboard", "web")


class SignalingStoreTests(unittest.TestCase):
    def setUp(self):
        self.store = SignalingStore(session_ttl_s=5.0, poll_timeout_s=0.02)

    def test_camera_ids_are_frozen(self):
        self.assertEqual(CAMERA_IDS, ("d405", "d455f"))

    def test_session_ids_are_unique(self):
        first = self.store.create_session()
        second = self.store.create_session()
        self.assertNotEqual(first, second)

    def test_session_lifecycle(self):
        session_id = self.store.create_session()
        self.store.put_offer(session_id, "d405", "offer-sdp")
        self.store.touch_session(session_id)
        self.store.delete_session(session_id)
        with self.assertRaises(UnknownSession):
            self.store.touch_session(session_id)
        with self.assertRaises(UnknownSession):
            self.store.get_offer(session_id, "d405", timeout=0.0)

    def test_camera_allowlist(self):
        session_id = self.store.create_session()
        with self.assertRaises(UnknownCamera):
            self.store.put_offer(session_id, "other", "sdp")
        with self.assertRaises(UnknownCamera):
            self.store.get_answer(session_id, "other", timeout=0.0)

    def test_active_sessions_are_newest_first_and_minimal(self):
        first = self.store.create_session()
        second = self.store.create_session()
        self.store.put_offer(first, "d405", "offer-1")
        sessions = self.store.active_sessions()

        self.assertEqual(
            [entry["session_id"] for entry in sessions], [second, first]
        )
        self.assertEqual(set(sessions[0]), {"session_id", "offer_rev"})
        self.assertEqual(sessions[0]["offer_rev"], {"d405": 0, "d455f": 0})
        self.assertEqual(sessions[1]["offer_rev"]["d405"], 1)

    def test_active_sessions_excludes_expired_sessions(self):
        now = [1000.0]
        store = SignalingStore(session_ttl_s=1.0, clock=lambda: now[0])
        expired = store.create_session()
        now[0] += 2.0
        live = store.create_session()
        self.assertEqual(
            [entry["session_id"] for entry in store.active_sessions()], [live]
        )

    def test_offer_and_answer_round_trip(self):
        session_id = self.store.create_session()
        self.assertEqual(self.store.put_offer(session_id, "d405", "offer-1"), 1)
        self.assertEqual(
            self.store.get_offer(session_id, "d405", after_rev=0),
            {"sdp": "offer-1", "revision": 1},
        )
        self.assertIsNone(
            self.store.get_offer(session_id, "d405", after_rev=1)
        )
        self.assertEqual(self.store.put_answer(session_id, "d405", "answer-1"), 1)
        self.assertEqual(
            self.store.get_answer(session_id, "d405", after_rev=0),
            {"sdp": "answer-1", "revision": 1},
        )

    def test_mailboxes_are_independent(self):
        first = self.store.create_session()
        second = self.store.create_session()
        self.store.put_offer(first, "d405", "only-d405")

        self.assertIsNone(
            self.store.get_offer(first, "d455f", timeout=0.0)
        )
        self.assertIsNone(
            self.store.get_offer(second, "d405", timeout=0.0)
        )
        self.store.put_offer(second, "d455f", "second-d455f")
        self.store.delete_session(first)
        self.assertEqual(
            self.store.get_offer(second, "d455f", after_rev=0)["sdp"],
            "second-d455f",
        )

    def test_stats_round_trip(self):
        session_id = self.store.create_session()
        stats = {"framesDecoded": 420, "fps": 30.0}
        self.assertEqual(self.store.put_stats(session_id, "d405", stats, "live"), 1)
        result = self.store.get_stats(session_id, "d405", after_rev=0)
        self.assertEqual(result["stats"], stats)
        self.assertEqual(result["health"], "live")

    def test_expired_session_and_mailbox_are_pruned(self):
        now = [1000.0]
        store = SignalingStore(session_ttl_s=1.0, clock=lambda: now[0])
        session_id = store.create_session()
        store.put_offer(session_id, "d405", "stale")
        now[0] += 2.0
        with self.assertRaises(UnknownSession):
            store.get_offer(session_id, "d405", timeout=0.0)

    def test_long_poll_wakes_on_new_offer(self):
        session_id = self.store.create_session()

        def offer_later():
            time.sleep(0.02)
            self.store.put_offer(session_id, "d455f", "late-offer")

        threading.Thread(target=offer_later, daemon=True).start()
        result = self.store.get_offer(session_id, "d455f", timeout=0.5)
        self.assertEqual(result["sdp"], "late-offer")


class SignalingHttpTests(unittest.TestCase):
    def setUp(self):
        self.cache = StateCache()
        self.cache.publish({"schema": 2, "events": []})
        self.store = SignalingStore(session_ttl_s=5.0, poll_timeout_s=0.02)
        self.server = DashboardHTTPServer(
            ("127.0.0.1", 0), self.cache, WEB_DIR, self.store
        )
        self.thread = threading.Thread(
            target=self.server.serve_forever, daemon=True
        )
        self.thread.start()
        self.port = self.server.server_address[1]

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2.0)

    def request(self, method, path, body=None):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        try:
            payload = json.dumps(body).encode() if body is not None else b""
            headers = {"Content-Type": "application/json"} if body is not None else {}
            connection.request(method, path, body=payload, headers=headers)
            response = connection.getresponse()
            return response.status, response.read()
        finally:
            connection.close()

    def new_session(self):
        status, body = self.request("POST", "/api/webrtc/session")
        self.assertEqual(status, 201)
        return json.loads(body)["session_id"]

    def test_session_lifecycle_over_http(self):
        session_id = self.new_session()
        self.assertTrue(session_id)
        self.assertEqual(
            self.request("POST", f"/api/webrtc/keepalive?session_id={session_id}")[0],
            204,
        )
        self.assertEqual(
            self.request("DELETE", f"/api/webrtc/session?session_id={session_id}")[0],
            204,
        )
        self.assertEqual(
            self.request("DELETE", f"/api/webrtc/session?session_id={session_id}")[0],
            404,
        )

    def test_session_discovery_over_http(self):
        first = self.new_session()
        second = self.new_session()
        self.request(
            "POST",
            f"/api/webrtc/offer?session_id={first}&camera_id=d405",
            {"sdp": "offer-1"},
        )
        status, body = self.request("GET", "/api/webrtc/sessions")
        self.assertEqual(status, 200)
        sessions = json.loads(body)["sessions"]
        self.assertEqual(
            [entry["session_id"] for entry in sessions], [second, first]
        )
        self.assertEqual(sessions[0]["offer_rev"]["d405"], 0)
        self.assertEqual(sessions[1]["offer_rev"]["d405"], 1)

    def test_offer_answer_exchange_over_http(self):
        session_id = self.new_session()
        base = f"session_id={session_id}&camera_id=d405"

        status, body = self.request(
            "POST", f"/api/webrtc/offer?{base}", {"sdp": "browser-offer"}
        )
        self.assertEqual(status, 202)
        self.assertEqual(json.loads(body)["revision"], 1)

        status, body = self.request("GET", f"/api/webrtc/offer?{base}")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["sdp"], "browser-offer")

        status, _ = self.request(
            "GET", f"/api/webrtc/offer?{base}&after_rev=1&timeout=0.01"
        )
        self.assertEqual(status, 204)

        status, _ = self.request(
            "POST", f"/api/webrtc/answer?{base}", {"sdp": "gateway-answer"}
        )
        self.assertEqual(status, 202)
        status, body = self.request("GET", f"/api/webrtc/answer?{base}")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["sdp"], "gateway-answer")

    def test_gateway_stats_over_http(self):
        session_id = self.new_session()
        base = f"session_id={session_id}&camera_id=d455f"
        status, _ = self.request(
            "POST", f"/api/webrtc/stats?{base}",
            {"stats": {"fps": 30.0}, "health": "live"},
        )
        self.assertEqual(status, 202)
        status, body = self.request("GET", f"/api/webrtc/stats?{base}")
        self.assertEqual(status, 200)
        document = json.loads(body)
        self.assertEqual(document["stats"]["fps"], 30.0)
        self.assertEqual(document["health"], "live")

    def test_camera_and_session_isolation_over_http(self):
        session_id = self.new_session()
        self.request(
            "POST", f"/api/webrtc/offer?session_id={session_id}&camera_id=d405",
            {"sdp": "d405-only"},
        )
        status, _ = self.request(
            "GET",
            f"/api/webrtc/offer?session_id={session_id}&camera_id=d455f"
            "&timeout=0.01",
        )
        self.assertEqual(status, 204)

        status, _ = self.request(
            "POST",
            "/api/webrtc/offer?session_id=missing&camera_id=d405",
            {"sdp": "invalid"},
        )
        self.assertEqual(status, 404)

    def test_invalid_requests_are_rejected(self):
        session_id = self.new_session()
        base = f"session_id={session_id}&camera_id=d405"
        status, _ = self.request("POST", f"/api/webrtc/offer?{base}", {})
        self.assertEqual(status, 400)
        status, _ = self.request("POST", f"/api/webrtc/offer?session_id={session_id}",
                                 {"sdp": "x"})
        self.assertEqual(status, 400)

        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        try:
            connection.request("POST", f"/api/webrtc/offer?{base}", b"not json")
            self.assertEqual(connection.getresponse().status, 400)
        finally:
            connection.close()

    def test_non_webrtc_writes_are_rejected(self):
        self.assertEqual(self.request("POST", "/api/state", {})[0], 405)
        self.assertEqual(self.request("DELETE", "/api/state")[0], 405)

    def test_m1_m2_state_is_unchanged(self):
        status, body = self.request("GET", "/api/state")
        self.assertEqual(status, 200)
        document = json.loads(body)
        self.assertEqual(document["schema"], 2)
        self.assertNotIn("webrtc", document)


if __name__ == "__main__":
    unittest.main(verbosity=2)
