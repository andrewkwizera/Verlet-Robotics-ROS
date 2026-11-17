"""Static checks for the Phase 1c diagnostic browser harness.

The page is now a thin renderer over the shared browser client used by the
production dashboard, so the media/signaling assertions live against
``webrtc_client.js`` and the DOM/hook assertions live against the wrapper.
"""

from __future__ import annotations

import os
import unittest

_PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEB_DIR = os.path.join(_PACKAGE_ROOT, "dashboard", "web")


class DiagnosticPageTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        for name, attribute in (
            ("webrtc_diagnostic.html", "html"),
            ("webrtc_diagnostic.js", "js"),
            ("webrtc_client.js", "client"),
            ("index.html", "index"),
        ):
            with open(os.path.join(WEB_DIR, name), encoding="utf-8") as handle:
                setattr(cls, attribute, handle.read())

    def test_two_independent_video_elements(self):
        self.assertIn('id="video-d405"', self.html)
        self.assertIn('id="video-d455f"', self.html)
        self.assertIn('"d405", "d455f"', self.client)
        self.assertIn("new global.RTCPeerConnection", self.client)

    def test_shared_client_is_loaded_before_the_wrapper(self):
        self.assertIn('src="webrtc_client.js"', self.html)
        self.assertIn('src="webrtc_diagnostic.js"', self.html)
        self.assertLess(
            self.html.index('src="webrtc_client.js"'),
            self.html.index('src="webrtc_diagnostic.js"'),
        )
        self.assertIn("window.YAMWebRTC.createController", self.js)
        self.assertNotIn("RTCPeerConnection", self.js)
        self.assertNotIn("/api/webrtc/", self.js)

    def test_h264_preference_and_non_trickle_ice(self):
        self.assertIn("video/h264", self.client)
        self.assertIn("setCodecPreferences", self.client)
        self.assertIn("waitForIceGathering", self.client)
        self.assertIn("iceGatheringState", self.client)

    def test_uses_phase1b_signaling_and_keepalive(self):
        self.assertIn('"/api/webrtc/session"', self.client)
        self.assertIn('"/api/webrtc/" + route', self.client)
        for route in ("offer", "answer", "stats", "keepalive"):
            with self.subTest(route=route):
                self.assertIn(f'signalingPath("{route}"', self.client)
        self.assertIn("DEFAULT_KEEPALIVE_MS", self.client)

    def test_camera_failures_are_isolated(self):
        self.assertIn("Camera setup is isolated", self.client)
        self.assertIn("state.cameras[id].error", self.client)
        self.assertIn("function retry(id)", self.client)
        self.assertIn("resetCamera(id)", self.client)
        self.assertIn("Best effort only", self.client)

    def test_browser_and_gateway_stats_are_labelled_separately(self):
        self.assertIn('source: "browser"', self.client)
        self.assertIn('source: "gateway"', self.client)
        self.assertIn("getStats()", self.client)
        self.assertIn("gatewayHealth", self.client)

    def test_session_ids_are_absent_from_public_snapshot(self):
        self.assertIn(
            "const result = { phase: state.phase, cameras: {} };", self.client
        )
        self.assertNotIn("sessionId:", self.client[self.client.index("function snapshot()"):])
        self.assertNotIn("sessionId", self.html)

    def test_exposes_cdp_snapshot_and_per_camera_retry(self):
        self.assertIn("window.__webrtcDiagnostic", self.js)
        self.assertIn("snapshot,", self.js)
        self.assertIn("retry,", self.js)
        self.assertIn("async function retry(id)", self.js)
        self.assertIn("window.__webrtcDiagnosticReady", self.js)

    def test_diagnostic_wrapper_is_not_loaded_by_production_page(self):
        self.assertNotIn("webrtc_diagnostic.js", self.index)
        self.assertNotIn("webrtc_diagnostic.html", self.index)


if __name__ == "__main__":
    unittest.main(verbosity=2)
