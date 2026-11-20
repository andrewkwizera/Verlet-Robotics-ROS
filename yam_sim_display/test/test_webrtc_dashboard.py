"""Static checks for M3 Phase 1d dashboard WebRTC integration.

No media connection is opened here. These tests guard the production-page
contract: one shared client, two independent panels, unchanged fallback and
no coupling from WebRTC health into M1/M2 supervision or safety.
"""

from __future__ import annotations

import os
import unittest

_PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEB_DIR = os.path.join(_PACKAGE_ROOT, "dashboard", "web")


def read(name: str) -> str:
    with open(os.path.join(WEB_DIR, name), encoding="utf-8") as handle:
        return handle.read()


class DashboardWebRTCIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.html = read("index.html")
        cls.app = read("app.js")
        cls.css = read("style.css")
        cls.client = read("webrtc_client.js")

    def camera_section(self) -> str:
        start = self.html.index('id="card-camera"')
        end = self.html.index('id="card-events"')
        return self.html[start:end]

    def test_shared_client_is_loaded_before_app(self):
        self.assertIn('src="/webrtc_client.js"', self.html)
        self.assertIn('src="/app.js"', self.html)
        self.assertLess(
            self.html.index('src="/webrtc_client.js"'),
            self.html.index('src="/app.js"'),
        )

    def test_two_panels_live_inside_existing_camera_card(self):
        camera = self.camera_section()
        for marker in (
            'id="camera-pending"',
            'id="camera-mount"',
            'id="camera-webrtc"',
            'id="webrtc-video-d405"',
            'id="webrtc-video-d455f"',
            'id="webrtc-state-d405"',
            'id="webrtc-state-d455f"',
            'id="webrtc-retry-d405"',
            'id="webrtc-retry-d455f"',
        ):
            with self.subTest(marker=marker):
                self.assertIn(marker, camera)
        self.assertLess(
            camera.index('id="webrtc-video-d405"'),
            camera.index('id="webrtc-video-d455f"'),
        )

    def test_existing_grid_selector_contract_is_unchanged(self):
        self.assertIn("#card-camera { grid-row: 1; grid-column: 3 / 9;", self.css)
        self.assertIn("#card-robots { grid-row: 1; grid-column: 1 / 3;", self.css)
        self.assertIn("#card-events { grid-row: 2; grid-column: 1 / 3;", self.css)
        self.assertIn("#card-comms { grid-row: 2; grid-column: 3 / 6;", self.css)
        self.assertIn("#card-joints { grid-row: 2; grid-column: 6 / 9;", self.css)

    def test_fallback_is_phase_driven_and_unchanged_when_disabled(self):
        self.assertIn('WEBRTC_INACTIVE_PHASES = [', self.app)
        for phase in ("starting", "checking-signaling", "disabled", "stopped"):
            self.assertIn(f'"{phase}"', self.app)
        self.assertIn('phase === "disabled"', self.client)
        self.assertIn("webrtcFrame.hidden = !webrtcVisible", self.app)
        self.assertIn("if (frame) frame.hidden = webrtcVisible", self.app)
        self.assertIn("camera.state === \"pending\" ? \"Pending\"", self.app)

    def test_uses_shared_client_with_independent_cameras(self):
        self.assertIn("window.YAMWebRTC.createController", self.app)
        self.assertIn('const WEBRTC_CAMERAS = ["d405", "d455f"]', self.app)
        self.assertIn('getVideo: (id) => $(`webrtc-video-${id}`)', self.app)
        self.assertIn("onChange: renderWebRTC", self.app)
        self.assertIn("Camera setup is isolated", self.client)

    def test_browser_and_gateway_metrics_render_separately(self):
        self.assertIn("`video ", self.app)
        self.assertIn("`transport ", self.app)
        self.assertIn("`quality ", self.app)
        self.assertIn("`gateway ", self.app)
        self.assertIn("${fps} FPS", self.app)
        self.assertIn("${bitrateMbps} Mbps", self.app)
        self.assertIn("jitter ${jitterMs} ms", self.app)
        self.assertIn("RTT ${rttMs} ms", self.app)
        self.assertIn("${browser.freezeCount || 0} freezes", self.app)
        self.assertIn("${gateway.health", self.app)
        self.assertIn('source: "browser"', self.client)
        self.assertIn('source: "gateway"', self.client)

    def test_transport_metrics_are_not_labelled_glass_to_glass(self):
        self.assertIn("browser estimate", self.html)
        self.assertIn("validated with a physical timing test", self.html)
        self.assertIn("currentRoundTripTime", self.client)
        self.assertIn("framesDropped", self.client)
        self.assertIn("freezeCount", self.client)

    def test_rendered_frame_timing_is_estimated_and_optional(self):
        self.assertIn("requestVideoFrameCallback", self.client)
        self.assertIn("cancelVideoFrameCallback", self.client)
        self.assertIn("metadata.captureTime", self.client)
        self.assertIn("metadata.receiveTime", self.client)
        self.assertIn("metadata.expectedDisplayTime", self.client)
        self.assertIn("cameraToScreenMedianMs", self.client)
        self.assertIn("cameraToScreenP95Ms", self.client)
        self.assertIn("camera timing unavailable", self.app)

    def test_retry_is_selected_camera_only_and_best_effort(self):
        self.assertIn("window.__YAM_WEBRTC__ = {", self.app)
        self.assertIn("webrtcController.retry(id)", self.app)
        self.assertIn("Best effort for the selected camera only", self.app)
        self.assertIn("Retry D405 only", self.html)
        self.assertIn("Retry D455f only", self.html)
        self.assertIn("Best effort only", self.client)
        self.assertIn("verified independent recovery path", self.app)
        self.assertIn("verified recovery path", self.client)

    def test_webrtc_health_stays_out_of_supervision_and_state(self):
        for forbidden in (
            "supervision.webrtc",
            "readiness.prerequisites.push",
            "safety.webrtc",
            'fetch("/api/webrtc',
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, self.app)
        self.assertIn("fetch(\"/api/state\", { cache: \"no-store\" })", self.app)
        self.assertEqual(self.app.count("fetch("), 1)

    def test_cdp_hook_is_private_and_has_no_session_ids(self):
        self.assertIn("window.__YAM_WEBRTC_READY__", self.app)
        self.assertIn("webrtcController.snapshot()", self.app)
        self.assertNotIn("session_id", self.app)
        self.assertNotIn("sessionId", self.html)
        self.assertNotIn("sessionId:", self.client[self.client.index("function snapshot()"):])

    def test_client_remains_media_only(self):
        for forbidden in (
            "avdec_h264", "decodebin", "x264enc", "videoconvert",
            "videoscale", "videorate", "audio",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, self.client)
        self.assertIn("video/h264", self.client)
        self.assertIn("setCodecPreferences", self.client)
        self.assertIn("iceServers: []", self.client)


if __name__ == "__main__":
    unittest.main(verbosity=2)
