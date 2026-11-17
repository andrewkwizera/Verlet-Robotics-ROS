"""Static checks for concise, operational dashboard copy.

These tests guard against project-phase language and long explanatory notes
returning to operator-visible UI assets. Diagnostics and safety guidance must
remain present.
"""

from __future__ import annotations

import os
import unittest

_PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEB_DIR = os.path.join(_PACKAGE_ROOT, "dashboard", "web")

UI_FILES = (
    "index.html",
    "app.js",
    "session.js",
    "style.css",
    "webrtc_client.js",
    "webrtc_diagnostic.html",
    "webrtc_diagnostic.js",
)

FORBIDDEN_COPY = (
    "Milestone",
    "Phase 0",
    "Phase 1",
    "M3 ",
    "research prototype",
    "Test-only",
    "glass-to-glass",
    "GStreamer",
    "HLS",
    "MJPEG",
    "input-driven",
    "Limitation:",
    "not prerequisites",
    "cannot certify",
)


def read(name: str) -> str:
    with open(os.path.join(WEB_DIR, name), encoding="utf-8") as handle:
        return handle.read()


class DashboardUICopyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sources = {name: read(name) for name in UI_FILES}

    def test_no_project_phase_language_in_ui_assets(self):
        for name, source in self.sources.items():
            for phrase in FORBIDDEN_COPY:
                with self.subTest(file=name, phrase=phrase):
                    self.assertNotIn(phrase, source)

    def test_topbar_uses_a_short_operational_badge(self):
        index = self.sources["index.html"]
        self.assertIn('<span class="badge">Read-only</span>', index)
        self.assertIn('title="Safety status"', index)
        self.assertIn('title="Console session"', index)

    def test_camera_fallback_gives_concise_operator_guidance(self):
        app = self.sources["app.js"]
        self.assertIn('setText("camera-title", "Camera feed unavailable")', app)
        self.assertIn(
            "Connect the video gateway to show D405 and D455f feeds.", app
        )

    def test_drawer_copy_is_concise(self):
        index = self.sources["index.html"]
        self.assertIn("Reasons, checks and events", index)
        self.assertIn("Diagnostics (non-blocking)", index)
        self.assertIn("DIAGNOSTIC", self.sources["app.js"])

    def test_safety_and_session_guidance_are_preserved(self):
        self.assertIn(
            "this console does not certify safety", self.sources["app.js"]
        )
        self.assertIn("no robot effect", self.sources["index.html"])
        self.assertIn("no robot effect", self.sources["session.js"])

    def test_diagnostic_page_copy_is_neutral(self):
        html = self.sources["webrtc_diagnostic.html"]
        self.assertIn("<title>YAM WebRTC gateway diagnostic</title>", html)
        self.assertIn("<h1>WebRTC gateway diagnostic</h1>", html)
        self.assertIn("Independent feeds · H.264 passthrough · no transcoding", html)


if __name__ == "__main__":
    unittest.main(verbosity=2)
