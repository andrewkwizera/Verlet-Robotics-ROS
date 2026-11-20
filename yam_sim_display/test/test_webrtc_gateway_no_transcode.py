"""Static check that the gateway media path cannot transcode."""

from __future__ import annotations

import json
import os
import re
import sys
import unittest

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_PACKAGE_ROOT = os.path.dirname(_TEST_DIR)
if _PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, _PACKAGE_ROOT)

BANNED_FACTORIES = {
    "avdec_h264", "decodebin", "decodebin3", "openh264dec", "vah264dec",
    "vaapih264dec", "nvh264dec", "vtdec", "x264enc", "openh264enc",
    "vah264enc", "vaapih264enc", "nvh264enc", "venc", "encodebin",
    "videoconvert", "videoscale", "videorate",
}
REQUIRED_FACTORIES = {
    "udpsrc", "rtpjitterbuffer", "rtph264depay", "h264parse",
    "rtph264pay", "queue", "webrtcbin",
}


class NoTranscodeTests(unittest.TestCase):
    def test_camera_pipeline_contains_only_passthrough_factories(self):
        source_path = os.path.join(
            _PACKAGE_ROOT, "gateway", "camera.py"
        )
        with open(source_path, encoding="utf-8") as handle:
            source = handle.read()

        factories = set(re.findall(
            r"(?:self\._make|Gst\.ElementFactory\.make)\(\s*['\"]([^'\"]+)",
            source,
        ))
        self.assertTrue(REQUIRED_FACTORIES.issubset(factories))
        self.assertFalse(factories & BANNED_FACTORIES)
        self.assertIn("media=video,encoding-name=H264", source)
        self.assertIn('self.queue.set_property("leaky", 2)', source)
        self.assertNotIn('max-size-buffers", 300', source)

    def test_gateway_does_not_import_ros(self):
        for name in ("gateway/camera.py", "gateway/gateway.py", "gateway/run.py"):
            with self.subTest(name=name):
                path = os.path.join(_PACKAGE_ROOT, name)
                with open(path, encoding="utf-8") as handle:
                    source = handle.read()
                self.assertNotIn("import rclpy", source)
                self.assertNotIn("import rcl_interfaces", source)

    def test_default_config_keeps_expected_udp_ports(self):
        path = os.path.join(
            _PACKAGE_ROOT, "config", "webrtc_gateway.json"
        )
        with open(path, encoding="utf-8") as handle:
            document = json.load(handle)
        self.assertEqual(document["cameras"]["d405"]["port"], 5100)
        self.assertEqual(document["cameras"]["d455f"]["port"], 5102)


if __name__ == "__main__":
    unittest.main(verbosity=2)
