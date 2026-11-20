"""Configuration tests for the standalone gateway (pure stdlib, no ROS/Gst)."""

from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
import unittest

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_PACKAGE_ROOT = os.path.dirname(_TEST_DIR)
if _PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, _PACKAGE_ROOT)

from gateway.config import (  # noqa: E402
    CAMERA_IDS,
    DEFAULT_CONFIG,
    GatewayConfig,
    from_args,
    parse_camera_spec,
)


class GatewayConfigTests(unittest.TestCase):
    def test_defaults_use_expected_camera_ports(self):
        config = GatewayConfig.default()
        self.assertEqual(config.signaling_url, "http://127.0.0.1:8080")
        self.assertEqual(CAMERA_IDS, ("d405", "d455f"))
        self.assertEqual(
            {camera.camera_id: camera.port for camera in config.cameras},
            {"d405": 5100, "d455f": 5102},
        )
        self.assertEqual(
            {camera.camera_id: camera.host for camera in config.cameras},
            {"d405": "127.0.0.1", "d455f": "127.0.0.1"},
        )
        self.assertTrue(config.run_dir)
        self.assertEqual(config.jitter_buffer_ms, 30)
        self.assertEqual(config.queue_max_packets, 32)

    def test_load_from_json_file(self):
        document = {
            "signaling_url": "http://10.0.0.2:8080/",
            "cameras": {"d405": {"host": "10.0.0.3", "port": 6000}},
            "poll_timeout_s": 2.5,
        }
        with tempfile.NamedTemporaryFile(
            "w", suffix=".json", delete=False, encoding="utf-8"
        ) as handle:
            json.dump(document, handle)
            path = handle.name
        try:
            config = GatewayConfig.from_file(path)
            self.assertEqual(config.signaling_url, "http://10.0.0.2:8080")
            self.assertEqual(len(config.cameras), 1)
            self.assertEqual(config.cameras[0].host, "10.0.0.3")
            self.assertEqual(config.cameras[0].port, 6000)
            self.assertEqual(config.poll_timeout_s, 2.5)
        finally:
            os.unlink(path)

    def test_cli_overrides_are_additive(self):
        config = from_args([
            "--signaling", "http://127.0.0.1:18080",
            "--camera", "d405:192.168.1.20:6000",
            "--stats-interval", "0.25",
            "--run-dir", "/tmp/yam-test-gateway",
        ])
        self.assertEqual(config.signaling_url, "http://127.0.0.1:18080")
        self.assertEqual(config.run_dir, "/tmp/yam-test-gateway")
        self.assertEqual(config.stats_interval_s, 0.25)
        by_id = {camera.camera_id: camera for camera in config.cameras}
        self.assertEqual(by_id["d405"].host, "192.168.1.20")
        self.assertEqual(by_id["d405"].port, 6000)
        self.assertEqual(by_id["d455f"].port, 5102)

    def test_parse_camera_spec(self):
        self.assertEqual(
            parse_camera_spec("d455f:10.1.2.3:5002"),
            ("d455f", "10.1.2.3", 5002),
        )
        for value in ("d405", "nope:127.0.0.1:5000", "d405:host:abc"):
            with self.subTest(value=value):
                with self.assertRaises((ValueError, argparse.ArgumentTypeError)):
                    parse_camera_spec(value)

    def test_only_camera_filters_other_defaults(self):
        config = from_args(["--only-camera", "d455f"])
        self.assertEqual(
            [camera.camera_id for camera in config.cameras], ["d455f"]
        )
        self.assertEqual(config.cameras[0].port, 5102)

    def test_validation(self):
        invalid_documents = [
            {"signaling_url": "ftp://example"},
            {"cameras": {}},
            {"cameras": {"other": {"host": "x", "port": 5000}}},
            {"cameras": {"d405": {"host": "", "port": 5000}}},
            {"cameras": {"d405": {"host": "x", "port": 70000}}},
            {"poll_timeout_s": 40.0},
            {"keepalive_interval_s": 0.0},
            {"stats_interval_s": -1.0},
            {"jitter_buffer_ms": -1},
            {"jitter_buffer_ms": 501},
            {"queue_max_packets": 0},
            {"queue_max_packets": 1001},
        ]
        for document in invalid_documents:
            with self.subTest(document=document):
                data = dict(DEFAULT_CONFIG)
                data.update(document)
                with self.assertRaises(ValueError):
                    GatewayConfig.from_mapping(data)


if __name__ == "__main__":
    unittest.main(verbosity=2)
