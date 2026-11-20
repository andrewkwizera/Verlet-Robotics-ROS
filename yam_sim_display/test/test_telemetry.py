"""Unit tests for the pure dashboard telemetry layer.

Runs without ROS: ``python3 -m unittest discover -s test`` from the package
root, or ``python3 test/test_telemetry.py``.
"""

import math
import os
import sys
import unittest

_PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, _PACKAGE_ROOT)

from dashboard.config import DEFAULTS, DashboardConfig  # noqa: E402
from dashboard.telemetry import (  # noqa: E402
    STATE_CONNECTED,
    STATE_DISCONNECTED,
    STATE_STALE,
    STATE_WARNING,
    EventLog,
    JointStream,
    TelemetryStore,
    compute_rate,
    compute_tracking,
    worst_state,
)

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


class FakeClock:
    def __init__(self, start: float = 1000.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def make_config(**overrides) -> DashboardConfig:
    values = dict(DEFAULTS)
    values.update(overrides)
    return DashboardConfig.from_parameter_dict(values)


def pose(*values):
    return dict(zip(JOINTS, values))


# --------------------------------------------------------------------- config


class ConfigTests(unittest.TestCase):
    def test_defaults_are_valid(self):
        config = DashboardConfig.from_parameter_dict({})
        self.assertEqual(config.leader_topic, "/leader/joint_states")
        self.assertEqual(config.follower_topic, "/follower/joint_states")
        self.assertEqual(config.port, 8080)
        self.assertEqual(config.bind_address, "127.0.0.1")
        self.assertEqual(config.network_rtt_host, "")
        self.assertFalse(config.webrtc_enabled)
        self.assertLess(config.warning_s, config.stale_s)
        self.assertLess(config.stale_s, config.disconnect_s)

    def test_threshold_order_enforced(self):
        with self.assertRaises(ValueError):
            make_config(**{
                "freshness.warning_ms": 2000.0,
                "freshness.stale_ms": 1000.0,
            })

    def test_invalid_port_rejected(self):
        with self.assertRaises(ValueError):
            make_config(port=70000)

    def test_empty_bind_rejected(self):
        with self.assertRaises(ValueError):
            make_config(bind_address="   ")

    def test_optional_network_rtt_host_is_validated(self):
        config = make_config(network_rtt_host="192.0.2.20")
        self.assertEqual(config.network_rtt_host, "192.0.2.20")
        with self.assertRaises(ValueError):
            make_config(network_rtt_host="not-an-ip")

    def test_rate_band_enforced(self):
        with self.assertRaises(ValueError):
            make_config(**{
                "diagnostics.rate_min_hz": 80.0,
                "diagnostics.rate_max_hz": 20.0,
            })


# ----------------------------------------------------------------------- rate


class RateTests(unittest.TestCase):
    def test_no_rate_from_single_sample(self):
        self.assertIsNone(compute_rate([1.0], 1.0, 2.0))

    def test_rate_from_uniform_samples(self):
        times = [i * 0.02 for i in range(51)]  # 50 Hz over 1 s
        rate = compute_rate(times, times[-1], 2.0)
        self.assertAlmostEqual(rate, 50.0, places=6)

    def test_rate_ignores_samples_outside_window(self):
        times = [0.0, 0.02, 0.04, 10.0, 10.02]
        rate = compute_rate(times, 10.02, 1.0)
        self.assertAlmostEqual(rate, 50.0, places=6)

    def test_zero_span_is_not_a_rate(self):
        self.assertIsNone(compute_rate([5.0, 5.0], 5.0, 2.0))


# --------------------------------------------------------------------- stream


class JointStreamTests(unittest.TestCase):
    def setUp(self):
        self.stream = JointStream("leader")
        self.clock = FakeClock()

    def test_initial_state_is_disconnected(self):
        self.assertEqual(self.stream.state, STATE_DISCONNECTED)
        self.assertIsNone(self.stream.age_ms(self.clock.now))

    def test_full_state_progression(self):
        self.stream.update(pose(*([0.0] * 6)), None, self.clock.now)
        self.stream.update_state(self.clock.now, 0.1, 0.2, 0.5)
        self.assertEqual(self.stream.state, STATE_CONNECTED)

        self.clock.advance(0.15)
        self.stream.update_state(self.clock.now, 0.1, 0.2, 0.5)
        self.assertEqual(self.stream.state, STATE_WARNING)

        self.clock.advance(0.10)
        self.stream.update_state(self.clock.now, 0.1, 0.2, 0.5)
        self.assertEqual(self.stream.state, STATE_STALE)

        self.clock.advance(0.40)
        self.stream.update_state(self.clock.now, 0.1, 0.2, 0.5)
        self.assertEqual(self.stream.state, STATE_DISCONNECTED)

    def test_rejects_wrong_joint_count(self):
        self.stream.update(pose(*([0.0] * 6)), None, self.clock.now)
        accepted = self.stream.update({"joint_1": 1.0}, None, self.clock.now)
        self.assertFalse(accepted)
        self.assertEqual(self.stream.rejected, 1)

    def test_rejects_non_finite_values(self):
        accepted = self.stream.update(
            {"joint_1": float("nan")}, None, self.clock.now
        )
        self.assertFalse(accepted)
        self.assertEqual(self.stream.count, 0)

    def test_source_stamp_gap(self):
        self.stream.update(pose(*([0.0] * 6)), 1_000_000_000, self.clock.now)
        self.stream.update(pose(*([0.0] * 6)), 1_020_000_000, self.clock.now)
        self.assertAlmostEqual(self.stream.source_dt_ms, 20.0)

    def test_out_of_order_stamp_keeps_latest(self):
        self.stream.update(pose(*([0.0] * 6)), 2_000_000_000, self.clock.now)
        self.stream.update(pose(*([0.0] * 6)), 1_000_000_000, self.clock.now)
        self.assertEqual(self.stream.source_stamp_ns, 2_000_000_000)
        self.assertIsNone(self.stream.source_dt_ms)

    def test_static_joint_detection(self):
        for value in (0.1, 0.2, 0.3):
            self.stream.update(
                pose(value, 0.0, 0.0, 0.0, 0.0, 0.0), None, self.clock.now
            )
        static = self.stream.static_joints(0.001, min_samples=1)
        self.assertEqual(
            static, ["joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        )

    def test_static_detection_needs_min_samples(self):
        self.stream.update(pose(*([0.0] * 6)), None, self.clock.now)
        self.assertEqual(self.stream.static_joints(0.001, min_samples=50), [])


# ------------------------------------------------------------------ tracking


class TrackingTests(unittest.TestCase):
    def test_per_joint_and_aggregates(self):
        leader = pose(0.10, 0.20, 0.30, 0.0, 0.0, 0.0)
        follower = pose(0.08, 0.20, 0.24, 0.0, 0.0, 0.0)
        result = compute_tracking(leader, follower, static_joints=["joint_4"])

        self.assertTrue(result["available"])
        self.assertAlmostEqual(result["max_abs_error_rad"], 0.06)
        self.assertEqual(result["max_abs_error_joint"], "joint_3")
        expected_rms = math.sqrt((0.02 ** 2 + 0.0 + 0.06 ** 2) / 6)
        self.assertAlmostEqual(result["rms_error_rad"], expected_rms)

        joint_3 = next(r for r in result["per_joint"] if r["name"] == "joint_3")
        self.assertAlmostEqual(joint_3["error_mrad"], 60.0)
        joint_4 = next(r for r in result["per_joint"] if r["name"] == "joint_4")
        self.assertTrue(joint_4["static"])

    def test_returns_none_without_shared_joints(self):
        self.assertIsNone(compute_tracking({"a": 0.0}, {"b": 0.0}))

    def test_worst_state(self):
        self.assertEqual(
            worst_state([STATE_CONNECTED, STATE_STALE]), STATE_STALE
        )
        self.assertEqual(
            worst_state([STATE_DISCONNECTED, STATE_WARNING]), STATE_DISCONNECTED
        )


# --------------------------------------------------------------------- events


class EventLogTests(unittest.TestCase):
    def test_log_is_bounded(self):
        log = EventLog(max_entries=3)
        for index in range(10):
            log.add("info", "test", f"event {index}")
        self.assertEqual(len(log), 3)
        self.assertEqual(log.snapshot()[-1]["message"], "event 9")

    def test_ids_are_monotonic(self):
        log = EventLog(max_entries=5)
        log.add("info", "test", "first")
        log.add("info", "test", "second")
        ids = [event["id"] for event in log.snapshot()]
        self.assertEqual(ids, [1, 2])


# ---------------------------------------------------------------------- store


class StoreTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.config = make_config(**{
            "freshness.warning_ms": 100.0,
            "freshness.stale_ms": 200.0,
            "freshness.disconnect_ms": 500.0,
        })
        self.store = TelemetryStore(self.config, clock=self.clock)

    def _feed(self, role, joints=None, stamp=None):
        values = joints or pose(*([0.0] * 6))
        self.store.on_joint_state(
            role,
            JOINTS,
            [values[name] for name in JOINTS],
            source_stamp_ns=stamp,
            now=self.clock.now,
        )

    def test_snapshot_before_any_data(self):
        snapshot = self.store.evaluate()
        self.assertEqual(snapshot["system"]["state"], STATE_DISCONNECTED)
        self.assertFalse(snapshot["tracking"]["available"])
        self.assertFalse(snapshot["latency"]["available"])
        self.assertIn("Unavailable", snapshot["latency"]["display"])
        self.assertEqual(snapshot["camera"]["state"], "pending")

    def test_connection_event_emitted_once(self):
        self._feed("leader")
        self._feed("follower")
        self.store.evaluate()
        connected = [
            event for event in self.store.events.snapshot()
            if "connected" in event["message"]
        ]
        self.assertEqual(len(connected), 2)

        for _ in range(20):
            self.store.evaluate()
        connected_again = [
            event for event in self.store.events.snapshot()
            if "connected" in event["message"]
        ]
        self.assertEqual(len(connected_again), 2)

    def test_stale_then_recovered_events(self):
        self._feed("leader")
        self._feed("follower")
        self.store.evaluate()

        self.clock.advance(0.25)
        snapshot = self.store.evaluate()
        self.assertEqual(snapshot["streams"]["leader"]["state"], STATE_STALE)

        messages = [event["message"] for event in self.store.events.snapshot()]
        self.assertTrue(any("stale" in message for message in messages))

        self._feed("leader")
        self.store.evaluate()
        messages = [event["message"] for event in self.store.events.snapshot()]
        self.assertTrue(any("recovered" in message for message in messages))

    def test_tracking_unavailable_when_one_stream_stale(self):
        self._feed("leader", pose(0.1, 0.0, 0.0, 0.0, 0.0, 0.0))
        self._feed("follower", pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.store.evaluate()
        self.assertTrue(self.store.snapshot()["tracking"]["available"])

        self.clock.advance(0.25)
        snapshot = self.store.evaluate()
        self.assertFalse(snapshot["tracking"]["available"])

    def test_tracking_error_reported(self):
        self._feed("leader", pose(0.10, 0.20, 0.30, 0.0, 0.0, 0.0))
        self._feed("follower", pose(0.08, 0.20, 0.24, 0.0, 0.0, 0.0))
        snapshot = self.store.evaluate()
        tracking = snapshot["tracking"]
        self.assertAlmostEqual(tracking["max_abs_error_rad"], 0.06)
        self.assertEqual(tracking["max_abs_error_joint"], "joint_3")

    def test_tracking_threshold_event_fires_once(self):
        for _ in range(3):
            self._feed("leader", pose(0.50, 0.0, 0.0, 0.0, 0.0, 0.0))
            self._feed("follower", pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
            self.store.evaluate()

        alerts = [
            event for event in self.store.events.snapshot()
            if event["source"] == "tracking"
        ]
        self.assertEqual(len(alerts), 1)

    def test_camera_and_latency_are_honest(self):
        self._feed("leader")
        self._feed("follower")
        snapshot = self.store.evaluate()
        self.assertFalse(snapshot["latency"]["available"])
        self.assertIsNone(snapshot["camera"]["stream_url"])
        self.assertIn("GStreamer", snapshot["camera"]["reason"])

    def test_first_connection_with_backlog_is_not_a_reconnect(self):
        # A RELIABLE subscriber can receive its publisher's history in one
        # burst; that must still read as the first connection.
        for _ in range(5):
            self._feed("leader")
        self._feed("follower")
        self.store.evaluate()

        messages = [event["message"] for event in self.store.events.snapshot()]
        self.assertTrue(any("Leader telemetry connected" in m for m in messages), messages)
        self.assertFalse(
            any("recovered after disconnect" in m for m in messages), messages
        )

    def test_no_age_events_before_first_connection(self):
        self.clock.advance(10.0)
        snapshot = self.store.evaluate()
        self.assertEqual(snapshot["streams"]["leader"]["state"], STATE_DISCONNECTED)
        self.assertEqual(len(self.store.events), 0)

    def test_rate_alert_suppressed_during_initial_window(self):
        # 20 messages delivered in a burst: rate would look enormous, but no
        # full observation window has elapsed yet.
        for _ in range(20):
            self._feed("leader")
            self._feed("follower")
        self.store.evaluate()
        self.assertEqual(
            [e for e in self.store.events.snapshot() if "publish rate" in e["message"]],
            [],
        )

    def test_rate_alert_after_window_with_fast_stream(self):
        config = make_config(**{
            "freshness.warning_ms": 100.0,
            "freshness.stale_ms": 200.0,
            "freshness.disconnect_ms": 500.0,
            "diagnostics.rate_window_s": 1.0,
        })
        store = TelemetryStore(config, clock=self.clock)
        # 200 Hz stream: 201 samples over 1 s.
        for index in range(201):
            store.on_joint_state(
                "leader", JOINTS, [0.0] * 6, None, now=self.clock.now
            )
            self.clock.now += 0.005
        store.evaluate()
        messages = [e["message"] for e in store.events.snapshot()]
        self.assertTrue(any("publish rate" in m and "outside" in m for m in messages), messages)

    def test_rate_alert_evaluated_while_tracking_is_available(self):
        # Regression: an early ``return`` in the tracking branch used to skip
        # the rate-band check on every tick where tracking did not transition.
        config = make_config(**{
            "freshness.warning_ms": 100.0,
            "freshness.stale_ms": 200.0,
            "freshness.disconnect_ms": 500.0,
            "diagnostics.rate_window_s": 1.0,
        })
        store = TelemetryStore(config, clock=self.clock)
        for _ in range(201):
            for role in ("leader", "follower"):
                store.on_joint_state(
                    role, JOINTS, [0.0] * 6, None, now=self.clock.now
                )
            self.clock.now += 0.005
        snapshot = store.evaluate()
        self.assertTrue(snapshot["tracking"]["available"])

        messages = [e["message"] for e in store.events.snapshot()]
        rate_events = [m for m in messages if "publish rate" in m and "outside" in m]
        self.assertEqual(len(rate_events), 2, messages)
        self.assertEqual(
            [e for e in store.events.snapshot() if e["source"] == "tracking"], []
        )

    def test_tracking_events_are_not_derived_from_stale_poses(self):
        self._feed("leader", pose(0.50, 0.0, 0.0, 0.0, 0.0, 0.0))
        self._feed("follower", pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.store.evaluate()
        first = [
            e for e in self.store.events.snapshot() if e["source"] == "tracking"
        ]
        self.assertEqual(len(first), 1)

        # Freeze both streams; the cached pose must not keep raising events.
        self.clock.advance(0.30)
        for _ in range(10):
            self.store.evaluate()
        stale_events = [
            e for e in self.store.events.snapshot() if e["source"] == "tracking"
        ]
        self.assertEqual(len(stale_events), 1, stale_events)

        # Fresh telemetry returns with the error still out of bounds: that is
        # a new crossing and must be reported once (alert was re-armed).
        self._feed("leader", pose(0.50, 0.0, 0.0, 0.0, 0.0, 0.0))
        self._feed("follower", pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.store.evaluate()
        recovered = [
            e for e in self.store.events.snapshot() if e["source"] == "tracking"
        ]
        self.assertEqual(len(recovered), 2, recovered)

    def test_rate_band_is_not_evaluated_on_a_stale_stream(self):
        config = make_config(**{
            "freshness.warning_ms": 100.0,
            "freshness.stale_ms": 200.0,
            "freshness.disconnect_ms": 500.0,
            "diagnostics.rate_window_s": 1.0,
        })
        store = TelemetryStore(config, clock=self.clock)
        for _ in range(201):  # 200 Hz: immediately out of band
            for role in ("leader", "follower"):
                store.on_joint_state(
                    role, JOINTS, [0.0] * 6, None, now=self.clock.now
                )
            self.clock.now += 0.005
        store.evaluate()
        self.assertTrue(
            any("outside" in e["message"] for e in store.events.snapshot())
        )

        # The stream now goes stale. Rate must not be re-evaluated (and must
        # not emit a bogus "back at 0 Hz" recovery) from a frozen stream.
        self.clock.advance(0.30)
        for _ in range(10):
            store.evaluate()
        rate_events = [
            e["message"] for e in store.events.snapshot()
            if "publish rate" in e["message"]
        ]
        self.assertEqual(len(rate_events), 2, rate_events)
        self.assertFalse(any("back at" in m for m in rate_events), rate_events)

    def test_reconnect_after_disconnect_is_reported(self):
        self._feed("leader")
        self._feed("follower")
        self.store.evaluate()

        self.clock.advance(0.6)  # past the 500 ms disconnect threshold
        snapshot = self.store.evaluate()
        self.assertEqual(
            snapshot["streams"]["leader"]["state"], STATE_DISCONNECTED
        )

        self._feed("leader")
        snapshot = self.store.evaluate()
        self.assertEqual(snapshot["streams"]["leader"]["state"], STATE_CONNECTED)

        messages = [event["message"] for event in self.store.events.snapshot()]
        self.assertTrue(
            any("recovered after disconnect" in message for message in messages),
            messages,
        )
        # The original connection must not be re-announced.
        self.assertEqual(
            len([m for m in messages if "telemetry connected" in m]), 2, messages
        )

    def test_warning_event_never_calls_message_age_latency(self):
        self._feed("leader")
        self._feed("follower")
        self.store.evaluate()

        self.clock.advance(0.15)
        self.store.evaluate()

        warnings = [
            event for event in self.store.events.snapshot()
            if event["level"] == "warning" and event["source"] == "leader"
        ]
        self.assertEqual(len(warnings), 1)
        self.assertIn("message age", warnings[0]["message"])
        self.assertNotIn("latency", warnings[0]["message"].lower())

    def test_snapshot_contains_config_for_frontend(self):
        snapshot = self.store.evaluate()
        self.assertEqual(snapshot["config"]["refresh_ms"], 100)

    def test_static_joints_flagged_in_tracking(self):
        config = make_config(**{
            "freshness.warning_ms": 100.0,
            "freshness.stale_ms": 200.0,
            "freshness.disconnect_ms": 500.0,
            "diagnostics.static_joint_min_samples": 1,
        })
        store = TelemetryStore(config, clock=self.clock)
        for index in range(4):
            # Joints 1-3 move; joints 4-6 stay constant, as in the simulation.
            moving = [0.05 * index, 0.10 * index, 0.15 * index, 0.0, 0.0, 0.0]
            store.on_joint_state("leader", JOINTS, moving, None, now=self.clock.now)
            store.on_joint_state("follower", JOINTS, moving, None, now=self.clock.now)
        snapshot = store.evaluate()
        self.assertEqual(
            snapshot["tracking"]["static_joints"],
            ["joint_4", "joint_5", "joint_6"],
        )
        self.assertIn("static", snapshot["tracking"]["note"].lower())


class DashboardYamlTests(unittest.TestCase):
    """Guards the on-disk config against ROS parameter type mismatches."""

    @staticmethod
    def _flatten(prefix, node, out):
        for key, value in node.items():
            name = f"{prefix}.{key}" if prefix else key
            if isinstance(value, dict):
                DashboardYamlTests._flatten(name, value, out)
            else:
                out[name] = value
        return out

    def test_yaml_keys_are_known_and_typed_correctly(self):
        try:
            import yaml
        except ImportError:  # pragma: no cover - PyYAML ships with ROS 2
            self.skipTest("PyYAML not available")

        path = os.path.join(_PACKAGE_ROOT, "config", "dashboard.yaml")
        with open(path, encoding="utf-8") as handle:
            document = yaml.safe_load(handle)

        section = document["yam_dashboard"]["ros__parameters"]
        flat = self._flatten("", section, {})

        for name, value in flat.items():
            self.assertIn(name, DEFAULTS, f"{name} is not a declared parameter")
            default = DEFAULTS[name]
            # bool is an int subclass; no boolean parameters exist here.
            if isinstance(default, bool):
                self.assertIsInstance(value, bool, name)
            elif isinstance(default, float):
                self.assertIsInstance(value, float, name)
                self.assertNotIsInstance(value, bool)
            elif isinstance(default, int):
                self.assertIsInstance(value, int, name)
                self.assertNotIsInstance(value, bool)
            else:
                self.assertIsInstance(value, str, name)

    def test_yaml_config_parses(self):
        try:
            import yaml
        except ImportError:  # pragma: no cover
            self.skipTest("PyYAML not available")

        path = os.path.join(_PACKAGE_ROOT, "config", "dashboard.yaml")
        with open(path, encoding="utf-8") as handle:
            document = yaml.safe_load(handle)

        flat = self._flatten(
            "", document["yam_dashboard"]["ros__parameters"], {}
        )
        config = DashboardConfig.from_parameter_dict(flat)
        self.assertEqual(config.port, DEFAULTS["port"])
        self.assertEqual(config.bind_address, DEFAULTS["bind_address"])
        self.assertLess(config.warning_s, config.stale_s)


if __name__ == "__main__":
    unittest.main(verbosity=2)
