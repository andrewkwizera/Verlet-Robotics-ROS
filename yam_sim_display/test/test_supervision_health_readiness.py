"""Health and readiness rules (pure Python, no ROS)."""

from __future__ import annotations

import os
import sys
import unittest

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
if _TEST_DIR not in sys.path:
    sys.path.insert(0, _TEST_DIR)

from supervision_testkit import (  # noqa: E402
    FakeSafetyInput,
    make_engine,
    make_snapshot,
)
from dashboard.supervision.safety import SafetyReading  # noqa: E402


def normal_forever() -> SafetyReading:
    return SafetyReading(state="normal", detail="test", source="test-safety")


def codes(payload: dict) -> list:
    return [reason["code"] for reason in payload.get("reasons", [])]


class HealthTests(unittest.TestCase):
    def setUp(self):
        self.engine, self.events, self.clock = make_engine()

    def evaluate(self, snapshot=None):
        return self.engine.evaluate(
            snapshot or make_snapshot(), self.clock.now
        )["health"]

    def test_both_streams_connected_is_healthy(self):
        payload = self.evaluate()
        self.assertEqual(payload["state"], "healthy")
        self.assertEqual(payload["label"], "Healthy")
        self.assertEqual(payload["reasons"], [])
        self.assertEqual(payload["evidence"]["leader_state"], "connected")

    def test_disconnected_stream_makes_health_disconnected(self):
        payload = self.evaluate(make_snapshot(leader_state="disconnected"))
        self.assertEqual(payload["state"], "disconnected")
        self.assertIn("health.leader.disconnected", codes(payload))

    def test_warning_stream_makes_health_degraded(self):
        payload = self.evaluate(make_snapshot(follower_state="warning"))
        self.assertEqual(payload["state"], "degraded")
        self.assertIn("health.follower.warning", codes(payload))

    def test_rate_out_of_band_is_degraded(self):
        payload = self.evaluate(make_snapshot(leader_rate=120.0))
        self.assertEqual(payload["state"], "degraded")
        self.assertIn("health.leader.rate_out_of_band", codes(payload))

    def test_rejected_frame_delta_degrades_then_recovers(self):
        self.evaluate(make_snapshot(leader_rejected=0))
        payload = self.evaluate(make_snapshot(leader_rejected=2))
        self.assertEqual(payload["state"], "degraded")
        self.assertIn("health.leader.rejected_frames", codes(payload))

        self.clock.advance(6.0)
        payload = self.evaluate(make_snapshot(leader_rejected=2))
        self.assertEqual(payload["state"], "healthy")
        self.assertNotIn("health.leader.rejected_frames", codes(payload))

    def test_source_stamp_gap_is_degraded(self):
        payload = self.evaluate(make_snapshot(leader_dt=900.0))
        self.assertEqual(payload["state"], "degraded")
        self.assertIn("health.leader.source_stamp_gap", codes(payload))

    def test_reasons_are_sorted_by_severity(self):
        payload = self.evaluate(make_snapshot(
            leader_state="disconnected", follower_rate=120.0
        ))
        self.assertEqual(payload["state"], "disconnected")
        found = codes(payload)
        self.assertEqual(found[0], "health.leader.disconnected")
        self.assertIn("health.follower.rate_out_of_band", found)

    def test_since_duration_advances(self):
        first = self.evaluate()
        self.assertEqual(first["since_uptime_s"], 0.0)
        self.clock.advance(2.5)
        second = self.evaluate()
        self.assertAlmostEqual(second["since_uptime_s"], 2.5, places=3)


class ReadinessTests(unittest.TestCase):
    def setUp(self):
        self.safety = FakeSafetyInput(normal_forever())

    def engine(self, safety=None):
        return make_engine(safety_input=safety if safety is not None else self.safety)

    @staticmethod
    def readiness(supervisor, clock, snapshot=None):
        return supervisor.evaluate(
            snapshot or make_snapshot(), clock.now
        )["readiness"]

    def test_all_pass_requires_ready_hold(self):
        supervisor, _, clock = self.engine()
        first = self.readiness(supervisor, clock)
        self.assertEqual(first["state"], "initializing")
        clock.advance(0.99)
        self.assertEqual(self.readiness(supervisor, clock)["state"], "initializing")
        clock.advance(0.02)
        self.assertEqual(self.readiness(supervisor, clock)["state"], "ready")

    def test_unknown_safety_blocks_readiness(self):
        supervisor, _, clock = self.engine(safety=None if False else FakeSafetyInput(None))
        # Explicitly use the shipped NullSafetyInput instead.
        from dashboard.supervision.safety import NullSafetyInput
        supervisor, _, clock = make_engine(safety_input=NullSafetyInput())
        first = self.readiness(supervisor, clock)
        self.assertEqual(first["state"], "initializing")
        clock.advance(5.0)
        payload = self.readiness(supervisor, clock)
        self.assertEqual(payload["state"], "not_ready")
        self.assertEqual(codes(payload), ["readiness.safety_normal"])
        self.assertIn("safety status is unknown", payload["reasons"][0]["message"])

        clock.advance(60.0)
        self.assertEqual(self.readiness(supervisor, clock)["state"], "not_ready")

    def test_startup_grace_bounds_initializing(self):
        supervisor, _, clock = self.engine()
        failing = make_snapshot(leader_state="disconnected")
        self.assertEqual(self.readiness(supervisor, clock, failing)["state"], "initializing")
        clock.advance(4.9)
        self.assertEqual(self.readiness(supervisor, clock, failing)["state"], "initializing")
        clock.advance(0.2)
        self.assertEqual(self.readiness(supervisor, clock, failing)["state"], "not_ready")

    def test_safety_fault_is_an_immediate_failure(self):
        input_ = FakeSafetyInput(SafetyReading(state="fault", source="test-safety"))
        supervisor, _, clock = self.engine(input_)
        payload = self.readiness(supervisor, clock)
        self.assertEqual(payload["state"], "not_ready")
        self.assertEqual(codes(payload), ["readiness.safety_normal"])

    def test_losing_a_prerequisite_drops_ready_immediately(self):
        supervisor, _, clock = self.engine()
        self.readiness(supervisor, clock)
        clock.advance(1.0)
        self.assertEqual(self.readiness(supervisor, clock)["state"], "ready")

        payload = self.readiness(
            supervisor, clock, make_snapshot(leader_state="warning")
        )
        self.assertEqual(payload["state"], "not_ready")
        self.assertEqual(codes(payload), ["readiness.leader_fresh"])

    def test_hold_resets_after_a_failure(self):
        supervisor, _, clock = self.engine()
        self.readiness(supervisor, clock)          # initializing, all pass since t0
        clock.advance(0.5)
        self.readiness(supervisor, clock)          # 0.5 s held
        failing = make_snapshot(follower_state="warning")
        self.readiness(supervisor, clock, failing)  # reset
        self.readiness(supervisor, clock)          # restart hold at t0+0.5
        clock.advance(0.5)
        self.assertEqual(self.readiness(supervisor, clock)["state"], "initializing")
        clock.advance(0.6)
        self.assertEqual(self.readiness(supervisor, clock)["state"], "ready")

    def test_each_prerequisite_can_fail_alone(self):
        cases = {
            "leader_fresh": make_snapshot(leader_state="warning"),
            "follower_fresh": make_snapshot(follower_state="warning"),
            "rates_in_band": make_snapshot(leader_rate=5.0),
            "joint_set_complete": make_snapshot(joint_count=5),
            "tracking_available": make_snapshot(tracking=False),
        }
        for expected, snapshot in cases.items():
            with self.subTest(prerequisite=expected):
                supervisor, _, clock = self.engine()
                self.readiness(supervisor, clock, snapshot)  # start grace clock
                clock.advance(6.0)
                payload = self.readiness(supervisor, clock, snapshot)
                self.assertEqual(payload["state"], "not_ready")
                failing = [
                    item["id"] for item in payload["prerequisites"]
                    if item["required"] and not item["passed"]
                ]
                self.assertEqual(failing, [expected])
                self.assertEqual(codes(payload), [f"readiness.{expected}"])

        # The safety prerequisite has no telemetry analogue.
        from dashboard.supervision.safety import NullSafetyInput
        supervisor, _, clock = make_engine(safety_input=NullSafetyInput())
        self.readiness(supervisor, clock)  # start grace clock
        clock.advance(6.0)
        payload = self.readiness(supervisor, clock)
        failing = [
            item["id"] for item in payload["prerequisites"]
            if item["required"] and not item["passed"]
        ]
        self.assertEqual(failing, ["safety_normal"])

    def test_alignment_is_diagnostic_not_a_prerequisite(self):
        supervisor, _, clock = self.engine()
        snapshot = make_snapshot(max_error=0.50)
        self.readiness(supervisor, clock, snapshot)
        clock.advance(1.0)
        payload = self.readiness(supervisor, clock, snapshot)
        self.assertEqual(payload["state"], "ready")
        prerequisite_ids = [item["id"] for item in payload["prerequisites"]]
        self.assertNotIn("alignment", prerequisite_ids)
        self.assertEqual(payload["diagnostics"][0]["id"], "alignment")
        self.assertFalse(payload["diagnostics"][0]["passed"])
        self.assertTrue(payload["diagnostics"][0]["diagnostic"])

    def test_prerequisite_evidence_is_always_present(self):
        from dashboard.supervision.safety import NullSafetyInput
        supervisor, _, clock = make_engine(safety_input=NullSafetyInput())
        clock.advance(6.0)
        payload = self.readiness(supervisor, clock)
        for item in payload["prerequisites"]:
            self.assertTrue(item["evidence"], item["id"])
            self.assertIn("passed", item)
            self.assertIn("source", item)

        reason = payload["reasons"][0]
        self.assertIn("detail", reason["evidence"])
        self.assertIn("safety", reason["message"].lower())

    def test_readiness_note_never_claims_safety(self):
        supervisor, _, clock = self.engine()
        payload = self.readiness(supervisor, clock)
        self.assertIn("not a safety certification", payload["note"])
        self.assertTrue(payload["gated_by_safety"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
