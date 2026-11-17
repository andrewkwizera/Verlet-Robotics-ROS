"""Safety dimension rules (pure Python, no ROS).

The invariant under test is deliberately narrow and absolute: safety is never
Normal unless an independent, fresh, explicitly-configured input says so.
"""

from __future__ import annotations

import os
import sys
import unittest

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
if _TEST_DIR not in sys.path:
    sys.path.insert(0, _TEST_DIR)

from supervision_testkit import (  # noqa: E402
    FakeClock,
    FakeSafetyInput,
    make_engine,
    make_snapshot,
)
from dashboard.supervision.safety import (  # noqa: E402
    NullSafetyInput,
    SafetyReading,
)
from dashboard.supervision.states import (  # noqa: E402
    SAFETY_FAULT,
    SAFETY_NORMAL,
    SAFETY_UNKNOWN,
    SAFETY_WARNING,
)


def codes(payload: dict) -> list:
    return [reason["code"] for reason in payload.get("reasons", [])]


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.input = FakeSafetyInput(None)
        self.engine, self.events, _ = make_engine(
            self.clock, safety_input=self.input
        )

    def safety(self) -> dict:
        return self.engine.evaluate(make_snapshot(), self.clock.now)["safety"]

    def test_null_input_is_unknown_and_never_normal(self):
        engine, _, clock = make_engine(self.clock, safety_input=NullSafetyInput())
        for _ in range(50):
            payload = engine.evaluate(make_snapshot(), clock.now)["safety"]
            self.assertEqual(payload["state"], SAFETY_UNKNOWN)
            self.assertEqual(payload["label"], "Unknown")
            self.assertIn("safety.no_source", codes(payload))
            self.assertEqual(payload["source"], "none")
            self.assertFalse(payload["latch"])
            clock.advance(1.0)

    def test_configured_input_without_reading_is_unknown(self):
        payload = self.safety()
        self.assertEqual(payload["state"], SAFETY_UNKNOWN)
        self.assertIn("safety.awaiting_input", codes(payload))

    def test_fresh_explicit_normal_is_normal(self):
        self.clock.advance(0.1)
        self.input.set(SafetyReading(
            state=SAFETY_NORMAL,
            source="test-safety",
            received_at=self.clock.now,
        ))
        payload = self.safety()
        self.assertEqual(payload["state"], SAFETY_NORMAL)
        self.assertEqual(payload["reasons"], [])

    def test_normal_requires_an_explicit_received_at(self):
        # A reading without a timestamp is accepted as current by the adapter,
        # but only because the adapter itself produced it.  The important
        # invariant is that no reading at all can never become Normal.
        self.input.set(SafetyReading(state=SAFETY_NORMAL, source="test-safety"))
        self.assertEqual(self.safety()["state"], SAFETY_NORMAL)
        self.input.set(None)
        self.assertEqual(self.safety()["state"], SAFETY_UNKNOWN)

    def test_stale_normal_becomes_unknown(self):
        self.clock.advance(0.1)
        self.input.set(SafetyReading(
            state=SAFETY_NORMAL,
            source="test-safety",
            received_at=self.clock.now,
        ))
        self.assertEqual(self.safety()["state"], SAFETY_NORMAL)

        self.clock.advance(2.01)
        payload = self.safety()
        self.assertEqual(payload["state"], SAFETY_UNKNOWN)
        self.assertIn("safety.input_stale", codes(payload))
        self.assertNotEqual(payload["state"], SAFETY_NORMAL)

    def test_explicit_warning_is_warning(self):
        self.input.set(SafetyReading(
            state=SAFETY_WARNING,
            detail="guard open",
            source="test-safety",
        ))
        payload = self.safety()
        self.assertEqual(payload["state"], SAFETY_WARNING)
        self.assertIn("guard open", payload["reasons"][0]["message"])

    def test_fault_latches_until_sustained_fresh_normal(self):
        self.clock.advance(0.1)
        self.input.set(SafetyReading(
            state=SAFETY_FAULT,
            detail="drive fault",
            source="test-safety",
            received_at=self.clock.now,
        ))
        payload = self.safety()
        self.assertEqual(payload["state"], SAFETY_FAULT)
        self.assertTrue(payload["latch"])
        self.assertIn("safety.fault", codes(payload))

        # Nothing clears it while the input is absent.
        self.input.set(None)
        self.assertEqual(self.safety()["state"], SAFETY_FAULT)
        self.assertIn("safety.fault_latched", codes(self.safety()))

        # A single fresh Normal is not enough; the clear hold must elapse.
        self.clock.advance(0.1)
        self.input.set(SafetyReading(
            state=SAFETY_NORMAL,
            source="test-safety",
            received_at=self.clock.now,
        ))
        payload = self.safety()
        self.assertEqual(payload["state"], SAFETY_FAULT)
        self.assertIn("safety.fault_latched", codes(payload))

        self.clock.advance(1.0)
        self.input.set(SafetyReading(
            state=SAFETY_NORMAL,
            source="test-safety",
            received_at=self.clock.now,
        ))
        payload = self.safety()
        self.assertEqual(payload["state"], SAFETY_NORMAL)
        self.assertFalse(payload["latch"])

    def test_warning_does_not_clear_latched_fault(self):
        self.input.set(SafetyReading(state=SAFETY_FAULT, source="test-safety"))
        self.assertEqual(self.safety()["state"], SAFETY_FAULT)
        self.input.set(SafetyReading(state=SAFETY_WARNING, source="test-safety"))
        payload = self.safety()
        self.assertEqual(payload["state"], SAFETY_FAULT)
        self.assertIn("safety.fault_latched", codes(payload))

    def test_stale_input_after_fault_stays_fault(self):
        self.clock.advance(0.1)
        self.input.set(SafetyReading(
            state=SAFETY_FAULT,
            source="test-safety",
            received_at=self.clock.now,
        ))
        self.assertEqual(self.safety()["state"], SAFETY_FAULT)
        self.clock.advance(10.0)
        payload = self.safety()
        self.assertEqual(payload["state"], SAFETY_FAULT)
        self.assertIn("safety.input_stale_after_fault", codes(payload))

    def test_unrecognised_value_is_unknown_never_normal(self):
        for value in ("", "nominal", "OK", "healthy", "0"):
            with self.subTest(value=value):
                engine, _, clock = make_engine(
                    FakeClock(),
                    safety_input=FakeSafetyInput(SafetyReading(
                        state=value, source="test-safety"
                    )),
                )
                payload = engine.evaluate(make_snapshot(), clock.now)["safety"]
                self.assertEqual(payload["state"], SAFETY_UNKNOWN)
                self.assertNotEqual(payload["state"], SAFETY_NORMAL)

    def test_payload_documents_why_safety_is_unknown(self):
        payload = self.safety()
        self.assertIn("independent safety system", payload["explanation"])
        self.assertIn("None is configured", payload["required_input"])
        self.assertEqual(payload["source"], "test-safety")

    def test_telemetry_health_does_not_influence_safety(self):
        # Even a flawless telemetry picture leaves safety Unknown.
        engine, _, clock = make_engine(self.clock, safety_input=NullSafetyInput())
        payload = engine.evaluate(make_snapshot(), clock.now)
        self.assertEqual(payload["health"]["state"], "healthy")
        self.assertEqual(payload["safety"]["state"], SAFETY_UNKNOWN)

        broken = engine.evaluate(
            make_snapshot(leader_state="disconnected", follower_state="disconnected"),
            clock.now,
        )
        self.assertEqual(broken["safety"]["state"], SAFETY_UNKNOWN)


class SafetyConfigTests(unittest.TestCase):
    def test_unsupported_safety_source_is_rejected(self):
        from dashboard.supervision.config import (
            SUPERVISION_DEFAULTS,
            SupervisionConfig,
        )

        for value in ("bridge_status", "yam_teleop", "telemetry", ""):
            with self.subTest(source=value):
                values = dict(SUPERVISION_DEFAULTS)
                values["supervision.safety.source"] = value
                with self.assertRaises(ValueError):
                    SupervisionConfig.from_parameter_dict(values)

    def test_none_source_is_accepted_and_normal_is_never_implied(self):
        from dashboard.supervision.config import (
            SUPERVISION_DEFAULTS,
            SupervisionConfig,
        )

        values = dict(SUPERVISION_DEFAULTS)
        values["supervision.safety.source"] = "none"
        config = SupervisionConfig.from_parameter_dict(values)
        self.assertEqual(config.safety_source, "none")
        self.assertFalse(NullSafetyInput().configured)


if __name__ == "__main__":
    unittest.main(verbosity=2)
