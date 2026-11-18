"""Pure Python tests for the read-only tracking status monitor."""
import unittest

from dashboard.bridge_status import BridgeStatusMonitor, parse_bridge_status


class BridgeStatusTests(unittest.TestCase):
    def test_whitespace_reason_and_numbers(self):
        result = parse_bridge_status(
            "tracking=DISABLED leader=FRESH follower=STALE "
            "actuation_authorized=true max_separation_rad=0.028 "
            "worst_joint=5 last_disarm_reason=Follower feedback lost"
        )
        self.assertEqual(result["tracking"], "DISABLED")
        self.assertEqual(result["last_disarm_reason"], "Follower feedback lost")
        self.assertEqual(result["worst_joint"], "5")
        self.assertAlmostEqual(result["max_separation_rad"], 0.028)

    def test_stale_data_never_claims_tracking_armed(self):
        monitor = BridgeStatusMonitor()
        self.assertFalse(monitor.snapshot(now=5.0)["available"])
        self.assertTrue(monitor.update(
            "tracking=ENABLED leader=FRESH follower=FRESH", now=10.0
        ))
        self.assertEqual(monitor.snapshot(now=11.0)["tracking"], "ENABLED")
        self.assertEqual(monitor.snapshot(now=13.0)["tracking"], "UNKNOWN")
        self.assertFalse(monitor.snapshot(now=13.0)["fresh"])

    def test_invalid_message_does_not_overwrite(self):
        monitor = BridgeStatusMonitor()
        monitor.update("tracking=DISABLED", now=1.0)
        self.assertFalse(monitor.update("arbitrary message", now=1.5))
        self.assertEqual(monitor.snapshot(now=1.5)["tracking"], "DISABLED")


if __name__ == "__main__":
    unittest.main()
