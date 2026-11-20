"""Tests for optional read-only network RTT probing."""

from dashboard.network_probe import NetworkRTTProbe


def test_empty_target_disables_probe_without_starting_thread():
    probe = NetworkRTTProbe("")
    try:
        snapshot = probe.snapshot()
        assert snapshot["available"] is False
        assert snapshot["display"] == "Network RTT not configured"
        assert probe._thread is None
    finally:
        probe.stop()
