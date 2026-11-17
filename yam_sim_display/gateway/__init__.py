"""Standalone Follower-side WebRTC gateway (M3 Phase 1c).

The package deliberately keeps ``config`` and ``signaling_client`` free of
GStreamer imports; ``gateway.gateway`` and ``gateway.camera`` require GStreamer.
"""

__all__ = ["config", "signaling_client", "gateway", "camera", "run"]
