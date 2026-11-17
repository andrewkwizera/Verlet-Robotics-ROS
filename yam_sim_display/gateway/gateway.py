"""Orchestration for two independent gateway camera workers.

Only this module owns threads.  Each camera has its own signaling loop, offer
revision and webrtcbin pipeline.  Session discovery is shared so a lost session
can be replaced without restarting the process or resetting the other camera.
"""

from __future__ import annotations

import json
import os
import threading
import time
from typing import Optional

from .camera import CameraPipeline
from .config import CameraConfig, GatewayConfig
from .signaling_client import (
    SessionExpired,
    SessionSelector,
    SignalingClient,
    SignalingError,
)

NO_RTP_AFTER_S = 3.0


class CameraWorker:
    """Signaling state and media pipeline for exactly one camera."""

    def __init__(
        self,
        camera: CameraConfig,
        client: SignalingClient,
        selector: SessionSelector,
        run_dir: str,
        poll_timeout_s: float,
        stop_event: threading.Event,
    ) -> None:
        self.camera = camera
        self.client = client
        self.selector = selector
        self.run_dir = run_dir
        self.poll_timeout_s = poll_timeout_s
        self.stop_event = stop_event

        self.session_id: Optional[str] = None
        self.after_rev = 0
        self.health = "starting"
        self._pipeline: Optional[CameraPipeline] = None
        self._lock = threading.Lock()
        self._last_ingress_packets = 0
        self._last_ingress_at = time.monotonic()

    # -- signaling loop ---------------------------------------------------

    def run(self) -> None:
        while not self.stop_event.is_set():
            info = self.selector.current()
            if info is None:
                self.stop_event.wait(0.5)
                continue

            if info.session_id != self.session_id:
                # New browser session: process any offers present there, but
                # keep the current pipeline until that offer actually arrives.
                self.session_id = info.session_id
                self.after_rev = 0

            try:
                offer = self.client.get_offer(
                    self.session_id,
                    self.camera.camera_id,
                    after_rev=self.after_rev,
                    timeout_s=self.poll_timeout_s,
                )
            except SessionExpired:
                self.selector.invalidate(self.session_id)
                self.session_id = None
                continue
            except SignalingError as exc:
                print(f"[gateway] {self.camera.camera_id}: poll failed: {exc}", flush=True)
                self.stop_event.wait(1.0)
                continue

            if offer is None:
                continue
            sdp, revision = offer
            if revision <= self.after_rev:
                continue
            self._handle_offer(sdp, revision)

    def _handle_offer(self, sdp: str, revision: int) -> None:
        """Rebuild only this camera's pipeline for a new offer revision."""
        candidate: Optional[CameraPipeline] = None
        try:
            # Dispose the old webrtcbin before the new one gathers ICE: two
            # simultaneous peer connections on one camera caused candidate
            # conflicts during browser retries.
            with self._lock:
                previous = self._pipeline
                self._pipeline = None
            if previous is not None:
                previous.dispose()

            candidate = CameraPipeline(
                self.camera.camera_id,
                self.camera.host,
                self.camera.port,
                self.run_dir,
            )
            answer = candidate.handle_offer(sdp)
            self.client.post_answer(
                self.session_id, self.camera.camera_id, answer
            )
            candidate.answer_posted = True

            with self._lock:
                self._pipeline = candidate
                self.after_rev = revision
                self.health = "negotiating"
                self._last_ingress_packets = 0
                self._last_ingress_at = time.monotonic()
            print(
                f"[gateway] {self.camera.camera_id}: answer posted "
                f"(offer rev {revision})",
                flush=True,
            )
        except SessionExpired:
            if candidate is not None:
                candidate.dispose()
            self.selector.invalidate(self.session_id)
            self.session_id = None
        except Exception as exc:  # noqa: BLE001 - one camera must not kill the other
            if candidate is not None:
                candidate.dispose()
            with self._lock:
                # Avoid retrying the same bad offer in a tight loop; a browser
                # retry creates a new revision.
                self.after_rev = revision
                self.health = "error"
            print(
                f"[gateway] {self.camera.camera_id}: negotiation failed: {exc}",
                flush=True,
            )

    # -- stats / health ---------------------------------------------------

    def snapshot(self) -> tuple[dict, str]:
        """Return gateway-side stats plus a coarse per-camera health value."""
        with self._lock:
            pipeline = self._pipeline

        if pipeline is None:
            return {
                "source": "gateway",
                "camera": self.camera.camera_id,
                "host": self.camera.host,
                "port": self.camera.port,
                "ts": time.time(),
                "ingress_packets": 0,
                "ingress_bytes": 0,
                "connection_state": "none",
                "ice_connection_state": "none",
                "ice_gathering_state": "none",
                "negotiated_pt": None,
                "webrtc": {},
            }, self.health

        stats = pipeline.stats()
        status = pipeline.status()
        now = time.monotonic()
        packets = int(stats.get("ingress_packets", 0))

        with self._lock:
            if packets > self._last_ingress_packets:
                self._last_ingress_packets = packets
                self._last_ingress_at = now
            no_rtp = (
                status.get("negotiated_pt") is not None
                and (now - self._last_ingress_at) > NO_RTP_AFTER_S
            )

            if status.get("error"):
                health = "error"
            elif status.get("ice_connection_state") in ("failed", "disconnected"):
                health = "failed"
            elif status.get("connection_state") in ("failed", "disconnected"):
                health = "failed"
            elif no_rtp:
                health = "no-rtp"
            elif status.get("connection_state") == "connected":
                health = "connected"
            elif status.get("answer_posted"):
                health = "negotiating"
            else:
                health = "starting"
            self.health = health
        return stats, health

    def dispose(self) -> None:
        with self._lock:
            pipeline = self._pipeline
            self._pipeline = None
        if pipeline is not None:
            pipeline.dispose()


class Gateway:
    """Owns the camera workers, keepalive and gateway-stats threads."""

    def __init__(
        self,
        config: GatewayConfig,
        client: Optional[SignalingClient] = None,
    ) -> None:
        self.config = config
        self.client = client or SignalingClient(config.signaling_url)
        self.selector = SessionSelector(self.client)
        self.stop_event = threading.Event()
        os.makedirs(config.run_dir, exist_ok=True)
        self.stats_path = os.path.join(config.run_dir, "gateway_stats.jsonl")
        self.workers = [
            CameraWorker(
                camera,
                self.client,
                self.selector,
                config.run_dir,
                config.poll_timeout_s,
                self.stop_event,
            )
            for camera in config.cameras
        ]
        self._threads: list[threading.Thread] = []

    # -- lifecycle --------------------------------------------------------

    def start(self) -> None:
        for worker in self.workers:
            self._start_thread(
                f"yam-webrtc-{worker.camera.camera_id}", worker.run
            )
        self._start_thread("yam-webrtc-keepalive", self._keepalive_loop)
        self._start_thread("yam-webrtc-stats", self._stats_loop)
        print(
            f"[gateway] started {len(self.workers)} passthrough cameras; "
            f"stats: {self.stats_path}",
            flush=True,
        )

    def stop(self) -> None:
        self.stop_event.set()
        for thread in self._threads:
            thread.join(timeout=self.config.poll_timeout_s + 2.0)
        for worker in self.workers:
            worker.dispose()

    def _start_thread(self, name: str, target) -> None:
        thread = threading.Thread(target=target, name=name, daemon=True)
        thread.start()
        self._threads.append(thread)

    # -- background loops -------------------------------------------------

    def _keepalive_loop(self) -> None:
        while not self.stop_event.wait(self.config.keepalive_interval_s):
            info = self.selector.current()
            if info is None:
                continue
            try:
                self.client.keepalive(info.session_id)
            except SessionExpired:
                self.selector.invalidate(info.session_id)
            except SignalingError:
                pass

    def _stats_loop(self) -> None:
        next_at = 0.0
        while not self.stop_event.is_set():
            now = time.monotonic()
            if now < next_at:
                self.stop_event.wait(min(0.1, next_at - now))
                continue
            next_at = now + self.config.stats_interval_s

            for worker in self.workers:
                stats, health = worker.snapshot()
                self._write_stats(worker.camera.camera_id, stats, health)
                info = self.selector.current()
                if info is None:
                    continue
                try:
                    self.client.post_stats(
                        info.session_id, worker.camera.camera_id, stats, health
                    )
                except SessionExpired:
                    self.selector.invalidate(info.session_id)
                except SignalingError:
                    pass

    def _write_stats(self, camera_id: str, stats: dict, health: str) -> None:
        entry = {"camera": camera_id, "health": health, "stats": stats}
        with open(self.stats_path, "a", encoding="utf-8") as handle:
            handle.write(json.dumps(entry, separators=(",", ":")) + "\n")
