"""Stdlib-only configuration for the standalone WebRTC gateway.

No ROS and no YAML dependency: the gateway must run as an ordinary process on
the Follower side.  JSON config supplies the defaults; CLI flags override them.
"""

from __future__ import annotations

import argparse
import json
import os
import tempfile
from dataclasses import dataclass
from typing import Any, Mapping, Optional

CAMERA_IDS = ("d405", "d455f")

DEFAULT_CONFIG: dict[str, Any] = {
    "signaling_url": "http://127.0.0.1:8080",
    "cameras": {
        "d405": {"host": "127.0.0.1", "port": 5100},
        "d455f": {"host": "127.0.0.1", "port": 5102},
    },
    "poll_timeout_s": 10.0,
    "keepalive_interval_s": 60.0,
    "stats_interval_s": 1.0,
    "jitter_buffer_ms": 30,
    "queue_max_packets": 32,
    "run_dir": "",
}


@dataclass(frozen=True)
class CameraConfig:
    camera_id: str
    host: str
    port: int


@dataclass(frozen=True)
class GatewayConfig:
    signaling_url: str
    cameras: tuple[CameraConfig, ...]
    poll_timeout_s: float
    keepalive_interval_s: float
    stats_interval_s: float
    jitter_buffer_ms: int
    queue_max_packets: int
    run_dir: str

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "GatewayConfig":
        url = str(data.get("signaling_url", DEFAULT_CONFIG["signaling_url"]))
        if not url.startswith(("http://", "https://")):
            raise ValueError("signaling_url must be an http(s) URL")

        raw_cameras = data.get("cameras", DEFAULT_CONFIG["cameras"])
        if not isinstance(raw_cameras, dict) or not raw_cameras:
            raise ValueError("cameras must be a non-empty object")

        cameras = []
        seen: set[str] = set()
        for camera_id, spec in raw_cameras.items():
            if camera_id not in CAMERA_IDS:
                raise ValueError(f"unknown camera id: {camera_id}")
            if camera_id in seen:
                raise ValueError(f"duplicate camera id: {camera_id}")
            seen.add(camera_id)
            if not isinstance(spec, dict):
                raise ValueError(f"camera {camera_id} must be an object")
            host = str(spec.get("host", "127.0.0.1")).strip()
            if not host:
                raise ValueError(f"camera {camera_id} host must not be empty")
            port = int(spec.get("port", 0))
            if not 1 <= port <= 65535:
                raise ValueError(f"camera {camera_id} port out of range: {port}")
            cameras.append(CameraConfig(camera_id, host, port))

        poll_timeout = float(data.get("poll_timeout_s", 10.0))
        keepalive = float(data.get("keepalive_interval_s", 60.0))
        stats_interval = float(data.get("stats_interval_s", 1.0))
        if not 0.1 <= poll_timeout <= 30.0:
            raise ValueError("poll_timeout_s must be between 0.1 and 30")
        if keepalive <= 0.0 or stats_interval <= 0.0:
            raise ValueError("intervals must be positive")

        jitter_buffer_ms = int(data.get("jitter_buffer_ms", 30))
        queue_max_packets = int(data.get("queue_max_packets", 32))
        if not 0 <= jitter_buffer_ms <= 500:
            raise ValueError("jitter_buffer_ms must be between 0 and 500")
        if not 1 <= queue_max_packets <= 1000:
            raise ValueError("queue_max_packets must be between 1 and 1000")

        run_dir = str(data.get("run_dir") or "").strip()
        if not run_dir:
            run_dir = os.path.join(
                tempfile.gettempdir(), "yam_webrtc_gateway"
            )

        return cls(
            signaling_url=url.rstrip("/"),
            cameras=tuple(cameras),
            poll_timeout_s=poll_timeout,
            keepalive_interval_s=keepalive,
            stats_interval_s=stats_interval,
            jitter_buffer_ms=jitter_buffer_ms,
            queue_max_packets=queue_max_packets,
            run_dir=run_dir,
        )

    @classmethod
    def from_file(cls, path: str) -> "GatewayConfig":
        with open(path, encoding="utf-8") as handle:
            data = json.load(handle)
        if not isinstance(data, dict):
            raise ValueError("gateway config must be a JSON object")
        return cls.from_mapping(data)

    @classmethod
    def default(cls) -> "GatewayConfig":
        return cls.from_mapping(DEFAULT_CONFIG)


def parse_camera_spec(value: str) -> tuple[str, str, int]:
    """Parse ``ID:HOST:PORT`` from the command line."""
    parts = value.split(":")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected ID:HOST:PORT")
    camera_id, host, raw_port = parts
    if camera_id not in CAMERA_IDS:
        raise argparse.ArgumentTypeError(f"unknown camera id: {camera_id}")
    try:
        port = int(raw_port)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid port: {raw_port}") from exc
    return camera_id, host, port


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="YAM standalone WebRTC gateway")
    parser.add_argument("--config", help="JSON config file")
    parser.add_argument("--signaling", help="Leader dashboard signaling base URL")
    parser.add_argument(
        "--camera", action="append", type=parse_camera_spec,
        help="ID:HOST:PORT override (repeatable)",
    )
    parser.add_argument(
        "--only-camera", action="append", choices=CAMERA_IDS,
        help="run only this camera (repeatable)",
    )
    parser.add_argument("--run-dir", help="directory for stats and SDP dumps")
    parser.add_argument("--poll-timeout", type=float, help="offer poll timeout (s)")
    parser.add_argument("--keepalive-interval", type=float)
    parser.add_argument("--stats-interval", type=float)
    parser.add_argument("--jitter-buffer-ms", type=int)
    parser.add_argument("--queue-max-packets", type=int)
    return parser


def from_args(argv: Optional[list[str]] = None) -> GatewayConfig:
    args = build_parser().parse_args(argv)
    data: dict[str, Any] = dict(DEFAULT_CONFIG)
    data["cameras"] = {
        key: dict(value) for key, value in DEFAULT_CONFIG["cameras"].items()
    }
    if args.config:
        with open(args.config, encoding="utf-8") as handle:
            loaded = json.load(handle)
        if not isinstance(loaded, dict):
            raise ValueError("gateway config must be a JSON object")
        data.update(loaded)

    if args.signaling:
        data["signaling_url"] = args.signaling
    if args.camera:
        cameras = dict(data.get("cameras") or {})
        for camera_id, host, port in args.camera:
            cameras[camera_id] = {"host": host, "port": port}
        data["cameras"] = cameras
    if args.only_camera:
        selected = set(args.only_camera)
        cameras = dict(data.get("cameras") or {})
        data["cameras"] = {
            camera_id: spec
            for camera_id, spec in cameras.items()
            if camera_id in selected
        }
    if args.run_dir:
        data["run_dir"] = args.run_dir
    if args.poll_timeout is not None:
        data["poll_timeout_s"] = args.poll_timeout
    if args.keepalive_interval is not None:
        data["keepalive_interval_s"] = args.keepalive_interval
    if args.stats_interval is not None:
        data["stats_interval_s"] = args.stats_interval
    if args.jitter_buffer_ms is not None:
        data["jitter_buffer_ms"] = args.jitter_buffer_ms
    if args.queue_max_packets is not None:
        data["queue_max_packets"] = args.queue_max_packets

    return GatewayConfig.from_mapping(data)
