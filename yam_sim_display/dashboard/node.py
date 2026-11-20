"""ROS 2 node feeding the operator dashboard.

Execution model
---------------
* ROS callbacks run on a ``MultiThreadedExecutor``; they only copy values into
  ``TelemetryStore`` (a short, lock-protected operation).
* A node timer evaluates derived state and caches a JSON-ready snapshot.
* The HTTP server runs on its own daemon thread and only reads that cached
  snapshot, so a slow or absent browser can never block ROS callbacks.

The node is strictly read-only: it subscribes to joint states and publishes
nothing, so it cannot influence the control path.
"""

from __future__ import annotations

import signal
import threading
import time
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .config import DEFAULTS, DashboardConfig
from .bridge_status import BridgeStatusMonitor
from .network_probe import NetworkRTTProbe
from .http_server import ServerThread
from .state_cache import StateCache
from .supervision import SUPERVISION_DEFAULTS, SupervisionConfig, Supervisor
from .telemetry import TelemetryStore
from .webrtc_signaling import SignalingStore

# Derived-state evaluation is decoupled from the browser poll interval so a
# slow refresh rate cannot hide a freshness transition.
EVALUATE_PERIOD_S = 0.05


def _joint_qos() -> QoSProfile:
    """Match the default publisher QoS used by the simulation sources."""
    return QoSProfile(
        depth=10,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class DashboardNode(Node):
    def __init__(self, web_dir: str) -> None:
        super().__init__("yam_dashboard")

        values = {
            name: self.declare_parameter(name, default).value
            for name, default in DEFAULTS.items()
        }
        self.config = DashboardConfig.from_parameter_dict(values)

        supervision_values = {
            name: self.declare_parameter(name, default).value
            for name, default in SUPERVISION_DEFAULTS.items()
        }
        self.supervision_config = SupervisionConfig.from_parameter_dict(
            supervision_values
        )

        self.store = TelemetryStore(self.config)
        self.bridge_status = BridgeStatusMonitor()
        self.network_probe = NetworkRTTProbe(self.config.network_rtt_host)
        self.cache = StateCache()
        # Milestone 2 ships no safety adapter: safety is Unknown, never Normal.
        self.supervisor = Supervisor(
            self.supervision_config,
            self.config,
            self.store.events,
        )

        qos = _joint_qos()
        self.create_subscription(
            JointState,
            self.config.leader_topic,
            lambda msg: self._on_joint_state("leader", msg),
            qos,
        )
        self.create_subscription(
            JointState,
            self.config.follower_topic,
            lambda msg: self._on_joint_state("follower", msg),
            qos,
        )

        # Read-only physical tracking status from the follower bridge.
        # No publishers, services, or robot commands are created here.
        self.create_subscription(
            String, "/yam_teleop/status", self._on_bridge_status,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        self.create_timer(EVALUATE_PERIOD_S, self._tick)

        self._lock = threading.Lock()
        self._reported_first = {"leader": False, "follower": False}

        # M3 signaling is independent of ROS state and disabled by default.
        self.signaling = (
            SignalingStore(camera_ids=self.config.webrtc_cameras)
            if self.config.webrtc_enabled else None
        )
        self._server = ServerThread(
            self.cache,
            self.config.bind_address,
            self.config.port,
            web_dir,
            self.signaling,
        )
        self._server.start()
        self._running = True

        host, port = self._server.address[0], self._server.address[1]
        self.get_logger().info(
            f"YAM operator dashboard listening on http://{host}:{port} "
            f"(read-only; leader={self.config.leader_topic}, "
            f"follower={self.config.follower_topic})"
        )
        self.get_logger().info(
            "Supervision enabled; safety source="
            f"'{self.supervision_config.safety_source}' "
            "(no safety input -> Unknown)"
        )
        if self.config.bind_address not in ("127.0.0.1", "localhost", "::1"):
            if self.signaling is None:
                self.get_logger().warning(
                    f"Dashboard bound to {self.config.bind_address}; it is "
                    "unauthenticated and read-only, so restrict access at the "
                    "network layer."
                )
            else:
                self.get_logger().warning(
                    f"Dashboard bound to {self.config.bind_address}; "
                    "telemetry is read-only but WebRTC signaling is "
                    "unauthenticated, so restrict access at the network layer."
                )

    # -- ROS callbacks ------------------------------------------------------

    def _on_joint_state(self, role: str, msg: JointState) -> None:
        stamp_ns: Optional[int] = None
        stamp = msg.header.stamp
        if stamp.sec or stamp.nanosec:
            stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

        self.store.on_joint_state(
            role, list(msg.name), list(msg.position), source_stamp_ns=stamp_ns
        )

        topic = (
            self.config.leader_topic
            if role == "leader"
            else self.config.follower_topic
        )
        with self._lock:
            if not self._reported_first[role]:
                self._reported_first[role] = True
                self.get_logger().info(
                    f"Receiving {role} joint states "
                    f"({len(msg.name)} joints) from {topic}"
                )

    def _on_bridge_status(self, msg: String) -> None:
        self.bridge_status.update(msg.data)

    def _tick(self) -> None:
        if not self._running:
            return
        now = time.monotonic()
        telemetry = self.store.evaluate(now)
        supervision = self.supervisor.evaluate(telemetry, now)

        # Publish one merged, JSON-ready snapshot.  The M1 keys are untouched;
        # schema 2 only signals the additive ``supervision`` block.
        merged = dict(telemetry)
        merged["schema"] = 2
        merged["supervision"] = supervision
        merged["robot_control"] = self.bridge_status.snapshot(now)
        merged["latency"] = self.network_probe.snapshot(now)
        merged["events"] = self.store.events.snapshot()
        self.cache.publish(merged)

    # -- lifecycle ----------------------------------------------------------

    def shutdown(self) -> None:
        self._running = False
        self.network_probe.stop()
        self._server.stop()


def _ignore_further_interrupts() -> None:
    """Make teardown non-interruptible once shutdown has begun.

    A second Ctrl-C (or the extra SIGINT a launch file relays) can arrive while
    the HTTP server is closing its listening socket. Aborting there used to
    raise ``KeyboardInterrupt`` out of ``finally`` and could leave the port
    bound. Ignoring SIGINT for the rest of teardown keeps the shutdown clean.
    """
    try:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    except (ValueError, OSError):  # not the main thread / unsupported platform
        pass


def run(web_dir: str) -> int:
    """Entry point shared by the installed launcher."""
    rclpy.init(args=None)
    node: Optional[DashboardNode] = None
    executor: Optional[MultiThreadedExecutor] = None
    try:
        node = DashboardNode(web_dir)
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
    finally:
        _ignore_further_interrupts()
        try:
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.shutdown()
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 0
