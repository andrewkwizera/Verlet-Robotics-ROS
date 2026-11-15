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

from .config import DEFAULTS, DashboardConfig
from .http_server import ServerThread
from .telemetry import TelemetryStore

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
        self.store = TelemetryStore(self.config)

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

        self.create_timer(EVALUATE_PERIOD_S, self._tick)

        self._lock = threading.Lock()
        self._reported_first = {"leader": False, "follower": False}

        self._server = ServerThread(
            self.store,
            self.config.bind_address,
            self.config.port,
            web_dir,
        )
        self._server.start()
        self._running = True

        host, port = self._server.address[0], self._server.address[1]
        self.get_logger().info(
            f"YAM operator dashboard listening on http://{host}:{port} "
            f"(read-only; leader={self.config.leader_topic}, "
            f"follower={self.config.follower_topic})"
        )
        if self.config.bind_address not in ("127.0.0.1", "localhost", "::1"):
            self.get_logger().warning(
                f"Dashboard bound to {self.config.bind_address}; it is "
                "unauthenticated and read-only, so restrict access at the "
                "network layer."
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

    def _tick(self) -> None:
        if not self._running:
            return
        self.store.evaluate()

    # -- lifecycle ----------------------------------------------------------

    def shutdown(self) -> None:
        self._running = False
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
