#!/usr/bin/env python3
"""Read-only YAM joint/status telemetry over a private Tailscale UDP link.

Important safety boundary:
  * Publishes incoming data only to /wan/<role>/joint_states.
  * NEVER publishes to /leader/joint_states on the follower NUC, or to any
    robot controller, command topic, or arming service.
  * The follower's bridge status is optionally mirrored to /yam_teleop/status
    on the LEADER NUC solely for its read-only dashboard. It is NOT a safety
    authority, and its contents can be delayed by the Internet connection.
  * Source timestamps are retained inside wire messages for diagnostics, but
    not converted to one-way delay without independently verified clock sync.
"""

from __future__ import annotations

import argparse
import ipaddress
import json
import math
import socket
import sys
import time
import uuid

JOINTS = tuple(f"joint_{i}" for i in range(1, 7))
TAILNET = ipaddress.IPv4Network("100.64.0.0/10")
MAX_DATAGRAM = 1200
SCHEMA = 1


def tailnet_ipv4(value: str) -> str:
    try:
        address = ipaddress.IPv4Address(value)
    except ipaddress.AddressValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc
    if address not in TAILNET:
        raise argparse.ArgumentTypeError(
            f"{value} is not a Tailscale IPv4 address (100.64.0.0/10)"
        )
    return str(address)


def normalize_joint_state(names, positions):
    if not isinstance(names, list) or not isinstance(positions, list):
        raise ValueError("Joint names and positions must be lists")
    if len(names) != 6 or len(positions) != 6 or set(names) != set(JOINTS):
        raise ValueError("Expected the six YAM joint names exactly once")
    ordered = dict(zip(names, positions))
    result = []
    for name in JOINTS:
        value = ordered[name]
        if isinstance(value, bool) or not isinstance(value, (float, int)):
            raise ValueError("Invalid joint type")
        number = float(value)
        if not math.isfinite(number) or abs(number) > 12.5:
            raise ValueError("Nonfinite or out-of-range joint position")
        result.append(number)
    return result


def decode_packet(raw: bytes, expected_role: str) -> dict:
    if len(raw) > MAX_DATAGRAM:
        raise ValueError("Packet too large")
    try:
        packet = json.loads(raw.decode("utf-8"))
    except (ValueError, UnicodeDecodeError) as exc:
        raise ValueError("Invalid JSON") from exc
    if not isinstance(packet, dict) or packet.get("v") != SCHEMA:
        raise ValueError("Invalid schema")
    if packet.get("role") != expected_role:
        raise ValueError("Unexpected robot role")
    session = packet.get("session")
    sequence = packet.get("seq")
    if not isinstance(session, str) or len(session) != 16 or not all(
        c in "0123456789abcdef" for c in session
    ):
        raise ValueError("Invalid session")
    if type(sequence) is not int or sequence < 0 or sequence > 2**63 - 1:
        raise ValueError("Invalid sequence")
    packet["position"] = normalize_joint_state(packet.get("name"), packet.get("position"))
    status = packet.get("status", "")
    if not isinstance(status, str) or len(status) > 512:
        raise ValueError("Invalid status")
    packet["status"] = status
    return packet


def self_test():
    p = {"v": 1, "role": "follower", "session": "a" * 16, "seq": 1,
         "name": list(reversed(JOINTS)), "position": [6, 5, 4, 3, 2, 1],
         "status": "tracking=DISABLED"}
    assert decode_packet(json.dumps(p).encode(), "follower")["position"] == [1, 2, 3, 4, 5, 6]
    assert tailnet_ipv4("100.73.207.42") == "100.73.207.42"
    for bad in (b"garbage", b"{}", json.dumps({**p, "seq": -1}).encode(),
                json.dumps({**p, "position": [float('nan')] * 6}).encode()):
        try:
            decode_packet(bad, "follower")
        except ValueError:
            pass
        else:
            raise AssertionError("Invalid input was not rejected")
    print("PASS: packet/schema/range/role and Tailscale address checks")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--role", choices=("leader", "follower"), required=True)
    parser.add_argument("--local-ip", type=tailnet_ipv4, required=True,
                        help="tailscale ip -4 on this NUC")
    parser.add_argument("--peer-ip", type=tailnet_ipv4, required=True,
                        help="tailscale ip -4 on the other NUC")
    parser.add_argument("--listen-port", type=int, required=True)
    parser.add_argument("--peer-port", type=int, required=True)
    parser.add_argument("--rate", type=float, default=10.0,
                        help="send rate Hz, default 10 (observational only)")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    if args.local_ip == args.peer_ip:
        parser.error("local-ip and peer-ip must differ")
    if any(not 1024 <= p <= 65535 for p in (args.listen_port, args.peer_port)):
        parser.error("ports must be 1024..65535")
    if not math.isfinite(args.rate) or not 1 <= args.rate <= 20:
        parser.error("--rate must be 1..20 Hz")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    class WANMonitor(Node):
        def __init__(self):
            super().__init__(f"yam_wan_readonly_{args.role}")
            self.role = args.role
            self.remote_role = "follower" if self.role == "leader" else "leader"
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind((args.local_ip, args.listen_port))
            self.socket.setblocking(False)
            self.session = uuid.uuid4().hex[:16]
            self.seq = 0
            self.latest = None
            self.latest_rx_at = None
            self.latest_status = ""
            self.status_rx_at = None
            self.rx_session = None
            self.rx_seq = -1
            self.rx_packets = 0
            self.missing = 0
            self.last_rx = None
            # The dashboard subscribes RELIABLE. The local relay must also
            # publish RELIABLE even though its WAN input is UDP.
            self.publisher = self.create_publisher(
                JointState, f"/wan/{self.remote_role}/joint_states",
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            )
            self.status_out = None
            self.dashboard_status = None
            if self.role == "leader":
                self.status_out = self.create_publisher(
                    String, "/wan/follower/teleop_status", 10
                )
                # The dashboard's current, fixed read-only status topic.
                self.dashboard_status = self.create_publisher(
                    String, "/yam_teleop/status", 10
                )
            self.create_subscription(JointState, f"/{self.role}/joint_states",
                                     self._on_local_joint, qos_profile_sensor_data)
            if self.role == "follower":
                self.create_subscription(String, "/yam_teleop/status",
                                         self._on_local_status, qos_profile_sensor_data)
            self.create_timer(1.0 / args.rate, self._send)
            self.create_timer(0.01, self._receive)
            self.create_timer(5.0, self._log_health)
            self.get_logger().warning(
                f"READ-ONLY WAN telemetry; local {args.local_ip}:{args.listen_port} "
                f"peer {args.peer_ip}:{args.peer_port} at {args.rate:g} Hz. "
                "NO motor commands, NO remote actuation."
            )

        def _on_local_joint(self, msg):
            try:
                positions = normalize_joint_state(list(msg.name), list(msg.position))
            except ValueError:
                return
            self.latest = positions
            self.latest_rx_at = time.monotonic()

        def _on_local_status(self, msg):
            self.latest_status = msg.data[:512]
            self.status_rx_at = time.monotonic()

        def _send(self):
            if (self.latest is None or self.latest_rx_at is None or
                    time.monotonic() - self.latest_rx_at > 0.25):
                # Do not rebroadcast a frozen local motor-feedback sample.
                return
            status = (self.latest_status if self.role == "follower" and
                      self.status_rx_at is not None and
                      time.monotonic() - self.status_rx_at < 2.0 else "")
            self.seq += 1
            packet = {"v": SCHEMA, "role": self.role, "session": self.session,
                      "seq": self.seq, "sent_ns": time.time_ns(),
                      "name": list(JOINTS), "position": self.latest,
                      "status": status}
            data = json.dumps(packet, separators=(",", ":"), allow_nan=False).encode()
            if len(data) > MAX_DATAGRAM:
                self.get_logger().error("Telemetry packet exceeded size limit")
                return
            try:
                self.socket.sendto(data, (args.peer_ip, args.peer_port))
            except OSError as exc:
                self.get_logger().error(f"WAN send failed: {exc}")

        def _receive(self):
            for _ in range(32):
                try:
                    raw, (source_ip, _source_port) = self.socket.recvfrom(MAX_DATAGRAM + 1)
                except BlockingIOError:
                    break
                except OSError as exc:
                    self.get_logger().error(f"WAN receive failed: {exc}")
                    break
                if source_ip != args.peer_ip:
                    continue
                try:
                    frame = decode_packet(raw, self.remote_role)
                except ValueError:
                    continue
                if frame["session"] != self.rx_session:
                    self.rx_session = frame["session"]
                    self.rx_seq = -1
                if frame["seq"] <= self.rx_seq:
                    continue
                if self.rx_seq >= 0:
                    self.missing += max(0, frame["seq"] - self.rx_seq - 1)
                self.rx_seq = frame["seq"]
                self.rx_packets += 1
                self.last_rx = time.monotonic()
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "wan_readonly_telemetry_NOT_synchronized"
                msg.name = list(JOINTS)
                msg.position = frame["position"]
                self.publisher.publish(msg)
                if self.status_out and frame["status"]:
                    state = String()
                    state.data = frame["status"]
                    self.status_out.publish(state)
                    self.dashboard_status.publish(state)

        def _log_health(self):
            age = "never" if self.last_rx is None else f"{(time.monotonic()-self.last_rx)*1000:.0f}ms"
            self.get_logger().info(
                f"WAN from {self.remote_role}: received={self.rx_packets} "
                f"sequence_gaps={self.missing} last_arrival_age={age}; "
                "remote display is NOT synchronized tracking error"
            )

        def shutdown(self):
            self.socket.close()

    rclpy.init()
    node = None
    try:
        node = WANMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
