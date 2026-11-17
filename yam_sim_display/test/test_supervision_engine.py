"""Supervision engine, API shape, read-only wiring and static layout tests.

Everything here runs without ROS.  The HTTP transport is exercised against a
real loopback server, and the frontend files are checked as static assets.
"""

from __future__ import annotations

import copy
import ast
import http.client
import json
import os
import re
import sys
import threading
import unittest

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_PACKAGE_ROOT = os.path.dirname(_TEST_DIR)
if _TEST_DIR not in sys.path:
    sys.path.insert(0, _TEST_DIR)

from supervision_testkit import (  # noqa: E402
    FakeSafetyInput,
    WALL,
    make_engine,
    make_snapshot,
)
from dashboard.http_server import DashboardHTTPServer  # noqa: E402
from dashboard.state_cache import StateCache  # noqa: E402
from dashboard.supervision.states import (  # noqa: E402
    SAFETY_UNKNOWN,
    SESSION_ACTIONS,
    SESSION_ACTION_TARGET,
    SESSION_STATES,
    SESSION_TRANSITIONS,
)

WEB_DIR = os.path.join(_PACKAGE_ROOT, "dashboard", "web")
SUPERVISION_DIR = os.path.join(_PACKAGE_ROOT, "dashboard", "supervision")
CONFIG_DIR = os.path.join(_PACKAGE_ROOT, "config")


def read(path: str) -> str:
    with open(path, "r", encoding="utf-8") as handle:
        return handle.read()


def payload(supervisor, clock, snapshot=None) -> dict:
    return supervisor.evaluate(
        snapshot if snapshot is not None else make_snapshot(), clock.now
    )


def codes(dimension: dict) -> list:
    return [reason["code"] for reason in dimension.get("reasons", [])]


def js_object(source: str, name: str) -> dict:
    """Parse a simple ``const NAME = { key: ["v", ...], ... };`` literal."""
    marker = f"const {name} = {{"
    start = source.index(marker)
    body_start = source.index("{", start)
    body_end = source.index("};", body_start)
    body = source[body_start + 1:body_end]
    table = {}
    for line in body.splitlines():
        match = re.match(r"\s*(\w+):\s*\[(.*?)\]\s*,?\s*$", line)
        if not match:
            continue
        table[match.group(1)] = re.findall(r'"([^"]*)"', match.group(2))
    return table


def parse_python(path: str) -> ast.Module:
    return ast.parse(read(path), filename=path)


def imported_modules(tree: ast.Module) -> set:
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module.split(".")[0])
    return names


def non_docstring_literals(tree: ast.Module) -> list:
    """String constants that are *code*, not module/class/function docs."""
    docstrings = set()
    for node in ast.walk(tree):
        if isinstance(
            node,
            (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef),
        ):
            doc = ast.get_docstring(node, clean=False)
            if doc:
                docstrings.add(doc)
    literals = []
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Constant)
            and isinstance(node.value, str)
            and node.value not in docstrings
        ):
            literals.append(node.value)
    return literals


def attribute_calls(tree: ast.Module, name: str) -> list:
    return [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == name
    ]


class EngineContractTests(unittest.TestCase):
    def setUp(self):
        self.engine, self.events, self.clock = make_engine()

    def test_payload_contract_and_json_serialisable(self):
        block = payload(self.engine, self.clock)
        for key in (
            "version", "generated_at", "health", "readiness", "safety",
            "session", "config", "available",
        ):
            self.assertIn(key, block)
        self.assertEqual(block["version"], 1)
        self.assertEqual(block["generated_at"], WALL)
        self.assertTrue(block["available"])

        for name in ("health", "readiness", "safety"):
            self.assertIn("state", block[name])
            self.assertIn("label", block[name])
            self.assertIn("since", block[name])
            self.assertIn("since_uptime_s", block[name])
            self.assertIn("reasons", block[name])
            self.assertIn("evidence", block[name])

        encoded = json.dumps(block, allow_nan=False)
        self.assertEqual(json.loads(encoded)["health"]["state"], "healthy")

    def test_there_is_no_combined_overall_state(self):
        block = payload(self.engine, self.clock)
        for forbidden in ("overall", "system_state", "combined", "verdict"):
            self.assertNotIn(forbidden, block)
            for name in ("health", "readiness", "safety"):
                self.assertNotIn(forbidden, block[name])

    def test_input_snapshot_is_not_mutated(self):
        snapshot = make_snapshot()
        before = copy.deepcopy(snapshot)
        payload(self.engine, self.clock, snapshot)
        self.assertEqual(snapshot, before)

    def test_initial_evaluation_does_not_log_events(self):
        payload(self.engine, self.clock)
        self.assertEqual(self.events.snapshot(), [])

    def test_state_transitions_log_one_event_each(self):
        healthy = make_snapshot()
        broken = make_snapshot(leader_state="disconnected")

        payload(self.engine, self.clock, healthy)
        payload(self.engine, self.clock, healthy)
        self.assertEqual(len(self.events), 0)

        payload(self.engine, self.clock, broken)
        first = [
            event for event in self.events.snapshot()
            if event["source"] == "supervision.health"
        ]
        self.assertEqual(len(first), 1)
        self.assertEqual(first[0]["level"], "error")
        self.assertIn("Healthy \u2192 Disconnected", first[0]["message"])
        self.assertIn("health.leader.disconnected", first[0]["message"])

        payload(self.engine, self.clock, broken)
        health_events = [
            event for event in self.events.snapshot()
            if event["source"] == "supervision.health"
        ]
        self.assertEqual(len(health_events), 1, "same state must not re-log")

        payload(self.engine, self.clock, healthy)
        health_events = [
            event for event in self.events.snapshot()
            if event["source"] == "supervision.health"
        ]
        self.assertEqual(len(health_events), 2)
        self.assertIn("Disconnected \u2192 Healthy", health_events[-1]["message"])

    def test_readiness_transition_event_is_warning_not_error(self):
        self.engine.evaluate(make_snapshot(), self.clock.now)
        self.clock.advance(6.0)
        block = payload(self.engine, self.clock)
        self.assertEqual(block["readiness"]["state"], "not_ready")

        readiness_events = [
            event for event in self.events.snapshot()
            if event["source"] == "supervision.readiness"
        ]
        self.assertEqual(len(readiness_events), 1)
        self.assertEqual(readiness_events[0]["level"], "warning")
        self.assertIn("readiness.safety_normal", readiness_events[0]["message"])

    def test_new_reason_is_logged_once_per_episode(self):
        self.engine.evaluate(make_snapshot(leader_rate=120.0), self.clock.now)
        self.assertEqual(self.engine.evaluate(
            make_snapshot(leader_rate=120.0), self.clock.now
        )["health"]["state"], "degraded")
        self.assertEqual(len(self.events), 0)

        # A second, distinct reason while already degraded should surface once.
        self.engine.evaluate(make_snapshot(leader_rate=120.0, leader_rejected=1),
                             self.clock.now)
        self.engine.evaluate(make_snapshot(leader_rate=120.0, leader_rejected=1),
                             self.clock.now)
        logged = [
            event for event in self.events.snapshot()
            if "rejected_frames" in event["message"]
        ]
        self.assertEqual(len(logged), 1)
        self.assertEqual(logged[0]["level"], "warning")

    def test_four_dimensions_are_independent(self):
        block = payload(self.engine, self.clock)
        self.assertEqual(block["health"]["state"], "healthy")
        self.assertEqual(block["readiness"]["state"], "initializing")
        self.assertEqual(block["safety"]["state"], SAFETY_UNKNOWN)
        self.assertEqual(block["session"]["managed_by"], "client")

        self.clock.advance(6.0)
        block = payload(self.engine, self.clock)
        self.assertEqual(block["health"]["state"], "healthy")
        self.assertEqual(block["readiness"]["state"], "not_ready")
        self.assertEqual(block["safety"]["state"], SAFETY_UNKNOWN)

    def test_missing_telemetry_degrades_without_crashing(self):
        self.engine.evaluate(None, self.clock.now)
        self.clock.advance(6.0)
        block = self.engine.evaluate(None, self.clock.now)
        self.assertEqual(block["health"]["state"], "disconnected")
        self.assertEqual(block["readiness"]["state"], "not_ready")
        self.assertEqual(block["safety"]["state"], SAFETY_UNKNOWN)
        self.assertIn("readiness.safety_normal", codes(block["readiness"]))
        self.assertIn("health.leader.disconnected", codes(block["health"]))
        json.dumps(block, allow_nan=False)

    def test_recovery_clears_reasons(self):
        payload(self.engine, self.clock)
        broken = make_snapshot(follower_state="stale", follower_rate=None)
        degraded = payload(self.engine, self.clock, broken)
        self.assertEqual(degraded["health"]["state"], "degraded")
        self.assertTrue(codes(degraded["health"]))

        recovered = payload(self.engine, self.clock)
        self.assertEqual(recovered["health"]["state"], "healthy")
        self.assertEqual(recovered["health"]["reasons"], [])

    def test_session_block_is_vocabulary_only(self):
        block = payload(self.engine, self.clock)
        session = block["session"]
        self.assertEqual(session["managed_by"], "client")
        self.assertEqual(tuple(session["states"]), SESSION_STATES)
        self.assertEqual(
            {state: tuple(targets) for state, targets in session["transitions"].items()},
            dict(SESSION_TRANSITIONS),
        )
        self.assertEqual(
            {state: tuple(actions) for state, actions in session["actions"].items()},
            dict(SESSION_ACTIONS),
        )
        self.assertIn("No POST endpoint exists", session["note"])
        self.assertNotIn("state", session)

    def test_session_action_targets_resolve(self):
        for state, actions in SESSION_ACTIONS.items():
            for action in actions:
                self.assertIn(SESSION_ACTION_TARGET[action],
                              SESSION_TRANSITIONS[state])

    def test_since_duration_resets_on_transition(self):
        payload(self.engine, self.clock)
        self.clock.advance(2.0)
        before = payload(self.engine, self.clock)["health"]
        self.assertAlmostEqual(before["since_uptime_s"], 2.0, places=3)

        payload(self.engine, self.clock, make_snapshot(leader_state="warning"))
        after = payload(
            self.engine, self.clock, make_snapshot(leader_state="warning")
        )["health"]
        self.assertEqual(after["state"], "degraded")
        self.assertLess(after["since_uptime_s"], 0.001)


class ConfigFileTests(unittest.TestCase):
    @staticmethod
    def flatten(mapping, prefix=""):
        out = {}
        for key, value in mapping.items():
            name = f"{prefix}{key}"
            if isinstance(value, dict):
                out.update(ConfigFileTests.flatten(value, f"{name}."))
            else:
                out[name] = value
        return out

    def test_supervision_yaml_matches_defaults_and_types(self):
        import yaml

        from dashboard.supervision.config import (
            SUPERVISION_DEFAULTS,
            SupervisionConfig,
        )

        document = yaml.safe_load(
            read(os.path.join(CONFIG_DIR, "supervision.yaml"))
        )
        values = self.flatten(
            document["yam_dashboard"]["ros__parameters"]
        )
        self.assertEqual(set(values), set(SUPERVISION_DEFAULTS))
        for name, default in SUPERVISION_DEFAULTS.items():
            self.assertIsInstance(
                values[name], type(default),
                f"{name} must be {type(default).__name__} in supervision.yaml",
            )
        config = SupervisionConfig.from_parameter_dict(values)
        self.assertEqual(config.safety_source, "none")
        self.assertEqual(config.safety_timeout_s, 2.0)

    def test_dashboard_yaml_unchanged_and_still_matching(self):
        import yaml

        from dashboard.config import DEFAULTS

        document = yaml.safe_load(
            read(os.path.join(CONFIG_DIR, "dashboard.yaml"))
        )
        values = self.flatten(
            document["yam_dashboard"]["ros__parameters"]
        )
        self.assertEqual(set(values), set(DEFAULTS))
        for name, default in DEFAULTS.items():
            self.assertIsInstance(
                values[name], type(default),
                f"{name} must be {type(default).__name__} in dashboard.yaml",
            )

    def test_launch_passes_both_parameter_files(self):
        source = read(
            os.path.join(_PACKAGE_ROOT, "launch", "dashboard.launch.py")
        )
        self.assertIn('"dashboard.yaml"', source)
        self.assertIn('"supervision.yaml"', source)
        self.assertIn("parameters = [config_file, supervision_file]", source)
        self.assertIn('"startup_grace_s"', source)
        self.assertIn('"ready_hold_s"', source)

    def test_invalid_supervision_values_are_rejected(self):
        from dashboard.supervision.config import (
            SUPERVISION_DEFAULTS,
            SupervisionConfig,
        )

        cases = {
            "supervision.ready_hold_s": 0.0,
            "supervision.safety.timeout_ms": 0.0,
            "supervision.readiness.expected_joint_count": 0,
            "supervision.health.source_dt_warn_ms": 0.0,
            "supervision.session.inactivity_s": 0.0,
            "supervision.startup_grace_s": -1.0,
        }
        for name, value in cases.items():
            with self.subTest(name=name):
                values = dict(SUPERVISION_DEFAULTS)
                values[name] = value
                with self.assertRaises(ValueError):
                    SupervisionConfig.from_parameter_dict(values)


class HttpTransportTests(unittest.TestCase):
    def setUp(self):
        self.cache = StateCache()
        self.cache.publish({
            "schema": 2,
            "ready": True,
            "supervision": {"health": {"state": "healthy"}},
            "events": [],
        })
        self.server = DashboardHTTPServer(("127.0.0.1", 0), self.cache, WEB_DIR)
        self.thread = threading.Thread(
            target=self.server.serve_forever, daemon=True
        )
        self.thread.start()
        self.port = self.server.server_address[1]

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2.0)

    def request(self, method: str, path: str, body: bytes = b""):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        try:
            connection.request(method, path, body=body)
            response = connection.getresponse()
            return response.status, response.read()
        finally:
            connection.close()

    def test_get_state_serves_cached_snapshot(self):
        status, body = self.request("GET", "/api/state")
        self.assertEqual(status, 200)
        document = json.loads(body)
        self.assertEqual(document["schema"], 2)
        self.assertEqual(document["supervision"]["health"]["state"], "healthy")

    def test_post_is_refused(self):
        status, _ = self.request("POST", "/api/state", b"{}")
        self.assertEqual(status, 405)

    def test_put_and_patch_are_refused(self):
        for method in ("PUT", "PATCH"):
            with self.subTest(method=method):
                status, _ = self.request(method, "/api/state", b"{}")
                self.assertEqual(status, 501)

    def test_delete_is_refused(self):
        status, _ = self.request("DELETE", "/api/state")
        self.assertEqual(status, 405)

    def test_webrtc_routes_are_absent_when_disabled(self):
        status, _ = self.request("GET", "/api/webrtc/offer?session_id=x&camera_id=d405")
        self.assertEqual(status, 404)
        status, _ = self.request("POST", "/api/webrtc/session", b"{}")
        self.assertEqual(status, 404)

    def test_head_is_not_a_state_endpoint(self):
        status, _ = self.request("HEAD", "/")
        self.assertEqual(status, 405)

    def test_static_assets_are_served(self):
        for path, marker in (
            ("/", b"YAM Operator Console"),
            ("/app.js", b"function renderSupervision"),
            ("/session.js", b"YAMSession"),
            ("/style.css", b".sup-pill"),
        ):
            with self.subTest(path=path):
                status, body = self.request("GET", path)
                self.assertEqual(status, 200)
                self.assertIn(marker, body)

    def test_path_traversal_is_refused(self):
        status, _ = self.request("GET", "/../telemetry.py")
        self.assertEqual(status, 404)


class StaticReadOnlyAuditTests(unittest.TestCase):
    ROS_MODULES = {"rclpy", "sensor_msgs", "rcl_interfaces", "rosidl_runtime_py"}

    def test_supervision_layer_has_no_ros_imports(self):
        for name in sorted(os.listdir(SUPERVISION_DIR)):
            if not name.endswith(".py"):
                continue
            tree = parse_python(os.path.join(SUPERVISION_DIR, name))
            self.assertFalse(
                imported_modules(tree) & self.ROS_MODULES,
                f"{name} must not import ROS",
            )

    def test_telemetry_and_cache_stay_ros_free(self):
        for name in ("telemetry.py", "state_cache.py", "config.py"):
            tree = parse_python(os.path.join(_PACKAGE_ROOT, "dashboard", name))
            self.assertFalse(imported_modules(tree) & self.ROS_MODULES, name)

    def test_node_never_publishes_and_only_reads_joint_states(self):
        tree = parse_python(os.path.join(_PACKAGE_ROOT, "dashboard", "node.py"))
        self.assertEqual(attribute_calls(tree, "create_publisher"), [])

        subscriptions = attribute_calls(tree, "create_subscription")
        self.assertEqual(len(subscriptions), 2)
        topics = sorted(ast.unparse(call.args[1]) for call in subscriptions)
        self.assertEqual(
            topics, ["self.config.follower_topic", "self.config.leader_topic"]
        )
        for call in subscriptions:
            self.assertEqual(ast.unparse(call.args[0]), "JointState")

        literals = " ".join(non_docstring_literals(tree))
        for forbidden in ("/yam_teleop", "cmd_vel", "joint_command"):
            self.assertNotIn(forbidden, literals)

    def test_no_command_topics_anywhere_in_dashboard(self):
        forbidden = (
            "/yam_teleop/status", "/cmd_vel", "/joint_command",
            "/forward_position_controller/commands",
        )
        for root, _dirs, files in os.walk(os.path.join(_PACKAGE_ROOT, "dashboard")):
            for name in files:
                if not name.endswith(".py"):
                    continue
                tree = parse_python(os.path.join(root, name))
                joined = " ".join(non_docstring_literals(tree))
                for token in forbidden:
                    self.assertNotIn(token, joined, f"{name} references {token}")

    def test_http_layer_has_no_write_handlers(self):
        tree = parse_python(
            os.path.join(_PACKAGE_ROOT, "dashboard", "http_server.py")
        )
        handlers = {
            node.name
            for node in ast.walk(tree)
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            and node.name.startswith("do_")
        }
        self.assertEqual(handlers, {"do_GET", "do_HEAD", "do_POST", "do_DELETE"})

    def test_node_merges_supervision_additively(self):
        source = read(os.path.join(_PACKAGE_ROOT, "dashboard", "node.py"))
        self.assertIn("merged = dict(telemetry)", source)
        self.assertIn('merged["schema"] = 2', source)
        self.assertIn('merged["supervision"] = supervision', source)
        self.assertIn('merged["events"] = self.store.events.snapshot()', source)

    def test_telemetry_contract_file_is_untouched_by_supervision(self):
        source = read(os.path.join(_PACKAGE_ROOT, "dashboard", "telemetry.py"))
        self.assertNotIn("supervision", source.lower())
        self.assertNotIn("safety", source.lower())


class FrontendLayoutTests(unittest.TestCase):
    def setUp(self):
        self.html = read(os.path.join(WEB_DIR, "index.html"))
        self.css = read(os.path.join(WEB_DIR, "style.css"))
        self.app = read(os.path.join(WEB_DIR, "app.js"))
        self.session = read(os.path.join(WEB_DIR, "session.js"))

    def test_section_order_is_robots_camera_events_comms_joints(self):
        # DOM order follows visual reading order (placement is CSS grid driven)
        order = [
            'id="card-robots"',
            'id="card-camera"',
            'id="card-events"',
            'id="card-comms"',
            'id="card-joints"',
        ]
        positions = [self.html.index(marker) for marker in order]
        self.assertEqual(positions, sorted(positions))
        self.assertLess(
            self.html.index('id="panel-leader"'),
            self.html.index('id="panel-follower"'),
        )

    def test_four_supervision_pills_live_in_topbar(self):
        topbar = self.html[self.html.index("<header"):self.html.index("</header>")]
        for dimension in ("health", "readiness", "safety", "session"):
            self.assertIn(f'id="sup-{dimension}"', topbar)
        self.assertEqual(self.html.count('class="sup-pill'), 4)
        self.assertNotIn('id="card-supervision"', self.html)

    def test_there_is_one_main_region(self):
        self.assertEqual(self.html.count("<main>"), 1)
        self.assertEqual(self.html.count("</main>"), 1)

    def test_drawer_and_event_history_exist(self):
        self.assertIn('id="supervision-drawer"', self.html)
        self.assertIn('role="dialog"', self.html)
        self.assertIn('aria-modal="true"', self.html)
        self.assertIn('id="event-filters"', self.html)
        self.assertIn('id="drawer-event-log"', self.html)
        self.assertIn("events.slice(-3)", self.app)

    def test_single_screen_and_compact_breakpoints(self):
        self.assertIn("@media (min-width: 1200px)", self.css)
        self.assertIn(
            "@media (min-width: 1200px) and (max-height: 800px)", self.css
        )
        self.assertIn(
            "@media (max-width: 1199px), (max-height: 699px)", self.css
        )
        self.assertIn("body.drawer-open", self.css)
        self.assertIn("height: 100dvh", self.css)

    def test_grid_columns_and_quarter_layout(self):
        # Eight-column grid: robots 1/4 (2 cols) + camera 3/4 (6 cols).
        self.assertIn(
            "grid-template-columns: repeat(8, minmax(0, 1fr));", self.css
        )
        self.assertIn("#card-robots { grid-row: 1; grid-column: 1 / 3;", self.css)
        self.assertIn("#card-camera { grid-row: 1; grid-column: 3 / 9;", self.css)
        # Row 2: alerts under robots, comms + joints under camera.
        self.assertIn("#card-events { grid-row: 2; grid-column: 1 / 3;", self.css)
        self.assertIn("#card-comms { grid-row: 2; grid-column: 3 / 6;", self.css)
        self.assertIn("#card-joints { grid-row: 2; grid-column: 6 / 9;", self.css)

    def test_overview_cards_removed_and_comms_body_exists(self):
        # Top overview cards duplicated comms/joint telemetry, so they are gone.
        self.assertNotIn('id="card-overview"', self.html)
        self.assertNotIn('class="stat-row"', self.html)
        self.assertNotIn("renderOverview", self.app)
        self.assertIn('class="comms-body"', self.html)

    def test_drawer_event_history_stacks_message_below_header(self):
        # Long source labels used to overflow their column into the message.
        self.assertIn("#drawer-event-log li {", self.css)
        self.assertIn('"time level source"', self.css)
        self.assertIn('"message message message"', self.css)
        self.assertIn("#drawer-event-log .event-source {", self.css)
        self.assertIn("text-overflow: ellipsis;", self.css)

    def test_diagnostics_report_overflow(self):
        self.assertIn('data-overflow', self.html)
        self.assertIn('data-horizontal-overflow', self.html)
        self.assertIn("main.scrollHeight - main.clientHeight", self.app)
        self.assertIn("main.scrollWidth - main.clientWidth", self.app)

    def test_web_assets_never_call_ros_or_write_endpoints(self):
        for source in (self.app, self.session):
            for forbidden in ("/cmd", "yam_teleop"):
                self.assertNotIn(forbidden, source)
        self.assertEqual(self.app.count("fetch("), 1)
        self.assertIn('fetch("/api/state", { cache: "no-store" })', self.app)
        self.assertNotIn("method:", self.app)

    def test_session_transition_tables_match_python(self):
        transitions = js_object(self.session, "TRANSITIONS")
        actions = js_object(self.session, "ACTIONS")
        self.assertEqual(
            {state: tuple(targets) for state, targets in transitions.items()},
            dict(SESSION_TRANSITIONS),
        )
        self.assertEqual(
            {state: tuple(items) for state, items in actions.items()},
            dict(SESSION_ACTIONS),
        )

    def test_session_self_test_hook_exists(self):
        self.assertIn('params.get("selftest") === "1"', self.app)
        self.assertIn("runSelfTest", self.session)
        self.assertIn('getElementById("session-selftest")', self.session)


if __name__ == "__main__":
    unittest.main(verbosity=2)
