#!/usr/bin/env python3
"""Optional end-to-end harness for M3 Phase 1c/1d (not part of default pytest).

Starts the Phase 1b dashboard, the standalone gateway, two unchanged synthetic
H.264/RTP senders from the Phase 0 spike, and headless Chrome. It verifies two
independent browser video connections, independent failure/recovery, gateway
stats, and the no-transcode element dumps. ``--page`` selects either the
Phase 1c diagnostic harness (default) or the Phase 1d production dashboard.

Requires GStreamer and Google Chrome. Run under WSL Ubuntu-24.04.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import shutil
import signal
import subprocess
import sys
import time
import urllib.request

sys.dont_write_bytecode = True

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.dirname(TEST_DIR)
REPO_ROOT = os.path.dirname(PACKAGE_ROOT)
SPIKE_DIR = os.path.join(PACKAGE_ROOT, "spike", "webrtc_poc")
if SPIKE_DIR not in sys.path:
    sys.path.insert(0, SPIKE_DIR)
# Package root must come first: the spike contains gateway.py, which would
# otherwise shadow the standalone gateway package.
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

import cdp  # noqa: E402  (unchanged spike client, imported without bytecode writes)
from gateway.signaling_client import SignalingClient  # noqa: E402

CAMERAS = ("d405", "d455f")
PORTS = {"d405": 5100, "d455f": 5102}
PAGES = {
    "diagnostic": {
        "path": "/webrtc_diagnostic.html",
        "ready": "window.__webrtcDiagnosticReady",
        "snapshot": (
            "window.__webrtcDiagnostic"
            " ? window.__webrtcDiagnostic.snapshot() : null"
        ),
        "retry": "window.__webrtcDiagnostic.retry('d405')",
    },
    "dashboard": {
        "path": "/",
        "ready": "window.__YAM_WEBRTC_READY__",
        "snapshot": (
            "window.__YAM_WEBRTC__"
            " ? window.__YAM_WEBRTC__.snapshot() : null"
        ),
        "retry": "window.__YAM_WEBRTC__.retry('d405')",
    },
}
BANNED_FACTORIES = {
    "avdec_h264", "decodebin", "decodebin3", "openh264dec", "vah264dec",
    "vaapih264dec", "nvh264dec", "vtdec", "x264enc", "openh264enc",
    "vah264enc", "vaapih264enc", "nvh264enc", "venc", "encodebin",
    "videoconvert", "videoscale", "videorate",
}

SENDER_WRAPPER = r"""
import importlib.util
import sys
sys.dont_write_bytecode = True
run_dir, path = sys.argv[1], sys.argv[2]
sys.argv = [path] + sys.argv[3:]
spec = importlib.util.spec_from_file_location("yam_spike_sender", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.RUN_DIR = run_dir
raise SystemExit(module.main())
"""


def wait_http(url: str, timeout: float = 20.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with urllib.request.urlopen(url, timeout=2.0) as response:
                if response.status == 200:
                    return
        except Exception:  # noqa: BLE001 - polling
            pass
        time.sleep(0.2)
    raise RuntimeError(f"timed out waiting for {url}")


def spawn(name, command, env, run_dir, shell=False):
    log_path = os.path.join(run_dir, f"e2e-{name}.log")
    log = open(log_path, "wb")
    process = subprocess.Popen(
        command,
        stdout=log,
        stderr=subprocess.STDOUT,
        cwd=REPO_ROOT,
        env=env,
        shell=shell,
        start_new_session=True,
    )
    return process, log_path


def sender_args(run_dir, camera_id):
    sender_path = os.path.join(SPIKE_DIR, "sender.py")
    label = camera_id.upper()
    background = "0x102030FF" if camera_id == "d405" else "0x301020FF"
    foreground = "0xFFFFFFFF" if camera_id == "d405" else "0xFFFFD0FF"
    return [
        sys.executable, "-c", SENDER_WRAPPER, run_dir, sender_path,
        "--id", camera_id, "--label", label,
        "--port", str(PORTS[camera_id]),
        "--background", background, "--foreground", foreground,
    ]


def terminate(process, log_path=None):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        process.terminate()
    try:
        process.wait(timeout=4.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            process.kill()
        process.wait(timeout=2.0)


def snapshot(ws, page):
    return ws.evaluate(PAGES[page]["snapshot"], timeout=15) or {}


def wait_ready(ws, page, timeout=30.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            if ws.evaluate("document.readyState", timeout=5) == "complete" and \
                    ws.evaluate(f"Boolean({PAGES[page]['ready']})", timeout=5):
                return
        except cdp.CDPError:
            pass
        time.sleep(0.5)
    raise RuntimeError(f"{page} page did not become ready")


def verify_page_dom(ws, page):
    if page == "diagnostic":
        expression = (
            "document.querySelectorAll('video[id^=\"video-\"]').length === 2"
        )
    else:
        expression = (
            "document.getElementById('camera-webrtc')"
            " && !document.getElementById('camera-webrtc').hidden"
            " && document.getElementById('camera-frame').hidden"
            " && document.querySelectorAll('#camera-webrtc video').length === 2"
        )
    if not ws.evaluate(expression, timeout=10):
        raise RuntimeError(f"{page} page does not show two camera panels")


def wait_live(ws, cameras, page, timeout=120.0):
    deadline = time.monotonic() + timeout
    last = {}
    while time.monotonic() < deadline:
        last = snapshot(ws, page)
        entries = last.get("cameras", {})
        if all(
            entries.get(cam, {}).get("live")
            and entries.get(cam, {}).get("browser", {}).get("framesDecoded", 0) > 0
            for cam in cameras
        ):
            return last
        time.sleep(1.0)
    raise RuntimeError("timed out waiting for live cameras: " + json.dumps(last))


def decoded(snap, camera_id):
    return int(
        snap.get("cameras", {}).get(camera_id, {})
        .get("browser", {}).get("framesDecoded", 0) or 0
    )


def verify_no_transcode(run_dir):
    files = sorted(glob.glob(os.path.join(run_dir, "gateway-elements-*.txt")))
    if len(files) != 2:
        raise RuntimeError(f"expected 2 element dumps, found {len(files)}")
    for path in files:
        with open(path, encoding="utf-8") as handle:
            factories = {line.strip() for line in handle if line.strip()}
        hits = factories & BANNED_FACTORIES
        if hits:
            raise RuntimeError(f"{os.path.basename(path)} contains {sorted(hits)}")
    return [os.path.basename(path) for path in files]


def verify_gateway_stats(base_url):
    client = SignalingClient(base_url)
    sessions = client.active_sessions()
    if not sessions:
        raise RuntimeError("no active signaling session found")
    session_id = sessions[0].session_id  # never written to results or logs
    observed = {}
    for camera_id in CAMERAS:
        payload = client.get_stats(session_id, camera_id, timeout_s=2.0)
        if not payload or not payload.get("stats"):
            raise RuntimeError(f"no gateway stats for {camera_id}")
        stats = payload["stats"]
        if stats.get("source") != "gateway":
            raise RuntimeError(f"{camera_id} stats are not labeled as gateway stats")
        observed[camera_id] = {
            "health": payload["health"],
            "ingress_packets": stats.get("ingress_packets", 0),
            "negotiated_pt": stats.get("negotiated_pt"),
        }
    return observed


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--page",
        choices=sorted(PAGES),
        default="diagnostic",
        help="browser page under test (default: diagnostic)",
    )
    parser.add_argument("--dashboard-port", type=int, default=18180)
    parser.add_argument("--debug-port", type=int, default=9223)
    parser.add_argument(
        "--run-dir",
        default=os.path.join(PACKAGE_ROOT, "gateway", "run", "e2e"),
    )
    parser.add_argument(
        "--check-retry",
        action="store_true",
        help=(
            "best-effort browser renegotiation probe; recorded in results but "
            "not an acceptance criterion"
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_dir = os.path.abspath(args.run_dir)
    shutil.rmtree(run_dir, ignore_errors=True)
    os.makedirs(run_dir, exist_ok=True)

    base_url = f"http://127.0.0.1:{args.dashboard_port}"
    debug_base = f"http://127.0.0.1:{args.debug_port}"
    page_url = base_url + PAGES[args.page]["path"]

    env = os.environ.copy()
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    env["PYTHONUNBUFFERED"] = "1"

    processes = {}
    logs = {}
    ws = None
    results = {"passed": False, "page": args.page, "run_dir": run_dir}

    try:
        dashboard_cmd = (
            "source /opt/ros/jazzy/setup.bash && exec python3 "
            f"{os.path.join(PACKAGE_ROOT, 'dashboard', 'run.py')} "
            f"--ros-args -p webrtc.enabled:=true -p port:={args.dashboard_port}"
        )
        processes["dashboard"], logs["dashboard"] = spawn(
            "dashboard", ["bash", "-lc", dashboard_cmd], env, run_dir, shell=False
        )
        wait_http(base_url + "/api/state")

        processes["gateway"], logs["gateway"] = spawn(
            "gateway",
            [
                sys.executable, os.path.join(PACKAGE_ROOT, "gateway", "run.py"),
                "--signaling", base_url, "--run-dir", run_dir,
            ],
            env,
            run_dir,
        )
        wait_http(base_url + "/api/webrtc/sessions")

        for camera_id in CAMERAS:
            processes[f"sender_{camera_id}"], logs[f"sender_{camera_id}"] = spawn(
                f"sender_{camera_id}", sender_args(run_dir, camera_id), env, run_dir
            )
        time.sleep(2.0)
        for camera_id in CAMERAS:
            if processes[f"sender_{camera_id}"].poll() is not None:
                raise RuntimeError(f"{camera_id} sender exited early")

        chrome = shutil.which("google-chrome") or "google-chrome"
        profile = os.path.join(run_dir, "chrome-profile")
        shutil.rmtree(profile, ignore_errors=True)
        chrome_cmd = [
            chrome,
            "--headless=new", "--no-sandbox", "--disable-gpu",
            "--disable-dev-shm-usage", "--no-first-run",
            "--no-default-browser-check", "--disable-sync", "--disable-extensions",
            "--disable-background-networking", "--disable-component-update",
            "--disable-background-timer-throttling", "--disable-renderer-backgrounding",
            "--disable-backgrounding-occluded-windows",
            "--autoplay-policy=no-user-gesture-required",
            "--disable-features=WebRtcHideLocalIpsWithMdns",
            "--window-size=1600,900",
            f"--remote-debugging-address=127.0.0.1",
            f"--remote-debugging-port={args.debug_port}",
            f"--user-data-dir={profile}",
            "about:blank",
        ]
        processes["chrome"], logs["chrome"] = spawn(
            "chrome", chrome_cmd, env, run_dir
        )
        wait_http(debug_base + "/json/version", timeout=30.0)

        target = cdp.open_target(debug_base, page_url)
        ws = cdp.WebSocket(target["webSocketDebuggerUrl"], timeout=30.0)
        wait_ready(ws, args.page)

        wait_live(ws, CAMERAS, args.page)
        verify_page_dom(ws, args.page)
        results["both_streams_live"] = True

        # Stop d405 only; d455f must keep producing decoded frames.
        before_stop = snapshot(ws, args.page)
        terminate(processes.pop("sender_d405"), logs.get("sender_d405"))
        time.sleep(3.0)
        after_stop = snapshot(ws, args.page)
        other_kept_decoding = (
            decoded(after_stop, "d455f") > decoded(before_stop, "d455f")
            and after_stop.get("cameras", {}).get("d455f", {}).get("live")
        )
        if not other_kept_decoding:
            raise RuntimeError("d455f stopped decoding when d405 sender was stopped")
        results["independent_failure"] = True

        # Restart only the d405 sender. Its existing webrtcbin pipeline stays
        # bound, so RTP resumes without touching d455f or its peer connection.
        processes["sender_d405"], logs["sender_d405"] = spawn(
            "sender_d405", sender_args(run_dir, "d405"), env, run_dir
        )
        recovered = wait_live(ws, CAMERAS, args.page, timeout=90.0)
        if not recovered.get("cameras", {}).get("d455f", {}).get("live"):
            raise RuntimeError("d455f was disrupted by d405 recovery")
        results["independent_recovery"] = True
        results["gateway_stats"] = verify_gateway_stats(base_url)

        if args.check_retry:
            # Browser-side renegotiation is a diagnostic convenience, not an
            # acceptance criterion. The required recovery path above restarts
            # only the stopped synthetic sender and leaves both peer
            # connections untouched.
            try:
                ws.evaluate(
                    PAGES[args.page]["retry"],
                    await_promise=True,
                    timeout=30,
                )
                wait_live(ws, CAMERAS, args.page, timeout=90.0)
                results["per_camera_retry_best_effort"] = {"passed": True}
            except Exception as exc:  # noqa: BLE001 - best-effort diagnostic
                results["per_camera_retry_best_effort"] = {
                    "passed": False,
                    "note": (
                        "browser renegotiation is best effort and is not an "
                        "acceptance criterion"
                    ),
                    "error": str(exc)[:500],
                }

        results["no_transcode_dumps"] = verify_no_transcode(run_dir)
        results["passed"] = True
        print(json.dumps(results, indent=2))
        return 0
    except Exception as exc:  # noqa: BLE001 - harness reports failures as JSON
        results["error"] = str(exc)
        results["logs"] = logs
        print(json.dumps(results, indent=2), file=sys.stderr)
        return 1
    finally:
        if ws is not None:
            ws.close()
        for name in ("chrome", "sender_d405", "sender_d455f", "gateway", "dashboard"):
            terminate(processes.get(name), logs.get(name))
        with open(os.path.join(run_dir, "e2e_results.json"), "w", encoding="utf-8") as handle:
            json.dump(results, handle, indent=2)


if __name__ == "__main__":
    raise SystemExit(main())
