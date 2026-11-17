/* YAM Operator Console — read-only supervision.
 *
 * Presentation layer only. It consumes the JSON snapshot produced by the ROS
 * node and never speaks to ROS directly. The four supervision dimensions are
 * displayed independently; no combined "overall" verdict is stored.
 *
 * The forward-kinematics + projection code below is derived from the real
 * URDF joint origins/axes in yam_description/urdf/yam_joints.xacro, so the
 * drawing is driven by live joint values rather than being decorative.
 */

"use strict";

/* ------------------------------------------------------------------ utils */

const SVG_NS = "http://www.w3.org/2000/svg";

const $ = (id) => document.getElementById(id);

function setText(id, text) {
  const node = $(id);
  if (node) node.textContent = text;
  return node;
}

function svgEl(tag, attrs, parent) {
  const node = document.createElementNS(SVG_NS, tag);
  if (attrs) {
    for (const key of Object.keys(attrs)) node.setAttribute(key, attrs[key]);
  }
  if (parent) parent.appendChild(node);
  return node;
}

function fmt(value, digits = 3) {
  if (value === null || value === undefined || Number.isNaN(value)) return "\u2014";
  let text = Number(value).toFixed(digits);
  if (/^-0\.?0*$/.test(text)) text = text.slice(1);
  return text;
}

function fmtAge(ms) {
  if (ms === null || ms === undefined || Number.isNaN(ms)) return "\u2014";
  if (ms < 1000) return `${Math.round(ms)} ms`;
  return `${(ms / 1000).toFixed(2)} s`;
}

function fmtRate(hz) {
  if (hz === null || hz === undefined || Number.isNaN(hz)) return "unavailable";
  return `${hz.toFixed(1)} Hz`;
}

function fmtDuration(seconds) {
  if (seconds === null || seconds === undefined || Number.isNaN(Number(seconds))) {
    return "\u2014";
  }
  const total = Math.max(0, Math.floor(Number(seconds)));
  const hours = Math.floor(total / 3600);
  const minutes = Math.floor((total % 3600) / 60);
  const secs = total % 60;
  const mm = String(minutes).padStart(2, "0");
  const ss = String(secs).padStart(2, "0");
  return hours > 0 ? `${hours}:${mm}:${ss}` : `${mm}:${ss}`;
}

function stateClass(state) {
  return `state-${state || "disconnected"}`;
}

function pillClass(state) {
  return `pill pill-${state || "unknown"}`;
}

function shortTime(iso) {
  const parsed = new Date(iso);
  if (Number.isNaN(parsed.getTime())) return String(iso || "").slice(11, 19);
  return parsed.toLocaleTimeString([], { hour12: false });
}

function setClass(element, className) {
  if (element) element.className = className;
}

/* ------------------------------------------------------- kinematics (URDF)
 * Each entry: joint origin xyz/rpy relative to its parent link, the rotation
 * axis, and the resulting child link. Values copied from yam_joints.xacro.
 */

const CHAIN = [
  { name: "joint_1", xyz: [0, 0, 0.068],                                   rpy: [0, 1.5708, 3.1416],                        axis: [-1, 0, 0] },
  { name: "joint_2", xyz: [-0.0455, -0.0339, -0.02],                        rpy: [0, 0, -3.14159],                           axis: [0, 1, 0] },
  { name: "joint_3", xyz: [0, -0.0688, 0.264],                              rpy: [0, 0, -3.14159],                           axis: [0, 1, 0] },
  { name: "joint_4", xyz: [-0.0600003, -0.0688, -0.244999],                 rpy: [0, 0, 0],                                  axis: [0, 1, 0] },
  { name: "joint_5", xyz: [-0.0405003, 0.0338995, -0.0739989],              rpy: [0, 0, 0],                                  axis: [1, 0, 0] },
  { name: "joint_6", xyz: [0.0404996, 0, -0.0356],                          rpy: [3.1415926535897927, 0, 1.5707963267949736], axis: [0, 0, 1] },
];

const TOOL_LENGTH_M = 0.09;

function identity() {
  return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
}

function multiply(a, b) {
  const out = new Array(16).fill(0);
  for (let r = 0; r < 4; r += 1) {
    for (let c = 0; c < 4; c += 1) {
      let sum = 0;
      for (let k = 0; k < 4; k += 1) sum += a[r * 4 + k] * b[k * 4 + c];
      out[r * 4 + c] = sum;
    }
  }
  return out;
}

function translation(x, y, z) {
  const m = identity();
  m[3] = x; m[7] = y; m[11] = z;
  return m;
}

function rotX(a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [1, 0, 0, 0, 0, c, -s, 0, 0, s, c, 0, 0, 0, 0, 1];
}

function rotY(a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [c, 0, s, 0, 0, 1, 0, 0, -s, 0, c, 0, 0, 0, 0, 1];
}

function rotZ(a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
}

function rpy(r, p, y) {
  return multiply(rotZ(y), multiply(rotY(p), rotX(r)));
}

function axisAngle(axis, angle) {
  const [ax, ay, az] = axis;
  const norm = Math.hypot(ax, ay, az) || 1;
  const x = ax / norm, y = ay / norm, z = az / norm;
  const c = Math.cos(angle), s = Math.sin(angle), t = 1 - c;
  return [
    t * x * x + c,     t * x * y - s * z, t * x * z + s * y, 0,
    t * x * y + s * z, t * y * y + c,     t * y * z - s * x, 0,
    t * x * z - s * y, t * y * z + s * x, t * z * z + c,     0,
    0, 0, 0, 1,
  ];
}

function originOf(m) {
  return [m[3], m[7], m[11]];
}

function forwardKinematics(joints) {
  let frame = identity();
  const points = [{ name: "base", point: [0, 0, 0] }];

  for (const segment of CHAIN) {
    const raw = joints ? joints[segment.name] : undefined;
    const q = Number.isFinite(Number(raw)) ? Number(raw) : 0;

    frame = multiply(frame, translation(...segment.xyz));
    frame = multiply(frame, rpy(...segment.rpy));
    frame = multiply(frame, axisAngle(segment.axis, q));

    points.push({ name: segment.name, point: originOf(frame) });
  }

  const tool = multiply(frame, translation(0, 0, TOOL_LENGTH_M));
  points.push({ name: "tool", point: originOf(tool) });
  return points;
}

/* Fixed orthographic camera: azimuth/elevation chosen so the arm's working
 * plane is readable and vertical motion stays vertical on screen. */
const VIEW = { width: 420, height: 330, cx: 198, cy: 292, scale: 520 };
const AZIMUTH = -0.9;
const ELEVATION = 0.38;

function project(point) {
  const ca = Math.cos(AZIMUTH), sa = Math.sin(AZIMUTH);
  const ce = Math.cos(ELEVATION), se = Math.sin(ELEVATION);
  const sx = -sa * point[0] + ca * point[1];
  const sy = -ca * se * point[0] - sa * se * point[1] + ce * point[2];
  return [VIEW.cx + sx * VIEW.scale, VIEW.cy - sy * VIEW.scale];
}

/* ------------------------------------------------------------ robot view */

function drawGrid(root) {
  const grid = svgEl("g", { stroke: "#16222e", "stroke-width": "1" }, root);
  for (let x = 30; x < VIEW.width; x += 30) {
    svgEl("line", { x1: x, y1: 0, x2: x, y2: VIEW.height }, grid);
  }
  for (let y = 30; y < VIEW.height; y += 30) {
    svgEl("line", { x1: 0, y1: y, x2: VIEW.width, y2: y }, grid);
  }
}

function drawArm(root, joints, color, dimmed) {
  const points = forwardKinematics(joints).map((entry) => ({
    name: entry.name,
    xy: project(entry.point),
  }));

  const group = svgEl("g", {
    opacity: dimmed ? "0.4" : "1",
    "stroke-linecap": "round",
  }, root);

  // Base plate.
  const base = points[0].xy;
  svgEl("ellipse", {
    cx: base[0], cy: base[1] + 6, rx: 34, ry: 9,
    fill: "rgba(148,163,184,0.10)", stroke: "#2b3a4b", "stroke-width": "1.5",
  }, group);

  // Links: thicker near the base, thinner towards the tool.
  for (let i = 1; i < points.length; i += 1) {
    const width = Math.max(3.5, 10 - i * 1.1);
    svgEl("line", {
      x1: points[i - 1].xy[0], y1: points[i - 1].xy[1],
      x2: points[i].xy[0], y2: points[i].xy[1],
      stroke: color, "stroke-width": width.toFixed(1),
      opacity: i > 6 ? "0.55" : "0.92",
    }, group);
  }

  // Revolute joints + labels.
  for (let i = 1; i <= CHAIN.length; i += 1) {
    const [x, y] = points[i].xy;
    svgEl("circle", {
      cx: x, cy: y, r: 5.2,
      fill: "#0b1119", stroke: color, "stroke-width": "2",
    }, group);
    const label = svgEl("text", {
      x: x + 9, y: y - 7,
      fill: "#5d7186", "font-size": "9.5",
      "font-family": "ui-monospace, monospace",
    }, group);
    label.textContent = `J${i}`;
  }

  // Tool / gripper.
  const tool = points[points.length - 1].xy;
  const wrist = points[points.length - 2].xy;
  const dx = tool[0] - wrist[0];
  const dy = tool[1] - wrist[1];
  const len = Math.hypot(dx, dy) || 1;
  const nx = -dy / len, ny = dx / len;
  for (const sign of [-1, 1]) {
    svgEl("line", {
      x1: tool[0], y1: tool[1],
      x2: tool[0] + nx * 11 * sign + (dx / len) * 9,
      y2: tool[1] + ny * 11 * sign + (dy / len) * 9,
      stroke: color, "stroke-width": "2.4", opacity: "0.8",
    }, group);
  }
}

function drawOverlay(root, title, subtitle) {
  svgEl("rect", {
    x: 0, y: 0, width: VIEW.width, height: VIEW.height,
    fill: "rgba(8,12,17,0.66)",
  }, root);

  const titleText = svgEl("text", {
    x: VIEW.width / 2, y: VIEW.height / 2 - 4,
    "text-anchor": "middle",
    fill: "#f87171", "font-size": "13",
    "font-weight": "700", "letter-spacing": "2",
    "font-family": "ui-monospace, monospace",
  }, root);
  titleText.textContent = title;

  if (subtitle) {
    const subText = svgEl("text", {
      x: VIEW.width / 2, y: VIEW.height / 2 + 16,
      "text-anchor": "middle",
      fill: "#93a4b8", "font-size": "10.5",
      "font-family": "ui-monospace, monospace",
    }, root);
    subText.textContent = subtitle;
  }
}

function renderRobot(role, stream, color) {
  const svg = $(`viz-${role}`);
  const stateEl = $(`viz-state-${role}`);
  if (!svg) return;

  while (svg.firstChild) svg.removeChild(svg.firstChild);
  drawGrid(svg);

  const hasData = stream && stream.count > 0;
  const connected = stream && stream.state === "connected";
  const joints = hasData ? stream.joints : null;

  drawArm(svg, joints, color, !connected);

  if (!hasData) {
    drawOverlay(svg, "NO TELEMETRY", "waiting for joint states");
    if (stateEl) {
      stateEl.textContent = "no data";
      stateEl.className = "robot-state state-disconnected";
    }
  } else if (!connected) {
    drawOverlay(
      svg,
      stream.state === "stale" ? "STALE" : stream.state.toUpperCase(),
      `last message ${fmtAge(stream.age_ms)} ago`
    );
    if (stateEl) {
      stateEl.textContent = stream.state;
      stateEl.className = `robot-state ${stateClass(stream.state)}`;
    }
  } else if (stateEl) {
    stateEl.textContent = "live";
    stateEl.className = "robot-state state-connected";
  }
}

/* ------------------------------------------------------ per-robot summaries */

function renderRobotSummaries(data) {
  const streams = data.streams || {};
  const tracking = data.tracking || {};
  for (const role of ["leader", "follower"]) {
    const stream = streams[role] || {};
    const foot = $(`viz-foot-${role}`);
    if (!foot) continue;
    let text;
    if (role === "follower" && tracking.available) {
      text = `${fmtRate(stream.rate_hz)} \u00b7 max |err| `
        + `${fmt(tracking.max_abs_error_rad * 1000, 1)} mrad \u00b7 RMS `
        + `${fmt(tracking.rms_error_rad * 1000, 1)} mrad`;
    } else if (stream.count) {
      text = `${stream.state_label || stream.state || "\u2014"} \u00b7 `
        + `${fmtRate(stream.rate_hz)} \u00b7 age ${fmtAge(stream.age_ms)}`;
    } else {
      text = "awaiting telemetry";
    }
    foot.textContent = text;
  }
}

/* ---------------------------------------------------------- joint telemetry */

function renderJoints(data) {
  const body = $("joint-table").querySelector("tbody");
  const tracking = data.tracking || {};
  const streams = data.streams || {};
  const config = data.config || {};
  const warn = config.max_error_warn_rad || 0.15;

  body.textContent = "";

  const rows = tracking.available ? tracking.per_joint : [];
  const leader = (streams.leader || {}).joints || {};
  const follower = (streams.follower || {}).joints || {};
  const names = rows.length
    ? rows.map((row) => row.name)
    : Object.keys(leader).length ? Object.keys(leader) : Object.keys(follower);

  if (!names.length) {
    const tr = document.createElement("tr");
    const td = document.createElement("td");
    td.colSpan = 6;
    td.className = "val-muted";
    td.textContent = "No joint-state data received yet.";
    tr.appendChild(td);
    body.appendChild(tr);
    setText("joint-sub", "\u2014");
    setText("joint-note", "Waiting for /leader/joint_states and /follower/joint_states.");
    return;
  }

  for (const name of names) {
    const row = rows.find((entry) => entry.name === name) || null;
    const tr = document.createElement("tr");

    const jointCell = document.createElement("td");
    jointCell.className = "joint-name";
    jointCell.textContent = name.replace("_", " ");
    if (row && row.static) {
      const tag = document.createElement("span");
      tag.className = "tag";
      tag.textContent = "STATIC";
      jointCell.appendChild(tag);
    }
    tr.appendChild(jointCell);

    const leaderCell = document.createElement("td");
    leaderCell.className = "val-leader";
    leaderCell.textContent = fmt(leader[name]);
    tr.appendChild(leaderCell);

    const followerCell = document.createElement("td");
    followerCell.className = "val-follower";
    followerCell.textContent = fmt(follower[name]);
    tr.appendChild(followerCell);

    const errorCell = document.createElement("td");
    const mradCell = document.createElement("td");
    const barCell = document.createElement("td");
    barCell.className = "bar-col";

    if (row) {
      const magnitude = Math.abs(row.error_rad);
      errorCell.textContent = fmt(row.error_rad);
      errorCell.style.color = magnitude >= warn ? "var(--bad)"
        : magnitude >= warn * 0.5 ? "var(--warn)" : "var(--ok)";

      mradCell.textContent = fmt(row.error_mrad, 1);
      mradCell.style.color = magnitude >= warn ? "var(--bad)"
        : magnitude >= warn * 0.5 ? "var(--warn)" : "var(--text-dim)";

      const halfWidth = Math.min(50, (magnitude / warn) * 50);
      const track = document.createElement("div");
      track.className = "bar-track";
      const fill = document.createElement("div");
      fill.className = "bar-fill";
      fill.style.width = `${halfWidth}%`;
      fill.style.left = row.error_rad >= 0 ? "50%" : `${50 - halfWidth}%`;
      fill.style.background = magnitude >= warn ? "var(--bad)"
        : magnitude >= warn * 0.5 ? "var(--warn)" : "var(--ok)";
      const center = document.createElement("div");
      center.className = "bar-center";
      track.appendChild(fill);
      track.appendChild(center);
      barCell.appendChild(track);
    } else {
      errorCell.textContent = "\u2014";
      mradCell.textContent = "\u2014";
      errorCell.className = "val-muted";
      mradCell.className = "val-muted";
    }

    tr.appendChild(errorCell);
    tr.appendChild(mradCell);
    tr.appendChild(barCell);
    body.appendChild(tr);
  }

  $("joint-sub").textContent = tracking.available
    ? `max ${fmt(tracking.max_abs_error_rad * 1000, 1)} mrad \u00b7 RMS ${fmt(tracking.rms_error_rad * 1000, 1)} mrad`
    : "tracking unavailable";

  const note = $("joint-note");
  if (tracking.available && tracking.note) {
    note.innerHTML = `<strong>Note:</strong> ${tracking.note}`;
  } else if (tracking.available) {
    note.textContent = "Error is computed per joint as leader \u2212 follower.";
  } else {
    note.textContent = tracking.note || "Tracking error requires both streams.";
  }
}

/* ------------------------------------------------------ communication view */

function renderComms(data) {
  const body = $("comm-table").querySelector("tbody");
  const streams = data.streams || {};
  const config = data.config || {};
  body.textContent = "";

  for (const role of ["leader", "follower"]) {
    const stream = streams[role] || {};
    const tr = document.createElement("tr");

    const nameCell = document.createElement("td");
    nameCell.className = "joint-name";
    nameCell.textContent = role;
    tr.appendChild(nameCell);

    const stateCell = document.createElement("td");
    const pill = document.createElement("span");
    pill.className = pillClass(stream.state);
    pill.style.fontSize = "10.5px";
    pill.style.padding = "2px 8px 2px 7px";
    const dot = document.createElement("span");
    dot.className = "dot";
    const text = document.createElement("span");
    text.textContent = stream.state_label || "\u2014";
    pill.appendChild(dot);
    pill.appendChild(text);
    stateCell.appendChild(pill);
    tr.appendChild(stateCell);

    const rateCell = document.createElement("td");
    rateCell.textContent = fmtRate(stream.rate_hz);
    tr.appendChild(rateCell);

    const ageCell = document.createElement("td");
    ageCell.textContent = fmtAge(stream.age_ms);
    tr.appendChild(ageCell);

    const countCell = document.createElement("td");
    countCell.textContent = stream.count === undefined ? "\u2014" : String(stream.count);
    tr.appendChild(countCell);

    const deltaCell = document.createElement("td");
    deltaCell.textContent = stream.source_dt_ms === null || stream.source_dt_ms === undefined
      ? "\u2014" : `${stream.source_dt_ms.toFixed(1)} ms`;
    tr.appendChild(deltaCell);

    body.appendChild(tr);
  }

  $("latency-display").textContent = (data.latency || {}).display
    || "Unavailable \u2014 no valid latency measurement source";
  const latencyReason = $("latency-reason");
  if (latencyReason) latencyReason.textContent = (data.latency || {}).reason || "";
}

/* ---------------------------------------------------------------- camera */

const WEBRTC_CAMERAS = ["d405", "d455f"];
const WEBRTC_INACTIVE_PHASES = [
  "starting", "checking-signaling", "disabled", "stopped",
];

let webrtcController = null;
let webrtcVisible = false;
let webrtcSummary = null;

function setCameraBadge(label, warning) {
  setText("camera-badge", label);
  const badge = $("camera-badge");
  if (badge) badge.className = warning ? "badge badge-warn" : "badge";
}

function renderCamera(data) {
  const camera = (data && data.camera) || {};
  const frame = $("camera-frame");
  const webrtcFrame = $("camera-webrtc");
  const pending = $("camera-pending");
  const mount = $("camera-mount");

  // WebRTC status is presentation-only. It never feeds /api/state,
  // supervision, readiness, safety or the operator session.
  if (frame) frame.hidden = webrtcVisible;
  if (webrtcFrame) webrtcFrame.hidden = !webrtcVisible;

  if (webrtcVisible) {
    if (pending) pending.hidden = true;
    if (mount) mount.hidden = true;
    const entries = (webrtcSummary && webrtcSummary.cameras) || {};
    const liveCount = WEBRTC_CAMERAS.filter(
      (id) => entries[id] && entries[id].live
    ).length;
    const phase = webrtcSummary ? webrtcSummary.phase : "starting";
    if (liveCount === WEBRTC_CAMERAS.length) {
      setCameraBadge("Live", false);
    } else if (phase === "start-failed" || phase === "session-expired") {
      setCameraBadge("Unavailable", true);
    } else if (liveCount > 0) {
      setCameraBadge("Partial", true);
    } else {
      setCameraBadge("Connecting", true);
    }
    return;
  }

  // Existing fallback: unchanged when WebRTC signaling is disabled.
  setText("camera-title", "Camera feed unavailable");
  setText(
    "camera-reason",
    "Connect the video gateway to show D405 and D455f feeds."
  );
  if (camera.stream_url) {
    if (pending) pending.hidden = true;
    if (mount) {
      mount.hidden = false;
      if (!mount.querySelector("img")) {
        const img = document.createElement("img");
        img.alt = "Live camera feed";
        img.src = camera.stream_url;
        mount.appendChild(img);
      }
    }
    setCameraBadge("Live", false);
  } else {
    if (pending) pending.hidden = false;
    if (mount) mount.hidden = true;
    setCameraBadge(
      camera.state === "pending" ? "Pending" : "Unavailable", true
    );
  }
}

function renderWebRTCCamera(id, entry, phase) {
  const panel = $(`webrtc-panel-${id}`);
  if (!panel) return;
  const browser = (entry && entry.browser) || {};
  const gateway = (entry && entry.gateway) || {};
  const gatewayStats = gateway.stats || {};
  const decoded = Number(browser.framesDecoded || 0);
  const live = Boolean(entry && entry.live && decoded > 0);
  const state = $(`webrtc-state-${id}`);
  const stateText = live ? "live"
    : (entry && entry.error ? "error"
      : ((entry && entry.connectionState) || "waiting"));

  setText(`webrtc-state-${id}`, stateText);
  if (state) {
    state.className = `webrtc-state ${live ? "state-connected"
      : (entry && entry.error ? "state-disconnected" : "state-stale")}`;
  }

  const pt = gatewayStats.negotiated_pt === undefined
    || gatewayStats.negotiated_pt === null ? "\u2014" : gatewayStats.negotiated_pt;
  const lines = [
    `browser \u00b7 ${decoded} frames \u00b7 ${browser.packetsReceived || 0} packets`,
    `gateway \u00b7 ${gateway.health || "unknown"} \u00b7 `
      + `${gatewayStats.ingress_packets || 0} packets \u00b7 PT ${pt}`,
  ];
  if (entry && entry.error) {
    lines.push(entry.error);
  } else if (!entry || !entry.answerApplied) {
    lines.push("waiting for gateway answer");
  } else if (!live) {
    lines.push("answer applied; waiting for RTP");
  }
  setText(`webrtc-detail-${id}`, lines.join("\n"));

  const retry = $(`webrtc-retry-${id}`);
  if (retry) retry.disabled = phase !== "session-active";
}

function renderWebRTC(snap) {
  webrtcSummary = snap || { phase: "starting", cameras: {} };
  webrtcVisible = !WEBRTC_INACTIVE_PHASES.includes(webrtcSummary.phase);
  for (const id of WEBRTC_CAMERAS) {
    renderWebRTCCamera(
      id,
      ((webrtcSummary.cameras || {})[id] || {}),
      webrtcSummary.phase
    );
  }
  renderCamera(lastSnapshot || {});
}

window.__YAM_WEBRTC__ = {
  snapshot() {
    return webrtcController
      ? webrtcController.snapshot()
      : { phase: "starting", cameras: {} };
  },
  retry(id) {
    if (!webrtcController) return Promise.reject(new Error("client not ready"));
    // Best effort for the selected camera only; sender restart is the
    // verified independent recovery path.
    return webrtcController.retry(id);
  },
  restartAll() {
    if (!webrtcController) return Promise.reject(new Error("client not ready"));
    return webrtcController.restartAll();
  },
};
window.__YAM_WEBRTC_READY__ = false;

function startWebRTC() {
  if (webrtcController || !window.YAMWebRTC) return;
  webrtcController = window.YAMWebRTC.createController({
    cameras: WEBRTC_CAMERAS,
    getVideo: (id) => $(`webrtc-video-${id}`),
    onChange: renderWebRTC,
  });

  for (const id of WEBRTC_CAMERAS) {
    const button = $(`webrtc-retry-${id}`);
    if (button) {
      button.addEventListener("click", () => {
        window.__YAM_WEBRTC__.retry(id).catch(() => {
          renderWebRTC(webrtcController.snapshot());
        });
      });
    }
  }

  renderWebRTC(webrtcController.snapshot());
  window.__YAM_WEBRTC_READY__ = true;
  webrtcController.start().catch(() => {
    renderWebRTC(webrtcController.snapshot());
  });
}

/* ------------------------------------------------------------ supervision */

const SUPERVISION_DIMENSIONS = ["health", "readiness", "safety"];
const SESSION_PILL_CLASS = {
  idle: "sup-unknown",
  active: "sup-ok",
  paused: "sup-warn",
  ended: "sup-neutral",
};

function supervisionTone(dimension, state) {
  const tones = {
    health: { healthy: "ok", degraded: "warn", disconnected: "bad" },
    readiness: { ready: "ok", not_ready: "warn", initializing: "unknown" },
    // Unknown must never adopt the healthy colour.
    safety: { normal: "ok", warning: "warn", fault: "bad", unknown: "unknown" },
  };
  return (tones[dimension] || {})[state] || "unknown";
}

let session = null;
let drawerSignature = "";
let eventFilter = "all";

function renderSupervision(data) {
  const supervision = data.supervision;
  if (!supervision) {
    for (const dimension of SUPERVISION_DIMENSIONS) {
      setSupervisionPill(dimension, "Unavailable", "unknown", []);
    }
    return;
  }

  for (const dimension of SUPERVISION_DIMENSIONS) {
    const payload = supervision[dimension] || {};
    const reasons = payload.reasons || [];
    setSupervisionPill(
      dimension,
      payload.label || "Unknown",
      supervisionTone(dimension, payload.state),
      reasons
    );
  }

  if (session && supervision.session) {
    if (supervision.session.transitions) {
      session.setTransitions(supervision.session.transitions);
    }
    if (Number.isFinite(supervision.session.inactivity_s) && session.setInactivity) {
      session.setInactivity(supervision.session.inactivity_s);
    }
  }

  renderDrawer(data);
}

function setSupervisionPill(dimension, label, tone, reasons) {
  const pill = $(`sup-${dimension}`);
  const stateEl = $(`sup-${dimension}-state`);
  const countEl = $(`sup-${dimension}-count`);
  if (pill) {
    pill.className = `sup-pill sup-${tone}`;
    pill.setAttribute(
      "aria-label",
      `${dimension}: ${label}. ${reasons.length} reason(s). Open supervision detail.`
    );
    pill.title = reasons.length
      ? `${dimension}: ${label} — ${reasons[0].message}`
      : `${dimension}: ${label}`;
  }
  if (stateEl) stateEl.textContent = label;
  if (countEl) {
    countEl.textContent = reasons.length ? String(reasons.length) : "";
    countEl.hidden = !reasons.length;
  }
}

function drawerDataSignature(data) {
  const supervision = data.supervision || {};
  const parts = [data.events ? data.events.length : 0, eventFilter];
  for (const dimension of SUPERVISION_DIMENSIONS) {
    const payload = supervision[dimension] || {};
    parts.push(payload.state || "");
    parts.push((payload.reasons || []).map((reason) => reason.code).join(","));
    if (dimension === "readiness") {
      parts.push((payload.prerequisites || [])
        .map((item) => `${item.id}:${item.passed ? 1 : 0}`).join(","));
      parts.push((payload.diagnostics || [])
        .map((item) => `${item.id}:${item.passed ? 1 : 0}`).join(","));
    }
  }
  if (session) {
    const snap = session.snapshot();
    // Deliberately excludes elapsed time: it changes every second and would
    // rebuild the whole drawer (losing scroll position) while open.
    parts.push(snap.state, snap.ended_summary ? "ended" : "");
  }
  return parts.join("|");
}

function renderDrawer(data) {
  const signature = drawerDataSignature(data);
  if (signature === drawerSignature) return;
  drawerSignature = signature;

  const supervision = data.supervision || {};
  const container = $("drawer-dimensions");
  if (container) {
    container.textContent = "";
    for (const dimension of ["health", "readiness", "safety"]) {
      container.appendChild(dimensionCard(dimension, supervision[dimension]));
    }
    container.appendChild(sessionCard());
  }

  renderPrerequisites(supervision.readiness || {});
  renderSafetyDetail(supervision.safety || {});
  renderSessionDetail();
  renderFullEvents(data);
}

function dimensionCard(dimension, payload) {
  const card = document.createElement("article");
  card.className = "dim-card";

  const head = document.createElement("div");
  head.className = "dim-head";
  const name = document.createElement("span");
  name.className = "dim-name";
  name.textContent = dimension;
  const value = document.createElement("span");
  value.className = `dim-value tone-${supervisionTone(dimension, (payload || {}).state)}`;
  value.textContent = (payload || {}).label || "Unavailable";
  head.appendChild(name);
  head.appendChild(value);
  card.appendChild(head);

  const meta = document.createElement("p");
  meta.className = "dim-meta";
  meta.textContent = payload
    ? `for ${fmtDuration(payload.since_uptime_s)}`
    : "waiting for dashboard data";
  card.appendChild(meta);

  const reasons = (payload || {}).reasons || [];
  if (!reasons.length) {
    const empty = document.createElement("p");
    empty.className = "dim-empty";
    empty.textContent = payload ? "No active reasons." : "No data.";
    card.appendChild(empty);
    return card;
  }

  const list = document.createElement("ul");
  list.className = "reason-list";
  for (const reason of reasons) {
    const item = document.createElement("li");
    const code = document.createElement("code");
    code.textContent = reason.code;
    const text = document.createElement("span");
    text.textContent = reason.message;
    item.appendChild(code);
    item.appendChild(text);
    list.appendChild(item);
  }
  card.appendChild(list);
  return card;
}

function sessionCard() {
  const card = document.createElement("article");
  card.className = "dim-card";
  const head = document.createElement("div");
  head.className = "dim-head";
  const name = document.createElement("span");
  name.className = "dim-name";
  name.textContent = "session";
  const value = document.createElement("span");
  const snap = session ? session.snapshot() : { state: "idle", label: "Idle" };
  value.className = `dim-value tone-${SESSION_PILL_CLASS[snap.state].replace("sup-", "")}`;
  value.textContent = snap.label;
  head.appendChild(name);
  head.appendChild(value);
  card.appendChild(head);
  const meta = document.createElement("p");
  meta.className = "dim-meta";
  meta.textContent = snap.state === "idle"
    ? "not started"
    : `elapsed ${snap.elapsed_text} · console only`;
  card.appendChild(meta);
  return card;
}

function renderPrerequisites(readiness) {
  const body = $("drawer-prereq-table")
    ? $("drawer-prereq-table").querySelector("tbody") : null;
  if (!body) return;
  body.textContent = "";
  const prerequisites = readiness.prerequisites || [];
  if (!prerequisites.length) {
    const tr = document.createElement("tr");
    const td = document.createElement("td");
    td.colSpan = 4;
    td.className = "val-muted";
    td.textContent = "Waiting for supervision data.";
    tr.appendChild(td);
    body.appendChild(tr);
  }
  for (const item of prerequisites) {
    const tr = document.createElement("tr");
    const name = document.createElement("td");
    name.className = "joint-name";
    name.textContent = item.description || item.id;
    const source = document.createElement("td");
    source.textContent = item.source || "\u2014";
    const result = document.createElement("td");
    const chip = document.createElement("span");
    chip.className = `result-chip ${item.passed ? "result-pass" : "result-fail"}`;
    chip.textContent = item.passed ? "PASS" : "FAIL";
    result.appendChild(chip);
    const evidence = document.createElement("td");
    evidence.className = "evidence";
    evidence.textContent = item.evidence || "\u2014";
    tr.appendChild(name);
    tr.appendChild(source);
    tr.appendChild(result);
    tr.appendChild(evidence);
    body.appendChild(tr);
  }

  const note = $("drawer-readiness-note");
  if (note) note.textContent = readiness.note || "";
  const diagnostics = $("drawer-diagnostics");
  if (diagnostics) {
    diagnostics.textContent = "";
    for (const item of readiness.diagnostics || []) {
      const row = document.createElement("div");
      row.className = "diagnostic-row";
      const label = document.createElement("span");
      label.textContent = `${item.description}: `;
      const value = document.createElement("span");
      value.className = item.passed ? "val-ok" : "val-warn";
      value.textContent = `${item.evidence} ${item.passed ? "(within)" : "(outside)"}`;
      const tag = document.createElement("span");
      tag.className = "tag";
      tag.textContent = "DIAGNOSTIC";
      row.appendChild(label);
      row.appendChild(tag);
      row.appendChild(value);
      diagnostics.appendChild(row);
    }
  }
}

function renderSafetyDetail(safety) {
  const explanation = $("drawer-safety-explanation");
  const required = $("drawer-safety-required");
  if (explanation) {
    explanation.textContent = safety.explanation
      || "Depends on external safety inputs; this console does not certify safety.";
  }
  if (required) {
    required.textContent = safety.required_input || "";
  }
}

function renderSessionDetail() {
  const container = $("drawer-session");
  if (!container) return;
  const snap = session ? session.snapshot() : null;
  if (!snap) {
    container.textContent = "Session controls unavailable.";
    return;
  }
  const lines = [`State: ${snap.label}`, `Elapsed: ${snap.elapsed_text}`];
  if (snap.pause_reason) lines.push(`Pause reason: ${snap.pause_reason}`);
  if (snap.state === "ended" && snap.ended_summary) {
    lines.push(
      `Last session: ${snap.ended_summary.duration_text}, `
      + `${snap.ended_summary.transitions} transition(s), `
      + `${snap.ended_summary.events_seen} event(s) seen`
    );
  }
  container.textContent = lines.join(" \u00b7 ");
}

/* ----------------------------------------------------------------- events */

let lastPreviewSignature = "";
let lastFullSignature = "";

function renderEvents(data) {
  const events = data.events || [];
  const newest = events.length ? events[events.length - 1].id : 0;
  const signature = `${events.length}:${newest}`;

  if (session) session.setEventCount(events.length);

  if (signature !== lastPreviewSignature) {
    lastPreviewSignature = signature;
    const list = $("event-log");
    if (list) {
      list.textContent = "";
      if (!events.length) {
        const empty = document.createElement("li");
        empty.className = "event-empty";
        empty.textContent = "No events recorded yet. State changes appear here.";
        list.appendChild(empty);
      } else {
        for (const event of events.slice(-3).reverse()) {
          list.appendChild(eventItem(event));
        }
      }
    }
    const count = $("event-count");
    if (count) {
      count.textContent = events.length
        ? `${Math.min(3, events.length)} of ${events.length} event${events.length === 1 ? "" : "s"}`
        : "no events";
    }
  }

  renderFullEvents(data);
}

function fullEventsSignature(data) {
  const events = data.events || [];
  const newest = events.length ? events[events.length - 1].id : 0;
  const sessionLength = session
    ? (session.snapshot().history || []).length
    : 0;
  return `${events.length}:${newest}:${eventFilter}:${sessionLength}`;
}

function renderFullEvents(data) {
  const list = $("drawer-event-log");
  if (!list) return;
  const signature = fullEventsSignature(data);
  if (signature === lastFullSignature) return;
  lastFullSignature = signature;
  const events = filteredEvents(data.events || []);
  list.textContent = "";
  if (!events.length) {
    const empty = document.createElement("li");
    empty.className = "event-empty";
    empty.textContent = eventFilter === "all"
      ? "No events recorded yet."
      : `No ${eventFilter} events recorded yet.`;
    list.appendChild(empty);
  } else {
    for (const event of events.slice().reverse()) {
      list.appendChild(eventItem(event));
    }
  }
  const count = $("drawer-event-count");
  if (count) {
    count.textContent = `${events.length} event${events.length === 1 ? "" : "s"}`
      + (eventFilter === "all" ? "" : ` matching ${eventFilter}`);
  }
}

function filteredEvents(events) {
  if (eventFilter === "all") return events;
  if (eventFilter === "session") {
    if (!session) return [];
    return (session.snapshot().history || []).map((entry) => ({
      id: `session-${entry.at}`,
      time: new Date(entry.at).toISOString(),
      level: "info",
      source: "session",
      message: `${entry.from} → ${entry.to} (${entry.action})`
        + (entry.reason ? ` — ${entry.reason}` : ""),
    }));
  }
  return events.filter((event) => {
    const source = String(event.source || "");
    if (eventFilter === "health") {
      return source === "supervision.health"
        || source === "leader" || source === "follower" || source === "tracking";
    }
    return source === `supervision.${eventFilter}`;
  });
}

function eventItem(event) {
  const item = document.createElement("li");

  const time = document.createElement("span");
  time.className = "event-time";
  time.textContent = shortTime(event.time);
  item.appendChild(time);

  const level = document.createElement("span");
  level.className = `event-level event-level-${event.level}`;
  level.textContent = event.level;
  item.appendChild(level);

  const source = document.createElement("span");
  source.className = "event-source";
  source.textContent = event.source;
  item.appendChild(source);

  const message = document.createElement("span");
  message.className = "event-message";
  message.textContent = event.message;
  item.appendChild(message);

  return item;
}

function setEventFilter(filter) {
  eventFilter = filter;
  for (const chip of document.querySelectorAll("#event-filters .chip")) {
    chip.classList.toggle("chip-active", chip.dataset.filter === filter);
  }
  drawerSignature = "";
  lastFullSignature = "";
  renderFullEvents(lastSnapshot || {});
}

/* ----------------------------------------------------------------- drawer */

let drawerReturnFocus = null;

function openDrawer(sectionId) {
  const drawer = $("supervision-drawer");
  const backdrop = $("drawer-backdrop");
  if (!drawer || !backdrop) return;
  drawerReturnFocus = document.activeElement;
  drawer.hidden = false;
  backdrop.hidden = false;
  document.body.classList.add("drawer-open");
  const close = $("drawer-close");
  if (close) close.focus();
  if (sectionId) {
    const section = $(sectionId);
    if (section) section.scrollIntoView({ block: "start" });
  }
}

function closeDrawer() {
  const drawer = $("supervision-drawer");
  const backdrop = $("drawer-backdrop");
  if (!drawer || !backdrop) return;
  drawer.hidden = true;
  backdrop.hidden = true;
  document.body.classList.remove("drawer-open");
  if (drawerReturnFocus && typeof drawerReturnFocus.focus === "function") {
    drawerReturnFocus.focus();
  }
  drawerReturnFocus = null;
}

function trapFocus(event) {
  const drawer = $("supervision-drawer");
  if (!drawer || drawer.hidden || event.key !== "Tab") return;
  const focusable = drawer.querySelectorAll(
    "button, [href], input, select, textarea, [tabindex]:not([tabindex='-1'])"
  );
  if (!focusable.length) return;
  const first = focusable[0];
  const last = focusable[focusable.length - 1];
  if (event.shiftKey && document.activeElement === first) {
    event.preventDefault();
    last.focus();
  } else if (!event.shiftKey && document.activeElement === last) {
    event.preventDefault();
    first.focus();
  }
}

function bindDrawer() {
  for (const pill of document.querySelectorAll(".sup-pill")) {
    pill.addEventListener("click", () => openDrawer());
  }
  const openLog = $("open-log");
  if (openLog) {
    openLog.addEventListener("click", () => {
      openDrawer("drawer-events-section");
      const sessionChip = document.querySelector("#event-filters .chip[data-filter='all']");
      if (sessionChip) sessionChip.focus();
    });
  }
  const close = $("drawer-close");
  if (close) close.addEventListener("click", closeDrawer);
  const backdrop = $("drawer-backdrop");
  if (backdrop) backdrop.addEventListener("click", closeDrawer);
  const filters = $("event-filters");
  if (filters) {
    filters.addEventListener("click", (event) => {
      const chip = event.target.closest(".chip");
      if (chip) setEventFilter(chip.dataset.filter);
    });
  }
  document.addEventListener("keydown", (event) => {
    if (event.key === "Escape") closeDrawer();
    trapFocus(event);
  });
}

/* -------------------------------------------------------------- diagnostics */

const DIAG_ENABLED = new URLSearchParams(window.location.search).get("diag") === "1";

function updateDiag() {
  const element = $("layout-diag");
  if (!element) return;
  const root = document.documentElement;
  const body = document.body;
  const main = document.querySelector("main");
  const innerWidth = window.innerWidth;
  const innerHeight = window.innerHeight;

  // Body/main can clip overflow (single-screen profile), so measure the
  // element scroll extents too rather than only the document.
  const verticalOverflow = Math.max(
    0,
    root.scrollHeight - innerHeight,
    body.scrollHeight - innerHeight,
    main ? main.scrollHeight - main.clientHeight : 0
  );
  const horizontalOverflow = Math.max(
    0,
    root.scrollWidth - innerWidth,
    body.scrollWidth - innerWidth,
    main ? main.scrollWidth - main.clientWidth : 0
  );
  const profile = (innerWidth < 1200 || innerHeight < 700)
    ? "stacked"
    : (innerHeight < 900 ? "compact" : "full");

  element.dataset.profile = profile;
  element.dataset.innerWidth = String(innerWidth);
  element.dataset.innerHeight = String(innerHeight);
  element.dataset.scrollWidth = String(Math.max(root.scrollWidth, body.scrollWidth));
  element.dataset.scrollHeight = String(Math.max(root.scrollHeight, body.scrollHeight));
  element.dataset.overflow = String(verticalOverflow);
  element.dataset.horizontalOverflow = String(horizontalOverflow);

  if (DIAG_ENABLED) {
    element.hidden = false;
    element.textContent = `${profile} ${innerWidth}x${innerHeight} `
      + `vertical_overflow=${verticalOverflow} `
      + `horizontal_overflow=${horizontalOverflow}`;
  }
}

/* ------------------------------------------------------------- transport */

function setTransport(online) {
  const pill = $("link-pill");
  if (pill) pill.className = online ? "pill pill-connected" : "pill pill-disconnected";
  const text = $("link-text");
  if (text) text.textContent = online ? "Console online" : "Dashboard offline";
}

function renderOffline() {
  setTransport(false);
  for (const dimension of SUPERVISION_DIMENSIONS) {
    setSupervisionPill(dimension, "Offline", "unknown", []);
  }
  renderRobot("leader", null, "#38bdf8");
  renderRobot("follower", null, "#fbbf24");
}

/* ------------------------------------------------------------- poll loop */

const DEFAULT_INTERVAL_MS = 100;
let pollTimer = null;
let lastSnapshot = null;

function render(data) {
  if (!data || data.ready === false) return;
  lastSnapshot = data;
  setTransport(true);
  renderSupervision(data);
  renderJoints(data);
  renderComms(data);
  renderCamera(data);
  renderEvents(data);
  renderRobot("leader", (data.streams || {}).leader, "#38bdf8");
  renderRobot("follower", (data.streams || {}).follower, "#fbbf24");
  renderRobotSummaries(data);
  updateDiag();
}

async function poll() {
  let interval = DEFAULT_INTERVAL_MS;
  let data = null;
  try {
    const response = await fetch("/api/state", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    data = await response.json();
  } catch (error) {
    if (session) session.notifyPoll(false);
    renderOffline();
    updateDiag();
    pollTimer = window.setTimeout(poll, interval);
    return;
  }

  if (session) session.notifyPoll(true);
  try {
    render(data);
  } catch (error) {
    // A rendering bug must not look like a transport failure.
    console.error("render failed", error);
    updateDiag();
  }
  const configured = Number((data.config || {}).refresh_ms);
  if (Number.isFinite(configured)) {
    interval = Math.min(2000, Math.max(50, configured));
  }
  pollTimer = window.setTimeout(poll, interval);
}

function start() {
  session = YAMSession.createSession({ inactivityS: 30 });
  YAMSession.mount({
    pill: $("sup-session"),
    stripState: $("sup-session-state"),
    timer: $("session-timer"),
    start: $("session-start"),
    pause: $("session-pause"),
    resume: $("session-resume"),
    end: $("session-end"),
    summary: $("drawer-session"),
  }, session);
  bindDrawer();
  startWebRTC();
  const params = new URLSearchParams(window.location.search);
  if (params.get("selftest") === "1") YAMSession.runSelfTest();
  // Diagnostic hook: opens the detail drawer so its DOM can be verified by a
  // headless browser without simulating a click.
  if (params.get("drawer") === "1") openDrawer("drawer-events-section");
  renderRobot("leader", null, "#38bdf8");
  renderRobot("follower", null, "#fbbf24");
  updateDiag();
  window.addEventListener("resize", updateDiag);
  window.setTimeout(updateDiag, 0);
  poll();
}

document.addEventListener("DOMContentLoaded", start);
