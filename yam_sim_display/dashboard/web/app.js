/* YAM Operator Console — Milestone 1 (read-only observer).
 *
 * Presentation layer only. It consumes the JSON snapshot produced by the ROS
 * node and never speaks to ROS directly.
 *
 * The forward-kinematics + projection code below is derived from the real
 * URDF joint origins/axes in yam_description/urdf/yam_joints.xacro, so the
 * drawing is driven by live joint values rather than being decorative.
 */

"use strict";

/* ------------------------------------------------------------------ utils */

const SVG_NS = "http://www.w3.org/2000/svg";

const $ = (id) => document.getElementById(id);

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

/* --------------------------------------------------------------- overview */

function renderOverview(data) {
  const system = data.system || {};
  const streams = data.streams || {};

  const overall = $("stat-system");
  overall.textContent = system.state_label || "\u2014";
  overall.className = `stat-value ${stateClass(system.state)}`;
  $("stat-system-note").textContent = system.label || "\u2014";
  $("overview-detail").textContent = system.detail || "\u2014";

  for (const role of ["leader", "follower"]) {
    const stream = streams[role] || {};
    const value = $(`stat-${role}`);
    value.textContent = stream.state_label || "Disconnected";
    value.className = `stat-value ${stateClass(stream.state)}`;
    $(`stat-${role}-note`).textContent = stream.count
      ? `${fmtRate(stream.rate_hz)} \u00b7 age ${fmtAge(stream.age_ms)}`
      : "no messages received";
  }

  const tracking = data.tracking || {};
  const errorValue = $("stat-error");
  if (tracking.available) {
    errorValue.textContent = `${fmt(tracking.max_abs_error_rad * 1000, 1)} mrad`;
    const warnRad = (data.config || {}).max_error_warn_rad || 0.15;
    errorValue.className = `stat-value ${tracking.max_abs_error_rad >= warnRad ? "state-warning" : "state-connected"}`;
    $("stat-error-note").textContent =
      `RMS ${fmt(tracking.rms_error_rad * 1000, 1)} mrad \u00b7 worst ${tracking.max_abs_error_joint || "\u2014"}`;
  } else {
    errorValue.textContent = "\u2014";
    errorValue.className = "stat-value val-muted";
    $("stat-error-note").textContent = "needs both streams";
  }

  const pill = $("link-pill");
  pill.className = pillClass(system.state);
  $("link-text").textContent = system.state_label || "Unknown";
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
    $("joint-sub").textContent = "\u2014";
    $("joint-note").textContent = "Waiting for /leader/joint_states and /follower/joint_states.";
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
    note.innerHTML = `<strong>Limitation:</strong> ${tracking.note}`;
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
  $("latency-reason").textContent = (data.latency || {}).reason || "";

  const foot = $("foot-poll");
  if (foot) {
    foot.textContent = `polling ${config.refresh_ms || "\u2014"} ms \u00b7 `
      + `stale ${config.stale_ms || "\u2014"} ms \u00b7 schema ${data.schema || 1}`;
  }
}

/* ---------------------------------------------------------------- camera */

function renderCamera(data) {
  const camera = data.camera || {};
  const pending = $("camera-pending");
  const mount = $("camera-mount");

  $("camera-title").textContent = camera.title || "Camera feed pending";
  $("camera-reason").textContent = camera.reason || "";

  if (camera.stream_url) {
    pending.hidden = true;
    mount.hidden = false;
    if (!mount.querySelector("img")) {
      const img = document.createElement("img");
      img.alt = "Live camera feed";
      img.src = camera.stream_url;
      mount.appendChild(img);
    }
    $("camera-badge").textContent = "Live";
    $("camera-badge").className = "badge";
  } else {
    pending.hidden = false;
    mount.hidden = true;
    $("camera-badge").textContent = camera.state === "pending" ? "Pending" : "Unavailable";
    $("camera-badge").className = "badge badge-warn";
  }
}

/* ---------------------------------------------------------------- events */

let lastEventSignature = "";

function renderEvents(data) {
  const events = data.events || [];
  const list = $("event-log");
  const newest = events.length ? events[events.length - 1].id : 0;
  const signature = `${events.length}:${newest}`;
  if (signature === lastEventSignature) return;
  lastEventSignature = signature;

  list.textContent = "";
  $("event-count").textContent = events.length
    ? `${events.length} event${events.length === 1 ? "" : "s"}`
    : "no events";

  if (!events.length) {
    const empty = document.createElement("li");
    empty.className = "event-empty";
    empty.textContent = "No events recorded yet. State changes appear here.";
    list.appendChild(empty);
    return;
  }

  for (let i = events.length - 1; i >= 0; i -= 1) {
    const event = events[i];
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

    list.appendChild(item);
  }
}

/* ------------------------------------------------------------- poll loop */

const DEFAULT_INTERVAL_MS = 100;
let pollTimer = null;

function render(data) {
  if (!data || data.ready === false) return;
  renderOverview(data);
  renderJoints(data);
  renderComms(data);
  renderCamera(data);
  renderEvents(data);
  renderRobot("leader", (data.streams || {}).leader, "#38bdf8");
  renderRobot("follower", (data.streams || {}).follower, "#fbbf24");

  const tracking = data.tracking || {};
  const leaderFoot = $("viz-foot-leader");
  const followerFoot = $("viz-foot-follower");
  const leader = (data.streams || {}).leader || {};
  const follower = (data.streams || {}).follower || {};

  if (leaderFoot) {
    leaderFoot.textContent = leader.count
      ? `${leader.joint_order ? leader.joint_order.length : 0} joints \u00b7 `
        + `${fmtRate(leader.rate_hz)} \u00b7 age ${fmtAge(leader.age_ms)}`
      : "awaiting telemetry";
  }
  if (followerFoot) {
    followerFoot.textContent = tracking.available
      ? `max |err| ${fmt(tracking.max_abs_error_rad * 1000, 1)} mrad \u00b7 `
        + `RMS ${fmt(tracking.rms_error_rad * 1000, 1)} mrad`
      : follower.count ? `${fmtRate(follower.rate_hz)} \u00b7 tracking unavailable`
        : "awaiting telemetry";
  }
}

async function poll() {
  let interval = DEFAULT_INTERVAL_MS;
  try {
    const response = await fetch("/api/state", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const data = await response.json();
    render(data);
    const configured = Number((data.config || {}).refresh_ms);
    if (Number.isFinite(configured)) {
      interval = Math.min(2000, Math.max(50, configured));
    }
  } catch (error) {
    const pill = $("link-pill");
    pill.className = "pill pill-disconnected";
    $("link-text").textContent = "Dashboard offline";
    $("stat-system").textContent = "Dashboard offline";
    $("stat-system").className = "stat-value state-disconnected";
    $("stat-system-note").textContent = "cannot reach /api/state";
    renderRobot("leader", null, "#38bdf8");
    renderRobot("follower", null, "#fbbf24");
  } finally {
    pollTimer = window.setTimeout(poll, interval);
  }
}

function start() {
  const foot = $("foot-source");
  if (foot) foot.textContent = "yam_sim_display dashboard \u00b7 read-only observer";
  renderRobot("leader", null, "#38bdf8");
  renderRobot("follower", null, "#fbbf24");
  poll();
}

document.addEventListener("DOMContentLoaded", start);
