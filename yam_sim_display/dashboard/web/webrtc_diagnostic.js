"use strict";

/* WebRTC diagnostic page wrapper.
 *
 * All media and signaling live in the shared YAMWebRTC client. This file only
 * renders that client's state into the diagnostic page and exposes the CDP
 * hooks used by the E2E harness.
 */

const CAMERAS = ["d405", "d455f"];

let controller = null;

function $(id) { return document.getElementById(id); }

function setPhase(text, kind) {
  const pill = $("session-state");
  if (!pill) return;
  pill.textContent = text;
  pill.className = "pill" + (kind ? " " + kind : "");
}

function setCameraPill(id, text, kind) {
  const pill = $("state-" + id);
  if (!pill) return;
  pill.textContent = text;
  pill.className = "pill" + (kind ? " " + kind : "");
}

function phaseTone(phase) {
  if (phase === "session-active") return "ok";
  if (phase === "disabled") return "";
  if (phase === "stopped") return "";
  if (phase === "checking-signaling" || phase === "creating-session") return "";
  return "bad";
}

function renderMetrics(id, entry) {
  const element = $("metrics-" + id);
  if (!element) return;
  const browser = entry.browser || {};
  const gateway = entry.gateway || {};
  const stats = gateway.stats || {};
  const rows = [
    ["connection", entry.connectionState],
    ["ICE", entry.iceConnectionState],
    ["video readyState", String(entry.readyState)],
    ["browser framesDecoded", String(browser.framesDecoded || 0)],
    ["estimated camera-to-screen latest ms", browser.timingAvailable
      ? Number(browser.cameraToScreenLatestMs || 0).toFixed(1) : "unavailable"],
    ["estimated camera-to-screen median ms", browser.timingAvailable
      ? Number(browser.cameraToScreenMedianMs || 0).toFixed(1) : "unavailable"],
    ["estimated camera-to-screen P95 ms", browser.timingAvailable
      ? Number(browser.cameraToScreenP95Ms || 0).toFixed(1) : "unavailable"],
    ["timing samples", String(browser.cameraToScreenSamples || 0)],
    ["capture-to-receive ms", browser.captureToReceiveMs === null
      || browser.captureToReceiveMs === undefined
      ? "-" : Number(browser.captureToReceiveMs).toFixed(1)],
    ["receive-to-display ms", browser.receiveToDisplayMs === null
      || browser.receiveToDisplayMs === undefined
      ? "-" : Number(browser.receiveToDisplayMs).toFixed(1)],
    ["browser FPS", Number(browser.fps || 0).toFixed(1)],
    ["browser bitrate Mbps", (Number(browser.bitrateKbps || 0) / 1000).toFixed(2)],
    ["browser frames dropped", String(browser.framesDropped || 0)],
    ["browser freezes", String(browser.freezeCount || 0)],
    ["transport jitter ms", Number(browser.jitterMs || 0).toFixed(1)],
    ["transport RTT ms", browser.roundTripTimeMs === null
      || browser.roundTripTimeMs === undefined
      ? "-" : Number(browser.roundTripTimeMs).toFixed(1)],
    ["transport packets lost", String(browser.packetsLost || 0)],
    ["browser packetsReceived", String(browser.packetsReceived || 0)],
    ["gateway health", gateway.health || "unknown"],
    ["gateway ingress packets", String(stats.ingress_packets || 0)],
    ["gateway negotiated PT", String(stats.negotiated_pt || "-")],
    ["error", entry.error || "-"],
  ];
  element.textContent = "";
  for (const [key, value] of rows) {
    const dt = document.createElement("dt");
    const dd = document.createElement("dd");
    dt.textContent = key;
    dd.textContent = value;
    element.appendChild(dt);
    element.appendChild(dd);
  }
}

function render(snap) {
  if (!snap) return;
  setPhase(snap.phase, phaseTone(snap.phase));

  for (const id of CAMERAS) {
    const entry = (snap.cameras || {})[id] || {};
    const overlay = $("overlay-" + id);
    if (overlay) {
      if (entry.error) {
        overlay.textContent = entry.error;
      } else if (entry.live) {
        overlay.textContent = `decoding (${entry.browser.framesDecoded || 0} frames)`;
      } else if (entry.answerApplied) {
        overlay.textContent = "answer applied; waiting for RTP";
      } else if (snap.phase === "disabled") {
        overlay.textContent = "WebRTC signaling disabled";
      } else {
        overlay.textContent = "waiting for gateway answer";
      }
    }
    setCameraPill(
      id,
      entry.live ? "live" : (entry.error ? "error" : entry.connectionState),
      entry.live ? "ok" : (entry.error ? "bad" : "")
    );
    renderMetrics(id, entry);
  }
}

function snapshot() {
  return controller ? controller.snapshot() : { phase: "starting", cameras: {} };
}

async function retry(id) {
  if (!controller) throw new Error("client not ready");
  return await controller.retry(id);
}

async function restartAll() {
  if (!controller) throw new Error("client not ready");
  return await controller.restartAll();
}

window.__webrtcDiagnostic = {
  snapshot,
  retry,
  restartAll,
};

window.addEventListener("load", () => {
  controller = window.YAMWebRTC.createController({
    cameras: CAMERAS,
    getVideo: (id) => $("video-" + id),
    onChange: render,
  });
  controller.start();
  window.__webrtcDiagnosticReady = true;
  for (const id of CAMERAS) {
    const button = $("retry-" + id);
    if (button) {
      button.addEventListener("click", () => {
        retry(id).catch((error) => {
          render(snapshot());
          console.warn("retry failed for " + id + ": " + error.message);
        });
      });
    }
  }
});
