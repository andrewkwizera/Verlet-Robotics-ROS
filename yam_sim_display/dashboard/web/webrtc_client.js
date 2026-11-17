"use strict";

/* Shared browser-side WebRTC client for the operator console and diagnostic
 * page.
 *
 * One controller owns one signaling session and one recvonly
 * RTCPeerConnection per camera. Cameras are independent: a failure or retry in
 * one mailbox never closes the other peer connection. Session ids are opaque
 * secrets held only in this closure; they are never written to the DOM, logs,
 * snapshots or errors.
 */

(function (global) {
  const CAMERAS = ["d405", "d455f"];
  const DEFAULT_KEEPALIVE_MS = 60000;
  const DEFAULT_POLL_TIMEOUT_S = 10;
  const ICE_GATHER_TIMEOUT_MS = 6000;

  function signalingPath(route, params) {
    const query = new URLSearchParams();
    for (const [key, value] of Object.entries(params || {})) {
      query.set(key, String(value));
    }
    const suffix = query.toString();
    return "/api/webrtc/" + route + (suffix ? "?" + suffix : "");
  }

  async function requestJson(path, options) {
    const response = await fetch(path, options || {});
    if (!response.ok) {
      const error = new Error(
        response.status === 404 ? "session expired" : "HTTP " + response.status
      );
      error.status = response.status;
      error.sessionExpired = response.status === 404;
      throw error;
    }
    if (response.status === 204) return null;
    return await response.json();
  }

  async function postJson(path, body) {
    return await requestJson(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
  }

  function h264Preferences() {
    if (!global.RTCRtpReceiver || !global.RTCRtpReceiver.getCapabilities) {
      return [];
    }
    const capabilities = global.RTCRtpReceiver.getCapabilities("video");
    if (!capabilities || !capabilities.codecs) return [];
    const h264 = capabilities.codecs.filter(
      (codec) => codec.mimeType && codec.mimeType.toLowerCase() === "video/h264"
    );
    const others = capabilities.codecs.filter((codec) => !h264.includes(codec));
    return h264.concat(others);
  }

  async function waitForIceGathering(pc) {
    if (pc.iceGatheringState === "complete") return;
    await new Promise((resolve) => {
      let done = false;
      const finish = () => {
        if (done) return;
        done = true;
        pc.removeEventListener("icegatheringstatechange", check);
        resolve();
      };
      const check = () => {
        if (pc.iceGatheringState === "complete") finish();
      };
      pc.addEventListener("icegatheringstatechange", check);
      global.setTimeout(finish, ICE_GATHER_TIMEOUT_MS);
    });
  }

  function createController(options) {
    const opts = options || {};
    const cameras = Array.isArray(opts.cameras) && opts.cameras.length
      ? opts.cameras.slice() : CAMERAS.slice();
    const keepaliveMs = Number(opts.keepaliveMs) > 0
      ? Number(opts.keepaliveMs) : DEFAULT_KEEPALIVE_MS;
    const pollTimeoutS = Number(opts.pollTimeoutS) > 0
      ? Number(opts.pollTimeoutS) : DEFAULT_POLL_TIMEOUT_S;
    const onChange = typeof opts.onChange === "function" ? opts.onChange : null;
    const getVideo = typeof opts.getVideo === "function"
      ? opts.getVideo
      : (id) => (opts.videos || {})[id] || null;

    const state = {
      sessionId: null,
      phase: "starting",
      restarting: false,
      stopped: false,
      keepaliveTimer: null,
      cameras: {},
    };

    for (const id of cameras) {
      state.cameras[id] = {
        id,
        pc: null,
        generation: 0,
        live: false,
        error: null,
        connectionState: "new",
        iceConnectionState: "new",
        readyState: 0,
        videoFrames: 0,
        browserStats: {},
        gatewayHealth: "unknown",
        gatewayStats: {},
        answerApplied: false,
        statsTimer: null,
      };
    }

    function notify() {
      if (!onChange) return;
      try {
        onChange(snapshot());
      } catch (error) {
        /* A broken renderer must not break media or signaling. */
      }
    }

    function setPhase(text) {
      if (state.phase === text) return;
      state.phase = text;
      notify();
    }

    function isCurrent(id, sessionId, pc, generation) {
      const entry = state.cameras[id];
      return Boolean(
        entry
        && !state.stopped
        && state.sessionId === sessionId
        && entry.pc === pc
        && entry.generation === generation
      );
    }

    function resetCamera(id) {
      const entry = state.cameras[id];
      if (!entry) return;
      entry.generation += 1;
      if (entry.statsTimer !== null) {
        global.clearInterval(entry.statsTimer);
        entry.statsTimer = null;
      }
      if (entry.pc !== null) {
        try { entry.pc.close(); } catch (_) { /* already closed */ }
        entry.pc = null;
      }
      const video = getVideo(id);
      if (video && video.srcObject) {
        try {
          video.srcObject.getTracks().forEach((track) => track.stop());
        } catch (_) { /* best effort */ }
        video.srcObject = null;
      }
      entry.live = false;
      entry.error = null;
      entry.connectionState = "new";
      entry.iceConnectionState = "new";
      entry.readyState = 0;
      entry.videoFrames = 0;
      entry.browserStats = {};
      entry.gatewayHealth = "unknown";
      entry.gatewayStats = {};
      entry.answerApplied = false;
      notify();
    }

    function clearKeepalive() {
      if (state.keepaliveTimer !== null) {
        global.clearInterval(state.keepaliveTimer);
        state.keepaliveTimer = null;
      }
    }

    function startKeepalive() {
      clearKeepalive();
      const sessionId = state.sessionId;
      state.keepaliveTimer = global.setInterval(async () => {
        if (state.stopped || state.sessionId !== sessionId) return;
        try {
          await postJson(
            signalingPath("keepalive", { session_id: sessionId }), {}
          );
        } catch (error) {
          if (error.sessionExpired) handleSessionLoss();
        }
      }, keepaliveMs);
    }

    function handleSessionLoss() {
      if (state.restarting || state.stopped || !state.sessionId) return;
      state.restarting = true;
      state.sessionId = null;
      clearKeepalive();
      setPhase("session-expired");
      for (const id of cameras) resetCamera(id);
      global.setTimeout(() => {
        if (state.stopped) return;
        state.restarting = false;
        start(true);
      }, 1000);
    }

    async function setupCamera(id, sessionId) {
      const entry = state.cameras[id];
      if (!entry) throw new Error("unknown camera");
      const generation = entry.generation;
      const pc = new global.RTCPeerConnection({ iceServers: [] });
      entry.pc = pc;
      entry.live = false;
      entry.error = null;
      entry.connectionState = "new";
      entry.iceConnectionState = "new";
      entry.answerApplied = false;
      notify();

      const transceiver = pc.addTransceiver("video", { direction: "recvonly" });
      const preferences = h264Preferences();
      if (preferences.length && transceiver.setCodecPreferences) {
        try {
          transceiver.setCodecPreferences(preferences);
        } catch (error) {
          entry.error = "codec preference: " + error.message;
        }
      }

      pc.ontrack = (event) => {
        if (!isCurrent(id, sessionId, pc, generation)) return;
        const video = getVideo(id);
        if (video && event.streams && event.streams[0]) {
          video.srcObject = event.streams[0];
          const play = video.play();
          if (play && typeof play.catch === "function") {
            play.catch(() => { /* autoplay policy; diagnostic only */ });
          }
        }
      };
      pc.onconnectionstatechange = () => {
        if (!isCurrent(id, sessionId, pc, generation)) return;
        entry.connectionState = pc.connectionState;
        notify();
      };
      pc.oniceconnectionstatechange = () => {
        if (!isCurrent(id, sessionId, pc, generation)) return;
        entry.iceConnectionState = pc.iceConnectionState;
        notify();
      };

      const offer = await pc.createOffer();
      if (!isCurrent(id, sessionId, pc, generation)) return;
      await pc.setLocalDescription(offer);
      await waitForIceGathering(pc);
      if (!isCurrent(id, sessionId, pc, generation)) return;

      await postJson(
        signalingPath("offer", { session_id: sessionId, camera_id: id }),
        { sdp: pc.localDescription.sdp }
      );
      if (!isCurrent(id, sessionId, pc, generation)) return;
      pollAnswer(id, sessionId, pc, generation);
      pollGatewayStats(id, sessionId, pc, generation);
    }

    async function pollAnswer(id, sessionId, pc, generation) {
      let afterRev = 0;
      while (isCurrent(id, sessionId, pc, generation)) {
        try {
          const payload = await requestJson(signalingPath("answer", {
            session_id: sessionId,
            camera_id: id,
            after_rev: afterRev,
            timeout: pollTimeoutS,
          }));
          if (!payload) continue;
          afterRev = payload.revision;
          if (!payload.sdp || state.cameras[id].answerApplied) continue;
          if (!isCurrent(id, sessionId, pc, generation)) return;
          await pc.setRemoteDescription({ type: "answer", sdp: payload.sdp });
          state.cameras[id].answerApplied = true;
          startStats(id, sessionId, pc, generation);
          notify();
        } catch (error) {
          if (error.sessionExpired) { handleSessionLoss(); return; }
          if (!isCurrent(id, sessionId, pc, generation)) return;
          state.cameras[id].error = String(error.message || error);
          notify();
          await new Promise((resolve) => global.setTimeout(resolve, 1000));
        }
      }
    }

    async function pollGatewayStats(id, sessionId, pc, generation) {
      let afterRev = 0;
      while (isCurrent(id, sessionId, pc, generation)) {
        try {
          const payload = await requestJson(signalingPath("stats", {
            session_id: sessionId,
            camera_id: id,
            after_rev: afterRev,
            timeout: pollTimeoutS,
          }));
          if (payload && isCurrent(id, sessionId, pc, generation)) {
            afterRev = payload.revision;
            state.cameras[id].gatewayHealth = payload.health || "unknown";
            state.cameras[id].gatewayStats = payload.stats || {};
            notify();
          }
        } catch (error) {
          if (error.sessionExpired) { handleSessionLoss(); return; }
        }
        await new Promise((resolve) => global.setTimeout(resolve, 1000));
      }
    }

    function startStats(id, sessionId, pc, generation) {
      const entry = state.cameras[id];
      if (entry.statsTimer !== null) {
        global.clearInterval(entry.statsTimer);
        entry.statsTimer = null;
      }
      const refresh = async () => {
        if (!isCurrent(id, sessionId, pc, generation)) return;
        try {
          const report = await pc.getStats();
          let inbound = null;
          report.forEach((item) => {
            if (
              item.type === "inbound-rtp"
              && item.kind === "video"
              && !item.isRemote
            ) {
              inbound = item;
            }
          });
          const video = getVideo(id);
          const quality = video && video.getVideoPlaybackQuality
            ? video.getVideoPlaybackQuality() : null;
          entry.readyState = video ? video.readyState : 0;
          entry.videoFrames = quality ? quality.totalVideoFrames : 0;
          entry.browserStats = inbound ? {
            source: "browser",
            packetsReceived: inbound.packetsReceived || 0,
            framesDecoded: inbound.framesDecoded || 0,
            bytesReceived: inbound.bytesReceived || 0,
            frameWidth: inbound.frameWidth || 0,
            frameHeight: inbound.frameHeight || 0,
            jitter: inbound.jitter || 0,
          } : { source: "browser" };
          const decoded = Number(entry.browserStats.framesDecoded || 0);
          entry.live = decoded > 0 || entry.videoFrames > 0;
        } catch (error) {
          entry.error = String(error.message || error);
        }
        if (isCurrent(id, sessionId, pc, generation)) notify();
      };
      refresh();
      entry.statsTimer = global.setInterval(refresh, 1000);
    }

    async function start(force) {
      if (state.stopped) return false;
      if (state.restarting || state.sessionId) return false;
      if (state.phase === "disabled" && !force) return false;

      state.restarting = true;
      try {
        setPhase("checking-signaling");
        const discovery = await requestJson(signalingPath("sessions"));
        if (!discovery || !Array.isArray(discovery.sessions)) {
          throw new Error("invalid signaling discovery response");
        }

        setPhase("creating-session");
        const session = await requestJson("/api/webrtc/session", {
          method: "POST",
        });
        if (!session || !session.session_id) {
          throw new Error("session response had no id");
        }
        state.sessionId = session.session_id;
        setPhase("session-active");
        startKeepalive();
        // Camera setup is isolated: a failure in one mailbox never tears down
        // the other camera's peer connection or stats pollers.
        await Promise.all(cameras.map(async (id) => {
          try {
            await setupCamera(id, state.sessionId);
          } catch (error) {
            const entry = state.cameras[id];
            if (entry) entry.error = String(error.message || error);
            notify();
          }
        }));
      } catch (error) {
        state.sessionId = null;
        clearKeepalive();
        if (error.status === 404) {
          setPhase("disabled");
          return false;
        }
        setPhase("start-failed");
        for (const id of cameras) {
          state.cameras[id].error = String(error.message || error);
        }
        notify();
        global.setTimeout(() => {
          if (!state.stopped) {
            state.restarting = false;
            start(true);
          }
        }, 2000);
        return false;
      } finally {
        state.restarting = false;
        notify();
      }
      return true;
    }

    function retry(id) {
      if (!cameras.includes(id)) return Promise.reject(new Error("unknown camera"));
      const sessionId = state.sessionId;
      if (!sessionId) return Promise.reject(new Error("no active session"));
      const entry = state.cameras[id];
      resetCamera(id);
      entry.error = null;
      notify();
      // Best effort only: give Chrome a moment to release previous ICE
      // sockets. Sender restart remains the verified recovery path.
      return new Promise((resolve, reject) => {
        global.setTimeout(async () => {
          if (state.stopped || state.sessionId !== sessionId) {
            reject(new Error("session changed"));
            return;
          }
          try {
            await setupCamera(id, sessionId);
            resolve(true);
          } catch (error) {
            entry.error = String(error.message || error);
            notify();
            reject(error);
          }
        }, 1000);
      });
    }

    function restartAll() {
      state.sessionId = null;
      clearKeepalive();
      for (const id of cameras) resetCamera(id);
      state.phase = "starting";
      return start(true);
    }

    function stop() {
      state.stopped = true;
      state.sessionId = null;
      clearKeepalive();
      for (const id of cameras) resetCamera(id);
      setPhase("stopped");
      notify();
    }

    function snapshot() {
      const result = { phase: state.phase, cameras: {} };
      for (const id of cameras) {
        const entry = state.cameras[id];
        result.cameras[id] = {
          id,
          live: Boolean(entry.live),
          connectionState: entry.connectionState,
          iceConnectionState: entry.iceConnectionState,
          readyState: entry.readyState,
          videoFrames: entry.videoFrames,
          answerApplied: Boolean(entry.answerApplied),
          error: entry.error,
          browser: Object.assign({ source: "browser" }, entry.browserStats),
          gateway: {
            source: "gateway",
            health: entry.gatewayHealth,
            stats: Object.assign({}, entry.gatewayStats),
          },
        };
      }
      return result;
    }

    return {
      CAMERAS: cameras.slice(),
      snapshot,
      start,
      retry,
      restartAll,
      stop,
    };
  }

  global.YAMWebRTC = { CAMERAS, createController, signalingPath };
})(window);
