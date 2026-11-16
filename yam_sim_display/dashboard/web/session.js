/* Operator-session dimension (client-side only).
 *
 * The session is bookkeeping for this browser tab.  It has no ROS handle, no
 * service call, no POST endpoint and therefore no effect on the robot.
 *
 * The canonical transition table is published by the supervision API; this
 * fallback is asserted against the Python copy by the test suite.
 */

"use strict";

(function (global) {
  const STATES = ["idle", "active", "paused", "ended"];

  const TRANSITIONS = {
    idle: ["active"],
    active: ["paused", "ended"],
    paused: ["active", "ended"],
    ended: ["active"],
  };

  const LABELS = {
    idle: "Idle",
    active: "Active",
    paused: "Paused",
    ended: "Ended",
  };

  const ACTION_TARGET = {
    start: "active",
    pause: "paused",
    resume: "active",
    end: "ended",
  };

  // The target table alone cannot distinguish "start" from "resume" (both go
  // to active), so actions are also constrained per state.
  const ACTIONS = {
    idle: ["start"],
    active: ["pause", "end"],
    paused: ["resume", "end"],
    ended: ["start"],
  };

  const STORAGE_KEY = "yam.operator.session.v1";

  function formatElapsed(ms) {
    const total = Math.max(0, Math.floor(ms / 1000));
    const hours = Math.floor(total / 3600);
    const minutes = Math.floor((total % 3600) / 60);
    const seconds = total % 60;
    const mm = String(minutes).padStart(2, "0");
    const ss = String(seconds).padStart(2, "0");
    return hours > 0 ? `${hours}:${mm}:${ss}` : `${mm}:${ss}`;
  }

  function cloneTransitions(table) {
    const out = {};
    for (const state of STATES) {
      if (!table || !Array.isArray(table[state])) return null;
      out[state] = table[state].filter((target) => STATES.includes(target));
    }
    return out;
  }

  function createSession(options) {
    const opts = Object.assign({
      inactivityS: 30,
      persist: true,
      autoTimer: true,
      onChange: null,
    }, options || {});

    let transitions = cloneTransitions(opts.transitions) || cloneTransitions(TRANSITIONS);
    let state = "idle";
    let startedAt = null;
    let endedAt = null;
    let pauseReason = "";
    let lastPollOk = Date.now();
    let hiddenAt = null;
    let eventsSeen = 0;
    let history = [];
    let endedSummary = null;
    let timer = null;
    let listeners = [];

    function snapshot() {
      return {
        state,
        label: LABELS[state] || state,
        elapsed_ms: elapsedMs(),
        elapsed_text: formatElapsed(elapsedMs()),
        pause_reason: pauseReason,
        ended_summary: endedSummary,
        can: {
          start: transitions[state].includes("active"),
          pause: transitions[state].includes("paused"),
          resume: transitions[state].includes("active"),
          end: transitions[state].includes("ended"),
        },
        transitions: cloneTransitions(transitions),
        history: history.slice(),
      };
    }

    function elapsedMs() {
      if (startedAt === null) return 0;
      return Math.max(0, (endedAt === null ? Date.now() : endedAt) - startedAt);
    }

    function persist() {
      if (!opts.persist) return;
      try {
        if (startedAt === null) {
          global.sessionStorage.removeItem(STORAGE_KEY);
          return;
        }
        global.sessionStorage.setItem(STORAGE_KEY, JSON.stringify({
          state,
          startedAt,
          endedAt,
          pauseReason,
          eventsSeen,
          history,
        }));
      } catch (error) {
        /* sessionStorage may be unavailable; the session still works. */
      }
    }

    function restore() {
      if (!opts.persist) return;
      let raw = null;
      try {
        raw = global.sessionStorage.getItem(STORAGE_KEY);
      } catch (error) {
        return;
      }
      if (!raw) return;
      try {
        const saved = JSON.parse(raw);
        if (!STATES.includes(saved.state)) return;
        state = saved.state;
        startedAt = Number.isFinite(saved.startedAt) ? saved.startedAt : null;
        endedAt = Number.isFinite(saved.endedAt) ? saved.endedAt : null;
        pauseReason = typeof saved.pauseReason === "string" ? saved.pauseReason : "";
        eventsSeen = Number.isFinite(saved.eventsSeen) ? saved.eventsSeen : 0;
        history = Array.isArray(saved.history) ? saved.history : [];
        if (state === "ended" && startedAt !== null) {
          endedSummary = buildSummary();
        }
      } catch (error) {
        state = "idle";
      }
    }

    function buildSummary() {
      return {
        duration_s: Math.round(elapsedMs() / 1000),
        duration_text: formatElapsed(elapsedMs()),
        transitions: history.length,
        events_seen: eventsSeen,
        ended_at: endedAt,
      };
    }

    function notify() {
      persist();
      const snap = snapshot();
      for (const listener of listeners) {
        try {
          listener(snap);
        } catch (error) {
          /* a broken listener must not break the session */
        }
      }
      if (typeof opts.onChange === "function") {
        try {
          opts.onChange(snap);
        } catch (error) {
          /* ignore */
        }
      }
    }

    function transition(action, reason) {
      const target = ACTION_TARGET[action];
      if (
        !target
        || !ACTIONS[state].includes(action)
        || !transitions[state].includes(target)
      ) {
        return {
          ok: false,
          action,
          from: state,
          to: state,
          reason: `invalid_transition:${state}->${action}`,
        };
      }

      const from = state;
      const now = Date.now();
      if (action === "start") {
        startedAt = now;
        endedAt = null;
        pauseReason = "";
        eventsSeen = 0;
        endedSummary = null;
        history = [];
      } else if (action === "end") {
        endedAt = now;
        pauseReason = reason || pauseReason;
        endedSummary = buildSummary();
      } else if (action === "pause") {
        pauseReason = reason || "operator";
      } else if (action === "resume") {
        pauseReason = "";
      }

      state = target;
      history.push({ from, to: target, action, at: now, reason: reason || "" });
      notify();
      return { ok: true, action, from, to: target };
    }

    function dispatch(action) {
      return transition(action, "");
    }

    function setTransitions(table) {
      const next = cloneTransitions(table);
      if (!next) return;
      // Called on every poll; only notify when the table actually changes.
      if (JSON.stringify(next) === JSON.stringify(transitions)) return;
      transitions = next;
      notify();
    }

    function setInactivity(seconds) {
      const value = Number(seconds);
      if (Number.isFinite(value) && value > 0) {
        opts.inactivityS = value;
      }
    }

    function setEventCount(count) {
      if (Number.isFinite(count)) eventsSeen = Math.max(eventsSeen, count);
    }

    function notifyPoll(ok) {
      if (ok) lastPollOk = Date.now();
    }

    function checkInactivity() {
      if (state !== "active") return;
      const now = Date.now();
      if (hiddenAt !== null && now - hiddenAt >= opts.inactivityS * 1000) {
        transition("pause", "inactivity");
        return;
      }
      if (now - lastPollOk >= opts.inactivityS * 1000) {
        transition("pause", "inactivity");
      }
    }

    function startTimer() {
      if (!opts.autoTimer || timer !== null) return;
      timer = global.setInterval(function () {
        checkInactivity();
        notify();
      }, 1000);
    }

    function stopTimer() {
      if (timer !== null) {
        global.clearInterval(timer);
        timer = null;
      }
    }

    function subscribe(listener) {
      listeners.push(listener);
      listener(snapshot());
      return function unsubscribe() {
        listeners = listeners.filter((item) => item !== listener);
      };
    }

    if (global.document) {
      global.document.addEventListener("visibilitychange", function () {
        if (global.document.hidden) {
          hiddenAt = Date.now();
        } else {
          checkInactivity();
          hiddenAt = null;
          lastPollOk = Date.now();
        }
      });
    }

    restore();

    return {
      STATES,
      snapshot,
      dispatch,
      subscribe,
      setTransitions,
      setInactivity,
      setEventCount,
      notifyPoll,
      startTimer,
      stopTimer,
      summary: buildSummary,
      // Exposed for the deterministic self-test only.
      _transitionTable: function () { return cloneTransitions(transitions); },
    };
  }

  function runSelfTest() {
    const results = [];

    function check(name, condition, detail) {
      results.push({ name, pass: Boolean(condition), detail: detail || "" });
    }

    const paths = {
      idle: [],
      active: ["start"],
      paused: ["start", "pause"],
      ended: ["start", "end"],
    };
    const actions = ["start", "pause", "resume", "end"];

    for (const state of STATES) {
      for (const action of actions) {
        const session = createSession({ persist: false, autoTimer: false });
        let ok = true;
        for (const step of paths[state]) {
          ok = session.dispatch(step).ok && ok;
        }
        const before = session.snapshot().state;
        const result = session.dispatch(action);
        const expected = ACTIONS[state].includes(action)
          && TRANSITIONS[state].includes(ACTION_TARGET[action]);
        check(
          `${state}:${action}`,
          ok && before === state && result.ok === expected,
          result.reason
        );
      }
    }

    const session = createSession({ persist: false, autoTimer: false });
    session.dispatch("start");
    const ended = session.dispatch("end");
    check("ended_summary_present", ended.ok && session.snapshot().ended_summary !== null);
    check("ended_accepts_new_start", session.dispatch("start").ok);
    check(
      "format_elapsed",
      formatElapsed(65000) === "01:05" && formatElapsed(3665000) === "1:01:05"
    );

    const failures = results.filter((item) => !item.pass);
    const element = global.document ? global.document.getElementById("session-selftest") : null;
    if (element) {
      element.setAttribute("data-selftest", failures.length ? "fail" : "pass");
      element.textContent = failures.length
        ? failures.map((item) => `${item.name}: ${item.detail}`).join(" | ")
        : `${results.length} session checks passed`;
    }
    global.__YAM_SESSION_SELFTEST__ = results;
    return results;
  }

  function mount(elements, session) {
    const el = elements || {};
    const render = function (snap) {
      if (el.state) el.state.textContent = snap.label;
      if (el.stripState) el.stripState.textContent = snap.label;
      if (el.pill) {
        el.pill.className = `sup-pill sup-${sessionPillTone(snap.state)}`;
      }
      if (el.timer) el.timer.textContent = snap.elapsed_text;
      if (el.start) {
        el.start.hidden = !snap.can.start;
        el.start.textContent = snap.state === "ended" ? "New session" : "Start session";
      }
      if (el.pause) el.pause.hidden = !snap.can.pause;
      if (el.resume) el.resume.hidden = !snap.can.resume || snap.state === "ended";
      if (el.end) el.end.hidden = !snap.can.end;
      if (el.note) {
        el.note.textContent = snap.state === "paused" && snap.pause_reason === "inactivity"
          ? "Paused automatically after inactivity. Affects this console only."
          : "Affects this console only — no robot effect.";
      }
      if (el.summary) {
        if (snap.state === "ended" && snap.ended_summary) {
          const summary = snap.ended_summary;
          el.summary.textContent = `Last session: ${summary.duration_text}, `
            + `${summary.transitions} transition(s), ${summary.events_seen} event(s) seen.`;
        } else {
          el.summary.textContent = snap.state === "idle"
            ? "No session started."
            : `Elapsed ${snap.elapsed_text}; ${snap.can.pause ? "running" : "paused"}.`;
        }
      }
      if (el.drawer) {
        const lines = [
          `State: ${snap.label}`,
          `Elapsed: ${snap.elapsed_text}`,
          `Transitions: ${snap.state === "ended" && snap.ended_summary
            ? snap.ended_summary.transitions : "—"}`,
        ];
        if (snap.pause_reason) lines.push(`Pause reason: ${snap.pause_reason}`);
        el.drawer.textContent = lines.join(" · ");
      }
    };

    if (el.start) el.start.addEventListener("click", () => session.dispatch("start"));
    if (el.pause) el.pause.addEventListener("click", () => session.dispatch("pause"));
    if (el.resume) el.resume.addEventListener("click", () => session.dispatch("resume"));
    if (el.end) el.end.addEventListener("click", () => session.dispatch("end"));

    session.subscribe(render);
    session.startTimer();
    return session;
  }

  function sessionPillTone(state) {
    if (state === "active") return "ok";
    if (state === "paused") return "warn";
    if (state === "ended") return "neutral";
    return "unknown";
  }

  global.YAMSession = {
    STATES,
    TRANSITIONS,
    ACTIONS,
    LABELS,
    createSession,
    mount,
    runSelfTest,
    formatElapsed,
  };
})(window);
