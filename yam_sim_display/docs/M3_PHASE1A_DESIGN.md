# M3 Phase 1a — Frozen Design for WebRTC Camera Integration

> Historical design baseline: the implemented physical sender now creates
> follower-local gateway copies on UDP `5100/5102` while retaining native
> leader outputs on `5000/5002`. See `CAMERA_DASHBOARD_RUNBOOK.md` for the
> current workflow. Physical validation is still required before closing the
> gates recorded in this document.

Status: **Phase 1a design freeze. Approved for Emrys's review.**
No implementation files are part of Phase 1a. Phase 1b (dashboard signaling) and all runtime code are explicitly out of scope for this deliverable.

This document is the single source of truth for Phase 1a. It consolidates the M3 Phase 0 PoC findings with the approved M3 Phase 1 conditions and freezes the configuration, API, authentication, schema, test mapping, and the physical validation gate.

### Scope amendment for the research prototype

The user subsequently simplified the Phase 1 scope. Operator authentication, gateway credentials, TLS/certificates, production security hardening, and STUN/TURN deployment are **deferred** and are not implemented in Phase 1b. Phase 1b uses the existing 8080 listener with the existing `bind_address` control (loopback by default, explicit VPN interface address on the NUC), one `webrtc.enabled` ROS parameter, no authentication, and media-only write routes restricted to `/api/webrtc/*`. The frozen sections below remain the long-term reference; where they describe authentication, TLS, or STUN/TURN, those parts are future work, not Phase 1b acceptance criteria.

---

## 1. Scope and ground rules

### 1.1 Constraints

- All new implementation must live inside `yam_sim_display/`.
- `yam_sim_display/spike/` and `yam_sim_display/spike/webrtc_poc/` are preserved verbatim.
- No changes to `yam_leader/`, `yam_follower/`, contributor-owned camera sender/receiver pipelines, DDS, VPN, firewall, camera senders, or network configuration.
- No robot actuation and no command publishing.
- Synthetic PoC results are feasibility evidence only, never physical camera validation.
- Architecture remains transport-independent; the existing read-only safety boundary is preserved.

### 1.2 Verified baseline in the current tree

- `dashboard/http_server.py` is GET/HEAD-only. `do_GET` serves static assets and `/api/state` from a cached snapshot; it never queries ROS.
- `dashboard/node.py` has exactly two `create_subscription` calls, both `JointState`, zero `create_publisher`, and no command-topic literals.
- `test/test_supervision_engine.py:504-514` asserts the handler set is exactly `{"do_GET", "do_HEAD"}`; lines `473-502` assert no publishers, two `JointState` subscriptions, and no forbidden command tokens (`/yam_teleop`, `cmd_vel`, `joint_command`, `/forward_position_controller/commands`) anywhere under `dashboard/`.
- `node.py` merges additively and sets `schema = 2`.
- Safety source is `none`, so safety is Unknown and readiness is Not Ready by design. `telemetry.py` deliberately reports `latency.available:false` and camera `state:pending`.
- The frontend has a single camera card driven by `camera.stream_url`.
- D405 uses UDP 5000 and D455f uses UDP 5002 (`yam_follower/README_TWO_NUCS.md:83`).
- The PoC demonstrated two independent `webrtcbin` pipelines and two browser `RTCPeerConnection`s, H.264 forced via `setCodecPreferences`, PT restamp `96 -> 102`, no transcode, failure isolation, ~150/70 ms burned-in-clock latency, and ~3.4 Mbps per camera.

### 1.3 Approved conditions applied

| Approved decision | Frozen into |
| --- | --- |
| Gateway Follower-side preferred, conditional on physical RTP topology; Leader-side fallback requires investigation | §2, §8 (gate `PV-RTP-01`) |
| Same 8080 listener; explicit VPN-interface binding; localhost remains default | §2, §3, §4 |
| Separate long-lived gateway credential and short-lived browser session; gateway secret never in JS; HTTPS required off-loopback | §5 |
| Per-tab sessions and independent camera mailboxes | §4, §6 |
| Legacy `camera` block unchanged | §7 |
| Schema stays `2`, additive optional `webrtc` | §7 |
| Camera IDs `d405`, `d455f` | §3, §7 |
| Non-trickle ICE | §3, §6 |
| Prefer VPN-interface candidates; preserve browser mDNS privacy | §3, §6 |
| STUN/TURN optional and unprovisioned until confirmed | §3, §4 |

---

## 2. Consolidated final architecture

```
                          LEADER NUC                                  FOLLOWER NUC
  ┌───────────────────────────────────────────────┐   ┌─────────────────────────────────────────────┐
  │ dashboard_node (ROS, read-only)               │   │ camera senders (contributor-owned,          │
  │  GET /api/state, static assets                │   │ UNMODIFIED) -> RTP UDP 5000 / 5002          │
  │  signaling store: (session_id, camera_id)     │<--+-- webrtc_gateway process (GStreamer)        │
  │  8080 listener, bind_address default loopback │   │  d405 leg --+                              │
  │  cookie session auth + Bearer gateway auth    │   │  d455f leg --+ DTLS/SRTP ----------------->│
  └──────────────────────┬────────────────────────┘   └─────────────────────────┬─────────────────┘
                         | same-origin (cookie auto-sent)                        |
                         v                                                       |
              Operator browser on PC (Leader-side)  <----- WebRTC media ---------+
              two <video>; getStats(); per-camera health
```

- **Leader:** existing `dashboard_node` plus an isolated signaling store inside the HTTP layer. ROS behavior is identical to M2: no publisher, two `JointState` subscriptions, no command literals.
- **Follower:** a new `webrtc_gateway` executable launched by a new `yam_sim_display` launch file. GStreamer/glib never enter `dashboard/node.py` or `http_server.py`.
- **Failure domains:** a gateway crash cannot take down the dashboard; the dashboard cannot touch the ROS control path; signaling loss cannot drop an already-negotiated peer connection.
- **Transport independence:** a `MediaTransport` protocol defines `offer`, `answer`, `candidates`, `stats`, and `health`. The dashboard store and UI depend on the protocol, not on GStreamer. WebRTC is the Phase 1 implementation.
- **Media path per camera (unchanged from PoC, no transcode):**

  `udpsrc -> rtpjitterbuffer -> rtph264depay -> h264parse -> rtph264pay -> queue -> webrtcbin`

  `rtph264pay` payload type is re-stamped from the negotiated answer. There is no encoder, decoder, or converter.
- **RTP source host/port are config values**, so the same gateway supports Follower-local or Leader-side topologies without code changes.
- **Candidate policy:** host candidates on the VPN interface are preferred. STUN (`srflx`) and TURN (`relay`) are optional and unprovisioned. Browser mDNS privacy is preserved: `.local` candidates are neither requested nor relied upon.
- **Latency honesty:** only RTT and jitter are reported as network metrics. No glass-to-glass latency is computed or displayed; `latency.available` stays false.

---

## 3. Frozen configuration parameters and defaults

### 3.1 Dashboard node

Existing parameters (`port=8080`, `bind_address=127.0.0.1`, `refresh_ms=100`, freshness and diagnostics settings) are unchanged. New parameters:

| Parameter (dotted) | Type | Default | Purpose |
| --- | --- | --- | --- |
| `webrtc.enabled` | bool | `False` | Feature gate. When false, no `/api/webrtc/*` routes are registered and the `webrtc` key is omitted from `/api/state`, so the M2 snapshot is byte-identical. |
| `webrtc.camera_ids` | list[str] | `["d405","d455f"]` | Allowlist for `camera_id` on every endpoint. |
| `webrtc.gateway_credential_file` | str | `""` | Path to the long-lived gateway secret, read from file only. Empty means gateway auth fails closed. |
| `webrtc.operator_credential_file` | str | `""` | Path to the operator credential store used to issue browser sessions. |
| `webrtc.session_ttl_s` | float | `300.0` | Lifetime of the short-lived browser session cookie. |
| `webrtc.max_sessions` | int | `64` | Global cap on live per-tab sessions. |
| `webrtc.session_cleanup_interval_s` | float | `10.0` | TTL sweep interval for sessions and mailboxes. |
| `webrtc.require_https` | bool | `True` | Credentials over plaintext on a non-loopback peer are rejected. |
| `webrtc.allow_insecure_loopback` | bool | `True` | Dev-only plaintext exception on loopback. |
| `webrtc.reject_wildcard_bind` | bool | `True` | With signaling enabled, reject `bind_address` of `0.0.0.0` or `::`, forcing an explicit VPN-interface IP. |
| `webrtc.offer_timeout_s` | float | `10.0` | Gateway long-poll timeout for offers. |
| `webrtc.answer_timeout_s` | float | `10.0` | Browser long-poll timeout for answers. |
| `webrtc.gateway_stats_max_age_s` | float | `5.0` | Age beyond which gateway health is reported stale. |
| `webrtc.browser_poll_ms` | int | `1000` | Browser poll cadence, exposed via `/api/webrtc/config`. |

Binding rule: `bind_address` remains `127.0.0.1` as shipped. For M3 deployment it is set explicitly to the VPN interface address. Wildcard bind is rejected when `webrtc.enabled=True`. There is no separate signaling port and no CORS.

### 3.2 Gateway process

| Parameter | Type | Default |
| --- | --- | --- |
| `camera_ids` | list[str] | `["d405","d455f"]` |
| `cameras.d405.rtp_host` / `cameras.d405.rtp_port` | str / int | `127.0.0.1` / `5000` |
| `cameras.d455f.rtp_host` / `cameras.d455f.rtp_port` | str / int | `127.0.0.1` / `5002` |
| `signaling.base_url` | str | `""` (must be supplied) |
| `signaling.gateway_credential_file` | str | `""` |
| `signaling.tls_ca_file` | str | `""` |
| `signaling.insecure_skip_tls_verify` | bool | `False` |
| `signaling.request_timeout_s` | float | `10.0` |
| `ice.preferred_interface` | str | `""` (VPN interface name; empty means auto-select) |
| `ice.allowed_cidrs` | list[str] | `[]` (empty means all host candidates, preference still applied) |
| `ice.allow_mdns_candidates` | bool | `False` |
| `ice.stun_server` | str | `""` (unprovisioned) |
| `ice.turn_server` | str | `""` (unprovisioned) |
| `ice.turn_username` | str | `""` (unprovisioned) |
| `ice.turn_credential_file` | str | `""` (unprovisioned) |
| `ice.gather_timeout_s` | float | `10.0` |
| `stats.interval_s` | float | `1.0` |
| `stats.history_size` | int | `300` |
| `reconnect.initial_backoff_s` | float | `1.0` |
| `reconnect.max_backoff_s` | float | `30.0` |

### 3.3 Frozen constants (not configurable in Phase 1)

- Path prefix: `/api/webrtc`. Paths outside it accept no write methods.
- Camera IDs: `d405`, `d455f`.
- ICE mode: non-trickle only; `/candidate` returns `501`.
- Candidate preference: VPN interface first; no mDNS reliance; no STUN/TURN unless explicitly configured.
- Cookie name `yam_session`; attributes `HttpOnly`, `Secure` when TLS is used, `SameSite=Strict`, `Max-Age = session_ttl_s`.
- Gateway header: `Authorization: Bearer <gateway_secret>`.
- Top-level schema `2`; additive block sub-schema `1`; block omitted when disabled.
- No transcode; `video/H264`; PT re-stamped from the negotiated answer.

---

## 4. Frozen API endpoint contracts

All paths are under `/api/webrtc/`. Any `POST`, `DELETE`, `PUT`, or `PATCH` to any other path returns `405`. Unknown camera IDs return `404`; malformed bodies `400`; wrong credential class `403`; missing or invalid credential `401`; rate limit `429`.

| Method | Path | Auth | Request | Success | Notes |
| --- | --- | --- | --- | --- | --- |
| `POST` | `/session` | Operator credential in body | `{user, password}` | `201 {session_id, camera_ids, expires_at}` plus `Set-Cookie` | Only endpoint accepting an operator credential. Issues the short-lived cookie. |
| `POST` | `/keepalive` | Browser cookie | — | `204` | Refreshes session TTL. |
| `DELETE` | `/session?session_id=` | Browser cookie | — | `204` | Destroys session and all its mailboxes. |
| `POST` | `/offer` | Browser cookie | `{camera_id, session_id, sdp}` | `202 {revision}` | Writes only mailbox `(session_id, camera_id)`. |
| `GET` | `/answer?camera_id=&session_id=` | Browser cookie | — | `200 {sdp, revision}` or `204` on timeout | Browser long-poll, `answer_timeout_s`. |
| `GET` | `/config` | Browser cookie | — | `200 {camera_ids, poll_ms, ice:{servers:[], prefer_vpn_candidates:true, allow_mdns:false, non_trickle:true}}` | No secrets and no credential filenames. |
| `GET` | `/status?camera_id=` | Browser cookie | — | `200` per-camera signaling state | Diagnostic only. |
| `GET` | `/offer?camera_id=&session_id=` | Gateway Bearer | — | `200 {sdp, revision}` or `204` on timeout | Gateway long-poll, `offer_timeout_s`. |
| `POST` | `/answer` | Gateway Bearer | `{camera_id, session_id, sdp}` | `202 {revision}` | Writes the answer into that one mailbox. |
| `POST` | `/candidate` | Gateway Bearer | — | `501` | Reserved for the non-trickle contract. |
| `POST` | `/gateway-stats` | Gateway Bearer | `{camera_id, session_id, stats, health}` | `202` | Feeds the gateway half of diagnostics. |
| `POST` | `/gateway-heartbeat` | Gateway Bearer | `{camera_id, session_id}` | `204` | Liveness for gateway health. |

### 4.1 Authentication behavior

1. **Gateway credential** — long-lived, read from `webrtc.gateway_credential_file` by the gateway process only. Accepted only on gateway-scoped endpoints. Never rendered into HTML or JavaScript, never returned by `/api/webrtc/config`, never logged.
2. **Browser session** — the operator credential is exchanged at `POST /session` for an opaque cookie valid for `session_ttl_s`. Accepted only on browser-scoped endpoints. The cookie is `HttpOnly`, so JavaScript cannot read the token; JS uses only the non-secret `session_id`.
3. **Scope separation** — a Bearer token presented to a browser endpoint, or a cookie presented to a gateway endpoint, returns `403`.
4. **Transport** — with `require_https=True`, any request carrying credentials over plaintext on a non-loopback peer is rejected. The loopback exception applies only when `allow_insecure_loopback=True`. The `Secure` cookie flag is set whenever TLS is in use.
5. **Mailbox model** — key `(session_id, camera_id)`; fields `offer`, `answer`, `revision`, `created_at`, `expires_at`, `gateway_stats`, `gateway_health`. Cameras are independent: an error in `d405` cannot write, clear, or expire `d455f`. The TTL sweep removes all mailboxes belonging to an expired session.
6. `do_GET` and `do_HEAD` for `/api/state` and static assets are unchanged.

---

## 5. Frozen `webrtc` schema

When `webrtc.enabled=False`, the `webrtc` key is absent and all existing fields (`schema=2`, `camera`, `latency`, `supervision`, `events`) are unchanged. When enabled:

```json
"webrtc": {
  "schema": 1,
  "enabled": true,
  "signaling": {
    "state": "ok|degraded|down",
    "tls_required": true,
    "session_count": 2,
    "last_error": null,
    "last_update": "<iso>"
  },
  "cameras": [
    {
      "id": "d405",
      "label": "D405",
      "state": "disabled|connecting|live|degraded|stale|disconnected|error",
      "session_active": true,
      "connection_state": "new|connecting|connected|disconnected|failed|closed",
      "ice_connection_state": "new|checking|connected|completed|failed|disconnected|closed",
      "codec": "video/H264",
      "resolution": "1280x720",
      "fps": 30.0,
      "bitrate_kbps": 3309.3,
      "packets_lost": 0,
      "jitter_ms": 0.4,
      "freeze_count": 0,
      "rtt_ms": 12.3,
      "candidate_type": "host|srflx|relay|unknown",
      "mdns_privacy": true,
      "gateway": {
        "state": "waiting|answering|connected|degraded|error",
        "last_heartbeat": "<iso>"
      },
      "diagnostics": [{"code": "RTC_ICE_FAILED", "message": "...", "since": "<iso>"}],
      "last_update": "<iso>"
    },
    { "id": "d455f", "label": "D455f", "same shape": true }
  ]
}
```

Rules:

- `rtt_ms` and `jitter_ms` are network metrics only. There is no glass-to-glass latency field; `latency.available` stays `false`; the legacy `camera` block stays `state:pending`, `stream_url:null`.
- The browser computes `connection_state`, `ice_connection_state`, codec, resolution, fps, packets lost, jitter, freeze count, RTT, and candidate type from `getStats()`. The gateway computes `gateway.*`, incoming RTP bitrate, and heartbeat.
- No field in `webrtc` is an input to `HealthEvaluator`, `ReadinessEvaluator`, `SafetyEvaluator`, or `MonitorView`. Safety remains Unknown and readiness remains Not Ready.

---

## 6. Acceptance criteria mapped to tests

New test files are planned for Phase 1b/1g; they do not exist yet. Tests marked *existing* must remain unchanged except where noted.

| # | Acceptance criterion | Test(s) |
| --- | --- | --- |
| 1 | Two simultaneous 1280x720 H.264 streams render in the production UI | `test/test_webrtc_gateway.py::test_two_camera_pipelines_answer_and_stream`; `test/test_webrtc_ui.py::test_two_video_elements_independent` |
| 2 | Killing one camera leaves the other live | `test/test_webrtc_gateway.py::test_single_camera_failure_isolation`; `test/test_webrtc_signaling.py::test_camera_mailbox_isolation` |
| 3 | No transcode; PT restamped | `test/test_webrtc_gateway.py::test_payload_type_restamp_and_no_transcode` (reuses the PoC element-list check) |
| 4 | Per-camera health independent; safety Unknown / readiness Not Ready | `test/test_webrtc_schema.py::test_webrtc_not_consumed_by_supervision`; `test/test_supervision_health_readiness.py` *(existing, unchanged)*; `test/test_webrtc_signaling.py::test_camera_mailbox_isolation` |
| 5 | POST/DELETE restricted to `/api/webrtc/*`; auth enforced; no ROS path | `test/test_webrtc_signaling.py::test_non_webrtc_write_returns_405`; `test/test_auth_scopes.py::test_credential_scope_separation`; `test/test_supervision_engine.py::test_node_never_publishes_and_only_reads_joint_states` *(existing, unchanged)*; `::test_no_command_topics_anywhere_in_dashboard` *(existing, unchanged)* |
| 6 | Stats visible per camera | `test/test_webrtc_ui.py::test_stats_rendered_per_camera`; `test/test_webrtc_schema.py::test_stats_fields_present` |
| 7 | No glass-to-glass latency claim | `test/test_telemetry.py::test_camera_and_latency_are_honest` *(existing, unchanged)*; `test/test_webrtc_schema.py::test_no_latency_field_in_webrtc_block` |
| 8 | M1/M2 tests pass with narrow update | Full suite; only `test/test_supervision_engine.py::test_http_layer_has_no_write_handlers` is narrowed to `{"do_GET","do_HEAD","do_POST","do_DELETE"}` plus explicit rejection assertions |
| 9 | Diff scoped to `yam_sim_display/`; `spike/` untouched | `test/test_repo_scope.py::test_spike_tree_unmodified` plus review gate `git diff --name-only` |
| 10 | Config documented and frozen | `test/test_webrtc_config.py::test_defaults_match_yaml`; `::test_camera_ids_frozen`; `::test_stun_turn_defaults_empty` |

Additional auth and security tests required by the approved conditions:

- `test/test_auth_scopes.py::test_gateway_secret_absent_from_js_bundle`
- `test/test_auth_scopes.py::test_browser_cookie_is_httponly_secure_samesite`
- `test/test_auth_scopes.py::test_wildcard_bind_rejected_when_signaling_enabled`
- `test/test_auth_scopes.py::test_credentials_require_https_off_loopback`
- `test/test_webrtc_signaling.py::test_per_tab_session_mailbox_isolation`
- `test/test_webrtc_signaling.py::test_session_ttl_cleanup`

---

## 7. Physical validation gate `PV-RTP-01`

**Question:** On the real Follower NUC, are the D405 and D455f RTP streams on UDP 5000 and 5002 observable by a new process without modifying contributor sender/receiver pipelines?

- **Evidence required:** packet capture and `ss -ulpn` on both NUCs showing where RTP terminates; sender destination from the ignored local camera configuration; confirmation that no file under `yam_leader/` or `yam_follower/` changed.
- **Pass:** keep the Follower-side gateway and set `cameras.*.rtp_host` to the observed source.
- **Fail:** Follower placement is invalid. The Leader-side gateway fallback requires a new investigation and design revision before Phase 1c. No contributor files may be touched.
- **Blocks:** Phase 1c (gateway) and Phase 1g (acceptance run). It does **not** block Phase 1b (dashboard signaling), which is topology-independent.

Additional deferred physical checks: real camera wire shape, burned-in-clock glass-to-glass latency, Ethernet/Wi-Fi/VPN candidate selection, STUN/TURN availability, NUC CPU/bandwidth, USB reconnect, `webrtcbin` version parity, and zero control-path impact during camera faults.

---

## 8. Phase sequence

| Phase | Deliverable | Exit criterion | Status |
| --- | --- | --- | --- |
| 1a | This frozen design document | Emrys review | **Complete** |
| 1b | Signaling store, POST/DELETE routing, auth, unit and HTTP tests, narrow strict-test update | Tests green; non-webrtc POST rejected | Not started |
| 1c | Production gateway module, two pipelines, PT restamp, no transcode, stats, executable and launch | Two synthetic streams answer; no-transcode check passes; `PV-RTP-01` passed | Not started |
| 1d | Frontend two-video cards, per-camera health, additive `webrtc` block | Both videos live; supervision unchanged | Not started |
| 1e | Diagnostics drawer, event-log integration, stats surfacing | Per-camera stats and RTT visible; no false latency | Not started |
| 1f | ICE, STUN/TURN, candidate-policy config and docs | Candidate policy configurable; clean fallback | Not started |
| 1g | Integration smoke test and full acceptance run on the dev PC | All 10 criteria in §6 met | Not started |
| 1h | Physical NUC validation plan and handoff only | §7 checklist owned by Emrys | Not started |

---

## 9. Review checklist for Emrys

- [ ] Confirm Follower-side gateway placement, conditional on `PV-RTP-01`.
- [ ] Confirm explicit VPN-interface binding with loopback default and no wildcard bind.
- [ ] Confirm two-tier authentication and the ban on exposing the gateway secret to JavaScript.
- [ ] Confirm HTTPS requirement off-loopback.
- [ ] Confirm per-tab sessions, independent camera mailboxes, and the frozen cookie contract.
- [ ] Confirm schema `2` with an optional `webrtc` sub-schema `1` and unchanged legacy `camera` block.
- [ ] Confirm `d405`/`d455f`, non-trickle ICE, VPN candidate preference, mDNS privacy, and unprovisioned STUN/TURN.
- [ ] Confirm the acceptance-criterion to test mapping, especially the single narrow change to `test_http_layer_has_no_write_handlers`.
- [ ] Confirm `PV-RTP-01` as a blocking gate for Phase 1c and 1g.

No Phase 1b work may begin until this review is approved.
