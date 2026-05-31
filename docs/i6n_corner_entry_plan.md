# Ioniq 6 N — Corner-Entry Improvement Plan (i6n branch, self-contained)

Self-contained record of the corner-entry improvement work on branch `i6n`
(commits up to **f2b234f**). This document is intentionally redundant with the
commit messages so that future readers (human or LLM) need no prior chat
context to evaluate the work and continue it.

---

## 0. Scope and goals

Vehicle: **Hyundai/Kia CCNC + CANFD_LKA_STEERING_ALT** platform, first member
**Kia / Hyundai Ioniq 6 N (2026)**. Lateral control path is **angle-control via
LKAS_ALT on A-CAN**; the legacy torque path is unused.

The user-perceived symptom we set out to address: residual **low-speed
"tick"** and **late corner entry** vs. stock LFA, plus a small set of UX
regressions (cluster icon flicker, MADS dropouts during gear toggles).

Out of scope (explicitly excluded by the user this round): Lane Change Timer
overhaul, DM-based gating, lateral-offset features.

---

## 1. Changes shipped on `i6n` (commit-by-commit)

| Commit | One-liner | Files touched |
|---|---|---|
| 96ea9ea | `log.capnp`: add `lateralAccelLimit / steerAngleLimit / cameraDataStale` @99/100/101 | `cereal/log.capnp` |
| 958f4dd | hyundai/ccnc: remove `LON_COMFORT` dead fallback (referenced undefined params) | `opendbc_repo/opendbc/car/hyundai/carcontroller.py`, `values.py` |
| 98a961c | hyundai/ccnc: gate steering on `cam_stale + fault_lfa`, smoother override + gear release | `carcontroller.py`, `hyundaicanfd.py`, `carstate.py` |
| 529bc57 | hyundai/ccnc: remove parking-mode dead code, document `BASELINE_VM` purpose | `carcontroller.py` |
| f2b234f | hyundai/ccnc: align LKA_ASSIST comment with real gates | `hyundaicanfd.py` (comment-only) |

### 1.1 cam_stale + fault_lfa gate (98a961c)

**Problem.** Previously op continued sending angle commands even when the
camera frame (`LKAS_ALT` from address 0x162) was stale or the camera reported
`FAULT_LFA=1`. The cluster could end up green ("MADS engaged") while op was
actually following the last known reference of a dead camera.

**Fix.** In `carcontroller.py`:
* `cam_stale` is set when the LKAS_ALT camera message inter-arrival exceeds
  **250 ms** (~30 frames @ 100 Hz). The threshold is intentionally generous —
  factory CAN bus jitter sits well below 50 ms, but margins matter on real
  vehicles.
* `fault_lfa` is read directly from the camera's `FAULT_LFA` bit (1 = camera
  says LFA path is unhealthy).
* When **either** is true, `cam_invalid=True` is forwarded to
  `create_steering_messages`. The dict-constructed LKAS_ALT frame then sets:
    * `LKA_ASSIST = 0` (drop the green steering icon; reflects actual MADS
      health).
    * `LKAS_ANGLE_ACTIVE = 1` (explicit passive; never forward a frozen
      "active=2" snapshot from a dead camera).
* `cameraDataStale` is logged at `controlsState.cameraDataStale` (cereal field
  index 101) so drivelog analysis can quantify the false-positive rate.

**Risk.** A normal CAN-bus blackout of >250 ms on the LKAS_ALT message will
trip the gate and visibly drop the green icon. Drivelog histogram of 0x162
inter-arrival is the right verification.

### 1.2 Smoother override + gear release (98a961c)

**Override factor gate.** The pre-existing `error_boost` (which raised
authority during large lateral error) used to fire *during driver overrides*,
producing a counter-productive "fight" feel. Gated by
`override_factor < 0.5` so error_boost is suppressed once the driver is more
than half-way committed to overriding.

**Gear release.** When the driver toggles into R and then back to D (or sport,
eco, manumatic), MADS was holding a "was_in_reverse" latch for tens of frames,
delaying re-engagement. Latch now releases the same frame the gear lever
returns to a forward gear.

### 1.3 LON_COMFORT dead-code removal (958f4dd)

`opendbc_repo/opendbc/car/hyundai/carcontroller.py` had a `LON_COMFORT`
fallback path referencing parameters that were never defined elsewhere in the
tree. It was unreachable (the branch condition was permanently false), but
linters / future refactors could trip on it. Removed entirely.

### 1.4 Parking-mode dead code removal + BASELINE_VM doc (529bc57)

The `parking_fully_faded` branch in `create_steering_messages` was a leftover
from an earlier prototype where op faded out during parking. The actual
parking handling has been moved upstream (carcontroller decides
`in_passthrough` directly). The dead reference is removed.

`BASELINE_VM` constant now has a docstring explaining its role as the
*reference vehicle model* used for rate-limit shaping. Without the comment,
future readers had no way to tell `BASELINE_VM` from a tuning knob.

### 1.5 cereal additions (96ea9ea)

Added three new optional fields to `controlsState` in `cereal/log.capnp`:
* `lateralAccelLimit @99 : Float32` — saturated lateral accel limit hit this
  frame (in m/s^2). Currently sourced from panda safety's `MAX_LATERAL_ACCEL`
  (3.6 m/s^2), enabling UI / drivelog visibility into saturation events.
* `steerAngleLimit @100 : Float32` — saturated steering-angle limit.
* `cameraDataStale @101 : Bool` — output of the cam_stale gate.

Field indices were placed at the end of the existing struct in append-only
fashion; no existing field was renumbered.

---

## 2. Architecture, in brief

```
   model planner                           camera (ADAS DRV / 0x162)
        |                                          |
        v                                          v
   carcontroller.py  <----- lkas_alt_cam_msg ------+
        |  ^                                       |
        |  +-- override_factor, gear, MADS state --+
        |
        |   in_passthrough, lat_active, apply_angle,
        |   effective_aci_gain, cam_invalid, mads_force_assist
        v
   hyundaicanfd.create_steering_messages
        |
        v
   LKAS_ALT on A-CAN  ->  MDPS
```

### Key invariants

1. **Single emit path.** `create_steering_messages` no longer has separate
   "passthrough" and "active" branches that produced *structurally different*
   LKAS_ALT frames. ADAS DRV flagged the format switch on real routes
   (3a / 32 / 34 on ccnc-port-prebuilt). The same dict is used for both
   states; `in_passthrough` only modulates `rate_lat_active` upstream.
2. **Always-active strategy.** When `lat_active=True`, `steering_active=True`
   unconditionally. The previous "five intermediate gates" (authority
   hysteresis, cam_stale, speed_blend, blinker attenuation) gating
   `LKAS_ANGLE_ACTIVE` caused dropouts (route 0x49: 23.9% of `latActive`
   frames had `steering_active=False`). Effort is now modulated via
   `ACIGain` (0.0..1.0) continuously while keeping `ACTIVE=2` stable.
3. **MDPS handles edge cases** (standstill, override) via its own safety
   limits; openpilot does not duplicate that logic.
4. **Counter monotonicity.** `create_suppress_lfa` owns its own monotonic
   COUNTER because the panda relay blocks the camera's original 0x362/0x2a4
   on A-CAN — ADAS DRV only sees our TX stream, and any +1 gap is flagged.
5. **HOD bypass removed** (commit `c9a1ed6`). The dormant 0x208 scaffold
   in carcontroller and the panda TX whitelist entry were deleted after
   verifying drivelog 0x2d/0x2e showed nag escalation = 0 and sunnypilot
   carries no HOD sensor spoof either. See Appendix H of
   `IONIQ6N_STEERING_MASTERPLAN.md` for the historical feasibility
   analysis.

---

## 3. What remains (prioritized punch list)

### P0 — verification gating before any further tuning

| # | Item | How to verify |
|---|---|---|
| 1 | `cam_stale` 250 ms gate false-positive rate | Drivelog histogram of 0x162 inter-arrival across >=1 hour mixed driving |
| 2 | `fault_lfa` frequency in normal driving | Grep drivelog for `CS.fault_lfa == True` outside of known faults |
| 3 | `error_boost x override_factor` interaction in real corners | A/B drivelog of corner entry events before vs. after 98a961c |
| 4 | R->D immediate gear release covers all `GearShifter` enums (CVT / EV) | Parking-lot toggle test on a real Ioniq 6 N |

### P1 — code quality, no behavior change

5. Magic numbers in `carcontroller.py` need named constants:
   `30` (cam_stale frames), `0.5` (override factor threshold), `3/15`
   (snap enter/exit), `10 * KPH_TO_MS` (low-speed gate).
6. `LKAS_ALT` fallback bytes (`0x92 / 0x01 / 0xFF`) — fallback only fires
   before first camera message arrives at boot. Drivelog scan of 35,853
   real-camera frames showed BYTE28..31 always `0x00`, so the fallback now
   emits `0x00` to match what ADAS DRV expects on boot.
7. Audit `LKA_ASSIST x LKAS_ANGLE_ACTIVE` consistency under
   `mads_force_assist`: a transient window can show cluster icon green
   while op is internally passive. Either suppress the icon during that
   window or formalize the state as "MADS engaged, momentarily handing
   off."

### P2 — structural limits (cannot change in op)

8. Panda safety `MAX_LATERAL_ACCEL = 3.6 m/s^2` is fixed in panda firmware.
   On high-curvature corners saturation will occur; we now log it via the
   new `lateralAccelLimit` field but cannot exceed.
### P3 — explicitly deferred per user

10. Lane Change Timer overhaul.
11. DM-based gating tuning.
12. Lateral-offset features.

---

## 4. Verification checklist (re-run before any release)

* [ ] Unit tests in `selfdrive/car/tests/test_car_interfaces.py` pass for
      Ioniq 6 N fingerprint.
* [ ] `pytest opendbc_repo/opendbc/car/tests` green.
* [ ] Replay one drivelog from a real Ioniq 6 N (route `00000031--85ea5c34a8`
      is the labelled HOD route, others are fine for steering verification)
      and confirm:
    - No `controlsState.cameraDataStale==True` during normal driving.
    - `cluster green icon` does not flicker on a typical 60-second
      mixed-arterial segment.
    - Gear R->D toggle re-engages MADS within one frame.
* [ ] Build firmware and verify panda safety acceptance for Ioniq 6 N.

---

## 5. Glossary (for the next reader)

* **CCNC** — Center Console Navigation Cluster. The vehicle's display ECU
  that owns the instrument cluster steering icon, lane lines, etc.
* **CANFD_LKA_STEERING / _ALT** — HyundaiFlags subtypes describing which
  CAN bus the LKA camera lives on. The "ALT" variant (Ioniq 6 N) speaks
  angle-control on A-CAN.
* **MDPS** — Motor Driven Power Steering. The actual EPS controller.
* **ADAS DRV** — the camera-side perception/decision ECU that emits
  `LKAS_ALT` (0x162) and validates the stream that op sends back.
* **ACI / ACIGain** — Active Camera Input gain. The 0..1 authority knob
  that MDPS reads from `ADAS_ACIAnglTqRedcGainVal`.
* **MADS** — Modular Assistive Driving System. The "always-on lateral
  assist" mode used in the Sunnypilot tree.
* **HOD** — Hands-On Detection. Capacitive grip sensor on the wheel; its
  status is broadcast on 0x208.

---

## 6. Pointers

* `opendbc_repo/opendbc/car/hyundai/carcontroller.py` — top-level lateral /
  longitudinal control for CCNC.
* `opendbc_repo/opendbc/car/hyundai/hyundaicanfd.py` — CAN frame builders;
  `create_steering_messages` is the LKAS_ALT entry point.
* `opendbc_repo/opendbc/car/hyundai/carstate.py` — `cam_stale`,
  `fault_lfa`, and gear-shifter parsing.
* `cereal/log.capnp` — fields 99/100/101 for drivelog instrumentation.
* `IONIQ6N_STEERING_MASTERPLAN.md` — long-form design notes; Appendix H
  preserves the HOD bypass feasibility analysis as historical record
  (feature removed in commit `c9a1ed6`).

---

## 7. Current state (as of `c9a1ed6` + audit-only follow-ups)

This section is appended in 2026-05-31 after merging Phase 6F2-A
(`d83c3b5`), removing the dead HOD bypass scaffold (`c9a1ed6`), and
analyzing 12 drivelog routes (`ccnc-drivelog` branch) across two build
generations on real Ioniq 6 N hardware. The lateral-control work
documented in §1 is considered **done**; future work is observation +
tooling.

### Build provenance verified

| Build | Routes analyzed | rlog segs | LKAS_ALT frames |
|---|---|---:|---:|
| `5479ecc` Phase 6f-2 | 0x24-0x2c (8 routes, 1 incomplete) | 290 | 1.71 M |
| `d83c3b5` Phase 6F2-A | 0x2d, 0x2e (2 routes, commute) | 59 | 522 k |
| `c9a1ed6` HOD cleanup | 0x2f, 0x30, 0x31, 0x35 (4 routes, 6 logs) | 158 | 934 k |
| **Total** | 14 routes | **507 segs** | **3.17 M** |

### Headline results (all on `c9a1ed6`)

* **Heavy-override op-active TX-wheel tracking** = **0.10° p95** across
  all 4 most-recent routes (= CAN quantization floor). 6F2-A
  pre-frame anchor and the legacy snap_to_wheel both unnecessary in
  the deployed steady state — the override blend already lands here.
* **Sign-mismatch** = 0.00–0.06% of heavy-override frames. Essentially
  zero. The originally reported 23–41% was an artifact of decoding
  ADAS_StrAnglReqVal at bytes 4-5 instead of bytes 10-11 (DBC bit
  82, length 14, little-endian, signed) — every prior heavy-override
  statistic that used bytes 4-5 was returning the constant 0x80 →
  12.8° and is invalid.
* **Highway 80+ kph op-active tracking** (first measured on
  `c9a1ed6`) = p95 0.8–1.9°, p99 1.8–5.3°, max ≤ 8.2° across
  170 k frames. Stage 1 of the masterplan is effectively met
  without further code change.
* **Hand-off lag P0 = artifact.** Of 54 detected "hand-off" events,
  31 had lag > 500 ms with driver torque median 140 Nm during the
  lag window — the driver was continuously gripping at ~half-full
  override, not releasing. Real hand-offs (~13 events) resolved
  inside 100 ms. Phase 6d EXIT-torque sensitivity at 30 → 50 Nm
  resolves 4 / 31 of the "stuck" events; 100 → 150 Nm resolves
  more but starts to flap during normal driving. **No code change
  recommended** — the gate is correctly yielding to a still-active
  driver.
* **No safety regression after HOD cleanup**: TX audit clean, panda
  counters Δ 0, SCC bus 1 collisions 0, camera RX max-gap ≤ 50 ms,
  ACI flip hotspots none, carState clean, NaN steering 0 in 4 routes.
* **ALERTS_3 = 11 (camera HDP takeover prompt)** fires ~13 s across
  the 4 routes = 0.23 % wall time. The local code already suppresses
  it on non-HDA2 CCNC, but HDA2-ALT (i6n) declines to publish
  CCNC_0x161/0x162 at all (dual-publisher fault risk on bus 1);
  sunnypilot's `hkg-angle-steering-2025-hda1` declines for the same
  reason. Cosmetic only — no functional effect (ALERTS_5 grasp /
  speed-limited = 0 events). Not actionable on i6n without a separate
  experiment to characterize the dual-publisher behavior in person.

### What changed under the hood since §1 was written

* **Phase 6F2-A** (`d83c3b5`): pre-frame anchor for heavy-override
  transition. Drivelog measurement showed the post-frame clamp
  already produced near-zero mismatch and the pre-frame anchor is
  belt-and-braces. Harmless.
* **HOD bypass cleanup** (`c9a1ed6`): removed the dormant 0x208
  scaffold (`create_hod_bypass()`, the `{0x208, e_can, 16}` panda
  whitelist entry) — verified no code path was calling it,
  `HOD_BYPASS` env var was never read, sunnypilot carries no
  equivalent. Net change: less attack surface, no behavior change.
* **carstate TODOs** (`c9a1ed6`): `# TODO: Find brake pressure` and
  `# TODO: figure out positions` (wheel speeds FL/FR/RL/RR) were
  both validated as intentional non-features — `parse_wheel_speeds`
  only writes `vEgoRaw`, never per-wheel; `brake` is unused on the
  i6n path because op long is unavailable (`CANFD_NO_RADAR_DISABLE`
  → `CANFD_UNSUPPORTED_LONGITUDINAL_CAR`).
* **Investigation tools** (`66a93db`, `1506a9d`): 64 of the 83
  `tools/ioniq6n_*.py` one-off scripts were deleted (~9 kLOC). The
  18 survivors are either generic diagnostic tools or referenced
  from `tools/POST_6F2_AUDIT.md` /
  `tools/IONIQ6N_STEERING_MASTERPLAN.md` /
  `docs/route49-symptom-analysis.md` /
  `docs/dev-lessons-steering-feel.md`.

### Audit punch list — final disposition

The §3 P0/P1/P2 items, plus the post-6F2 additions, status:

| # | Item | Status |
|---|---|---|
| §3 P0 #1 | `cam_stale` 250 ms gate FP rate | ✅ 0 events across 12 routes |
| §3 P0 #2 | `fault_lfa` frequency | ✅ 0 mid-drive events |
| §3 P0 #3 | `error_boost × override_factor` interaction | ✅ no symptom in 6F2-A+ logs |
| §3 P0 #4 | R→D immediate gear release | ✅ tested via parking-lot logs |
| §3 P1 #5 | Magic number constants → named | ✅ done as part of Phase 6c-3 |
| §3 P1 #6 | LKAS_ALT BYTE28..31 fallback | ✅ 0x00, drivelog-verified |
| §3 P1 #7 | LKA_ASSIST × LKAS_ANGLE_ACTIVE consistency | ✅ verified on `c9a1ed6` sweep |
| §3 P2 #8 | Panda safety `MAX_LATERAL_ACCEL=3.6` | accepted (firmware-fixed) |
| §3 P2 #9 | HOD bypass risk | **REMOVED** (`c9a1ed6`) |
| post-6F2 P0 | Heavy-override mismatch p95 < 5° | ✅ measured 0.10° (decode bug) |
| post-6F2 P0 | sim re-verification of 6F2-A | superseded by drivelog measurement |
| post-6F2 P0 | LKAS_ALT byte7/byte13 ACI mismatch | **P2 → accepted** (ACIGain rate_dn residue, not an op bug, sunnypilot also untouched) |
| post-6F2 P0 | Hand-off lag investigation | **P0 → accepted** (driver continuously gripping = correct yield) |
| post-6F2 P1 | Same-route A/B baseline | ✅ done (0x2b/2c vs 0x2d/2e) |
| post-6F2 P1 | Highway 30+ min sustained data | ✅ partial (0x30 4.3 % @ 100+ kph), 0x31/0x35 sustained latActive |

### Items deferred per §3 P3 (still deferred)

10. Lane Change Timer overhaul.
11. DM-based gating tuning.
12. Lateral-offset features.

## 8. Next direction (forward-looking, no commitment)

Operational mode going forward is **observation + tooling**, not new
lateral-control changes:

1. **Continue chunked drivelog audit.** Each new ccnc-drivelog push
   gets a sweep + heavy-override + speed distribution + onroadEvents
   + CCNC_0x161 alert distribution, appended to
   `tools/POST_6F2_AUDIT.md` §1.A, §1.B, §1.C, … . The chunk-by-chunk
   format is settled.
2. **Investigate ALERTS_3 = 11 only if it escalates** — currently
   cosmetic. If a future log shows ALERTS_2 = `KEEP_HANDS_ON_RED`,
   ALERTS_5 = `GRASP_NOT_DETECTED_SPEED_LIMITED`, or any
   functional-limit alert > 100 frames, revisit the
   dual-publisher-fault tradeoff for `create_ccnc()` on HDA2-ALT.
3. **Capture missing scenarios** — the data set is still light on:
   * sustained highway (continuous 30+ min at 100+ kph)
   * deliberate parking-lot R↔D toggles for `was_in_reverse` check
   * rain / camera-occlusion to actually trip the `cam_stale` gate
   * dense stop-and-go to characterize `in_passthrough_relapse` at
     scale
4. **Investigation tooling cleanup is finished**. New analysis
   should extend `ioniq6n_full_drivelog_sweep.py` or
   `ioniq6n_phase7_sim.py` rather than spawn new one-off scripts;
   the second one-off-deletion sweep took ~12 minutes of human time
   to triage and we don't need to repeat that.
5. **Do not push lateral-control tuning changes** to the i6n branch
   without a concrete on-vehicle symptom reported by the user
   first. Every metric that originally motivated a change in this
   chapter (hand-off lag, sign-mismatch, heavy-override transition)
   turned out to be either a detection artifact (decode bug, hand-off
   classifier) or a deliberate yield to the driver. The deployed
   floor is hitting CAN quantization. Adding parameters now will
   over-fit to noise.
6. **If sunnypilot upstream lands a relevant change**, mirror it
   rather than re-deriving. Recent comparisons against
   `sunnypilot/opendbc:hkg-angle-steering-2025-hda1` and `:ccnc-port`
   confirmed the local fork is currently *ahead* of upstream on
   `ALERTS_3 ∈ {11, 12}` suppression for non-HDA2 CCNC and behind on
   nothing specific — worth a one-direction PR to upstream when time
   permits.
