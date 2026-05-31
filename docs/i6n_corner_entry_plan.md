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
