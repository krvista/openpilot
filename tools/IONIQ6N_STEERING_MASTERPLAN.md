# Ioniq 6 N (2026 ccNC) — Tesla-grade Steering Feel Master Plan

**Branch:** `claude/steering-feel-masterplan-BIIQD`
**Baseline head:** `2f921df` (objective re-analyzer) on top of `be0bebf` (panda rebuild) on top of `81c451f` (residual tick + takeover alert + I5 sim)
**Date:** 2026-04-15

---

## Context

Earlier sessions landed a working angle-control path for the Ioniq 6 N
(HDA2-ALT + CCNC), converged on a speed-indexed rate table, added
camera-mirrored ACI bytes, dual-threshold hysteresis, aci_gain ramp,
CCNC_0x161 op-only alert suppression, and rebuilt the panda firmware so
the TX whitelist actually honors the new 0x161/0x162 path.

A prior plan claimed we were already at **MAE 0.11° vs Tesla 0.07°** and
only needed Kalman filter / jerk limiter / MPC to close the remaining
0.04° gap. The fresh objective re-analysis (`tools/ioniq6n_reanalysis.py`,
1,242,341 frames across 214 segments, routes 28–2d) **contradicts that
baseline**:

* op-mode MAE is actually **0.42–2.24°** depending on speed
* the **same car**’s stock LFA achieves **0.02–0.20° MAE** on the same
  routes — i.e. the hardware is already Tesla-grade; openpilot’s desired
  angle generation is the bottleneck
* a 35–39 Hz desired-angle oscillation persists across every build,
  including `5540440` which claimed to fix a 40 Hz oscillation
* parking clips the rate limiter on 11.2% of frames
* **no drivelog exists post-81c451f**, so the hysteresis / aci_gain_ramp
  / op_driving alert suppression are deployed but unverified

The goal of this plan is therefore recalibrated: **match stock LFA
accuracy first, then exceed Tesla using openpilot-only advantages**
(lookahead, cross-wind feedforward, model-predictive shaping).

---

## Measured current state (routes 28–2d, 1.24M frames)

### Mode mix per route

| Route | commit  | segs | op%  | lfa_pass% | manual% |
|-------|---------|------|------|-----------|---------|
| 28    | 77adfed | 37   | 16.7 | 50.8      | 32.2    |
| 29    | 8ac35e4 | 15   | 20.8 | 35.6      | 38.9    |
| 2a    | 5540440 | 33   | 17.5 | 47.7      | 26.2    |
| 2b    | a816b9c | 51   | 35.9 | 43.4      | 16.0    |
| 2c    | a816b9c | 35   | 30.1 | 45.3      | 24.6    |
| 2d    | 611a505 | 37   | 14.0 | 58.7      | 25.6    |

### Tracking accuracy (|desired − actual|, degrees)

| Bucket  | op MAE   | op p95 | lfa_pass MAE | op / lfa |
|---------|----------|--------|--------------|----------|
| parking | **2.24** | 7.23   | 0.20         | 11.2×    |
| 20 km/h | 1.88     | 6.19   | 1.78         | 1.1×     |
| 30 km/h | 1.24     | 4.08   | 0.20         | 6.2×     |
| 40 km/h | 1.04     | 3.67   | 0.20         | 5.2×     |
| 50 km/h | 1.05     | 3.39   | 0.20         | 5.3×     |
| 60–70   | 0.83     | 2.75   | 0.03         | 28×      |
| 80–90   | 0.42     | 1.40   | 0.02         | 21×      |

### Persistent symptoms

| Symptom                | Measurement                                                 |
|------------------------|-------------------------------------------------------------|
| Low-speed tick         | 2–5 km/h \|Δdesired\|>0.3° rate 12.7–19.4% per route (0000002d: 16.1%) |
| 37 Hz oscillation      | op direction-change rate 35.4–39.3 Hz on every route        |
| Rate-limit clipping    | 11.2% at parking, 1–5% elsewhere                            |
| Alerts / ACIGain decode | heuristic returned 0% / 0.533 — **requires DBC decoder**    |

### Data gaps

* **parking (0–10 km/h) op**: only 1.6 min of op across all 214 segs
* **100+ km/h op**: 0.1 min
* **post-81c451f**: 0 segs

---

## Problems identified (priority)

| #  | Problem                                                        | Evidence                    | Impact                  |
|----|----------------------------------------------------------------|-----------------------------|-------------------------|
| P1 | op desired-angle amplifies planner curvature noise             | 37 Hz; op/lfa MAE ratio 5–28× | Root cause of feel      |
| P2 | low-speed rate-limit undersized                                 | parking 11.2% clip          | Low-speed responsiveness |
| P3 | Feedforward-only → no steady-state error removal               | op MAE 0.4–2.2°             | All speeds               |
| P4 | Low-speed & highway op data deficit                            | 1.6 / 0.1 min               | Can’t tune what we can’t see |
| P5 | 81c451f not present in any drivelog                            | all routes ≤ 611a505        | Deployment unverified    |
| P6 | Alerts / ACIGain decode missing                                 | heuristic returned zeros    | Can’t quantify fixes     |

---

## Plan

### Stage 0 — Measurement infrastructure (0.5 week, no driving)

Goal: every metric we care about measured with DBC-grade accuracy.

* Rewrite `tools/ioniq6n_reanalysis.py` to use opendbc’s `CANParser`
  against the HDA2-ALT DBC for bus-2 messages.
* Accurately decode for each 10 ms frame:
  * `LKAS_ALT` 0x110 — `ADAS_StrAnglReqVal`, `ADAS_ACIAnglTqRedcGainVal`,
    `LKAS_ANGLE_ACTIVE`, `LKA_ASSIST`
  * `CCNC_0x161` — `ALERTS_2`, `ALERTS_3`, `ALERTS_5`, `SOUNDS_2`,
    `SOUNDS_4`, `LFA_ICON`
* Verify the stage-0 decoder on ≥1 segment by cross-checking against a
  known camera command (manual / lfa_passthrough).
* Re-run the full 214-seg analysis with the new decoder; update this
  plan with the real ALERTS / ACIGain numbers replacing the current
  “heuristic = 0%” placeholders.
* Deliverable: an `ioniq6n_reanalysis.py` that produces a
  one-page objective report usable as the canonical current-state
  snapshot; an updated table under “Measured current state”.

### Stage 1 — Deploy-verification drive (1 week, user)

Goal: confirm hysteresis / aci_gain_ramp / alert-suppression actually
work on the car now that be0bebf ships the corresponding panda safety.

User drives with HEAD=be0bebf (or any commit ≥81c451f). Target data:

* 20–30 min op on a route similar to 0000002d
* **20 min parking op @ 0–10 km/h** (Priority 1 #9–10 from old plan —
  still missing)
* **30 min highway op @ 100–110 km/h** (Priority 1 #6–8 — still missing)
* ≥10 min stock-LFA reference at each of parking and highway (cruise OFF)

Verification gates (on the new logs, via Stage 0 decoder):

| Gate | Old value | Target |
|------|-----------|--------|
| 2–5 km/h \|Δdesired\|>0.3° rate | 16.1% | **<3%** |
| op direction-change rate (Hz) | 37.9 | still ≤37 ⇒ Stage 2 required; else declare fixed |
| ALERTS_3∈{11,12} in op mode | (to be measured in S0) | **<0.1%** |
| ALERTS_3 in manual / lfa_passthrough | (S0) | **unchanged** (preservation check) |

### Stage 2 — Planner-side jerk LP filter (1–2 weeks, code)

Rationale: the 37 Hz oscillation is present in every build including the
one that claimed to fix it, so its source is not the removed PID. Most
likely source is the modelV2 `desiredCurvature` output itself. Tesla’s
`apply_steer_angle_limits_vm` implicitly low-passes via lateral-jerk
constraints; we currently don’t.

* Add a 1st-order low-pass on `desired_curvature` in
  `selfdrive/controls/controlsd.py` (or a jerk-limited slew rather than
  pure LP — evaluate both in sim).
* Tuning starting point: τ ≈ 200 ms (fc ≈ 0.8 Hz) — A/B vs τ = 120 ms
  and a jerk-limited rate based on MAX_LATERAL_JERK = 3.0 m/s³.
* Expected effect (from `tools/ioniq6n_rate_comparison.py` simulations
  prior to on-car):
  * op direction-change rate: 37 Hz → <5 Hz
  * op MAE 60–90 km/h: 0.42° → ≤0.10°
* Risk: phase lag at curve entry — compensate with a 1-step lookahead
  FF using `modelV2.action.desiredCurvatures[future_index]`.

### Stage 3 — VM-based rate limiter (1 week, code)

Switch from the hand-tuned speed table to `apply_steer_angle_limits_vm`:

* `MAX_LATERAL_ACCEL = 3.0 m/s²` (ISO 11270)
* `MAX_LATERAL_JERK = 3.0 m/s³`
* `MAX_ANGLE_RATE = 4.0°/20ms` (200°/s, 80% of Tesla’s 250°/s)
* STEER_ANGLE_MAX unchanged at 176.7°

Expected: parking clip 11.2% → <2%; faster low-speed transients
without a speed-specific hand tune.

### Stage 4 — Camera-referenced feedforward (2–3 weeks, code + data)

The key plan delta vs prior sessions. Stock LFA achieves 0.02–0.20° MAE
on this exact car/route combination because its reference comes from
the camera, not from a controlsd-derived curvature. We can use that
reference:

```
desired_angle = α(v) · cam_angle
              + (1 − α(v)) · op_angle_from_curvature
              + β · integral(angle_error)
```

* α(v): 0.7 at parking → 0.3 at highway (camera owns low speed, op owns
  lane centering at highway where camera drifts toward rightmost lane)
* β: small steady-state error correction, enabled only after Stage 2
  suppresses high-frequency content (safe regime for a light I term)
* Safety: cam_angle is already within assist bounds; rate limiter from
  Stage 3 caps its delta

Expected: op MAE converges to stock-LFA values per bucket, i.e. below
the Tesla baseline in every measured regime.

### Stage 5 — Low-gain feedback PID (1 week, code)

Re-introduce an angle-error PID but only:

* after Stage 2 (so the input is smooth)
* only below 10 km/h (eliminates the 2.24° parking MAE)
* gains P=0.03, I=0.005, with a 2 Hz LP and ±1.5° anti-windup
* resets on disengage

### Stage 6 — Integration, A/B, release (1 week)

* Re-run Stage 0 analyzer to confirm all targets met
* Blind subjective A/B (S2+S3 vs S2+S3+S4 vs S2+S3+S4+S5)
* Freeze parameters; write release notes

---

## Target performance (post Stage 4)

| Bucket  | Current op MAE | Tesla (ref) | Target | Rationale |
|---------|----------------|-------------|--------|-----------|
| parking | 2.24°          | ~0.15°*     | 0.20°  | Match stock LFA (same HW) |
| 20 km/h | 1.88°          | ~0.12°*     | 0.18°  | Match stock LFA            |
| 30–50   | 1.05°          | ~0.10°*     | 0.15°  | Exceed Tesla               |
| 60–70   | 0.83°          | ~0.05°      | 0.05°  | Match Tesla                |
| 80–90   | 0.42°          | 0.02°       | 0.02°  | Match Tesla                |

\* Tesla low-speed numbers are extrapolations from the 90 km/h datapoint
in the existing comparison; update after Stage 1 logs land.

---

## Critical files

| Area                  | File                                                         |
|-----------------------|--------------------------------------------------------------|
| Rate limit & authority | `opendbc_repo/opendbc/car/hyundai/carcontroller.py`         |
| LKAS_ALT packer / CCNC | `opendbc_repo/opendbc/car/hyundai/hyundaicanfd.py`          |
| Flags & param wiring   | `opendbc_repo/opendbc/car/hyundai/interface.py`             |
| Bus-2 capture          | `opendbc_repo/opendbc/car/hyundai/carstate.py`              |
| Rate table             | `opendbc_repo/opendbc/car/hyundai/values.py`                |
| Safety TX whitelist    | `opendbc_repo/opendbc/safety/modes/hyundai_canfd.h`         |
| LatControlAngle        | `selfdrive/controls/lib/latcontrol_angle.py`                |
| Planner → curvature    | `selfdrive/controls/controlsd.py`                           |
| VM rate helper         | `opendbc_repo/opendbc/car/lateral.py` (`apply_steer_angle_limits_vm`) |
| Objective analyzer     | `tools/ioniq6n_reanalysis.py`                               |
| Rate variant comparison| `tools/ioniq6n_rate_comparison.py`                          |
| Torque cross-sim       | `tools/ioniq5_torque_sim.py`                                |
| CCNC op-vs-LFA report  | `tools/ioniq6n_op_vs_lfa_analysis.py`                       |

---

## Verification

Each stage completes only when the Stage-0 analyzer re-run produces:

* numerical targets for that stage met on the latest drivelog, and
* no regression on any other metric beyond a 10% tolerance

The Stage-0 analyzer is the single source of truth for numbers in this
plan; subjective feedback is recorded as a separate note but does not
gate stage promotion.
