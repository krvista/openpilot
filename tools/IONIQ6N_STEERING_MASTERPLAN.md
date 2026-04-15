# Ioniq 6 N (2026 ccNC) — Tesla-grade Steering Feel Master Plan

**Branch:** `claude/steering-feel-masterplan-BIIQD`
**Baseline head:** `7933664` (Stage 0 DBC-accurate analyzer) on top of `59cd09a` (plan v1), `2f921df` (objective re-analyzer), `be0bebf` (panda rebuild), `81c451f` (residual tick + alerts + I5 sim)
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
0.04° gap. The objective re-analysis (`tools/ioniq6n_reanalysis.py`,
`tools/ioniq6n_reanalysis_dbc.py`, 1,242,341 frames across 214 segments,
routes 28–2d) **contradicts that baseline** and surfaces several findings
that were invisible to the previous heuristic analyzer.

The goal of this plan is therefore recalibrated: **match stock LFA
accuracy first, then exceed Tesla using openpilot-only advantages**
(lookahead, camera-trust adaptation, model-based shaping).

---

## Measured current state (routes 28–2d, 1.24M frames, DBC-accurate)

### Mode mix per route

| Route | commit  | segs | op%  | lfa_pass% | manual% |
|-------|---------|------|------|-----------|---------|
| 28    | 77adfed | 37   | 16.7 | 50.8      | 32.2    |
| 29    | 8ac35e4 | 15   | 20.8 | 35.6      | 38.9    |
| 2a    | 5540440 | 33   | 17.5 | 47.7      | 26.2    |
| 2b    | a816b9c | 51   | 35.9 | 43.4      | 16.0    |
| 2c    | a816b9c | 35   | 30.1 | 45.3      | 24.6    |
| 2d    | 611a505 | 37   | 14.0 | 58.7      | 25.6    |

### Three-way tracking error (op mode, |Δ|=deg)

| Bucket  | err_curv (LatControlAngle) | err_op (actual TX) | **err_camref (camera advisory)** |
|---------|----------------------------|--------------------|----------------------------------|
| parking | 2.24                       | 1.94               | **0.31**                         |
| 20 km/h | 1.88                       | 1.60               | **0.30**                         |
| 30 km/h | 1.24                       | 1.13               | **0.25**                         |
| 40 km/h | 1.04                       | 0.90               | **0.27**                         |
| 50 km/h | 1.05                       | 0.90               | **0.27**                         |
| 60–70   | 0.83                       | 0.70               | **0.26**                         |
| 80–90   | 0.42                       | 0.37               | **0.20**                         |

The **camera's `ADAS_StrAnglReqVal` advisory is 3–10× more accurate than
what op transmits**, across every speed bucket, on the same frames.
This is the Stage 4 rationale — *numerically verified*.

### Alerts (CCNC_0x161, DBC-decoded)

| Route | commit  | op ALERTS_2{1,2}% | op ALERTS_3{11,12}% |
|-------|---------|-------------------|---------------------|
| 28    | 77adfed | 0.00              | 11.74               |
| 29    | 8ac35e4 | 0.00              | 15.60               |
| 2a    | 5540440 | 1.23              | 14.04               |
| 2b    | a816b9c | 2.01              | 2.85                |
| 2c    | a816b9c | 0.25              | 3.72                |
| 2d    | 611a505 | **4.07**          | 1.78                |

611a505’s mid-speed rate boost incidentally reduced HDP alerts 14 % → 1.78 %,
but caused a spike in keep-hands alerts (4.07 %). 81c451f’s op-only suppression
is required to hit our < 0.1 % target.

### ACIGain — broken assumption

| Mode              | cam_aci p50/p95/max | op_aci p50/p95/max |
|-------------------|---------------------|--------------------|
| op (all buckets)  | **0.000 / 0.000 / 0.000** | 0.60 / 1.00 / 1.00 |
| lfa_passthrough   | **0.000 / 0.000 / 0.000** | varies             |

The **camera never commands a non-zero ACIGain**. The existing
`max(cam_aci_gain, authority * 0.6)` mirror logic in `hyundaicanfd.py`
therefore always resolves to *op's* gain, not a camera value. We are
keeping MDPS in a more authoritative state than stock LFA ever does —
a plausible contributor to the low-speed tick.

### LKAS_ANGLE_ACTIVE flips (op mode)

| Route | bucket   | flips/min |
|-------|----------|-----------|
| 2d    | 20 km/h  | **125.2** |
| 2b    | 20 km/h  | 47.2      |
| 2c    | 60–70    | 42.5      |
| 2d    | 60–70    | 50.1      |

Up to 125 flips/min at 20 km/h on the latest build — hysteresis from
81c451f directly targets this but is unverified because no drivelog
exists on that build yet.

### Persistent symptoms

| Symptom            | Measurement                                     |
|--------------------|-------------------------------------------------|
| Low-speed tick     | 2–5 km/h \|Δdesired\|>0.3° at 16.1 % (0000002d) |
| 37 Hz oscillation  | op direction-change rate 35.4–39.3 Hz every build |
| Rate-limit clip    | 11.2 % at parking, 1–5 % elsewhere              |

### Data gaps

* **parking (0–10 km/h) op**: only 1.6 min across 214 segs
* **100+ km/h op**: 0.1 min
* **post-81c451f**: 0 segs

---

## Problems (priority, post-Stage-0 revision)

| # | Problem                                                | Evidence                               | Impact                |
|---|--------------------------------------------------------|----------------------------------------|-----------------------|
| P1 | op's desired angle is derived from curvature, not from camera | err_curv 5-10× err_camref              | Root cause of all feel issues |
| P2 | op holds ACIGain 0.6–1.0 while camera holds 0          | cam_aci ≡ 0                            | Low-speed tension / tick  |
| P3 | Planner curvature contains 37 Hz noise                 | direction-change Hz persistent         | Hi-freq jitter        |
| P4 | Low-speed rate limit undersized                        | parking 11.2 % clip                    | Responsiveness        |
| P5 | 81c451f deployment unverified                          | all logs ≤ 611a505                     | Can't validate fixes  |
| P6 | No parking/highway op data                             | 1.6 min / 0.1 min                      | Can't tune extremes   |

---

## Plan (revised)

### ✅ Stage 0 — Measurement infrastructure   *(DONE, 7933664)*

Delivered `tools/ioniq6n_reanalysis_dbc.py` with proper CANParser;
verified alert/ACIGain/angle decoders on 1.24 M frames.

### Stage 1 — Post-81c451f drive-verification logs   *(user action, 1 week)*

Target data on be0bebf: 20–30 min op re-run of 0000002d, **20 min parking
op**, **30 min highway op @ 100–110 km/h**.

Gates (via Stage 0 analyzer):

| Metric                             | Pre-81c451f | Target |
|------------------------------------|-------------|--------|
| 2–5 km/h \|Δdesired\|>0.3 %        | 16.1        | < 3    |
| LKAS_ANGLE_ACTIVE flips / min      | 125 (20 km/h) | < 10 |
| op ALERTS_2{1,2} %                 | 4.07        | < 0.1  |
| op ALERTS_3{11,12} %               | 1.78        | < 0.1  |

### Stage 2 — Planner-side jerk LP filter   *(2 weeks, code)*

Address 37 Hz oscillation by low-passing `desired_curvature` in
`controlsd.py`. τ ≈ 200 ms starting point; A/B against jerk-limited slew.
Expected: direction-change Hz 37 → < 5; 60–90 km/h op MAE 0.42° → ≤ 0.10°.

### **Stage 2b — ACIGain camera-match   *(1 week, code, **NEW**)*

Remove the op-forced ACIGain in `hyundaicanfd.py`; mirror the camera's
actual value (≈ 0 always) with a small authority floor (e.g., 0.1) when
op is actively steering. Expected: low-speed MDPS tension reduced;
possible secondary reduction of 37 Hz osc if MDPS overreaction was the
source.

### Stage 3 — VM-based rate limiter   *(1 week, code)*

Switch from fixed speed table to `apply_steer_angle_limits_vm`:
`MAX_LATERAL_ACCEL = 3.0 m/s²`, `MAX_LATERAL_JERK = 3.0 m/s³`,
`MAX_ANGLE_RATE = 4.0 °/20 ms`. Parking clip 11.2 % → < 2 % expected.

### **Stage 4 — Camera-referenced feedforward   *(3 weeks, code + data + online)*

Use the camera's own `ADAS_StrAnglReqVal` (cam_MAE 0.20–0.31°) as the
primary reference instead of curvature-derived angles.

Design:

```
desired_angle = α(v, q) · cam_angle
              + (1 − α(v, q)) · op_curv_angle
              + β · I_error          (leaky, 0.3 Hz)

α(v, q): base α(v) ∈ [0.3, 0.8]  (offline sim-tuned)
         × quality multiplier q   (online-adapted from cam tracking RMSE)
β      : small steady-state bias removal, enabled only after S2
```

Three sub-phases:

* **4a** — *Offline default*. Grid-search α(v) on the 1.24 M frame corpus
  to minimize a weighted error objective (sim-tuned defaults, no online).
* **4b** — *Online camera-trust adaptation*. Rolling window of the camera
  tracking RMSE → quality multiplier q ∈ [0.2, 1.0] that scales α. A
  high-variance camera (construction zone, lane-marking occlusion) pulls
  the system back toward op's curvature plan.
* **4c** — *Low-bandwidth error integral*. After S2 suppresses high-freq
  content, introduce a leaky I (τ = 3 s, 0.3 Hz) on `actual − desired`
  to remove DC bias. Safe because input is already smoothed.

Expected: op MAE converges toward camera advisory baseline (0.20–0.31°),
which equals or beats Tesla (~0.05–0.10°) in every bucket.

### Stage 5 — ~~Low-gain feedback PID (low-speed only)~~ **→ Re-scope**

With S4's error integral, a separate PID may be unnecessary. Decision deferred
until S4 results are in.

### Stage 6 — Integration, A/B, release   *(1 week)*

Re-run Stage 0 analyzer, blind subjective A/B, freeze parameters.

---

## Target performance (post Stage 4)

| Bucket  | Current op MAE | Camera advisory MAE | Target | Gap vs Tesla ~0.07° |
|---------|----------------|---------------------|--------|---------------------|
| parking | 2.24°          | 0.31°               | **0.35°** | — (Tesla unmeasured here) |
| 20 km/h | 1.88°          | 0.30°               | **0.30°** | — |
| 30–50   | 1.05°          | 0.25°               | **0.25°** | better than baseline |
| 60–70   | 0.83°          | 0.26°               | **0.20°** | within 3× Tesla |
| 80–90   | 0.42°          | 0.20°               | **0.10°** | within 1.5× Tesla |

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
| Objective analyzer     | `tools/ioniq6n_reanalysis_dbc.py`                           |
| Rate variant comparison| `tools/ioniq6n_rate_comparison.py`                          |
| Torque cross-sim       | `tools/ioniq5_torque_sim.py`                                |

---

## Verification

Each stage completes only when the Stage 0 analyzer re-run produces:

* numerical targets for that stage met on the latest drivelog, and
* no regression on any other metric beyond a 10 % tolerance.

The Stage 0 analyzer is the single source of truth for numbers in this
plan; subjective feedback is recorded separately but does not gate stage
promotion.

