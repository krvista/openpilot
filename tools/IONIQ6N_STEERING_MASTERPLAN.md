# Ioniq 6 N (2026 ccNC) — Tesla-grade Steering Feel Master Plan

**Branch:** `claude/steering-feel-masterplan-BIIQD`
**Baseline head:** `7933664` (Stage 0 DBC-accurate analyzer) on top of `59cd09a` (plan v1), `2f921df` (objective re-analyzer), `be0bebf` (panda rebuild), `81c451f` (residual tick + alerts + I5 sim)
**Date:** 2026-04-15

---

## Platform scope

This plan targets the **HDA2-ALT + CCNC angle-control platform** — any
Hyundai/Kia/Genesis car whose `CarParams.flags` end up containing both
`HyundaiFlags.CCNC` **and** `HyundaiFlags.CANFD_LKA_STEERING_ALT`. The
second flag is auto-detected in `interface.py` from the presence of
LKAS_ALT (0x110) on the camera bus at fingerprint time; the first is
declared statically per car in `values.py`. Members today:

* `HYUNDAI_IONIQ_6_N` (2026) — the first and only measured member; all
  tuning defaults in this plan derive from its 1.24 M-frame drivelog.

Future members (any 2025+ MY Hyundai/Kia/Genesis car with the same ADAS
architecture — LKAS_ALT on camera bus, ADAS_StrAnglReqVal commanding
MDPS) will **inherit the entire behaviour automatically** once their
fingerprint lands with `CCNC` declared. No platform-logic code changes
needed; per-car tuning overrides (rate table, α, etc.) remain optional
in `CarControllerParams.__init__` and the module-level CAMREF_*
constants.

Logic entry points use the `is_ccnc_angle_platform(CP.flags)` helper in
`opendbc_repo/opendbc/car/hyundai/carcontroller.py`.

## Context

Earlier sessions landed a working angle-control path for this platform
(first on the Ioniq 6 N), converged on a speed-indexed rate table,
added camera-mirrored ACI bytes, dual-threshold hysteresis, aci_gain
ramp, CCNC_0x161 op-only alert suppression, and rebuilt the panda
firmware so the TX whitelist actually honors the new 0x161/0x162 path.

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

---

## Safety / recognition change checklist

Two distinct incidents so far have surfaced as the same user-facing alert
("Unknown Vehicle Variant" + `carState.valid=False`), with completely
different root causes. To prevent a third, every change under
`opendbc_repo/opendbc/safety/` or touching Ioniq 6 N / CCNC fingerprinting
must pass this checklist **before** committing.

### 1. Prior fixes that must stay intact

The foundations below were landed by session
`session_01DhaSf2nVWu1Ar3ZXYJ7SLx` (commits `772d52b`, `bf5bc8e`,
`b347e01`, `0e572eb`, `bdb1ce0`, `c405c73`, `2c7474a`, `323c796`).
Do not remove or weaken any of them without replacing the equivalent
behaviour:

* `ignore_counter = true` **and** `max_counter = 0U` on 0x35, 0x100,
  0x105 — required because CCNC platforms use a +2 counter increment.
  Either flag alone is insufficient: `safety.h` checks `max_counter > 0`
  before consulting `ignore_counter`.
* `HyundaiFlags.CANFD_ALT_BUTTONS` + `HYUNDAI_PARAM_CANFD_ALT_BUTTONS`
  (value 32) for Ioniq 6 N — uses 0x1aa instead of 0x1cf.
* `HyundaiFlags.CANFD_ALT_DOORS_BLINKERS` for Ioniq 6 N — 0x20a (seatbelt),
  0x400 (blinkers), 0x3e2 (doors). Dropping this makes `seatbeltUnlatched`
  stuck True and blocks cruise engagement.
* `HyundaiFlags.CCNC` safety param flag (1024) for CCNC-equipped cars,
  including HDA2-ALT Ioniq 6 N — enables CCNC TX whitelist entries.

### 2. Before editing a TX whitelist

Run `tools/ioniq6n_reanalysis_dbc.py` and inspect the bus-counts report
for the address you plan to add. Then:

| Observation in drivelog                                       | `check_relay` |
|---------------------------------------------------------------|---------------|
| Msg appears on a **remote** bus (e.g. camera bus 2) and panda forwards it to the target bus where op wants exclusive TX | **`true`** — panda blocks the forward and stock_ecu_check sanity-tests that the relay works |
| Msg is **natively published on the same bus we intend to TX on** (e.g. HDA2-ALT 0x161 on bus 1 from a gateway ECU) | **`false`** — we cannot silence a native source; setting `true` triggers `stock_ecu_check` → `relay_malfunction` within 1 s of boot |
| Msg has no source except op (new synthetic address)           | **`false`** (no stock source to guard against)                   |

Historical precedent: non-HDA2 CCNC uses `check_relay = true` because
0x161/0x162 are camera-sourced on bus 2 and forwarded. HDA2-ALT
**requires `check_relay = false`** because the same addresses are native
on bus 1 and cannot be forwarded-blocked. See commit `c6a33de` /
`51a38a4` for the incident and fix.

### 3. RX check additions

Any new RX entry for a CCNC car (Ioniq 5/6 N platform) must default to:

* `ignore_counter = true` **and** `max_counter = 0U` if the counter
  increments by a non-1 step, otherwise the normal increment is assumed
* `ignore_quality_flag = true` unless the message actually carries a
  valid quality flag
* `ignore_alive = true` for messages whose startup latency would cause
  canValid=False during the boot window

### 4. Firmware rebuild rule

Any change to `opendbc_repo/opendbc/safety/modes/**.h` or
`opendbc/safety/safety.h` **requires a panda firmware rebuild**. User-
space-only deploys leave stale firmware on the device. Procedure:

```bash
cd /home/user/openpilot/panda
source .venv/bin/activate
uv pip install -e ../opendbc_repo --no-deps   # once, so opendbc points
                                              # at the local checkout
scons --minimal -c && scons --minimal -j2
# then commit gitversion.h, version, panda_h7.bin.signed,
# panda_h7/main.bin, panda_h7/main.elf
```

Verify in commit: `panda/board/obj/version` matches your HEAD's short
hash, and `arm-none-eabi-nm panda/board/obj/panda_h7/main.elf | grep
HYUNDAI_CANFD_LKA_STEERING_ALT_CCNC` returns exactly one symbol.

### 5. Commit discipline

* Commit messages must cite the specific measurement they rely on
  (bus-counts from reanalysis, per-bucket tick_frac from
  `ioniq6n_tick_comparison.py`, etc.) — no bare "expected" claims.
* When a change is purely code (no firmware), say so explicitly; when
  firmware is required, include the build hash (`DEV-<short>-DEBUG`)
  and note that panda reflash is required for deployment.
* Session IDs — `session_01DhaSf2nVWu1Ar3ZXYJ7SLx` (foundations) and
  `session_015hMh1GdaYfs1QjUywn2Nj9` (steering feel) — are preserved as
  commit trailers for ancestry traceability.

---

## Appendix H: Hands-on-detection (HOD) suppression feasibility

Goal explored: make MADS (openpilot lateral) drive without ever
triggering the "please hold wheel" warning, mirroring Tesla's behaviour.

### Findings (route 00000030, 5 segments, 240 s, `tools/ioniq6n_find_hod.py`)

1. **DBC's `HOD_FD_01_100ms` (0x2AF / 687) is NOT present on any bus.**
   `opendbc/dbc/hyundai_canfd_generated.dbc` captures an older K-platform
   HOD address; Ioniq 6 N (2026 HDA2-ALT + CCNC) publishes its capacitive
   HOD on a different, undocumented ID. User confirmed the sensor exists
   ("핸들에 손만 대면 경고가 꺼져").

2. **Top undocumented HOD candidates** (bus 1, rate ~20 Hz, non-counter
   slow-toggle bit signatures matched against 4 min of intermittent
   hand grip events):

   | Addr   | DLC | Hz   | Slow bits (transition count)              |
   |--------|-----|------|-------------------------------------------|
   | 0x1b5  | 32  | 20.7 | byte 13 bit 4–7 : 2–12,  byte 14 bit 0–2 : 2 |
   | 0x1ba  | 24  | 20.8 | byte 18 bit 0/2/5 : 8,   byte 19 bit 3 : 8, byte 8 bit 0 : 8 |
   | 0x1e5  | 16  | 20.8 | byte 3 bit 7 : 8,        byte 4 bit 1 : 8, byte 10–11 bit 0 : 8–10 |
   | 0x3e5  | 24  | 26.3 | byte 13 bit 5 : 2,       byte 11 bit 2/3 : 8 |
   | 0x175  | 24  | 52.1 | byte 16 bit 0/1/6/7 : 8,  byte 9 bit 0 : 12 (hidden-in-known) |

   (Counter bits following the exponential-halving pattern
   156 → 78 → 39 → 20 → 10 were filtered out as carriers, not HOD.)

3. **Dual-publisher trap (identical to 0x161 incident).** Every
   candidate lives on bus 1 (E-CAN) as a *native* publisher — not a
   camera-forwarded bus 2 address. panda TX on bus 1 therefore
   interleaves with the real sensor frames; CCNC/MDPS will detect the
   inconsistency and surface the ADAS flicker we already fought once.
   check_relay cannot block a native bus 1 publisher.

### Conclusion

Naive HOD spoofing is infeasible on this platform for the same reason
CCNC_0x161 spoofing was: the receiver sees both our frame and the real
frame on the same bus and treats the pair as inconsistent. To actually
suppress hands-on warnings without flicker, one of the following is
required, both out of scope for the current branch:

* **UDS CommunicationControl(0x28) disableRxAndTx on the source ECU**
  (likely a steering-column capacitive sensor module), then spoof. Needs
  module discovery (no 7xx diag IDs observed on bus 1 in this route),
  carries legal + safety-gateway risk, and fights the factory torque
  supervisor.
* **Labelled grip/release drivelog** — same `ioniq6n_find_hod.py`
  analysis but correlated against user-noted grip timestamps — would
  disambiguate the 3 top candidates (0x1b5 / 0x1ba / 0x1e5) from
  coincidental slow signals. Without it, the ranking is pattern-based,
  not empirical.

### Decision

Option 2 (HOD spoof) joins option 1 (CCNC_0x161 spoof) in the
"architecturally blocked" bucket. Masterplan continues with option 3:
accept the factory hands-on timer and focus the remaining effort on
making it unobtrusive (aci_gain, camref tracking, residual-tick cadence)
rather than suppressing it at the CAN layer.

Tools: `tools/ioniq6n_hod_probe.py` (0x2AF absence proof),
`tools/ioniq6n_find_hod.py` (full bus-1 enumeration + bit-transition
HOD candidate ranking). Both preserved under version control for any
future re-attempt with a labelled drivelog.

### Addendum (route 00000031--85ea5c34a8, labelled grip/release log)

The labelled drivelog was captured as promised: 8 strong-grip + 5 light-
touch cycles (user-reported) at the wheel, with the car stationary and
engine on, followed by a driving lap. Tools:

* `tools/ioniq6n_hod_correlate.py` — phase-split (stationary vs driving)
  transition scorer. Correctly rules OUT 0x1b5/0x1ba/0x1e5: their "slow
  bits" had Phase-A/Phase-B transition ratios of exactly 11.5 = log-time
  ratio, i.e. pure CAN-counter noise proportional to duration.
* `tools/ioniq6n_hod_decode.py` — byte-level time-series dump of next
  generation candidates (0x35c / 0x3e3 / 0x35a / 0x35b): they form an
  *event log* family where byte[2] is a monotonic event counter and
  byte[0..1] are a cryptographic MAC. They signal HOD transitions but
  the state value is not directly readable from them.
* `tools/ioniq6n_hod_align.py` — uses 0x35c byte[2] as ground-truth
  event timeline, scores every bus-1 byte by (# low-cardinality value
  transitions aligned with events / total). Top score: **0.79 for
  0x208 byte 10, 5 distinct values {0, 1, 2, 3, 4}**.
* `tools/ioniq6n_hod_0x208_inspect.py` — frame-structure autopsy.

### FOUND: 0x208 byte 10 is HOD_Dir_Status

Empirically confirmed by matching the time-series against the labelled
grip/release timeline:

```
t=  0.06s  byte10=0  HANDS_OFF  (initial, hands off wheel)
t= 66.26s  byte10=1  TOUCH_SOFT  ← first strong grip begins
t= 66.46s  byte10=3  GRIP_SOFT
t= 66.66s  byte10=4  GRIP_STRONG  (hand fully on)
t= 88.66s  byte10=3  GRIP_SOFT   (release begins, ~22s grip)
t= 88.85s  byte10=1  TOUCH_SOFT
t= 89.26s  byte10=0  HANDS_OFF
... 12 more cycles with matching pattern, ±0.5 s event alignment.
```

Complete 0x208 frame structure (bus 1, 10.4 Hz, DLC=16):

| Byte  | Role                            | Values observed           |
|-------|---------------------------------|---------------------------|
| 0-1   | CRC / MAC (checksum)            | 256 distinct, pseudo-rand |
| 2     | Counter (+2 each frame, wraps at 256, bit 0 always 0 → effectively 7-bit mod-128) | 128 distinct |
| 3-9   | Reserved                        | all 0x00                  |
| **10** | **HOD_Dir_Status (this is it)** | **{0, 1, 2, 3, 4}**      |
| 11    | Enable flag                     | 0x01 (always)             |
| 12    | Raw capacitive pressure (≈)     | 0x00–0x3d, tracks state   |
| 13    | Raw capacitive area (≈)         | 0x00–0x2d, tracks state   |
| 14    | Enable/valid flag               | 0x01 (always)             |
| 15    | Reserved                        | 0x00 (always)             |

Semantic mapping (matches DBC `HOD_FD_01_100ms`/`HOD_Dir_Status`):

* 0 = HANDS_OFF      (no touch)
* 1 = TOUCH_SOFT     (light contact, ~fingertip)
* 2 = TOUCH_STRONG   (seen only once in 920 s — essentially transitional)
* 3 = GRIP_SOFT      (palm contact, loose hold)
* 4 = GRIP_STRONG    (firm two-hand grip)

Per-state sample payload (bytes 0-15 hex):
```
state 0:  XX XX YY 00 00 00 00 00 00 00 00 01 00 00 01 00
state 1:  XX XX YY 00 00 00 00 00 00 00 01 01 08 06 01 00
state 3:  XX XX YY 00 00 00 00 00 00 00 03 01 2b 17 01 00
state 4:  XX XX YY 00 00 00 00 00 00 00 04 01 33 28 01 00
```
(XX = CRC bytes, YY = counter byte)

### Revised spoofing feasibility

| Factor | Status |
|--------|--------|
| Address found | ✅ 0x208 byte 10 |
| Counter protection | ⚠️ byte 2 +2/frame — trivial to forge |
| CRC protection | ❌ byte 0-1 — algorithm unknown, must be reverse-engineered |
| Native bus-1 publisher | ❌ same dual-publisher trap as 0x161 |
| Consumer uniqueness | 🤔 *possibly* single-consumer (hands-off timer only, unlike 0x161 which is consumed by both HUD renderer and LFA alerts) — would decide whether spoofing flickers |

0x208 is materially different from 0x161/0x162 in two ways that matter:

1. **Simpler consumer model.** 0x161 was a pan-feature alert carrier
   (ops, hands-off, lane alerts, ACC alerts) consumed by multiple CCNC
   state machines; spoofing one field desynchronised another. 0x208
   appears to feed only the hands-off supervisor — a single consumer
   whose "latest frame wins" semantics would let a 2x-rate TX dominate.
2. **CRC exists but is content-specific.** This means a naive replay
   attack fails (stale counter rejected), but *if* the CRC is the
   standard Hyundai CRC16-E2E (as used elsewhere in the safety stack),
   an openpilot-side implementation is tractable.

### Next steps (not executed on this branch)

1. **Reverse-engineer 0x208 CRC (byte 0-1).** Likely CRC16-AUTOSAR /
   CRC16-E2E over (data_id || counter || payload). Brute-force the
   data_id against the 4593 captured frames. If it matches Hyundai's
   standard profile, done.
2. **Test single-consumer hypothesis.** Bench-TX 0x208 with
   byte10=4, byte12=0x33, byte13=0x28 at 20 Hz (2× factory rate) on a
   known-valid CRC. Watch for CCNC flicker the way 0x161 spoof flickered.
   If stable, MADS-without-HOD-warning is unlocked.
3. **Escalate to UDS CommunicationControl** only if (1) and (2) fail.

### Decision (updated)

Option 2 is **no longer "architecturally blocked"** — it is "gated on
CRC reverse-engineering + one bench test". The core unknown has narrowed
from "which message?" to "which CRC?". This moves option 2 from the
infeasible bucket to the "tractable but out-of-scope for current branch"
bucket. Masterplan still proceeds on option 3 for this branch; option 2
is now a concrete follow-on project with a defined next step.

### Update: CRC solved, option 2 implemented as opt-in (commits ceee9b9 + 3a2262c)

The CRC reverse-engineering problem turned out to be already solved:
`opendbc/car/hyundai/hyundaicanfd.py::hkg_can_fd_checksum` (CRC16-XMODEM
+ address mix + DLC-16 XOR 0x041D) validates **4593 / 4593** captured
0x208 frames exactly (see `tools/ioniq6n_verify_0x208_crc.py`). The
Ioniq 6 N HOD register uses the standard Hyundai CAN FD profile — no
per-address data_id, no secure key, no custom polynomial.

Implementation landed as an opt-in bypass:

* `hyundaicanfd.create_hod_bypass(bus, counter)` — synthesizes a valid
  0x208 frame announcing `GRIP_STRONG` with correct CRC.
* `carcontroller.py` — emits at 10 Hz (matching factory 10.4 Hz) on
  E-CAN when `os.environ["HOD_BYPASS"] == "1"` AND on the HDA2-ALT +
  CCNC platform AND `CC.enabled`.
* `hyundai_canfd.h` — adds 0x208 to the HDA2-ALT CCNC TX whitelist
  with `check_relay = false` (required: native E-CAN publisher).
* Panda firmware rebuilt at `DEV-ceee9b98-DEBUG`.

The feature is **dormant by default.** `HOD_BYPASS` is not exported in
any launch script and is not persisted to Params; rollback is literally
`unset HOD_BYPASS` + restart.

---

## Appendix I: Engagement mode matrix (what each commit actually touches)

Recurring question: "does the latest HOD / CCNC work change who controls
lateral and longitudinal in each ACC/LFA combination?" Answer: **no.**
None of the commits on this branch alter engagement logic. This
appendix is the audit trail.

### Mode semantics (platform = Ioniq 6 N HDA2-ALT + CCNC, unchanged)

| User setting          | Longitudinal | Lateral               |
|-----------------------|--------------|-----------------------|
| LFA only, MADS off    | (none)       | factory LFA (camera ECU native) |
| LFA only, MADS on     | (none)       | **openpilot/MADS** via `LKAS_ALT` + `apply_angle` |
| ACC + LFA, MADS off   | factory SCC  | factory LFA           |
| ACC + LFA, MADS on    | factory SCC  | **openpilot/MADS**    |

`CP.openpilotLongitudinalControl = False` on this platform, so
longitudinal is **always** factory SCC. openpilot never commands accel.
Lateral control transfers to openpilot iff `CC.latActive = True`, which
MADS drives directly (independent of ACC).

### Per-commit engagement impact

| Commit | Files touched | Engagement impact |
|--------|---------------|-------------------|
| `4d59c6a` "Unknown Vehicle Variant" + flicker fix | `carstate.py` RX-capture gate + `carcontroller.py` CCNC gate + `hyundai_canfd.h` TX whitelist | ❌ none (RX-side VLDict auto-registration bug; TX-side pulled 0x161/0x162 off the wire) |
| `74d9303` panda rebuild | firmware binaries | ❌ none |
| `b3d267e` / `5e01681` / `b366862` HOD discovery | `tools/*.py` | ❌ none (analysis only) |
| `ceee9b9` HOD bypass implementation | `hyundaicanfd.py` + `carcontroller.py` + `hyundai_canfd.h` | ⚠️ ONLY when `HOD_BYPASS=1` AND `CC.enabled`. One additional TX (0x208). Does not change who holds lateral or longitudinal; only suppresses the factory hands-off warning. In LFA-only mode (`CC.enabled=False`) the bypass is dormant. |
| `3a2262c` panda rebuild | firmware binaries | ❌ none |

### Side-effect to be aware of

After the 0x161/0x162 TX removal (`4d59c6a`), openpilot no longer
suppresses factory LFA alerts either. The cluster may surface "take
over steering" / hands-off chimes while MADS is actively driving.
This does not change *who controls lateral* (openpilot does, via
`LKAS_ALT`); it only changes *what the driver sees on the cluster*.
The `HOD_BYPASS=1` experimental path exists specifically to address
this surface-level issue without resurrecting the 0x161/0x162
dual-publisher flicker.

