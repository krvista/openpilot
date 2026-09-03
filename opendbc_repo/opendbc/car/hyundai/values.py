import re
from dataclasses import dataclass, field

import numpy as np
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts, SupportType
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, p16

from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

Ecu = CarParams.Ecu


class CarControllerParams:
  ACCEL_MIN = -3.5 # m/s
  ACCEL_MAX = 2.0 # m/s

  # CCNC angle-control platform: VM-based jerk/accel limits.
  # The path activates automatically for any Hyundai/Kia car whose `flags`
  # contains BOTH `HyundaiFlags.CCNC` AND `HyundaiFlags.CANFD_LKA_STEERING_ALT`
  # — Ioniq 6 N 2026 is the first member. Limits below mirror panda safety
  # (`HYUNDAI_CANFD_ANGLE_STEERING_LIMITS` in hyundai_canfd.h).
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    360,
    ([], []),
    ([], []),
    MAX_LATERAL_ACCEL=3.0 + (9.81 * 0.06),  # ~3.59 m/s² (matches panda safety)
    MAX_LATERAL_JERK=3.0 + (9.81 * 0.06),   # ~3.59 m/s³ (matches panda safety)
    MAX_ANGLE_RATE=5.0,                       # deg/frame — sunnypilot default
  )

  # Phase 10a (low-speed grab fix): below ~30 km/h the VM jerk limit (∝ 1/v²) is
  # very loose, so the fixed MAX_ANGLE_RATE (5°/frame) is the only thing bounding
  # per-frame angle change. ccnc-drivelog 0x07-0x0a (162 seg) measured command
  # steps up to 4.1°/frame at ~20 km/h on override-recovery — 100% of the >2°/frame
  # steps were <30 km/h — felt as a sudden wheel "grab". Taper the per-frame step
  # down at low speed so op returns to lane-keep smoothly. Applied as an extra clamp
  # AFTER the VM limiter in carcontroller. Kill switch: set _V = [5.0, 5.0].
  MAX_ANGLE_RATE_LOWSPEED_BP = [2.8, 8.3]     # m/s (~10, ~30 km/h)
  MAX_ANGLE_RATE_LOWSPEED_V  = [2.5, 5.0]     # deg/frame (5.0 == MAX_ANGLE_RATE)

  # Low-speed angle smoothing (sp_smooth_angle): EMA on commanded angle where
  # alpha is interpolated from v_ego_raw. Strong smoothing at low speed
  # (alpha=0.05), no smoothing at/above 18 m/s. Mirrors sunnypilot reference.
  SMOOTHING_ANGLE_VEGO_MATRIX = [0, 8.5, 11, 13.8, 18]
  # Phase 6h-1: jitter absorption moved upstream into controlsd's speed-dependent
  # LP (+matched lead). The angle-domain EMA shrinks to a light linear filter:
  # re-sim on real 6g-1 desired-angle streams (0x40/0x42, op-active <40 km/h)
  # measured |out-in| p95 0.748->0.302 deg (-60%), |bias| 0.081->0.029 (-64%),
  # at only +4% 2-8 Hz RMS (recovered 3.2x upstream). Kill switch: restore
  # [0.05, 0.05, 0.15, 0.4, 1] / 0.4 / 1.0(LO)/4.0(HI).
  # (history: Phase 6c-3 heavy matrix [0.05, 0.05, 0.15, 0.4, 1] absorbed model
  # curvature jitter measured on drivelog 0000001f at 2.0-2.5 deg/frame.)
  SMOOTHING_ANGLE_ALPHA_MATRIX = [0.3, 0.3, 0.5, 0.7, 1]
  SMOOTHING_ANGLE_MAX_VEGO = SMOOTHING_ANGLE_VEGO_MATRIX[-1]
  # Phase 6g-1: make the EMA slew/maneuver-aware so it suppresses jitter WITHOUT
  # eating corner-entry response. ccnc-drivelog 0x40 seg5 (KST 07:27:43-49, ~40 km/h)
  # showed op going near-straight (commanded ~0.002 1/m vs lane ~0.0038) into a
  # left bend, running wide to the right line (clearance 1.4 m -> 0.77 m) until the
  # driver grabbed (+40°, ~1500 Nm). Root cause: this EMA (alpha 0.05-0.16 at city
  # speed) lags a building corner command with no lead compensation.
  # Fix: release the EMA toward alpha=1 as the gap |desired - apply_last| grows past
  # a jitter floor, so small (<=LO) oscillations keep the heavy low-speed smoothing
  # (저속 떨림 absorption preserved) while a real corner (>=HI) passes through.
  # Kill switch: set SMOOTHING_ANGLE_RELEASE_HI_DEG huge -> pure speed-EMA (pre-6g-1).
  SMOOTHING_ANGLE_RELEASE_LO_DEG = 1.0   # |gap| at/below this = jitter, keep speed-alpha
  SMOOTHING_ANGLE_RELEASE_HI_DEG = 1.0e6  # Phase 6h-1: release OFF. With alpha>=0.3
                                          # linear, the gap-release nonlinearity (the
                                          # alpha 0.05->1, ~14x gain jump that was the
                                          # transfer mechanism of the 0x42 seg4 "휙")
                                          # is obsolete. Re-enable: 4.0 (= 6g-2).
  # Phase 6g-2: the 6g-1 release went all the way to alpha=1, so a command overshoot
  # (0x42 seg4 S-curve: desiredCurvature spiked 2.6x, wheel slammed to -35°, driver
  # grabbed) hit the wheel undamped. Cap the released alpha so a fast catch-up keeps
  # ~30% damping (firm, not a "휙"). Kill switch: RELEASE_MAX = 1.0 (= 6g-1).
  SMOOTHING_ANGLE_RELEASE_MAX = 0.7
  # Phase 6g-2 introduced a low-speed micro-jitter deadband at 0.4 deg.
  # Phase 6h-1: re-sim on the deployed 6g-1 streams measured the 0.4 deg deadband
  # at +11% p95 tracking error / +44% bias for ZERO smoothness gain (2-8 Hz RMS
  # unchanged) — the dither absorption now lives upstream in controlsd tau(v).
  # Shrink to the CAN LSB (0.1 deg). Kill switch: 0.0 (off) / 0.4 (= 6g-2).
  SMOOTHING_ANGLE_DEADBAND_DEG = 0.1
  SMOOTHING_ANGLE_DEADBAND_MAX_VEGO = 11.0  # m/s (~40 km/h); deadband only below this

  # Phase 5: driver-override thresholds for CANFD_LKA_STEERING_ALT angle-control.
  # Problem observed in routes 42/43: at ~30 km/h, driver turning wheel >90°
  # applies only 60-80 Nm torque, never reaching the old fixed 150 Nm
  # FULL_OVERRIDE threshold → MADS kept fighting back (user-reported "tic
  # tic"). Fix: speed-dependent full-override torque.
  #   - below LOW_V_SPEED  (~29 km/h): FULL at 60 Nm  (responsive at low speed)
  #   - above HIGH_V_SPEED (~54 km/h): FULL at 120 Nm (stable at high speed)
  #   - between: linear interpolation
  # DEADZONE also reduced 30 → 25 Nm for earlier onset of override ramp.
  DRIVER_TORQUE_DEADZONE = 25.0
  DRIVER_TORQUE_FULL_OVERRIDE_LOW_V  = 60.0
  DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V = 120.0
  DRIVER_TORQUE_LOW_V_SPEED  = 8.0    # m/s ≈ 29 km/h
  DRIVER_TORQUE_HIGH_V_SPEED = 15.0   # m/s ≈ 54 km/h

  # Angle-control MDPS (Ioniq 6 N CCNC + CANFD_LKA_STEERING_ALT) reports
  # STEERING_COL_TORQUE that INCLUDES EPS reaction force during angle tracking,
  # not just driver input like torque-control cars. Route 0x49 evidence on
  # ~210k latActive frames:
  #   - Light hand grip (steeringPressed=False): p50=36, p75=92, p90=184 Nm
  #   - Active driver steering (steeringPressed=True): p50=381, p75=488, p90=619 Nm
  #
  # 2026-05-12 (drivelog 0000000d, 22,309 op-active samples): the prior
  # thresholds (DEADZONE=200, FULL=450/600) made override_factor sit at
  # ~0.20 even at 250 Nm — desired_angle_deg blend (carcontroller.py:746)
  # only pulled 20% toward the wheel, and op kept commanding the lane-keep
  # angle. Result: 32.9% of latActive frames had op commanding >1° opposite
  # to the driver's torque direction; 78.2% under blinker; 69.8% under
  # <8 m/s. ACIGain (lowered in v2/v3) reduced MDPS authority, but the
  # commanded angle itself was still toward the lane — the residual
  # fraction reached the wheel as resistance.
  #
  # Revised thresholds tighten the override ramp so the driver-intent
  # zone blends desired_angle_deg linearly toward the actual wheel:
  #   - DEADZONE 200 -> 100: starts blending just above light-grip p75
  #     (92). Light grip (<100 Nm) still gets override_factor=0 —
  #     preserves hands-off stability for cruising.
  #   - FULL_OVERRIDE_LOW_V  450 -> 200: full wheel-tracking at active-
  #     steering p25 (250) range; reaches 0.50 by 150 Nm.
  #   - FULL_OVERRIDE_HIGH_V 600 -> 350: same shape at highway, where
  #     EPS reaction adds more to the reading.
  # DRIVER_TORQUE_LOW_V_SPEED=8.0 / HIGH_V_SPEED=15.0 unchanged.
  #
  # 2026-05-12 (Phase 6 driver-torque retune): drivelog 0000000f+10
  # showed only a 51% snap-entry rate in the 150-200 Nm band — the
  # transition zone where the driver clearly intends to override but
  # the snap-entry trip (190 Nm at the time) sat just above. Lowered
  # FULL_OVERRIDE_LOW_V 200→180 so the snap trip falls to 172 Nm,
  # still safely above the light-grip p90 (184 Nm) and absorbed by
  # the Phase 5 30 ms steer_torque LPF against ±5 Nm CAN noise.
  DRIVER_TORQUE_DEADZONE_ANGLE              = 100.0
  DRIVER_TORQUE_FULL_OVERRIDE_LOW_V_ANGLE   = 180.0
  DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V_ANGLE  = 350.0
  # Phase 5b (B2) blinker variant: driver light grip (~92 Nm light-grip
  # p90, measured in drivelog 0000000e where the non-blinker 100 Nm
  # deadzone left override_factor=0 during 50% of lane-change frames)
  # should immediately start the override blend. Lowered deadzone +
  # full-override thresholds keep the blend curve meaningful in the
  # light-grip range so op yields the wheel as soon as the driver
  # signals intent. Combined with the Phase 5d (A2) blinker ceiling
  # cap of 0.45 in compute_torque_reduction_gain, both the command-
  # and authority-side of the actuator stop pulling against the driver.
  # Phase 10b (blinker yield strengthening): ccnc-drivelog 0x07-0x0a (162 seg)
  # measured driver |torque| during blinker-on op-active at p50=253 / p95=499 Nm
  # versus p50=104 / p95=370 without blinker — the driver applied ~2.4x the force
  # to finish a lane change because op kept resisting until the 130-220 Nm
  # full-override point. Lower the blinker deadzone + full-override thresholds so a
  # light lane-change input yields op with much less effort. Kill: restore 70/130/220.
  # Phase 10c review: low-V full-override raised 85 -> 100. 85 sat below the
  # measured resting-grip p50 (~104 Nm), so merely holding the wheel with the
  # blinker on was already a FULL override; 100 keeps the yield cheap for a real
  # lane-change input while resting hands stay inside the blend zone.
  DRIVER_TORQUE_DEADZONE_ANGLE_BLINKER       = 45.0
  DRIVER_TORQUE_FULL_OVERRIDE_LOW_V_BLINKER  = 100.0
  DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V_BLINKER = 150.0
  # Phase 10d: undebounced heavy-grip-anchor torque floor while the blinker is on.
  # steeringPressed (the anchor's normal gate) needs |tq| > 350 Nm for 5 frames on
  # angle-control cars, which kept the driver fighting op through the start of every
  # signaled lane change (routes 0x00-0x0b: per-episode peak |tq| p50 = 519-562).
  # 220 clears the +90..180 Nm column-torque sensor offset with margin, so a
  # hands-off blinker (10c hands-off authority) can never false-fire the anchor.
  # Kill switch: 1e9 (anchor reverts to steeringPressed-only).
  # Phase 26: DRIVER-TORQUE DOMAIN (raw 220 -> 120; the carcontroller now
  # feeds driver_tq = |raw| - hold_comp, and the straight-line hold baseline
  # is ~98 Nm, so 120 here == the old raw 220 in a straight). Phase 22 showed
  # the "+90..180 offset" is actually op's own holding torque, which RISES
  # with curvature — the raw 220 test that "can never false-fire" hands-off
  # is crossed on 25-50% of hands-off curve frames (0x36-0x3d), firing the
  # anchor with no driver input. Raw-domain revert: 220/180.
  BLINKER_ANCHOR_TORQUE_NM = 120.0
  # Phase 14-2: stateful anchor. The single-threshold test flapped 3.84x/s in
  # low-speed blinker waits (0x2e-0x2f) — fire after FIRE_FRAMES sustained
  # >= TORQUE_NM, hold while >= RELEASE_NM (40 Nm band) with a minimum hold.
  BLINKER_ANCHOR_RELEASE_NM      = 80.0   # Phase 26: driver-domain (was raw 180)
  BLINKER_ANCHOR_FIRE_FRAMES     = 3
  BLINKER_ANCHOR_MIN_HOLD_FRAMES = 30     # 0.3 s
  # Phase 14-2b: sustained let-go before release (was a single sub-180 frame,
  # which flapped 2-3x/s against the 3-frame fire debounce under offset-band
  # noise). 0.5 s matches the 13a-standard release counters
  # (LOW_SPEED_GRIP_RELEASE_FRAMES / ANGLE_PASSIVE_EXIT_FRAMES).
  BLINKER_ANCHOR_RELEASE_FRAMES  = 50     # 0.5 s
  # Phase 12c (10c fix): torque breakpoints for the blinker ACIGain ceiling taper
  # (0.45 -> 0.28). 10c reused [DEADZONE_BLINKER=45, FULL_OVERRIDE_LOW_V_BLINKER=100]
  # which sit BELOW the +90..180 Nm column-torque sensor offset, so a hands-off
  # blinker already read as "real force" and the ceiling collapsed to ~0.28 —
  # routes 0x10-0x28 measured hands-off-blinker gain p50 = 0.260 against 10c's
  # stated intent of 0.45. Start the taper at the 10d blinker-intent threshold
  # (220 clears the offset with margin) so true hands-off keeps 0.45 and the
  # 0.28 yield engages only on real force. Sim on 0x10-0x28: hands-off blinker
  # gain p50 0.260 -> 0.364 (true hands-off frames 0.45), grip-episode min gain
  # unchanged at 0.100. Kill switch: restore [45.0, 100.0].
  ACIGAIN_BLINKER_GATE_START_NM = 100.0   # Phase 22: driver-torque domain (was raw 220)
  ACIGAIN_BLINKER_GATE_FULL_NM  = 180.0   # Phase 22: driver-torque domain (was raw 300)
  # Phase 12a (P1): TX-layer steady-curve trim. Routes 0x10-0x28 (24 routes,
  # 214 sustained-curve windows) measured TX/cmd = 1.00 but wheel/TX = 0.55-0.63:
  # this MDPS realizes only ~60% of a sustained angle command under tire
  # self-aligning load (|TX-wheel| p50 = 3.7°), independent of ACIGain (delivery
  # 0.58 even at gain 0.8-1.0, corr 0.15) and not an offset/steer-ratio artifact
  # (L/R symmetric 0.587/0.622, straight-line residual -0.44°). The model's
  # visual loop hides the deficit by inflating cmd ~1.6x, which is why curves
  # start shallow and progressively tighten (entry one-shot rate 41-54%,
  # city overshoot tail p90 +36%). This trim closes the residual at the TX
  # layer instead: slow (rate-limited), capped, gated on a sustained curve and
  # hands-off, bled on exit/grip. It stacks safely with latcontrol's Phase 7a
  # curvature integrator (cap ~1° equivalent): both are driven by the same
  # residual, so as the residual closes both stop growing — no double count.
  # The trimmed command still passes VM + BASELINE_VM limiters and panda safety.
  # Kill switch: CURVE_TRIM_RATE_DPS = 0.0.
  CURVE_TRIM_MIN_CMD_DEG    = 4.0     # curve gate: |desired| must exceed this
  CURVE_TRIM_SUSTAIN_FRAMES = 100     # ... for 1 s before the trim starts
  # Phase 15: slew restored 1.0 -> 2.0 deg/s and cap widened. 13b halved the
  # slew fearing weave feed-through, but the tau=1.5 s residual LP owns that
  # defense: 0x30-0x33 replay shows 2.0 deg/s doubles the in-curve trim reach
  # (p50 0.72 -> 1.44 deg, p90 2.35 -> 4.69) at the SAME trim 2-8 Hz content
  # as 1.5 deg/s (0.035 vs 0.036 deg — LP-dominated), while the on-road
  # tracking gap sat unchanged at |cmd-wheel| p50 = 5.45 deg because the slew
  # x curve-duration product was too small to matter (median trim +0.88 deg).
  CURVE_TRIM_RATE_DPS       = 2.0
  CURVE_TRIM_CAP_SPEEDS_MS  = [8.3, 27.8]   # 30 -> 100 km/h
  CURVE_TRIM_CAP_DEG        = [6.0, 4.0]    # tighter cap at speed
  CURVE_TRIM_BLEED_TAU_S    = 0.5     # decay when gate drops (grip/straight/inactive)
  CURVE_TRIM_FLIP_TAU_S     = 0.15    # fast decay of trim opposing the curve direction (S-curve flip)
  # Phase 12b (P2): hysteresis deadband on the desired angle. Model replan churn
  # reverses the command direction 6.5-7.7x/10s in sustained curves and the wheel
  # follows 4.3-5.8 of those (routes 0x10-0x28). A +/-0.15° backlash band absorbs
  # micro-reversals with ZERO added lag for any move larger than the band
  # (0.15° is ~2% of a typical 9° curve command). Kill switch: 0.0.
  CMD_HYSTERESIS_DEG = 0.15   # legacy flat value (kept for reference/kill)
  # Phase 32 (0x4a-0x4b 저속 떨림 복귀): speed-scheduled band. Measured
  # low-speed plan churn ~0.5° RMS at 0.8-2.5 Hz; the flat 0.15 band passed
  # nearly all of it to TX, and Phase 31 raised delivery (mean low-speed
  # authority +10%, transmission wheel/TX 0.48->0.56). 0.5° below ~25 km/h
  # absorbs the churn at the SOURCE (harness closed-loop: 79-94% wheel-band
  # cut alone) with bounded lag (deadband lateral cost peaks 0.012 m/s2
  # mid-taper — negligible); tapers to legacy 0.15 by ~43 km/h. m/s bkpts.
  # OPEN ITEM (review): source(+5%) x transmission(+17%) predicts only
  # ~+23% of the measured +47% RMS rise — a second contributor is
  # uncharacterized. This hysteresis-only ship isolates it: residual shake
  # on the next drive = the missing half, and the measured-but-reserved
  # ceiling step (see base_ceiling comment) is the next lever.
  # Kill: V = [0.15, 0.15] (legacy flat).
  CMD_HYSTERESIS_SPEEDS_MS = np.array([7.0, 12.0])
  CMD_HYSTERESIS_V         = np.array([0.50, 0.15])
  # Phase 33: blindspot-gated large-correction softening. A pending swing
  # of >= BLIND_CORR_ERR_DEG toward a side whose BSM radar reports occupied
  # (held BLIND_HOLD_FRAMES past the last active frame) recovers at the
  # tapered rate with the error boost off — never a snap toward an occupied
  # lane, never a refusal to correct (rate floor = reference 0.004).
  # Kill: BLIND_CORR_ERR_DEG = 1e9.
  BLIND_CORR_ERR_DEG         = 3.0
  BLIND_CORR_ERR_RELEASE_DEG = 2.0   # hysteresis: engage 3.0, hold to 2.0
  BLIND_HOLD_FRAMES          = 100
  # Phase 37b: no blinker CONCESSION toward an occupied side. With the blinker
  # on, the driver-override thresholds drop (DEADZONE 45 / FULL 100-150), the
  # ACIGain ceiling falls to 0.28-0.45 and the blinker anchor hands the wheel
  # over at 120 Nm — a resting hand then drifts the car across. When the BSM
  # radar reports the blinker side occupied (BLIND_HOLD debounce), all three
  # stay at their non-blinker values: op keeps the lane unless the driver
  # really grips (DRIVER_PRESSED / non-blinker override). 0x58-0x5d: 4 of 51
  # same-side blinker+BSM episodes, all resolved by the driver waiting for the
  # radar to clear — so this costs nothing on the corpus. If the anchor is
  # already ACTIVE when the side becomes occupied, its effect stops in one
  # frame (heavy_grip_anchor needs override_factor >= 0.9, computed from the
  # concession-selected thresholds); the flag itself lingers inert for the
  # release debounce, and the re-assert runs under the 37a recovery caps
  # (frames_since_apply_anchor == 0, gain <= 0.012/frame). Kill: False.
  BSM_BLINKER_NO_CONCESSION  = True
  # Phase 13a: low-speed (<20 km/h) scenario gate + offset-proof grip signal.
  # Routes 0x2a-0x2d (first build with Phase 11): below 20 km/h the passthrough
  # latch keyed on `hands_off` (override_factor <= 0.5), whose low-V full-override
  # point (180 Nm) sits inside the +90..180 Nm column-torque offset — the latch
  # flapped plan<->wheel 1.1-2.2x/s for 99% of low-speed hands-off time, which IS
  # the reported strong/frequent wheel shaking. Additionally the user finds
  # model-planned steering at low speed uncomfortable in free maneuvers
  # (intersection turns / alleys, |cmd| 100°+) while wanting it in traffic
  # crawl and gentle lane keeping. Below 20 km/h steering is therefore allowed
  # only when: traffic-following (lead within TRAFFIC_FOLLOW_* in carcontroller,
  # widened 3/5 -> 8/12 m) OR gentle path (|cmd| under the 45/35 hysteresis pair).
  # Transitions carry asymmetric dwell (fast to-passive, slow to-active) so the
  # gate cannot flap. The grip latch: 0x2a-0x2d measured hands-off |tq| at
  # 5-20 km/h of p50=156 / p90=284 / p99=367 Nm (offset + road load), so NO
  # fixed threshold below ~350 separates grip from hands-off — a 220 Nm gate
  # flapped 3.5x/s in replay, no better than the override_factor it replaced.
  # Latch therefore ENTERS on the debounced steeringPressed flag (350 Nm x
  # 5 frames — real grip only) and RELEASES on sustained sub-260 Nm
  # (= ACIGAIN_GRIP_FULL_NM) for LOW_SPEED_GRIP_RELEASE_FRAMES. Flap-free by
  # construction: entry needs real grip, release needs a sustained let-go.
  # Parking mode remains the higher-priority passive latch, unchanged.
  # Kill switch: both LOW_SPEED_CMD_*_DEG = 1e9 (scenario gate always open).
  # Phase 26: enter keys on the hold-compensated driver_pressed. The RELEASE
  # threshold stays at the raw-domain 260: release tests only ever run while
  # the latch holds op passive, where Phase 26b gates hold_comp to 0 (op
  # contributes nothing to the bar) — driver_tq == raw there, so 260 keeps
  # the weeks-validated 14-1 release semantics bit-identical. G1 measurement
  # (608 passive-signature windows, 855 s): rolling-passive hands-off |tq|
  # p50 = 147-233 by speed band, sustained floor p50 = 158 — a 160 threshold
  # would leave only 50.5% of windows releasable vs 87.8% at 260 (rolling
  # bar friction/caster load is NOT the parked ~0 Nm).
  LOW_SPEED_GRIP_RELEASE_NM        = 260.0
  LOW_SPEED_GRIP_RELEASE_FRAMES    = 50     # 0.5 s sustained let-go to resume
  # Phase 14-3: single 40° boundary -> 45/35 hysteresis pair (lane-keep scale;
  # intersection turns are 100°+). ~10% of residual low-speed flips on
  # 0x2e-0x2f clustered at the old single threshold.
  LOW_SPEED_CMD_PASSIVE_DEG        = 45.0   # exceed while steering -> go passive
  LOW_SPEED_CMD_ACTIVE_DEG         = 35.0   # fall below while passive -> re-engage
  LOW_SPEED_SCEN_TO_PASSIVE_FRAMES = 30     # 0.3 s to yield
  LOW_SPEED_SCEN_TO_ACTIVE_FRAMES  = 100    # 1.0 s to (re)engage
  # Phase 18 creep gate (see carcontroller): under ENTER the scenario gate
  # requires traffic-following AND blinker-off — the model command itself
  # oscillates near standstill (0x36-0x37: cmd RMS 1.2-2.4°, wheel shake at
  # ~1 Hz in 27% of creep steering time) and only lead-anchored queue crawl
  # measured quiet. Kill switch: CREEP_GATE_ENTER_MS = 0.0.
  CREEP_GATE_ENTER_MS = 10.0 / 3.6
  CREEP_GATE_EXIT_MS  = 12.0 / 3.6
  # Phase 13b: 12a trim repair + noise hardening. As shipped, 12a armed almost
  # never (sustained-curve TX-cmd p50 = +0.02° on 0x2a-0x2d) because its gate
  # reused the offset-corrupted `hands_off`, whose torque test flaps in the
  # p50=156/p90=284 Nm hands-off band (see 13a note). Repair: gate on the
  # debounced steeringPressed only — real grip still cuts the trim (and bleeds
  # it), while sensor-offset torque can no longer reset the 1 s sustain.
  # Hardening: the residual consumes the RAW wheel angle whose dominant motion
  # is a 0.6-1.2 Hz weave — a tau=0.3 s LP passed it nearly unattenuated
  # (replayed trim 1-8 Hz RMS 0.093°, the size of the whole v2 jitter budget).
  # tau=1.5 s + slew 1°/s put the trim two octaves below the weave; the
  # deadband keeps residual noise out of the integrator and sets the closure
  # floor (~0.7° of the 3.7° deficit). The angle gate holds with hysteresis
  # (arm >= CURVE_TRIM_MIN_CMD_DEG 4°, hold >= 3°) so it cannot flap at the
  # curve threshold (53 re-arms/min in replay pre-fix).
  CURVE_TRIM_HOLD_CMD_DEG       = 3.0
  CURVE_TRIM_RESID_LP_TAU_S     = 1.5
  CURVE_TRIM_RESID_DEADBAND_DEG = 0.7

  # Phase 9: yield-by-authority. The driver yield is done entirely on the ACIGain
  # authority axis (reduce how hard MDPS follows op, not WHERE op points), NOT a
  # command-side wheel-blend. §1.K element-isolation proved the old grip-blend was
  # the SOLE input->TX 2-8 Hz injector (it mixed the noisy measured wheel into the
  # angle command and false-fired on the +90..180 Nm column-torque sensor offset);
  # authority reduction injects nothing — it just lets the driver win, and op
  # keeps commanding its own clean trajectory. Gated on the DEBOUNCED
  # steeringPressed flag (not the offset-corrupted torque) so hands-off ACIGain is
  # the legacy curve (full authority + §5c drift recovery); only real grip drops
  # authority to the floor over [deadzone, GRIP_FULL] to provide the yield.
  # Safety-neutral: only reshapes the op-side ACIGain; panda's VM angle-limit +
  # ACIGain bound are unchanged.
  # (Pruned dead experiments — restore from git / POST_6F2_AUDIT if ever needed:
  #  Phase 8 column-torque offset estimation, ineffective on real CAN §1.J;
  #  Phase 8b grip-blend wheel-LP, superseded by this; Phase 10 sunnypilot shelf,
  #  net-negative under our pressed-gate §1.O.)
  ACIGAIN_GRIP_FULL_NM = 110.0    # Phase 22: DRIVER-torque domain (was raw 220); kill: 220.0 + HOLD_SCALE=0
  ACIGAIN_GRIP_FLOOR   = 0.08     # Phase 21: 0.10 -> 0.08; kill: 0.10
  # Phase 22: op holding-torque baseline (subtracted before the yield curve;
  # see carcontroller). Parked measurement: true sensor offset p99 = 3 Nm —
  # the 90-180 Nm 'offset' was the MDPS's own effort. Fit (n=31k hands-off):
  # hold = 122 + 132*lat_acc; applied at 80%, capped. Kill: SCALE = 0.0
  # (together with restoring the raw-domain breakpoints — see carcontroller).
  # Phase 23: hands-off band constants (the caller used to hard-code legacy
  # 350/0.19 literals, dead-coding every hands-off change since Phase 21) +
  # speed-scheduled floors. Past full-yield the felt residual force IS the
  # floor, so highway relief for firm-grip spirited driving goes there;
  # city floors keep tracking authority. Kill: FLOOR_V tables flat at
  # [0.15,0.15]/[0.08,0.08], HANDSOFF_FULL 140 -> 350 + HOLD_BASE_V/
  # LAGAIN_V all-zero (raw domain).
  # Phase 25: full-yield points speed-scheduled — at speed the taper completes
  # earlier so a given driver push yields proportionally MORE than in town
  # (city calibration at/below 40 km/h unchanged; floors keep the Phase 23
  # schedule). General principle: driver input must win more decisively as
  # speed rises. Kill: flat [140,140]/[110,110].
  ACIGAIN_FULL_SPEEDS_KPH   = [40.0, 120.0]
  ACIGAIN_HANDSOFF_FULL_V   = [140.0, 100.0]   # driver-torque domain
  ACIGAIN_GRIP_FULL_V       = [110.0, 80.0]
  ACIGAIN_HANDSOFF_FULL_NM  = 140.0   # low-speed anchor (kept for reference/kill)
  ACIGAIN_FLOOR_SPEEDS_KPH  = [80.0, 140.0]
  ACIGAIN_HANDSOFF_FLOOR_V  = [0.15, 0.10]
  ACIGAIN_GRIP_FLOOR_V      = [0.08, 0.05]   # superseded for GRIP by the Phase 35a tables below (kept for kill)
  # Phase 35a: "at speed, when I hold the wheel, assist must get out of the
  # way" (field request; calibration point = the driver's strongest everyday
  # grip, Yongsan tunnel section 0x5a seg11 at 89-107 km/h: |tq| p50 750 /
  # peak 1100 Nm; everyday pressed p50 421 Nm — all already past the 350
  # override point, so the felt residual is the FLOOR and the DROP LATENCY).
  # Replayed 0x58/0x5a (v >= 43 km/h): grip-onset gain<0.2 latency p50 0.40 s,
  # p90 1.50 s; mean gain during grip 0.239. Three grip-side changes, all
  # scheduled to start at 60 km/h so city behaviour (<= 40 km/h) is untouched:
  #   floor 0.08 -> 0.03 (60 km/h+), full-yield point 110 -> 80 driver-Nm
  #   (60 km/h+; the Phase 25 schedule gave 102.5 at 60 km/h), and a rate_dn floor
  #   of 0.03/frame for a real push (see GATE_NM below) so 1.0 -> floor ~0.3 s.
  # Axes carry the pre-35a knots (80 / 120 km/h) so a VALUE-ONLY kill restores
  # the Phase 23/25 schedules exactly:
  #   kill FLOOR35_V = [0.08, 0.08, 0.08, 0.05]  (== [80,140]->[0.08,0.05])
  #   kill FULL35_V  = [110.0, 102.5, 80.0]      (== [40,120]->[110,80]; 102.5 at 60)
  #   kill RATE_DN_FLOOR_V = [0.0, 0.0]
  ACIGAIN_GRIP_FLOOR35_SPEEDS_KPH = [40.0, 60.0, 80.0, 140.0]
  ACIGAIN_GRIP_FLOOR35_V          = [0.08, 0.03, 0.03, 0.03]
  ACIGAIN_GRIP_FULL35_SPEEDS_KPH  = [40.0, 60.0, 120.0]
  ACIGAIN_GRIP_FULL35_V           = [110.0, 80.0, 80.0]
  ACIGAIN_GRIP_RATE_DN_SPEEDS_KPH = [40.0, 60.0]
  ACIGAIN_GRIP_RATE_DN_FLOOR_V    = [0.0, 0.03]
  # The fast-descent floor must NOT engage on a resting hand: hands-off
  # driver_tq at speed is p90 ~119 / p95 ~147 and crosses 100 about 3x per
  # second, so a gate at the arm level (100) ran a 7.5x asymmetric ratchet
  # hands-off (verification: hands-off mean gain -8%, diverged-low-gain
  # exposure 1.2% -> 5.5%). Gate = driver_pressed OR driver_tq >= this level
  # (hands-off p95 is ~147; a real grip, pressed p50 raw 421 = driver ~359,
  # blows through it in its first frames). Measured on the same replay
  # (0x58/0x5a, >= 60 km/h, hands-off = EPS-unpressed):
  #   config        HO mean g  HO div>2&g<0.5  rest-band g  GRIP g  onset p50/p90
  #   pre-35a          0.813       11.7%          0.537      0.195   0.38 / 1.50 s
  #   gate 100 (v1)    0.778       15.4%          0.344      0.089   0.00 / 0.15 s
  #   gate 160 (SHIP)  0.803       13.0%          0.498      0.097   0.01 / 0.18 s
  #   pressed-only     0.821       11.3%          0.542      0.138   0.22 / 1.50 s
  # 160 keeps the grip-side gain (onset p90 1.50 -> 0.18 s) at a ~1% hands-off
  # cost; pressed-only preserves hands-off fully but sub-230 grips never get
  # the fast descent (p90 back at 1.50 s). The +1.3pp diverged-low-gain
  # exposure is the "driver wins at speed" trade the field request asked for.
  ACIGAIN_GRIP_RATE_DN_GATE_NM    = 160.0
  # Phase 36: low-speed CURVE-conditional ceiling with a continuous ramp.
  # The 24a hands-off ceiling (0.18 @0 -> 0.30 @20 -> 0.75 @40 km/h) is the
  # measured shake/authority balance on STRAIGHTS, where model churn lives;
  # in real low-speed curves it starves delivery: 0x58-0x5b hands-off curve
  # windows (|plan| >= 8 deg) follow only wheel/plan p50 0.33 at 10-20 km/h
  # (0.52 at 20-30, 0.64 at 30-40); the three 16 km/h S-curve misses were TX
  # 11-16 deg with the wheel at 1.6-3 deg under a ~0.28 ceiling. Phase 33's
  # GLOBAL step was rolled back (33b) — this raises authority only as the
  # plan is actually curving, so straights keep the 24a rung.
  #   ceiling = base + w * (curve - base),  w = interp(curve_deg, [3, 12], [0, 1])
  # curve_deg = asymmetric EMA of |commanded angle|: rise tau 0.15 s, fall
  # tau 0.5 s. Real curve entries on the corpus ramp the plan 3 -> 8 deg in
  # p50 0.28 s (p10 0.08 s), so a symmetric 0.5 s EMA left w at p50 0.16 at
  # the 8 deg point and 35% of entries with w < 0.1 — the raise arrived
  # after the curve (verifier measurement); 0.15 s rise gives w p50 0.36 and
  # 7% below 0.1 at the same point. The slow fall keeps the exit clean
  # (w >= 0.3 lingers p90 0.34 s after |plan| < 3 deg, no overshoot).
  # What bounds the ceiling modulation is NOT frequency attenuation (a
  # first-order LP at 0.5 s is only -9 dB at 0.8 Hz) but the ramp-input
  # slew: dw/frame p99 = 0.019 on the corpus -> ceiling moves <= ~0.005 per
  # frame, an order of magnitude under the 0.04 rate_up cap, so the gain
  # cannot step. The rise is additionally slew-capped (MAX_RISE_DPS, above
  # the corpus p99 so it is a bound, not a filter): an instantaneous plan
  # step — not seen on the corpus but possible on a lane-change plan flip —
  # would otherwise move the ceiling 0.02/frame; with the cap w moves
  # <= 0.03/frame -> ceiling <= ~0.008/frame at 20 km/h (two gain quanta).
  # No threshold anywhere (field requirement: no step response).
  # Above 40 km/h curve == base (no change). Shake population caveat: ~30%
  # of the Phase 32/33b shake windows (<8 deg wheel excursion) sit in gentle
  # curves with w > 0.2 and DO get a raise, so the next-drive shake metric
  # must be split by w (<0.1 / 0.1-0.5 / >0.5) to be interpretable.
  # Kill: CURVE_CEILING_V = [0.18, 0.30, 0.75, 0.95] (== base) -> identical.
  ACIGAIN_CURVE_CEILING_SPEEDS_KPH = [0.0, 20.0, 40.0, 120.0]
  ACIGAIN_CURVE_CEILING_V          = [0.30, 0.55, 0.75, 0.95]
  ACIGAIN_CURVE_RAMP_DEG           = [3.0, 12.0]
  ACIGAIN_CURVE_MEAS_TAU_RISE_S    = 0.15
  ACIGAIN_CURVE_MEAS_TAU_FALL_S    = 0.5
  ACIGAIN_CURVE_MEAS_MAX_RISE_DPS  = 27.0    # ramp-input rise slew cap (deg/s)
  # Phase 37a: high-speed RECOVERY softening ("correct the error over a longer
  # time at speed"). Two levers, both inert on planned driving:
  # (1) ACIGain rise-rate cap tapered with speed. The 0.04/frame rise (full
  #     authority in 0.25 s) is what lets a post-release correction snap at
  #     highway speed; 0x58-0x5d clean releases >= 60 km/h: gain 0.2 -> 0.8 in
  #     p50 0.4 s, wheel jerk p50 2.6 m/s^3 (n=6), 105 km/h case 15 deg/s.
  #     Cap 0.04 @60 -> 0.012 @100 km/h stretches the recovery to ~1 s. The
  #     35b anchored tail (0.012) sits under the cap everywhere.
  # (2) command lateral-jerk cap inside the recovery window (<= 3 s after an
  #     anchor): 3.59 (panda) @60 -> 2.5 m/s^3 @80+ km/h. Corpus: recovery
  #     frames sat AT the panda cap 41% of the time at 100+ km/h (0x58-0x5b),
  #     planned (op-driven) frames would be clipped 0.8% @80-100 by 2.5.
  #     100 km/h: 16 -> 11 deg/s.
  # RAIN tables (blended by the wiper-derived rain weight, see RAIN_*): one
  # step tighter, plus an earlier yield start so a firm hand wins sooner.
  # Kill: RATE_UP_CAP_V = [0.04, 0.04]; RECOVERY_JERK_CAP_V = [3.59, 3.59].
  ACIGAIN_RATE_UP_CAP_SPEEDS_KPH = [60.0, 100.0]
  ACIGAIN_RATE_UP_CAP_V          = [0.04, 0.012]
  ACIGAIN_RATE_UP_CAP_RAIN_V     = [0.02, 0.008]
  RECOVERY_JERK_CAP_SPEEDS_KPH   = [60.0, 80.0]
  RECOVERY_JERK_CAP_V            = [3.59, 2.5]    # m/s^3 (3.59 == panda limit -> no-op)
  RECOVERY_JERK_CAP_RAIN_V       = [2.5, 1.5]
  RECOVERY_JERK_CAP_FRAMES       = 300            # recovery window after an apply anchor
  ACIGAIN_GRIP_START_NM          = 30.0           # yield-curve start (dry)
  ACIGAIN_GRIP_START_RAIN_NM     = 20.0
  # Rain mode input: front-wiper switch state from ECAN 0x35c byte 18 bit 0
  # (CCNC_WIPER.FRONT_WIPER_ON). 2026-09-01 rain routes (0x58/0x59): bit held
  # 1 for 42 min / 24 min with zero toggles from the moment the windshield got
  # wet to arrival; four dry routes (~3 h): never set. Debounced ON 5 s / OFF
  # 60 s, message stale > 1 s -> off, and the weight RAMPS (no step): up
  # tau 3 s, down tau 10 s. Kill: RAIN_WIPER_ON_FRAMES = 10**9.
  RAIN_WIPER_ON_FRAMES   = 500
  RAIN_WIPER_OFF_FRAMES  = 6000
  RAIN_STALE_NS          = int(1.0e9)
  RAIN_RAMP_UP_TAU_S     = 3.0
  RAIN_RAMP_DN_TAU_S     = 10.0
  # Phase 35b: anchored post-release recovery. Hannam bridge merge (0x5a seg5,
  # 43 -> 60 km/h): after the driver released, gain climbed at the reference
  # 0.004/frame (post-grip taper region, |apply-wheel| 2.5-3.2 deg) for 1.1 s
  # while the plan moved fast; the wheel lagged the plan 5-6 deg and the
  # driver had to grab. Corpus (v >= 43 km/h, 29 clean releases): recovery to
  # gain >= 0.6 p50 0.34 s / p90 2.13 s, max plan-wheel divergence p90 8.9 deg,
  # while the released-wheel delivery rate stayed p90 41 deg/s — the VM jerk
  # limit, not the gain ramp, is what bounds delivery once apply starts AT the
  # wheel (anchor / one-shot). So within ANCHORED_RECOVERY_FRAMES after apply
  # was last pinned to the wheel, at >= 40 km/h, the post-grip taper floor
  # beyond 2 deg is 0.012 instead of 0.004 (3x): a "fresh chase" of the live
  # plan from the wheel is not the stale-command slam the taper was built for.
  # Kill: ANCHORED_RECOVERY_FRAMES = 0.
  ANCHORED_RECOVERY_FRAMES     = 150   # 1.5 s
  ANCHORED_RECOVERY_SPEED_KPH  = 40.0
  ANCHORED_RECOVERY_RATE_UP    = 0.012
  # Phase 31: hold-torque model refit. The Phase 22 linear fit
  # (0.8*(122 + 132*lat_acc), cap 240) had no speed term and a slope the
  # binned corpus contradicts — settling measurement (1.1M hands-off
  # frames): the STRAIGHT-line baseline is speed-dependent (p50 raw 140 at
  # 11-29 km/h down to ~81 at 54-90) and the lat_acc term SATURATES by
  # ~0.3 m/s2 with a speed-dependent gain (EPS assist mapping). The old
  # under-compensation at low speed is the measured root of the hands-off
  # driver_tq inflation (p50 40.6 / p75 111) behind three review cycles of
  # threshold defects (Phase 26 premise, 29 taper premise, 30 timeout).
  # New form: comp(v, la) = B(v) + G(v) * S(la), fitted on the p45 of the
  # !pressed population per (v, la) cell (p45 splits the light-touch
  # contamination above from the true hands-off mass below). Residuals:
  # most populated cells within +/-20 Nm, but NOT all — review-measured
  # outliers: (8-15 m/s, la>=1.2) -97; (30-42, la 0.2-0.3) -71; (3-8,
  # la>=2.0) -69; (30-42, la 0.1-0.2) -66; (22-30, straight) +31 — and six
  # cells where this model is worse than the old linear one. Net corpus
  # |comp error| still drops 34.8 -> 14.6 Nm (58%). The p45 choice is a
  # DELIBERATE direction trade: over-compensating frames (masking real
  # light input) rise 26% -> 58% while phantom torque halves — acceptable
  # only because the EPS raw-350 arm is the anchor backstop, so the
  # safety-critical consumer does not depend on driver_pressed.
  # B(v) below 5.5 m/s is anchored on the 11-29 km/h straight cell and held
  # flat; there is no data below ~3 m/s (crawl
  # is dominated by passive states where 26b gates comp to 0; parked true
  # hold is ~0) — collect a lot-drive data point before trusting it.
  # B(36) is set to 62 (the 130 km/h STRAIGHT cell p45 = 72), not the
  # joint-fit 38 which the sparse mid-la cells dragged down — measured
  # effect: 25-45 m/s hands-off blinker-fire false rate 12.5% -> 10.7%
  # (old model 11.7%). DRIVER-DOMAIN THRESHOLDS
  # ARE UNCHANGED by design: they were always defined against the true
  # driver contribution — this makes them read closer to it. Raw-torque
  # equivalence points shift with speed as a consequence (e.g. pressed
  # 230 == raw ~303 at 54 km/h vs ~370 at 20 km/h) — verified against the
  # corpus for false-fire rates in the Phase 31 replay.
  # Kill: BASE_V and LAGAIN_V all-zero (comp 0 -> raw domain everywhere).
  # NOTE units: these speed breakpoints are m/s (= 20/38/59/90/130 km/h) —
  # unlike every other ACIGAIN_* speed table in this class, which is KPH.
  # np.array (31b): avoids a list->ndarray coercion on every 100 Hz frame.
  ACIGAIN_HOLD_BASE_SPEEDS_MS = np.array([5.5, 10.5, 16.5, 25.0, 36.0])
  ACIGAIN_HOLD_BASE_V         = np.array([140.0, 102.0, 64.0, 62.0,
                                          62.0])  # <- hand-set (fit said 38); see note above
  ACIGAIN_HOLD_LAGAIN_V       = np.array([53.0, 86.0, 133.0, 123.0, 87.0])
  ACIGAIN_HOLD_LA_BP          = np.array([0.0, 0.1, 0.3])
  ACIGAIN_HOLD_LA_S           = np.array([0.0, 0.5, 1.0])
  # 31b (review): guard only — the tables' structural maximum is 197 Nm
  # (max over knots of B+G), so this cap CANNOT bind today. It exists to
  # catch a future table edit that would silently start clipping; a unit
  # test pins model-max < cap so that edit becomes a conscious decision.
  ACIGAIN_HOLD_MAX_NM     = 220.0
  # Phase 34: still-straight crawl override. The fitted tables have no data
  # below ~3 m/s, so B extrapolated flat at 140. Measurement history matters
  # here: a dedicated garage run (0x55-0x57) FIRST suggested this fix, but
  # carStateSP.lateralControlPaused showed 100% of its free-crawl frames were
  # PASSTHROUGH (Phase 18 creep gate: no lead below ~10 km/h) — op never
  # actuated, so that table described a RELEASED wheel, not op hold torque.
  # The domain where this comp actually applies is TRAFFIC-FOLLOWING
  # stop-and-go creep, and the commute corpus (0x51-0x54) has it: 27.5k
  # eff-active (!paused) & !EPS-pressed & !standstill & v<3 frames:
  #   still-straight (|ang|<5, |rate|<10; 72%)   p45 = 28 Nm   <- B says 140
  #   straight, wheel moving (dry friction)      p45 = 117
  #   5-30 deg (still / moving)                  p45 = 125 / 148
  #   >= 30 deg                                  p45 = 198
  # i.e. crawl hold torque is MOTION/ANGLE-modal, and flat 140 is right for
  # every state EXCEPT still-straight, where it over-compensates by ~112 and
  # pushes the pressed raw equivalent to 370 — BEHIND the EPS hardware flag
  # (~350). Override only that state. The still-straight tq distribution has
  # a heavy tail (p45 28 / p75 104 / p95 226) — sub-350 LIGHT HANDS resting
  # on the wheel in stop-and-go contaminate the "hands-off" population — so
  # the override value is a sensitivity/false-pressed trade, measured on the
  # 19.7k still-straight frames (single-frame >=230 / >=100 rates):
  #   comp 140 (old):  0.04% / 4.0%   pressed raw equiv 370 (behind EPS)
  #   comp  70 (SHIP): 0.83% / 10.1%  pressed raw equiv 300  <- user-set point
  #   comp  28 (p45):  2.66% / 19.2%  pressed raw equiv 258 (reads resting
  #                                   hands as grips ~2.3% of creep time)
  # 70 is deliberately ABOVE the 28 p45 fit point: the +42 headroom absorbs
  # most of the light-touch band while still moving grip recognition ahead
  # of the EPS hardware flag (300 < 350). The gate only LOWERS comp (raises
  # sensitivity), so per the asymmetry design rule its cost lands on the
  # hands-off side; the 4 Nm/frame slew absorbs both edges (140<->70 ~0.2 s).
  # Adjacent band 3.0-5.5 m/s is intentionally NOT gated: the same
  # over-compensation exists there (still-straight raw p45 = 39 vs comp 140,
  # pressed raw equiv 370 > EPS 350 — the 31b defect-3 dead band stays open
  # in that band), and it is acceptable NOT because the user's setpoint was
  # crawl-only but because the EPS arms carry the safety-critical consumers
  # there (verified gate-by-gate, not asserted): at v < 8 m/s the
  # heavy_grip_anchor EPS arm needs override>=0.9 == raw >= 172, far below
  # the EPS-350 flag, so the arm is live (no repeat of the 31b highway
  # inversion); and 31b fix 3 put the EPS OR directly on the low-speed
  # latch entry. Uncovered residue: real_grip yield depth and trim_gate —
  # both minor.
  # Arm-chain note (verification round): the +3.4pp arm-sustained time is
  # harmless ALONE (anchor_recent arms only on pressed), but driver_pressed
  # is ITSELF a pressed arm and rose 12x (0.05 -> 0.63% machine ON-time).
  # Accepted because the one-shot additionally needs div > 2 deg plus a
  # release edge, and the crawl consequence (apply := wheel at <5 deg,
  # curve_trim zeroed when already ~0) is bounded.
  # Rate gate hysteresis (verification round, MANDATORY): the derived wheel
  # rate at 100 Hz with 0.1 deg resolution is quantized in 10 deg/s steps
  # and its still-straight p90 sits exactly ON 10.0 — a single-threshold
  # gate measured p50 4.2 toggles/s with mean dwell ~24 frames > the 17.5
  # the slew needs for a full 140<->70 swing, i.e. a NEW 2-4 Hz authority
  # modulation in the exact band Phases 32/33b cleaned. Enter below 10,
  # exit above 14 (off the quantization levels).
  # Kill switch: CRAWL_STILL_SPEED_MS = 0.0 (gate never true -> Phase 31).
  ACIGAIN_CRAWL_STILL_COMP_NM = 70.0
  CRAWL_STILL_SPEED_MS        = 3.0    # m/s (NOT kph — matches HOLD tables)
  CRAWL_STILL_ANG_DEG         = 5.0
  CRAWL_STILL_RATE_DPS        = 10.0   # enter below
  CRAWL_STILL_RATE_EXIT_DPS   = 14.0   # exit above (hysteresis, see note)
  # Phase 26: slew guard on hold_comp (per 10 ms frame). A single-frame
  # angle/speed sensor spike would otherwise jump the compensation by up to
  # +142 Nm and mask a real driver input for that window; genuine curve
  # entries measure ~1 Nm/frame. 4 Nm/frame = 400 Nm/s.
  ACIGAIN_HOLD_SLEW_NM    = 4.0
  # Phase 26: hold-compensated equivalent of the EPS steeringPressed flag,
  # used by every carcontroller latch (the EPS flag itself — raw 350 enter /
  # 280 exit, 5-frame counter, carstate.py R4 — is untouched for core
  # openpilot override/alert semantics). 250 driver-domain == raw ~350 in a
  # straight (hold baseline ~98 Nm); in curves the raw requirement rises
  # with the hold estimate instead of the flag tripping on op's own effort
  # (41% of raw pressed frames sat at hold-model >= 300 Nm on 0x36-0x3d).
  # The 0.8 exit-hysteresis factor and 5-frame debounce mirror the EPS flag.
  # High-lat_acc note (2026-08-12 review): the linear fit extrapolated past
  # ~1.5 m/s2 would predict hands-off torque above every threshold here, but
  # MEASURED hands-off |tq| saturates instead of following the fit (p50 by
  # lat_acc bin: 0.3-1.2 -> ~222, 1.2-1.6 -> 187, 1.6-2.2 -> 204, 2.2-5.0 ->
  # 133 Nm; >=350 tail flat at ~5% across all bins), so the capped model
  # slightly OVER-compensates there and self-fire risk does not grow with la.
  # The >1.5 m/s2 band is thin in the replay set (~7.3k frames) — verify
  # hands-off driver_pressed episodes at high lat_acc on future spirited logs.
  # Phase 31: 250 -> 230. The refit raises low-speed compensation (98 ->
  # 140 at the straight baseline), which pushed marginal real grips
  # (episode peaks 250-290 driver-domain under the old comp) below the
  # threshold: corpus real-grip retention 87.7% -> 75.9% at 250. 230
  # restores 83.9%; the 6-frame machine's hands-off false rate measured
  # 0.00% on the comp-on population (instantaneous >=230 tail 0.52%,
  # debounced away). Review upper bound WITHOUT modeling the 26b passive
  # gate: 0.108%, clustered at crawl speeds in passive states where comp
  # is correctly 0 and raw>=230 IS a grip — i.e. the residual is the
  # gated population, not a false fire.
  # The EPS raw-350 arm remains the anchor backstop either way.
  DRIVER_PRESSED_NM     = 230.0
  # 31b (review): raw-domain threshold used when the 26b gate has comp OFF
  # (passive states) — there driver_tq == raw and 230/184 sat inside the
  # measured rolling-passive band (p50 147-233); 250/200 is the
  # weeks-validated raw operating point.
  DRIVER_PRESSED_RAW_NM = 250.0
  DRIVER_PRESSED_FRAMES = 5
  # Phase 28 (0x41 yank fix). Field failure at 86 km/h (route 0x41 seg 16-17,
  # build 126c0ca): a 430-500 Nm grip in an 8° curve put hold_comp at its
  # 240 cap, so driver_pressed needed raw ~490 and the heavy-grip anchor
  # DISENGAGED (the Phase 26 review's accepted "350-490 raw band" residual —
  # the EPS raw-350 pressed flag was True through the whole hold). With no
  # anchor, apply_angle_last tracked the plan to 5-6° away from the held
  # wheel; on release driver_tq collapsed to ~0 -> ACIGain recovered at full
  # rate WITH the error boost active (real_grip False) -> the MDPS slammed
  # the wheel toward the stale apply (-0.3° -> +8.8° in 0.3 s at 86 km/h).
  # Corpus fact that shapes this design (adversarial review, 508k hands-off
  # frames): hands-off driver_tq is NOT small — p50 40.6 / p75 111 /
  # p90 162.5 / p99 250 (the hold fit under-compensates below ~1.2 m/s2) —
  # so NO driver_tq magnitude threshold below driver_pressed itself can
  # separate a sub-pressed grip from hands-off. The fix therefore keys on
  # the weeks-validated EPS flag and on divergence evidence instead:
  # 1) heavy_grip_anchor regains CS.out.steeringPressed as an OR-arm (the
  #    pre-Phase-26 operating point: it anchored 153/274 frames of the
  #    field event's divergence window and ran for weeks without a yank;
  #    its consequence — apply := wheel — is benign, unlike the STEER_REQ=0
  #    latches Phase 26 converted).
  # 2) release re-anchor: only AFTER a recent real anchor episode
  #    (heavy_grip_anchor within REANCHOR_RECENT_FRAMES), when the driver
  #    lets go (driver_tq < 30) with a stored divergence > 2°, snap apply
  #    to the wheel. Covers the anchor's dropout gaps (pressed hysteresis /
  #    override_factor dips while still holding) where divergence builds.
  #    Fires keep the episode memory (F1 review fix) and are rate-limited
  #    by a refractory instead of a disarm; each fire zeroes curve_trim so
  #    the wound trim cannot recreate the dumped divergence (F5).
  #    The anchor_recent gate makes hands-off firing structurally rare:
  #    hands-off anchor engagement needs the EPS flag AND override >= 0.9
  #    simultaneously (hard-curve tail only).
  # 3) error-boost hold-off while anchor_recent: recovery from a just-
  #    released grip starts unboosted; hands-off drift recovery elsewhere
  #    keeps its boost (the arm-based v1 gate suppressed 14.6% of hands-off
  #    boost frames — rejected in review).
  REANCHOR_ARM_NM          = 100.0
  REANCHOR_ARM_FRAMES      = 30
  REANCHOR_ARM_CAP_FRAMES  = 100
  REANCHOR_FIRE_NM         = 30.0
  # Phase 30: the one-shot fires only AT the release edge (within 3 frames
  # of the torque falling below FIRE). Divergence stored during the grip is
  # by definition present at that instant; anything that appears later is
  # op's OWN post-release approach progress (VM builds 2° in ~0.1 s), and
  # dumping that re-anchored the handover once per release (harness: a
  # -1.9° wrong-direction step at frame 11 with a 20-frame window).
  REANCHOR_EDGE_FRAMES     = 3
  REANCHOR_MIN_DIV_DEG     = 2.0
  # 2 s anchor-episode memory for the re-anchor gate. Set by the PRESSED
  # anchor arms only (a blinker-arm anchor is not grip evidence — G3 review:
  # including it produced 129 context-free hands-off fires). Measured
  # anchor->release gap (n=440): p50 0.30 / p90 1.86 / p95 3.00 s — the 2 s
  # window covers ~91%; the 9.1% slower-easing tail is an accepted residual.
  REANCHOR_RECENT_FRAMES   = 200
  # Boost hold-off window: only the first 0.25 s after an anchor frame — the
  # recovery transient. Reusing the full 2 s memory suppressed the boost on
  # 26.1% of hands-off frames (G1 review, worse than the rejected v1 gate);
  # 20-30 frames lands at ~7-9%.
  BOOST_HOLDOFF_FRAMES     = 25
  # Phase 30: the Phase 29/29b divergence LEASH (REANCHOR_LEASH_DEG 2.0 +
  # LEASH_WHEEL_LP/SNAP) was REMOVED — torque cannot distinguish a resting
  # hand from a released one (hands-off driver_tq p50 40.6 / p75 111), so
  # the clamp equally pinned apply to a free, caster-unwinding wheel:
  # corpus audit measured 36 urgent-regrab windows in the pinned state,
  # including a field lane-departure (0x47 seg15, Namsan-3 tunnel
  # approach). Its target class (rigid slow-ease slam) was model-only and
  # stays covered by the one-shot dump + recovery taper + VM rate limits.
  # Torque-gated rescues capped out at 50-53% coverage (silence-timeout /
  # arm-budget, both defeated by hands-off torque noise). See the Phase 30
  # commit for the full audit.

  # Phase 6d angle-aware passive thresholds. Drivelog 0000002[01]
  # (94.7k frames, Phase 6c build b6e5842) showed sustained-grip
  # self-centering fight: at |wheel| 190-200° with 307-348 Nm grip,
  # mean B1 blend reached 0.88-0.94 (op committed to wheel-follow)
  # while the caster naturally returned the wheel — MDPS therefore
  # held an active torque target on every new frame. Driving STEER_REQ
  # to 0 in this regime lets the wheel coast freely on the caster.
  #   - ENTER_WHEEL_DEG = 40°: clearly above gentle-curve wheel range
  #     (highway typically <30°, lane changes <20°), so the latch
  #     never trips during ordinary lane-keeping.
  #   - ENTER_TORQUE_NM = 60: above the light-grip p90 (~92 Nm) so
  #     resting hands do not arm the latch, but well below the
  #     active-driver p25 (~250 Nm) range.
  #   - EXIT_TORQUE_NM  = 30: 30 Nm hysteresis band; sits comfortably
  #     above the ±5 Nm CAN noise floor so noise does not chatter.
  # Phase 14-1: the original 60/30 Nm thresholds sit inside the +90..180 Nm
  # column-torque offset band (hands-off |tq| p50=156/p90=284 on 0x2a-0x2d):
  # exit <30 Nm was near-impossible while moving (sticky passive, 69% of
  # gentle low-speed time) and the 30 Nm boundary flapped. Entry now keys on
  # debounced steeringPressed at the 40° geometry gate; exit is a sustained
  # let-go (!pressed & <260 Nm for EXIT_FRAMES), mirroring the 13a latch.
  # Phase 26: entry keys on the hold-compensated driver_pressed — the raw
  # pairing self-triggered in hands-off curves: a >= 40° curve at speed
  # generates hold torque past the raw pressed threshold (202 geo-entry
  # candidate episodes on 0x36-0x3d), dropping op passive mid-curve with
  # nobody on the wheel. The EXIT threshold stays at the raw 260: the exit
  # test only runs while angle-passive holds op passive, where Phase 26b
  # gates hold_comp to 0 — driver_tq == raw, and G1 measured the rolling-
  # passive hands-off bar at p50 147-233 Nm (sustained floor p50 158), so a
  # 160 exit would stick the latch in half the measured windows. 260 keeps
  # the validated 14-1 exit semantics unchanged.
  ANGLE_PASSIVE_ENTER_WHEEL_DEG = 40.0
  ANGLE_PASSIVE_EXIT_TORQUE_NM  = 260.0
  ANGLE_PASSIVE_EXIT_FRAMES     = 50      # 0.5 s sustained let-go
  # Phase 6f-3 low-speed intent-disagreement OR-arm. The 6d-1 wheel-angle
  # gate (>= 40°) misses the case where the driver pushes hard while the
  # wheel is still near-straight and op commands the opposite direction.
  # ccnc-drivelog routes 0x3c-0x3f (9899a611, 152 min): 137 sign-disagree
  # clusters over 13,285 frames covering 16-28% of low-speed op-active
  # time, with ~55% of clusters at |wheel|<10°. OR-arm catches them while
  # reusing the existing 5-frame sustain and torque-only exit.
  INTENT_DISAGREE_VEGO_MS    = 30.0 / 3.6   # ≤30 km/h
  # Phase 14-1: 30 -> 260 Nm. At 30 Nm the sign test read the sensor offset,
  # not the driver (a coin flip), and flapped around the low-torque tail.
  # 260 clears the offset so an arm means a REAL opposing push; 0.3 s sustain
  # replaces the shared 5-frame counter for this path.
  INTENT_DISAGREE_TQ_MIN_NM  = 160.0   # Phase 26: driver-domain (was raw 260)
  INTENT_DISAGREE_DELTA_DEG  = 5.0           # |apply_angle_last - wheel|
  INTENT_DISAGREE_SUSTAIN_FRAMES = 30        # 0.3 s
  # Phase 6e-1 transient-blip filter. The Phase 6d entry conjunction
  # is met by sub-50 ms wheel spikes (road bumps, sensor noise) when
  # combined with a driver's reactive grip — once latched, the
  # torque-only exit holds STEER_REQ=0 across the entire reactive
  # window even though no genuine driver-active turn occurred.
  # Require 5 consecutive frames (50 ms) of entry conjunction before
  # latching, mirroring the Phase 5e VM_REJECT_FORCE_PASSIVE_FRAMES
  # 1-counter pattern. Exit and stay-zone behaviour are unchanged.
  ANGLE_PASSIVE_MIN_ENTER_FRAMES = 5

  def __init__(self, CP):
    self.STEER_DELTA_UP = 3
    self.STEER_DELTA_DOWN = 7
    self.STEER_DRIVER_ALLOWANCE = 50
    self.STEER_DRIVER_MULTIPLIER = 2
    self.STEER_DRIVER_FACTOR = 1
    self.STEER_THRESHOLD = 150
    self.STEER_STEP = 1  # 100 Hz

    if CP.flags & HyundaiFlags.CANFD:
      self.STEER_MAX = 270
      self.STEER_DRIVER_ALLOWANCE = 250
      self.STEER_DRIVER_MULTIPLIER = 2
      self.STEER_THRESHOLD = 250
      self.STEER_DELTA_UP = 2
      self.STEER_DELTA_DOWN = 3
      if CP.flags & HyundaiFlags.CCNC:
        self.STEER_STEP = 1  # 100 Hz — matches panda safety frequency
        self.STEER_THRESHOLD = 350  # angle-control: EPS reaction inflates torque

    # To determine the limit for your car, find the maximum value that the stock LKAS will request.
    # If the max stock LKAS request is <384, add your car to this list.
    elif CP.carFingerprint in (CAR.GENESIS_G80, CAR.HYUNDAI_ELANTRA, CAR.HYUNDAI_ELANTRA_GT_I30, CAR.HYUNDAI_IONIQ,
                               CAR.HYUNDAI_IONIQ_EV_LTD, CAR.HYUNDAI_SANTA_FE_PHEV_2022, CAR.HYUNDAI_SONATA_LF, CAR.KIA_FORTE, CAR.KIA_NIRO_PHEV,
                               CAR.KIA_OPTIMA_H, CAR.KIA_OPTIMA_H_G4_FL, CAR.KIA_SORENTO):
      self.STEER_MAX = 255

    # these cars have significantly more torque than most HKG; limit to 70% of max
    elif CP.flags & HyundaiFlags.ALT_LIMITS:
      self.STEER_MAX = 270
      self.STEER_DELTA_UP = 2
      self.STEER_DELTA_DOWN = 3

    elif CP.flags & HyundaiFlags.ALT_LIMITS_2:
      self.STEER_MAX = 170
      self.STEER_DELTA_UP = 2
      self.STEER_DELTA_DOWN = 3

    # Default for most HKG
    else:
      self.STEER_MAX = 384


class HyundaiSafetyFlags(IntFlag):
  EV_GAS = 1
  HYBRID_GAS = 2
  LONG = 4
  CAMERA_SCC = 8
  CANFD_LKA_STEERING = 16
  CANFD_ALT_BUTTONS = 32
  ALT_LIMITS = 64
  CANFD_LKA_STEERING_ALT = 128
  FCEV_GAS = 256
  ALT_LIMITS_2 = 512
  CCNC = 1024


class HyundaiFlags(IntFlag):
  # Dynamic Flags

  # Default assumption: all cars use LFA (ADAS) steering from the camera.
  # CANFD_LKA_STEERING/CANFD_LKA_STEERING_ALT cars typically have both LKA (camera) and LFA (ADAS) steering messages,
  # with LKA commands forwarded to the ADAS DRV ECU.
  # Most HDA2 trims are assumed to be equipped with the ADAS DRV ECU, though some variants may not be equipped with one.
  CANFD_LKA_STEERING = 1
  CANFD_ALT_BUTTONS = 2
  CANFD_ALT_GEARS = 2 ** 2
  CANFD_CAMERA_SCC = 2 ** 3

  ALT_LIMITS = 2 ** 4
  ENABLE_BLINKERS = 2 ** 5
  CANFD_ALT_GEARS_2 = 2 ** 6
  SEND_LFA = 2 ** 7
  USE_FCA = 2 ** 8
  CANFD_LKA_STEERING_ALT = 2 ** 9

  # these cars use a different gas signal
  HYBRID = 2 ** 10
  EV = 2 ** 11

  # Static flags

  # If 0x500 is present on bus 1 it probably has a Mando radar outputting radar points.
  # If no points are outputted by default it might be possible to turn it on using  selfdrive/debug/hyundai_enable_radar_points.py
  MANDO_RADAR = 2 ** 12
  CANFD = 2 ** 13

  # The radar does SCC on these cars when HDA I, rather than the camera
  RADAR_SCC = 2 ** 14
  # The camera does SCC on these cars, rather than the radar
  CAMERA_SCC = 2 ** 15
  CHECKSUM_CRC8 = 2 ** 16
  CHECKSUM_6B = 2 ** 17

  # these cars require a special panda safety mode due to missing counters and checksums in the messages
  LEGACY = 2 ** 18

  # these cars have not been verified to work with longitudinal yet - radar disable, sending correct messages, etc.
  UNSUPPORTED_LONGITUDINAL = 2 ** 19

  # These CAN FD cars do not accept communication control to disable the ADAS ECU,
  # responds with 0x7F2822 - 'conditions not correct'
  CANFD_NO_RADAR_DISABLE = 2 ** 20

  CLUSTER_GEARS = 2 ** 21
  TCU_GEARS = 2 ** 22

  MIN_STEER_32_MPH = 2 ** 23

  HAS_LDA_BUTTON = 2 ** 24

  FCEV = 2 ** 25

  ALT_LIMITS_2 = 2 ** 26

  CCNC = 2 ** 27

  # These cars use different CAN addresses for doors, seatbelts, and blinkers
  CANFD_ALT_DOORS_BLINKERS = 2 ** 28


@dataclass
class HyundaiCarDocs(CarDocs):
  package: str = "Smart Cruise Control (SCC)"


@dataclass
class HyundaiNonSccCarDocs(CarDocs):
  package: str = "No Smart Cruise Control (Non-SCC)"
  support_type: SupportType = SupportType.COMMUNITY
  support_link: str = "community"


@dataclass
class HyundaiPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: "hyundai_kia_generic"})

  def init(self):
    if self.flags & HyundaiFlags.MANDO_RADAR:
      self.dbc_dict = {Bus.pt: "hyundai_kia_generic", Bus.radar: 'hyundai_kia_mando_front_radar_generated'}

    if self.flags & HyundaiFlags.MIN_STEER_32_MPH:
      self.specs = self.specs.override(minSteerSpeed=32 * CV.MPH_TO_MS)


@dataclass
class HyundaiNonSccPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: "hyundai_kia_generic"})

  def init(self):
    self.sp_flags |= HyundaiFlagsSP.NON_SCC

    if self.flags & HyundaiFlags.MIN_STEER_32_MPH:
      self.specs = self.specs.override(minSteerSpeed=32 * CV.MPH_TO_MS)


@dataclass
class HyundaiCanFDPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: "hyundai_canfd_generated"})

  def init(self):
    self.flags |= HyundaiFlags.CANFD


class CAR(Platforms):
  # Hyundai
  HYUNDAI_AZERA_6TH_GEN = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Azera 2022", "All", car_parts=CarParts.common([CarHarness.hyundai_k]))],
    CarSpecs(mass=1600, wheelbase=2.885, steerRatio=14.5),
  )
  HYUNDAI_AZERA_HEV_6TH_GEN = HyundaiPlatformConfig(
    [
      HyundaiCarDocs("Hyundai Azera Hybrid 2019", "All", car_parts=CarParts.common([CarHarness.hyundai_c])),
      HyundaiCarDocs("Hyundai Azera Hybrid 2020", "All", car_parts=CarParts.common([CarHarness.hyundai_k])),
    ],
    CarSpecs(mass=1675, wheelbase=2.885, steerRatio=14.5),
    flags=HyundaiFlags.HYBRID,
  )
  HYUNDAI_ELANTRA = HyundaiPlatformConfig(
    [
      # TODO: 2017-18 could be Hyundai G
      HyundaiCarDocs("Hyundai Elantra 2017-18", min_enable_speed=19 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_b])),
      HyundaiCarDocs("Hyundai Elantra 2019", min_enable_speed=19 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_g])),
    ],
    # steerRatio: 14 is Stock | Settled Params Learner values are steerRatio: 15.401566348670535, stiffnessFactor settled on 1.0081302973865127
    CarSpecs(mass=1275, wheelbase=2.7, steerRatio=15.4, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.LEGACY | HyundaiFlags.CLUSTER_GEARS | HyundaiFlags.MIN_STEER_32_MPH,
  )
  HYUNDAI_ELANTRA_GT_I30 = HyundaiPlatformConfig(
    [
      HyundaiCarDocs("Hyundai Elantra GT 2017-20", car_parts=CarParts.common([CarHarness.hyundai_e])),
      HyundaiCarDocs("Hyundai i30 2017-19", car_parts=CarParts.common([CarHarness.hyundai_e])),
    ],
    HYUNDAI_ELANTRA.specs,
    flags=HyundaiFlags.LEGACY | HyundaiFlags.CLUSTER_GEARS | HyundaiFlags.MIN_STEER_32_MPH,
  )
  HYUNDAI_ELANTRA_2021 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Elantra 2021-23", video="https://youtu.be/_EdYQtV52-c", car_parts=CarParts.common([CarHarness.hyundai_k]))],
    CarSpecs(mass=2800 * CV.LB_TO_KG, wheelbase=2.72, steerRatio=12.9, tireStiffnessFactor=0.65),
    flags=HyundaiFlags.CHECKSUM_CRC8,
  )
  HYUNDAI_ELANTRA_HEV_2021 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Elantra Hybrid 2021-23", video="https://youtu.be/_EdYQtV52-c",
                    car_parts=CarParts.common([CarHarness.hyundai_k]))],
    CarSpecs(mass=3017 * CV.LB_TO_KG, wheelbase=2.72, steerRatio=12.9, tireStiffnessFactor=0.65),
    flags=HyundaiFlags.CHECKSUM_CRC8 | HyundaiFlags.HYBRID,
  )
  HYUNDAI_GENESIS = HyundaiPlatformConfig(
    [
      # TODO: check 2015 packages
      HyundaiCarDocs("Hyundai Genesis 2015-16", min_enable_speed=19 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_j])),
      HyundaiCarDocs("Genesis G80 2017", "All", min_enable_speed=19 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_j])),
    ],
    CarSpecs(mass=2060, wheelbase=3.01, steerRatio=16.5, minSteerSpeed=60 * CV.KPH_TO_MS),
    flags=HyundaiFlags.CHECKSUM_6B | HyundaiFlags.LEGACY,
  )
  HYUNDAI_IONIQ = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Ioniq Hybrid 2017-19", car_parts=CarParts.common([CarHarness.hyundai_c]))],
    CarSpecs(mass=1490, wheelbase=2.7, steerRatio=13.73, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.HYBRID | HyundaiFlags.MIN_STEER_32_MPH,
  )
  HYUNDAI_IONIQ_HEV_2022 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Ioniq Hybrid 2020-22", car_parts=CarParts.common([CarHarness.hyundai_h]))],
    CarSpecs(mass=1490, wheelbase=2.7, steerRatio=13.73, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.HYBRID | HyundaiFlags.LEGACY,
  )
  HYUNDAI_IONIQ_EV_LTD = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Ioniq Electric 2019", car_parts=CarParts.common([CarHarness.hyundai_c]))],
    CarSpecs(mass=1490, wheelbase=2.7, steerRatio=13.73, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.EV | HyundaiFlags.LEGACY | HyundaiFlags.MIN_STEER_32_MPH,
  )
  HYUNDAI_IONIQ_EV_2020 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Ioniq Electric 2020", "All", car_parts=CarParts.common([CarHarness.hyundai_h]))],
    CarSpecs(mass=1490, wheelbase=2.7, steerRatio=13.73, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.EV,
  )
  HYUNDAI_IONIQ_PHEV_2019 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Ioniq Plug-in Hybrid 2019", car_parts=CarParts.common([CarHarness.hyundai_c]))],
    CarSpecs(mass=1490, wheelbase=2.7, steerRatio=13.73, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.HYBRID | HyundaiFlags.MIN_STEER_32_MPH,
  )
  HYUNDAI_IONIQ_PHEV = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Ioniq Plug-in Hybrid 2020-22", "All", car_parts=CarParts.common([CarHarness.hyundai_h]))],
    CarSpecs(mass=1490, wheelbase=2.7, steerRatio=13.73, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.HYBRID,
  )
  HYUNDAI_KONA = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Kona 2020", min_enable_speed=6 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_b]))],
    CarSpecs(mass=1275, wheelbase=2.6, steerRatio=13.42, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.CLUSTER_GEARS | HyundaiFlags.ALT_LIMITS,
  )
  HYUNDAI_KONA_2022 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Kona 2022-23", car_parts=CarParts.common([CarHarness.hyundai_o]))],
    CarSpecs(mass=1491, wheelbase=2.6, steerRatio=13.42, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.CAMERA_SCC | HyundaiFlags.ALT_LIMITS_2,
  )
  HYUNDAI_KONA_2ND_GEN = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Hyundai Kona (without HDA II) 2024-25", car_parts=CarParts.common([CarHarness.hyundai_l]))],
    CarSpecs(mass=1590, wheelbase=2.66, steerRatio=13.6, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.CCNC,
  )
  HYUNDAI_KONA_HEV_2ND_GEN = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Hyundai Kona Hybrid (without HDA II) 2024", car_parts=CarParts.common([CarHarness.hyundai_l]))],
    CarSpecs(mass=1590, wheelbase=2.66, steerRatio=13.6, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.CCNC,
  )
  HYUNDAI_KONA_EV = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Kona Electric 2018-21", car_parts=CarParts.common([CarHarness.hyundai_g]))],
    CarSpecs(mass=1685, wheelbase=2.6, steerRatio=13.42, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.EV | HyundaiFlags.ALT_LIMITS,
  )
  HYUNDAI_KONA_EV_2022 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Kona Electric 2022-23", car_parts=CarParts.common([CarHarness.hyundai_o]))],
    CarSpecs(mass=1743, wheelbase=2.6, steerRatio=13.42, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.CAMERA_SCC | HyundaiFlags.EV | HyundaiFlags.ALT_LIMITS,
  )
  HYUNDAI_KONA_EV_2ND_GEN = HyundaiCanFDPlatformConfig(
    [
      HyundaiCarDocs("Hyundai Kona Electric (with HDA II, Korea only) 2023", video="https://www.youtube.com/watch?v=U2fOCmcQ8hw",
                    car_parts=CarParts.common([CarHarness.hyundai_r])),
      HyundaiCarDocs("Hyundai Kona Electric (without HDA II) 2024", car_parts=CarParts.common([CarHarness.hyundai_a])),
    ],
    CarSpecs(mass=1740, wheelbase=2.66, steerRatio=13.6, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.EV | HyundaiFlags.CANFD_NO_RADAR_DISABLE | HyundaiFlags.CCNC,
  )
  HYUNDAI_KONA_HEV = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Kona Hybrid 2020", car_parts=CarParts.common([CarHarness.hyundai_i]))],  # TODO: check packages,
    CarSpecs(mass=1425, wheelbase=2.6, steerRatio=13.42, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.HYBRID | HyundaiFlags.ALT_LIMITS,
  )
  HYUNDAI_NEXO_1ST_GEN = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Nexo 2021", "All", car_parts=CarParts.common([CarHarness.hyundai_h]))],
    CarSpecs(mass=3990 * CV.LB_TO_KG, wheelbase=2.79, steerRatio=14.19),  # https://www.hyundainews.com/assets/documents/original/42768-2021NEXOProductGuideSpecs.pdf
    flags=HyundaiFlags.FCEV,
  )
  HYUNDAI_SANTA_FE = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Santa Fe 2019-20", "All", video="https://youtu.be/bjDR0YjM__s",
                    car_parts=CarParts.common([CarHarness.hyundai_d]))],
    CarSpecs(mass=3982 * CV.LB_TO_KG, wheelbase=2.766, steerRatio=16.55, tireStiffnessFactor=0.82),
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.CHECKSUM_CRC8,
  )
  HYUNDAI_SANTA_FE_2022 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Santa Fe 2021-23", "All", video="https://youtu.be/VnHzSTygTS4",
                    car_parts=CarParts.common([CarHarness.hyundai_l]))],
    HYUNDAI_SANTA_FE.specs,
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.CHECKSUM_CRC8,
  )
  HYUNDAI_SANTA_FE_HEV_2022 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Santa Fe Hybrid 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_l]))],
    HYUNDAI_SANTA_FE.specs,
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.CHECKSUM_CRC8 | HyundaiFlags.HYBRID,
  )
  HYUNDAI_SANTA_FE_PHEV_2022 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Santa Fe Plug-in Hybrid 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_l]))],
    HYUNDAI_SANTA_FE.specs,
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.CHECKSUM_CRC8 | HyundaiFlags.HYBRID,
  )
  HYUNDAI_SONATA = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Sonata 2020-23", "All", video="https://www.youtube.com/watch?v=ix63r9kE3Fw",
                   car_parts=CarParts.common([CarHarness.hyundai_a]))],
    CarSpecs(mass=1513, wheelbase=2.84, steerRatio=13.27 * 1.15, tireStiffnessFactor=0.65),  # 15% higher at the center seems reasonable
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.CHECKSUM_CRC8,
  )
  HYUNDAI_SONATA_2024 = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Hyundai Sonata (without HDA II) 2024-25", car_parts=CarParts.common([CarHarness.hyundai_a]))],
    CarSpecs(mass=1556, wheelbase=2.84, steerRatio=12.81),
    flags=HyundaiFlags.CCNC,
  )
  HYUNDAI_SONATA_LF = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Sonata 2018-19", car_parts=CarParts.common([CarHarness.hyundai_e]))],
    CarSpecs(mass=1536, wheelbase=2.804, steerRatio=13.27 * 1.15),  # 15% higher at the center seems reasonable
    flags=HyundaiFlags.UNSUPPORTED_LONGITUDINAL | HyundaiFlags.TCU_GEARS,
  )
  HYUNDAI_STARIA_4TH_GEN = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Hyundai Staria 2023", "All", car_parts=CarParts.common([CarHarness.hyundai_k]))],
    CarSpecs(mass=2205, wheelbase=3.273, steerRatio=11.94),  # https://www.hyundai.com/content/dam/hyundai/au/en/models/staria-load/premium-pip-update-2023/spec-sheet/STARIA_Load_Spec-Table_March_2023_v3.1.pdf
  )
  HYUNDAI_TUCSON = HyundaiPlatformConfig(
    [
      HyundaiCarDocs("Hyundai Tucson 2021", min_enable_speed=19 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_l])),
      HyundaiCarDocs("Hyundai Tucson Diesel 2019", car_parts=CarParts.common([CarHarness.hyundai_l])),
    ],
    CarSpecs(mass=3520 * CV.LB_TO_KG, wheelbase=2.67, steerRatio=16.1, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.TCU_GEARS,
  )
  HYUNDAI_PALISADE = HyundaiPlatformConfig(
    [
      HyundaiCarDocs("Hyundai Palisade 2020-22", "All", video="https://youtu.be/TAnDqjF4fDY?t=456", car_parts=CarParts.common([CarHarness.hyundai_h])),
      HyundaiCarDocs("Kia Telluride 2020-22", "All", car_parts=CarParts.common([CarHarness.hyundai_h])),
    ],
    CarSpecs(mass=1999, wheelbase=2.9, steerRatio=15.6 * 1.15, tireStiffnessFactor=0.63),
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.CHECKSUM_CRC8,
  )
  HYUNDAI_VELOSTER = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Veloster 2019-20", min_enable_speed=5. * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_e]))],
    CarSpecs(mass=2917 * CV.LB_TO_KG, wheelbase=2.8, steerRatio=13.75 * 1.15, tireStiffnessFactor=0.5),
    flags=HyundaiFlags.LEGACY | HyundaiFlags.TCU_GEARS,
  )
  HYUNDAI_SONATA_HYBRID = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Sonata Hybrid 2020-23", "All", car_parts=CarParts.common([CarHarness.hyundai_a]))],
    HYUNDAI_SONATA.specs,
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.CHECKSUM_CRC8 | HyundaiFlags.HYBRID,
  )
  HYUNDAI_SONATA_HEV_2024 = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Hyundai Sonata Hybrid (without HDA II) 2024-25", car_parts=CarParts.common([CarHarness.hyundai_a]))],
    CarSpecs(mass=1616, wheelbase=2.84, steerRatio=13.27),
    flags=HyundaiFlags.CCNC,
  )
  HYUNDAI_IONIQ_5 = HyundaiCanFDPlatformConfig(
    [
      HyundaiCarDocs("Hyundai Ioniq 5 (Southeast Asia and Europe only) 2022-24", "All", car_parts=CarParts.common([CarHarness.hyundai_q])),
      HyundaiCarDocs("Hyundai Ioniq 5 (without HDA II) 2022-24", "Highway Driving Assist", car_parts=CarParts.common([CarHarness.hyundai_k])),
      HyundaiCarDocs("Hyundai Ioniq 5 (with HDA II) 2022-24", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_q])),
    ],
    CarSpecs(mass=1948, wheelbase=2.97, steerRatio=14.26, tireStiffnessFactor=0.65),
    flags=HyundaiFlags.EV,
  )
  HYUNDAI_IONIQ_5_N = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Hyundai Ioniq 5 N (with HDA II) 2024", car_parts=CarParts.common([CarHarness.hyundai_s]))],
    CarSpecs(mass=2205, wheelbase=3.00, steerRatio=14.26, tireStiffnessFactor=1.3),
    flags=HyundaiFlags.EV | HyundaiFlags.CCNC,
  )
  HYUNDAI_IONIQ_6 = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Hyundai Ioniq 6 (with HDA II) 2023-24", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_p]))],
    HYUNDAI_IONIQ_5.specs,
    flags=HyundaiFlags.EV | HyundaiFlags.CANFD_NO_RADAR_DISABLE,
  )
  HYUNDAI_IONIQ_6_N = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Hyundai Ioniq 6 N (with HDA II) 2026", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_s]))],
    CarSpecs(mass=2175, wheelbase=2.965, steerRatio=14.96, tireStiffnessFactor=1.15),
    flags=HyundaiFlags.EV | HyundaiFlags.CANFD_NO_RADAR_DISABLE | HyundaiFlags.CCNC | HyundaiFlags.CANFD_ALT_BUTTONS | HyundaiFlags.CANFD_ALT_DOORS_BLINKERS,
  ) 
  HYUNDAI_TUCSON_4TH_GEN = HyundaiCanFDPlatformConfig(
    [
      HyundaiCarDocs("Hyundai Tucson 2022", car_parts=CarParts.common([CarHarness.hyundai_n])),
      HyundaiCarDocs("Hyundai Tucson 2023-24", "All", car_parts=CarParts.common([CarHarness.hyundai_n])),
      HyundaiCarDocs("Hyundai Tucson Hybrid 2022-24", "All", car_parts=CarParts.common([CarHarness.hyundai_n])),
      HyundaiCarDocs("Hyundai Tucson Plug-in Hybrid 2024", "All", car_parts=CarParts.common([CarHarness.hyundai_n])),
    ],
    CarSpecs(mass=1630, wheelbase=2.756, steerRatio=13.7, tireStiffnessFactor=0.385),
  )
  HYUNDAI_TUCSON_2025 = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Hyundai Tucson (without HDA II) 2025-26", car_parts=CarParts.common([CarHarness.hyundai_n]))],
    CarSpecs(mass=1630, wheelbase=2.756, steerRatio=13.7, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.CCNC,
  )
  HYUNDAI_TUCSON_HEV_2025 = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Hyundai Tucson Hybrid (without HDA II) 2025", car_parts=CarParts.common([CarHarness.hyundai_n]))],
    CarSpecs(mass=1630, wheelbase=2.756, steerRatio=13.7, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.CCNC,
  )
  HYUNDAI_SANTA_CRUZ_1ST_GEN = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Hyundai Santa Cruz 2022-24", car_parts=CarParts.common([CarHarness.hyundai_n]))],
    # weight from Limited trim - the only supported trim, steering ratio according to Hyundai News https://www.hyundainews.com/assets/documents/original/48035-2022SantaCruzProductGuideSpecsv2081521.pdf
    CarSpecs(mass=1870, wheelbase=3, steerRatio=14.2),
  )
  HYUNDAI_SANTA_CRUZ_2025 = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Hyundai Santa Cruz (without HDA II) 2025", car_parts=CarParts.common([CarHarness.hyundai_n]))],
    CarSpecs(mass=1920, wheelbase=3, steerRatio=14.2),
    flags=HyundaiFlags.CCNC,
  )
  HYUNDAI_CUSTIN_1ST_GEN = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Custin 2023", "All", car_parts=CarParts.common([CarHarness.hyundai_k]))],
    CarSpecs(mass=1690, wheelbase=3.055, steerRatio=17),  # mass: from https://www.hyundai-motor.com.tw/clicktobuy/custin#spec_0, steerRatio: from learner
    flags=HyundaiFlags.CHECKSUM_CRC8,
  )

  # Kia
  KIA_FORTE = HyundaiPlatformConfig(
    [
      HyundaiCarDocs("Kia Forte 2019-21", min_enable_speed=6 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_g])),
      HyundaiCarDocs("Kia Forte 2022-23", car_parts=CarParts.common([CarHarness.hyundai_e])),
    ],
    CarSpecs(mass=2878 * CV.LB_TO_KG, wheelbase=2.8, steerRatio=13.75, tireStiffnessFactor=0.5)
  )
  KIA_K4_2025 = HyundaiCanFDPlatformConfig(
    [
      HyundaiCarDocs("Kia K4 (without HDA II) 2025", car_parts=CarParts.common([CarHarness.hyundai_a])),
      HyundaiCarDocs("Kia K4 (with HDA II) 2025", car_parts=CarParts.common([CarHarness.hyundai_r])),
    ],
    CarSpecs(mass=2987 * CV.LB_TO_KG, wheelbase=2.72, steerRatio=13.4),
    flags=HyundaiFlags.CCNC,
  )
  KIA_K5_2021 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia K5 2021-24", car_parts=CarParts.common([CarHarness.hyundai_a]))],
    CarSpecs(mass=3381 * CV.LB_TO_KG, wheelbase=2.85, steerRatio=13.27, tireStiffnessFactor=0.5),  # 2021 Kia K5 Steering Ratio (all trims)
    flags=HyundaiFlags.CHECKSUM_CRC8,
  )
  KIA_K5_2025 = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Kia K5 (without HDA II) 2025", car_parts=CarParts.common([CarHarness.hyundai_m]))],
    CarSpecs(mass=3230 * CV.LB_TO_KG, wheelbase=2.85, steerRatio=13.27),
    flags=HyundaiFlags.CCNC,
  )
  KIA_K5_HEV_2020 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia K5 Hybrid 2020-22", car_parts=CarParts.common([CarHarness.hyundai_a]))],
    KIA_K5_2021.specs,
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.CHECKSUM_CRC8 | HyundaiFlags.HYBRID,
  )
  KIA_K7_2017 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia K7 2017", car_parts=CarParts.common([CarHarness.hyundai_c]))],
    CarSpecs(mass=1648, wheelbase=2.86, steerRatio=16.8),
  )
  KIA_K8_HEV_1ST_GEN = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Kia K8 Hybrid (with HDA II) 2023", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_q]))],
    # mass: https://carprices.ae/brands/kia/2023/k8/1.6-turbo-hybrid, steerRatio: guesstimate from K5 platform
    CarSpecs(mass=1630, wheelbase=2.895, steerRatio=13.27)
  )
  KIA_NIRO_EV = HyundaiPlatformConfig(
    [
      HyundaiCarDocs("Kia Niro EV 2019", "All", video="https://www.youtube.com/watch?v=lT7zcG6ZpGo", car_parts=CarParts.common([CarHarness.hyundai_h])),
      HyundaiCarDocs("Kia Niro EV 2020", "All", video="https://www.youtube.com/watch?v=lT7zcG6ZpGo", car_parts=CarParts.common([CarHarness.hyundai_f])),
      HyundaiCarDocs("Kia Niro EV 2021", "All", video="https://www.youtube.com/watch?v=lT7zcG6ZpGo", car_parts=CarParts.common([CarHarness.hyundai_c])),
      HyundaiCarDocs("Kia Niro EV 2022", "All", video="https://www.youtube.com/watch?v=lT7zcG6ZpGo", car_parts=CarParts.common([CarHarness.hyundai_h])),
    ],
    CarSpecs(mass=3543 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=13.6, tireStiffnessFactor=0.385),  # average of all the cars
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.EV,
  )
  KIA_NIRO_EV_2ND_GEN = HyundaiCanFDPlatformConfig(
    [
      HyundaiCarDocs("Kia Niro EV (without HDA II) 2023-25", "All", car_parts=CarParts.common([CarHarness.hyundai_a])),
      HyundaiCarDocs("Kia Niro EV (with HDA II) 2025", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_r])),
    ],
    KIA_NIRO_EV.specs,
    flags=HyundaiFlags.EV,
  )
  KIA_NIRO_PHEV = HyundaiPlatformConfig(
    [
      HyundaiCarDocs("Kia Niro Hybrid 2018", min_enable_speed=10. * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_c])),
      HyundaiCarDocs("Kia Niro Plug-in Hybrid 2018-19", "All", min_enable_speed=10. * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_c])),
      HyundaiCarDocs("Kia Niro Plug-in Hybrid 2020", car_parts=CarParts.common([CarHarness.hyundai_d])),
    ],
    KIA_NIRO_EV.specs,
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.HYBRID | HyundaiFlags.UNSUPPORTED_LONGITUDINAL | HyundaiFlags.MIN_STEER_32_MPH,
  )
  KIA_NIRO_PHEV_2022 = HyundaiPlatformConfig(
    [
      HyundaiCarDocs("Kia Niro Plug-in Hybrid 2021", car_parts=CarParts.common([CarHarness.hyundai_d])),
      HyundaiCarDocs("Kia Niro Plug-in Hybrid 2022", car_parts=CarParts.common([CarHarness.hyundai_f])),
    ],
    KIA_NIRO_EV.specs,
    flags=HyundaiFlags.HYBRID | HyundaiFlags.MANDO_RADAR,
  )
  KIA_NIRO_HEV_2021 = HyundaiPlatformConfig(
    [
      HyundaiCarDocs("Kia Niro Hybrid 2021", car_parts=CarParts.common([CarHarness.hyundai_d])),
      HyundaiCarDocs("Kia Niro Hybrid 2022", car_parts=CarParts.common([CarHarness.hyundai_f])),
    ],
    KIA_NIRO_EV.specs,
    flags=HyundaiFlags.HYBRID,
  )
  KIA_NIRO_HEV_2ND_GEN = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Kia Niro Hybrid 2023", car_parts=CarParts.common([CarHarness.hyundai_a]))],
    KIA_NIRO_EV.specs,
  )
  KIA_OPTIMA_G4 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia Optima 2017", "Advanced Smart Cruise Control",
                    car_parts=CarParts.common([CarHarness.hyundai_b]))],  # TODO: may support 2016, 2018
    CarSpecs(mass=3558 * CV.LB_TO_KG, wheelbase=2.8, steerRatio=13.75, tireStiffnessFactor=0.5),
    flags=HyundaiFlags.LEGACY | HyundaiFlags.TCU_GEARS | HyundaiFlags.MIN_STEER_32_MPH,
  )
  KIA_OPTIMA_G4_FL = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia Optima 2019-20", car_parts=CarParts.common([CarHarness.hyundai_g]))],
    CarSpecs(mass=3558 * CV.LB_TO_KG, wheelbase=2.8, steerRatio=13.75, tireStiffnessFactor=0.5),
    flags=HyundaiFlags.UNSUPPORTED_LONGITUDINAL | HyundaiFlags.TCU_GEARS,
  )
  # TODO: may support adjacent years. may have a non-zero minimum steering speed
  KIA_OPTIMA_H = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia Optima Hybrid 2017", "Advanced Smart Cruise Control", car_parts=CarParts.common([CarHarness.hyundai_c]))],
    CarSpecs(mass=3558 * CV.LB_TO_KG, wheelbase=2.8, steerRatio=13.75, tireStiffnessFactor=0.5),
    flags=HyundaiFlags.HYBRID | HyundaiFlags.LEGACY,
  )
  KIA_OPTIMA_H_G4_FL = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia Optima Hybrid 2019", car_parts=CarParts.common([CarHarness.hyundai_h]))],
    CarSpecs(mass=3558 * CV.LB_TO_KG, wheelbase=2.8, steerRatio=13.75, tireStiffnessFactor=0.5),
    flags=HyundaiFlags.HYBRID | HyundaiFlags.UNSUPPORTED_LONGITUDINAL,
  )
  KIA_SELTOS = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia Seltos 2021", car_parts=CarParts.common([CarHarness.hyundai_a]))],
    CarSpecs(mass=1337, wheelbase=2.63, steerRatio=14.56),
    flags=HyundaiFlags.CHECKSUM_CRC8,
  )
  KIA_SPORTAGE_5TH_GEN = HyundaiCanFDPlatformConfig(
    [
      HyundaiCarDocs("Kia Sportage 2023-24", car_parts=CarParts.common([CarHarness.hyundai_n])),
      HyundaiCarDocs("Kia Sportage Hybrid 2023", car_parts=CarParts.common([CarHarness.hyundai_n])),
    ],
    # weight from SX and above trims, average of FWD and AWD version, steering ratio according to Kia News https://www.kiamedia.com/us/en/models/sportage/2023/specifications
    CarSpecs(mass=1725, wheelbase=2.756, steerRatio=13.6),
  )
  KIA_SPORTAGE_HEV_2026 = HyundaiCanFDPlatformConfig(
    [
      HyundaiCarDocs("Kia Sportage Hybrid 2026", car_parts=CarParts.common([CarHarness.hyundai_n])),
    ],
    CarSpecs(mass=1812, wheelbase=2.756, steerRatio=13.7),
  )
  KIA_SORENTO = HyundaiPlatformConfig(
    [
      HyundaiCarDocs("Kia Sorento 2018", "Advanced Smart Cruise Control & LKAS", video="https://www.youtube.com/watch?v=Fkh3s6WHJz8",
                     car_parts=CarParts.common([CarHarness.hyundai_e])),
      HyundaiCarDocs("Kia Sorento 2019", video="https://www.youtube.com/watch?v=Fkh3s6WHJz8", car_parts=CarParts.common([CarHarness.hyundai_e])),
    ],
    CarSpecs(mass=1985, wheelbase=2.78, steerRatio=14.4 * 1.1),  # 10% higher at the center seems reasonable
    flags=HyundaiFlags.CHECKSUM_6B | HyundaiFlags.UNSUPPORTED_LONGITUDINAL,
  )
  KIA_SORENTO_4TH_GEN = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Kia Sorento 2021-23", car_parts=CarParts.common([CarHarness.hyundai_k]))],
    CarSpecs(mass=3957 * CV.LB_TO_KG, wheelbase=2.81, steerRatio=13.5),  # average of the platforms
    flags=HyundaiFlags.RADAR_SCC,
  )
  KIA_SORENTO_2024 = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Kia Sorento (without HDA II) 2024-25", car_parts=CarParts.common([CarHarness.hyundai_a]))],
    CarSpecs(mass=3957 * CV.LB_TO_KG, wheelbase=2.81, steerRatio=13.5),
    flags=HyundaiFlags.CCNC,
  )
  KIA_SORENTO_HEV_4TH_GEN = HyundaiCanFDPlatformConfig(
    [
      HyundaiCarDocs("Kia Sorento Hybrid 2021-23", "All", car_parts=CarParts.common([CarHarness.hyundai_a])),
      HyundaiCarDocs("Kia Sorento Plug-in Hybrid 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_a])),
    ],
    CarSpecs(mass=4395 * CV.LB_TO_KG, wheelbase=2.81, steerRatio=13.5),  # average of the platforms
    flags=HyundaiFlags.RADAR_SCC,
  )
  KIA_STINGER = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia Stinger 2018-20", video="https://www.youtube.com/watch?v=MJ94qoofYw0",
                    car_parts=CarParts.common([CarHarness.hyundai_c]))],
    CarSpecs(mass=1825, wheelbase=2.78, steerRatio=14.4 * 1.15)  # 15% higher at the center seems reasonable
  )
  KIA_STINGER_2022 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia Stinger 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_k]))],
    KIA_STINGER.specs,
  )
  KIA_CEED = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia Ceed 2019-21", car_parts=CarParts.common([CarHarness.hyundai_e]))],
    CarSpecs(mass=1450, wheelbase=2.65, steerRatio=13.75, tireStiffnessFactor=0.5),
    flags=HyundaiFlags.LEGACY,
  )
  KIA_EV6 = HyundaiCanFDPlatformConfig(
    [
      HyundaiCarDocs("Kia EV6 (Southeast Asia only) 2022-24", "All", car_parts=CarParts.common([CarHarness.hyundai_p])),
      HyundaiCarDocs("Kia EV6 (without HDA II) 2022-24", "Highway Driving Assist", car_parts=CarParts.common([CarHarness.hyundai_l])),
      HyundaiCarDocs("Kia EV6 (with HDA II) 2022-24", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_p]))
    ],
    CarSpecs(mass=2055, wheelbase=2.9, steerRatio=16, tireStiffnessFactor=0.65),
    flags=HyundaiFlags.EV,
  )
  KIA_CARNIVAL_4TH_GEN = HyundaiCanFDPlatformConfig(
    [
      HyundaiCarDocs("Kia Carnival 2022-24", car_parts=CarParts.common([CarHarness.hyundai_a])),
      HyundaiCarDocs("Kia Carnival (China only) 2023", car_parts=CarParts.common([CarHarness.hyundai_k]))
    ],
    CarSpecs(mass=2087, wheelbase=3.09, steerRatio=14.23),
    flags=HyundaiFlags.RADAR_SCC,
  )

  # Genesis
  GENESIS_GV60_EV_1ST_GEN = HyundaiCanFDPlatformConfig(
    [
      HyundaiCarDocs("Genesis GV60 (Advanced Trim) 2023", "All", car_parts=CarParts.common([CarHarness.hyundai_a])),
      HyundaiCarDocs("Genesis GV60 (Performance Trim) 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_k])),
    ],
    CarSpecs(mass=2205, wheelbase=2.9, steerRatio=17.6),
    flags=HyundaiFlags.EV,
  )
  GENESIS_G70 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Genesis G70 2018", "All", car_parts=CarParts.common([CarHarness.hyundai_f]))],
    CarSpecs(mass=1640, wheelbase=2.84, steerRatio=13.56),
    flags=HyundaiFlags.LEGACY,
  )
  GENESIS_G70_2020 = HyundaiPlatformConfig(
    [
      # TODO: 2021 MY harness is unknown
      HyundaiCarDocs("Genesis G70 2019-21", "All", car_parts=CarParts.common([CarHarness.hyundai_f])),
      # TODO: From 3.3T Sport Advanced 2022 & Prestige 2023 Trim, 2.0T is unknown
      HyundaiCarDocs("Genesis G70 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_l])),
    ],
    GENESIS_G70.specs,
    flags=HyundaiFlags.MANDO_RADAR,
  )
  GENESIS_GV70_1ST_GEN = HyundaiCanFDPlatformConfig(
    [
      # TODO: Hyundai P is likely the correct harness for HDA II for 2.5T (unsupported due to missing ADAS ECU, is that the radar?)
      HyundaiCarDocs("Genesis GV70 (2.5T Trim, without HDA II) 2022-24", "All", car_parts=CarParts.common([CarHarness.hyundai_l])),
      HyundaiCarDocs("Genesis GV70 (3.5T Trim, without HDA II) 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_m])),
    ],
    CarSpecs(mass=1950, wheelbase=2.87, steerRatio=14.6),
    flags=HyundaiFlags.RADAR_SCC,
  )
  GENESIS_GV70_ELECTRIFIED_1ST_GEN = HyundaiCanFDPlatformConfig(
    [
      HyundaiCarDocs("Genesis GV70 Electrified (Australia Only) 2022", "All", car_parts=CarParts.common([CarHarness.hyundai_q])),
      HyundaiCarDocs("Genesis GV70 Electrified (with HDA II) 2023-24", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_q])),
    ],
    CarSpecs(mass=2260, wheelbase=2.87, steerRatio=17.1),
    flags=HyundaiFlags.EV,
  )
  GENESIS_G80 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Genesis G80 2018-19", "All", car_parts=CarParts.common([CarHarness.hyundai_h]))],
    CarSpecs(mass=2060, wheelbase=3.01, steerRatio=16.5),
    flags=HyundaiFlags.LEGACY,
  )
  GENESIS_G80_2ND_GEN_FL = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Genesis G80 (2.5T Advanced Trim, with HDA II) 2024", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_p]))],
    CarSpecs(mass=2060, wheelbase=3.00, steerRatio=14.0),
  )
  GENESIS_G90 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Genesis G90 2017-20", "All", car_parts=CarParts.common([CarHarness.hyundai_c]))],
    CarSpecs(mass=2200, wheelbase=3.15, steerRatio=12.069),
  )
  GENESIS_GV80 = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Genesis GV80 2023", "All", car_parts=CarParts.common([CarHarness.hyundai_m]))],
    CarSpecs(mass=2258, wheelbase=2.95, steerRatio=14.14),
    flags=HyundaiFlags.RADAR_SCC,
  )

  # port extensions
  HYUNDAI_BAYON_1ST_GEN_NON_SCC = HyundaiNonSccPlatformConfig(
    [HyundaiNonSccCarDocs("Hyundai Bayon Non-SCC 2021", car_parts=CarParts.common([CarHarness.hyundai_n]))],
    CarSpecs(mass=1150, wheelbase=2.58, steerRatio=13.27 * 1.15),
    flags=HyundaiFlags.CHECKSUM_CRC8,
  )
  HYUNDAI_ELANTRA_2022_NON_SCC = HyundaiNonSccPlatformConfig(
    [HyundaiNonSccCarDocs("Hyundai Elantra Non-SCC 2022", car_parts=CarParts.common([CarHarness.hyundai_k]))],
    HYUNDAI_ELANTRA_2021.specs,
    flags=HyundaiFlags.CHECKSUM_CRC8,
  )
  HYUNDAI_KONA_NON_SCC = HyundaiNonSccPlatformConfig(
    [HyundaiNonSccCarDocs("Hyundai Kona Non-SCC 2019", car_parts=CarParts.common([CarHarness.hyundai_b]))],
    HYUNDAI_KONA.specs,
    flags=HyundaiFlags.ALT_LIMITS,
  )
  HYUNDAI_KONA_EV_NON_SCC = HyundaiNonSccPlatformConfig(
    [HyundaiNonSccCarDocs("Hyundai Kona Electric Non-SCC 2019", car_parts=CarParts.common([CarHarness.hyundai_g]))],
    HYUNDAI_KONA_EV.specs,
    flags=HyundaiFlags.EV | HyundaiFlags.ALT_LIMITS,
  )
  KIA_CEED_PHEV_2022_NON_SCC = HyundaiNonSccPlatformConfig(
    [HyundaiNonSccCarDocs("Kia Ceed Plug-in Hybrid Non-SCC 2022", car_parts=CarParts.common([CarHarness.hyundai_i]))],
    CarSpecs(mass=1650, wheelbase=2.65, steerRatio=13.75, tireStiffnessFactor=0.5),
    flags=HyundaiFlags.HYBRID,
  )
  KIA_FORTE_2019_NON_SCC = HyundaiNonSccPlatformConfig(
    [HyundaiNonSccCarDocs("Kia Forte Non-SCC 2019", car_parts=CarParts.common([CarHarness.hyundai_g]))],
    KIA_FORTE.specs,
    sp_flags=HyundaiFlagsSP.NON_SCC_NO_FCA,
  )
  KIA_FORTE_2021_NON_SCC = HyundaiNonSccPlatformConfig(
    [HyundaiNonSccCarDocs("Kia Forte Non-SCC 2021", car_parts=CarParts.common([CarHarness.hyundai_g]))],
    KIA_FORTE.specs,
  )
  KIA_SELTOS_2023_NON_SCC = HyundaiNonSccPlatformConfig(
    [HyundaiNonSccCarDocs("Kia Seltos Non-SCC 2023-24", car_parts=CarParts.common([CarHarness.hyundai_l]))],
    KIA_SELTOS.specs,
    flags=HyundaiFlags.CHECKSUM_CRC8,
  )
  GENESIS_G70_2021_NON_SCC = HyundaiNonSccPlatformConfig(
    [HyundaiNonSccCarDocs("Genesis G70 Non-SCC 2021", car_parts=CarParts.common([CarHarness.hyundai_f]))],
    GENESIS_G70_2020.specs,
    flags=HyundaiFlags.CHECKSUM_CRC8,
    sp_flags=HyundaiFlagsSP.NON_SCC_RADAR_FCA,
  )


class Buttons:
  NONE = 0
  RES_ACCEL = 1
  SET_DECEL = 2
  GAP_DIST = 3
  CANCEL = 4  # on newer models, this is a pause/resume button


def get_platform_codes(fw_versions: list[bytes]) -> set[tuple[bytes, bytes | None]]:
  # Returns unique, platform-specific identification codes for a set of versions
  codes = set()  # (code-Optional[part], date)
  for fw in fw_versions:
    code_match = PLATFORM_CODE_FW_PATTERN.search(fw)
    part_match = PART_NUMBER_FW_PATTERN.search(fw)
    date_match = DATE_FW_PATTERN.search(fw)
    if code_match is not None:
      code: bytes = code_match.group()
      part = part_match.group() if part_match else None
      date = date_match.group() if date_match else None
      if part is not None:
        # part number starts with generic ECU part type, add what is specific to platform
        code += b"-" + part[-5:]

      codes.add((code, date))
  return codes


def match_fw_to_car_fuzzy(live_fw_versions, vin, offline_fw_versions) -> set[str]:
  # Non-electric CAN FD platforms often do not have platform code specifiers needed
  # to distinguish between hybrid and ICE. All EVs so far are either exclusively
  # electric or specify electric in the platform code.
  fuzzy_platform_blacklist = {str(c) for c in (CANFD_CAR - EV_CAR - CANFD_FUZZY_WHITELIST)}
  candidates: set[str] = set()

  for candidate, fws in offline_fw_versions.items():
    # Keep track of ECUs which pass all checks (platform codes, within date range)
    valid_found_ecus = set()
    valid_expected_ecus = {ecu[1:] for ecu in fws if ecu[0] in PLATFORM_CODE_ECUS}
    for ecu, expected_versions in fws.items():
      addr = ecu[1:]
      # Only check ECUs expected to have platform codes
      if ecu[0] not in PLATFORM_CODE_ECUS:
        continue

      # Expected platform codes & dates
      codes = get_platform_codes(expected_versions)
      expected_platform_codes = {code for code, _ in codes}
      expected_dates = {date for _, date in codes if date is not None}

      # Found platform codes & dates
      codes = get_platform_codes(live_fw_versions.get(addr, set()))
      found_platform_codes = {code for code, _ in codes}
      found_dates = {date for _, date in codes if date is not None}

      # Check platform code + part number matches for any found versions
      if not any(found_platform_code in expected_platform_codes for found_platform_code in found_platform_codes):
        break

      if ecu[0] in DATE_FW_ECUS:
        # If ECU can have a FW date, require it to exist
        # (this excludes candidates in the database without dates)
        if not len(expected_dates) or not len(found_dates):
          break

        # Check any date within range in the database, format is %y%m%d
        if not any(min(expected_dates) <= found_date <= max(expected_dates) for found_date in found_dates):
          break

      valid_found_ecus.add(addr)

    # If all live ECUs pass all checks for candidate, add it as a match
    if valid_expected_ecus.issubset(valid_found_ecus):
      candidates.add(candidate)

  return candidates - fuzzy_platform_blacklist


HYUNDAI_VERSION_REQUEST_LONG = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xf100)  # Long description

HYUNDAI_VERSION_REQUEST_ALT = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xf110)  # Alt long description

HYUNDAI_ECU_MANUFACTURING_DATE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.ECU_MANUFACTURING_DATE)

HYUNDAI_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40])

# Regex patterns for parsing platform code, FW date, and part number from FW versions
PLATFORM_CODE_FW_PATTERN = re.compile(b'((?<=' + HYUNDAI_VERSION_REQUEST_LONG[1:] +
                                      b')[A-Z]{2}[A-Za-z0-9]{0,2})')
DATE_FW_PATTERN = re.compile(b'(?<=[ -])([0-9]{6}$)')
PART_NUMBER_FW_PATTERN = re.compile(b'(?<=[0-9][.,][0-9]{2} )([0-9]{5}[-/]?[A-Z][A-Z0-9]{3}[0-9])')

# We've seen both ICE and hybrid for these platforms, and they have hybrid descriptors (e.g. MQ4 vs MQ4H)
CANFD_FUZZY_WHITELIST = {CAR.KIA_SORENTO_4TH_GEN, CAR.KIA_SORENTO_HEV_4TH_GEN, CAR.KIA_K8_HEV_1ST_GEN,
                         # TODO: the hybrid variant is not out yet
                         CAR.KIA_CARNIVAL_4TH_GEN}

# List of ECUs expected to have platform codes, camera and radar should exist on all cars
# TODO: use abs, it has the platform code and part number on many platforms
PLATFORM_CODE_ECUS = [Ecu.fwdRadar, Ecu.fwdCamera, Ecu.eps]
# So far we've only seen dates in fwdCamera
# TODO: there are date codes in the ABS firmware versions in hex
DATE_FW_ECUS = [Ecu.fwdCamera]

# Note: an ECU on CAN FD cars may sometimes send 0x30080aaaaaaaaaaa (flow control continue) while we
# are attempting to query ECUs. This currently does not seem to affect fingerprinting from the camera
FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    # TODO: add back whitelists
    # CAN queries (OBD-II port)
    Request(
      [HYUNDAI_VERSION_REQUEST_LONG],
      [HYUNDAI_VERSION_RESPONSE],
    ),

    # CAN & CAN-FD queries (from camera)
    Request(
      [HYUNDAI_VERSION_REQUEST_LONG],
      [HYUNDAI_VERSION_RESPONSE],
      bus=0,
      auxiliary=True,
    ),
    Request(
      [HYUNDAI_VERSION_REQUEST_LONG],
      [HYUNDAI_VERSION_RESPONSE],
      bus=1,
      auxiliary=True,
      obd_multiplexing=False,
    ),

    # CAN & CAN FD query to understand the three digit date code
    # LKA steering cars usually use 6 digit date codes, so skip bus 1
    Request(
      [HYUNDAI_ECU_MANUFACTURING_DATE],
      [HYUNDAI_VERSION_RESPONSE],
      bus=0,
      auxiliary=True,
      logging=True,
    ),

    # CAN-FD alt request logging queries for hvac and parkingAdas
    Request(
      [HYUNDAI_VERSION_REQUEST_ALT],
      [HYUNDAI_VERSION_RESPONSE],
      bus=0,
      auxiliary=True,
      logging=True,
    ),
    Request(
      [HYUNDAI_VERSION_REQUEST_ALT],
      [HYUNDAI_VERSION_RESPONSE],
      bus=1,
      auxiliary=True,
      logging=True,
      obd_multiplexing=False,
    ),
  ],
  # We lose these ECUs without the comma power on these cars.
  # Note that we still attempt to match with them when they are present
  non_essential_ecus={
    Ecu.abs: [CAR.HYUNDAI_PALISADE, CAR.HYUNDAI_SONATA, CAR.HYUNDAI_SANTA_FE_2022, CAR.KIA_K5_2021, CAR.HYUNDAI_ELANTRA_2021,
              CAR.HYUNDAI_SANTA_FE, CAR.HYUNDAI_KONA_EV_2022, CAR.HYUNDAI_KONA_EV, CAR.HYUNDAI_CUSTIN_1ST_GEN, CAR.KIA_SORENTO,
              CAR.KIA_CEED, CAR.KIA_SELTOS],
  },
  extra_ecus=[
    (Ecu.adas, 0x730, None),              # ADAS Driving ECU on platforms with LKA steering
    (Ecu.parkingAdas, 0x7b1, None),       # ADAS Parking ECU (may exist on all platforms)
    (Ecu.hvac, 0x7b3, None),              # HVAC Control Assembly
    (Ecu.cornerRadar, 0x7b7, None),
    (Ecu.combinationMeter, 0x7c6, None),  # CAN FD Instrument cluster
  ],
  # Custom fuzzy fingerprinting function using platform codes, part numbers + FW dates:
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
)

CHECKSUM = {
  "crc8": CAR.with_flags(HyundaiFlags.CHECKSUM_CRC8),
  "6B": CAR.with_flags(HyundaiFlags.CHECKSUM_6B),
}

CAN_GEARS = {
  # which message has the gear. hybrid and EV use ELECT_GEAR
  "use_cluster_gears": CAR.with_flags(HyundaiFlags.CLUSTER_GEARS),
  "use_tcu_gears": CAR.with_flags(HyundaiFlags.TCU_GEARS),
}

CANFD_CAR = CAR.with_flags(HyundaiFlags.CANFD)
CANFD_RADAR_SCC_CAR = CAR.with_flags(HyundaiFlags.RADAR_SCC)  # TODO: merge with UNSUPPORTED_LONGITUDINAL_CAR

CANFD_UNSUPPORTED_LONGITUDINAL_CAR = CAR.with_flags(HyundaiFlags.CANFD_NO_RADAR_DISABLE)  # TODO: merge with UNSUPPORTED_LONGITUDINAL_CAR

CAMERA_SCC_CAR = CAR.with_flags(HyundaiFlags.CAMERA_SCC)

HYBRID_CAR = CAR.with_flags(HyundaiFlags.HYBRID)

EV_CAR = CAR.with_flags(HyundaiFlags.EV)

LEGACY_SAFETY_MODE_CAR = CAR.with_flags(HyundaiFlags.LEGACY)

# TODO: another PR with (HyundaiFlags.LEGACY | HyundaiFlags.UNSUPPORTED_LONGITUDINAL | HyundaiFlags.CAMERA_SCC |
#       HyundaiFlags.CANFD_RADAR_SCC | HyundaiFlags.CANFD_NO_RADAR_DISABLE | )
UNSUPPORTED_LONGITUDINAL_CAR = CAR.with_flags(HyundaiFlags.LEGACY) | CAR.with_flags(HyundaiFlags.UNSUPPORTED_LONGITUDINAL)

# port extensions
NON_SCC_CAR = CAR.with_sp_flags(HyundaiFlagsSP.NON_SCC)

DBC = CAR.create_dbc_map()
