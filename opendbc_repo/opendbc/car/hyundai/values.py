import re
from dataclasses import dataclass, field
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
  BLINKER_ANCHOR_TORQUE_NM = 220.0
  # Phase 14-2: stateful anchor. The single 220 Nm test flapped 3.84x/s in
  # low-speed blinker waits (0x2e-0x2f) — fire after FIRE_FRAMES sustained
  # >= 220, hold while >= RELEASE_NM (40 Nm band) with a minimum hold.
  BLINKER_ANCHOR_RELEASE_NM      = 180.0
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
  ACIGAIN_BLINKER_GATE_START_NM = 220.0
  ACIGAIN_BLINKER_GATE_FULL_NM  = 300.0
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
  CMD_HYSTERESIS_DEG = 0.15
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
  LOW_SPEED_GRIP_RELEASE_NM        = 260.0
  LOW_SPEED_GRIP_RELEASE_FRAMES    = 50     # 0.5 s sustained let-go to resume
  # Phase 14-3: single 40° boundary -> 45/35 hysteresis pair (lane-keep scale;
  # intersection turns are 100°+). ~10% of residual low-speed flips on
  # 0x2e-0x2f clustered at the old single threshold.
  LOW_SPEED_CMD_PASSIVE_DEG        = 45.0   # exceed while steering -> go passive
  LOW_SPEED_CMD_ACTIVE_DEG         = 35.0   # fall below while passive -> re-engage
  LOW_SPEED_SCEN_TO_PASSIVE_FRAMES = 30     # 0.3 s to yield
  LOW_SPEED_SCEN_TO_ACTIVE_FRAMES  = 100    # 1.0 s to (re)engage
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
  ACIGAIN_GRIP_FULL_NM = 260.0    # torque (when pressed) at which authority hits the floor
  ACIGAIN_GRIP_FLOOR   = 0.10     # min ACIGain under real grip (legacy 0.19 @ 350 Nm)

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
  INTENT_DISAGREE_TQ_MIN_NM  = 260.0
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
