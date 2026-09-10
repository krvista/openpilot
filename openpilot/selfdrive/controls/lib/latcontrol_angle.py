import math
import numpy as np

from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.params import Params

# TODO This is speed dependent
STEER_ANGLE_SATURATION_THRESHOLD = 2.5  # Degrees

# Phase 6h-6: low-pass on livePose/liveParameters roll before roll compensation.
# Road bank is quasi-static (changes over seconds), but the roll ESTIMATE carries
# 2-8 Hz noise that g*roll/u^2 amplifies at city speed: ccnc-drivelog 0x49/0x4a
# (20-40 km/h hands-off) measured the roll term injecting 0.138e-4 1/m of 2-8 Hz
# curvature — ~50% of the gap between the smooth desiredCurvature (0.18e-4) and
# the wobbling achieved curvature (0.45e-4). Offline replay: tau=0.5 s removes
# 82% of that injection (0.138 -> 0.024e-4) with zero steady-state banking loss.
# This is why the 6f-4/6h-1 curvature smoothing never killed the felt low-speed
# wobble — roll compensation is added DOWNSTREAM of it, here.
# Kill switch: ROLL_LP_TAU = 0.0 (raw roll, pre-6h-6).
ROLL_LP_TAU = 0.6  # s

# Phase 7a: closed-loop curvature trim. The lateral chain was pure feed-forward;
# nothing corrected the persistent gap between commanded and ACHIEVED curvature
# (ccnc-drivelog 0x49/0x4a: 1s-LP |desired-achieved| p50 8-10e-4 1/m in corners,
# 2-3e-4 overall — actuation under-delivery, angle-offset/crown bias). A slow,
# hard-bounded integrator on that error fills it. Strictly bounded authority:
# cap = min(LAT_FB_CAP, LAT_FB_ACCEL_CAP/v^2) ~= 1.0 deg wheel-equivalent on the
# Ioniq 6 N. Anti-windup is BLEED, not hold: while the driver interacts
# (steeringPressed), the actuator clips/yields (steer_limited_by_safety,
# curvature_limited) or the error is yield-sized (>LAT_FB_ERR_MAX), the trim
# decays (tau 2 s) so no stale trim is released after an override — replay on
# 0x49/0x4a measured worst released trim ~1.0 deg-equiv (p90), |trim| p50
# 2.6e-4. Does NOT address the model-plan entry deficit (that error is not
# visible to desired-vs-achieved). Kill switch: LAT_FB_KI = 0.0 (bit-identical).
LAT_FB_KI = 0.8             # 1/s
LAT_FB_CAP = 10e-4          # 1/m. Phase 7a-5 REVERT to the validated 7a-2 value.
                            # 7a-3 tried 10e-4 -> 14e-4 (and 7a-4 unlocked sharp
                            # corners via a sustained-error gate) to close the
                            # corner curvature deficit (achieved/desired ~0.82
                            # inst / 0.90 steady). On-road (0x5d-0x61, build
                            # 63bac87) the deficit DID NOT close — instead
                            # over-correction (steady ratio >1.1) doubled
                            # 10.8% -> 22.5%, concentrated in the sharp corners
                            # 7a-4 unlocked (matched-bin 0% -> 23-26%), and the
                            # driver confirmed "inside-cut / over-steering" in
                            # corners. So pushing the trim harder cuts inside
                            # rather than tracking: the ~0.90 steady ratio at this
                            # cap is near the practical limit (the residual is
                            # mostly model-plan deficit + entry lag, which the
                            # desired-vs-achieved trim cannot fix without
                            # over-cutting). Reverted to 10e-4 + the instantaneous
                            # ERR_MAX gate (7a-4 disabled below). Kill: LAT_FB_KI=0.
                            # (7a-2 was 4e-4 -> 10e-4 on 0x4b/0x4c, validated good.)
                            # (Phase 7a-2 was 4e-4 -> 10e-4 on 0x4b/0x4c.)
LAT_FB_ACCEL_CAP = 0.5      # m/s^2; speed-aware cap = ACCEL_CAP / v^2
# Phase 7a-4: the "large error -> bleed" gate now keys on a short LP of the error,
# not the instantaneous error, so transient spikes (model jumps, brief glitches)
# no longer bleed the trim while SUSTAINED sharp-corner under-delivery does get
# integrated. Pooled 0x50-0x5b corner steady-deficit (1s-LP): 23% of corner
# frames sit >15e-4, locked behind the old instantaneous 15e-4 gate (bled, never
# corrected) — these are persistent (1s) deficits, i.e. real sharp-corner
# shortfall, not spikes. Gate on a 0.3 s LP and raise the sustained threshold to
# 22e-4 so those integrate (still bounded by LAT_FB_CAP=14e-4 + the v^2 accel
# cap, so the trim authority/safety envelope is UNCHANGED — only WHICH corners
# reach it changes). Keep a hard INSTANTANEOUS guard (30e-4) so a genuine large
# spike still bleeds immediately; steeringPressed still gates overrides. Kill
# switch: LAT_FB_ERR_LP_TAU=0 + LAT_FB_ERR_MAX=15e-4 -> previous behaviour.
LAT_FB_ERR_LP_TAU   = 0.0   # s; 7a-5 REVERT: 0 = gate on the INSTANTANEOUS error
                            # (the 7a-4 0.3 s sustained gate unlocked sharp
                            # corners and over-corrected them 0%->23-26% in
                            # matched bins; driver felt inside-cut). Restored.
LAT_FB_ERR_MAX      = 15e-4 # 1/m; instantaneous error above this = yield/bleed (7a-2 value)
LAT_FB_ERR_MAX_HARD = 30e-4 # 1/m; redundant when LP_TAU=0 (15e-4 gate subsumes it); kept for the code path
LAT_FB_BLEED_FROZEN = 2.0   # s
LAT_FB_BLEED_INACTIVE = 0.5 # s
# Phase 7a-7 (i6nv3 routes 0000000b/c): below ~30 km/h the CCNC EPS does not act on small angle
# commands (trim-free 18-21 km/h band: |cmd-wheel| > 3 deg on 38-46 % of hands-off frames; 30-33 km/h
# still 36-40 %, clearing only from ~33-36 km/h), so the trim wound to its cap against a wheel that never
# moved (10-15 % of frames at cap at 22-30 km/h vs 2-7 % above 30 km/h) and stored up to 2.6 deg that
# was released as a jump the moment the plan crossed the deadband — Banpo bridge ramp (route c
# 1571.5-1572.6 s, 27 km/h): in the worst aligned 1 s the command swung 7.8 deg, the plan 5.4 deg and
# the stale trim flipping sign under the 7b entry boost the other 2.4 deg. Passthrough (bleed) below
# 8.3 m/s: replay of routes b/c — plan-quiet 1-s command swing p95 below 30 km/h 1.75/1.08 -> 0.75/0.74
# deg, stale-trim frames 4.9/3.8 % -> 0, that ramp window 7.8 -> 5.4 deg; 30-54 km/h swing p99
# unchanged (trim-at-cap 3.0 -> 2.5 % / 6.3 -> 5.5 % from decel carry-over), above 54 km/h identical.
# What is given up at 22-30 km/h where the EPS did track: trim p50 0.3-0.4 deg, p90 1.2 deg
# (<= 0.07 m/s^2 of lateral accel at 8 m/s). Kill switch: LAT_FB_MIN_SPEED = 6.0 (7a-5/7a-6 behaviour).
LAT_FB_MIN_SPEED = 8.3      # m/s (below: passthrough region, bleed); 7a-7, was 6.0
# Phase 7a-6 (i6nv3 route 00000007, first drive with angleFbInteg logged): hands-off the trim sat AT ITS
# CAP 46 % of the time (30-50 km/h: 50 %) while the curvature error it integrates was tiny (|median|
# 0.14e-3, mean -0.13e-3) — a constant sub-deadband bias of the EPS/VM chain that the EPS never acts
# on, so the command stood ~2 deg beyond the wheel (steerCmdGapDeg median 2.8 deg) for nothing, and
# that stored 2 deg was released whenever the EPS did respond (corner entry, driver touch; 16 % of the
# driver grabs had the trim at cap vs 9 % baseline). Two changes, both in the "less trim" direction the
# 7a-5 revert already chose: (1) errors inside ERR_DEADBAND integrate as 0 (the EPS cannot act on
# them either), (2) a slow leak so a constant residual bias settles at KI*tau*(e-db) instead of the
# cap. The leak time constant scales with the speed-aware cap (tau_eff = LEAK_TAU * cap / LAT_FB_CAP,
# review) so the error that would saturate the trim, db + cap/(KI*tau_eff), stays ~0.45e-3 at every
# speed instead of collapsing toward the deadband on the highway. Corner deficits (0.8-1.0e-3) still
# reach the cap: 0.9 s on a rising entry (7b boost), 2.7 s steady, at 14 m/s. Offline replay of route
# 7: time at cap 46 % -> 0 %, released trim on a grab 2.8 deg -> 0.8 deg (verifier probes).
# Kill switch: LAT_FB_ERR_DEADBAND = 0.0 and LAT_FB_LEAK_TAU = 0.0 (bit-identical to 7a-5).
# 7a-6 RESULT (route 00000008, the first drive with it, same evening road as route 00000006 = 7a-5): the
# trim did drop as predicted (time at cap 46 % -> 7 %, cmd-wheel gap 2.8 -> 1.6 deg) but lane keeping
# got WORSE — lane-centre |offset| median 0.03 -> 0.12 m, >0.5 m off-centre 0.2 % -> 4.9 %, driver
# corrections 2.6 -> 4.4 /min, most of the big grabs in moderate corners with good lane lines (cmd ~=
# wheel, the driver adding what op no longer asked for). The "useless" standing trim was holding the
# car centred on crowned / gently curved roads through the EPS deadband after all: the 0.2e-3
# deadband + leak cut exactly the moderate-corner band (0.3-0.6e-3 deficits). Both OFF again = 7a-5,
# bit-identical. The code path and tests stay for a future speed/curve-gated variant.
# A/B (2026-09-08): the route-8 comparison is confounded — hands-off share 26 % vs 10 %, median speed 57 vs
# 47 km/h, and on GPS+speed-matched locations (133 pairs) route 8 was BETTER (|offset| 0.09 vs 0.20 m) while
# the speed-binned aggregate says worse. So 7a-6 is neither confirmed nor refuted: it sits behind the
# Params bool LatFbTrim7a6 — ON by default for the on-road validation (driver's call, 2026-09-08); 0 = 7a-5,
# bit-identical. Judge with tools/i6nv3_bench/ab_lane_compare.py over matched drives.
LAT_FB_ERR_DEADBAND = 0.2e-3  # 1/m when LatFbTrimDeadband is on; ~0.56 deg of wheel at 50 km/h
LAT_FB_LEAK_TAU     = 5.0     # s at the full 10e-4 cap when on; scaled with the speed-aware cap
# Phase 7b: entry-scheduled gain. The base KI reaches the cap in ~0.5 s — half
# the 1 s entry window. While the commanded curvature magnitude is RISING
# (corner building) integrate faster so the trim arrives within ~0.2 s of
# entry. Authority cap/freezes unchanged (same risk envelope as 7a).
# Kill switch: LAT_FB_ENTRY_BOOST = 1.0.
LAT_FB_ENTRY_BOOST = 2.5


class LatControlAngle(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.sat_check_min_speed = 5.
    self.use_steer_limited_by_safety = CP.brand in ("tesla", "hyundai")
    self._roll_lp = 0.0
    self._roll_lp_init = False
    self._fb_integ = 0.0  # Phase 7a closed-loop curvature trim state
    self._des_slow = 0.0  # Phase 7b rising-entry detector (EMA 0.5 s)
    self._fb_err_lp = 0.0  # Phase 7a-4: 0.3 s LP of fb_err for the sustained-error gate
    self._trim_7a6 = True   # Phase 7a-6 (Params LatFbTrim7a6, default on for the on-road validation), read once at start
    try:
      self._trim_7a6 = bool(Params().get_bool("LatFbTrim7a6"))
    except Exception:
      pass

  def _filtered_roll(self, roll: float) -> float:
    if ROLL_LP_TAU <= 0.0:
      return roll
    # Non-finite guard: the LP update is recursive, so a single NaN roll
    # estimate would poison the filter state permanently (stock raw-roll was
    # stateless and recovered next frame). Hold the last good value instead.
    if not math.isfinite(roll):
      return self._roll_lp
    if not self._roll_lp_init:
      self._roll_lp = roll
      self._roll_lp_init = True
    else:
      self._roll_lp += (self.dt / (ROLL_LP_TAU + self.dt)) * (roll - self._roll_lp)
    return self._roll_lp

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    angle_log = log.ControlsState.LateralAngleState.new_message()
    # Track roll continuously (also while inactive) so engage starts warm.
    roll_filtered = self._filtered_roll(params.roll)

    # Phase 7a: closed-loop curvature trim (see constants block).
    if LAT_FB_KI > 0.0:
      curv_actual = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, roll_filtered)
      fb_err = desired_curvature - curv_actual
      # Non-finite guard: the bleed paths are multiplicative, so a single NaN
      # frame (CAN glitch on steeringAngleDeg / model output) would poison
      # the integrator and error-LP state permanently — the pre-7a chain was
      # pure FF and recovered on the next clean frame. Zero the frame's error
      # and flush any state already hit; the trim just re-integrates.
      if not math.isfinite(fb_err):
        fb_err = 0.0
      if not (math.isfinite(self._fb_integ) and math.isfinite(self._fb_err_lp)):
        self._fb_integ = 0.0
        self._fb_err_lp = 0.0
      # Phase 7a-4: short LP of the error so the yield gate keys on SUSTAINED
      # deficit, not transient spikes (see constants block).
      err_lp_a = self.dt / (LAT_FB_ERR_LP_TAU + self.dt) if LAT_FB_ERR_LP_TAU > 0.0 else 1.0
      self._fb_err_lp += err_lp_a * (fb_err - self._fb_err_lp)
      if not active or CS.vEgo < LAT_FB_MIN_SPEED or not math.isfinite(CS.vEgo):
        self._fb_integ *= max(1.0 - self.dt / LAT_FB_BLEED_INACTIVE, 0.0)
      elif (CS.steeringPressed or steer_limited_by_safety or curvature_limited
            or abs(self._fb_err_lp) > LAT_FB_ERR_MAX or abs(fb_err) > LAT_FB_ERR_MAX_HARD):
        self._fb_integ *= max(1.0 - self.dt / LAT_FB_BLEED_FROZEN, 0.0)
      else:
        cap = min(LAT_FB_CAP, LAT_FB_ACCEL_CAP / max(CS.vEgo, 5.0) ** 2)
        rising = abs(desired_curvature) > self._des_slow * 1.02
        ki = LAT_FB_KI * (LAT_FB_ENTRY_BOOST if rising else 1.0)
        # 7a-6: deadband on the error, slow leak on the state (see constants)
        db = LAT_FB_ERR_DEADBAND if self._trim_7a6 else 0.0
        tau = LAT_FB_LEAK_TAU if self._trim_7a6 else 0.0
        err_eff = 0.0 if abs(fb_err) < db else fb_err - math.copysign(db, fb_err)
        leak = (self._fb_integ * self.dt / (tau * cap / LAT_FB_CAP)) if tau > 0.0 else 0.0
        self._fb_integ = float(np.clip(self._fb_integ + ki * err_eff * self.dt - leak, -cap, cap))
      # (same guard for the 7b rising-entry EMA — recursive state)
      if math.isfinite(desired_curvature):
        self._des_slow += (self.dt / 0.5) * (abs(desired_curvature) - self._des_slow)
    else:
      self._fb_integ = 0.0

    if not active:
      angle_log.active = False
      angle_steers_des = float(CS.steeringAngleDeg)
    else:
      angle_log.active = True
      # Phase 6h-5: restore full roll compensation above 15 m/s. The 0.5 cap at
      # all speeds >=10 m/s systematically under-compensates banked/crowned
      # roads (steady-state lateral offset); low-speed damping kept for noisy
      # livePose roll. Kill switch: [0.0, 5.0, 10.0] / [0.0, 0.2, 0.5].
      roll_gain = float(np.interp(CS.vEgo, [0.0, 5.0, 10.0, 15.0], [0.0, 0.2, 0.5, 1.0]))
      roll_damped = roll_filtered * roll_gain  # 6h-6: LP'd roll (see ROLL_LP_TAU)
      angle_steers_des = math.degrees(VM.get_steer_from_curvature(-(desired_curvature + self._fb_integ), CS.vEgo, roll_damped))
      angle_steers_des += params.angleOffsetDeg

    if self.use_steer_limited_by_safety:
      # these cars' carcontrollers calculate max lateral accel and jerk, so we can rely on carOutput for saturation
      angle_control_saturated = steer_limited_by_safety
    else:
      # for cars which use a method of limiting torque such as a torque signal (Nissan and Toyota)
      # or relying on EPS (Ford Q3), carOutput does not capture maxing out torque  # TODO: this can be improved
      angle_control_saturated = abs(angle_steers_des - CS.steeringAngleDeg) > STEER_ANGLE_SATURATION_THRESHOLD
    angle_log.saturated = bool(self._check_saturation(angle_control_saturated, CS, steer_limited_by_safety, curvature_limited))
    angle_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    angle_log.steeringAngleDesiredDeg = angle_steers_des
    return 0, float(angle_steers_des), angle_log
