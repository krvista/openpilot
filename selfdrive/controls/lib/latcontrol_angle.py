import math
import numpy as np

from cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl

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
LAT_FB_CAP = 4e-4           # 1/m
LAT_FB_ACCEL_CAP = 0.5      # m/s^2; speed-aware cap = ACCEL_CAP / v^2
LAT_FB_ERR_MAX = 15e-4      # 1/m; larger error = yield/clip, don't integrate
LAT_FB_BLEED_FROZEN = 2.0   # s
LAT_FB_BLEED_INACTIVE = 0.5 # s
LAT_FB_MIN_SPEED = 6.0      # m/s (below: passthrough region, bleed)


class LatControlAngle(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.sat_check_min_speed = 5.
    self.use_steer_limited_by_safety = CP.brand in ("tesla", "hyundai")
    self._roll_lp = 0.0
    self._roll_lp_init = False
    self._fb_integ = 0.0  # Phase 7a closed-loop curvature trim state

  def _filtered_roll(self, roll: float) -> float:
    if ROLL_LP_TAU <= 0.0:
      return roll
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
      if not active or CS.vEgo < LAT_FB_MIN_SPEED:
        self._fb_integ *= max(1.0 - self.dt / LAT_FB_BLEED_INACTIVE, 0.0)
      elif CS.steeringPressed or steer_limited_by_safety or curvature_limited or abs(fb_err) > LAT_FB_ERR_MAX:
        self._fb_integ *= max(1.0 - self.dt / LAT_FB_BLEED_FROZEN, 0.0)
      else:
        cap = min(LAT_FB_CAP, LAT_FB_ACCEL_CAP / max(CS.vEgo, 5.0) ** 2)
        self._fb_integ = float(np.clip(self._fb_integ + LAT_FB_KI * fb_err * self.dt, -cap, cap))
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
