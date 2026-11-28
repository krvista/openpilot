import math
import numpy as np

from openpilot.cereal import log
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

# Phase 75k (report §103.6-2): 'Turn Exceeds Steering Limit'. For brands whose car controller limits the request the
# saturation input is "carOutput differs from the command"; the port passed that same flag as the argument of
# _check_saturation that must be False, so the alert could only come from the curvature limit (a1: it should have
# sounded 0.6 s before the driver took over at 72 km/h with the request 5.5 deg behind). Upstream passes False; so does
# this again.
# While the car controller holds the request passive by design or the factory LFA steers (self.request_paused, set by
# controlsd every frame) the timer runs down instead: those states have their own indication and are not a steering
# limit. A driver press runs it down as before.
# Kill: SAT_CHECK_UPSTREAM = False (the pre-75 call).
SAT_CHECK_UPSTREAM = True


class LatControlAngle(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.sat_check_min_speed = 5.
    self.use_steer_limited_by_safety = CP.brand in ("tesla", "hyundai")
    self._roll_lp = 0.0
    self._roll_lp_init = False
    self.request_paused = False   # Phase 75k (controlsd sets it every frame)

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
      # (+ 0.0: where the Phase 7a trim, removed in Phase 89, was added - it was 0.0 on the direct path and turned a -0.0
      # curvature into +0.0; kept so the angle stays bit-identical)
      angle_steers_des = math.degrees(VM.get_steer_from_curvature(-(desired_curvature + 0.0), CS.vEgo, roll_damped))
      angle_steers_des += params.angleOffsetDeg

    if self.use_steer_limited_by_safety:
      # these cars' carcontrollers calculate max lateral accel and jerk, so we can rely on carOutput for saturation
      angle_control_saturated = steer_limited_by_safety
    else:
      # for cars which use a method of limiting torque such as a torque signal (Nissan and Toyota)
      # or relying on EPS (Ford Q3), carOutput does not capture maxing out torque  # TODO: this can be improved
      angle_control_saturated = abs(angle_steers_des - CS.steeringAngleDeg) > STEER_ANGLE_SATURATION_THRESHOLD
    if not SAT_CHECK_UPSTREAM:
      angle_log.saturated = bool(self._check_saturation(angle_control_saturated, CS, steer_limited_by_safety, curvature_limited))
    elif getattr(self, "request_paused", False):   # Phase 75k: passive by design -> not a steering limit
      angle_log.saturated = bool(self._check_saturation(False, CS, False, False))
    else:
      angle_log.saturated = bool(self._check_saturation(angle_control_saturated, CS, False, curvature_limited))
    angle_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    angle_log.steeringAngleDesiredDeg = angle_steers_des
    return 0, float(angle_steers_des), angle_log
