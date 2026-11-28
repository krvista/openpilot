#!/usr/bin/env python3
import math
from numbers import Number

import numpy as np
from collections import deque

from openpilot.cereal import log
from opendbc.car.structs import car
import openpilot.cereal.messaging as messaging
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_CTRL, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog

from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from openpilot.selfdrive.controls.lib.bsm_guard import BsmLaneGuard, BSM_LANE_GUARD_M, BSM_LANE_GUARD_MIN_PROB
from openpilot.selfdrive.controls.lib.lowconf_rate_cap import LowConfRateCap, LOWCONF_CAP_DPS, LOWCONF_RELEASE_DPS
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_curvature import LatControlCurvature
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose

from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt

State = log.SelfdriveState.OpenpilotState
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection


def _interp(x, xp, fp):
  """Scalar np.interp (ascending xp, clamped ends, NaN in -> NaN out) without the numpy call
  overhead: same arithmetic (slope * (x - xp[j]) + fp[j]) so results match np.interp."""
  if x != x:
    return float("nan")
  if x <= xp[0]:
    return float(fp[0])
  if x >= xp[-1]:
    return float(fp[-1])
  for j in range(len(xp) - 1):
    if x < xp[j + 1]:
      return (fp[j + 1] - fp[j]) / (xp[j + 1] - xp[j]) * (x - xp[j]) + fp[j]
  return float(fp[-1])


def _clip(v, lo, hi):
  """Scalar np.clip (NaN passes through)."""
  return lo if v < lo else (hi if v > hi else v)


class _ModelFrame:
  """Everything state_control derives from ONE modelV2 message (20 Hz), computed once and reused
  by the 100 Hz control frames. Profile (route 00000005 seg 13, in-process): two cubic polyfits +
  ~8 np.fromiter + ~12 np.interp per 100 Hz frame were ~45 % of state_control; all of them depend
  only on the model message (the per-frame speed enters afterwards). Values are identical — the
  same arrays go into the same polyfit — the work just happens 5x less often. The expensive parts
  (plan-point fits, lane-centre curvature, BSM lane inputs) are computed on first use, so a
  disengaged frame — which never reads them — pays nothing (review: eager computation made
  disengaged frames slower than before)."""
  __slots__ = ("ref", "key", "fallback", "n", "_pos", "_fit24", "lane_min", "lane_max", "y_std", "lane_min_p",
               "_bsm", "lane_change_state", "lane_change_dir", "_lanes", "_psi", "_lfit")
  _UNSET = object()

  def __init__(self, model_v2, key):
    self.ref = model_v2; self.key = key
    self.fallback = model_v2.action.desiredCurvature
    self.n = len(model_v2.position.x)
    self._pos = None
    self._fit24 = self._bsm = self._lanes = _ModelFrame._UNSET
    self._psi = _ModelFrame._UNSET   # Phase 78
    self._lfit = _ModelFrame._UNSET  # Phase 82
    probs = getattr(model_v2, "laneLineProbs", [])
    self.lane_min = min(float(probs[1]), float(probs[2])) if len(probs) >= 4 else 1.0
    self.lane_max = max(float(probs[1]), float(probs[2])) if len(probs) >= 4 else 1.0   # Phase 49
    self.lane_min_p = min(float(probs[1]), float(probs[2])) if len(probs) >= 3 else 0.0
    y_std_list = getattr(model_v2.position, "yStd", [])
    self.y_std = float(y_std_list[5]) if len(y_std_list) > 5 else 0.0
    meta = getattr(model_v2, "meta", None)
    self.lane_change_state = meta.laneChangeState if meta is not None else LaneChangeState.off
    self.lane_change_dir = meta.laneChangeDirection if meta is not None else LaneChangeDirection.none

  def _fit(self, nn):
    """(x, polyfit-3 coefficients or None) over the first nn plan points; None when n < 5."""
    if self.n < 5:
      return None
    if self._pos is None:
      self._pos = (list(self.ref.position.x), list(self.ref.position.y))
    m = min(self.n, nn)
    x = np.array(self._pos[0][:m], dtype=np.float64); y = np.array(self._pos[1][:m], dtype=np.float64)
    c = None
    if np.all(np.isfinite(x)) and np.all(np.isfinite(y)):
      try:
        c = np.polyfit(x, y, 3)
      except (np.linalg.LinAlgError, ValueError):
        c = None
    return (x, c)

  @property
  def fit24(self):
    if self._fit24 is _ModelFrame._UNSET:
      self._fit24 = self._fit(24)
    return self._fit24

  @property
  def psi_plan(self):
    """Phase 78: plan heading on ModelConstants.T_IDXS; None = missing, short or non-finite."""
    if self._psi is _ModelFrame._UNSET:
      self._psi = None
      try:
        psi = np.array(list(self.ref.orientation.z), dtype=np.float64)
        if len(psi) == len(_PLAN_T_IDXS) and np.all(np.isfinite(psi)):
          self._psi = psi
      except (AttributeError, TypeError, ValueError):
        pass
    return self._psi

  def lane_preview(self, dist):
    """Phase 73c: (lane-centre y at the car, lane-centre y at dist, plan y at dist, lane width at the car, lane width at
    dist) from the two inner lane lines and the plan position; None = unreadable / non-finite / plan too short."""
    if self._lanes is _ModelFrame._UNSET:
      self._lanes = None
      try:
        lls = getattr(self.ref, "laneLines", [])
        arr = [np.fromiter(a, dtype=np.float64) for a in (lls[1].x, lls[1].y, lls[2].x, lls[2].y, self.ref.position.x, self.ref.position.y)]
        if all(len(a) >= 5 and np.all(np.isfinite(a)) for a in arr):
          self._lanes = arr
      except (IndexError, AttributeError, TypeError, ValueError):
        self._lanes = None
    if self._lanes is None:
      return None
    xl, yl, xr, yr, px, py = self._lanes
    if not (px[-1] >= dist and xl[-1] >= dist and xr[-1] >= dist):
      return None
    l0, r0 = float(np.interp(0.0, xl, yl)), float(np.interp(0.0, xr, yr))
    ld, rd = float(np.interp(dist, xl, yl)), float(np.interp(dist, xr, yr))
    return 0.5 * (l0 + r0), 0.5 * (ld + rd), float(np.interp(dist, px, py)), r0 - l0, rd - ld

  def lane_fit(self, xf):
    """Phase 82: (lane-centre y at the car, heading, curvature) of a quadratic through the lane centre (two inner lines)
    over 0..xf m; None = unreadable / fewer than 3 points."""
    self.lane_preview(0.0)                 # reads and checks the lines once
    if self._lanes is None:
      return None
    key = int(round(xf))
    if self._lfit is _ModelFrame._UNSET:
      self._lfit = {}
    if key not in self._lfit:
      xl, yl, xr, yr = self._lanes[:4]
      m = xl <= key + 1e-6
      out = None
      if int(np.count_nonzero(m)) >= 3 and xr[-1] >= xl[m][-1]:
        try:
          c = np.polyfit(xl[m], 0.5 * (yl[m] + np.interp(xl[m], xr, yr)), 2)
          if np.all(np.isfinite(c)):
            out = (float(c[2]), float(c[1]), float(2.0 * c[0]))
        except (np.linalg.LinAlgError, ValueError):
          out = None
      self._lfit[key] = out
    return self._lfit[key]

  @property
  def bsm_lane(self):
    """Phase 37b BSM guard inputs (y_left, y_right, p_left, p_right); None = unreadable -> guard reset."""
    if self._bsm is _ModelFrame._UNSET:
      self._bsm = None
      try:
        lls = getattr(self.ref, "laneLines", [])
        probs = getattr(self.ref, "laneLineProbs", [])
        if len(lls) >= 3 and len(probs) >= 3 and len(lls[1].y) > 0 and len(lls[2].y) > 0:
          self._bsm = (float(lls[1].y[0]), float(lls[2].y[0]), float(probs[1]), float(probs[2]))
      except (IndexError, TypeError, ValueError):
        self._bsm = None
    return self._bsm


ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())

# Lateral accel envelope used to normalize the predicted ratio for the
# curveSpeedAdvisory event. Matches CarControllerParams.ANGLE_LIMITS.
# MAX_LATERAL_ACCEL in opendbc/car/hyundai/values.py — angle-control panda
# safety enforces the same number, so the ratio is comparable to the
# threshold that trips vmLimitTripped. Non-angle cars never trip the
# envelope but the ratio is still computed for unified logging.
LAT_ACCEL_ENVELOPE = 3.0 + 9.81 * 0.06  # ~3.59 m/s²

# EMA time constant for the predicted ratio (s). Short enough to react to
# an imminent ramp/curve, long enough to suppress single-frame modelV2
# noise. With DT_CTRL=0.01s this gives alpha ≈ 0.033 per frame.
PRED_RATIO_TAU = 0.3

_PLAN_T_IDXS = np.array(ModelConstants.T_IDXS, dtype=np.float64)
# Phase 64b: the lateral delay the model is told while lagd has not published one (CP.steerActuatorDelay 0.1 + 0.2), and
# how often _update_model_lat_delay re-reads the LagdToggle / LagdValueCache params.
LOOKAHEAD_DELAY_BASE_S = 0.3
LOOKAHEAD_DELAY_PARAM_FRAMES = 300    # LagdToggle / LagdValueCache re-read every 3 s (modeld: every 3 s)

# i6n Phase 72 (report §96-97): DIRECT lateral path. With the factory LFA held off the vehicle's ADAS controller relays
# our request and the MDPS follows it in 180 ms at 0.95 (request -> yaw 0.32 s). The command is the model's action
# itself - the model already plans for the lateral delay lagd publishes - and only the guards about the MODEL stay
# (non-finite hold, confidence blend, lane-dropout hold, low-confidence rate cap, departure hold, BSM guard, turn-exit
# handling) plus clip_curvature.
# Phase 89: this is the only path. The lead layers fitted to the factory LFA steering on its own schedule (6h-1
# smoothing with matched lead, 6f-5 / 6h-2 cubic lead, 55b / 55c plan lead, 60 / 64b / 68 / 69a delay match, 7c entry
# assist, latcontrol 7a trim) and the profile switch that selected them were removed.
# Phase 46: controlsState.steerFlags — which lateral guards acted on this frame's command, so drive-log judgments of
# Phase 39/41/41-2/7c/37b no longer have to re-derive them from inputs (report §26 "observability" gap). Telemetry
# only: nothing reads these bits back. Cost: a few int ORs per 100 Hz frame and one UInt16 in controlsState.
STEER_FLAG_LOWCONF_GATE = 1 << 0      # Phase 41 low-confidence gate on (lane_min < LOWCONF_LANE_MIN, hands off)
STEER_FLAG_LOWCONF_CAPPED = 1 << 1    # Phase 41 rate cap actually held the command back
STEER_FLAG_LOWCONF_RELEASE = 1 << 2   # Phase 41 release ramp running (gate just dropped)
STEER_FLAG_LOOKAHEAD_CAP = 1 << 3     # (logs before Phase 89 only) Phase 41-2 lookahead t_ahead clipped by its cap
STEER_FLAG_CONF_BLEND = 1 << 4        # Phase 6g confidence blend toward the previous command (confidence < 1)
STEER_FLAG_LANE_DROPOUT = 1 << 5      # Phase 39 dropout hold
STEER_FLAG_DEPARTURE_HOLD = 1 << 6    # lane-departure guard held the command
STEER_FLAG_BSM_HOLD = 1 << 7          # Phase 37b BSM lane guard held the command
STEER_FLAG_ENTRY_ASSIST = 1 << 8      # (logs before Phase 89 only) Phase 7c geometry entry assist added curvature
STEER_FLAG_LOOKAHEAD_ORIENT = 1 << 9  # (logs before Phase 89 only) Phase 55b lead from the plan yaw-rate change
STEER_FLAG_LOOKAHEAD_BUDGET = 1 << 10 # (logs before Phase 89 only) Phase 55b orient lead clipped by the jerk budget
STEER_FLAG_DIRECT = 1 << 11           # Phase 72 direct path in use while lateral is active (Phase 89: always)
STEER_FLAG_ENTRY_LEAD = 1 << 13       # Phase 78 entry lead is adding curvature
STEER_FLAG_LANE_CENTER = 1 << 12      # Phase 73c lane-centre term is adding curvature
STEER_FLAG_LANE_FIT = 1 << 14         # Phase 82 lane-fit part is adding curvature
STEER_FLAG_LANE_AVOID = 1 << 15       # Phase 83 lane terms (partly) released for an avoidance

# i6n Phase 73c (report §101, first solo drive a1): LANE-CENTRE term on the direct path. Steering alone, the model keeps
# the car parallel to the lane but hardly pulls it to the middle: its action fits  K = 0.77 k_lane + 0.048 psi +
# 0.55e-3 e  (psi = lane heading relative to the car, e = lane centre relative to the car in m) - a lateral "spring" of
# 0.24 1/s^2 at 76 km/h (period 13 s) with no integral action. A curvature bias of 0.05e-3 (road crown, model bias)
# therefore parks the car 10-25 cm off centre: a1 sat 19 cm LEFT of the centre by the factory camera (the factory LFA:
# +0.4 cm), 12-27 cm by the model's own lines with the plan staying parallel out to 60 m, and ran 10-30 cm to the inside
# of curves (factory 0-8 cm). The model's lane lines themselves agree with the factory camera to 3.4 cm (r 0.94).
#   proportional: the plan is steered through the lane centre at the preview distance L = v * LANE_CENTER_T_S -
#     LANE_CENTER_WEIGHT * 2 (centre(L) - plan(L)) / L^2 - which acts before the car has left the centre (the plan
#     cutting inside at a corner entry is corrected as it is made); capped at LANE_CENTER_ACC / v^2, low-passed.
#   integral: LANE_CENTER_KI x the lane-centre error at the car, on near-straight road only (|action| below
#     LANE_CENTER_I_MAX_K; held in curves), capped at LANE_CENTER_I_CAP - removes what a steady bias leaves.
# Loop simulation with the fitted model law, the measured delay (0.32 s) and 0.05e-3 of bias: steady error 17.6 cm ->
# 1-2 cm, <= 3 swings, robust to +0.2 s of delay and to half the model's own gains; the inside error in a curve 22 ->
# 7 / 12 / 15 cm at 50 / 76 / 100 km/h. Used only with both inner lines confident (LANE_CENTER_PROB), a plausible lane
# width at the car and at L that does not open or close (split / merge), no blinker / lane change / driver press /
# dropout hold, from LANE_CENTER_SPEED_MS up. Otherwise the proportional part fades with its low-pass and the integral
# decays (LANE_CENTER_I_BLEED_S). LANE_CENTER_BIAS_M: the model's lane centre sits 3.4 cm to the right of the factory
# camera's at CameraOffset 0 (report §99). Kill: LANE_CENTER_WEIGHT = 0.0 (and LANE_CENTER_KI = 0.0).
LANE_CENTER_WEIGHT = 0.5
LANE_CENTER_T_S = 2.0
LANE_CENTER_L_M = (12.0, 80.0)
LANE_CENTER_ACC = 0.4
LANE_CENTER_TAU_S = 0.3
LANE_CENTER_KI = 0.06e-3       # Phase 77 (was 0.03e-3)
LANE_CENTER_I_CAP = 0.30e-3    # Phase 77 (was 0.15e-3)
LANE_CENTER_I_MAX_K = 1.0e-3
LANE_CENTER_I_BLEED_S = 20.0
LANE_CENTER_PROB = [0.5, 0.8]
LANE_CENTER_SPEED_MS = [30.0 / 3.6, 45.0 / 3.6]
LANE_CENTER_WIDTH_M = (2.5, 4.6)
LANE_CENTER_WIDTH_RATIO = 0.25
LANE_CENTER_BIAS_M = -0.034
# Phase 77 (route a7, report 107): the first drive with the term sat +5 cm right of the target on straights and still
# cut curves 15 cm inside. On straights the term was pushing -0.12e-3 on average (road crown + the model's own
# preference) with its integral at the 0.15e-3 cap and 25 s slow -> KI and the cap doubled. In curves the integral
# is frozen and the proportional part only sees the plan 2 s ahead, which the model keeps inside -> a second
# proportional part on the car's own offset from the target, by speed (none at 45 km/h, where the plan part is already
# as stiff as the model: the loop simulation overshoots there). Simulation at 60 / 80 / 110 km/h: curve cut
# 15.5 / 22.4 / 27.5 -> 10.4 / 11.7 / 13.9 cm, straight error after 20 s 8.5-10.7 -> 4.4-5.1 cm, step overshoot
# <= 22 % (not reproduced later - see Phase 79 below). Kill: LANE_CENTER_KE all zero (and the two values above back to
# 0.03e-3 / 0.15e-3).
# Phase 78 (report 108-109): ENTRY LEAD - where a curve is building up the wheel starts ENTRY_LEAD_S earlier. The
# model's action is its plan read at the told delay; the same plan read ENTRY_LEAD_S later is what the action will be
# then. The difference is added only while it makes the action larger on the side it is already going to (entry, not
# exit, not a straight: weight 0 below ENTRY_LEAD_K[0] of future curvature), capped at ENTRY_LEAD_CAP (about 0.5 deg of
# wheel at 60 km/h). It is washed out with ENTRY_LEAD_WASHOUT_S: only the START of the build-up is brought forward and
# the extra is then taken out again even if the curve keeps building (replay of 24 entries: without the washout the term
# sat at its cap for ~3 s and moved the car ~20 cm to the inside for 20-50 ms of lead). Not during lane changes, blinker,
# driver press, lane dropout or below ENTRY_LEAD_MIN_MS. Kill: ENTRY_LEAD_S = 0.0.
ENTRY_LEAD_S = 0.10
ENTRY_LEAD_CAP = 0.15e-3
ENTRY_LEAD_K = [0.15e-3, 0.4e-3]
ENTRY_LEAD_WASHOUT_S = 0.6
ENTRY_LEAD_MIN_MS = 30.0 / 3.6
ENTRY_LEAD_TAU_S = 0.05
ENTRY_LEAD_FRAME_S = 0.075     # modeld: frame delay + half a model interval on top of the told delay
# Phase 79 (report 110): the offset part is trimmed before its first drive. A closed-loop simulation fitted to both solo
# drives (10 model variants) could not reproduce the "<= 22 %" above: the Phase 77 schedule [0, 0.5e-3, 0.8e-3] @
# 45 / 60 / 80 km/h overshoots a 20 cm step by 21 / 24 / 27 % at 65 / 75 / 85 km/h (59-90 % with a weak heading gain)
# and leaves a worst-variant delay margin of 0.27 / 0.19 / 0.12 / 0.04 s at 65 / 75 / 85 / 100 km/h, for 1-2 cm of
# scatter the data cannot resolve. Flat 0.3e-3 from 60 km/h, fading to 0.15e-3 at 110: overshoot 14 / 12 / 11 %, worst
# margin 0.33 / 0.27 / 0.22 / 0.13 s, about half of the simulated scatter gain kept. The integral (straight offset) is
# untouched. Kill: LANE_CENTER_KE = [0.0, 0.0, 0.0, 0.0]. Phase 77 values: speeds 45 / 60 / 80 / 110, [0, 0.5e-3,
# 0.8e-3, 0.8e-3].
# Phase 82 (report 114): LANE FIT - the offset part above is replaced by a fit to the lane itself. A quadratic through
# the lane centre over the next LANE_FIT_RANGE_S gives the car's offset, the lane's heading and the lane's curvature;
# the curvature that would put the car on the centre over 0.5-1.5 x v * LANE_FIT_T_S ahead is
#   1.432 e / L^2 + 1.653 psi / L + k_lane        (least squares of e + psi x + k x^2 / 2 against dk x^2 / 2)
# and LANE_FIT_GAIN of its difference to the model action is added (>= 45 km/h; Phase 88: 40, same weight / cap / low-pass as the
# other parts, own target LANE_CENTER_BIAS_M). The heading is corrected for the camera sitting LANE_FIT_ARM_M ahead of
# the rear axle (in a steady curve the lane heading at the camera is arm x curvature: measured median 1.6-2.0 m; without
# the correction the heading part pushes 0.2-0.5e-3 to the INSIDE and cancels the rest). The lane curvature is read
# near the car on purpose: fitted 1.5 s ahead (as upstream's Blendful does) the simulation turned in early (peak
# inside 18 -> 26 cm at 65 km/h) and swung out at the exit (5 -> 20 cm).
# Simulation with the measured fit noise, 0.5 s of lead and +-1 m of arm error, 7 model variants x both bias signs,
# R 400 m at 65 km/h, against Phase 79: steady inside 10.4 -> 6.5 cm (4.0-9.0 over the arm error), peak inside at the
# entry 17.5 -> 12.6, exit swing 4.9 -> 4.8, S-curve swing 30 -> 21, 20 cm step overshoot 14 -> 13 %, worst delay
# margin 0.50 -> 0.44 s (0.37 -> 0.40 at 85, 0.29 -> 0.35 at 100 km/h), straight scatter 8.4 -> 6.7 cm, wheel activity
# unchanged. On the logs (a1 / a7 / a8 / a9) the term changes 0.018e-3 per 0.5 s on straights (current additions
# 0.034e-3, model action 0.115e-3). Kill: LANE_FIT_GAIN = 0.0 and LANE_CENTER_KE back to [0, 0.3e-3, 0.3e-3, 0.15e-3].
# Phase 83 (report 115): AVOIDANCE RELEASE - when the model deliberately plans away from where it usually drives in the
# lane (works, a parked car in the lane, a cyclist) the lane terms must not pull it back. 'usual' = slow average
# (LANE_AVOID_USUAL_S) of plan - lane centre at the preview distance; the plan's distance from it, less an allowance for
# curves (LANE_AVOID_CURVE x |action| L^2 / 2: the plan cuts curves by itself), takes the weight of all three parts from
# 1 at LANE_AVOID_DEV_M[0] to 0 at [1] - at once on the way down, LANE_AVOID_RECOVER_S back up - and holds the integral.
# While released the average learns half as fast, so a long diversion is taken over slowly; LANE_AVOID_RESET_S without a
# usable lane forgets it; after that the first LANE_AVOID_WARMUP_S only average (one frame is no 'usual'). 60 routes,
# 8.1 h hands-off with both lines: fully released 0.45 % of the time (3.4 episodes of >= 0.5 s per hour, 40 % of them with
# the driver's hand on the wheel against 7 % overall). Real controlsd replayed on a7 / a8 / a9: mean weight 0.978-0.987,
# fully released 0-0.4 %, curve correction of Phase 82 3-9 % smaller. Kill: LANE_AVOID_RELEASE = False.
LANE_AVOID_RELEASE = True
LANE_AVOID_DEV_M = [0.20, 0.45]
LANE_AVOID_CURVE = 0.25
LANE_AVOID_USUAL_S = 10.0
LANE_AVOID_RECOVER_S = 1.5
LANE_AVOID_RESET_S = 5.0
LANE_AVOID_WARMUP_S = 3.0
# Phase 84 (report 116, consistency audit): the Phase 80b / 82 / 83 states are cleared wherever the older lane-term states
# already were - lateral off cleared _lc_p/_lc_i/_lc_e but left the fit part (0.1e-3 on the first frame after 5 s off),
# the usual spot of the road before (lane terms at a quarter for seconds) and the 1 s blend exemption; a lane change
# kept or forgot the usual spot of the OLD lane depending on whether it lasted more or less than LANE_AVOID_RESET_S
# (logged lane changes: 5.0-7.9 s, half within 0.6 s of the limit) - now always forgotten, i.e. 3 s of warm-up in the
# new lane. A non-finite lane probability no longer wipes every lane-term state in one frame (weight 0 instead).
# Kill: LANE_STATE_RESET = False.
LANE_STATE_RESET = True
# Phase 88 (report 119 item 5): the lane fit comes in over 40-55 km/h instead of 45-60 - below 60 km/h the car still
# cut 11 cm to the inside of curves (45-60 km/h, 161 s) against 3.5 cm at 60-80 where the fit is fully on. Real
# controlsd replayed on b0 / b2 / b6 / aa / ac, steady hands-off curves of 45-60 km/h (137 s, 41 curves): the fit
# part's outward curvature goes from 2.7 to 4.6 % of the curve's (5.3 % at 60-80), outward in 98 % of the changed
# seconds; 35-45 km/h 0.0 -> 1.2 %; nothing changes from 60 km/h up. Fit state per frame p99 x 1.15, max equal,
# bit-14 edges + 4 %, steps of the final command identical. Simulation (R 400 / 200 / 125 m, 45 / 50 / 55 km/h):
# steady inside cut -2.1 / -1.8 / -1.6 cm (about a fifth), entry peak and S-curve swing smaller, 20 cm step overshoot
# + 0.5-1.3 points, delay margin 0.68 / 0.61 / 0.55 s (0.53 at 60 km/h), wheel activity + 2-5 %. 35-50 km/h was tried
# and rejected: twice the effect, but fit state steps p99 x 1.33 / max x 1.4 and wheel activity + 8.5 % at 50 km/h.
# Kill: LANE_FIT_SPEED_MS = [45.0 / 3.6, 60.0 / 3.6].
LANE_FIT_GAIN = 0.35
LANE_FIT_SPEED_MS = [40.0 / 3.6, 55.0 / 3.6]   # Phase 88 (was 45-60 km/h)
LANE_FIT_T_S = 1.5
LANE_FIT_RANGE_S = 1.0
LANE_FIT_RANGE_M = (15.0, 45.0)
LANE_FIT_ARM_M = 1.8
LANE_CENTER_KE_SPEED_MS = [45.0 / 3.6, 60.0 / 3.6, 80.0 / 3.6, 110.0 / 3.6]
LANE_CENTER_KE = [0.0, 0.0, 0.0, 0.0]      # 1/m per m of offset. Phase 79: [0, 0.3e-3, 0.3e-3, 0.15e-3]; off since Phase 82

# Phase 75 (report §103), controls side:
# 75j  While the model action is non-finite the held command (MODEL_NONFINITE_*) already contains the lane-centre
#      term; adding it again every frame made the command run away (14 x in 0.3 s). The add is skipped, the two states
#      are kept as they are. Kill: LANE_CENTER_NONFINITE_GUARD = False.
# 75n  Minimum steer speed with hysteresis: lateral goes off LAT_MIN_SPEED_HYST_MS below CP.minSteerSpeed (4 km/h) and
#      comes back above it (5 km/h) - it flapped for 0.1-2.2 s around 5.0 km/h. Kill: LAT_MIN_SPEED_HYST_MS = 0.0.
LANE_CENTER_NONFINITE_GUARD = True
LAT_MIN_SPEED_HYST_MS = 1.0 / 3.6

# Phase 6g-1: floor on the model-confidence damping below. The damping blends the
# command toward the PREVIOUS (straighter) curvature when lane/position confidence
# drops, which on corner entry right after an intersection (low lane confidence as
# lines reconnect) froze op near-straight and the car ran wide to the outside line
# (ccnc-drivelog 0x40 seg9 ~10 min, left line 0.53 m; seg5 right line 0.77 m w/ driver
# takeover). Floor confidence so a low-confidence transition can still pull at most
# (1 - CONF_FLOOR) toward the stale command. Kill switch: CONF_FLOOR = 0.0 (pre-6g-1).
LAT_CONF_FLOOR = 0.5
# Phase 6g-2: taper the floor to 0 at a TRUE lane dropout. ccnc-drivelog 0x42 seg4
# (KST 07:25:24, S-curve reversal) showed the flat floor letting a low-confidence
# _lookahead_curvature spike (~0.024 1/m) through while lane probs collapsed to
# 0.07: op over-commanded desiredCurvature 2.6x and swung the wheel to -35° until
# the driver grabbed (-1166 Nm). Below CONF_FLOOR_LANE_LO the floor is removed so
# confidence falls toward 0 and the command freezes (pre-6g-1 behaviour = spike
# blocked); the reconnection band [LO, HI] keeps the floor that fixed seg9/seg5.
CONF_FLOOR_LANE_LO = 0.20
CONF_FLOOR_LANE_HI = 0.30
# Phase 39: lane-DROPOUT latch. 6g-2 only truly freezes at lane_min <= 0.05
# (conf_l = interp(lane_min, [0.05, 0.30])); a real dropout sits at 0.08-0.23
# and is HALF-PASSED at 0.15-0.7 per 100 Hz frame, i.e. the plan goes through
# within ~0.1 s. i6nv3 route 00000004 seg 8 (09-06, 55-67 km/h): both lines
# at p50 0.13 for 7 s, the model plan swung to -11.8e-3 1/m (wheel +29 deg),
# op followed (controlsState -10.0e-3) and the car crossed the line until the
# driver grabbed at -674 Nm. Latch: enter below LANE_LO, exit after
# DROPOUT_EXIT_S above LANE_HI; while latched the plan is ignored and the
# command DECAYS toward the measured vehicle curvature (tau DROPOUT_TAU_S) —
# "hold what the road/driver has" instead of freezing a stale command for the
# whole blackout (offline replay: peak -11.1e-3 -> -5.3e-3 with a plain freeze,
# lower still with the decay). Kill: LANE_DROPOUT_LATCH = False.
# Phase 39-2 (i6nv3 route 00000005, 09-07 commute, 38 min): the 39 latch fired 74 times
# (175 s latched): 39 below 25 km/h (intersection turns / unmarked lanes, where the
# lines leave the camera view and the plan is RIGHT), 33 with a blinker or an ALC in
# progress (crossing a line drops the probs by design), 41 ran into the 3 s cap and
# re-latched every 6 s on unmarked roads, and in real corners the decay-to-straight
# pulled op out of the curve (seg 8: cmd -9.1e-3 decayed to -1.2e-3 while the road
# was -14e-3 — the driver steered through at 664 Nm). Rework:
#   * HOLD the pre-dropout command instead of decaying it to straight (a corner is
#     kept, a straight stays straight; the decay had no basis in the road);
#   * enter only after ENTRY_S continuous frames below ENTRY_LM (single-frame dips
#     at 0.13-0.19 were most of the 74), and only above MIN_SPEED;
#   * never during / right after a lane change or with a blinker on;
#   * after any release, re-arm only once the lines are back (LANE_HI for REARM_S) —
#     an unmarked road gets ONE hold, then the 6g blend follows the model as before.
LANE_DROPOUT_LATCH = True
LANE_DROPOUT_ENTRY_LM = 0.15      # entry threshold (the 6g blend floor still tapers from LANE_LO)
LANE_DROPOUT_ENTRY_S = 0.3        # continuous frames below ENTRY_LM before latching
LANE_DROPOUT_MIN_SPEED = 40.0 / 3.6
LANE_DROPOUT_EXIT_S = 0.5         # continuous frames above LANE_HI to release
LANE_DROPOUT_MAX_S = 3.0          # hand back to the 6g blend after this long
LANE_DROPOUT_REARM_S = 1.0        # lines back above LANE_HI this long before a new latch may start
LANE_DROPOUT_LC_HOLDOFF_S = 1.0   # no latch this long after a lane change ends
CONF_BLEND_LC_EXEMPT = True       # Phase 80b: no low-confidence blend during op's own lane change. Kill: False
CONF_BLEND_LC_AFTER_S = 1.0       # ... and this long after it
# Phase 49 (i6nv3 routes 3d-42, 10-07..10-12, CD210, report §39/§40): the latch fired 39 times / 64 s in 1.9 h of
# lateral control ("Lane Lines Lost" repeating). 12 had the OTHER inner line clear at 0.71-0.98 — every one the
# curb/wall-side right line weak in the rightmost lane (Namsan tunnel 1: right 0.14-0.19, left 0.97, four alerts in one
# minute) — with hold = plan = road, i.e. an alert and nothing else; 17 were < 60 km/h intersection turns where the
# lines leave the camera, the driver was already turning (16 grabs) and the hold kept the pre-turn command against a
# plan that was right (10 of 17). Replayed on the six routes (39/39 logged latches reproduced) -> 10 latches / 7.8 s
# (probes/dropout_sim.py "P49"):
#   * both inner lines must be weak: the stronger one also below LANE_DROPOUT_MAX_LM (one clear line anchors the plan;
#     the Phase 41 rate cap still covers the weak-line side at lane_min < 0.3, hands off), and a running hold ends
#     once the stronger line is back above CONF_FLOOR_LANE_HI for EXIT_S (LANE_DROPOUT_EXIT_ON_ONE_LINE);
#   * no latch while the driver holds the wheel (carState.steeringPressed): a press restarts the 0.3 s entry debounce
#     (a hold needs 0.3 s hands-off with both lines gone — letting go mid-turn must not snap to a stale command), and
#     a press ends a running hold (one hold per blackout, as for a lane change). While the driver holds, the car
#     controller's grip yield already drops op's authority to <= 0.2 in ~0.17 s (Phase 42/44a), so the hold mostly
#     fought a driver who was turning.
# Trade-off: the original Phase 39 case (route 04 seg 8, both lines 0.14/0.09, hands resting on the wheel) is no longer
# held; it is left to the grip yield and the Phase 41 6 deg/s cap. Kill (pre-49 behaviour): LANE_DROPOUT_MAX_LM = 1.01,
# LANE_DROPOUT_HANDS_OFF = False, LANE_DROPOUT_EXIT_ON_ONE_LINE = False.
LANE_DROPOUT_MAX_LM = CONF_FLOOR_LANE_HI   # the stronger inner line must be below this too
LANE_DROPOUT_HANDS_OFF = True
LANE_DROPOUT_EXIT_ON_ONE_LINE = True
# Phase 56 (report §56/§57): turn exits. Both op-caused line crossings on the 55b drives (4a t455 / t2990, 0.6 m over
# the right line) came straight out of an intersection right turn while accelerating through 40 km/h: the new road's
# lines read 0.0-0.3, so the dropout latch held the command of 0.3 s earlier — a turn still unwinding (+6.1 / +8.1e-3
# vs the model's +4 -> +0.2) — and once the lines returned the Phase 41 6 deg/s cap let it go slowly (command 3e-3
# right of the model for 1.4-1.9 s). The driver's counter-torque (100-470) stayed under the 350 EPS press threshold.
# Phase 46+ logs: 14 of 46 dropout holds were such stale-turn holds (3 of 5 on 55b). Turn-exit window = the CAR's
# curvature (wheel angle) peaked >= TURN_EXIT_K within TURN_EXIT_S and the model action is now <= 1/TURN_EXIT_RATIO
# of that peak: 7.5 % of active frames >= 30 km/h; it would have blocked 17 of the 46 holds, both crossings among them.
# (a) no dropout latch in that window; (b) in that window a command moving toward straight is not rate-capped by
# Phase 41 (treated like a declared manoeuvre); (c) a held dropout ends when the driver turns against it with
# >= LANE_DROPOUT_COUNTER_TQ (angle = -curvature: opposing torque has the SAME sign as the held curvature);
# (d) in that window a command moving toward straight is not frozen by the 6g low-confidence blend either (replay of
# 4a t451-458 through the real state_control: without (d) the blend kept the command 1-2e-3 above the unwinding model
# for 1 s).
# Kill: TURN_EXIT_K = 1e9 (a, b), LANE_DROPOUT_COUNTER_TQ = 1e9 (c).
TURN_EXIT_K = 8e-3                    # 1/m (radius 125 m): intersection / tight-turn scale
TURN_EXIT_S = 5.0
TURN_EXIT_RATIO = 2.0
LANE_DROPOUT_COUNTER_TQ = 200.0       # carState.steeringTorque units (EPS press threshold 350)
TURN_EXIT_SOFT_EXEMPT = True          # (b) Phase 41 cap and (d) 6g blend exemption while unwinding; False = (a)+(c) only
# Phase 57 (report §59, routes 50-52): in a CURVE the model's plan is right even when the lane lines read low — on the
# Namsan road (lines 0.1-0.5) the Phase 41 6 deg/s cap and the 6g blend held the command 1.5-4 s behind the model on
# the second corner of S-bends (4a-like late response, then a jump when the lines returned). All Phase 46+ logs: when
# the cap / blend acted with |action| >= 2e-3 the model action was closer to the car's path 0.6 s later in 56-63 % of
# frames (median error 2.5-3.0 vs 3.5-4.8e-3) and the driver grabbed 2-4x more often, mostly WITH the model's turn.
# The cap and the blend were made for merges / splits on near-straight road (|action| small), so they fade out with
# |action|: cap rate 6 -> 30 deg/s and confidence floor -> 1 over LOWCONF_CURVE_K. Kill: LOWCONF_CURVE_K = [1e9, 1e9].
LOWCONF_CURVE_K = [1.5e-3, 3.0e-3]   # 1/m: |model action| at which the low-confidence restraints start / have faded out

# Non-finite model action handling. A NaN/inf model_v2.action.desiredCurvature
# reaches the recursive state below: self.desired_curvature (via np.clip in
# clip_curvature) latches NaN PERMANENTLY, and the actuator finite-guard then
# zeroes curvature/steeringAngleDeg every frame — one glitch frame = a silent,
# unrecoverable 0-deg steering command for the rest of the drive.
# A short PURE HOLD of the last command rides out a transient glitch with no
# jerk. It must be BOUNDED, though: nothing downstream (selfdrived only checks
# frameDropPerc/posenetOK, never the value) faults on a model that keeps
# emitting non-finite actions, so an unbounded hold would keep the car turning
# on a stale curvature indefinitely with no alert. After the hold window the
# target is ramped to straight ("bleed, not hold").
# Kill switch: RAMP_END_S = HOLD_S = large -> pure hold.
MODEL_NONFINITE_HOLD_S = 0.20      # pure hold (identical to the last command)
MODEL_NONFINITE_RAMP_END_S = 2.20  # ramp factor reaches 0 here; it multiplies the previous output every frame, so the
                                   # command is at 0.76 / 0.09 / 0.00 of the held value 0.3 / 0.5 / 0.7 s into the gap


class Controls(ControlsExt):
  def __init__(self) -> None:
    self.params = Params()
    cloudlog.info("controlsd is waiting for CarParams")
    self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
    cloudlog.info("controlsd got CarParams")

    # Initialize sunnypilot controlsd extension and base model state
    ControlsExt.__init__(self, self.CP, self.params)
    self._lc_p = 0.0   # Phase 73c lane-centre term: low-passed proportional part
    self._lc_i = 0.0   #                             integral part
    self._lc_e = 0.0   # Phase 77: low-passed proportional part on the car's own offset
    self._lat_speed_ok = False   # Phase 75n: above the minimum steer speed on the last frame

    self.CI = interfaces[self.CP.carFingerprint](self.CP, self.CP_SP)

    self.sm = messaging.SubMaster(['lateralDelay', 'vehicleParameters', 'lateralTorqueParameters', 'modelV2', 'selfdriveState',
                                   'extrinsicsCalibration', 'deviceMotion', 'longitudinalPlan', 'lateralManeuverPlan', 'carState', 'carOutput',
                                   'driverMonitoringState', 'onroadEvents', 'driverAssistance'] + self.sm_services_ext,
                                  poll='selfdriveState')
    self.pm = messaging.PubMaster(['carControl', 'controlsState'] + self.pm_services_ext)

    self.steer_limited_by_safety = False
    self.curvature = 0.0
    self.bsm_guard = BsmLaneGuard(DT_CTRL)   # Phase 37b
    self.lane_dropout = False                # Phase 39
    self.lane_dropout_clear_frames = 0
    self.lane_dropout_frames = 0
    self.lane_dropout_low_frames = 0         # 39-2: consecutive frames below ENTRY_LM
    self.lane_dropout_armed = True           # 39-2: cleared on release, set back after REARM_S above LANE_HI
    self.lane_dropout_rearm_frames = 0
    self.lane_dropout_lc_holdoff = 0         # 39-2: frames left after a lane change / blinker
    self.lane_dropout_hold_k = 0.0           # 39-2: the command held while latched
    self.lane_dropout_k_hist = deque(maxlen=int(LANE_DROPOUT_ENTRY_S / DT_CTRL) + 1)   # 39-2: commands of the last ENTRY_S
    self._turn_peak_buf = deque(maxlen=int(TURN_EXIT_S / 0.1))   # Phase 56: per-0.1 s max |model action|
    self._turn_peak_cur = 0.0
    self._turn_peak_n = 0
    self._mf = None                          # _ModelFrame cache (see class)
    self.desired_curvature = 0.0
    self.predicted_lat_accel_ratio = 0.0
    self._model_nonfinite_frames = 0  # consecutive non-finite model actions
    self.lowconf_cap = LowConfRateCap(DT_CTRL)   # Phase 41: low-lane-confidence steering-rate cap
    self.steer_flags = 0                         # Phase 46: STEER_FLAG_* of the last frame

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose | None = None

    self.LoC = LongControl(self.CP, self.CP_SP)
    self.VM = VehicleModel(self.CP)
    self.LaC: LatControl
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CP_SP, self.CI, DT_CTRL)
    elif self.CP.steerControlType == car.CarParams.SteerControlType.curvature:
      self.LaC = LatControlCurvature(self.CP, self.CP_SP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CP_SP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CP_SP, self.CI, DT_CTRL)

    self.LaC = ControlsExt.initialize_lateral_control(self, self.LaC, self.CI, DT_CTRL)

  def update(self):
    self.sm.update(15)
    if self.sm.updated["extrinsicsCalibration"]:
      self.pose_calibrator.feed_extrinsics_calibration(self.sm['extrinsicsCalibration'])
    if self.sm.updated["deviceMotion"]:
      device_motion = Pose.from_device_motion(self.sm['deviceMotion'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_motion)

  def _model_frame(self, model_v2):
    key = (id(model_v2), self.sm.logMonoTime.get('modelV2', 0))
    mf = self._mf
    if mf is None or mf.key != key or mf.ref is not model_v2:
      mf = self._mf = _ModelFrame(model_v2, key)
    return mf

  def _turn_exit_update(self, k_car, k_action):
    """Phase 56: True while unwinding from a turn the CAR actually made — its curvature (from the wheel angle) peaked
    >= TURN_EXIT_K within the last TURN_EXIT_S and the model action is now <= 1/TURN_EXIT_RATIO of that peak. The car's
    curvature, not the action, so a model spike during a dropout (held, the car does not turn) never counts as a turn.
    Peak kept as per-0.1 s maxima (cheap at 100 Hz)."""
    c = abs(float(k_car)) if math.isfinite(k_car) else 0.0
    a = abs(float(k_action)) if math.isfinite(k_action) else 0.0
    self._turn_peak_cur = max(self._turn_peak_cur, c)
    self._turn_peak_n += 1
    if self._turn_peak_n >= int(round(0.1 / DT_CTRL)):
      self._turn_peak_buf.append(self._turn_peak_cur)
      self._turn_peak_cur = 0.0
      self._turn_peak_n = 0
    peak = max(max(self._turn_peak_buf, default=0.0), self._turn_peak_cur)
    return peak >= TURN_EXIT_K and peak >= TURN_EXIT_RATIO * a

  def _update_model_lat_delay(self):
    """Phase 64b: the lateral delay modeld is using (sunnypilot get_lat_delay: lagd's value with LagdToggle on, else the
    cached fixed value). The toggle / cache params are re-read every LOOKAHEAD_DELAY_PARAM_FRAMES."""
    n = getattr(self, "_lat_delay_param_n", 0)
    if n % LOOKAHEAD_DELAY_PARAM_FRAMES == 0:
      try:
        self._lagd_toggle = self.params.get_bool("LagdToggle")
        self._lagd_value_cache = float(self.params.get("LagdValueCache", return_default=True))
      except Exception:  # noqa: BLE001 - a param read must never take controls down; keep the last values
        pass
    self._lat_delay_param_n = n + 1
    if getattr(self, "_lagd_toggle", True):
      d = float(self.sm["lateralDelay"].lateralDelay)
    else:
      d = getattr(self, "_lagd_value_cache", LOOKAHEAD_DELAY_BASE_S)
    self._model_lat_delay = d if math.isfinite(d) and d > 0.0 else LOOKAHEAD_DELAY_BASE_S

  def _lane_center(self, mf, CS, k_action, usable_ctx):
    """Phase 73c: curvature added on the direct path to bring the plan (and the car) to the lane centre."""
    v = max(float(CS.vEgo), 1.0)
    p = getattr(self, "_lc_p", 0.0); integ = getattr(self, "_lc_i", 0.0)
    w = 0.0; raw = 0.0; e0 = 0.0; raw_e = 0.0
    pe = getattr(self, "_lc_e", 0.0)
    pf = getattr(self, "_lc_f", 0.0); raw_f = 0.0
    usual = getattr(self, "_lc_usual", None); wa = getattr(self, "_lc_wa", 1.0); seen = False; w_i = 0.0
    n_us = getattr(self, "_lc_usual_n", 0)
    if usable_ctx and LANE_CENTER_WEIGHT > 0.0 and math.isfinite(v):
      dist = float(_clip(v * LANE_CENTER_T_S, LANE_CENTER_L_M[0], LANE_CENTER_L_M[1]))
      pv = mf.lane_preview(dist)
      if pv is not None:
        c0, cd_, pd, w0, wd = pv
        if (LANE_CENTER_WIDTH_M[0] <= w0 <= LANE_CENTER_WIDTH_M[1] and LANE_CENTER_WIDTH_M[0] <= wd <= LANE_CENTER_WIDTH_M[1]
            and abs(wd / w0 - 1.0) <= LANE_CENTER_WIDTH_RATIO):
          w = (float(_interp(mf.lane_min_p, LANE_CENTER_PROB, [0.0, 1.0]))
               * float(_interp(v, LANE_CENTER_SPEED_MS, [0.0, 1.0])))
          if LANE_STATE_RESET and not math.isfinite(w):
            w = 0.0
          if LANE_AVOID_RELEASE and w > 0.0 and math.isfinite(k_action):          # Phase 83
            off = pd - cd_
            if usual is None:
              usual = off; n_us = 0
            # the usual spot is a plain average for the first LANE_AVOID_WARMUP_S (one frame is no 'usual'); no release yet
            warm = n_us * DT_CTRL >= LANE_AVOID_WARMUP_S
            dev = abs(off - usual) - LANE_AVOID_CURVE * abs(k_action) * dist ** 2 / 2.0
            wa_t = float(_interp(dev, LANE_AVOID_DEV_M, [1.0, 0.0])) if warm else 1.0
            wa = wa_t if wa_t < wa else wa + DT_CTRL / (LANE_AVOID_RECOVER_S + DT_CTRL) * (wa_t - wa)
            a_us = DT_CTRL / (LANE_AVOID_USUAL_S * (1.0 if wa >= 1.0 - 1e-6 else 2.0) + DT_CTRL)
            usual += max(a_us, 1.0 / (n_us + 1.0)) * (off - usual)
            n_us = min(n_us + 1, 10 ** 6)
            seen = True
            w *= wa
            w_i = w if wa >= 0.5 else 0.0                                          # mostly released: the integral is held
          else:
            w_i = w
          cap = LANE_CENTER_ACC / v ** 2
          raw = float(_clip(LANE_CENTER_WEIGHT * w * 2.0 * ((cd_ + LANE_CENTER_BIAS_M) - pd) / dist ** 2, -cap, cap))
          e0 = c0 + LANE_CENTER_BIAS_M
          ke = float(_interp(v, LANE_CENTER_KE_SPEED_MS, LANE_CENTER_KE))     # Phase 77: the car's own offset
          raw_e = float(_clip(ke * w * e0, -cap - raw, cap - raw)) if ke > 0.0 else 0.0   # both parts share the cap
          wf = w * float(_interp(v, LANE_FIT_SPEED_MS, [0.0, 1.0])) if LANE_FIT_GAIN > 0.0 else 0.0
          if wf > 0.0 and math.isfinite(k_action):                              # Phase 82: fit to the lane near the car
            ft = mf.lane_fit(float(_clip(v * LANE_FIT_RANGE_S, LANE_FIT_RANGE_M[0], LANE_FIT_RANGE_M[1])))
            if ft is not None:
              _, psi_l, k_l = ft
              lf = max(v * LANE_FIT_T_S, 10.0)
              fit = 1.432 * e0 / lf ** 2 + 1.653 * (psi_l - LANE_FIT_ARM_M * k_l) / lf + k_l
              raw_f = float(_clip(LANE_FIT_GAIN * wf * (fit - k_action), -cap - raw - raw_e, cap - raw - raw_e))
    p += DT_CTRL / (LANE_CENTER_TAU_S + DT_CTRL) * (raw - p)
    pe += DT_CTRL / (LANE_CENTER_TAU_S + DT_CTRL) * (raw_e - pe)
    pf += DT_CTRL / (LANE_CENTER_TAU_S + DT_CTRL) * (raw_f - pf)
    if w_i > 0.0:
      if abs(k_action) < LANE_CENTER_I_MAX_K and LANE_CENTER_KI > 0.0:
        integ = float(_clip(integ + LANE_CENTER_KI * w_i * e0 * DT_CTRL, -LANE_CENTER_I_CAP, LANE_CENTER_I_CAP))
    elif not seen or wa >= 0.5:                           # released for an avoidance: held, not bled
      integ *= max(1.0 - DT_CTRL / LANE_CENTER_I_BLEED_S, 0.0)
    # Phase 83 bookkeeping: no usable lane for LANE_AVOID_RESET_S forgets the usual spot (another road)
    gap = 0 if seen else getattr(self, "_lc_avoid_gap", 0) + 1
    if gap * DT_CTRL >= LANE_AVOID_RESET_S:
      usual = None
    if not seen:
      wa += DT_CTRL / (LANE_AVOID_RECOVER_S + DT_CTRL) * (1.0 - wa)
    if usual is not None and not math.isfinite(usual) or not math.isfinite(wa):
      usual = None; wa = 1.0
    self._lc_usual, self._lc_wa, self._lc_avoid_gap, self._lc_usual_n = usual, wa, gap, n_us
    if not (math.isfinite(p) and math.isfinite(integ) and math.isfinite(pe) and math.isfinite(pf)):
      p = integ = pe = pf = 0.0
    self._lc_p, self._lc_i, self._lc_e, self._lc_f = p, integ, pe, pf
    return p + integ + pe + pf

  def _entry_lead(self, mf, CS, k_action, usable_ctx):
    """Phase 78: curvature that starts the wheel ENTRY_LEAD_S earlier while a curve builds up (see ENTRY_LEAD_*)."""
    v = float(CS.vEgo); raw = 0.0
    el = getattr(self, "_entry_lead_lp", 0.0)
    if usable_ctx and ENTRY_LEAD_S > 0.0 and math.isfinite(v) and v >= ENTRY_LEAD_MIN_MS and math.isfinite(k_action):
      psi = mf.psi_plan
      if psi is not None:
        t1 = max(float(getattr(self, "_model_lat_delay", LOOKAHEAD_DELAY_BASE_S)), 0.05) + ENTRY_LEAD_FRAME_S
        t2 = t1 + ENTRY_LEAD_S
        d = (2.0 / v) * (float(np.interp(t2, _PLAN_T_IDXS, psi)) / t2 - float(np.interp(t1, _PLAN_T_IDXS, psi)) / t1)
        kf = k_action + d
        if math.isfinite(d) and d * kf > 0.0 and abs(kf) > abs(k_action):     # growing, toward the side it is going to
          raw = float(_clip(float(_interp(abs(kf), ENTRY_LEAD_K, [0.0, 1.0])) * d, -ENTRY_LEAD_CAP, ENTRY_LEAD_CAP))
    wo = getattr(self, "_entry_lead_wo", 0.0)
    wo += DT_CTRL / (ENTRY_LEAD_WASHOUT_S + DT_CTRL) * (raw - wo)
    pulse = raw - wo
    if pulse * raw <= 0.0:          # never the other way: when the lead ends the pulse is simply over
      pulse = 0.0
    el += DT_CTRL / (ENTRY_LEAD_TAU_S + DT_CTRL) * (pulse - el)
    if not (math.isfinite(el) and math.isfinite(wo)):
      el = wo = 0.0
    self._entry_lead_lp, self._entry_lead_wo = el, wo
    return el

  def _lookahead_curvature(self, model_v2):
    """The model's action for this frame (Phase 72 direct path; the look-ahead lead layers were removed in Phase 89),
    with the non-finite hold."""
    mf = self._model_frame(model_v2)
    fallback = mf.fallback

    # Non-finite model action: hold the last command briefly, then ramp to
    # straight (see MODEL_NONFINITE_* above). Nothing non-finite may reach the
    # recursive state in state_control().
    if not math.isfinite(fallback):
      self._model_nonfinite_frames += 1
      ramp = _clip((MODEL_NONFINITE_RAMP_END_S - self._model_nonfinite_frames * DT_CTRL) /
                   (MODEL_NONFINITE_RAMP_END_S - MODEL_NONFINITE_HOLD_S), 0.0, 1.0)
      return self.desired_curvature * float(ramp)
    self._model_nonfinite_frames = 0
    return fallback

  def _predicted_lat_accel_excess(self, model_v2, v_ego, lookahead_s=1.5):
    """Predicted v²·κ at lookahead_s ahead, normalized by LAT_ACCEL_ENVELOPE.

    Returns 0.0 when v_ego is too low, the trajectory is too short, or the
    polyfit is unusable. Used by curveSpeedAdvisory to give the driver a
    soft heads-up ~1.5 s before the angle-control envelope trips.
    """
    if v_ego < 2.0:
      return 0.0

    mf = self._model_frame(model_v2)
    if mf.n < 5:
      return 0.0

    dist_ahead = min(v_ego * lookahead_s, 30.0)
    if dist_ahead < 1.0:
      return 0.0

    # cubic fit of the first 24 plan points, computed once per model frame (_ModelFrame)
    x, c = mf.fit24
    if x[-1] < dist_ahead or c is None:
      return 0.0

    curv_pred = 6.0 * c[0] * dist_ahead + 2.0 * c[1]
    if not np.isfinite(curv_pred):
      return 0.0

    a_lat_pred = v_ego * v_ego * abs(curv_pred)
    return float(a_lat_pred / LAT_ACCEL_ENVELOPE)

  def state_control(self):
    CS = self.sm['carState']

    # Update VehicleModel
    lp = self.sm['vehicleParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    self.curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    # R2: self.lat_delay was referenced in the torque branch below but never
    # assigned (latent AttributeError on torque-tuned cars; angle cars like the
    # Ioniq 6 N never took that branch). Assign it once here and reuse downstream.
    self.lat_delay = self.sm["lateralDelay"].lateralDelay + LAT_SMOOTH_SECONDS
    self._update_model_lat_delay()   # Phase 64b

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['lateralTorqueParameters']
      if self.sm.all_checks(['lateralTorqueParameters']) and torque_params.useParams:
        self.LaC.update_torque_parameters(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

        self.LaC.extension.update_limits()

      self.LaC.extension.update_model_v2(self.sm['modelV2'])

      self.LaC.extension.update_lateral_lag(self.lat_delay)

    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']
    mf = self._model_frame(model_v2)    # per-model-frame derived values (see _ModelFrame)
    turn_exit = self._turn_exit_update(self.curvature, mf.fallback)   # Phase 56

    # curveSpeedAdvisory: EMA-filtered ratio of predicted v²·κ at 1.5 s
    # lookahead to LAT_ACCEL_ENVELOPE. Updated every frame (cheap) so the
    # advisory can fire just before latActive engages too.
    raw_ratio = self._predicted_lat_accel_excess(model_v2, CS.vEgo, lookahead_s=1.5)
    alpha = DT_CTRL / max(PRED_RATIO_TAU, DT_CTRL)
    self.predicted_lat_accel_ratio = (1.0 - alpha) * self.predicted_lat_accel_ratio + alpha * raw_ratio

    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled

    # Check which actuators can be enabled
    min_steer_speed = max(self.CP.minSteerSpeed, 0.3)
    if getattr(self, "_lat_speed_ok", False):   # Phase 75n: was above the minimum -> stays on down to the lower edge
      min_steer_speed = max(min_steer_speed - LAT_MIN_SPEED_HYST_MS, 0.3)
    standstill = abs(CS.vEgo) <= min_steer_speed or CS.standstill
    self._lat_speed_ok = not standstill

    # Get which state to use for active lateral control
    _lat_active = self.get_lat_active(self.sm)

    CC.latActive = _lat_active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                   (not standstill or self.CP.steerAtStandstill)
    CC.longActive = CC.enabled and not any(e.overrideLongitudinal for e in self.sm['onroadEvents']) and \
                    (self.CP.openpilotLongitudinalControl or not self.CP_SP.pcmCruiseSpeed)

    if hasattr(self.LaC, "request_paused"):
      self.LaC.request_paused = self.get_request_paused(self.sm)   # Phase 75k

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Enable blinkers while lane changing
    if mf.lane_change_state != LaneChangeState.off:
      CC.leftBlinker = mf.lane_change_dir == LaneChangeDirection.left
      CC.rightBlinker = mf.lane_change_dir == LaneChangeDirection.right

    if not CC.latActive:
      self.LaC.reset()
      self.bsm_guard.reset()   # Phase 37b: guard hold counter
      self.lowconf_cap.reset()   # Phase 41
      self.lane_dropout = False   # Phase 39
      self.lane_dropout_clear_frames = 0
      self.lane_dropout_frames = 0
      self.lane_dropout_low_frames = 0
      self.lane_dropout_k_hist.clear()      # 39-2 review: a stale command must never become the hold value
      self.lane_dropout_lc_holdoff = 0
      if LANE_STATE_RESET:
        self._conf_blend_lc_frames = 0   # Phase 84
      # lane_dropout_armed is deliberately NOT reset: a one-frame steer fault must not grant a second hold
    if not CC.longActive:
      self.LoC.reset()

    # accel PID loop
    pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, self.CP_SP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
    actuators.accel = float(self.LoC.update(CC.longActive, CS, long_plan.aTarget, long_plan.shouldStop, pid_accel_limits))

    # Steering PID loop and lateral MPC
    # Reset desired curvature to current to avoid violating the limits on engage
    new_desired_curvature = self._lookahead_curvature(model_v2) if CC.latActive else self.curvature
    steer_flags = 0   # Phase 46

    # Phase 73c: lane-centre term
    if CC.latActive:
      lc_ok = (mf.lane_change_state == LaneChangeState.off and not (CS.leftBlinker or CS.rightBlinker)
               and not CS.steeringPressed and not self.lane_dropout)
      if LANE_STATE_RESET and mf.lane_change_state != LaneChangeState.off:
        self._lc_usual = None   # Phase 84: the new lane has its own usual spot
      if LANE_CENTER_NONFINITE_GUARD and self._model_nonfinite_frames > 0:
        lc_add = 0.0   # Phase 75j: the held command already contains the term; states kept as they are
      else:
        k_act = new_desired_curvature
        lc_add = self._lane_center(mf, CS, k_act, lc_ok)
        el_add = self._entry_lead(mf, CS, k_act, lc_ok)          # Phase 78
        if abs(el_add) > 2e-5:
          steer_flags |= STEER_FLAG_ENTRY_LEAD
        lc_add += el_add
        if abs(getattr(self, "_lc_f", 0.0)) > 2e-5:
          steer_flags |= STEER_FLAG_LANE_FIT
        if lc_ok and getattr(self, "_lc_wa", 1.0) < 0.99:
          steer_flags |= STEER_FLAG_LANE_AVOID
      new_desired_curvature = new_desired_curvature + lc_add
      if abs(lc_add) > 2e-5:
        steer_flags |= STEER_FLAG_LANE_CENTER
    else:
      self._lc_p = 0.0
      self._lc_i = 0.0
      self._lc_e = 0.0
      if LANE_STATE_RESET:   # Phase 84
        self._lc_f = 0.0
        self._lc_usual = None; self._lc_wa = 1.0; self._lc_usual_n = 0; self._lc_avoid_gap = 0
      self._entry_lead_lp = 0.0
      self._entry_lead_wo = 0.0

    # Model uncertainty damping: when the model is unsure about lane position
    # (e.g. lead car occluding lane lines, ambiguous lane split), blend toward
    # previous curvature to suppress jittery steering commands.
    #
    # CCNC drivelog (route 0x05, 26% blinker, 70 osc/min during MADS) showed
    # yStd[5] p99=0.071 (max 0.10) — the legacy yStd>0.3 gate was effectively
    # dead code on this trim. Two-signal trigger: a much lower yStd threshold
    # AND laneLineProbs MIN < 0.5 (left/right inner-lane confidence). When
    # either fires, scale confidence proportionally and blend with previous.
    if CC.latActive and self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      y_std = mf.y_std
      lane_min = mf.lane_min
      conf_y = float(_interp(y_std,    [0.05, 0.30], [1.0, 0.0]))
      conf_l = float(_interp(lane_min, [0.05, 0.30], [0.0, 1.0]))
      # Phase 6g-1/6g-2: floor the damping so a low-confidence corner-entry
      # transition cannot freeze op near-straight (it ran the car wide to the
      # outside line) — but TAPER the floor to 0 at a true dropout so a spiking
      # low-confidence command is frozen out instead of half-passed (6g-2).
      floor_eff = LAT_CONF_FLOOR * float(_clip(
        (lane_min - CONF_FLOOR_LANE_LO) / (CONF_FLOOR_LANE_HI - CONF_FLOOR_LANE_LO), 0.0, 1.0))
      curve_w = float(_interp(abs(mf.fallback), LOWCONF_CURVE_K, [0.0, 1.0]))   # Phase 57: in a curve trust the plan
      confidence = max(min(conf_y, conf_l), floor_eff, curve_w)
      # Phase 39 / 39-2: dropout latch (see constants)
      if LANE_DROPOUT_LATCH:
        lc_busy = (mf.lane_change_state != LaneChangeState.off) or bool(CS.leftBlinker or CS.rightBlinker)
        if lc_busy:
          self.lane_dropout_lc_holdoff = int(LANE_DROPOUT_LC_HOLDOFF_S / DT_CTRL)
        else:
          self.lane_dropout_lc_holdoff = max(self.lane_dropout_lc_holdoff - 1, 0)
        hands_on = LANE_DROPOUT_HANDS_OFF and bool(CS.steeringPressed)                     # Phase 49
        # Phase 56c: the driver turning against the held command (opposing torque has the held curvature's sign)
        counter = self.lane_dropout and self.lane_dropout_hold_k * float(CS.steeringTorque) > 0.0 and \
          abs(float(CS.steeringTorque)) >= LANE_DROPOUT_COUNTER_TQ
        both_weak = lane_min < LANE_DROPOUT_ENTRY_LM and mf.lane_max < LANE_DROPOUT_MAX_LM   # Phase 49
        self.lane_dropout_low_frames = self.lane_dropout_low_frames + 1 if (both_weak and not hands_on) else 0
        if not self.lane_dropout:
          self.lane_dropout_k_hist.append(float(self.desired_curvature))
        if not self.lane_dropout_armed:
          self.lane_dropout_rearm_frames = self.lane_dropout_rearm_frames + 1 if lane_min > CONF_FLOOR_LANE_HI else 0
          if self.lane_dropout_rearm_frames >= int(LANE_DROPOUT_REARM_S / DT_CTRL):
            self.lane_dropout_armed = True
        if self.lane_dropout and (self.lane_dropout_lc_holdoff > 0 or CS.vEgo < LANE_DROPOUT_MIN_SPEED or hands_on or counter):
          self.lane_dropout = False                   # a lane change / slowing down / a driver press ends the hold at once
          self.lane_dropout_armed = False; self.lane_dropout_rearm_frames = 0
          self.lane_dropout_k_hist.clear()
        if (not self.lane_dropout and self.lane_dropout_armed and self.lane_dropout_lc_holdoff == 0 and not hands_on
            and not turn_exit and CS.vEgo >= LANE_DROPOUT_MIN_SPEED
            and self.lane_dropout_low_frames >= int(LANE_DROPOUT_ENTRY_S / DT_CTRL)):
          self.lane_dropout = True
          self.lane_dropout_clear_frames = 0
          self.lane_dropout_frames = 0
          # hold what op was doing BEFORE the blackout (ENTRY_S ago), not what the blend let through meanwhile
          self.lane_dropout_hold_k = float(self.lane_dropout_k_hist[0]) if len(self.lane_dropout_k_hist) else float(self.desired_curvature)
        elif self.lane_dropout:
          lines_back = (mf.lane_max if LANE_DROPOUT_EXIT_ON_ONE_LINE else lane_min) > CONF_FLOOR_LANE_HI   # Phase 49
          self.lane_dropout_clear_frames = self.lane_dropout_clear_frames + 1 if lines_back else 0
          if self.lane_dropout_clear_frames >= int(LANE_DROPOUT_EXIT_S / DT_CTRL):
            self.lane_dropout = False                 # lines are back: the blend takes over (armed stays True)
            self.lane_dropout_k_hist.clear()          # the next hold value must be from AFTER this release
        if self.lane_dropout:
          self.lane_dropout_frames += 1
          if self.lane_dropout_frames > int(LANE_DROPOUT_MAX_S / DT_CTRL):
            self.lane_dropout = False                 # max duration: back to the 6g blend, no re-latch until the lines return
            self.lane_dropout_armed = False; self.lane_dropout_rearm_frames = 0
            self.lane_dropout_k_hist.clear()
      # Phase 80b: op's own lane change (model laneChangeStarting) and CONF_BLEND_LC_AFTER_S after it are not blended:
      # the lane lines vanish by design there, and the blend held the turn-in while the model was already unwinding
      # (a9 503.4-504.1: +18e-4 held, model +11.6 -> -12.8e-4). Not on a bare blinker / preLaneChange.
      if CONF_BLEND_LC_EXEMPT and mf.lane_change_state == LaneChangeState.laneChangeStarting:
        self._conf_blend_lc_frames = int(CONF_BLEND_LC_AFTER_S / DT_CTRL)
      else:
        self._conf_blend_lc_frames = max(getattr(self, "_conf_blend_lc_frames", 0) - 1, 0)
      conf_blend_lc = CONF_BLEND_LC_EXEMPT and self._conf_blend_lc_frames > 0
      if self.lane_dropout:
        new_desired_curvature = self.lane_dropout_hold_k
      elif confidence < 1.0 and not conf_blend_lc and not (TURN_EXIT_SOFT_EXEMPT and turn_exit and abs(new_desired_curvature) < abs(self.desired_curvature)):
        # (Phase 56d: unwinding out of a turn is not frozen by the low-confidence blend — it only ever moves toward straight)
        new_desired_curvature = confidence * new_desired_curvature + (1.0 - confidence) * self.desired_curvature
        steer_flags |= STEER_FLAG_CONF_BLEND
      if self.lane_dropout:
        steer_flags |= STEER_FLAG_LANE_DROPOUT
      # Phase 41: while the weaker inner lane line is below LOWCONF_LANE_MIN (merge / split / vanishing line) and the
      # driver is not holding the wheel, the command may change no faster than LOWCONF_CAP_DPS of steer angle per
      # second — a plan re-centring on an uncertain lane becomes a slow drift the driver can veto, not a snatch
      # (report §23: 14 of 21 driver-fought swings sat in this band; nothing in the logs tells them from legitimate
      # ones, so this is a rate policy, not a gate). Applied after the blend / dropout hold and before the departure
      # and BSM guards (those only ever hold, so they cannot re-introduce a step). See lowconf_rate_cap.py.
      if LOWCONF_CAP_DPS > 0.0:
        blinker_or_lc = (mf.lane_change_state != LaneChangeState.off) or bool(CS.leftBlinker or CS.rightBlinker)
        # Phase 56b: unwinding out of a turn is a manoeuvre, not a re-centring snatch — no cap toward straight
        unwinding = TURN_EXIT_SOFT_EXEMPT and turn_exit and abs(new_desired_curvature) < abs(self.desired_curvature)
        k_per_deg = abs(float(self.VM.calc_curvature(math.radians(1.0), max(float(CS.vEgo), 1.0), 0.0)))
        release_left_before = self.lowconf_cap.release_left   # the ramp decrements inside update(): read it first
        cap_dps = LOWCONF_CAP_DPS + (LOWCONF_RELEASE_DPS - LOWCONF_CAP_DPS) * curve_w   # Phase 57: 6 -> 30 deg/s in a curve
        new_desired_curvature = self.lowconf_cap.update(new_desired_curvature, self.desired_curvature, lane_min,
                                                        bool(CS.steeringPressed), blinker_or_lc or unwinding, float(CS.vEgo), k_per_deg,
                                                        cap_dps=cap_dps)
        if self.lowconf_cap.active:
          steer_flags |= STEER_FLAG_LOWCONF_GATE
        elif release_left_before > 0 and not (CS.steeringPressed or blinker_or_lc or unwinding):   # a press / blinker cancels the ramp
          steer_flags |= STEER_FLAG_LOWCONF_RELEASE
        if self.lowconf_cap.capped:
          steer_flags |= STEER_FLAG_LOWCONF_CAPPED

      # Lane-departure protection: when blinker is OFF, do not let op steer
      # FURTHER into a lane line flagged as departing. The input is
      # driverAssistance.{left,right}LaneDeparture, published by plannerd from
      # ldw.py (op-active predictor: sustained monotonic approach < 0.7 m) —
      # NOT the camera ECU (comment corrected, Phase 37b). With blinker on,
      # the driver intends a lane change so the gate is relaxed.
      blinker_on = bool(CS.leftBlinker or CS.rightBlinker)
      if not blinker_on and self.sm.valid['driverAssistance']:
        da = self.sm['driverAssistance']
        # left line departure → block further negative-curvature (=more left) commands
        # right line departure → block further positive-curvature (=more right) commands
        if (da.leftLaneDeparture and new_desired_curvature < self.desired_curvature) or \
           (da.rightLaneDeparture and new_desired_curvature > self.desired_curvature):
          new_desired_curvature = self.desired_curvature
          steer_flags |= STEER_FLAG_DEPARTURE_HOLD
      # Phase 37b: BSM lane guard — radar-occupied side + lane line close on
      # that side + no blinker toward it -> hold curvature (see bsm_guard.py).
      # Called EVERY active frame (no BSM short-circuit) so the hold-timeout
      # counter resets on the first unblocked frame — review: a short-circuit
      # latched an exhausted counter across the BSM-off gap and silently
      # left the next episode unguarded.
      if BSM_LANE_GUARD_M > 0.0:
        if mf.bsm_lane is not None:      # (lane-line inputs read once per model frame; unreadable -> reset, as before)
          _yl, _yr, _pl, _pr = mf.bsm_lane
          new_desired_curvature, _bsm_held = self.bsm_guard.update(
            new_desired_curvature, self.desired_curvature,
            bool(CS.leftBlindspot), bool(CS.rightBlindspot),
            bool(CS.leftBlinker), bool(CS.rightBlinker),
            _yl, _yr, _pl, _pr,
            BSM_LANE_GUARD_M, BSM_LANE_GUARD_MIN_PROB)
          if _bsm_held:
            steer_flags |= STEER_FLAG_BSM_HOLD
        else:
          self.bsm_guard.reset()
    if CC.latActive:
      steer_flags |= STEER_FLAG_DIRECT   # Phase 72
    self.steer_flags = steer_flags   # Phase 46: published in controlsState.steerFlags

    self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, new_desired_curvature, lp.roll)
    lat_delay = self.lat_delay  # R2: assigned once above

    actuators.curvature = self.desired_curvature
    steer, lateral_output, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                     self.steer_limited_by_safety, self.desired_curvature,
                                                     self.calibrated_pose, curvature_limited, lat_delay)
    actuators.torque = float(steer)
    if self.CP.steerControlType == car.CarParams.SteerControlType.curvature:
      actuators.curvature = float(lateral_output)
    else:
      actuators.steeringAngleDeg = float(lateral_output)
    # Ensure no NaNs/Infs
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, Number):
        continue

      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)

    return CC, lac_log

  def publish(self, CC, lac_log):
    CS = self.sm['carState']

    # Orientation and angle rates can be useful for carcontroller
    # Only calibrated (car) frame is relevant for the carcontroller
    CC.currentCurvature = self.curvature
    if self.calibrated_pose is not None:
      CC.orientationNED = self.calibrated_pose.orientation.xyz.tolist()
      CC.angularVelocity = self.calibrated_pose.angular_velocity.xyz.tolist()

    CC.cruiseControl.override = CC.enabled and not CC.longActive and (self.CP.openpilotLongitudinalControl or not self.CP_SP.pcmCruiseSpeed)
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (not CC.enabled or not self.CP.pcmCruise)
    CC.cruiseControl.resume = CC.enabled and CS.cruiseState.standstill and not self.sm['longitudinalPlan'].shouldStop

    hudControl = CC.hudControl
    hudControl.setSpeed = float(CS.vCruiseCluster * CV.KPH_TO_MS)
    hudControl.speedVisible = CC.enabled
    hudControl.lanesVisible = CC.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead
    hudControl.leadDistanceBars = self.sm['selfdriveState'].personality.raw + 1
    hudControl.visualAlert = self.sm['selfdriveState'].alertHudVisual

    hudControl.rightLaneVisible = True
    hudControl.leftLaneVisible = True
    if self.sm.valid['driverAssistance']:
      hudControl.leftLaneDepart = self.sm['driverAssistance'].leftLaneDeparture
      hudControl.rightLaneDepart = self.sm['driverAssistance'].rightLaneDeparture

    if self.get_lat_active(self.sm):
      CO = self.sm['carOutput']
      if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        self.steer_limited_by_safety = abs(CC.actuators.steeringAngleDeg - CO.actuatorsOutput.steeringAngleDeg) > \
                                              STEER_ANGLE_SATURATION_THRESHOLD
      else:
        self.steer_limited_by_safety = abs(CC.actuators.torque - CO.actuatorsOutput.torque) > 1e-2

    # TODO: both controlsState and carControl valids should be set by
    #       sm.all_checks(), but this creates a circular dependency

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    cs = dat.controlsState

    cs.curvature = self.curvature
    cs.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    cs.lateralPlanMonoTime = self.sm.logMonoTime['modelV2']
    cs.desiredCurvature = self.desired_curvature
    cs.laneDropout = bool(self.lane_dropout)   # Phase 39
    cs.steerCmdGapDeg = float(CC.actuators.steeringAngleDeg - CS.steeringAngleDeg) if self.CP.steerControlType == car.CarParams.SteerControlType.angle else 0.0
    cs.steerFlags = int(self.steer_flags)   # Phase 46
    cs.longControlState = self.LoC.long_control_state
    cs.upAccelCmd = float(self.LoC.pid.p)
    cs.uiAccelCmd = float(self.LoC.pid.i)
    cs.ufAccelCmd = float(self.LoC.pid.f)
    cs.forceDecel = bool(self.sm['driverMonitoringState'].noResponseForceDecel or
                         (self.sm['selfdriveState'].state == State.softDisabling))
    # i6nv2: predictedLatAccelRatio is an i6n ControlsState extension absent from the
    # 04-10 base cereal schema. Publishing it would AttributeError-crash controlsd on
    # engage. The ratio is still computed internally; only the (telemetry-only) publish
    # is dropped. curveSpeedAdvisory (its only consumer) is likewise disabled in selfdrived.

    # trigger the car's stock driver monitoring escalation
    CC.driverMonitoringEscalation = cs.forceDecel

    lat_tuning = self.CP.lateralTuning.which()
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      cs.lateralControlState.angleState = lac_log
    elif self.CP.steerControlType == car.CarParams.SteerControlType.curvature:
      cs.lateralControlState.curvatureState = lac_log
    elif lat_tuning == 'pid':
      cs.lateralControlState.pidState = lac_log
    elif lat_tuning == 'torque':
      cs.lateralControlState.torqueState = lac_log

    self.pm.send('controlsState', dat)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

  def run(self):
    rk = Ratekeeper(100, print_delay_threshold=None)
    while True:
      self.update()
      CC, lac_log = self.state_control()
      self.publish(CC, lac_log)
      self.get_params_sp(self.sm)
      self.run_ext(self.sm, self.pm)
      rk.monitor_time()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  controls = Controls()
  controls.run()


if __name__ == "__main__":
  main()
