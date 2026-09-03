#!/usr/bin/env python3
import math
from numbers import Number

import numpy as np

from cereal import car, log
import cereal.messaging as messaging
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_CTRL, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog

from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from openpilot.selfdrive.controls.lib.bsm_guard import BsmLaneGuard, BSM_LANE_GUARD_M, BSM_LANE_GUARD_MIN_PROB
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose

from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt

State = log.SelfdriveState.OpenpilotState
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

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

# --- Lateral command temporal smoothing (jerk reduction with lead compensation) ---
# From the comma controls-challenge result: tracking a jittery target in full pays its
# jerk; a FORWARD-LOOKING (lead) low-pass cuts jerk WITHOUT net lag because openpilot
# already knows the future path (Phase 6f-4 history: constant TAU/LEAD 0.06 -> 0.10).
# Phase 6h-1: speed-dependent command low-pass with MATCHED lead, replacing the
# heavy low-speed angle-domain EMA. Offline pre-validation (ccnc-drivelog 0x40/0x42,
# 8 segs): total tau 0.10->0.20 cuts <30 km/h 3-7 Hz desiredCurvature RMS 3.2x
# (1.80e-4 -> 5.62e-5 1/m); tau 0.30 adds nothing IN THAT 3-7 Hz BAND. Lead is
# matched per-speed so net phase stays ~0 (same principle as 6f-4).
# Phase 34: low-speed node 0.20 -> 0.30. The felt low-speed shake band is
# 0.8-2.5 Hz (not 3-7), and there tau 0.30 DOES help: offline replay of the
# full pipeline (LP + matched lead + 6h-2 bound) on 0x51-0x54 + 0x4c/0x4e
# model streams measures, vs 0.20 —
#   quasi-straight 3-30 km/h 0.8-2.5 Hz command churn:  -14%
#   1 s windows punching through the 0.5 deg TX hysteresis: 71% -> 52%
#   turn-initiation t63, plan-foreseen (lead active):    ~0 (slightly leads)
#   turn-initiation t63, lead-off (plan did not foresee): +0.245 s (was +0.175)
#   S-reversal zero-cross lag / peak retention:          +0.07 s / 0.93 (was 0.96)
# HONESTY NOTES (verification round):
# (a) the onset bracket is wide (-0.060..+0.245 = 305 ms) and lead-off is a
#     FLOOR on the replan worst case, not the worst case — right after a
#     replan the lead points at the OLD future and must unwind first, which
#     is worse than no lead. Margin to the 7a-5/33b failure region is
#     therefore smaller than the table reads.
# (b) 7c entry assist is NOT tau-independent: it consumes the leaded
#     new_desired_curvature, and at 7-8 m/s the tau-driven dk_max growth
#     (+0.0011..0.0014) exceeds the whole ENTRY_ASSIST_CAP (0.001). Benign
#     by substitution — lead and assist add in the SAME direction at entry,
#     so net entry command does not drop — but corner-onset t63 above is
#     slightly optimistic for it.
# (c) dist_ahead grows ~21% at low speed, so the x[-1] < dist_ahead fallback
#     (raw model curvature, no lead — safe degradation) fires somewhat more
#     often on stop approaches; rate unmeasured (corpus lacks position.x),
#     distiller now records it for the next rounds.
# tau 0.40/0.50 rejected: reversal retention 0.90/0.87 and replan +140/+200 ms
# approach the 7a-5/33b failure territory for -6/-11%p more churn. A
# gate-release variant (fast-tau when |input-output| large) measured 96-108%
# of current churn = no gain, but that TESTED VARIANT was invalid by
# construction: it released the LP while the lead horizon stayed sized for
# the slow tau, so net lead > lag re-injected band energy. A correctly
# matched gate (t_ahead recomputed from the instantaneous tau) is UNTESTED
# and remains the natural mitigation if the replan lag surfaces on-road —
# it would need its own validation (switching transient).
# Kill switch: TAU_V = [0.10]*3 (pre-6h-1); previous behaviour: [0.20, 0.12, 0.08].
LAT_CMD_SMOOTH_TAU_BP = [8.0, 13.0, 18.0]   # m/s
LAT_CMD_SMOOTH_TAU_V  = [0.30, 0.12, 0.08]

# Phase 6h-2: the lookahead lead is ADDITIVE and BOUNDED on top of the model plan
# instead of replacing it. The budget is the curvature equivalent of this much
# lateral jerk over the lookahead horizon (dk_max = J * t_ahead / v^2), so it
# scales with the 6h-1 matched lead: at low speed (tau 0.20) the budget is ~25%
# larger than the standalone pre-validation numbers, by design (lead-horizon
# proportional). Kill switch: 1e9 (= replace behaviour, pre-6h-2).
LOOKAHEAD_JERK_BUDGET = 0.7   # m/s^3
# Phase 7c-2: corner-entry lead cap. The corner under-steer is mostly ENTRY LAG
# (pooled 0x50-0x5b: achieved/desired 0.82 instantaneous vs 0.90 steady — the gap
# is lag; cross-correlation showed ~170 ms residual desired->achieved lag AFTER
# the current lookahead). Lag cannot be closed by the 7a integral without
# winding up and over-cutting on exit (the 7a-3/7a-4 over-correction reverted in
# 7a-5, which crossed a lane at the Seoul-Station S-curve). Feed-forward LEAD is
# the right tool — it anticipates instead of reacting, so no windup. Raise the
# t_ahead cap a SMALL step (0.25 -> 0.27 s, +20 ms) as a conservative probe; the
# additive lead stays bounded by LOOKAHEAD_JERK_BUDGET (dk_max) so it cannot
# over-command beyond the J=0.7 envelope even on an uncertain reverse curve.
# Validate at the S-curve (the overshoot worst case) before any larger step.
# Kill switch: LOOKAHEAD_T_AHEAD_CAP = 0.25 (previous behaviour).
LOOKAHEAD_T_AHEAD_CAP = 0.27  # s

# Phase 7c: confidence-gated geometry ENTRY ASSIST. The model plan under-commands
# corner entry vs lane geometry (0x44-0x4a: entry op/k_lane p50 0.83-0.93,
# under<0.6 32-41%) — invisible to the 7a feedback (desired-vs-achieved). When a
# well-tracked (lane_min>0.6) medium+ corner (|k_lane|>0.003) is BUILDING
# (|desired| rising) and the plan is short of the lane-implied curvature in the
# SAME direction, add the bounded shortfall. Replay (0x49/0x4a/0x42 incl. the
# seg4 S-reversal fixture): fires on 0.6-1.1% of op-active frames (entry only),
# assist p50 6-10e-4 (cap-bound), 0 fires inside the seg4 reversal window (the
# sign+rising gates stay silent through reversals). Applied BEFORE confidence
# damping and the 6h-1 LP (ramped in smoothly) and bounded downstream by
# clip_curvature/VM/panda as usual. Kill switch: ENTRY_ASSIST_CAP = 0.0.
ENTRY_ASSIST_CAP = 1e-3       # 1/m absolute cap
ENTRY_ASSIST_REL = 0.5        # ... and at most +50% of the plan
ENTRY_ASSIST_KLANE_MIN = 0.003
ENTRY_ASSIST_LANE_MIN = 0.6
ENTRY_ASSIST_MIN_SPEED = 7.0  # m/s

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

# Non-finite model action handling. A NaN/inf model_v2.action.desiredCurvature
# bypasses every isfinite check in _lookahead_curvature (they only cover the
# SAMPLED TRAJECTORY, not the fallback) and reaches the recursive state below:
# _lat_cmd_lp, _absdc_slow and self.desired_curvature (via np.clip in
# clip_curvature) all latch NaN PERMANENTLY, and the actuator finite-guard then
# zeroes curvature/steeringAngleDeg every frame — one glitch frame = a silent,
# unrecoverable 0-deg steering command for the rest of the drive.
# A short PURE HOLD of the last command rides out a transient glitch with no
# jerk. It must be BOUNDED, though: nothing downstream (selfdrived only checks
# frameDropPerc/posenetOK, never the value) faults on a model that keeps
# emitting non-finite actions, so an unbounded hold would keep the car turning
# on a stale curvature indefinitely with no alert. After the hold window the
# target is ramped to straight — same "bleed, not hold" rule the Phase 7a trim
# uses for its anti-windup. Kill switch: RAMP_END_S = HOLD_S = large -> pure hold.
MODEL_NONFINITE_HOLD_S = 0.20      # pure hold (identical to the last command)
MODEL_NONFINITE_RAMP_END_S = 2.20  # target fully ramped to 0 by here


class Controls(ControlsExt):
  def __init__(self) -> None:
    self.params = Params()
    cloudlog.info("controlsd is waiting for CarParams")
    self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
    cloudlog.info("controlsd got CarParams")

    # Initialize sunnypilot controlsd extension and base model state
    ControlsExt.__init__(self, self.CP, self.params)

    self.CI = interfaces[self.CP.carFingerprint](self.CP, self.CP_SP)

    self.sm = messaging.SubMaster(['liveDelay', 'liveParameters', 'liveTorqueParameters', 'modelV2', 'selfdriveState',
                                   'liveCalibration', 'livePose', 'longitudinalPlan', 'carState', 'carOutput',
                                   'driverMonitoringState', 'onroadEvents', 'driverAssistance', 'liveDelay'] + self.sm_services_ext,
                                  poll='selfdriveState')
    self.pm = messaging.PubMaster(['carControl', 'controlsState'] + self.pm_services_ext)

    self.steer_limited_by_safety = False
    self.curvature = 0.0
    self.bsm_guard = BsmLaneGuard(DT_CTRL)   # Phase 37b
    self.desired_curvature = 0.0
    self.predicted_lat_accel_ratio = 0.0
    self._lat_cmd_lp = 0.0  # state for the LAT_CMD_SMOOTH_TAU_* low-pass
    self._klane_lp = 0.0     # Phase 7c lane-geometry curvature (EMA 0.3 s)
    self._absdc_slow = 0.0   # Phase 7c rising-entry detector state
    self._model_nonfinite_frames = 0  # consecutive non-finite model actions

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose | None = None

    self.LoC = LongControl(self.CP, self.CP_SP)
    self.VM = VehicleModel(self.CP)
    self.LaC: LatControl
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CP_SP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CP_SP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CP_SP, self.CI, DT_CTRL)

    self.LaC = ControlsExt.initialize_lateral_control(self, self.LaC, self.CI, DT_CTRL)

  def update(self):
    self.sm.update(15)
    if self.sm.updated["liveCalibration"]:
      self.pose_calibrator.feed_live_calib(self.sm['liveCalibration'])
    if self.sm.updated["livePose"]:
      device_pose = Pose.from_live_pose(self.sm['livePose'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_pose)

  def _lookahead_curvature(self, model_v2, v_ego, lookahead_extra_s):
    """Phase 7: sample modelV2 trajectory at adaptive look-ahead distance.

    Phase 6h-1: lookahead_extra_s is the caller's LP time constant tau(v) so the
    phase lead always matches the smoothing lag (was a fixed 0.10 s constant)."""
    fallback = model_v2.action.desiredCurvature

    # Non-finite model action: hold the last command briefly, then ramp to
    # straight (see MODEL_NONFINITE_* above). Nothing non-finite may reach the
    # recursive state in state_control().
    if not math.isfinite(fallback):
      self._model_nonfinite_frames += 1
      ramp = np.clip((MODEL_NONFINITE_RAMP_END_S - self._model_nonfinite_frames * DT_CTRL) /
                     (MODEL_NONFINITE_RAMP_END_S - MODEL_NONFINITE_HOLD_S), 0.0, 1.0)
      return self.desired_curvature * float(ramp)
    self._model_nonfinite_frames = 0

    # capnp _DynamicListReader does not support slicing, so use np.fromiter
    pos_x = model_v2.position.x
    pos_y = model_v2.position.y
    n = len(pos_x)
    if n < 5:
      return fallback

    abs_curv = abs(fallback)
    # Phase 6h-2: gate lowered 0.001 -> 0.0008 to match the continuous blend at
    # the return (below 0.0008 the blend is 0, so behaviour is identical — this
    # only removes the replace/fallback discontinuity at the old gate).
    if abs_curv < 0.0008:
      return fallback
    # Phase 6f-5 lane-tracking responsiveness:
    #   base_s: extend the high-speed shelf to 140 km/h (38.9 m/s) -> 0.18 s so
    #     highway cruise (frequent Gyeongbu expressway runs to Yongin) sees an
    #     earlier preview of lane drift. The 0.13 s shelf at 100 km/h is kept
    #     as the third node so anything <=100 km/h is unchanged.
    #   boost_s: raise the corner peak from 0.12 -> 0.20 s while keeping the
    #     entry threshold (|curv|>=0.001) intact, so straight-line 4-5 Hz
    #     wobble (the band Phase 6f-4 just cut) is not amplified. ccnc-drivelog
    #     0x3a-0x3f cross-correlation showed the existing pipeline gives op a
    #     ~+87 ms duration-weighted lead over the wheel; this widens that lead
    #     in shallow-to-medium corners (where it matters most for entry feel).
    base_s = float(np.interp(v_ego, [5.6, 13.9, 27.8, 38.9], [0.08, 0.10, 0.13, 0.18]))
    boost_s = float(np.interp(abs_curv, [0.0008, 0.005], [0.0, 0.20]))  # R1: start aligned with the 0.0008 gate/blend (was 0.001)
    t_ahead = min(base_s + boost_s + lookahead_extra_s, LOOKAHEAD_T_AHEAD_CAP + lookahead_extra_s)
    dist_ahead = min(v_ego * t_ahead, 10.0)

    if dist_ahead < 0.3:
      return fallback

    n = min(n, 12)
    x = np.fromiter((pos_x[i] for i in range(n)), dtype=np.float64, count=n)
    y = np.fromiter((pos_y[i] for i in range(n)), dtype=np.float64, count=n)

    if x[-1] < dist_ahead or not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
      return fallback

    try:
      c = np.polyfit(x, y, 3)
    except (np.linalg.LinAlgError, ValueError):
      return fallback

    curv = 6.0 * c[0] * dist_ahead + 2.0 * c[1]
    if not np.isfinite(curv):
      return fallback
    # Phase 6h-2: ADDITIVE, BOUNDED lead instead of replacement. Pre-validation:
    # 0x42 seg4 spike: replace=0.0155 (model plan 0.0120 x1.30) -> bounded
    # J=0.7 = 0.0141; straights (|fb|<0.0015, n=2668) injected-noise p99
    # 0.00092->0.00058 (-37%); normal corners (n=1192) ratio p50/p90 unchanged.
    dk_max = LOOKAHEAD_JERK_BUDGET * max(t_ahead, 0.05) / max(v_ego, 5.0) ** 2
    blend = float(np.interp(abs_curv, [0.0008, 0.0015], [0.0, 1.0]))
    return fallback + blend * float(np.clip(float(curv) - fallback, -dk_max, dk_max))

  def _predicted_lat_accel_excess(self, model_v2, v_ego, lookahead_s=1.5):
    """Predicted v²·κ at lookahead_s ahead, normalized by LAT_ACCEL_ENVELOPE.

    Returns 0.0 when v_ego is too low, the trajectory is too short, or the
    polyfit is unusable. Used by curveSpeedAdvisory to give the driver a
    soft heads-up ~1.5 s before the angle-control envelope trips.
    """
    if v_ego < 2.0:
      return 0.0

    pos_x = model_v2.position.x
    pos_y = model_v2.position.y
    n = len(pos_x)
    if n < 5:
      return 0.0

    dist_ahead = min(v_ego * lookahead_s, 30.0)
    if dist_ahead < 1.0:
      return 0.0

    n = min(n, 24)
    x = np.fromiter((pos_x[i] for i in range(n)), dtype=np.float64, count=n)
    y = np.fromiter((pos_y[i] for i in range(n)), dtype=np.float64, count=n)

    if x[-1] < dist_ahead or not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
      return 0.0

    try:
      c = np.polyfit(x, y, 3)
    except (np.linalg.LinAlgError, ValueError):
      return 0.0

    curv_pred = 6.0 * c[0] * dist_ahead + 2.0 * c[1]
    if not np.isfinite(curv_pred):
      return 0.0

    a_lat_pred = v_ego * v_ego * abs(curv_pred)
    return float(a_lat_pred / LAT_ACCEL_ENVELOPE)

  def state_control(self):
    CS = self.sm['carState']

    # Update VehicleModel
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    self.curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    # R2: self.lat_delay was referenced in the torque branch below but never
    # assigned (latent AttributeError on torque-tuned cars; angle cars like the
    # Ioniq 6 N never took that branch). Assign it once here and reuse downstream.
    self.lat_delay = self.sm["liveDelay"].lateralDelay + LAT_SMOOTH_SECONDS

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['liveTorqueParameters']
      if self.sm.all_checks(['liveTorqueParameters']) and torque_params.useParams:
        self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

        self.LaC.extension.update_limits()

      self.LaC.extension.update_model_v2(self.sm['modelV2'])

      self.LaC.extension.update_lateral_lag(self.lat_delay)

    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']

    # curveSpeedAdvisory: EMA-filtered ratio of predicted v²·κ at 1.5 s
    # lookahead to LAT_ACCEL_ENVELOPE. Updated every frame (cheap) so the
    # advisory can fire just before latActive engages too.
    raw_ratio = self._predicted_lat_accel_excess(model_v2, CS.vEgo, lookahead_s=1.5)
    alpha = DT_CTRL / max(PRED_RATIO_TAU, DT_CTRL)
    self.predicted_lat_accel_ratio = (1.0 - alpha) * self.predicted_lat_accel_ratio + alpha * raw_ratio

    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled

    # Check which actuators can be enabled
    standstill = abs(CS.vEgo) <= max(self.CP.minSteerSpeed, 0.3) or CS.standstill

    # Get which state to use for active lateral control
    _lat_active = self.get_lat_active(self.sm)

    CC.latActive = _lat_active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                   (not standstill or self.CP.steerAtStandstill)
    CC.longActive = CC.enabled and not any(e.overrideLongitudinal for e in self.sm['onroadEvents']) and \
                    (self.CP.openpilotLongitudinalControl or not self.CP_SP.pcmCruiseSpeed)

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Enable blinkers while lane changing
    if model_v2.meta.laneChangeState != LaneChangeState.off:
      CC.leftBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.left
      CC.rightBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.right

    if not CC.latActive:
      self.LaC.reset()
      self.bsm_guard.reset()   # Phase 37b: guard hold counter
    if not CC.longActive:
      self.LoC.reset()

    # accel PID loop
    pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, self.CP_SP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
    actuators.accel = float(self.LoC.update(CC.longActive, CS, long_plan.aTarget, long_plan.shouldStop, pid_accel_limits))

    # Steering PID loop and lateral MPC
    # Reset desired curvature to current to avoid violating the limits on engage
    # Phase 6h-1 order: tau(v) first, so the lookahead lead matches the LP lag below.
    lat_smooth_tau = float(np.interp(CS.vEgo, LAT_CMD_SMOOTH_TAU_BP, LAT_CMD_SMOOTH_TAU_V))
    new_desired_curvature = self._lookahead_curvature(model_v2, CS.vEgo, lat_smooth_tau) if CC.latActive else self.curvature

    # Phase 7c geometry entry assist (constants above).
    if ENTRY_ASSIST_CAP > 0.0 and CC.latActive:
      try:
        lls = model_v2.laneLines
        probs = model_v2.laneLineProbs
        lane_min_p = min(float(probs[1]), float(probs[2])) if len(probs) >= 3 else 0.0
        xl = np.fromiter((x for x in lls[1].x), dtype=np.float64)
        yl = np.fromiter((y for y in lls[1].y), dtype=np.float64)
        xr = np.fromiter((x for x in lls[2].x), dtype=np.float64)
        yr = np.fromiter((y for y in lls[2].y), dtype=np.float64)
        lc = lambda xq: 0.5 * (np.interp(xq, xl, yl) + np.interp(xq, xr, yr))
        klane_raw = (lc(0.0) - 2.0 * lc(25.0) + lc(50.0)) / 625.0
        a = DT_CTRL / (0.3 + DT_CTRL)
        # NaN lane-line y values pass the length checks and np.interp forwards
        # them; one such frame would poison the EMA (abs(NaN) gates all go
        # False) and silently disable entry assist for the rest of the drive.
        if np.isfinite(klane_raw):
          self._klane_lp += a * (float(klane_raw) - self._klane_lp)
        self._absdc_slow += (DT_CTRL / 0.5) * (abs(new_desired_curvature) - self._absdc_slow)
        rising = abs(new_desired_curvature) > self._absdc_slow * 1.02
        if (CS.vEgo > ENTRY_ASSIST_MIN_SPEED and lane_min_p > ENTRY_ASSIST_LANE_MIN
            and model_v2.meta.laneChangeState == log.LaneChangeState.off
            and abs(self._klane_lp) > ENTRY_ASSIST_KLANE_MIN and rising
            and np.sign(self._klane_lp) == np.sign(new_desired_curvature)
            and abs(self._klane_lp) > abs(new_desired_curvature)):
          shortfall = abs(self._klane_lp) - abs(new_desired_curvature)
          assist = min(shortfall, ENTRY_ASSIST_CAP, ENTRY_ASSIST_REL * abs(new_desired_curvature))
          new_desired_curvature = new_desired_curvature + float(np.sign(new_desired_curvature)) * assist
      except (IndexError, ValueError):
        pass

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
      y_std_list = model_v2.position.yStd
      lane_probs = model_v2.laneLineProbs
      y_std = float(y_std_list[5]) if len(y_std_list) > 5 else 0.0
      lane_min = min(float(lane_probs[1]), float(lane_probs[2])) if len(lane_probs) >= 4 else 1.0
      conf_y = float(np.interp(y_std,    [0.05, 0.30], [1.0, 0.0]))
      conf_l = float(np.interp(lane_min, [0.05, 0.30], [0.0, 1.0]))
      # Phase 6g-1/6g-2: floor the damping so a low-confidence corner-entry
      # transition cannot freeze op near-straight (it ran the car wide to the
      # outside line) — but TAPER the floor to 0 at a true dropout so a spiking
      # low-confidence command is frozen out instead of half-passed (6g-2).
      floor_eff = LAT_CONF_FLOOR * float(np.clip(
        (lane_min - CONF_FLOOR_LANE_LO) / (CONF_FLOOR_LANE_HI - CONF_FLOOR_LANE_LO), 0.0, 1.0))
      confidence = max(min(conf_y, conf_l), floor_eff)
      if confidence < 1.0:
        new_desired_curvature = confidence * new_desired_curvature + (1.0 - confidence) * self.desired_curvature

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
      # Phase 37b: BSM lane guard — radar-occupied side + lane line close on
      # that side + no blinker toward it -> hold curvature (see bsm_guard.py).
      # Called EVERY active frame (no BSM short-circuit) so the hold-timeout
      # counter resets on the first unblocked frame — review: a short-circuit
      # latched an exhausted counter across the BSM-off gap and silently
      # left the next episode unguarded.
      if BSM_LANE_GUARD_M > 0.0:
        try:
          _lls = model_v2.laneLines
          _lp = model_v2.laneLineProbs
          if len(_lls) >= 3 and len(_lp) >= 3 and len(_lls[1].y) > 0 and len(_lls[2].y) > 0:
            new_desired_curvature, _ = self.bsm_guard.update(
              new_desired_curvature, self.desired_curvature,
              bool(CS.leftBlindspot), bool(CS.rightBlindspot),
              bool(CS.leftBlinker), bool(CS.rightBlinker),
              float(_lls[1].y[0]), float(_lls[2].y[0]), float(_lp[1]), float(_lp[2]),
              BSM_LANE_GUARD_M, BSM_LANE_GUARD_MIN_PROB)
          else:
            self.bsm_guard.reset()
        except (IndexError, TypeError, ValueError):
          self.bsm_guard.reset()

    # Temporal command smoothing w/ lead compensation (see constants). Applied AFTER
    # confidence damping / lane-departure and BEFORE clip_curvature, so ISO lateral
    # jerk/accel safety limits still bound the result. tau=0 disables (no-op).
    # Phase 6h-1: tau is speed-dependent (lat_smooth_tau, computed above so the
    # lookahead lead matches); the angle-domain EMA downstream is now light.
    if lat_smooth_tau > 0.0 and CC.latActive:
      alpha = DT_CTRL / (lat_smooth_tau + DT_CTRL)
      self._lat_cmd_lp = alpha * new_desired_curvature + (1.0 - alpha) * self._lat_cmd_lp
      new_desired_curvature = self._lat_cmd_lp
    else:
      self._lat_cmd_lp = new_desired_curvature

    self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, new_desired_curvature, lp.roll)
    lat_delay = self.lat_delay  # R2: assigned once above

    actuators.curvature = self.desired_curvature
    steer, steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                       self.steer_limited_by_safety, self.desired_curvature,
                                                       self.calibrated_pose, curvature_limited, lat_delay)
    actuators.torque = float(steer)
    actuators.steeringAngleDeg = float(steeringAngleDeg)
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
    cs.longControlState = self.LoC.long_control_state
    cs.upAccelCmd = float(self.LoC.pid.p)
    cs.uiAccelCmd = float(self.LoC.pid.i)
    cs.ufAccelCmd = float(self.LoC.pid.f)
    cs.forceDecel = bool((self.sm['driverMonitoringState'].awarenessStatus < 0.) or
                         (self.sm['selfdriveState'].state == State.softDisabling))
    # i6nv2: predictedLatAccelRatio is an i6n ControlsState extension absent from the
    # 04-10 base cereal schema. Publishing it would AttributeError-crash controlsd on
    # engage. The ratio is still computed internally; only the (telemetry-only) publish
    # is dropped. curveSpeedAdvisory (its only consumer) is likewise disabled in selfdrived.

    lat_tuning = self.CP.lateralTuning.which()
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      cs.lateralControlState.angleState = lac_log
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
