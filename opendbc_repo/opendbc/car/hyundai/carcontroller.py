import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, make_tester_present_msg, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits, apply_std_steer_angle_limits, common_fault_avoidance
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai import hyundaicanfd, hyundaican
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CAR
from opendbc.car.interfaces import CarControllerBase

from opendbc.sunnypilot.car.hyundai.escc import EsccCarController
from opendbc.sunnypilot.car.hyundai.icbm import IntelligentCruiseButtonManagementInterface
from opendbc.sunnypilot.car.hyundai.longitudinal.controller import LongitudinalController
from opendbc.sunnypilot.car.hyundai.lead_data_ext import LeadDataCarController
from opendbc.sunnypilot.car.hyundai.mads import MadsCarController

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2

# ── Stage 4: camera-referenced feedforward defaults ──
# α₀(v) selected by tools/ioniq6n_camref_sim.py (F+nav_gate0.3 winner) on
# 1.24M frames: 52-65% MAE reduction vs curvature-only across all buckets.
# F-all-high table: aggressive camera trust at low/mid speed (where camera
# advisory is most accurate at 0.20-0.31° MAE), tapered at highway where
# navigation intent matters more for lane changes.
CAMREF_ALPHA_BP = [0., 5., 10., 20., 30.]
CAMREF_ALPHA_V  = [0.80, 0.80, 0.80, 0.70, 0.60]

# nav-disagreement gate: clamp α when camera and planner visibly disagree
# (|cam_angle - op_curv_angle| > NAV_DISAGREE_DEG), so navigation maneuvers
# (lane change, obstacle avoidance) aren't overridden by the camera.
CAMREF_NAV_DISAGREE_DEG = 3.0
CAMREF_NAV_ALPHA_CAP    = 0.30

# Online camera-trust estimator parameters
CAMREF_TRUST_WINDOW_S = 30.0    # rolling RMSE window
CAMREF_TRUST_RMSE_REF = 1.5     # deg — above this, q collapses toward Q_MIN
CAMREF_TRUST_Q_MIN    = 0.20
CAMREF_TRUST_Q_MAX    = 1.00

# Stage 2b: ACIGain floor when op is actively steering. The camera itself
# commands ADAS_ACIAnglTqRedcGainVal = 0.000 at all times (DBC-decoded on
# 1.24M frames), so "camera-mirrored" in the old code always collapsed to
# `authority * 0.6` = op-forced 0.6-1.0 gain. That kept MDPS in a more
# authoritative (less assistive) state than stock LFA ever does and is a
# plausible contributor to the low-speed tick. Drop to a small floor so
# MDPS still recognises us as the source of truth but can do its natural
# smoothing on our angle command, matching stock LFA behavior.
ACI_GAIN_OP_FLOOR = 0.15


class CameraTrustEstimator:
  """Rolling RMSE of (cam_angle - actual) during lfa_passthrough periods.

  Produces a trust multiplier q ∈ [Q_MIN, Q_MAX]. When the camera is noisy
  (construction zone, lane-marking occlusion, heavy rain), rolling RMSE
  grows → q shrinks → the camera-reference blend falls back toward op's
  own curvature-derived plan. Slow-changing by design (30s window) so no
  contribution to high-frequency oscillation.
  """
  def __init__(self, window_s=CAMREF_TRUST_WINDOW_S, dt=0.02,
               rmse_ref=CAMREF_TRUST_RMSE_REF,
               q_min=CAMREF_TRUST_Q_MIN, q_max=CAMREF_TRUST_Q_MAX):
    import collections as _c
    self._buf = _c.deque(maxlen=int(window_s / dt))
    self._rmse_ref = rmse_ref
    self._q_min = q_min
    self._q_max = q_max
    self._last_q = q_max

  def update(self, cam_angle, actual_angle, cam_driving):
    if cam_driving:
      self._buf.append(cam_angle - actual_angle)
    if len(self._buf) >= 100:
      s = 0.0
      for e in self._buf:
        s += e * e
      rmse = (s / len(self._buf)) ** 0.5
      q = self._q_max - (rmse / self._rmse_ref) * (self._q_max - self._q_min)
      self._last_q = max(self._q_min, min(self._q_max, q))
    return self._last_q


def process_hud_alert(enabled, fingerprint, hud_control):
  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  # TODO: this is not accurate for all cars
  sys_state = 1
  if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if hud_control.leftLaneDepart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if hud_control.rightLaneDepart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController(CarControllerBase, EsccCarController, LeadDataCarController, LongitudinalController, MadsCarController,
                    IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    EsccCarController.__init__(self, CP, CP_SP)
    MadsCarController.__init__(self)
    LeadDataCarController.__init__(self, CP)
    LongitudinalController.__init__(self, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.CAN = CanBus(CP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.angle_limit_counter = 0

    self.accel_last = 0
    self.apply_torque_last = 0
    self.apply_angle_last = 0.0
    self.car_fingerprint = CP.carFingerprint
    self.last_button_frame = 0
    # CCNC LKA_ALT hysteresis state — prevents binary flip of LKAS_ANGLE_ACTIVE /
    # LKA_ASSIST at the authority boundary (the residual low-speed tick source).
    # aci_active_latched: True once authority>=0.3 and stays True until <0.05.
    # passthrough_latched: True when driver really has the wheel, stable across
    # small torque oscillations below the driver_torque_blend threshold.
    # aci_gain_ramp: first-order smoothing 0→1 over ~0.3 s on engagement.
    self.aci_active_latched = False
    self.passthrough_latched = False
    self.aci_gain_ramp = 0.0
    # Stage 4: online camera-trust estimator (30s rolling RMSE on passthrough).
    self.camera_trust = CameraTrustEstimator()
    # Logging aids for Stage 4 evaluation (readable via cereal reuse if wired later)
    self.camref_alpha_last = 0.0
    self.camref_q_last = CAMREF_TRUST_Q_MAX

  def update(self, CC, CC_SP, CS, now_nanos):
    EsccCarController.update(self, CS)
    LeadDataCarController.update(self, CC_SP)
    MadsCarController.update(self, self.CP, CC, CC_SP, self.frame)
    if self.frame % 5 == 0:
      LongitudinalController.update(self, CC, CS)

    actuators = CC.actuators
    hud_control = CC.hudControl

    # steering torque
    new_torque = int(round(actuators.torque * self.params.STEER_MAX))
    apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.params)

    # >90 degree steering fault prevention
    self.angle_limit_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringAngleDeg) >= MAX_ANGLE, CC.latActive,
                                                                       self.angle_limit_counter, MAX_ANGLE_FRAMES,
                                                                       MAX_ANGLE_CONSECUTIVE_FRAMES)

    if not CC.latActive:
      apply_torque = 0

    # Hold torque with induced temporary fault when cutting the actuation bit
    # FIXME: we don't use this with CAN FD?
    torque_fault = CC.latActive and not apply_steer_req

    self.apply_torque_last = apply_torque

    # accel + longitudinal
    accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    stopping = actuators.longControlState == LongCtrlState.stopping
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    can_sends = []

    # *** common hyundai stuff ***

    # tester present - w/ no response (keeps relevant ECU disabled)
    if self.frame % 100 == 0 and not ((self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC) or self.ESCC.enabled) and \
            self.CP.openpilotLongitudinalControl:
      # for longitudinal control, either radar or ADAS driving ECU
      addr, bus = 0x7d0, self.CAN.ECAN if self.CP.flags & HyundaiFlags.CANFD else 0
      if self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING.value:
        addr, bus = 0x730, self.CAN.ECAN
      can_sends.append(make_tester_present_msg(addr, bus, suppress_response=True))

      # for blinkers
      if self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.append(make_tester_present_msg(0x7b1, self.CAN.ECAN, suppress_response=True))

    # *** CAN/CAN FD specific ***
    if self.CP.flags & HyundaiFlags.CANFD:
      can_sends.extend(self.create_canfd_msgs(apply_steer_req, apply_torque, set_speed_in_units, accel,
                                              stopping, hud_control, CS, CC))
    else:
      can_sends.extend(self.create_can_msgs(apply_steer_req, apply_torque, torque_fault, set_speed_in_units, accel,
                                            stopping, hud_control, actuators, CS, CC))

    # Intelligent Cruise Button Management
    can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CS, CC_SP, self.packer, self.frame, self.last_button_frame, self.CAN))

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque
    # Only report commanded angle for cars using angle-based control (Ioniq 6 N).
    # For other cars, preserve the angle set by controlsd's lateral controller.
    if self.CP.flags & HyundaiFlags.CCNC and self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT:
      new_actuators.steeringAngleDeg = self.apply_angle_last
    new_actuators.accel = self.tuning.actual_accel

    self.frame += 1
    return new_actuators, can_sends

  def create_can_msgs(self, apply_steer_req, apply_torque, torque_fault, set_speed_in_units, accel, stopping, hud_control, actuators, CS, CC):
    can_sends = []

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                      hud_control)

    can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.CP, apply_torque, apply_steer_req,
                                              torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
                                              hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                              left_lane_warning, right_lane_warning,
                                              self.lkas_icon))

    # Button messages
    if not self.CP.openpilotLongitudinalControl:
      if CC.cruiseControl.cancel:
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL, self.CP))
      elif CC.cruiseControl.resume:
        # send resume at a max freq of 10Hz
        if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
          # send 25 messages at a time to increases the likelihood of resume being accepted
          can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP)] * 25)
          if (self.frame - self.last_button_frame) * DT_CTRL >= 0.15:
            self.last_button_frame = self.frame

    if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:
      # TODO: unclear if this is needed
      jerk = 3.0 if actuators.longControlState == LongCtrlState.pid else 1.0
      use_fca = self.CP.flags & HyundaiFlags.USE_FCA.value
      can_sends.extend(hyundaican.create_acc_commands(self.packer, CC.enabled, accel, jerk, int(self.frame / 2),
                                                      self.lead_data, hud_control, set_speed_in_units, stopping,
                                                      CC.cruiseControl.override, use_fca, self.CP,
                                                      CS.main_cruise_enabled, self.tuning, self.ESCC))

    # 20 Hz LFA MFA message
    if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
      can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC.enabled, self.lfa_icon))

    # 5 Hz ACC options
    if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl:
      can_sends.extend(hyundaican.create_acc_opt(self.packer, self.CP, self.ESCC))

    # 2 Hz front radar options
    if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl and not self.ESCC.enabled:
      can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

    return can_sends

  def create_canfd_msgs(self, apply_steer_req, apply_torque, set_speed_in_units, accel, stopping, hud_control, CS, CC):
    can_sends = []

    lka_steering = self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING
    lka_steering_long = lka_steering and self.CP.openpilotLongitudinalControl
    ccnc_non_hda2 = self.CP.flags & HyundaiFlags.CCNC and not lka_steering

    # steering control
    ccnc_lka_alt = bool(self.CP.flags & HyundaiFlags.CCNC) and bool(self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT)
    lkas_alt_cam_msg = getattr(CS, 'lkas_alt_cam_msg', None) if ccnc_lka_alt else None
    # Smooth low-speed authority ramp — replaces the old binary 3 km/h gate.
    # authority ramps 0→1 linearly as vEgoRaw rises from 1 km/h to 3 km/h.
    # Below 1 km/h: 0 (effectively no ACI command). Above 3 km/h: full.
    ACI_SPEED_FULL_MS = 3.0 / 3.6   # full authority at/above 3 km/h
    ACI_SPEED_ZERO_MS = 1.0 / 3.6   # zero authority at/below 1 km/h
    speed_blend = float(np.clip((CS.out.vEgoRaw - ACI_SPEED_ZERO_MS) /
                                 (ACI_SPEED_FULL_MS - ACI_SPEED_ZERO_MS), 0.0, 1.0))
    # Toyota LTA-style gradient driver override blending.
    DRIVER_TORQUE_DEADZONE = 30   # below this: no reduction (normal driving)
    DRIVER_TORQUE_FULL_OVERRIDE = 150  # at/above this: fully surrendered
    driver_abs_torque = abs(CS.out.steeringTorque)
    override_factor = float(np.clip((driver_abs_torque - DRIVER_TORQUE_DEADZONE) /
                                     (DRIVER_TORQUE_FULL_OVERRIDE - DRIVER_TORQUE_DEADZONE), 0.0, 1.0))
    driver_torque_blend = 1.0 - override_factor  # 1.0 = full ACI, 0.0 = fully yielded
    blinker_on = bool(CS.out.leftBlinker or CS.out.rightBlinker)

    # ---- Hysteresis on ACI engagement (fixes residual low-speed tick) ----
    # The old single-threshold `aci_active = authority > 0.05` flipped
    # LKAS_ANGLE_ACTIVE (1↔2) and LKA_ASSIST (0↔1) in a single 20 ms frame at
    # the boundary. ADAS ECU observes the flip as a mode change and briefly
    # re-arms the EPS actuator → the driver felt a tick when the wheel returned
    # to center at creep. Dual thresholds hold state through small wiggles.
    authority = driver_torque_blend * speed_blend if CC.latActive else 0.0
    if blinker_on:
      authority *= 0.2
    ACI_ENTER = 0.30
    ACI_EXIT  = 0.05
    if CC.latActive:
      if authority >= ACI_ENTER:
        self.aci_active_latched = True
      elif authority < ACI_EXIT:
        self.aci_active_latched = False
    else:
      self.aci_active_latched = False

    # Camera passthrough latch: engage passthrough when clearly not driving
    # (not lat_active AND driver has the wheel). Dual threshold on
    # driver_torque_blend so small wiggles don't flap passthrough on/off.
    if not CC.latActive and driver_torque_blend > 0.9:
      self.passthrough_latched = True
    elif CC.latActive or driver_torque_blend < 0.6:
      self.passthrough_latched = False

    # First-order ramp of ACI gain on re-engagement (smooths the
    # ADAS_ACIAnglTqRedcGainVal step). ~0.3 s at 100 Hz ≈ 30 frames.
    ACI_GAIN_RAMP_TAU_FRAMES = 30.0
    if self.aci_active_latched:
      self.aci_gain_ramp = min(1.0, self.aci_gain_ramp + 1.0 / ACI_GAIN_RAMP_TAU_FRAMES)
    else:
      self.aci_gain_ramp = 0.0

    # Ioniq 6 N angle control: standard speed-dependent rate limiter at 50 Hz,
    # matching Toyota LTA / Tesla / Nissan / PSA / Subaru / Rivian architecture.
    # No output filter — rate limit itself provides natural smoothing.
    if ccnc_lka_alt and self.frame % 2 == 0:
      angle_limits = CarControllerParams.ANGLE_LIMITS

      # Feedforward from LatControlAngle (includes live steerRatio,
      # angleOffsetDeg, roll compensation — r=0.985 without additional PID).
      op_curv_angle_deg = float(CC.actuators.steeringAngleDeg)

      # ── Stage 4: camera-referenced feedforward blend ──
      # Stage 0 re-analysis showed the camera's ADAS_StrAnglReqVal has MAE
      # 0.20-0.31° vs actual (3-10× better than op's curvature-derived angle
      # at every bucket). Blend the two with trust-adaptive α:
      #   α_eff = α₀(v) · q_trust   (and clamp to NAV_ALPHA_CAP on disagreement)
      # The trust q_trust is updated only during lfa_passthrough when the
      # camera drives the wheel directly — rolling 30s RMSE → multiplier,
      # immune to fast oscillation (by design, no classical PID loop).
      cam_angle_deg = None
      if lkas_alt_cam_msg is not None:
        cam_angle_deg = float(lkas_alt_cam_msg.get("ADAS_StrAnglReqVal", 0.0))
        # Feed the trust estimator: camera is "driving" the wheel whenever
        # openpilot is not (so actual ≈ response to cam_angle), with a light
        # driver torque gate so override frames don't poison the RMSE.
        cam_driving = (not CC.latActive) and (driver_torque_blend > 0.5)
        self.camref_q_last = self.camera_trust.update(
          cam_angle_deg, float(CS.out.steeringAngleDeg), cam_driving)

      desired_angle_deg = op_curv_angle_deg
      if cam_angle_deg is not None and CC.latActive and self.aci_active_latched:
        alpha_base = float(np.interp(CS.out.vEgoRaw, CAMREF_ALPHA_BP, CAMREF_ALPHA_V))
        alpha_eff = alpha_base * self.camref_q_last
        # Nav-gate: when planner disagrees with camera by > NAV_DISAGREE_DEG,
        # clamp α so lane-change / obstacle-avoidance maneuvers aren't
        # overridden by camera's lane-centering bias.
        if abs(cam_angle_deg - op_curv_angle_deg) > CAMREF_NAV_DISAGREE_DEG:
          alpha_eff = min(alpha_eff, CAMREF_NAV_ALPHA_CAP)
        desired_angle_deg = alpha_eff * cam_angle_deg + (1.0 - alpha_eff) * op_curv_angle_deg
        self.camref_alpha_last = alpha_eff
      else:
        self.camref_alpha_last = 0.0

      # Gradient driver override blend: smoothly yield toward actual wheel
      # position as driver torque increases (Toyota LTA TORQUE_WIND_DOWN style).
      if override_factor > 0:
        desired_angle_deg = (1.0 - override_factor) * desired_angle_deg + \
                            override_factor * float(CS.out.steeringAngleDeg)

      # When ACI is NOT latched (passthrough / driver taking over), force
      # `lat_active=False` into the rate limiter so it returns the actual
      # wheel angle. This keeps self.apply_angle_last tracking the physical
      # wheel during manual / passthrough periods — so when ACI re-engages,
      # apply_angle_last matches reality and the first active command has
      # NO step. Previously, a stale apply_angle_last could be several
      # degrees off from actual after driver input, causing a tick as the
      # rate limiter ramped it back.
      rate_lat_active = bool(CC.latActive) and self.aci_active_latched

      self.apply_angle_last = apply_std_steer_angle_limits(
        desired_angle_deg, self.apply_angle_last, CS.out.vEgoRaw,
        float(CS.out.steeringAngleDeg), rate_lat_active, angle_limits,
      )

    # Steering message TX: 50 Hz for CCNC LKA_ALT (matching Toyota/Tesla/Nissan),
    # 100 Hz for other Hyundai CAN FD cars (torque-based, unchanged).
    if not ccnc_lka_alt or self.frame % 2 == 0:
      can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, CC.enabled, apply_steer_req, apply_torque, self.lkas_icon,
                                                             apply_angle=self.apply_angle_last, lkas_alt_cam_msg=lkas_alt_cam_msg,
                                                             driver_torque_blend=driver_torque_blend,
                                                             blinker_on=blinker_on,
                                                             speed_blend=speed_blend,
                                                             aci_active=self.aci_active_latched,
                                                             aci_gain_ramp=self.aci_gain_ramp,
                                                             in_passthrough=self.passthrough_latched))

    # prevent LFA from activating on LKA steering cars by sending "no lane lines detected" to ADAS ECU
    # CCNC cars (Ioniq 5 N, Ioniq 6 N): pass through camera's lane lines so ADAS DRV accepts LKAS_ALT
    if self.frame % 5 == 0 and lka_steering:
      suppress_lanes = not bool(self.CP.flags & HyundaiFlags.CCNC)
      can_sends.append(hyundaicanfd.create_suppress_lfa(self.packer, self.CAN, CS.lfa_block_msg,
                                                        self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT,
                                                        suppress_lanes=suppress_lanes))

    # LFA and HDA icons
    # HDA2-ALT + CCNC (Ioniq 6 N) also gets create_ccnc() so we can suppress
    # spurious takeover alerts (ALERTS_3=11 HDP_DEACTIVATED_AUDIBLE, etc.)
    # while openpilot is steering. Requires carstate capture of msg_161 and
    # panda firmware with 0x161/0x162 on the HDA2-ALT+CCNC TX whitelist.
    ccnc_hda2_alt = ccnc_lka_alt and getattr(CS, 'msg_161', None)
    if self.frame % 5 == 0 and (not lka_steering or lka_steering_long or ccnc_hda2_alt):
      if ccnc_non_hda2 or ccnc_hda2_alt:
        op_driving = bool(ccnc_lka_alt and self.aci_active_latched)
        can_sends.extend(hyundaicanfd.create_ccnc(self.packer, self.CAN, self.CP.openpilotLongitudinalControl, CC.enabled, CC.hudControl, CC.leftBlinker,
                                                  CC.rightBlinker, CS.msg_161, CS.msg_162, CS.msg_1b5, CS.is_metric, CS.out, CS.main_cruise_enabled,
                                                  self.lfa_icon, op_driving=op_driving))
      else:
        can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled, self.lfa_icon))

    # blinkers
    if lka_steering and self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
      can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, CC.leftBlinker, CC.rightBlinker))

    if self.CP.openpilotLongitudinalControl:
      if lka_steering:
        can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.CAN, self.frame))
      elif not ccnc_non_hda2:
        can_sends.extend(hyundaicanfd.create_fca_warning_light(self.packer, self.CAN, self.frame))
      if self.frame % 2 == 0:
        can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CAN, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                         set_speed_in_units, hud_control, self.lead_data, CS.main_cruise_enabled, self.tuning,
                                                         CS.cruise_info if ccnc_non_hda2 else None))
        self.accel_last = accel
    else:
      # button presses
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.25:
        # cruise cancel
        if CC.cruiseControl.cancel:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            can_sends.append(hyundaicanfd.create_acc_cancel(self.packer, self.CP, self.CAN, CS.cruise_info))
            self.last_button_frame = self.frame
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter + 1, Buttons.CANCEL))
            self.last_button_frame = self.frame

        # cruise standstill resume
        elif CC.cruiseControl.resume:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            # TODO: resume for alt button cars
            pass
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter + 1, Buttons.RES_ACCEL))
            self.last_button_frame = self.frame

    return can_sends
