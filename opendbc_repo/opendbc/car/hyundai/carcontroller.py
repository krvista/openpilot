import os
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, make_tester_present_msg, structs, rate_limit
from opendbc.car.lateral import apply_driver_steer_torque_limits, apply_std_steer_angle_limits, apply_steer_angle_limits_vm, common_fault_avoidance
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai import hyundaicanfd, hyundaican
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CAR
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.vehicle_model import VehicleModel

from opendbc.sunnypilot.car.hyundai.escc import EsccCarController
from opendbc.sunnypilot.car.hyundai.icbm import IntelligentCruiseButtonManagementInterface
from opendbc.sunnypilot.car.hyundai.longitudinal.controller import LongitudinalController
from opendbc.sunnypilot.car.hyundai.lead_data_ext import LeadDataCarController
from opendbc.sunnypilot.car.hyundai.mads import MadsCarController
from openpilot.common.swaglog import cloudlog

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2

def is_ccnc_angle_platform(flags):
  """True for Hyundai/Kia cars on the HDA2-ALT + CCNC angle-control path.

  The path is entirely flag-gated (no fingerprint check) so new members
  only need `HyundaiFlags.CCNC | HyundaiFlags.CANFD_LKA_STEERING_ALT` in
  values.py to inherit the full behaviour (rate limiter, hysteresis,
  camera-ref blend, low-speed camera passthrough, ACI gain policy,
  op-only alert suppression). Ioniq 6 N 2026 is the first member;
  future 2025+ MY Hyundai/Kia/Genesis HDA2-ALT CCNC trims should
  reuse this helper.
  """
  return bool(flags & HyundaiFlags.CCNC) and bool(flags & HyundaiFlags.CANFD_LKA_STEERING_ALT)


# ACIGain floor when op is actively steering.
ACI_GAIN_OP_FLOOR = 0.15

# Low-speed freeze latch (hysteresis 15/17 km/h).
# Below 15 km/h, freeze steering angle (hold current angle,
# ACTIVE=2 maintained so EPS stays engaged, jitter eliminated).
# Grid search over 180 combos showed freeze=15kph optimal:
# 5-10kph:0°, 10-20kph:0.60°, 20-40kph:0.42° (score=1.72).
LOW_SPEED_PASSTHROUGH_ENTER_MS = 15.0 / 3.6   # ≈ 4.17 m/s
LOW_SPEED_PASSTHROUGH_EXIT_MS  = 17.0 / 3.6   # ≈ 4.72 m/s

LPF_DT = DT_CTRL  # 10 ms (100 Hz)

# Stuck-angle jitter break: inject ±0.05° micro-steps when angle is frozen
# for 400ms to prevent MDPS from entering low-power mode.
JITTER_DEADBAND = 0.03    # deg — below sensor quantization (0.1°)
JITTER_FRAMES = 20        # ~200ms at 100 Hz
JITTER_STEP = 0.05        # deg — imperceptible but keeps EPS alive



def compute_torque_reduction_gain(steering_torque, v_ego, lat_active, last_gain):
  """Sunnypilot-style ACIGain: torque + speed → gain with rate limit + quantization."""
  if lat_active:
    ceiling = float(np.interp(v_ego, [0.5, 1.5], [1.0, 0.85]))
    shelf = float(np.interp(v_ego, [2., 11.], [0.45, 0.6]))
    floor = float(np.interp(v_ego, [2., 22.], [0.1, 0.3]))
    bp1 = float(np.interp(v_ego, [2., 11.], [75., 125.]))
    bp2 = float(np.interp(v_ego, [2., 11.], [125., 150.]))
    bp3 = float(np.interp(v_ego, [2., 11.], [175., 275.]))
    bp4 = float(np.interp(v_ego, [2., 22.], [400., 700.]))
    target = float(np.interp(abs(steering_torque), [bp1, bp2, bp3, bp4],
                              [ceiling, shelf, shelf, floor]))
  else:
    target = 0.0
  gain = rate_limit(target, last_gain,
                    CarControllerParams.ACI_GAIN_RATE_DOWN,
                    CarControllerParams.ACI_GAIN_RATE_UP)
  return round(gain / CarControllerParams.ACI_GAIN_QUANT) * CarControllerParams.ACI_GAIN_QUANT


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
    # HDA2-ALT + CCNC hysteresis state — prevents binary flip of
    # LKAS_ANGLE_ACTIVE / LKA_ASSIST at the authority boundary (the
    # residual low-speed tick source).
    # aci_active_latched: True once authority>=0.3 and stays True until <0.05.
    # passthrough_latched: True when driver really has the wheel, stable across
    # small torque oscillations below the driver_torque_blend threshold.
    # aci_gain_ramp: first-order smoothing 0→1 over ~0.3 s on engagement.
    self.aci_active_latched = False
    self.passthrough_latched = False
    self.aci_gain_ramp = 0.0
    self.aci_gain_last = 0.0
    self.low_speed_cam_latched = False
    # Phase 4-A: vehicle model for VM-based jerk/accel limiting
    if is_ccnc_angle_platform(CP.flags):
      self.VM = VehicleModel(CP)
      from opendbc.car.hyundai.interface import CarInterface
      baseline_cp = CarInterface.get_non_essential_params("KIA_SPORTAGE_5TH_GEN")
      self.BASELINE_VM = VehicleModel(baseline_cp)
    # Variable-tau LPF state
    self.vtau_lpf = 0.0
    self.vtau_sustained_cnt = 0
    self.vtau_prev_sign = 0
    self.vtau_prev_op = 0.0
    # Phase 5: driver-override snap state — tracks whether MADS has
    # yielded to the driver (apply_angle_last follows actual wheel),
    # plus hysteresis counters so re-engage only happens after the
    # driver has fully released for OVERRIDE_SNAP_EXIT_FRAMES.
    self.override_snapped = False
    self.override_enter_cnt = 0
    self.override_exit_cnt  = 0
    # Phase 4-B: stuck-angle jitter break counter
    self.jitter_counter = 0
    self.jitter_sign = 1
    # F1: Camera message staleness tracker. If the camera ECU stops
    # sending LKAS_ALT on bus 2, the dict still arrives (CANParser caches)
    # but the COUNTER signal stops incrementing. After CAM_STALE_FRAMES
    # frames without COUNTER change, force steering_active=False so we
    # never emit an active frame with stale camera bytes.
    # (R4) Previous revision used id() of the dict, but carstate does
    # copy.copy() every frame → id() always changed → staleness never
    # detected. Tracking COUNTER fixes that.
    self.cam_msg_last_frame = 0
    self.cam_msg_last_counter = -1
    # HOD (hands-on detection) bypass. Opt-in via HOD_BYPASS=1 env var.
    # Default OFF: factory ECU also publishes 0x208 on E-CAN → dual-publisher
    # collision caused bus-off (busOffCnt 0→1,456, txErr 239/256).
    self.hod_bypass_enabled = os.environ.get("HOD_BYPASS") == "1"
    self.hod_bypass_counter = 0

    # Owned by openpilot so ADAS DRV sees a clean +1 sequence regardless of
    # camera-TX rate vs our frame%5==0 downsample. Wraps at 256, well above
    # any realistic continuity-watchdog window.
    self.suppress_lfa_counter = 0
    self.prev_fault_lfa = 0
    self.was_in_reverse = False
    self.parking_mode = False
    self.parking_enter_cnt = 0
    self.parking_exit_speed_cnt = 0
    self.parking_fade = 1.0

  def update(self, CC, CC_SP, CS, now_nanos):
    self._cc_sp = CC_SP
    EsccCarController.update(self, CS)
    LeadDataCarController.update(self, CC_SP)
    MadsCarController.update(self, self.CP, CC, CC_SP, self.frame)
    if self.frame % 5 == 0:
      # R5: On the HDA2-ALT + CCNC angle-control platform we never TX
      # SCC_CONTROL (gated out by F3 at line ~576), so the tuning state
      # produced here is only read by new_actuators.accel as a telemetry
      # report. Keeping the 5-frame cadence is intentional — it matches
      # non-CCNC Hyundai cars and avoids branching the call site.
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
    # R2: On the HDA2-ALT + CCNC angle-control platform the factory SCC
    # owns longitudinal; injecting resume/set buttons from openpilot at
    # MADS-engage time creates a race against the driver's own ACC button
    # press on the same MCU (CANFD_ALT_BUTTONS path). I6N already has
    # ALT_BUTTONS so the current ICBM is a no-op, but gating here makes
    # the safety contract explicit and future-proofs any new CCNC trim.
    if not is_ccnc_angle_platform(self.CP.flags):
      can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CS, CC_SP, self.packer, self.frame, self.last_button_frame, self.CAN))

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque
    # Only report commanded angle for cars on the HDA2-ALT + CCNC angle-
    # control platform. For other cars, preserve the angle set by
    # controlsd's lateral controller.
    if is_ccnc_angle_platform(self.CP.flags):
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

    # steering control: HDA2-ALT + CCNC angle-control platform path
    ccnc_lka_alt = is_ccnc_angle_platform(self.CP.flags)
    lkas_alt_cam_msg = getattr(CS, 'lkas_alt_cam_msg', None) if ccnc_lka_alt else None
    # F1 / R4: Camera staleness detection via LKAS_ALT COUNTER field.
    # carstate does copy.copy(cp_cam.vl["LKAS_ALT"]) every frame, so the
    # dict identity cannot indicate freshness — only the camera-driven
    # COUNTER does. If COUNTER doesn't change for CAM_STALE_FRAMES (≥ 25
    # frames ≈ 250 ms at 100 Hz), treat camera as dropped and force
    # steering_active=False in the packer.
    CAM_STALE_FRAMES = 25
    cam_stale = False
    if ccnc_lka_alt and lkas_alt_cam_msg is not None:
      cam_counter = int(lkas_alt_cam_msg.get("COUNTER", -1))
      if cam_counter != self.cam_msg_last_counter:
        self.cam_msg_last_frame = self.frame
        self.cam_msg_last_counter = cam_counter
      if (self.frame - self.cam_msg_last_frame) > CAM_STALE_FRAMES:
        cam_stale = True
    # Smooth low-speed authority ramp — replaces the old binary 3 km/h gate.
    # authority ramps 0→1 linearly as vEgoRaw rises from 1 km/h to 3 km/h.
    # Below 1 km/h: 0 (effectively no ACI command). Above 3 km/h: full.
    ACI_SPEED_FULL_MS = 3.0 / 3.6   # full authority at/above 3 km/h
    ACI_SPEED_ZERO_MS = 1.0 / 3.6   # zero authority at/below 1 km/h
    # F8 / R1: Guard against NaN/inf from wheel sensors and planner.
    # vEgoRaw: MDPS/wheel-speed glitch. steeringAngleDeg / steeringTorque:
    # STEERING_SENSORS CAN frame can go all-1s during a bus error or
    # harness fault. actuators.steeringAngleDeg: LatControlAngle can emit
    # NaN if the planner ever divides by zero. Any single NaN reaching
    # np.clip or float() downstream propagates and crashes the rate
    # limiter / trust estimator — so we clamp at the boundary.
    v_ego_safe = float(np.clip(CS.out.vEgoRaw, 0.0, 100.0)) if np.isfinite(CS.out.vEgoRaw) else 0.0
    steer_angle_safe = float(CS.out.steeringAngleDeg) if np.isfinite(CS.out.steeringAngleDeg) else 0.0
    steer_torque_safe = float(CS.out.steeringTorque) if np.isfinite(CS.out.steeringTorque) else 0.0
    lon_accel = float(CS.out.aEgo) if np.isfinite(CS.out.aEgo) else 0.0
    op_curv_raw = float(CC.actuators.steeringAngleDeg)
    op_curv_safe = op_curv_raw if np.isfinite(op_curv_raw) else steer_angle_safe
    speed_blend = float(np.clip((v_ego_safe - ACI_SPEED_ZERO_MS) /
                                 (ACI_SPEED_FULL_MS - ACI_SPEED_ZERO_MS), 0.0, 1.0))
    # Phase 5: speed-dependent gradient driver override blending.
    # Fixed 150 Nm full-override threshold was unreachable at low speed
    # (driver turning wheel 90° at 30 km/h applies only 60-80 Nm) → MADS
    # kept fighting the driver. Lower the full-override torque at low v,
    # keep the higher value at highway for stability.
    # Route 0x49 discovery: on angle-control MDPS the reported column torque
    # INCLUDES EPS reaction force during angle tracking. Light hand grip
    # reads p50=36 / p90=184 Nm (not pure driver input like torque-control
    # cars), so the torque-control thresholds flag every hand placement as
    # full override. Branch on ccnc_lka_alt to use the wider angle-calibrated
    # thresholds.
    if ccnc_lka_alt:
      DRIVER_TORQUE_DEADZONE = CarControllerParams.DRIVER_TORQUE_DEADZONE_ANGLE
      override_low_v  = CarControllerParams.DRIVER_TORQUE_FULL_OVERRIDE_LOW_V_ANGLE
      override_high_v = CarControllerParams.DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V_ANGLE
    else:
      DRIVER_TORQUE_DEADZONE = CarControllerParams.DRIVER_TORQUE_DEADZONE
      override_low_v  = CarControllerParams.DRIVER_TORQUE_FULL_OVERRIDE_LOW_V
      override_high_v = CarControllerParams.DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V
    full_override_torque = float(np.interp(v_ego_safe,
                                           [CarControllerParams.DRIVER_TORQUE_LOW_V_SPEED,
                                            CarControllerParams.DRIVER_TORQUE_HIGH_V_SPEED],
                                           [override_low_v, override_high_v]))
    driver_abs_torque = abs(steer_torque_safe)
    override_factor = float(np.clip((driver_abs_torque - DRIVER_TORQUE_DEADZONE) /
                                     max(full_override_torque - DRIVER_TORQUE_DEADZONE, 1.0), 0.0, 1.0))
    driver_torque_blend = 1.0 - override_factor  # 1.0 = full ACI, 0.0 = fully yielded

    # Phase 5: snap-to-wheel + grace-window state machine.
    # Enter snap after sustained full override; exit only after sustained
    # full release (OVERRIDE_SNAP_EXIT_FRAMES). While snapped, apply_angle_last
    # is forced to follow the actual wheel angle so MADS cannot build up a
    # restoring torque; on exit the rate limiter naturally ramps from there.
    if override_factor >= CarControllerParams.OVERRIDE_SNAP_ENTER_FACTOR:
      self.override_enter_cnt += 1
      self.override_exit_cnt = 0
    elif override_factor <= CarControllerParams.OVERRIDE_SNAP_EXIT_FACTOR:
      self.override_exit_cnt += 1
      self.override_enter_cnt = 0
    else:
      # in-between: hold counters (don't accumulate, don't reset)
      pass
    if not self.override_snapped and self.override_enter_cnt >= CarControllerParams.OVERRIDE_SNAP_ENTER_FRAMES:
      self.override_snapped = True
    elif self.override_snapped and self.override_exit_cnt >= CarControllerParams.OVERRIDE_SNAP_EXIT_FRAMES:
      self.override_snapped = False
    if not CC.latActive:
      # disengaged: reset so next engage doesn't inherit stale state
      self.override_snapped = False
      self.override_enter_cnt = 0
      self.override_exit_cnt = 0
      self.aci_gain_last = 0.0
    blinker_on = bool(CS.out.leftBlinker or CS.out.rightBlinker)

    # ---- ACI engagement: always-active for angle-control platforms ----
    # Angle-control MDPS: LKAS_ANGLE_ACTIVE=2 whenever latActive=True.
    # The hysteresis (authority enter/exit thresholds) that previously gated
    # the binary ACTIVE flag caused 23.9% of latActive frames to drop to
    # steering_active=False on route 0x49. Instead, ACIGain handles all
    # smooth modulation (speed, driver torque, blinker) while the ACTIVE
    # flag stays stable. MDPS's own internal safety limits handle edge cases.
    if ccnc_lka_alt:
      self.aci_active_latched = bool(CC.latActive)
    else:
      authority = driver_torque_blend * speed_blend if CC.latActive else 0.0
      if blinker_on and driver_torque_blend < 0.7:
        authority *= 0.3
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

    # Low-speed camera passthrough: at creep speed, stock LFA stays
    # fully passive (cam |Δ|≈0, LKAS_ANGLE_ACTIVE=1, ACIGain=0 — MDPS
    # idle). Emulate that by keeping `steering_active=False` in the
    # LKAS_ALT packer (via the speed_blend > 0.1 gate), which mirrors
    # the camera's fields verbatim in the op-emitted frame. Driver has
    # no assist AND no resistance at creep speed — identical to stock
    # LFA feel. Hysteresis on vEgoRaw prevents stop-and-go flapping.
    if CS.out.vEgoRaw < LOW_SPEED_PASSTHROUGH_ENTER_MS:
      self.low_speed_cam_latched = True
    elif CS.out.vEgoRaw > LOW_SPEED_PASSTHROUGH_EXIT_MS:
      self.low_speed_cam_latched = False
    # Combined passthrough latch — used below to force `rate_lat_active=False`
    # in the rate limiter so `apply_angle_last` tracks the actual wheel
    # while passive. The LKAS_ALT packer no longer takes a separate
    # passthrough code path (was a source of frame-format-switch faults
    # on routes 3a/32/34); instead it uses the unified `steering_active`
    # gate which resolves to passive for the same conditions.
    in_passthrough = self.passthrough_latched or self.low_speed_cam_latched

    # First-order ramp of ACI gain on re-engagement (smooths the
    # ADAS_ACIAnglTqRedcGainVal step). ~0.3 s at 100 Hz ≈ 30 frames.
    ACI_GAIN_RAMP_TAU_FRAMES = 30.0
    if self.aci_active_latched:
      self.aci_gain_ramp = min(1.0, self.aci_gain_ramp + 1.0 / ACI_GAIN_RAMP_TAU_FRAMES)
    else:
      self.aci_gain_ramp = 0.0

    # HDA2-ALT + CCNC angle control: op-only, VM-based jerk/accel limiter
    # at 100 Hz. Panda safety checks at 100Hz frequency — running the
    # rate limiter at 50Hz (frame%2) wasted half the allowed jerk budget
    # because panda only permits 10ms of delta per TX.
    if ccnc_lka_alt:
      # Variable-tau LPF: unified filter replacing curvature LPF + low-speed
      # LPF + jerk FF + error FB. Tau is a continuous function of angle
      # magnitude and speed — strong at center (suppresses straight-line
      # jitter), weak at large angles (fast curve tracking), with low-speed
      # smoothing built in. No step response because tau is continuous.
      entry_th = float(np.interp(v_ego_safe, CarControllerParams.VTAU_ENTRY_TH_BP,
                                              CarControllerParams.VTAU_ENTRY_TH_V))
      entering_curve = abs(op_curv_safe) > abs(self.vtau_lpf) + entry_th
      returning_to_center = abs(op_curv_safe) < abs(self.vtau_lpf) - CarControllerParams.VTAU_EXIT_TH

      if entering_curve or returning_to_center:
        # Soft trigger: fast LPF instead of an instantaneous state jump.
        # alpha @ 100Hz = 0.01/0.06 = 0.17 ⇒ ~5 frames to 95% — looks like a
        # smooth slew to EPS rather than a single rate-limited cliff.
        # Drivelog 0x15: cuts EPS frames with Δ>1° from 1.52% to 0.27% (5.6×)
        # at the cost of +9–58ms entry lag depending on stratum.
        vtau = 0.05
        # Skip the sustained-direction warmup so tau stays at 0.1s through the
        # curve instead of slowly ramping in.
        self.vtau_sustained_cnt = 60
      else:
        abs_angle = abs(self.vtau_lpf)
        angle_tau = float(np.interp(abs_angle, CarControllerParams.VTAU_ANGLE_BP,
                                    CarControllerParams.VTAU_ANGLE_V))
        speed_tau = float(np.interp(v_ego_safe, CarControllerParams.VTAU_SPEED_BP,
                                    CarControllerParams.VTAU_SPEED_V))
        vtau = max(angle_tau, speed_tau)

        # Adaptive tau: sustained same-direction change = real correction (not noise).
        # Noise oscillates direction every few frames; real corrections persist.
        cur_sign = 1 if op_curv_safe > self.vtau_lpf + 0.01 else (-1 if op_curv_safe < self.vtau_lpf - 0.01 else 0)
        if cur_sign != 0 and cur_sign == self.vtau_prev_sign:
          self.vtau_sustained_cnt = min(self.vtau_sustained_cnt + 1, 100)
        else:
          self.vtau_sustained_cnt = max(self.vtau_sustained_cnt - 2, 0)
        self.vtau_prev_sign = cur_sign
        vtau = float(np.interp(self.vtau_sustained_cnt, [0, 30, 60], [vtau, 0.5, 0.1]))

      # Rate-feedforward: scale tau down when op_curv_safe is changing fast so
      # the LPF tracks aggressive corner entries closer to the model's command.
      # Drive 0x15 aggressive entries (≥5°) d@90% 39→10ms (-74%, near-PURE);
      # trade-off is +0.2° on Δ p99 in ~1% transient frames (still below PURE's
      # 0.95°). Steady-state op_rate≈0 so this branch is inert during normal
      # driving — only activates on rapid command changes.
      op_rate = (op_curv_safe - self.vtau_prev_op) / LPF_DT
      if abs(op_rate) > 1.0:
        vtau = vtau / (1.0 + 0.05 * abs(op_rate))
        vtau = max(vtau, 0.01)
      self.vtau_prev_op = op_curv_safe

      # During passthrough or while op is inactive, the rate limiter forces
      # apply_angle_last == steer_angle_safe (lateral.py:129). Mirror that on
      # vtau_lpf so re-engagement starts from the actual wheel angle and the
      # next LPF convergence is short. Without this gate vtau_lpf accumulated
      # up to 9.8° away from the wheel during passthrough on drive 0x15,
      # producing an 80ms rate-saturated slew at the 17 km/h release boundary.
      if CC.latActive and not in_passthrough and vtau > 0.001:
        alpha = LPF_DT / (vtau + LPF_DT)
        self.vtau_lpf = alpha * op_curv_safe + (1.0 - alpha) * self.vtau_lpf
      else:
        self.vtau_lpf = steer_angle_safe
      desired_angle_deg = self.vtau_lpf

      # Driver override blend
      if override_factor > 0:
        desired_angle_deg = (1.0 - override_factor) * desired_angle_deg + \
                            override_factor * steer_angle_safe

      # Passthrough below 15 km/h or while the driver firmly has the wheel:
      # let `apply_steer_angle_limits_vm` track steer_angle_safe (line ~129
      # of lateral.py) and let the stuck-angle jitter break (further below)
      # short-circuit on its `else` branch. Without this gate the latch
      # comment on line ~462 — "force rate_lat_active=False" — wasn't
      # actually wired, leaving the ±0.05° jitter injection running at
      # standstill.
      rate_lat_active = bool(CC.latActive) and self.aci_active_latched and not in_passthrough

      if self.override_snapped:
        self.apply_angle_last = steer_angle_safe

      # VM-based jerk/accel limiter
      apply_angle = apply_steer_angle_limits_vm(
        desired_angle_deg, self.apply_angle_last, v_ego_safe,
        steer_angle_safe, rate_lat_active, self.params, self.VM,
      )
      if apply_angle is not None:
        apply_angle = apply_steer_angle_limits_vm(
          apply_angle, self.apply_angle_last, v_ego_safe,
          steer_angle_safe, rate_lat_active, self.params, self.BASELINE_VM,
        )
      if apply_angle is None:
        self.apply_angle_last = steer_angle_safe
        apply_steer_req = False
      else:
        self.apply_angle_last = apply_angle

      # Stuck-angle jitter break
      if rate_lat_active:
        if abs(self.apply_angle_last - self.vtau_lpf) < JITTER_DEADBAND:
          self.jitter_counter += 1
        else:
          self.jitter_counter = 0
        if self.jitter_counter >= JITTER_FRAMES:
          self.apply_angle_last += self.jitter_sign * JITTER_STEP
          self.jitter_sign *= -1
          self.jitter_counter = 0
      else:
        self.jitter_counter = 0

    # Steering message TX + angle computation: 100 Hz for CCNC angle platform.
    # Both vtau LPF and VM rate limiter run at 100Hz (STEER_STEP=1) to match
    # panda safety frequency and fully utilize the jerk budget.
    # MADS-driven LKA_ICON for LKAS_ALT message:
    # When cruiseState.available (ACC main on): reflect MADS state (green/off).
    # When ACC main off: None → camera passthrough (stock LFA icon).
    mads_enabled = getattr(self._cc_sp, 'mads', None) and self._cc_sp.mads.enabled
    if ccnc_lka_alt and CS.out.cruiseState.available:
      mads_lka_icon = 2 if mads_enabled else 0
    else:
      mads_lka_icon = None

    # Parking mode disabled — D+A strategy handles all speeds.
    if ccnc_lka_alt:
      self.parking_mode = False
      self.parking_fade = 1.0

    if ccnc_lka_alt:
      fault_lfa = getattr(CS, 'fault_lfa', 0)
      if fault_lfa and not self.prev_fault_lfa:
        cloudlog.warning(
          f"FAULT_LFA onset: cam_stale={cam_stale} aci_active={self.aci_active_latched} "
          f"gain={self.aci_gain_last:.3f} speed_blend={speed_blend:.2f} "
          f"steer_angle={steer_angle_safe:.1f} op_angle={op_curv_safe:.1f} "
          f"lat_active={CC.latActive} override={self.override_snapped} "
          f"cam_counter={self.cam_msg_last_counter} fault_das={getattr(CS, 'fault_das', 0)}"
        )
      elif not fault_lfa and self.prev_fault_lfa:
        cloudlog.warning("FAULT_LFA cleared")
      self.prev_fault_lfa = fault_lfa

    if ccnc_lka_alt:
      if CS.out.gearShifter == structs.CarState.GearShifter.reverse:
        self.was_in_reverse = True
      if self.was_in_reverse and CS.out.vEgo > 10 * CV.KPH_TO_MS:
        self.was_in_reverse = False

    parking_fully_faded = self.parking_mode and self.parking_fade < 0.01
    if parking_fully_faded:
      self.apply_angle_last = steer_angle_safe
      if lkas_alt_cam_msg is not None:
        lkas_alt_cam_msg = dict(lkas_alt_cam_msg)
        lkas_alt_cam_msg["LKA_ASSIST"] = 0
        lkas_alt_cam_msg["LKAS_ANGLE_ACTIVE"] = 1
        lkas_alt_cam_msg["ADAS_StrAnglReqVal"] = steer_angle_safe
    effective_lat_active = (CC.latActive and not self.override_snapped and apply_steer_req
                            and not self.was_in_reverse and not parking_fully_faded) if ccnc_lka_alt else apply_steer_req

    # ACIGain: sunnypilot-style torque reduction gain (torque + speed → gain).
    # Must use effective_lat_active (not CC.latActive) to ensure gain=0
    # when LKAS_ANGLE_ACTIVE=1 (passive). Panda rejects gain>0 with ACTIVE=1.
    effective_aci_gain = None
    if ccnc_lka_alt and lkas_alt_cam_msg is not None:
      effective_aci_gain = compute_torque_reduction_gain(
        steer_torque_safe, v_ego_safe, effective_lat_active, self.aci_gain_last)
      self.aci_gain_last = effective_aci_gain
    can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, CC.enabled, effective_lat_active, apply_torque, self.lkas_icon,
                                                         apply_angle=self.apply_angle_last, lkas_alt_cam_msg=lkas_alt_cam_msg,
                                                         driver_torque_blend=driver_torque_blend,
                                                         blinker_on=blinker_on,
                                                         speed_blend=speed_blend,
                                                         aci_active=self.aci_active_latched,
                                                         aci_gain_ramp=self.aci_gain_ramp,
                                                         in_passthrough=in_passthrough,
                                                         mads_lka_icon=mads_lka_icon,
                                                         lon_accel=lon_accel,
                                                         effective_aci_gain=effective_aci_gain))

    # prevent LFA from activating on LKA steering cars by sending "no lane lines detected" to ADAS ECU
    # CCNC cars (including the HDA2-ALT + CCNC angle-control platform):
    # pass through camera's lane lines so ADAS DRV accepts LKAS_ALT
    # F5: Skip suppress_lfa on early boot frames before CAM parser has
    # received CAM_0x362 (lfa_block_msg would have uninitialized keys,
    # causing a stale or zero COUNTER that panda/ADAS might reject).
    if self.frame % 5 == 0 and lka_steering and getattr(CS, 'lfa_block_msg', None):
      suppress_lanes = not bool(self.CP.flags & HyundaiFlags.CCNC)
      force_lanes = not suppress_lanes and bool(CC.latActive)
      can_sends.append(hyundaicanfd.create_suppress_lfa(self.packer, self.CAN, CS.lfa_block_msg,
                                                        self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT,
                                                        suppress_lanes=suppress_lanes,
                                                        override_counter=self.suppress_lfa_counter,
                                                        force_lanes=force_lanes))
      self.suppress_lfa_counter = (self.suppress_lfa_counter + 1) & 0xFF

    # LFA and HDA icons
    # Non-HDA2 CCNC cars get our create_ccnc() frame so we can render
    # HDP / LFA icons consistently with op state.
    #
    # HDA2-ALT + CCNC: alert-suppression feature DISABLED (2026-04-15).
    # On this platform CCNC_0x161/0x162 are natively published by a
    # gateway ECU on bus 1 (not forwarded from the camera — see
    # c6a33de). Any TX from openpilot on those addresses creates a
    # dual-publisher situation on bus 1; the other ADAS components
    # detect the duplication as a fault, flicker the cluster ADAS
    # icon, and eventually latch red with accumulated error counts.
    # Until we have a way to silence the source ECU (UDS session?),
    # we leave 0x161/0x162 entirely to the stock publisher and accept
    # the hands-on / HDP audible alerts as the cost of stability.
    # The rest of the HDA2-ALT + CCNC feature set (Stage 4 camera-ref
    # blend, low-speed passthrough, ACI floor, etc.) is unaffected.
    if self.frame % 5 == 0 and (not lka_steering or lka_steering_long):
      if ccnc_non_hda2:
        op_driving = bool(ccnc_lka_alt and self.aci_active_latched)
        can_sends.extend(hyundaicanfd.create_ccnc(self.packer, self.CAN, self.CP.openpilotLongitudinalControl, CC.enabled, CC.hudControl, CC.leftBlinker,
                                                  CC.rightBlinker, CS.msg_161, CS.msg_162, CS.msg_1b5, CS.is_metric, CS.out, CS.main_cruise_enabled,
                                                  self.lfa_icon, op_driving=op_driving))
      else:
        can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled, self.lfa_icon))

    # HOD (hands-on detection) bypass on HDA2-ALT + CCNC. TX 0x208 on
    # E-CAN at 10 Hz with GRIP_STRONG to keep the hands-off timer reset.
    # Active whenever MADS is enabled (not just latActive) — prevents
    # hands-on warning during transient latActive=False (driver override,
    # low-speed passthrough). Disabled only when MADS is fully OFF.
    # Gated on hod_bypass_enabled (HOD_BYPASS=1 env var) — default OFF.
    # Factory ECU already publishes 0x208 on bus 1; unconditional TX
    # creates a dual-publisher collision causing CAN bus 1 bus-off
    # (busOffCnt 0→1,456 over routes 0x28→0x4f, peak txErr=239/256).
    if ccnc_lka_alt and self.frame % 10 == 0 and mads_enabled and self.hod_bypass_enabled:
      can_sends.append(hyundaicanfd.create_hod_bypass(self.CAN.ECAN, self.hod_bypass_counter))
      self.hod_bypass_counter = (self.hod_bypass_counter + 2) & 0xFF

    # blinkers
    if lka_steering and self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
      can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, CC.leftBlinker, CC.rightBlinker))

    # F3: HDA2-ALT + CCNC platform NEVER does openpilot longitudinal —
    # factory SCC handles it. Guard against misconfig that would TX
    # SCC_CONTROL (0x1A0) on E-CAN, colliding with the factory SCC ECU.
    if self.CP.openpilotLongitudinalControl and not ccnc_lka_alt:
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
            # F3 (cont.): HDA2-ALT + CCNC must NOT TX SCC_CONTROL (0x1A0)
            # for ACC cancel — factory SCC natively publishes on bus 1, and
            # our TX creates a dual-publisher race (ACCEnable=3 within 20ms).
            # Driver cancels via the physical steering-wheel button instead.
            if not ccnc_lka_alt:
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
