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

# Low-speed freeze latch (hysteresis 20/22 km/h).
# Below 20 km/h, hand the wheel back to the driver / EPS so caster torque
# can self-center the wheel - needed for parking-lot maneuvers, where the
# user routinely reaches ~20 km/h. Grid search (commit d6236ed) showed
# freeze=20kph score=0.55 (vs 15kph=1.7); the original "no lateral assist
# 0-20kph" downside is actually the desired behaviour here. The
# traffic_following override below keeps op engaged when a close lead
# (<3m) is present, so stop-and-go traffic still gets lateral support.
LOW_SPEED_PASSTHROUGH_ENTER_MS = 20.0 / 3.6   # ~ 5.56 m/s
LOW_SPEED_PASSTHROUGH_EXIT_MS  = 22.0 / 3.6   # ~ 6.11 m/s

# Traffic-following lead distance hysteresis. When a lead is closer than
# this, op stays engaged below the freeze threshold so the wheel is held
# (stop-and-go traffic). Far hysteresis avoids chatter when the lead
# distance hovers around the boundary.
TRAFFIC_FOLLOW_NEAR_M = 3.0
TRAFFIC_FOLLOW_FAR_M  = 5.0

LPF_DT = DT_CTRL  # 10 ms (100 Hz)

# Stuck-angle jitter break: inject +-0.05 deg micro-steps when angle is frozen
# for 400ms to prevent MDPS from entering low-power mode.
JITTER_DEADBAND = 0.03    # deg - below sensor quantization (0.1 deg)
JITTER_FRAMES = 20        # ~200ms at 100 Hz
JITTER_STEP = 0.05        # deg - imperceptible but keeps EPS alive



def compute_torque_reduction_gain(steering_torque, v_ego, lat_active, last_gain,
                                  steering_error=0.0, blinker_on=False,
                                  override_factor=0.0):
  """Sunnypilot-style ACIGain: torque + speed -> gain with rate limit + quantization.

  Adds three adaptive mechanisms on top of the base 4-breakpoint torque/speed map:

  1. Error-based ceiling boost: when |apply_angle - measured_angle| exceeds the
     speed-dependent threshold, the ceiling is multiplied up to x2 (capped at
     1.0). Boost gives MDPS more authority to catch up when op's command and
     the actual wheel diverge - typically corner-entry transients.
  2. Blinker-aware authority map (4-level descent): under a turn signal the
     driver's intent overrides the standard authority map. An explicit
     4-point curve maps driver torque to op authority:
       hands-off / pure signalling:  0.70  (tq ~ 0)
       light grip / hand on wheel:   0.50  (tq ~ 30)
       active steering / turning:    0.40  (tq ~ 150)
       strong override:              0.30  (tq ~ 500)
     This descent makes "I want to change lanes" preserve authority for a
     smoother visual feedback, while "I'm doing it myself" yields op out
     of the way. Error-boost is bypassed under blinker - driver leads.
  3. Dynamic rate_dn: heavier driver torque -> faster gain decay. Breakpoints
     [150, 350, 600] Nm calibrated against CCNC route-0x49 active-steering
     torque distribution (p25=250, p50=381, p90=619 Nm). At 350 Nm op yields
     at the legacy -0.014 rate; light grip <150 Nm stays sticky.
  """
  if lat_active:
    if blinker_on:
      # Lane change: 4-level driver-intent map. Explicit override of the
      # standard ceiling/shelf/floor map (and error-boost) - when the driver
      # signals, their wheel input takes priority over MDPS authority and
      # tracking-error catch-up. Levels:
      #   tq ~ 0   (hands-off / pure signalling):    0.80
      #   tq ~ 30  (light grip - hand on wheel):     0.55
      #   tq ~ 100-125 (active steering - turning):  0.25
      #   tq ~ 350-500 (strong override):            0.15
      # The 30 Nm transition matches CCNC angle-control EPS reaction at
      # light hand placement (route 0x49: light grip p50=36 Nm).
      # 2026-05-11 (v2): user reported "op grips too strongly" during lane
      # changes — drivelog (routes 0000000a + 0000000c, ~9.4k blinker-active
      # samples) showed p50(100-150 Nm)=0.41 and p50(200-300 Nm)=0.36,
      # meaning op kept 40 %+ authority through the driver's clear-intent
      # zone. Lowered the three middle/active/heavy levels (0.70→0.55 /
      # 0.40→0.25 / 0.30→0.15) and pulled bp_active down (125-150 →
      # 100-125 Nm) so op yields much more as the driver starts turning.
      # The hands-off ceiling stays at 0.80 so when the driver releases,
      # the rate-up=0.05/frame ramp still hits a high ceiling within
      # ~80 ms and op finishes the lane change.
      bp_grip   = 30.0
      bp_active = float(np.interp(v_ego, [2., 11.], [100., 125.]))
      bp_heavy  = float(np.interp(v_ego, [2., 22.], [350., 500.]))
      target = float(np.interp(abs(steering_torque),
                                [0.0, bp_grip, bp_active, bp_heavy],
                                [0.80, 0.55,    0.25,      0.15]))
    else:
      ceiling = float(np.interp(v_ego, [0.5, 1.5], [1.0, 0.85]))
      shelf = float(np.interp(v_ego, [2., 11.], [0.22, 0.30]))
      floor = float(np.interp(v_ego, [2., 22.], [0.1, 0.3]))

      # Error-based ceiling boost. Speed-dependent error_start: at standstill
      # ignore <1.25 deg (column wind-up dominates), at highway sensitive to 0.2 deg.
      error_start = float(np.interp(v_ego, [0., 5.56, 11.1, 33.3],
                                            [1.25, 0.5, 0.3, 0.2]))
      error_mult = float(np.interp(abs(steering_error),
                                    [error_start, error_start * 2], [1.0, 2.0]))
      # Suppress the boost when the driver is clearly overriding (>50% of
      # full-override threshold). Otherwise the tracking-error catch-up
      # could fight a heavy hand: error rises BECAUSE the driver is
      # turning the wheel, then ACIGain ceiling jumps up to 2x just as op
      # should be yielding. Mirrors the blinker branch's "driver leads"
      # philosophy.
      if override_factor > 0.5:
        error_mult = 1.0
      ceiling = min(1.0, ceiling * error_mult)

      # Non-blinker descent breakpoints.
      # 2026-05-11 (v2): first tightening — bp1-bp4 + shelf pulled down
      # so light grip (50-150 Nm) starts the ceiling→shelf descent and
      # moderate driver intent (150-300 Nm) quickly transitions toward
      # floor.
      # 2026-05-11 (v3): user feedback after drivelog 0000000d — at
      # parking-lot speeds (<30 km/h), p75(50-100 Nm) was still ~0.95
      # because bp1=63 / bp2=113 at v=5 m/s left light grip above bp1
      # most of the time. Pulled bp1[50,75]→[30,50], bp2 down (bp2 was
      # [100,125], tried [70,100], settled at [50,70]) so steady-state
      # at 50 Nm reaches ~0.46 at v=5 m/s rather than 0.66. shelf
      # [0.30,0.40]→[0.22,0.30]. np.interp clamping leaves v≥11 m/s
      # (>=40 km/h) behaviour unchanged from v2; only <30 km/h light
      # grip is more responsive (50 Nm: 0.85→~0.46, 75 Nm: 0.38→~0.27).
      # Light grip <30 Nm still gets full ceiling.
      bp1 = float(np.interp(v_ego, [2., 11.], [30., 50.]))
      bp2 = float(np.interp(v_ego, [2., 11.], [50., 70.]))
      bp3 = float(np.interp(v_ego, [2., 11.], [150., 200.]))
      bp4 = float(np.interp(v_ego, [2., 22.], [300., 450.]))
      target = float(np.interp(abs(steering_torque), [bp1, bp2, bp3, bp4],
                                [ceiling, shelf, shelf, floor]))
  else:
    target = 0.0

  # Dynamic rate_dn aligned to CCNC active-steering torque distribution.
  rate_dn_mag = float(np.interp(abs(steering_torque), [150., 350., 600.],
                                                       [0.004, 0.014, 0.04]))
  # Blinker symmetric fast yield/recovery.
  #   Yield: rate_dn forced to 0.05 so the 1.0→0.5 yield reaches the wheel
  #     within ~100 ms — already in place.
  #   Recovery: when the driver releases mid-lane-change, the target jumps
  #     from the active-grip levels (0.15~0.25) up to the hands-off level
  #     (0.80). 2026-05-12 (4th): user feedback "깜빡이가 켜져있더라도
  #     운전자가 핸들을 놓으면 바로 op가 이어받도록" — bumped rate_up
  #     0.05 → 0.10 so the 0.15→0.80 ramp completes in ~40 ms (4 frames @
  #     100 Hz) instead of ~80 ms. Paired with the blinker override_factor
  #     branch (DEADZONE 70 / FULL 130/220) which drops override_factor
  #     to 0 the moment torque falls below ~70 Nm, so desired_angle_deg
  #     snaps to op within 1 frame and ACIGain hits ceiling 40 ms later.
  rate_up_mag = CarControllerParams.ACI_GAIN_RATE_UP
  if blinker_on:
    rate_dn_mag = max(rate_dn_mag, 0.05)
    rate_up_mag = max(rate_up_mag, 0.10)
  gain = rate_limit(target, last_gain, -rate_dn_mag, rate_up_mag)
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
    # HDA2-ALT + CCNC hysteresis state - prevents binary flip of
    # LKAS_ANGLE_ACTIVE / LKA_ASSIST at the authority boundary (the
    # residual low-speed tick source).
    # aci_active_latched: True once authority>=0.3 and stays True until <0.05.
    # passthrough_latched: True when driver really has the wheel, stable across
    # small torque oscillations below the driver_torque_blend threshold.
    # aci_gain_ramp: first-order smoothing 0->1 over ~0.3 s on engagement.
    self.aci_active_latched = False
    self.passthrough_latched = False
    self.aci_gain_ramp = 0.0
    self.aci_gain_last = 0.0
    self.low_speed_cam_latched = False
    self.traffic_following = False
    # Phase 4-A: vehicle model for VM-based jerk/accel limiting.
    # BASELINE_VM (Sportage 5th gen) is used as a SECOND, more conservative
    # safety check after the i6n-tuned VM in apply_steer_angle_limits_vm —
    # both must accept the angle, so the effective limit is the tighter of
    # the two. Mirrors how panda enforces lateral accel/jerk independently
    # of the on-device VM tuning.
    if is_ccnc_angle_platform(CP.flags):
      self.VM = VehicleModel(CP)
      from opendbc.car.hyundai.interface import CarInterface
      baseline_cp = CarInterface.get_non_essential_params("KIA_SPORTAGE_5TH_GEN")
      self.BASELINE_VM = VehicleModel(baseline_cp)
    # Variable-tau LPF state
    self.vtau_lpf = 0.0
    self.vtau_sustained_cnt = 0
    self.vtau_prev_sign = 0
    # Phase 5: driver-override snap state - tracks whether MADS has
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
    # copy.copy() every frame -> id() always changed -> staleness never
    # detected. Tracking COUNTER fixes that.
    self.cam_msg_last_frame = 0
    self.cam_msg_last_counter = -1
    # HOD (hands-on detection) bypass. Opt-in via HOD_BYPASS=1 env var.
    # Default OFF: factory ECU also publishes 0x208 on E-CAN -> dual-publisher
    # collision caused bus-off (busOffCnt 0->1,456, txErr 239/256).
    self.hod_bypass_enabled = os.environ.get("HOD_BYPASS") == "1"
    self.hod_bypass_counter = 0

    # Owned by openpilot so ADAS DRV sees a clean +1 sequence regardless of
    # camera-TX rate vs our frame%5==0 downsample. Wraps at 256, well above
    # any realistic continuity-watchdog window.
    self.suppress_lfa_counter = 0
    self.prev_fault_lfa = 0
    # Lateral-alert flags exposed to controlsd via CarOutput. card.py reads
    # these counters and trips Bool flags at threshold; selfdrived pushes
    # the matching onroadEvents (lateralAccelLimit / steerAngleLimit /
    # cameraDataStale). Frame counters here implement N-frame hysteresis
    # so transient single-frame trips do not spam alerts.
    self.alert_vm_limit_frames = 0
    self.alert_max_angle_frames = 0
    self.alert_cam_stale_frames = 0
    self.was_in_reverse = False

  def update(self, CC, CC_SP, CS, now_nanos):
    self._cc_sp = CC_SP
    EsccCarController.update(self, CS)
    LeadDataCarController.update(self, CC_SP)
    MadsCarController.update(self, self.CP, CC, CC_SP, self.frame)
    if self.frame % 5 == 0:
      # On the HDA2-ALT + CCNC angle-control platform we never TX
      # SCC_CONTROL (factory SCC owns longitudinal - see F3 guard below),
      # so the tuning state produced here is only read by
      # new_actuators.accel as a telemetry report. Keeping the 5-frame
      # cadence is intentional - it matches non-CCNC Hyundai cars and
      # avoids branching the call site.
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

    # Trip the steerAngleLimit alert when fault-avoidance has actually cut
    # the request bit (apply_steer_req=False because angle_limit_counter
    # exceeded MAX_ANGLE_FRAMES). Hysteresis: count up while still cut, decay
    # when active again. Driver-visible threshold = 5 frames (=50ms) so
    # transient single-frame cuts during normal lock-to-lock sweeps do not
    # raise the alert.
    if CC.latActive and abs(CS.out.steeringAngleDeg) >= MAX_ANGLE and not apply_steer_req:
      self.alert_max_angle_frames = min(self.alert_max_angle_frames + 1, 100)
    else:
      self.alert_max_angle_frames = max(self.alert_max_angle_frames - 2, 0)

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

  def _snap_apply_angle_to_wheel(self, steer_angle_safe, reason):
    """Single source of truth for "yield apply_angle_last to physical wheel".

    Two call sites (override_snapped + future) used to duplicate
    `self.apply_angle_last = steer_angle_safe` inline. Centralizing makes
    drivelog replay easier - every snap event becomes searchable in
    cloudlog by reason.
    """
    self.apply_angle_last = steer_angle_safe
    if (self.frame % 100) == 0:
      cloudlog.info(f"snap_to_wheel: reason={reason}")

  def _compute_effective_lat_active(self, CC, ccnc_lka_alt, apply_steer_req,
                                    in_passthrough,
                                    cam_stale_tripped=False, fault_lfa=False):
    """Decide whether the LKAS_ALT packer should mark op as actively steering.

    Returns (effective_lat_active: bool, false_reasons: list[str]). The
    reasons list is emitted at 1Hz to cloudlog so drivelog inspection can
    immediately answer "why was op passive at frame N?".

    Also gates two camera-side fault conditions: cam_stale_tripped (LKAS_ALT
    COUNTER not advancing for >=300 ms — sensor data is unreliable) and
    fault_lfa (the camera ECU itself reports LFA fault). Both must force op
    to passive so MDPS does not act on stale or faulted sensor input.
    """
    if not ccnc_lka_alt:
      return apply_steer_req, []
    reasons = []
    if not CC.latActive:           reasons.append("not_latActive")
    if self.override_snapped:      reasons.append("override_snapped")
    if not apply_steer_req:        reasons.append("no_steer_req")
    if self.was_in_reverse:        reasons.append("was_in_reverse")
    if in_passthrough:             reasons.append("in_passthrough")
    if cam_stale_tripped:          reasons.append("cam_stale")
    if fault_lfa:                  reasons.append("fault_lfa")
    return (len(reasons) == 0), reasons

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
    # dict identity cannot indicate freshness - only the camera-driven
    # COUNTER does. If COUNTER doesn't change for CAM_STALE_FRAMES (>= 25
    # frames ~ 250 ms at 100 Hz), treat camera as dropped and force
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
    # Surface staleness as a hysteretic alert flag - controlsd reads it
    # via CarOutput to push EventName.cameraDataStale. Counter decays
    # quickly when the stream resumes so a recovered link clears the alert
    # within ~10 frames.
    if cam_stale:
      self.alert_cam_stale_frames = min(self.alert_cam_stale_frames + 1, 100)
    else:
      self.alert_cam_stale_frames = max(self.alert_cam_stale_frames - 5, 0)
    # Smooth low-speed authority ramp - replaces the old binary 3 km/h gate.
    # authority ramps 0->1 linearly as vEgoRaw rises from 1 km/h to 3 km/h.
    # Below 1 km/h: 0 (effectively no ACI command). Above 3 km/h: full.
    ACI_SPEED_FULL_MS = 3.0 / 3.6   # full authority at/above 3 km/h
    ACI_SPEED_ZERO_MS = 1.0 / 3.6   # zero authority at/below 1 km/h
    # F8 / R1: Guard against NaN/inf from wheel sensors and planner.
    # vEgoRaw: MDPS/wheel-speed glitch. steeringAngleDeg / steeringTorque:
    # STEERING_SENSORS CAN frame can go all-1s during a bus error or
    # harness fault. actuators.steeringAngleDeg: LatControlAngle can emit
    # NaN if the planner ever divides by zero. Any single NaN reaching
    # np.clip or float() downstream propagates and crashes the rate
    # limiter / trust estimator - so we clamp at the boundary.
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
    # (driver turning wheel 90 deg at 30 km/h applies only 60-80 Nm) -> MADS
    # kept fighting the driver. Lower the full-override torque at low v,
    # keep the higher value at highway for stability.
    # Route 0x49 discovery: on angle-control MDPS the reported column torque
    # INCLUDES EPS reaction force during angle tracking. Light hand grip
    # reads p50=36 / p90=184 Nm (not pure driver input like torque-control
    # cars), so the torque-control thresholds flag every hand placement as
    # full override. Branch on ccnc_lka_alt to use the wider angle-calibrated
    # thresholds.
    blinker_on = bool(CS.out.leftBlinker or CS.out.rightBlinker)
    if ccnc_lka_alt:
      # 2026-05-12 (4th): blinker = explicit lane-change intent. Use lower
      # override thresholds so light hand placement already pulls
      # desired_angle_deg toward the wheel — no fighting the driver's
      # chosen direction. Non-blinker thresholds preserved.
      if blinker_on:
        DRIVER_TORQUE_DEADZONE = CarControllerParams.DRIVER_TORQUE_DEADZONE_ANGLE_BLINKER
        override_low_v  = CarControllerParams.DRIVER_TORQUE_FULL_OVERRIDE_LOW_V_BLINKER
        override_high_v = CarControllerParams.DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V_BLINKER
      else:
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
    # Don't enter override-snap during a turn signal. The blinker authority
    # map in compute_torque_reduction_gain already provides a continuous
    # 4-level descent (0.70 → 0.30) that yields to driver torque without
    # the binary snap state. Skipping entry under blinker keeps
    # effective_lat_active=True throughout the lane change so op's
    # steering command stream never goes silent — pair with the
    # symmetric blinker rate_up so release-to-takeover stays smooth.
    if override_factor >= CarControllerParams.OVERRIDE_SNAP_ENTER_FACTOR and not blinker_on:
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
    # fully passive (cam |delta|~0, LKAS_ANGLE_ACTIVE=1, ACIGain=0 - MDPS
    # idle). Emulate that by keeping `steering_active=False` in the
    # LKAS_ALT packer (via the speed_blend > 0.1 gate), which mirrors
    # the camera's fields verbatim in the op-emitted frame. Driver has
    # no assist AND no resistance at creep speed - identical to stock
    # LFA feel. Hysteresis on vEgoRaw prevents stop-and-go flapping.
    if CS.out.vEgoRaw < LOW_SPEED_PASSTHROUGH_ENTER_MS:
      self.low_speed_cam_latched = True
    elif CS.out.vEgoRaw > LOW_SPEED_PASSTHROUGH_EXIT_MS:
      self.low_speed_cam_latched = False
    # Traffic-following override: a close lead (<3m, with 5m exit) means
    # we're in stop-and-go traffic, NOT a parking lot - keep op engaged
    # so the wheel is held. lead_visible already has 0.5s on/off
    # hysteresis (lead_data_ext.py:_update_lead_visible_hysteresis). The
    # 3/5m distance band keeps the latch stable when the lead drifts near
    # the boundary. lead_distance/lead_visible are populated by
    # LeadDataCarController.update, called every frame from
    # carcontroller.update via the MRO chain at the top of update().
    if self.lead_visible and self.lead_distance < TRAFFIC_FOLLOW_NEAR_M:
      self.traffic_following = True
    elif (not self.lead_visible) or self.lead_distance > TRAFFIC_FOLLOW_FAR_M:
      self.traffic_following = False
    # Combined passthrough latch - used below to force `rate_lat_active=False`
    # in the rate limiter so `apply_angle_last` tracks the actual wheel
    # while passive. The LKAS_ALT packer no longer takes a separate
    # passthrough code path (was a source of frame-format-switch faults
    # on routes 3a/32/34); instead it uses the unified `steering_active`
    # gate which resolves to passive for the same conditions.
    in_passthrough = (self.passthrough_latched or self.low_speed_cam_latched) and not self.traffic_following

    # First-order ramp of ACI gain on re-engagement (smooths the
    # ADAS_ACIAnglTqRedcGainVal step). ~0.3 s at 100 Hz ~ 30 frames.
    ACI_GAIN_RAMP_TAU_FRAMES = 30.0
    if self.aci_active_latched:
      self.aci_gain_ramp = min(1.0, self.aci_gain_ramp + 1.0 / ACI_GAIN_RAMP_TAU_FRAMES)
    else:
      self.aci_gain_ramp = 0.0

    # HDA2-ALT + CCNC angle control: op-only, VM-based jerk/accel limiter
    # at 100 Hz. Panda safety checks at 100Hz frequency - running the
    # rate limiter at 50Hz (frame%2) wasted half the allowed jerk budget
    # because panda only permits 10ms of delta per TX.
    if ccnc_lka_alt:
      # Variable-tau LPF: unified filter replacing curvature LPF + low-speed
      # LPF + jerk FF + error FB. Tau is a continuous function of angle
      # magnitude and speed - strong at center (suppresses straight-line
      # jitter), weak at large angles (fast curve tracking), with low-speed
      # smoothing built in. No step response because tau is continuous.
      entry_th = float(np.interp(v_ego_safe, CarControllerParams.VTAU_ENTRY_TH_BP,
                                              CarControllerParams.VTAU_ENTRY_TH_V))
      entering_curve = abs(op_curv_safe) > abs(self.vtau_lpf) + entry_th
      returning_to_center = abs(op_curv_safe) < abs(self.vtau_lpf) - CarControllerParams.VTAU_EXIT_TH

      if entering_curve or returning_to_center:
        # Soft trigger: fast LPF instead of an instantaneous state jump.
        # alpha @ 100Hz = 0.01/0.06 = 0.17 => ~5 frames to 95% - looks like a
        # smooth slew to EPS rather than a single rate-limited cliff.
        # Drivelog 0x15: cuts EPS frames with delta>1 deg from 1.52% to 0.27% (5.6x)
        # at the cost of +9-58ms entry lag depending on stratum.
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
        # Highway upper-bound: angle_tau goes to 2.5 s for |angle|<1 deg (centered)
        # which is desirable as straight-line jitter suppression at low speed
        # but turns into sluggish path tracking at highway. Cap tau <= 0.15 s
        # at 25 m/s so psi-error corrections happen promptly.
        speed_max_tau = float(np.interp(v_ego_safe, [10.0, 25.0], [2.5, 0.15]))
        vtau = min(vtau, speed_max_tau)

        # Adaptive tau: sustained same-direction change = real correction (not noise).
        # Noise oscillates direction every few frames; real corrections persist.
        cur_sign = 1 if op_curv_safe > self.vtau_lpf + 0.01 else (-1 if op_curv_safe < self.vtau_lpf - 0.01 else 0)
        if cur_sign != 0 and cur_sign == self.vtau_prev_sign:
          self.vtau_sustained_cnt = min(self.vtau_sustained_cnt + 1, 100)
        else:
          self.vtau_sustained_cnt = max(self.vtau_sustained_cnt - 2, 0)
        self.vtau_prev_sign = cur_sign
        # Cap intermediate/final breakpoints with min() so the adaptive stage
        # NEVER increases tau. At highway speeds vtau has already been clamped
        # by speed_max_tau (<=0.15 s @ 25 m/s) below the legacy 0.5 mid-point -
        # without this guard, sustained_cnt building 0->30 would slow tau back
        # up to 0.5 s exactly when the driver wants a small lane-deviation
        # corrected quickly. Low-speed (vtau>=0.5) keeps the original
        # 2-stage descent unchanged.
        vtau = float(np.interp(self.vtau_sustained_cnt, [0, 30, 60],
                                [vtau, min(vtau, 0.5), min(vtau, 0.1)]))

      # During passthrough or while op is inactive, the rate limiter forces
      # apply_angle_last == steer_angle_safe (lateral.py). Mirror that on
      # vtau_lpf so re-engagement starts from the actual wheel angle and the
      # next LPF convergence is short. Without this gate vtau_lpf could
      # accumulate degrees away from the wheel during passthrough, producing
      # a rate-saturated slew at the release boundary.
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

      # Passthrough below 20 km/h (LOW_SPEED_PASSTHROUGH_ENTER_MS) or while
      # the driver firmly has the wheel: let `apply_steer_angle_limits_vm`
      # track steer_angle_safe and let the stuck-angle jitter break
      # short-circuit on its `else` branch. Without this gate the
      # +-0.05 deg jitter injection would keep running at standstill.
      rate_lat_active = bool(CC.latActive) and self.aci_active_latched and not in_passthrough

      if self.override_snapped:
        self._snap_apply_angle_to_wheel(steer_angle_safe, "override_snapped")

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
        # VM rate limiter rejected the command (lateral accel limit violated
        # while rate limit can't pull back fast enough - typically a tight
        # highway on-ramp where v^2 * kappa exceeds MAX_LATERAL_ACCEL=3.59 m/s^2).
        # Previous behavior was to set apply_steer_req=False AND snap
        # apply_angle_last to the actual wheel - MDPS released, the wheel
        # returned to caster, the cluster icon fell off, and the driver
        # experienced a SILENT disengage with no takeover alert. That was
        # observed mid-ramp on drives 0x09 seg27/28/30/36 (v=20-28 m/s,
        # sa=20-26 deg): apply_angle frozen for 1.5-3 s while wheel released.
        #
        # Instead, hold the previous compliant angle (apply_angle_last
        # unchanged) and keep apply_steer_req=True. MDPS continues to track
        # the held angle, the wheel doesn't snap to caster, the cluster icon
        # stays, and the driver feels op is "still working" rather than
        # "let go". Panda safety still validates per-frame angle delta vs
        # last-accepted, so this is safe (delta=0 is always within window).
        # Log telemetry so we can quantify how often this fires per drive.
        if v_ego_safe > 8.0:
          self.alert_vm_limit_frames = min(self.alert_vm_limit_frames + 1, 100)
          if (self.alert_vm_limit_frames % 100) == 1:  # rate-limit cloudlog to ~1Hz
            cloudlog.warning(
              f"VM_LIMIT_TRIP: holding apply_angle={self.apply_angle_last:.1f} "
              f"v={v_ego_safe:.1f} sa_meas={steer_angle_safe:.1f} "
              f"op_curv={op_curv_safe:.1f} max_lat_a="
              f"{CarControllerParams.ANGLE_LIMITS_VM.MAX_LATERAL_ACCEL:.2f}"
            )
      else:
        self.apply_angle_last = apply_angle
        self.alert_vm_limit_frames = max(self.alert_vm_limit_frames - 2, 0)

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
    # Whenever MADS is enabled, the cluster MUST show the green steering icon
    # (icon=2) regardless of whether ACC main is on. This includes low-speed
    # passthrough, override snap, and any transient CC.latActive=False window
    # where MADS is still the active assistance source.
    # Off-but-available (ACC main on, MADS off): icon=0 (off but visible).
    # ACC main off and MADS off: None -> camera passthrough (stock LFA icon).
    mads_enabled = getattr(self._cc_sp, 'mads', None) and self._cc_sp.mads.enabled
    if ccnc_lka_alt:
      if mads_enabled:
        mads_lka_icon = 2
      elif CS.out.cruiseState.available:
        mads_lka_icon = 0
      else:
        mads_lka_icon = None
    else:
      mads_lka_icon = None

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
      gear = CS.out.gearShifter
      if gear == structs.CarState.GearShifter.reverse:
        self.was_in_reverse = True
      elif gear in (structs.CarState.GearShifter.drive,
                    structs.CarState.GearShifter.sport,
                    structs.CarState.GearShifter.eco,
                    structs.CarState.GearShifter.manumatic,
                    structs.CarState.GearShifter.low,
                    structs.CarState.GearShifter.brake):
        # Driver explicitly shifted to a forward gear — clear the latch
        # immediately so multi-direction parking maneuvers (R -> D -> R ->
        # D crawl) re-engage op on each forward leg, instead of waiting
        # for a 10 km/h speed crossing that may never arrive in a tight
        # parking lot. `low` (L) and `brake` (B regen) cover EV/HEV regen
        # gear positions that may exist on future CCNC platforms even
        # though current Ioniq 6 N CAN-FD parses only emit {P,R,N,D,S}.
        self.was_in_reverse = False
      elif self.was_in_reverse and CS.out.vEgo > 10 * CV.KPH_TO_MS:
        # Fallback: ambiguous gear (park, neutral, brake, low, unknown);
        # clear once we are clearly under way.
        self.was_in_reverse = False

    # in_passthrough gates effective_lat_active so the LKAS_ALT packer
    # emits LKAS_ANGLE_ACTIVE=1 (= camera passive value) and ACIGain=0 below
    # 20 km/h (LOW_SPEED_PASSTHROUGH_ENTER_MS). Without this gate the angle
    # bit stayed at 2 even while apply_angle_last was being forced to follow
    # the wheel - MDPS read that as "hold this exact angle" and refused to
    # let caster torque return the wheel toward center, making parking-lot
    # maneuvers feel sticky.
    # Camera-side faults gate op steering identically to the alert
    # thresholds applied in card.py: stale LKAS_ALT counter for >=300 ms,
    # or any FAULT_LFA bit from the camera ECU. Both force MDPS into
    # passive (LKAS_ANGLE_ACTIVE=1, ACIGain=0) so we don't ride a stale
    # sensor or a faulted camera.
    cam_stale_tripped = bool(self.alert_cam_stale_frames >= 30)
    fault_lfa_bool = bool(getattr(CS, 'fault_lfa', 0))
    effective_lat_active, lat_passive_reasons = self._compute_effective_lat_active(
        CC, ccnc_lka_alt, apply_steer_req, in_passthrough,
        cam_stale_tripped=cam_stale_tripped, fault_lfa=fault_lfa_bool)
    # 1Hz cloudlog of why op is passive - invaluable for drivelog forensics.
    if ccnc_lka_alt and lat_passive_reasons and (self.frame % 100) == 0:
      cloudlog.info(f"lat_passive: {','.join(lat_passive_reasons)}")

    # ACIGain: sunnypilot-style torque reduction gain (torque + speed -> gain).
    # Must use effective_lat_active (not CC.latActive) to ensure gain=0
    # when LKAS_ANGLE_ACTIVE=1 (passive). Panda rejects gain>0 with ACTIVE=1.
    effective_aci_gain = None
    if ccnc_lka_alt and lkas_alt_cam_msg is not None:
      # Tracking error feeds the ceiling-boost: when op's commanded angle
      # diverges from the actual wheel (corner-entry transient before MDPS
      # catches up), grant temporary extra authority. NaN-safe via steer_angle_safe.
      steering_error = self.apply_angle_last - steer_angle_safe
      effective_aci_gain = compute_torque_reduction_gain(
        steer_torque_safe, v_ego_safe, effective_lat_active, self.aci_gain_last,
        steering_error=steering_error, blinker_on=blinker_on,
        override_factor=override_factor)
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
                                                         effective_aci_gain=effective_aci_gain,
                                                         mads_force_assist=bool(mads_enabled and ccnc_lka_alt),
                                                         cam_invalid=bool(cam_stale_tripped or fault_lfa_bool)))

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
    # gateway ECU on bus 1 (not forwarded from the camera - see
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
    # Active whenever MADS is enabled (not just latActive) - prevents
    # hands-on warning during transient latActive=False (driver override,
    # low-speed passthrough). Disabled only when MADS is fully OFF.
    # Gated on hod_bypass_enabled (HOD_BYPASS=1 env var) - default OFF.
    # Factory ECU already publishes 0x208 on bus 1; unconditional TX
    # creates a dual-publisher collision causing CAN bus 1 bus-off
    # (busOffCnt 0->1,456 over routes 0x28->0x4f, peak txErr=239/256).
    if ccnc_lka_alt and self.frame % 10 == 0 and mads_enabled and self.hod_bypass_enabled:
      can_sends.append(hyundaicanfd.create_hod_bypass(self.CAN.ECAN, self.hod_bypass_counter))
      self.hod_bypass_counter = (self.hod_bypass_counter + 2) & 0xFF

    # Note: LFAHDA_CLUSTER (0x1E0) op TX was tried in commit 7cda01d to
    # override factory HDA=0/LFA=0 emits, hoping to force the cluster's
    # green steering icon. Drivelog 0000000d (19 segs) confirmed zero
    # panda errors (busOff/canSendErrs/canFwdErrs all Δ0) but ALSO zero
    # cluster effect — the cluster ignores LFAHDA_CLUSTER fields and
    # reads CCNC_0x161 LFA_ICON, which factory keeps at HIDDEN because
    # op suppresses stock LFA via CAM_0x362. Re-enabling would require
    # finding a way to influence CCNC_0x161 without dual-publisher
    # conflict — a separate architectural task. TX code removed; see
    # git history (commit 7cda01d) for the prior attempt.

    # blinkers
    if lka_steering and self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
      can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, CC.leftBlinker, CC.rightBlinker))

    # F3: HDA2-ALT + CCNC platform NEVER does openpilot longitudinal -
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
            # for ACC cancel - factory SCC natively publishes on bus 1, and
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
