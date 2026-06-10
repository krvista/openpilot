import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, make_tester_present_msg, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits, apply_steer_angle_limits_vm, common_fault_avoidance
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

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2

# Width of the synthetic LFA_BUTTON pulse op emits on each MADS enabled-state
# edge to keep the cluster's LFA green icon in lockstep with MADS. 2 frames at
# 100 Hz ≈ 20 ms — same envelope the stock camera uses when it ACKs a real
# wheel-button press, so the gateway's edge-detector behaves identically.
LFA_SYNC_PULSE_FRAMES = 2

# Secondary safety VM uses this platform's CarSpecs to enforce a more
# conservative lateral accel/jerk envelope than the on-device i6n VM.
ANGLE_SAFETY_BASELINE_MODEL = "KIA_SPORTAGE_HEV_2026"


def get_baseline_safety_cp():
  from opendbc.car.hyundai.interface import CarInterface
  return CarInterface.get_non_essential_params(ANGLE_SAFETY_BASELINE_MODEL)


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


# Low-speed freeze latch (hysteresis 20/22 km/h).
# Below 20 km/h, hand the wheel back to the driver / EPS so caster torque
# can self-center the wheel - needed for parking-lot maneuvers, where the
# user routinely reaches ~20 km/h.
LOW_SPEED_PASSTHROUGH_ENTER_MS = 20.0 / 3.6   # ~ 5.56 m/s
LOW_SPEED_PASSTHROUGH_EXIT_MS  = 22.0 / 3.6   # ~ 6.11 m/s

# Traffic-following lead distance hysteresis. When a lead is closer than
# this, op stays engaged below the freeze threshold so the wheel is held
# (stop-and-go traffic).
TRAFFIC_FOLLOW_NEAR_M = 3.0
TRAFFIC_FOLLOW_FAR_M  = 5.0

# Failsafe: if apply_steer_angle_limits_vm returns None (lateral accel
# limit violated and rate limit cannot pull back fast enough) for this
# many consecutive frames, force STEER_REQ=0 instead of holding a
# frozen angle with the active flag. 50 ms is short enough to filter
# momentary rate-limit blips and shorter than the 500 ms alert window.
# Phase 5e introduced this (commit 6df2939); Phase 6d adds the
# angle-passive latch as an additional false-reason on the same chain.
VM_REJECT_FORCE_PASSIVE_FRAMES = 5

# Parking-mode latch — layered on top of the <20 km/h low_speed_cam_latched
# passthrough. The low-speed latch hands the wheel back for caster self-
# centering whenever speed is low; this stickier latch additionally keeps op
# passive up to ~30 km/h once a clear parking signature is seen, so the driver
# retains full manual control through lot ramps / tight maneuvers without op
# fighting the wheel. Entry requires BOTH a sustained low-speed window AND at
# least one large-wheel event during it; exit needs a sustained higher speed
# (hysteresis above the 30 km/h entry) so a 30 km/h crawl does not chatter the
# latch on/off. ENTER_WHEEL_DEG = 270° ≈ 18° road wheel ≈ 9 m turn radius
# (steerRatio 14.96): clearly tighter than city-intersection turns (~180°) so
# the latch is specific to parking-lot maneuvers. ccnc-drivelog route 0x39
# parking sections measured 180-360° on 48% of frames, so 270° is reliably
# crossed at least once on a multi-floor spiral ramp.
PARKING_MODE_ENTER_MS             = 30.0 / 3.6   # ≤30 km/h sustained to arm
PARKING_MODE_ENTER_SUSTAIN_FRAMES = 300          # 3 s @ 100 Hz
PARKING_MODE_ENTER_WHEEL_DEG      = 270.0        # ≈9 m radius parking turn
PARKING_MODE_EXIT_MS              = 33.0 / 3.6   # exit hysteresis above entry
PARKING_MODE_EXIT_SUSTAIN_FRAMES  = 200          # 2 s @ 100 Hz



def compute_torque_reduction_gain(steering_torque, v_ego_kph, lat_active, last_gain, steering_error, blinker_on=False):
  # Reference sunnypilot 17-line ACIGain shape (Phase 1 commit 54ab570),
  # augmented across Phase 5 and Phase 6 by stateless hooks that each
  # take a single per-frame signal as input:
  #   - Phase 5c (B3) — rate_up drift-recovery boost on |steering_error|.
  #   - Phase 5d (A2) — blinker ceiling cap to yield during lane changes.
  #   - Phase 6a (N6a) — torque interp [100, 350] aligned with the
  #                       100 Nm deadzone (was [140, 420]).
  #   - Phase 6c-2 (N7b commit b6e5842) — torque-aware suppression of
  #                       the error_mult boost so grip-induced error
  #                       does not amplify the ceiling against the driver.
  if lat_active:
    base_ceiling = np.interp(v_ego_kph, [0, 20, 40, 120], [0.4, 0.62, 0.85, 1.0])
    # Error-based boost reduction gain: at 0 kph, ignore errors under 1.25°.
    error_start = np.interp(v_ego_kph, [0, 20, 40, 120], [1.25, 0.5, 0.3, 0.2])
    error_mult_raw = np.interp(abs(steering_error), [error_start, error_start*2], [1.0, 2])
    # Phase 6c-2 N7b: the error_mult boost was designed to recover op
    # tracking when hands-off drift opens steering_error (i.e. ACIGain
    # ceiling 1→2x to push MDPS harder back onto the desired angle).
    # Under driver grip, the driver is the source of the error — boosting
    # MDPS then fights the driver. Drivelog 0000001f (Phase 6a build,
    # 71.5k urban frames in rain) measured sustained 200+ Nm grip with
    # mean dynamic_ceiling=1.0 (saturated) while the driver was actively
    # turning the wheel. Linearly suppress the boost from full at the
    # 100 Nm deadzone to zero at the 180 Nm low-v full-override point;
    # above 180 Nm there is no boost at all. The base_ceiling × 1 line
    # remains, so light-grip / hands-off recovery is unchanged.
    torque_suppress = np.interp(abs(steering_torque), [100, 180], [1.0, 0.0])
    error_mult = 1.0 + (error_mult_raw - 1.0) * torque_suppress
    dynamic_ceiling = min(1.0, base_ceiling * error_mult)
    # Phase 5d A2 (commit 4a4d29b): when the driver signals intent with
    # the blinker, force the MDPS ceiling down so a light-grip lane
    # change does not have to fight op torque. 0.45 mirrors the
    # sunnypilot 18d75ca 4-level descent at the moderate point of the
    # original curve. Combined with the Phase 5b (B2) blinker driver-
    # torque deadzone shift (70/130/220), this gives both command-side
    # (B1 blend on lowered override) and authority-side yield.
    if blinker_on:
      dynamic_ceiling = min(dynamic_ceiling, 0.45)
    target = np.interp(abs(steering_torque), [100, 350], [dynamic_ceiling, 0.19])
  else:
    target = 0.0
  delta = target - last_gain
  rate_dn = np.interp(abs(steering_torque), [0, 300, 700], [0.004, 0.01, 0.04])
  # Phase 5c B3 (commit 41a16ad): when |steering_error| > 0.5°, climb up
  # to 10× faster so ACIGain recovers from a brief grip event within
  # ~250 ms instead of 2.5 s. Below 0.5° rate_up matches the sunnypilot
  # reference 0.004 — no behaviour change on the steady-state path. The
  # 0.04 cap matches the legacy err_boost shape verified against
  # drivelog 0000001f (drift event ACIGain mean 0.977 with the boost
  # vs 0.583 with only the reference rate_up).
  rate_up = float(np.interp(abs(steering_error), [0.5, 1.5], [0.004, 0.04]))
  gain = last_gain + max(-rate_dn, min(rate_up, delta))
  return round(gain / 0.004) * 0.004


def sp_smooth_angle(v_ego_raw: float, apply_angle: float, apply_angle_last: float) -> float:
  """Speed-dependent, slew/maneuver-aware EMA on the commanded steering angle.

  Heavy smoothing at low speed (alpha=0.05) suppresses parking-lot / model-jitter
  micro-oscillations while alpha=1 at/above SMOOTHING_ANGLE_MAX_VEGO disables it.

  Phase 6g-1: the speed-alpha alone lagged corner-entry commands (no lead comp),
  so a real bend ran wide before op caught up. Release the EMA as the command gap
  grows past a jitter floor: small |gap| (oscillation) keeps the heavy speed-alpha
  (저속 떨림 absorption), a sustained/large |gap| (real maneuver) passes through.

  Phase 6g-2: (a) cap the released alpha at SMOOTHING_ANGLE_RELEASE_MAX so a command
  overshoot is not slammed to the wheel ("휙"); (b) a low-speed micro-deadband holds
  the angle for sub-perceptible command changes, killing the ~25 km/h 5-7 Hz dither.

  Phase 6h-1: jitter absorption moved upstream into controlsd's speed-dependent
  curvature LP with matched lead. This EMA is now a light linear filter
  (alpha >= 0.3) plus a CAN-LSB deadband (0.1 deg); the gap-release path is
  disabled via SMOOTHING_ANGLE_RELEASE_HI_DEG = 1e6 (constants-only switch —
  the code path is kept for rollback).
  """
  gap_signed = apply_angle - apply_angle_last
  gap = abs(gap_signed)
  # (6g-2c) low-speed micro-jitter deadband: ignore sub-threshold dither.
  if (v_ego_raw < CarControllerParams.SMOOTHING_ANGLE_DEADBAND_MAX_VEGO
      and gap < CarControllerParams.SMOOTHING_ANGLE_DEADBAND_DEG):
    return apply_angle_last
  adjusted_alpha = np.interp(v_ego_raw, CarControllerParams.SMOOTHING_ANGLE_VEGO_MATRIX,
                              CarControllerParams.SMOOTHING_ANGLE_ALPHA_MATRIX)
  adjusted_alpha = float(min(float(adjusted_alpha), 1.))
  # Maneuver release: scale alpha up with |gap| between LO and HI degrees, but only
  # up to RELEASE_MAX (6g-2a) so a fast catch-up keeps ~30% damping. If the speed
  # alpha already exceeds RELEASE_MAX (high speed) the release adds nothing.
  release = float(np.interp(gap, [CarControllerParams.SMOOTHING_ANGLE_RELEASE_LO_DEG,
                                  CarControllerParams.SMOOTHING_ANGLE_RELEASE_HI_DEG], [0.0, 1.0]))
  headroom = max(CarControllerParams.SMOOTHING_ANGLE_RELEASE_MAX - adjusted_alpha, 0.0)
  adjusted_alpha_limited = adjusted_alpha + headroom * release
  return (apply_angle * adjusted_alpha_limited) + (apply_angle_last * (1 - adjusted_alpha_limited))


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
    # Low-speed camera passthrough latch (kept-feature #11): hands back the
    # wheel to caster torque for parking-lot self-centering. Hysteresis on
    # vEgoRaw plus traffic_following keeps op engaged when a close lead is
    # present (stop-and-go traffic).
    self.low_speed_cam_latched = False
    self.traffic_following = False
    # Diagnostic: log CCNC_0x161.LFA_ICON transitions.
    self.prev_lfa_icon = -1
    # CCNC angle-control vehicle models.
    # VM uses the on-device i6n CP. BASELINE_VM uses KIA_SPORTAGE_HEV_2026
    # CarSpecs as a second, safety-baseline check after the i6n VM in
    # apply_steer_angle_limits_vm — both must accept the angle. Mirrors the
    # reference sunnypilot implementation and panda's hardcoded safety params.
    if is_ccnc_angle_platform(CP.flags):
      self.VM = VehicleModel(CP)
      self.BASELINE_VM = VehicleModel(get_baseline_safety_cp())
    # F1: camera staleness tracker (kept-feature #10) — LKAS_ALT COUNTER
    # advance is monitored so a stale link forces passive output.
    self.cam_msg_last_frame = 0
    self.cam_msg_last_counter = -1
    # ACIGain (ADAS_ACIAnglTqRedcGainVal) rate-limit state. Decoupled from
    # apply_torque_last so the torque-control branch (non-ccnc) and the
    # angle-control branch can rate-limit their own outputs independently.
    self.aci_gain_last = 0.0

    # Owned by openpilot so ADAS DRV sees a clean +1 sequence regardless of
    # camera-TX rate vs our frame%5==0 downsample.
    self.suppress_lfa_counter = 0
    self.prev_fault_lfa = 0
    # MADS↔cluster sync (kept-feature #12): on every MADS enabled-state edge,
    # hold LFA_BUTTON=1 on LKAS_ALT for LFA_SYNC_PULSE_FRAMES so the gateway
    # toggles the cluster icon exactly once per transition.
    self.prev_mads_enabled = False
    self.lfa_sync_pulse_remaining = 0
    # Lateral-alert hysteresis (kept-feature #18) — N-frame counters that
    # card.py reads via CarOutput to push onroadEvents.
    self.alert_vm_limit_frames = 0
    self.alert_vm_limit_cooldown_frames = 0
    self.alert_max_angle_frames = 0
    self.alert_cam_stale_frames = 0
    self.was_in_reverse = False
    # Phase 5e failsafe: persistent VM rate-limit rejection counter.
    self.vm_reject_consecutive_frames = 0
    # Phase 6d: 1-bool latch for angle-aware passive hysteresis. Set
    # True when |wheel| ≥ ANGLE_PASSIVE_ENTER_WHEEL_DEG AND |torque|
    # ≥ ANGLE_PASSIVE_ENTER_TORQUE_NM; cleared when |torque| drops
    # below ANGLE_PASSIVE_EXIT_TORQUE_NM or lat_active goes False.
    # Mirrors the Phase 5e vm_reject_consecutive_frames latch pattern
    # (1-bool / 1-counter "essential hysteresis", not a state machine).
    self.angle_passive_active = False
    # Phase 6e-1: counter for the 5-frame transient-blip filter on
    # angle_passive entry. Resets whenever the entry conjunction
    # becomes false or lat_active goes false.
    self.angle_passive_enter_frames = 0
    # Parking-mode latch (see PARKING_MODE_* constants). Arms on a sustained
    # ≤30 km/h window that contains a parking signature (≥270° wheel event OR
    # a reverse-gear event); holds op passive until a sustained >33 km/h
    # release, then re-arms clean.
    self.parking_low_speed_frames = 0
    self.parking_signature_seen = False
    self.parking_mode_active = False
    self.parking_exit_frames = 0

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

    # Steering. CCNC angle-control platform skips the torque-control limiter
    # and the MAX_ANGLE / common_fault_avoidance gate — both are torque-path
    # protections (EPS fault avoidance when torque is applied at high angle).
    # Angle-control uses LKAS_ANGLE_ACTIVE + ACIGain, which the panda
    # safety hooks (HYUNDAI_CANFD_ANGLE_STEERING_LIMITS) enforce
    # independently.
    if is_ccnc_angle_platform(self.CP.flags):
      apply_torque = 0
      apply_steer_req = bool(CC.latActive)
      torque_fault = False
      self.alert_max_angle_frames = max(self.alert_max_angle_frames - 2, 0)
    else:
      new_torque = int(round(actuators.torque * self.params.STEER_MAX))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.params)

      # >90 degree steering fault prevention
      self.angle_limit_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringAngleDeg) >= MAX_ANGLE, CC.latActive,
                                                                         self.angle_limit_counter, MAX_ANGLE_FRAMES,
                                                                         MAX_ANGLE_CONSECUTIVE_FRAMES)

      # steerAngleLimit alert hysteresis: count up while fault-avoidance has
      # actually cut the request bit, decay when active again. Threshold
      # 5 frames (=50 ms) so transient single-frame cuts during normal
      # lock-to-lock sweeps don't raise the alert.
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

    if self.alert_vm_limit_cooldown_frames > 0:
      self.alert_vm_limit_cooldown_frames -= 1

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

    # CCNC angle-control platform path
    ccnc_lka_alt = is_ccnc_angle_platform(self.CP.flags)
    lkas_alt_cam_msg = getattr(CS, 'lkas_alt_cam_msg', None) if ccnc_lka_alt else None

    # F1 / R4: Camera staleness detection via LKAS_ALT COUNTER. If the
    # COUNTER field stops advancing for >=CAM_STALE_FRAMES (~250 ms), treat
    # the camera link as dropped and force passive output.
    CAM_STALE_FRAMES = 25
    cam_stale = False
    if ccnc_lka_alt and lkas_alt_cam_msg is not None:
      cam_counter = int(lkas_alt_cam_msg.get("COUNTER", -1))
      if cam_counter != self.cam_msg_last_counter:
        self.cam_msg_last_frame = self.frame
        self.cam_msg_last_counter = cam_counter
      if (self.frame - self.cam_msg_last_frame) > CAM_STALE_FRAMES:
        cam_stale = True
    if cam_stale:
      self.alert_cam_stale_frames = min(self.alert_cam_stale_frames + 1, 100)
    else:
      self.alert_cam_stale_frames = max(self.alert_cam_stale_frames - 5, 0)

    # F8 / R1: NaN guards on raw sensor / planner inputs.
    v_ego_safe = float(np.clip(CS.out.vEgoRaw, 0.0, 100.0)) if np.isfinite(CS.out.vEgoRaw) else 0.0
    steer_angle_safe = float(CS.out.steeringAngleDeg) if np.isfinite(CS.out.steeringAngleDeg) else 0.0
    steer_torque_safe = float(CS.out.steeringTorque) if np.isfinite(CS.out.steeringTorque) else 0.0
    op_curv_raw = float(CC.actuators.steeringAngleDeg)
    op_curv_safe = op_curv_raw if np.isfinite(op_curv_raw) else steer_angle_safe
    blinker_on = bool(CS.out.leftBlinker or CS.out.rightBlinker)

    # Driver override factor — used as a hands_off gate for the low-speed
    # camera passthrough latch (kept-feature #11) and as the blend
    # coefficient for the heavy-grip yield (Phase 5a). When the blinker
    # is on, lower thresholds so a light grip during a lane change
    # immediately produces override_factor > 0 and op yields.
    if ccnc_lka_alt:
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
    override_factor = float(np.clip((abs(steer_torque_safe) - DRIVER_TORQUE_DEADZONE) /
                                     max(full_override_torque - DRIVER_TORQUE_DEADZONE, 1.0), 0.0, 1.0))

    # Low-speed camera passthrough latch (kept-feature #11).
    # 11th: STEER_THRESHOLD=350 Nm leaves a 100-350 Nm band where the driver
    # is actively steering but `steeringPressed` is False — require
    # override_factor ≤ 0.5 to also agree before declaring hands-off.
    hands_off = (not CS.out.steeringPressed) and (override_factor <= 0.5)
    if CS.out.vEgoRaw < LOW_SPEED_PASSTHROUGH_ENTER_MS and not hands_off:
      self.low_speed_cam_latched = True
    elif CS.out.vEgoRaw > LOW_SPEED_PASSTHROUGH_EXIT_MS or hands_off:
      self.low_speed_cam_latched = False
    # Traffic-following keeps op engaged in stop-and-go even below the
    # low-speed freeze threshold. lead_visible / lead_distance are
    # populated by LeadDataCarController.update.
    if self.lead_visible and self.lead_distance < TRAFFIC_FOLLOW_NEAR_M:
      self.traffic_following = True
    elif (not self.lead_visible) or self.lead_distance > TRAFFIC_FOLLOW_FAR_M:
      self.traffic_following = False
    in_passthrough = self.low_speed_cam_latched and not self.traffic_following

    # Parking-mode latch (layered on low_speed_cam_latched). A sustained
    # ≤30 km/h window containing a clear parking signature — a ≥270° (≈9 m
    # radius) wheel event OR a reverse-gear event — holds op passive up to
    # ~30 km/h so the driver keeps full manual control through lot ramps /
    # multi-point maneuvers. Reverse is included because it never happens in
    # normal forward road driving and the Ioniq 6 N's large turning circle
    # routinely needs a reverse to clear tight lot turns; arming on it also
    # bridges the slow forward crawl right after R→D (was_in_reverse clears
    # the instant a forward gear is selected). Exits only on a sustained
    # >33 km/h, then re-arms clean so a later low-speed stretch without a
    # parking signature does not re-trip on a stale flag.
    if v_ego_safe <= PARKING_MODE_ENTER_MS:
      self.parking_low_speed_frames = min(self.parking_low_speed_frames + 1,
                                          PARKING_MODE_ENTER_SUSTAIN_FRAMES)
      if (abs(steer_angle_safe) >= PARKING_MODE_ENTER_WHEEL_DEG
          or CS.out.gearShifter == structs.CarState.GearShifter.reverse):
        self.parking_signature_seen = True
    else:
      self.parking_low_speed_frames = 0
    if (self.parking_low_speed_frames >= PARKING_MODE_ENTER_SUSTAIN_FRAMES
        and self.parking_signature_seen):
      self.parking_mode_active = True
    if v_ego_safe > PARKING_MODE_EXIT_MS:
      self.parking_exit_frames = min(self.parking_exit_frames + 1,
                                     PARKING_MODE_EXIT_SUSTAIN_FRAMES)
    else:
      self.parking_exit_frames = 0
    if self.parking_mode_active and self.parking_exit_frames >= PARKING_MODE_EXIT_SUSTAIN_FRAMES:
      self.parking_mode_active = False
      self.parking_signature_seen = False
      self.parking_low_speed_frames = 0

    # CCNC angle-control: reference sp_smooth_angle EMA, then BASELINE_VM
    # double-limited apply_steer_angle_limits_vm. Mirrors the sunnypilot
    # reference flow.
    if ccnc_lka_alt:
      # Phase 6F2-A pre-frame anchor: the post-frame clamp later in this
      # method sets self.apply_angle_last := wheel AFTER the current frame's
      # apply_angle is already computed below at apply_steer_angle_limits_vm
      # using the stale (op_curv-tracking) apply_angle_last. On heavy-override
      # transition frames the TX'd apply therefore lags wheel by up to one
      # episode of VM drift. ccnc-drivelog 8-route sim on the Phase 6f-2
      # build measured heavy-override |apply - wheel| p95 = 27.0° (target
      # ≪ 5°). Anchoring here ensures the current frame's VM step starts
      # from the wheel, collapsing transition-frame mismatch. The later
      # post-frame clamp still runs, covering the angle_passive_active
      # case which is updated mid-method.
      if override_factor >= 0.9:
        self.apply_angle_last = float(np.clip(steer_angle_safe,
                                              -self.params.ANGLE_LIMITS.STEER_ANGLE_MAX,
                                               self.params.ANGLE_LIMITS.STEER_ANGLE_MAX))
      desired_angle = float(np.clip(op_curv_safe, -self.params.ANGLE_LIMITS.STEER_ANGLE_MAX,
                                                   self.params.ANGLE_LIMITS.STEER_ANGLE_MAX))
      if abs(v_ego_safe) < CarControllerParams.SMOOTHING_ANGLE_MAX_VEGO:
        desired_angle = sp_smooth_angle(v_ego_safe, desired_angle, self.apply_angle_last)

      # Heavy-grip yield blend: bias the commanded angle toward the actual
      # wheel proportionally to override_factor so the VM rate limiter's
      # slew target is closer to where the wheel already is — op stops
      # pulling against the driver without needing a snap state.
      # Light-grip dead-band: ignore override_factor ≤ 0.1 (~108 Nm low-v
      # / ~125 Nm high-v) so resting hands on the wheel produce no blend
      # and the reference flow continues unmodified. Moderate two-hand
      # grip saturates at override_factor=0.5 (~140 Nm low-v / ~225 Nm
      # high-v): drivelog 0000001f showed 38% of 150-300 Nm grip frames
      # stuck below full blend with the previous Phase 6a divisor 0.9 —
      # narrowing the divisor to 0.4 reaches full wheel-tracking at
      # typical two-hand grip torques (Phase 6c-1 commit 9d51e46).
      if override_factor > 0.1:
        blend = min((override_factor - 0.1) / 0.4, 1.0)
        desired_angle = (1.0 - blend) * desired_angle + blend * steer_angle_safe

      apply_angle = apply_steer_angle_limits_vm(
        desired_angle, self.apply_angle_last, v_ego_safe,
        steer_angle_safe, CC.latActive, self.params, self.VM,
      )
      if apply_angle is not None and self.CP.carFingerprint != ANGLE_SAFETY_BASELINE_MODEL:
        apply_angle = apply_steer_angle_limits_vm(
          apply_angle, self.apply_angle_last, v_ego_safe,
          steer_angle_safe, CC.latActive, self.params, self.BASELINE_VM,
        )

      if apply_angle is None:
        # VM rate limiter rejected the command (lateral accel limit violated
        # while rate limit can't pull back fast enough). Hold the previous
        # compliant angle (apply_angle_last unchanged), trip the alert.
        self.vm_reject_consecutive_frames += 1
        if v_ego_safe > 8.0:
          self.alert_vm_limit_frames = min(self.alert_vm_limit_frames + 1, 100)
          if self.alert_vm_limit_frames == 50 and self.alert_vm_limit_cooldown_frames == 0:
            self.alert_vm_limit_cooldown_frames = 1000
      else:
        self.vm_reject_consecutive_frames = 0
        self.apply_angle_last = apply_angle
        self.alert_vm_limit_frames = max(self.alert_vm_limit_frames - 2, 0)

      if not CC.latActive:
        self.apply_angle_last = float(np.clip(steer_angle_safe,
                                              -self.params.ANGLE_LIMITS.STEER_ANGLE_MAX,
                                               self.params.ANGLE_LIMITS.STEER_ANGLE_MAX))

    # MADS-driven LKA_ICON for the LKAS_ALT message:
    # MADS enabled → green icon (2), ACC main available → off-but-visible (0),
    # otherwise None → mirror camera value.
    mads_enabled = bool(getattr(self._cc_sp, 'mads', None) and self._cc_sp.mads.enabled)
    if ccnc_lka_alt:
      if mads_enabled:
        mads_lka_icon = 2
      elif CS.out.cruiseState.available:
        mads_lka_icon = 0
      else:
        mads_lka_icon = None
    else:
      mads_lka_icon = None

    # MADS↔cluster LFA sync pulse (kept-feature #12). On every MADS
    # enabled-state edge, hold LFA_BUTTON=1 for LFA_SYNC_PULSE_FRAMES so
    # the gateway toggles the cluster icon exactly once per transition.
    if ccnc_lka_alt and mads_enabled != self.prev_mads_enabled:
      self.lfa_sync_pulse_remaining = LFA_SYNC_PULSE_FRAMES
    self.prev_mads_enabled = mads_enabled
    lfa_sync_pulse = self.lfa_sync_pulse_remaining > 0
    if self.lfa_sync_pulse_remaining > 0:
      self.lfa_sync_pulse_remaining -= 1

    # FAULT_LFA edge state kept so we can decide cam_invalid without the
    # original cloudlog diagnostics.
    if ccnc_lka_alt:
      self.prev_fault_lfa = int(getattr(CS, 'fault_lfa', 0))
      lfa_icon = int(getattr(CS, 'msg_161', {}).get('LFA_ICON', 0)) if getattr(CS, 'msg_161', None) else 0
      self.prev_lfa_icon = lfa_icon

      # Reverse gear latch — used by the LKAS_ALT active gate so op stays
      # passive across a reverse-gear maneuver and through the subsequent
      # forward crawl, unless the driver shifts to a clear forward gear or
      # crosses 10 km/h. Existing safety behaviour, unrelated to vtau.
      gear = CS.out.gearShifter
      if gear == structs.CarState.GearShifter.reverse:
        self.was_in_reverse = True
      elif gear in (structs.CarState.GearShifter.drive,
                    structs.CarState.GearShifter.sport,
                    structs.CarState.GearShifter.eco,
                    structs.CarState.GearShifter.manumatic,
                    structs.CarState.GearShifter.low,
                    structs.CarState.GearShifter.brake):
        self.was_in_reverse = False
      elif self.was_in_reverse and CS.out.vEgo > 10 * CV.KPH_TO_MS:
        self.was_in_reverse = False

    # Effective lat_active for the LKAS_ALT packer. Gates camera-side
    # faults (cam_stale ≥30 frames, FAULT_LFA bit) identically to card.py's
    # alert thresholds so we never emit ACTIVE=2 with stale/faulted input.
    cam_stale_tripped = bool(self.alert_cam_stale_frames >= 30)
    fault_lfa_bool = bool(getattr(CS, 'fault_lfa', 0))
    # Phase 5e: force STEER_REQ=0 to MDPS after a brief sustained VM
    # rejection so panda receives an explicit "do not steer" rather than
    # a frozen angle held with the active flag.
    vm_reject_persistent = self.vm_reject_consecutive_frames >= VM_REJECT_FORCE_PASSIVE_FRAMES
    # Phase 6d: update the angle-aware passive latch before computing
    # effective_lat_active. Entry needs both the wheel-angle and
    # torque thresholds; exit only the torque threshold so a driver
    # who has lifted their hands releases the wheel back to op
    # irrespective of the current wheel position. The 30-60 Nm grip
    # band holds the previous latch value — implicit hysteresis
    # without a separate frame counter. The latch is also cleared
    # whenever lat_active is False so a re-engage starts clean.
    # Phase 6e-1 adds a 5-frame (50 ms) minimum on entry: a road bump
    # or sensor blip can momentarily satisfy both thresholds, but
    # the exit-only-on-torque rule would then hold STEER_REQ=0 across
    # the driver's entire reactive-grip window even though no genuine
    # driver-active turn occurred. The counter mirrors the Phase 5e
    # vm_reject_consecutive_frames pattern.
    if not bool(CC.latActive):
      self.angle_passive_active = False
      self.angle_passive_enter_frames = 0
    elif self.angle_passive_active:
      if abs(steer_torque_safe) < CarControllerParams.ANGLE_PASSIVE_EXIT_TORQUE_NM:
        self.angle_passive_active = False
        self.angle_passive_enter_frames = 0
    else:
      # Phase 6f-3 OR-arm: low-speed sign-disagreement also arms entry
      # even when |wheel| stays below the 40° geometry gate. The driver
      # pushes ≥30 Nm in the opposite direction of op's last-frame
      # apply_angle_last (≥5° off the wheel) at ≤30 km/h — the cluster
      # signature measured on ccnc-drivelog 0x3c-0x3f. Reuses the 5-frame
      # sustain and torque-only exit.
      low_intent_disagree = (
        v_ego_safe <= CarControllerParams.INTENT_DISAGREE_VEGO_MS
        and abs(steer_torque_safe) >= CarControllerParams.INTENT_DISAGREE_TQ_MIN_NM
        and abs(self.apply_angle_last - steer_angle_safe) >= CarControllerParams.INTENT_DISAGREE_DELTA_DEG
        and (np.sign(steer_torque_safe)
             * np.sign(self.apply_angle_last - steer_angle_safe)) < 0
      )
      if ((abs(steer_angle_safe) >= CarControllerParams.ANGLE_PASSIVE_ENTER_WHEEL_DEG
           and abs(steer_torque_safe) >= CarControllerParams.ANGLE_PASSIVE_ENTER_TORQUE_NM)
          or low_intent_disagree):
        self.angle_passive_enter_frames = min(self.angle_passive_enter_frames + 1,
                                              CarControllerParams.ANGLE_PASSIVE_MIN_ENTER_FRAMES)
        if self.angle_passive_enter_frames >= CarControllerParams.ANGLE_PASSIVE_MIN_ENTER_FRAMES:
          self.angle_passive_active = True
      else:
        self.angle_passive_enter_frames = 0
    # Phase 6e-2 + Phase 6f-1: clamp apply_angle_last to the actual
    # wheel position whenever either the angle-passive latch is
    # engaged OR the driver is in heavy-override territory
    # (override_factor >= 0.9, ≈ 200+ Nm grip — well above the B1
    # full-blend point). Both cases share the same root problem:
    # the B1 blend hook sets desired_angle = wheel, but VM rate
    # limiting starts from apply_angle_last which has already
    # advanced on a stale trajectory toward op_curv, so the
    # on-vehicle apply_angle takes many frames to catch up. Drivelog
    # 0000002[23] measured heavy-override frame mismatch p95 = 31° /
    # p99 = 95° / max = 122°, and 69.8% of all STEER_REQ=1
    # sign-mismatch frames (op opposite of wheel) sat in this
    # heavy-override population — predominantly lane-change cases
    # under blinker. Anchoring apply_angle_last to the wheel each
    # such frame turns the next-frame VM step into "wheel ± rate"
    # so apply tracks the wheel within 1-2 frames of the override
    # triggering. Stateless 1-line equivalent of the historical
    # snap_to_wheel state machine (per-frame condition, no latch).
    # When the driver naturally releases (override < 0.9), the
    # clamp lifts and the standard VM rate-limited transition
    # takes over.
    # parking_mode_active is included so apply_angle_last tracks the wheel
    # while op is held passive (CC.latActive may still be True), giving a
    # bump-free resume when the latch releases above 33 km/h.
    if self.angle_passive_active or override_factor >= 0.9 or self.parking_mode_active:
      self.apply_angle_last = float(np.clip(steer_angle_safe,
                                            -self.params.ANGLE_LIMITS.STEER_ANGLE_MAX,
                                             self.params.ANGLE_LIMITS.STEER_ANGLE_MAX))
    if ccnc_lka_alt:
      # Phase 6d adds angle_passive_active to the false-reason chain
      # alongside vm_reject_persistent (Phase 5e). STEER_REQ=0 in this
      # state releases the MDPS active hold so the wheel can return on
      # the caster while the driver completes an active turn; ACIGain
      # naturally ramps to zero through the existing rate_dn curve in
      # compute_torque_reduction_gain.
      effective_lat_active = (bool(CC.latActive) and bool(apply_steer_req)
                              and not self.was_in_reverse and not in_passthrough
                              and not cam_stale_tripped and not fault_lfa_bool
                              and not vm_reject_persistent
                              and not self.angle_passive_active
                              and not self.parking_mode_active)
    else:
      effective_lat_active = bool(apply_steer_req)

    # ACIGain (reference 17-line compute_torque_reduction_gain).
    effective_aci_gain = None
    if ccnc_lka_alt and lkas_alt_cam_msg is not None:
      steering_error = self.apply_angle_last - steer_angle_safe
      effective_aci_gain = compute_torque_reduction_gain(
        steer_torque_safe, v_ego_safe * CV.MS_TO_KPH,
        effective_lat_active, self.aci_gain_last, steering_error,
        blinker_on=blinker_on,
      )
      self.aci_gain_last = effective_aci_gain

    can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, CC.enabled, effective_lat_active, apply_torque, self.lkas_icon,
                                                         apply_angle=self.apply_angle_last, lkas_alt_cam_msg=lkas_alt_cam_msg,
                                                         mads_lka_icon=mads_lka_icon,
                                                         effective_aci_gain=effective_aci_gain,
                                                         mads_force_assist=bool(mads_enabled and ccnc_lka_alt),
                                                         cam_invalid=bool(cam_stale_tripped or fault_lfa_bool),
                                                         lfa_sync_pulse=lfa_sync_pulse))

    # prevent LFA from activating on LKA steering cars by sending "no lane lines detected" to ADAS ECU
    # CCNC cars (including the HDA2-ALT + CCNC angle-control platform):
    # pass through camera's lane lines so ADAS DRV accepts LKAS_ALT
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
    # HDA2-ALT + CCNC: leave 0x161/0x162 entirely to the stock publisher
    # to avoid a dual-publisher fault on bus 1.
    if self.frame % 5 == 0 and (not lka_steering or lka_steering_long):
      if ccnc_non_hda2:
        op_driving = bool(ccnc_lka_alt and CC.latActive)
        can_sends.extend(hyundaicanfd.create_ccnc(self.packer, self.CAN, self.CP.openpilotLongitudinalControl, CC.enabled, CC.hudControl, CC.leftBlinker,
                                                  CC.rightBlinker, CS.msg_161, CS.msg_162, CS.msg_1b5, CS.is_metric, CS.out, CS.main_cruise_enabled,
                                                  self.lfa_icon, op_driving=op_driving))
      else:
        can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled, self.lfa_icon))

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
            # for ACC cancel — factory SCC natively publishes on bus 1.
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
