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

# Width of the synthetic LFA_BUTTON pulse op emits on each MADS enabled-state
# edge to keep the cluster's LFA green icon in lockstep with MADS. 2 frames at
# 100 Hz ≈ 20 ms — same envelope the stock camera uses when it ACKs a real
# wheel-button press, so the gateway's edge-detector behaves identically.
LFA_SYNC_PULSE_FRAMES = 2

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
      # 2026-05-12 (6차): drivelog 0000000f+10 showed user feedback on
      # 30+ km/h parking-entry left turn with blinker active. At 242 Nm
      # the prior map produced target=0.22 (MDPS 22% authority) — the
      # driver still felt op pulling. Bumped down active/heavy levels
      # (0.25→0.18, 0.15→0.08) and shortened bp_heavy (350-500 → 250-350)
      # so strong-grip blinker zones reach floor 0.08 within active-steer
      # p25 (250 Nm) range. Hands-off (0.80) and light-grip (0.55)
      # unchanged — assistance for actual lane changes preserved.
      bp_grip   = 30.0
      bp_active = float(np.interp(v_ego, [2., 11.], [100., 125.]))
      bp_heavy  = float(np.interp(v_ego, [2., 22.], [250., 350.]))   # 6차: 350-500 → 250-350
      target = float(np.interp(abs(steering_torque),
                                [0.0, bp_grip, bp_active, bp_heavy],
                                [0.80, 0.55,    0.18,      0.08]))   # 6차: 0.25→0.18, 0.15→0.08
    else:
      ceiling = float(np.interp(v_ego, [0.5, 1.5], [1.0, 0.85]))
      # 15th: city shelf raised (was [0.22, 0.30]) so brief grip events at
      # 20-40 km/h don't dip ACIGain as deep. drives 14-16: at city speeds,
      # ACIGain mean during light-grip drift events was 0.51 because a recent
      # 50-100 Nm grip pulled target down to shelf=0.244 (v=5 m/s), then the
      # slow rate_up=0.004/frame couldn't climb back to ceiling before the
      # next drift event. Raising shelf 0.22→0.30 at v=2 and 0.30→0.40 at
      # v=11 means moderate-grip dip floors higher, ACIGain spends less
      # time in the deep-recovery regime. Marginal driver-yield reduction
      # at moderate grip (50-100 Nm at 30 km/h: 30% MDPS authority instead
      # of 24%) is acceptable.
      shelf = float(np.interp(v_ego, [2., 11.], [0.30, 0.40]))
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
  else:
    # 15th: rate_up boost so ACIGain climbs back to ceiling quickly after
    # brief grip events. Base rate_up=0.004/frame meant a full 0→1 climb
    # took 1.25 s — during that recovery window, EPS only has the partial
    # authority of whatever gain it landed at (mean 0.51 during city drift
    # events per drives 14-16). Two boost paths:
    #   - high tracking error (op detected wheel deviation ≥ 1°): climb
    #     10x faster (0.04/frame) so op authority reaches max within
    #     ~150 ms of the error onset.
    #   - light grip (driver torque < 30 Nm, hands-off / light hold): climb
    #     5x faster (0.02/frame). This raises the baseline ACIGain BEFORE
    #     any drift event so the deep dip is avoided.
    # Combined, ACIGain mean during city drift events rises 0.51→0.91 in
    # sim (drives 14-16, n=2,232 city light-grip drift frames), translating
    # to an estimated 41% reduction in apply→wheel lag (1.68° → 0.99°).
    # 16th: replace step-function boost with smooth np.interp ramps. Patch #15
    # used hard if/elif at err=1° and tq=30 — drives 19/1a showed 1,050 frames
    # where rate_up flapped 0.004↔0.04 within 200 ms at threshold boundaries,
    # causing 10x ACIGain step jumps each toggle → MDPS pulsed wheel ("탁탁" feel).
    # Smooth interp keeps the same end-points (0.004 base, 0.04 max on error,
    # 0.02 max on light grip) but transitions continuously across the boundary,
    # so per-frame rate_up change at boundary oscillation is ~1.4 quant instead
    # of 9 quant — visibility eliminated. Bands widened to 0.5-1.5° / 60-20 Nm
    # so endpoint variations stay within the smooth zone.
    err_boost = float(np.interp(abs(steering_error), [0.5, 1.5], [0.004, 0.04]))
    tq_boost  = float(np.interp(abs(steering_torque), [20.0, 60.0], [0.02, 0.004]))
    rate_up_mag = max(rate_up_mag, err_boost, tq_boost)
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
    # Diagnostic capture for CCNC_0x161.LFA_ICON transitions, used to chase
    # the open question of which signal actually drives the cluster's LFA
    # icon. Drivelog 00000013 (PR #9 in flight) showed the icon toggling
    # between HIDDEN(0) and GRAY(1) but never reaching GREEN(2), while op's
    # synthetic LFA_BUTTON pulse on LKAS_ALT had no observable effect.
    # cloudlog the surrounding state on every transition so the next
    # drivelog narrows the search.
    self.prev_lfa_icon = -1
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
    # 12th: post-override recovery hold. After override_snapped releases
    # with the wheel still away from center (driver just finished a 60°+
    # turn and lifted off mid-recovery), apply_angle_last is snapped to
    # the wheel value at snap-exit, but the VM rate limiter then converges
    # toward the model's straight-line desired angle at ~0.02°/frame while
    # the caster recovers the wheel at 5-10°/frame. Op ends up commanding
    # an angle far behind the wheel, causing MDPS to fight the natural
    # caster recovery. Hold passthrough + snap_to_wheel until the wheel
    # returns near center or a 2-second timeout. Drivelog 14-16 audit
    # (POST_PR11_AUDIT.md) showed 8 concerning events, worst with op-vs-
    # wheel deviation reaching 122.9° during a left-turn release at
    # 18.4 kph with blinker on.
    self.post_override_recovery = False
    self.recovery_remaining_frames = 0
    # 2026-05-12 (5차): noise + transition smoothing for override path.
    # steer_torque_lpf — 30 ms LPF on STEERING_COL_TORQUE absorbs ±5 Nm
    #   CAN noise at the DEADZONE boundary (70/100 Nm) so override_factor
    #   stays stable. tau matches OVERRIDE_SNAP_ENTER_FRAMES (3 frames).
    # blinker_frac  — 300 ms LPF on the blinker boolean used to lerp
    #   DEADZONE_ANGLE / FULL_OVERRIDE_*_ANGLE between the non-blinker
    #   (100/200/350) and blinker (70/130/220) constants. Removes the
    #   step in wheel-blend at light grip (92 Nm) when the driver flips
    #   the turn signal. ACIGain blinker branch + snap gate still use
    #   the raw boolean blinker_on so op authority yields instantly.
    self.steer_torque_lpf = 0.0
    self.blinker_frac = 0.0
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
    # MADS↔cluster sync. The stock gateway ECU (publishing CCNC_0x161/0x162 on
    # bus 1) maintains the cluster's LFA green icon and toggles it on each
    # rising edge of LKAS_ALT.LFA_BUTTON it observes. Since op intercepts
    # LKAS_ALT on the ADAS-DRV-facing bus, we can drive that bit ourselves:
    # on every MADS enabled-state edge we hold LFA_BUTTON=1 for
    # `LFA_SYNC_PULSE_FRAMES` carcontroller frames (~20 ms at 100 Hz),
    # matching the camera's native press-ACK pulse width so the gateway
    # toggles its internal LFA state exactly once per MADS transition.
    self.prev_mads_enabled = False
    self.lfa_sync_pulse_remaining = 0
    # Lateral-alert flags exposed to controlsd via CarOutput. card.py reads
    # these counters and trips Bool flags at threshold; selfdrived pushes
    # the matching onroadEvents (lateralAccelLimit / steerAngleLimit /
    # cameraDataStale). Frame counters here implement N-frame hysteresis
    # so transient single-frame trips do not spam alerts.
    self.alert_vm_limit_frames = 0
    # 10 s cooldown (1000 frames @ 100 Hz) gating vmLimitTripped publish after
    # each trip. Lets users distinguish "this corner is tight" from "this
    # whole road is tight" without spamming the alert for the same ramp.
    # cloudlog warnings still fire on every trip so the data is preserved.
    self.alert_vm_limit_cooldown_frames = 0
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
    if not CC.latActive:                  reasons.append("not_latActive")
    if self.override_snapped:             reasons.append("override_snapped")
    if self.post_override_recovery:       reasons.append("post_override_recovery")
    if not apply_steer_req:                reasons.append("no_steer_req")
    if self.was_in_reverse:                reasons.append("was_in_reverse")
    if in_passthrough:                     reasons.append("in_passthrough")
    if cam_stale_tripped:                  reasons.append("cam_stale")
    if fault_lfa:                          reasons.append("fault_lfa")
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
    # 2026-05-12 (5차): 30 ms LPF on driver torque so ±5 Nm CAN noise at
    # the DEADZONE boundary (70/100 Nm) does not flap override_factor
    # between 0 and ~0.08. tau matches OVERRIDE_SNAP_ENTER_FRAMES so the
    # filter never adds more delay than the snap counter already imposes
    # on a real driver-takeover.
    steer_torque_raw = float(CS.out.steeringTorque) if np.isfinite(CS.out.steeringTorque) else 0.0
    self.steer_torque_lpf = 0.25 * steer_torque_raw + 0.75 * self.steer_torque_lpf
    steer_torque_safe = self.steer_torque_lpf
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
    # 2026-05-12 (5차): smooth threshold transition between non-blinker
    # (100/200/350) and blinker (70/130/220) constants using a 300 ms LPF
    # on the blinker boolean. 4차 used a hard if/else, producing a 1-frame
    # step in override_factor (0→0.37 at 92 Nm light grip) the moment the
    # driver flipped the signal — risky for stability if the driver intended
    # to keep the current lane. With alpha=0.032 the threshold takes ~900 ms
    # to converge, well inside any real lane-change maneuver (>2 s) but
    # smooth enough to avoid a wheel-blend cliff at signal onset.
    blinker_target = 1.0 if blinker_on else 0.0
    self.blinker_frac = 0.032 * blinker_target + 0.968 * self.blinker_frac
    if ccnc_lka_alt:
      DZ_NB = CarControllerParams.DRIVER_TORQUE_DEADZONE_ANGLE
      DZ_BL = CarControllerParams.DRIVER_TORQUE_DEADZONE_ANGLE_BLINKER
      DRIVER_TORQUE_DEADZONE = DZ_NB + (DZ_BL - DZ_NB) * self.blinker_frac

      LO_NB = CarControllerParams.DRIVER_TORQUE_FULL_OVERRIDE_LOW_V_ANGLE
      LO_BL = CarControllerParams.DRIVER_TORQUE_FULL_OVERRIDE_LOW_V_BLINKER
      override_low_v = LO_NB + (LO_BL - LO_NB) * self.blinker_frac

      HI_NB = CarControllerParams.DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V_ANGLE
      HI_BL = CarControllerParams.DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V_BLINKER
      override_high_v = HI_NB + (HI_BL - HI_NB) * self.blinker_frac
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
    # 2026-05-12 (6차): exception for very heavy grip under blinker
    # (>200 Nm LPF). User reported parking-entry left turn with hazards
    # on (both blinkers) where op kept steering the lane direction
    # against a 242 Nm grip — blend alone (apply≈wheel) still left
    # MDPS authority of 22% so EPS pushed back. Treat 200+ Nm under
    # blinker as "driver clearly not changing lanes" and let snap take
    # over; exit grace 100 ms still releases promptly when driver lets go.
    HEAVY_SNAP_OVERRIDE_TQ = 200.0
    snap_blinker_override = blinker_on and abs(steer_torque_safe) > HEAVY_SNAP_OVERRIDE_TQ
    # 16th: heavy snap mismatch guard. When driver applies heavy torque BUT
    # apply_angle_last is aligned with the actual wheel (mismatch < 10°), the
    # driver is steering in the same direction op wants — suppress the snap so
    # STEER_REQ stays high and ACIGain keeps MDPS engaged. This pairs with
    # Fix-D (recovery early-exit) to provide fast smooth takeover the moment
    # the driver releases mid-turn (user-requested: "차선변경에서 차선을 넘자마자
    # 핸들을 놓는 것과 같이 부드럽지만 빠르게 개입"). The fighting case (driver
    # cranks against op's intent, mismatch ≥ 10°) still snaps and yields fully.
    # Patch #14 moderate path already uses mismatch ≥ 20° for the same pattern.
    HEAVY_ALIGNED_MISMATCH_DEG = 10.0
    heavy_grip_aligned = abs(self.apply_angle_last - steer_angle_safe) < HEAVY_ALIGNED_MISMATCH_DEG
    heavy_override_active = (override_factor >= CarControllerParams.OVERRIDE_SNAP_ENTER_FACTOR
                             and (not blinker_on or snap_blinker_override)
                             and not heavy_grip_aligned)

    # 14th: driver-active yield. Patches #12/#13 only react after the driver
    # releases (snap-exit or release-time mismatch). User reported that
    # during a 50°+ turn made with moderate grip (30-170 Nm — above a light
    # hold but below the heavy snap threshold of 170+ Nm at low speed), op
    # still blends into the wheel via the partial-override path, producing
    # a slight stutter. Extend override_snapped entry to fire under
    # "moderate driver-active steering": high wheel angle + meaningful
    # grip + the wheel position is NOT being commanded by op (mismatch
    # ≥ 20°). The mismatch gate excludes op-driven 50°+ corners (where the
    # driver simply has hands resting) so op doesn't disengage mid-curve.
    # Stay-condition uses lower thresholds so brief torque dips during the
    # turn don't break the snap, and an exit_factor=0.1 path won't fire
    # prematurely while the driver is still actively steering (the
    # moderate-grip torque is below DEADZONE so override_factor=0 — would
    # otherwise trigger the existing exit branch).
    DRIVER_ACTIVE_STEERING_ANGLE_DEG = 50.0
    DRIVER_ACTIVE_STEERING_TORQUE_NM = 30.0
    DRIVER_ACTIVE_MISMATCH_DEG       = 20.0
    DRIVER_ACTIVE_STAY_ANGLE_DEG     = 30.0
    DRIVER_ACTIVE_STAY_TORQUE_NM     = 20.0
    moderate_entry = (abs(steer_angle_safe) >= DRIVER_ACTIVE_STEERING_ANGLE_DEG
                      and abs(steer_torque_safe) >= DRIVER_ACTIVE_STEERING_TORQUE_NM
                      and not self.override_snapped
                      and abs(self.apply_angle_last - steer_angle_safe) >= DRIVER_ACTIVE_MISMATCH_DEG)
    moderate_stay = (abs(steer_angle_safe) >= DRIVER_ACTIVE_STAY_ANGLE_DEG
                     and abs(steer_torque_safe) >= DRIVER_ACTIVE_STAY_TORQUE_NM)

    if heavy_override_active or moderate_entry:
      self.override_enter_cnt += 1
      self.override_exit_cnt = 0
    elif override_factor <= CarControllerParams.OVERRIDE_SNAP_EXIT_FACTOR and not moderate_stay:
      self.override_exit_cnt += 1
      self.override_enter_cnt = 0
    else:
      # in-between or moderate-stay holding: hold counters (don't accumulate, don't reset)
      pass
    prev_override_snapped = self.override_snapped
    if not self.override_snapped and self.override_enter_cnt >= CarControllerParams.OVERRIDE_SNAP_ENTER_FRAMES:
      self.override_snapped = True
    elif self.override_snapped and self.override_exit_cnt >= CarControllerParams.OVERRIDE_SNAP_EXIT_FRAMES:
      self.override_snapped = False

    # 12th: post-override recovery hold. Triggered on snap exit when the
    # wheel is still > RECOVERY_ENTER_ABS_DEG from center — keeps op in
    # passthrough + apply_angle_last = wheel for up to RECOVERY_TIMEOUT_FRAMES
    # or until the wheel returns within RECOVERY_EXIT_ABS_DEG of center.
    # Re-armed if the driver re-engages override mid-recovery (handles a
    # quick second turn before the first finishes recovering).
    #
    # 13th: light-grip extension. Patch #12 only fires when override_snapped
    # was engaged (i.e. heavy grip 170+ Nm). For 50°+ turns made with
    # moderate grip (no snap entry), the blend mechanism leaves apply_angle
    # at e.g. 10-15° while wheel is at 50° — the lag is smaller than the
    # snap case but still uncomfortable on release. Detect this by
    # |apply_angle_last - wheel| ≥ HANDS_OFF_MISMATCH_DEG: large mismatch
    # at high |wheel| with driver hands-off means op was NOT in control of
    # the wheel position (i.e. the driver cranked it). Op-driven 50°+
    # corners keep apply_angle_last ≈ wheel (mismatch ~5°) so this gate
    # cleanly excludes them.
    RECOVERY_ENTER_ABS_DEG      = 30.0
    # 14th: raised from 10° to 20° — at 10° the caster torque is near zero,
    # so op re-engages essentially cold-starting from a near-static wheel,
    # producing a subtle hand-off stutter. At 20° caster is still active, so
    # op picks up while the wheel is still in motion and the transition is
    # smoother. The 10° → 20° change shortens recovery by ~80 ms on the
    # drive 15 worst case sim (no functional regression).
    RECOVERY_EXIT_ABS_DEG       = 20.0
    RECOVERY_TIMEOUT_FRAMES     = 200  # 2 s at 100 Hz
    HANDS_OFF_RECOVERY_ANGLE_DEG = 50.0
    HANDS_OFF_MISMATCH_DEG       = 20.0
    if prev_override_snapped and not self.override_snapped \
       and abs(steer_angle_safe) >= RECOVERY_ENTER_ABS_DEG:
      self.post_override_recovery = True
      self.recovery_remaining_frames = RECOVERY_TIMEOUT_FRAMES
    elif not self.post_override_recovery \
         and abs(steer_angle_safe) >= HANDS_OFF_RECOVERY_ANGLE_DEG \
         and override_factor <= 0.1 \
         and abs(self.apply_angle_last - steer_angle_safe) >= HANDS_OFF_MISMATCH_DEG:
      self.post_override_recovery = True
      self.recovery_remaining_frames = RECOVERY_TIMEOUT_FRAMES
    # 16th: recovery early-exit on release + op-centering. User-requested
    # behavior: "직진이 되기 전에 운전자가 핸들을 놓으면 부드럽지만 다시 빠르게
    # 개입해야함 (차선변경에서 차선을 넘자마자 핸들을 놓는 것과 같이)" — when the
    # driver releases mid-turn and op's command is already centering (same
    # direction as wheel, smaller magnitude), exit recovery immediately so
    # MADS takes the return-to-center back from the caster. The standard
    # |wheel|<20° gate makes the user wait until the wheel is almost centered;
    # this early-exit lets MADS pick up at e.g. wheel=+25° (lane change end)
    # when the driver lets go. Direction check guards against op commanding
    # opposite to the residual wheel angle (which would yank the wheel away).
    RECOVERY_EARLY_EXIT_FACTOR_TH = 0.1
    RECOVERY_EARLY_EXIT_OP_RATIO  = 0.7
    if self.post_override_recovery:
      self.recovery_remaining_frames -= 1
      released_and_op_centering = (override_factor <= RECOVERY_EARLY_EXIT_FACTOR_TH
                                    and abs(op_curv_safe) < RECOVERY_EARLY_EXIT_OP_RATIO * abs(steer_angle_safe)
                                    and op_curv_safe * steer_angle_safe >= 0)
      if abs(steer_angle_safe) < RECOVERY_EXIT_ABS_DEG \
         or self.recovery_remaining_frames <= 0 \
         or released_and_op_centering:
        self.post_override_recovery = False
        self.recovery_remaining_frames = 0

    if not CC.latActive:
      # disengaged: reset so next engage doesn't inherit stale state
      self.override_snapped = False
      self.override_enter_cnt = 0
      self.override_exit_cnt = 0
      self.post_override_recovery = False
      self.recovery_remaining_frames = 0
      self.aci_gain_last = 0.0
      # 5차: also reset 5차 smoothing states so next engage starts from
      # the actual driver torque / blinker boolean, not whatever residue
      # accumulated while disengaged.
      self.steer_torque_lpf = 0.0
      self.blinker_frac = 0.0

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
    #
    # Hands-off bypass: the passthrough only emulates stock LFA's
    # parking-lot self-centering bias, which only matters when the
    # driver has hands on the wheel. If the driver has lifted off
    # (steeringPressed=False), they are by definition not parking — they
    # are in stop-and-go or slow flow — and op should hold the lane.
    # `steeringPressed` is the carstate hysteresis flag, so light contact
    # without applied torque still trips it and preserves the parking feel.
    #
    # 11th: i6n's STEER_THRESHOLD=350 Nm leaves a 100-350 Nm band where the
    # driver IS actively steering (parking grip p90=184 Nm, hard p50=381 Nm
    # per values.py:159-160) but `steeringPressed` is still False, so the
    # PR #10 bypass wrongly classified these frames as hands-off. Require
    # override_factor to also agree (≤0.5, ~140 Nm at low speed given the
    # 100/180 deadzone/full curve) before declaring hands-off. Drivelog
    # 99b215d21bbf8735_00000013 sim (tools/ioniq6n_patch11_sim.py): of 47k
    # frames, 17,863 had steeringPressed=False but override_factor=1.0
    # (full driver override); restoring those to the latch saves 10,099
    # active-grip low-speed frames from op contention.
    hands_off = (not CS.out.steeringPressed) and (override_factor <= 0.5)
    if CS.out.vEgoRaw < LOW_SPEED_PASSTHROUGH_ENTER_MS and not hands_off:
      self.low_speed_cam_latched = True
    elif CS.out.vEgoRaw > LOW_SPEED_PASSTHROUGH_EXIT_MS or hands_off:
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
        # but turns into sluggish path tracking at highway. Cap tau at 25 m/s
        # so psi-error corrections happen promptly.
        # 2026-05-12 (6차): highway cap 0.15→0.22 — drivelog 0000000f+10
        # showed apply Δ p90 = 0.22°/frame on straights, op-side jitter
        # passing through. 0.22 s tau absorbs 50 Hz noise while keeping
        # psi corrections within 1 tau (~220 ms) — still well below the
        # ~2 s planner lane-change window.
        speed_max_tau = float(np.interp(v_ego_safe, [10.0, 25.0], [2.5, 0.22]))
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
      elif self.post_override_recovery:
        self._snap_apply_angle_to_wheel(steer_angle_safe, "post_override_recovery")

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
        # 2026-05-12 (7차): self-aligning protection during recentering.
        # When the wheel is well off-center, the driver has gone light
        # (releasing self-aligning torque), AND op's desired angle is
        # smaller in magnitude than the wheel, never let apply_angle pull
        # the wheel further from center. Drivelog 0000000f+10 confirmed
        # the user-reported >70° turn + release scenario is already
        # handled by snap entry (|apply-wheel|=0.29° p50 under strong
        # grip), but this guard catches any short transient where op
        # could still lead the wheel after snap exit. Conditions are
        # conservative - in drivelog only ~1% of light-grip frames meet
        # all four (large turn + light grip + op wants smaller + apply
        # outside), so normal op corner tracking is unaffected. Snap and
        # blend behavior in all other regimes is preserved.
        if apply_angle is not None and abs(steer_angle_safe) > 15.0 and \
           abs(steer_torque_safe) < 80.0:
          sign_w = float(np.sign(steer_angle_safe))
          op_wants_smaller = sign_w * op_curv_safe < sign_w * steer_angle_safe - 5.0
          apply_outside_wheel = sign_w * apply_angle > sign_w * steer_angle_safe
          if op_wants_smaller and apply_outside_wheel:
            apply_angle = steer_angle_safe
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
          # Rising edge of vmLimitTripped (frames just crossed the 50-frame
          # publish threshold and no cooldown active): start a 10 s window
          # during which card.py keeps vmLimitTripped=False even if the
          # frame counter is still above the threshold. Lets users tell
          # apart "this corner" vs "this whole road" without alert spam.
          if self.alert_vm_limit_frames == 50 and self.alert_vm_limit_cooldown_frames == 0:
            self.alert_vm_limit_cooldown_frames = 1000
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

    # MADS↔cluster sync pulse. Any MADS enabled-state edge — whether the
    # source is a stock LFA wheel-button press routed through
    # carstate.lfa_button_oem, an auto-attach with ACC, or a manual cancel —
    # rearms a short LFA_BUTTON pulse on op's outgoing LKAS_ALT. The stock
    # gateway interprets this as a button press and toggles the cluster icon
    # exactly once, so the green light tracks MADS state regardless of how the
    # transition was triggered. Gated to ccnc_lka_alt because non-HDA2-ALT
    # platforms don't share this gateway path.
    if ccnc_lka_alt and mads_enabled != self.prev_mads_enabled:
      self.lfa_sync_pulse_remaining = LFA_SYNC_PULSE_FRAMES
    self.prev_mads_enabled = mads_enabled
    lfa_sync_pulse = self.lfa_sync_pulse_remaining > 0
    if self.lfa_sync_pulse_remaining > 0:
      self.lfa_sync_pulse_remaining -= 1

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

    # Diagnostic: log CCNC_0x161.LFA_ICON transitions. Drivelog 00000013
    # showed the icon never reaching GREEN(2) on PR #9, only HIDDEN(0) /
    # GRAY(1) — the cluster ignores op's LKAS_ALT.LFA_BUTTON pulses, so
    # whatever drives this icon lives elsewhere. Capture the surrounding
    # state on each edge so the next drivelog can pinpoint the signal.
    if ccnc_lka_alt:
      lfa_icon = int(getattr(CS, 'msg_161', {}).get('LFA_ICON', 0)) if getattr(CS, 'msg_161', None) else 0
      if lfa_icon != self.prev_lfa_icon and self.prev_lfa_icon != -1:
        cam_lfa_btn = int(lkas_alt_cam_msg.get('LFA_BUTTON', 0)) if lkas_alt_cam_msg is not None else 0
        cloudlog.warning(
          f"LFA_ICON transition: {self.prev_lfa_icon} -> {lfa_icon} "
          f"mads={bool(mads_enabled)} cruise_en={CS.out.cruiseState.enabled} "
          f"cruise_avail={CS.out.cruiseState.available} cam_lfa_btn={cam_lfa_btn} "
          f"lat_active={CC.latActive} v_kph={CS.out.vEgoRaw*3.6:.1f} "
          f"in_passthrough={in_passthrough} override_snapped={self.override_snapped} "
          f"aci_active={self.aci_active_latched}"
        )
      self.prev_lfa_icon = lfa_icon

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
                                                         cam_invalid=bool(cam_stale_tripped or fault_lfa_bool),
                                                         lfa_sync_pulse=lfa_sync_pulse))

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
