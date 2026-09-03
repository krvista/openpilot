import math
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
LOW_SPEED_PASSTHROUGH_EXIT_MS  = 25.0 / 3.6   # Phase 24b: 22 -> 25 (경계 배회 차단; 0x3c-3d TX 발진 2건이 정확히 22km/h 경계)

# Traffic-following lead distance hysteresis. When a lead is closer than
# this, op stays engaged below the freeze threshold so the wheel is held
# (stop-and-go traffic).
# Phase 13a: widened 3/5 -> 8/12 m. The 3 m gate only recognized bumper-to-
# bumper standstill; queue crawling at 10-20 km/h runs 5-12 m gaps, exactly
# the regime the low-speed scenario gate should keep steering in.
TRAFFIC_FOLLOW_NEAR_M = 8.0
TRAFFIC_FOLLOW_FAR_M  = 12.0

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
# Phase 14-4 S3': lead-less lot-crawl signature. Window must stay under
# CREEP_MAX (25 km/h — lot aisles spike to 20-22) with no lead in the
# traffic-follow window, for CREEP_FRAMES, AND show the lot pattern: a dip
# below CREEP_DIP or a window mean under CREEP_MEAN. Steady 20-25 km/h
# surface-street cruising (school zones, alleys) has no dip and a high mean,
# so it does not arm.
PARKING_CREEP_MAX_MS   = 25.0 / 3.6
PARKING_CREEP_DIP_MS   = 8.0 / 3.6
PARKING_CREEP_MEAN_MS  = 12.0 / 3.6
PARKING_CREEP_FRAMES   = 1000                    # 10 s @ 100 Hz



def compute_hold_torque(v_ego, lat_acc):
  """Phase 31 hold-torque model (values.py ACIGAIN_HOLD_*): op's own
  torsion-bar contribution while actively steering, comp(v, la) =
  B(v) + G(v) * S(la). Pure function so the model is unit-testable in
  isolation (review 31b) — the kill switch (BASE_V/LAGAIN_V all-zero) and
  the cap headroom are pinned in phase_tests/test_driver_domain.py."""
  base = float(np.interp(v_ego, CarControllerParams.ACIGAIN_HOLD_BASE_SPEEDS_MS,
                         CarControllerParams.ACIGAIN_HOLD_BASE_V))
  gain = float(np.interp(v_ego, CarControllerParams.ACIGAIN_HOLD_BASE_SPEEDS_MS,
                         CarControllerParams.ACIGAIN_HOLD_LAGAIN_V))
  shape = float(np.interp(lat_acc, CarControllerParams.ACIGAIN_HOLD_LA_BP,
                          CarControllerParams.ACIGAIN_HOLD_LA_S))
  return min(base + gain * shape, CarControllerParams.ACIGAIN_HOLD_MAX_NM)


def compute_torque_reduction_gain(steering_torque, v_ego_kph, lat_active, last_gain, steering_error, blinker_on=False,
                                  grip_start=30.0, grip_full=140.0, grip_floor=0.15, suppress_error_boost=False,
                                  post_grip=False, rate_dn_floor=0.0, anchored_recovery=False, curve_deg=0.0, rate_up_cap=0.04):
  # Phase 22: the yield input is now DRIVER torque (caller subtracts the
  # op holding-torque baseline — see the call site). Parked-car measurement
  # proved the long-assumed "+90..180 Nm sensor offset" was actually the
  # MDPS's own holding effort read back through the torsion bar (parked
  # |tq| p99 = 3 Nm), so the raw-torque curve was self-yielding in curves
  # (hold p50 223 Nm ate 40-60% of authority hands-off). Driver-domain
  # breakpoints 30->140 (hands-off) / values.py 110 (pressed) are replay-
  # calibrated to keep 21b's felt resist on GENUINE driver input (+/-0%)
  # while restoring 100% hands-off curve authority (was 56%).
  # Kill: restore 90/300/0.15 here, 220 in values, and ACIGAIN_HOLD_BASE_V/
  # LAGAIN_V all-zero — all together (the domains are coupled).
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
    # Phase 6h-4: lower the hands-off MDPS authority ceiling toward the stock
    # operating point. Stage 0 (1.24M frames): the factory camera ALWAYS runs
    # ACIGain == 0 — low authority is this MDPS's normal regime ("3-10x more
    # accurate" claim retired as tautological, but the gain==0 operating point
    # stands). Lower stiffness = less command-dither/road-texture transmission.
    # Combined-build decision (X3): deployed at the HALF-STEP rung because this
    # build also changes command shaping (6h-1/2) and a tracking regression
    # could not be attributed at the full step. Ladder:
    # [0.4,0.62,0.85,1.0] (pre / kill switch) -> [0.32,0.5,0.75,0.95] (this) ->
    # [0.25,0.35,0.65,0.9] (full step, only after the W4 on-road gate passes).
    # Phase 19a: low-speed ceiling lowered to the 6h-4 ladder's full-step
    # LOW-END ([0.32,0.5] -> [0.25,0.40]); Phase 24a: one more step to
    # [0.18, 0.30] — 0x3c-0x3d residual low-speed shake sits in 20-30 km/h
    # windows where the COMMAND is quiet (|cmd|<4°, cmdRMS~0.1) but the wheel
    # shakes above TX: road-MDPS interaction amplified by hold authority, so
    # transmission scales with this ceiling. Doubles as the requested softer
    # low-speed hold. The 40/120 km/h points stay: highway authority carries
    # the tracking/trim work. Kill: [0.25,0.40,...] (24a) / [0.32,0.5,...] (19a).
    # Phase 32 (0x4a-0x4b): low-speed 0.8-2.5 Hz shake returned (+47% p50)
    # after the Phase 31 phantom-yield removal. Review-corrected mechanism
    # accounting: mean low-speed authority rose only +10% (0.251 -> 0.275;
    # at-ceiling fraction 52 -> 67%), transmission wheel/TX +17%, source
    # churn ~unchanged — which explains roughly HALF the RMS rise; the
    # remainder is an OPEN item (see values.py CMD_HYSTERESIS note). The
    # Phase 33b (0x4f/0x50): the reserve ceiling step was deployed and then
    # ROLLED BACK by its own kill criterion. Measured on-road: shake effect
    # ZERO (p50 0.302 -> 0.308) while low-speed curve-following collapsed
    # 0.73 -> 0.41 wheel/plan — the pre-agreed tracking-degradation trigger.
    # The zero shake effect closed the residual-shake investigation
    # (review-measured, CLOSED not open): the 0.30-vs-0.242 aggregate gap
    # vs the 29b baseline is a COMPOSITION ARTIFACT — shake per unit of
    # wheel motion is IDENTICAL across builds (0.346 vs 0.346, matched
    # plan-band), and the baseline's advantage exists only while its
    # dead-quiet windows (phantom deep-yield had op effectively off,
    # driver_tq>140 on 17.6% of low-speed frames) are included: gate the
    # windows on wheelStd>=0.4 and the gap INVERTS (base 0.352 vs 0.328).
    # Lesson: the aggregate quiet-window shake metric cannot distinguish
    # "less shake" from "less steering" — normalize by motion before it
    # drives another change. 24a ladder restored:
    base_ceiling = np.interp(v_ego_kph, [0, 20, 40, 120], [0.18, 0.30, 0.75, 0.95])
    # Phase 36: continuous curve-conditional raise at low speed (see
    # values.py ACIGAIN_CURVE_*). curve_deg is the caller's fast-rise /
    # slow-fall EMA of |commanded angle|; w ramps 0 -> 1 over 3 -> 12 deg,
    # no threshold.
    curve_ceiling = np.interp(v_ego_kph, CarControllerParams.ACIGAIN_CURVE_CEILING_SPEEDS_KPH,
                              CarControllerParams.ACIGAIN_CURVE_CEILING_V)
    curve_w = np.interp(curve_deg, CarControllerParams.ACIGAIN_CURVE_RAMP_DEG, [0.0, 1.0])
    base_ceiling = base_ceiling + curve_w * (curve_ceiling - base_ceiling)
    # Error-based boost reduction gain: at 0 kph, ignore errors under 1.25°.
    error_start = np.interp(v_ego_kph, [0, 20, 40, 120], [1.25, 0.5, 0.3, 0.2])
    error_mult_raw = np.interp(abs(steering_error), [error_start, error_start*2], [1.0, 2])
    # Phase 19b: the drift-recovery boost is a low-speed noise AMPLIFIER —
    # creep-band command oscillation opens |steering_error| past error_start
    # (measured firing 43-53% of hands-off low-speed time on 0x36-0x37), so
    # the boost doubled MDPS authority exactly when the command was noisiest.
    # Gate it out below 15 km/h, full strength again from 25 km/h; highway
    # drift recovery (its actual purpose) is untouched.
    # Kill switch: [0.0, 0.0] speeds -> boost always full.
    boost_speed_gate = np.interp(v_ego_kph, [15.0, 25.0], [0.0, 1.0])
    error_mult_raw = 1.0 + (error_mult_raw - 1.0) * boost_speed_gate
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
    torque_suppress = np.interp(abs(steering_torque), [30, 110], [1.0, 0.0])  # Phase 22: driver-domain
    # Phase 9: in yield-by-authority mode the command no longer tracks the wheel,
    # so steering_error reflects the driver's own divergence during grip; boosting
    # MDPS back to op's angle would then FIGHT the driver. The caller sets
    # suppress_error_boost on real grip (debounced steeringPressed) to disable the
    # boost — hands-off drift recovery (the boost's purpose) is kept because the
    # caller leaves it enabled whenever the driver is not pressing.
    # Phase 29: taper the boost away at delivery-scale errors, but ONLY in
    # the post-grip regime (caller passes post_grip = anchor_recent > 0).
    # Review-measured hands-off drift |apply-wheel| is NOT small — p50 2.22 /
    # p75 3.83 / p90 6.87° (dominated by the sustained-curve MDPS delivery
    # deficit) — so a magnitude-only taper killed the boost on 42.5% of
    # hands-off >=25 km/h frames, regressing exactly the curve-authority
    # work of Phases 12a/22/23/25. Grip-evidence gating (recent pressed
    # anchor OR armed episode memory) cuts the taper exposure to 24.3%
    # (anchor-only would be 10.3% but misses the sub-anchor 150-300 Nm
    # release class) while preserving the field-event catch (t=3.05
    # release had anchor_recent = 103).
    big_err_taper = np.interp(abs(steering_error), [2.5, 4.0], [1.0, 0.0]) if post_grip else 1.0
    error_mult = 1.0 if suppress_error_boost else (1.0 + (error_mult_raw - 1.0) * torque_suppress * big_err_taper)
    dynamic_ceiling = min(1.0, base_ceiling * error_mult)
    # Phase 5d A2 (commit 4a4d29b): when the driver signals intent with
    # the blinker, force the MDPS ceiling down so a light-grip lane
    # change does not have to fight op torque. 0.45 mirrors the
    # sunnypilot 18d75ca 4-level descent at the moderate point of the
    # original curve. Combined with the Phase 5b (B2) blinker driver-
    # torque deadzone shift (70/130/220), this gives both command-side
    # (B1 blend on lowered override) and authority-side yield.
    if blinker_on:
      # Phase 10b: lowered from 0.45 -> 0.28. ccnc-drivelog 0x07-0x0a measured the
      # driver applying ~2.4x torque during blinker lane changes; holding 45% MDPS
      # authority made them fight op.
      # Phase 10c review: 10b's flat 0.28 also stripped authority when the blinker
      # was on but the driver was NOT pressing (signaled lane-keep / model-initiated
      # lane change), leaving only 28% MDPS follow-through with nobody at the wheel.
      # Gate the drop on driver torque instead: hands-off keeps the pre-10b 0.45,
      # and the ceiling tapers to 0.28 as real force comes in, so the yield the
      # 10b data asked for still happens exactly when the driver acts.
      # Phase 12c: the 10c breakpoints ([45, 100] — the blinker deadzone/override
      # band) sit below the +90..180 Nm column-torque offset, so hands-off already
      # measured >=100 Nm and the intended 0.45 never applied (0x10-0x28: hands-off
      # blinker gain p50 = 0.260). Taper over the offset-clearing band instead —
      # see ACIGAIN_BLINKER_GATE_* in values.py for the sim numbers.
      # Kill switches: [0.45, 0.45] = pre-10b flat; [0.28, 0.28] = 10b flat.
      ceiling_blinker = float(np.interp(
        abs(steering_torque),
        [CarControllerParams.ACIGAIN_BLINKER_GATE_START_NM,
         CarControllerParams.ACIGAIN_BLINKER_GATE_FULL_NM],
        [0.45, 0.28]))
      dynamic_ceiling = min(dynamic_ceiling, ceiling_blinker)
    # Phase 9: authority reduction band is parameterized. Under real grip the
    # caller passes [100,260]->[ceiling,0.10] so authority drops harder to absorb
    # the removed command-blend's yield; hands-off keeps the legacy band
    # [100,350]->[ceiling,0.19].
    target = np.interp(abs(steering_torque), [grip_start, grip_full], [dynamic_ceiling, grip_floor])
  else:
    target = 0.0
  delta = target - last_gain
  rate_dn = np.interp(abs(steering_torque), [0, 300, 700], [0.004, 0.01, 0.04])
  # Phase 35a: at speed a real push must drop authority promptly — the
  # caller passes a speed-scheduled floor (0.03/frame from 60 km/h), already
  # gated on real grip evidence (driver_pressed OR driver_tq >= GATE_NM 160),
  # so a resting hand keeps the slow curve (verification round: a gate at the
  # arm level 100 ratcheted hands-off authority down 8%).
  if rate_dn_floor > 0.0:
    rate_dn = max(rate_dn, rate_dn_floor)
  # Phase 5c B3 (commit 41a16ad): when |steering_error| > 0.5°, climb up
  # to 10× faster so ACIGain recovers from a brief grip event within
  # ~250 ms instead of 2.5 s. Below 0.5° rate_up matches the sunnypilot
  # reference 0.004 — no behaviour change on the steady-state path. The
  # 0.04 cap matches the legacy err_boost shape verified against
  # drivelog 0000001f (drift event ACIGain mean 0.977 with the boost
  # vs 0.583 with only the reference rate_up).
  # Phase 29 (scenario-sweep finding): non-monotonic — the 10x
  # fast recovery is for DRIFT-scale errors (0.5-2.5°); at delivery-scale
  # errors (2.5°+ = a stored driver-made divergence being released, or a
  # road kick) recovering authority fast slams the wheel toward the stale
  # command. Back off to sub-reference rate there: converge first (wheel
  # glides to the command at low authority), then recover.
  # Phase 29: in the post-grip regime only (anchor_recent > 0), fast
  # recovery is confined below the leash boundary (2.0°) — beyond it the
  # rate drops to the reference 0.004, exactly one gain quantum, NOT lower
  # (sub-quantum rates round away against the 0.004 quantizer and freeze
  # the gain outright — a re-engagement deadlock). Outside the post-grip
  # regime the legacy drift-recovery shape is kept unchanged: hands-off
  # |apply-wheel| sits at p50 2.22° (deficit curves), so a magnitude-only
  # taper cut the mean hands-off recovery rate to 28% of legacy — rejected
  # in review.
  if post_grip:
    # Phase 35b: when apply was recently pinned to the wheel (anchor /
    # one-shot) the >2 deg region is a fresh, VM-rate-limited chase of the
    # live plan, not a stale command — recover 3x faster there (see
    # values.py ANCHORED_RECOVERY_*; Hannam bridge handover case).
    tail = CarControllerParams.ANCHORED_RECOVERY_RATE_UP if anchored_recovery else 0.004
    rate_up = float(np.interp(abs(steering_error), [0.5, 1.5, 2.0], [0.004, 0.04, tail]))
  else:
    rate_up = float(np.interp(abs(steering_error), [0.5, 1.5], [0.004, 0.04]))
  rate_up = min(rate_up, rate_up_cap)   # Phase 37a: speed/rain-tapered rise cap
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
    # Phase 12a (P1): steady-curve TX trim state.
    self.curve_trim = 0.0
    self.curve_trim_sustain = 0
    # Phase 13b: LP state of the trim residual (raw wheel noise must not
    # reach the trim integrator).
    self.trim_resid_lp = 0.0
    # Phase 12b (P2): desired-angle hysteresis (backlash) filter state.
    self.cmd_hyst = 0.0
    # Phase 13a: low-speed scenario gate state.
    self.in_low_speed_zone = False
    # Phase 18: creep-zone latch (10/12 km/h hysteresis).
    self.in_creep_zone = False
    self.low_speed_scen_ok = True
    self.lowv_scen_dwell = 0
    self.lowv_release_frames = 0
    # Phase 14-2: stateful blinker anchor.
    self.blinker_anchor_on = False
    self.blinker_anchor_fire = 0
    self.blinker_anchor_hold = 0
    self.blinker_anchor_release = 0
    # Phase 14-4 S3': lead-less lot-crawl window.
    self.creep_frames = 0
    self.creep_min = float('inf')
    self.creep_sum = 0.0
    # Phase 14-1: angle_passive redesigned entry/exit counters.
    self.intent_disagree_frames = 0
    self.angle_passive_release_frames = 0
    # Diagnostic: log CCNC_0x161.LFA_ICON transitions.
    self.prev_lfa_icon = -1
    # CCNC angle-control vehicle models.
    # VM uses the on-device i6n CP. BASELINE_VM uses KIA_SPORTAGE_HEV_2026
    # CarSpecs as a second, safety-baseline check after the i6n VM in
    # apply_steer_angle_limits_vm — both must accept the angle. Mirrors the
    # reference sunnypilot implementation and panda's hardcoded safety params.
    # R3 (2026-06-10 review): the specs are NOT identical — Sportage HEV 2026
    # (mass 1812, wb 2.756, sR 13.7) vs Ioniq 6 N (2175, 2.965, 14.96). For the
    # same lateral-jerk budget the baseline converts to a ~15% LOWER angle rate
    # (e.g. 77.7 vs 91.2 deg/s at 10 m/s), so the dual check's effective limit
    # is the stricter Sportage one. Accepted: strictly conservative, mirrors
    # panda's hardcoded baseline. If corner-entry rate budget ever needs the
    # full i6n envelope, revisit ANGLE_SAFETY_BASELINE_MODEL together with the
    # panda safety params (firmware) — not independently here.
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
    # Phase 26: hold-compensated driver-torque domain state (see the block at
    # the top of update()). hold_comp starts at the straight-line baseline so
    # the first frames after init behave like a straight, not like zero
    # compensation.
    # Starts at 0: op is not actuating at init, so the bar reading (if any)
    # is entirely the driver's.
    self.hold_comp_last = 0.0
    # Phase 34b: still-straight crawl gate state (rate hysteresis 10/14).
    self.crawl_still_on = False
    self.driver_pressed_cnt = 0
    self.driver_pressed = False
    # Phase 26b (review fix): previous frame's effective_lat_active — the
    # hold compensation only applies while op was actually actuating.
    self.prev_eff_lat_active = False
    # Phase 33: BSM occupancy holds (debounce radar flicker, 1 s).
    self.blind_left_hold = 0
    self.blind_right_hold = 0
    self.blind_caution_on = False
    # Phase 37a rain mode: wiper debounce counters and the ramped weight 0..1
    self.wiper_on_frames = 0
    self.wiper_off_frames = 0
    self.rain_active = False
    self.rain_w = 0.0
    # Phase 28 (0x41 yank fix): override-episode memory + anchor-recency +
    # re-arm edge for the release re-anchor / boost hold-off.
    self.reanchor_arm = 0
    # Phase 35b: frames since apply was last pinned to the wheel
    self.frames_since_apply_anchor = 10**6
    # Phase 36: asymmetric EMA (rise 0.15 s / fall 0.5 s) of |commanded angle|
    # driving the curve ceiling ramp
    self.curve_meas_lp = 0.0
    self.reanchor_ready = False
    self.anchor_recent_frames = 0
    # Phase 30: release-edge timer for the one-shot (init large: no edge yet).
    self.frames_since_drv_release = 1000
    # Phase 27: display-only flag mirrored into carStateSP.lateralControlPaused
    # by card.py — True while op is intentionally passive (parking mode /
    # low-speed passthrough / angle-passive) with CC.latActive still set.
    self.lat_passive_indicated = False

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
    # with debounced steeringPressed (Phase 14-1); cleared when the grip releases
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
    # Phase 14-4 S1b: cold-start-at-low-speed departure signature (decided once).
    self.boot_parking_pending = True

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

    # Phase 26: hold-compensated driver-torque domain, computed ONCE here and
    # consumed by every latch/grip test below (the ACIGain caller reuses it).
    # Phase 22 established that the torsion-bar reading while op steers is
    # mostly op's OWN holding torque (Phase 31 model: compute_hold_torque —
    # speed-dependent baseline + saturating lat_acc term; parked
    # |tq| p99 = 3 Nm). The yield curve was converted then, but the latches
    # stayed raw-domain and could self-trigger on the hold torque in curves:
    # routes 0x36-0x3d measure hands-off |tq| >= 220 Nm on ~50% of
    # 0.3-1.2 m/s2 frames, and 41% of all pressed frames sit where the hold
    # model predicts >= 300 Nm — i.e. op could read its own effort as driver
    # grip and relinquish control mid-curve (blinker-anchor self-fire: 464
    # candidate episodes; angle-passive geo-entry: 202). driver_tq subtracts
    # the predicted hold so thresholds see the DRIVER's contribution only;
    # straight-line raw equivalences are speed-dependent since Phase 31
    # (comp ~140 at 20 km/h down to ~62 at highway — see values.py).
    # Known blind spot (pre-existing — shared with the raw domain and the EPS
    # pressed flag): a counter-push that cancels the hold torque reads near
    # zero on the torsion bar until it exceeds ~2x the hold level.
    # Phase 26b (review fix): the hold model describes op's OWN torque, which
    # exists only while op actuates. In the passive states (angle-passive /
    # passthrough / parking / faults) STEER_REQ=0 and ACIGain has decayed to
    # zero — the entire bar reading is the driver's, and subtracting the
    # full compensation there under-read a holding driver: the passive-exit
    # tests could release against a 300-370 Nm grip and re-engage op on a
    # wheel the driver was holding (harness measured a ~1.2 Hz STEER_REQ
    # limit cycle at a steady 360 Nm). Gate on the PREVIOUS frame's
    # effective_lat_active; the 4 Nm/frame slew smooths both edges.
    # Phase 31b (review): computation hoisted inside the gate — it was
    # evaluated and discarded on every passive frame.
    if self.prev_eff_lat_active:
      lat_acc_est = (v_ego_safe ** 2) * abs(math.tan(math.radians(
        steer_angle_safe / max(self.CP.steerRatio, 1.0)))) / max(self.CP.wheelbase, 1.0)
      # Phase 31 refit (see values.py and compute_hold_torque).
      hold_target = compute_hold_torque(v_ego_safe, lat_acc_est)
      # Phase 34: still-straight crawl override (see values.py CRAWL_STILL_*).
      # Below 3 m/s the fitted model extrapolates B=140, but on the domain
      # where this comp actually applies — traffic-following stop-and-go
      # creep, measured eff-active on 0x51-0x54 — a centred, non-moving
      # wheel reads only p45=28 Nm; the 140 belongs to the dry-friction
      # MOVING/turned states (117-198, well matched). The override value
      # (70, raw pressed equiv 300) sits deliberately above the 28 fit
      # point to absorb the light-resting-hand tail — see values.py for
      # the measured trade table. Gate on measured
      # angle AND rate so the override drops out the moment op (or the
      # driver) starts turning; the gate only lowers comp, so a gripping
      # driver is seen sooner, never later.
      steer_rate_safe = (float(CS.out.steeringRateDeg)
                         if np.isfinite(CS.out.steeringRateDeg) else 0.0)
      # Rate hysteresis (enter <10 / exit >14 deg/s): a single threshold on
      # a 10 deg/s-quantized rate signal measured p50 4.2 toggles/s in-domain
      # with dwells long enough for full 140<->70 comp swings = a new 2-4 Hz
      # authority modulation in the cleaned shake band. See values.py.
      rate_thr = (CarControllerParams.CRAWL_STILL_RATE_EXIT_DPS if self.crawl_still_on
                  else CarControllerParams.CRAWL_STILL_RATE_DPS)
      self.crawl_still_on = (v_ego_safe < CarControllerParams.CRAWL_STILL_SPEED_MS
                             and abs(steer_angle_safe) < CarControllerParams.CRAWL_STILL_ANG_DEG
                             and abs(steer_rate_safe) < rate_thr)
      if self.crawl_still_on:
        hold_target = CarControllerParams.ACIGAIN_CRAWL_STILL_COMP_NM
    else:
      self.crawl_still_on = False
      hold_target = 0.0
    # Slew guard: a single-frame angle/speed sensor spike would otherwise jump
    # hold_comp by up to +142 Nm and mask a real driver input for that window.
    # Real curve entries measure ~1 Nm/frame; 4 Nm/frame passes them cleanly.
    self.hold_comp_last += float(np.clip(hold_target - self.hold_comp_last,
                                         -CarControllerParams.ACIGAIN_HOLD_SLEW_NM,
                                          CarControllerParams.ACIGAIN_HOLD_SLEW_NM))
    hold_comp = self.hold_comp_last
    driver_tq = max(0.0, abs(steer_torque_safe) - hold_comp)
    # driver_pressed: hold-compensated equivalent of CS.out.steeringPressed
    # (raw 350 enter / 280 exit hysteresis, 5-frame up/down counter — see
    # carstate.py R4). Phase 31: 230 driver-domain (raw-equivalent shifts
    # with the speed-dependent baseline, e.g. ~303 at 54 km/h / ~370 at
    # 20 km/h comp-on); in a curve the raw requirement rises with the hold
    # estimate instead of the flag self-triggering on op's own effort. The
    # EPS flag itself is left untouched for core override/alert semantics.
    # Phase 31b (review): in comp-OFF frames driver_tq == raw, so the 230
    # threshold degraded to a raw 230/184 test — inside the measured
    # rolling-passive hands-off band (p50 147-233, sustained floor 158),
    # worsening passive self-latch/sticky-release by 20/16 Nm. Use the
    # raw-calibrated 250 (exit 200, the weeks-validated point) whenever the
    # compensation is gated off; 230 applies only where it was fitted.
    pressed_base = (CarControllerParams.DRIVER_PRESSED_NM if self.prev_eff_lat_active
                    else CarControllerParams.DRIVER_PRESSED_RAW_NM)
    pressed_thr = pressed_base * (0.8 if self.driver_pressed else 1.0)
    self.driver_pressed_cnt = int(np.clip(self.driver_pressed_cnt + (1 if driver_tq > pressed_thr else -1),
                                          0, 2 * CarControllerParams.DRIVER_PRESSED_FRAMES + 1))
    self.driver_pressed = self.driver_pressed_cnt > CarControllerParams.DRIVER_PRESSED_FRAMES
    # Phase 28 (0x41 yank fix, see values.py): override-episode memory used
    # by the release re-anchor. NOTE the arm level (100) is hands-off-
    # reachable by design necessity (hands-off driver_tq p75 = 111). The
    # one-shot needs a recent REAL anchor episode (anchor_recent) and a
    # stored divergence on top of the arm; the arm ALONE does feed the
    # Phase 29 post_grip recovery taper (see the ACIGain call). Measured on
    # 0x58/0x5b (Phase 34 build, hands-off >= 6 m/s): arm>=30 covers 27.6%
    # of hands-off frames, but a counterfactual replay raising the arm level
    # to 150/180 left the "diverged >= 3 deg with gain < 0.5" exposure
    # unchanged (18.3 -> 18.0 -> 18.0%) — that exposure is the yield curve
    # on resting hands (driver_tq 30-140), not the taper, so the level stays.
    self.reanchor_arm = (min(self.reanchor_arm + 1, CarControllerParams.REANCHOR_ARM_CAP_FRAMES)
                         if driver_tq >= CarControllerParams.REANCHOR_ARM_NM
                         else max(self.reanchor_arm - 1, 0))
    self.frames_since_apply_anchor = min(self.frames_since_apply_anchor + 1, 10**6)
    # Phase 36: curve measure — asymmetric EMA of |commanded angle| (plan,
    # not apply_angle_last, so anchors cannot contaminate it). Fast rise so
    # the raise reaches the curve entry (plan ramps 3 -> 8 deg in ~0.3 s),
    # slow fall so the exit stays clean; runs unconditionally.
    _c_in = abs(op_curv_safe)
    _c_tau = (CarControllerParams.ACIGAIN_CURVE_MEAS_TAU_RISE_S if _c_in > self.curve_meas_lp
              else CarControllerParams.ACIGAIN_CURVE_MEAS_TAU_FALL_S)
    _c_step = (DT_CTRL / (_c_tau + DT_CTRL)) * (_c_in - self.curve_meas_lp)
    # rise slew cap: a bound on the ceiling slew for plan steps faster than
    # anything on the corpus (fall is already slower than the gain's rate_dn)
    self.curve_meas_lp += min(_c_step, CarControllerParams.ACIGAIN_CURVE_MEAS_MAX_RISE_DPS * DT_CTRL)
    # G2 review: re-arm EDGE — a fire consumes readiness, and only a genuine
    # re-grip (driver_tq back above the arm level) restores it. A pure
    # refractory let the dump repeat every 20 frames against a slow wheel
    # (10° command sawtooth at 5 Hz in the closed-loop probe); the edge
    # reduces that to one dump per grip while preserving the field event's
    # t=3.01 catch (the driver genuinely re-gripped at t=2.55-2.90).
    if driver_tq >= CarControllerParams.REANCHOR_ARM_NM:
      self.reanchor_ready = True
    self.anchor_recent_frames = max(self.anchor_recent_frames - 1, 0)
    # Phase 33: BSM occupancy hold — radar flags flicker at lane edges, so
    # an occupied report holds for 1 s past the last active frame.
    self.blind_left_hold = (CarControllerParams.BLIND_HOLD_FRAMES if bool(CS.out.leftBlindspot)
                            else max(self.blind_left_hold - 1, 0))
    self.blind_right_hold = (CarControllerParams.BLIND_HOLD_FRAMES if bool(CS.out.rightBlindspot)
                             else max(self.blind_right_hold - 1, 0))
    # Phase 37a rain mode: front-wiper switch (CCNC_WIPER) -> debounced flag
    # -> ramped weight. Unknown/stale input counts as OFF (conservative).
    wiper_on = bool(getattr(CS, "wiper_front_on", False)) and not bool(getattr(CS, "wiper_stale", True))
    if wiper_on:
      self.wiper_on_frames = min(self.wiper_on_frames + 1, 10**6); self.wiper_off_frames = 0
    else:
      self.wiper_off_frames = min(self.wiper_off_frames + 1, 10**6); self.wiper_on_frames = 0
    if self.wiper_on_frames >= CarControllerParams.RAIN_WIPER_ON_FRAMES:
      self.rain_active = True
    elif self.wiper_off_frames >= CarControllerParams.RAIN_WIPER_OFF_FRAMES:
      self.rain_active = False
    _rt = CarControllerParams.RAIN_RAMP_UP_TAU_S if self.rain_active else CarControllerParams.RAIN_RAMP_DN_TAU_S
    self.rain_w += (DT_CTRL / (_rt + DT_CTRL)) * ((1.0 if self.rain_active else 0.0) - self.rain_w)
    # Phase 30: frames since driver torque was last at/above the fire level —
    # the one-shot dump only makes sense at the release edge.
    self.frames_since_drv_release = (0 if driver_tq >= CarControllerParams.REANCHOR_FIRE_NM
                                     else min(self.frames_since_drv_release + 1, 1000))

    # Driver override factor — the heavy-grip anchor gate and yield blend
    # coefficient (Phase 5a lineage). When the blinker is on, lower
    # thresholds so a light grip during a lane change immediately produces
    # override_factor > 0 and op yields. (Phase 13a: the low-speed
    # passthrough latch no longer keys on this — its low-V full-override
    # point sits inside the column-torque offset; the latch now enters on
    # debounced steeringPressed and releases on a sustained sub-260 Nm.)
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

    # The heavy-override anchor (apply_angle_last := wheel for a bump-free resume)
    # fires on REAL heavy grip only — debounced steeringPressed AND override>=0.9 —
    # not the column-torque offset tripping override_factor>=0.9 hands-off at low
    # speed (where full_override_torque is only 180 Nm), which would re-inject the
    # raw wheel's 2-8 Hz into apply_angle_last.
    # Phase 10d (blinker anchor fast-path): steeringPressed on angle-control needs
    # |tq| > STEER_THRESHOLD(350) sustained 5 frames, so during a signaled lane
    # change the driver had to shove past ~350 Nm before the anchor yielded —
    # routes 0x00-0x0b measured per-episode peak |tq| p50 = 519-562 despite the
    # 10b/10c threshold cuts. With the blinker announcing intent, accept a lower
    # UNDEBOUNCED torque floor instead: BLINKER_ANCHOR_TORQUE_NM (220) clears the
    # +90..180 Nm column-torque offset (so hands-off blinker never false-fires,
    # preserving the 10c hands-off authority) yet engages ~130 Nm / ~50 ms earlier
    # than steeringPressed. Non-blinker behaviour unchanged.
    # Kill switch: BLINKER_ANCHOR_TORQUE_NM = 1e9 (pressed-only, pre-10d).
    # Phase 14-2: the single 220 Nm test flapped at 3.84 transitions/s during
    # low-speed blinker waits (routes 0x2e-0x2f) as |tq| wandered across the
    # threshold — the flicker case flagged at 10d's introduction. Make the
    # blinker anchor stateful: fire after 3 sustained frames >= 220, then hold
    # while |tq| >= 180 (40 Nm band) with a 0.3 s minimum hold.
    # Phase 26: driver-torque domain — the raw 220 Nm test sat inside the hold
    # band (hands-off curves cross it 25-50% of the time), so a signaled turn
    # on a curved road could fire the anchor with no driver input at all and
    # stall plan tracking mid-lane-change. 120 driver-domain == raw 220 in a
    # straight (identical behaviour there); in curves only a real hand fires it.
    # Phase 26b review: the fire test is additionally gated on the previous
    # frame's effective_lat_active — the anchor exists to yield to a driver
    # during OP-ACTIVE lane changes, and in comp-off (passive) frames the
    # 120 threshold would otherwise degenerate to a raw test far below the
    # measured rolling-passive bar level (p50 147-233 Nm), self-firing
    # hands-off. The keep/release path is left ungated so an episode that
    # legitimately fired op-active still ends on the driver's let-go.
    blinker_anchor_raw = (blinker_on and self.prev_eff_lat_active
                          and driver_tq >= CarControllerParams.BLINKER_ANCHOR_TORQUE_NM)
    if self.blinker_anchor_on:
      self.blinker_anchor_hold += 1
      keep = blinker_on and driver_tq >= CarControllerParams.BLINKER_ANCHOR_RELEASE_NM
      # Phase 14-2b: the release was a SINGLE sub-180 frame (vs the 3-frame
      # debounced fire) — asymmetric, so offset-band noise (100-300 Nm,
      # 1-5 Hz) still flapped the anchor at 2-3 transitions/s in sim, the
      # exact flicker 14-2 set out to remove. Release on a sustained let-go
      # instead, same 0.5 s standard as the 13a latches.
      self.blinker_anchor_release = 0 if keep else self.blinker_anchor_release + 1
      if (self.blinker_anchor_release >= CarControllerParams.BLINKER_ANCHOR_RELEASE_FRAMES
          and self.blinker_anchor_hold >= CarControllerParams.BLINKER_ANCHOR_MIN_HOLD_FRAMES):
        self.blinker_anchor_on = False
        self.blinker_anchor_release = 0
    else:
      self.blinker_anchor_fire = self.blinker_anchor_fire + 1 if blinker_anchor_raw else 0
      if self.blinker_anchor_fire >= CarControllerParams.BLINKER_ANCHOR_FIRE_FRAMES:
        self.blinker_anchor_on = True
        self.blinker_anchor_hold = 0
        self.blinker_anchor_fire = 0
        self.blinker_anchor_release = 0
    # Phase 26: driver_pressed (hold-compensated) replaces the raw EPS flag —
    # 41% of raw pressed frames sat where the hold model predicts >= 300 Nm,
    # so hard curves could anchor apply_angle_last to the wheel hands-off.
    # override_factor stays raw-domain deliberately: it is strictly more
    # permissive than driver_pressed, so the compensated term is the binding
    # gate of this AND.
    # Phase 28: CS.out.steeringPressed restored as an anchor OR-arm — the
    # 0x41 seg17 yank happened in the 350-490 raw curve band where the
    # hold-comp cap pushes driver_pressed out of reach while the EPS flag
    # held True the whole time; the flag anchored this exact regime for
    # weeks pre-Phase-26 with no yank. The anchor's consequence (apply :=
    # wheel) is benign, so the Phase 26 self-press concern (hard-curve
    # hands-off tail) costs a wheel-follow there, not a control cut; a
    # driver_tq magnitude arm cannot replace it (hands-off driver_tq p90 =
    # 162.5 — review-measured; the v1 160 arm re-injected wheel noise on
    # 7.5% of hands-off frames and was rejected).
    # Phase 31b (review): driver_pressed now anchors ON ITS OWN. The
    # override_factor>=0.9 AND was justified by "override is strictly more
    # permissive than driver_pressed" — the Phase 31 baseline drop inverted
    # that from ~54 km/h up (pressed raw-equivalent 303.5 at 15 m/s, 292 at
    # >=25 m/s vs the override point 325; margin was -22.6 Nm pre-31,
    # +21.5..+33 after), opening a band where a steady one-hand ~300 Nm
    # correction got deep yield (real_grip floor 0.05) with NO anchor and
    # NO anchor_recent — divergence building unanchored, the 0x41 slam
    # preconditions. NOTE the earlier review claim "the EPS raw-350 arm is
    # the anchor backstop" was WRONG as stated: that arm is co-gated by
    # override>=0.9, so in the gap band neither arm fired — a backstop
    # claim must be checked at ITS OWN gating conditions in the regime
    # where the primary fails. Corrected invariant: driver_pressed anchors
    # unconditionally (hands-off machine false rate 0.00% corpus; newly
    # anchored-alone frames measure 13.5 s / 2.8 h, hands-off 0.005%);
    # the EPS/blinker arms keep the raw override gate.
    heavy_grip_anchor = self.driver_pressed or (
      (override_factor >= 0.9) and (bool(CS.out.steeringPressed) or self.blinker_anchor_on))
    # G3 review: only the PRESSED arms count as grip evidence for the
    # re-anchor memory — a blinker-arm anchor can arise hands-off and, as
    # recency, produced 129 context-free hands-off dumps in corpus replay.
    if heavy_grip_anchor and (self.driver_pressed or bool(CS.out.steeringPressed)):
      self.anchor_recent_frames = CarControllerParams.REANCHOR_RECENT_FRAMES

    # Low-speed camera passthrough latch (kept-feature #11).
    # Phase 13a: the latch used to key on `hands_off` (override_factor <= 0.5),
    # but hands-off |tq| at low speed measures p50=156/p90=284/p99=367 Nm
    # (offset + road load, routes 0x2a-0x2d), so ANY fixed sub-350 torque
    # test flaps — with Phase 11 opening latActive below 20 km/h the latch
    # flapped plan<->wheel at 1.1-2.2 Hz, which was the reported wheel
    # shaking. Redesign: ENTER on the debounced steeringPressed flag (real
    # 350 Nm x 5-frame grip only), RELEASE on a sustained sub-260 Nm let-go.
    # Flap-free by construction.
    # F8/R1 note: use v_ego_safe here (not raw vEgoRaw) — a NaN speed frame
    # fails every comparison and would freeze both latches in whatever state
    # they held; the sanitizer maps it to 0 so they fail toward passive.
    # Phase 26: ENTER keys on the hold-compensated driver_pressed. The
    # release comparison keeps the raw-domain 260 threshold: it only runs
    # while the latch is passive, where 26b gates hold_comp to 0 so
    # driver_tq == raw (rolling-passive hands-off bar p50 147-233 Nm).
    # Phase 31b (review): the EPS flag is restored as an entry OR-arm — the
    # Phase 31 crawl baseline (B=140, unfitted below ~3 m/s) pushed the
    # driver_pressed raw-equivalent to ~370, ABOVE the raw-350 EPS flag this
    # latch was designed to mirror (13a), leaving a 350-370 dead band where
    # EPS says pressed but the shake-fix latch would not enter. EPS-350 is
    # itself hand-validated grip evidence at these speeds.
    if v_ego_safe < LOW_SPEED_PASSTHROUGH_ENTER_MS and (self.driver_pressed
                                                        or bool(CS.out.steeringPressed)):
      self.low_speed_cam_latched = True
      self.lowv_release_frames = 0
    elif self.low_speed_cam_latched:
      if (not self.driver_pressed) and \
         driver_tq < CarControllerParams.LOW_SPEED_GRIP_RELEASE_NM:
        self.lowv_release_frames += 1
      else:
        self.lowv_release_frames = 0
      if (v_ego_safe > LOW_SPEED_PASSTHROUGH_EXIT_MS
          or self.lowv_release_frames >= CarControllerParams.LOW_SPEED_GRIP_RELEASE_FRAMES):
        self.low_speed_cam_latched = False
        self.lowv_release_frames = 0
    # Speed-zone latch for the scenario gate (same 20/22 km/h hysteresis).
    if v_ego_safe < LOW_SPEED_PASSTHROUGH_ENTER_MS:
      self.in_low_speed_zone = True
    elif v_ego_safe > LOW_SPEED_PASSTHROUGH_EXIT_MS:
      self.in_low_speed_zone = False
    # Traffic-following keeps op engaged in stop-and-go even below the
    # low-speed freeze threshold. lead_visible / lead_distance are
    # populated by LeadDataCarController.update.
    if self.lead_visible and self.lead_distance < TRAFFIC_FOLLOW_NEAR_M:
      self.traffic_following = True
    elif (not self.lead_visible) or self.lead_distance > TRAFFIC_FOLLOW_FAR_M:
      self.traffic_following = False
    # Phase 13a scenario gate: below 20 km/h, steer only in the scenarios the
    # model handles well — traffic crawl (lead in the widened follow window) or
    # gentle lane keeping (|cmd| < 40°). Free low-speed maneuvers (intersection
    # turns / alleys, |cmd| 100°+) stay manual. Asymmetric dwell: 0.3 s to
    # yield, 1.0 s to (re)engage — the gate cannot flap. Note real grip
    # (low_speed_cam_latched) now yields even during traffic-following: with
    # the offset-proof threshold, latched means actual driver intent.
    # Phase 14-3: hysteresis on the gentle-path threshold (was a single 40°
    # boundary — ~10% of residual low-speed flips on 0x2e-0x2f): while steering
    # stays allowed until |cmd| exceeds 45°; once passive, return below 35°.
    gentle_thr = (CarControllerParams.LOW_SPEED_CMD_PASSIVE_DEG if self.low_speed_scen_ok
                  else CarControllerParams.LOW_SPEED_CMD_ACTIVE_DEG)
    # Phase 18 creep gate: below ~10 km/h the model's own command oscillates
    # (near-standstill vision curvature noise, cmd 2-8 Hz RMS 1.2-2.4° on
    # routes 0x36-0x37) and no amount of downstream smoothing removes it —
    # 27% of creep-band steering time measured wheel RMS > 0.15° at ~1 Hz,
    # the felt shake. In the creep zone steer ONLY when following a lead with
    # the blinker off (queue crawl — the one creep scenario where the command
    # is anchored to the lead and measured quiet). Free creep, turn-waiting
    # and blinker creep go manual: nothing there needs op steering. Zone has
    # its own 10/12 km/h hysteresis; the existing dwell debounces transitions.
    # Kill switch: CREEP_GATE_ENTER_MS = 0.0 (zone never arms).
    if v_ego_safe < CarControllerParams.CREEP_GATE_ENTER_MS:
      self.in_creep_zone = True
    elif v_ego_safe > CarControllerParams.CREEP_GATE_EXIT_MS:
      self.in_creep_zone = False
    if self.in_creep_zone:
      scen_raw = self.traffic_following and not blinker_on
    else:
      scen_raw = self.traffic_following or (abs(op_curv_safe) < gentle_thr)
    if scen_raw != self.low_speed_scen_ok:
      self.lowv_scen_dwell += 1
      need = (CarControllerParams.LOW_SPEED_SCEN_TO_ACTIVE_FRAMES if scen_raw
              else CarControllerParams.LOW_SPEED_SCEN_TO_PASSIVE_FRAMES)
      if self.lowv_scen_dwell >= need:
        self.low_speed_scen_ok = scen_raw
        self.lowv_scen_dwell = 0
    else:
      self.lowv_scen_dwell = 0
    in_passthrough = self.low_speed_cam_latched or (self.in_low_speed_zone and not self.low_speed_scen_ok)

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
    # Phase 14-4 S1b: EV departure coverage. An EV drives off immediately
    # while the device takes ~20-30 s to boot, so by the first control frame
    # the gear is already D and the S1 park-gear signature can never fire for
    # a departure. Proxy: a controller COLD-START observed at low speed is
    # almost always a lot/departure context — decide once, ~0.5 s in (CAN
    # settled). A mid-drive device reboot on a slow road is the only false
    # path and costs one passive stretch until the 33 km/h exit.
    if self.boot_parking_pending and self.frame >= 50:
      self.boot_parking_pending = False
      if v_ego_safe <= PARKING_MODE_ENTER_MS:
        self.parking_signature_seen = True
    if v_ego_safe <= PARKING_MODE_ENTER_MS:
      self.parking_low_speed_frames = min(self.parking_low_speed_frames + 1,
                                          PARKING_MODE_ENTER_SUSTAIN_FRAMES)
      if (abs(steer_angle_safe) >= PARKING_MODE_ENTER_WHEEL_DEG
          or CS.out.gearShifter == structs.CarState.GearShifter.reverse
          # Phase 14-4 S1: park gear = the strongest lot evidence available
          # without scene understanding — every departure starts in P, so the
          # boot-to-first->33km/h window is automatically a parking regime.
          or CS.out.gearShifter == structs.CarState.GearShifter.park
          # Phase 14-4 S2: door / seatbelt activity at standstill (pick-up,
          # drop-off, the moments around parking itself).
          or ((CS.out.doorOpen or CS.out.seatbeltUnlatched) and CS.out.standstill)):
        self.parking_signature_seen = True
    else:
      self.parking_low_speed_frames = 0
    # Phase 14-4 S3': lead-less lot-crawl pattern. A parking-lot drive is
    # crawl <-> short 20-22 km/h aisle spikes; surface streets hold 20-25
    # steadily. Signature: a 10 s window that (a) never exceeds 25 km/h,
    # (b) has no lead within TRAFFIC_FOLLOW_FAR_M (queue crawl excluded), and
    # (c) shows the lot pattern — a dip below 8 km/h or window mean < 12 km/h.
    # False-fire cost is bounded: passive only until the 33 km/h exit.
    lead_near = self.lead_visible and self.lead_distance < TRAFFIC_FOLLOW_FAR_M
    if v_ego_safe < PARKING_CREEP_MAX_MS and not lead_near:
      self.creep_frames += 1
      self.creep_min = min(self.creep_min, v_ego_safe)
      self.creep_sum += v_ego_safe
      if (self.creep_frames >= PARKING_CREEP_FRAMES
          and (self.creep_min < PARKING_CREEP_DIP_MS
               or self.creep_sum / self.creep_frames < PARKING_CREEP_MEAN_MS)):
        self.parking_signature_seen = True
    else:
      self.creep_frames = 0
      self.creep_min = float('inf')
      self.creep_sum = 0.0
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
      if heavy_grip_anchor:
        self.apply_angle_last = float(np.clip(steer_angle_safe,
                                              -self.params.ANGLE_LIMITS.STEER_ANGLE_MAX,
                                               self.params.ANGLE_LIMITS.STEER_ANGLE_MAX))
        self.frames_since_apply_anchor = 0
      elif (self.reanchor_arm >= CarControllerParams.REANCHOR_ARM_FRAMES
            and self.anchor_recent_frames > 0
            and self.reanchor_ready
            and driver_tq < CarControllerParams.REANCHOR_FIRE_NM
            # Phase 30: only at the release EDGE (torque above the fire level
            # within the last 0.2 s). Without this, a cleanly released op
            # re-approaching the plan builds its OWN 2° of progress and the
            # dump yanked it back to the wheel once per release — the fire
            # exists for divergence stored DURING the grip, which by
            # definition is present at the instant torque collapses.
            and self.frames_since_drv_release <= CarControllerParams.REANCHOR_EDGE_FRAMES
            and abs(self.apply_angle_last - steer_angle_safe) > CarControllerParams.REANCHOR_MIN_DIV_DEG):
        # Phase 28 release re-anchor (see values.py): the driver just let go
        # after an override episode that included a real anchor-grade grip
        # (anchor_recent), and apply carries a stored divergence — dump it
        # to the wheel so ACIGain recovery cannot slam the wheel toward a
        # stale command (0x41 seg17: -0.3° -> +8.8° at 86 km/h). op then
        # re-approaches the plan VM-rate-limited. The fire consumes the
        # re-arm edge (G2) but keeps the episode memory (F1: a disarm left
        # the memory dead at the field event's actual t=3.01 release). The
        # wound curve_trim is zeroed so it cannot recreate the divergence
        # just dumped (F5).
        self.apply_angle_last = float(np.clip(steer_angle_safe,
                                              -self.params.ANGLE_LIMITS.STEER_ANGLE_MAX,
                                               self.params.ANGLE_LIMITS.STEER_ANGLE_MAX))
        self.frames_since_apply_anchor = 0
        self.reanchor_ready = False
        self.curve_trim = 0.0
        self.trim_resid_lp = 0.0
      # Phase 30: the Phase 29/29b divergence leash that followed this
      # one-shot was REMOVED. It clamped apply to wheel±2° for 2 s after an
      # anchor episode to stop slow-ease tail re-divergence — but torque
      # cannot distinguish a lightly-resting hand from a fully-released one
      # (hands-off driver_tq p50 40.6 / p75 111), so it equally pinned apply
      # to a FREE, caster-unwinding wheel and abandoned the plan: corpus
      # audit found 36 urgent-regrab windows in the pinned state across 13
      # routes, including a field lane-departure at the Namsan-3 tunnel
      # approach (0x47 seg15: wheel unwound +21°->+8° with apply pinned to
      # it while the plan sat 25-30° away). The class it guarded (rigid
      # slow-ease slam) was model-only, never field-observed, and remains
      # covered by the one-shot dump above (release-instant divergence = 0)
      # plus the post-grip recovery taper (authority restored gently) plus
      # the VM rate limits (the designed comfort bound for the approach).
      # Neither torque-gated variant could fix it (silence-timeout: 50%
      # coverage, arm-budget: 53% — both defeated by hands-off torque noise
      # re-arming the state). Handover latency is the safety-dominant side.
      desired_angle = float(np.clip(op_curv_safe, -self.params.ANGLE_LIMITS.STEER_ANGLE_MAX,
                                                   self.params.ANGLE_LIMITS.STEER_ANGLE_MAX))

      # Phase 12b (P2): backlash/hysteresis band on the desired angle. The output
      # follows any move larger than the band with zero lag; micro-reversals
      # inside +/-CMD_HYSTERESIS_DEG (model replan churn, 6.5-7.7 direction
      # flips/10s on 0x10-0x28) are absorbed instead of being sent to the MDPS.
      # When inactive, track the raw desired so engagement starts band-centered.
      # Phase 32: speed-scheduled — the flat 0.15° band is ~3% of the
      # measured low-speed plan churn (0.5° RMS at 0.8-2.5 Hz), so nearly
      # all of it reached TX. 0.5° below 25 km/h absorbs the churn at the
      # SOURCE (complementing the ceiling cut, which reduces transmission);
      # tapers to the legacy 0.15 by 43 km/h so curve tracking above town
      # speed is untouched.
      hyst = float(np.interp(v_ego_safe, CarControllerParams.CMD_HYSTERESIS_SPEEDS_MS,
                             CarControllerParams.CMD_HYSTERESIS_V))
      if CC.latActive and hyst > 0.0:
        self.cmd_hyst = float(np.clip(self.cmd_hyst, desired_angle - hyst, desired_angle + hyst))
        desired_angle = self.cmd_hyst
      else:
        self.cmd_hyst = desired_angle

      # Phase 12a (P1): steady-curve trim — close the MDPS's sustained-curve
      # realization deficit (wheel/TX = 0.55-0.63, |TX-wheel| p50 3.7° on
      # 0x10-0x28) at the TX layer instead of waiting for the model's visual
      # loop to inflate the command. Slew-limited pursuit of the residual,
      # speed-scheduled cap, armed only after a sustained curve command while
      # hands-off; bleeds out on grip/straight/inactive. See values.py.
      trim_rate = CarControllerParams.CURVE_TRIM_RATE_DPS
      # Phase 13b: gate on the debounced steeringPressed only (the original
      # `hands_off` torque test flaps inside the p50=156/p90=284 Nm hands-off
      # band and blocked the trim almost always — TX-cmd p50 +0.02° as
      # shipped). Hold the angle gate with hysteresis (arm 4°, hold 3°) so
      # the gate cannot flap at the curve threshold.
      angle_gate = abs(desired_angle) >= (
        CarControllerParams.CURVE_TRIM_HOLD_CMD_DEG
        if self.curve_trim_sustain >= CarControllerParams.CURVE_TRIM_SUSTAIN_FRAMES
        else CarControllerParams.CURVE_TRIM_MIN_CMD_DEG)
      # Phase 26: gate on driver_pressed — the raw EPS flag self-triggers in
      # exactly the sustained curves the trim exists for (41% of pressed
      # frames sit at hold-model >= 300 Nm), so the delivery fix was being
      # disabled where the delivery deficit lives. (A Phase 28 draft added
      # the EPS flag back as an AND — G4 review measured that disabling the
      # trim on 37.4% of curve-eligible frames; dropped. The F5 wound-trim
      # problem is closed by zeroing curve_trim on a re-anchor fire instead.)
      trim_gate = (bool(CC.latActive) and not self.driver_pressed
                   and not self.parking_mode_active and not in_passthrough
                   and trim_rate > 0.0 and v_ego_safe > 3.0 and angle_gate)
      self.curve_trim_sustain = min(self.curve_trim_sustain + 1, 10 * CarControllerParams.CURVE_TRIM_SUSTAIN_FRAMES) if trim_gate else 0
      # S-curve direction flip: a trim built for the previous curve points the
      # wrong way once desired crosses zero. The 2°/s integrator (or the 0.5 s
      # exit bleed) alone would carry an opposing trim for seconds — replay
      # measured up to 4.5° opposing — so fast-bleed any trim opposing the
      # current curve direction, in every branch.
      if self.curve_trim * np.sign(desired_angle) < 0.0:
        self.curve_trim *= (1.0 - DT_CTRL / CarControllerParams.CURVE_TRIM_FLIP_TAU_S)
      if self.curve_trim_sustain >= CarControllerParams.CURVE_TRIM_SUSTAIN_FRAMES:
        trim_cap = float(np.interp(v_ego_safe,
                                   CarControllerParams.CURVE_TRIM_CAP_SPEEDS_MS,
                                   CarControllerParams.CURVE_TRIM_CAP_DEG))
        # Rate-limited integrator: step toward the residual's sign at up to
        # CURVE_TRIM_RATE_DPS until the residual closes (equilibrium residual=0,
        # i.e. wheel = pre-trim desired) or the cap binds. A pursuit law
        # (trim -> residual) would only close ~half the gap at equilibrium.
        # Phase 13b: the residual consumes the RAW measured wheel — replay
        # showed the un-filtered trim itself carrying 1-8 Hz RMS 0.069°, the
        # size of the whole v2 jitter budget. LP the residual (tau 0.3 s) and
        # apply a deadband so wheel noise never reaches the integrator; the
        # deadband also sets the closure floor (residual converges to ~0.7°
        # instead of 0 — a fifth of the 3.7° deficit it exists to close).
        residual = desired_angle - steer_angle_safe
        alpha = DT_CTRL / max(CarControllerParams.CURVE_TRIM_RESID_LP_TAU_S, DT_CTRL)
        self.trim_resid_lp += alpha * (residual - self.trim_resid_lp)
        db = CarControllerParams.CURVE_TRIM_RESID_DEADBAND_DEG
        res_eff = float(np.sign(self.trim_resid_lp)) * max(0.0, abs(self.trim_resid_lp) - db)
        step = trim_rate * DT_CTRL
        self.curve_trim = float(np.clip(self.curve_trim + np.clip(res_eff, -step, step),
                                        -trim_cap, trim_cap))
      else:
        self.trim_resid_lp = 0.0
        # exponential bleed toward 0 (tau = CURVE_TRIM_BLEED_TAU_S)
        self.curve_trim *= (1.0 - DT_CTRL / max(CarControllerParams.CURVE_TRIM_BLEED_TAU_S, DT_CTRL))
        if abs(self.curve_trim) < 0.01:
          self.curve_trim = 0.0
      desired_angle = float(np.clip(desired_angle + self.curve_trim,
                                    -self.params.ANGLE_LIMITS.STEER_ANGLE_MAX,
                                     self.params.ANGLE_LIMITS.STEER_ANGLE_MAX))

      if abs(v_ego_safe) < CarControllerParams.SMOOTHING_ANGLE_MAX_VEGO:
        desired_angle = sp_smooth_angle(v_ego_safe, desired_angle, self.apply_angle_last)

      # Phase 9 (yield-by-authority): op keeps its own clean commanded angle — the
      # driver yield is done entirely on the ACIGain authority axis below, so the
      # measured wheel's 2-8 Hz never enters the command. (The earlier command-side
      # grip-blend, Phase 5a/8b, was removed: it false-fired on the column-torque
      # offset and was the sole input->TX 2-8 Hz amplifier.)
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
        # Phase 10a: extra low-speed angle-rate taper to kill the felt "grab" on
        # override-recovery at <30 km/h (see values.py). The VM limiter already
        # ran; this only tightens the per-frame step further at low speed.
        rate_cap = float(np.interp(v_ego_safe,
                                   CarControllerParams.MAX_ANGLE_RATE_LOWSPEED_BP,
                                   CarControllerParams.MAX_ANGLE_RATE_LOWSPEED_V))
        apply_angle = float(np.clip(apply_angle, self.apply_angle_last - rate_cap,
                                                 self.apply_angle_last + rate_cap))
        # Phase 37a: inside the recovery window after an anchor (the command
        # chasing the plan from wherever the wheel was left), bound the
        # command's lateral jerk below the panda limit at highway speed so
        # the correction is spread over a longer time (see values.py
        # RECOVERY_JERK_CAP_*). Planned driving never reaches these rates.
        if self.frames_since_apply_anchor <= CarControllerParams.RECOVERY_JERK_CAP_FRAMES:
          _vk = v_ego_safe * CV.MS_TO_KPH
          _jn = float(np.interp(_vk, CarControllerParams.RECOVERY_JERK_CAP_SPEEDS_KPH, CarControllerParams.RECOVERY_JERK_CAP_V))
          _jr = float(np.interp(_vk, CarControllerParams.RECOVERY_JERK_CAP_SPEEDS_KPH, CarControllerParams.RECOVERY_JERK_CAP_RAIN_V))
          _j = _jn + self.rain_w * (_jr - _jn)
          if _j < self.params.ANGLE_LIMITS.MAX_LATERAL_JERK - 1e-6:
            _vs = max(v_ego_safe, 1.0)
            # same conversion as the VM limiter; take the tighter of the two
            # vehicle models so the taper is delivered in full (the baseline
            # safety VM is the binding one by ~10%)
            _cap = min(math.degrees(self.VM.get_steer_from_curvature(_j / (_vs ** 2), _vs, 0)),
                       math.degrees(self.BASELINE_VM.get_steer_from_curvature(_j / (_vs ** 2), _vs, 0))) * DT_CTRL
            apply_angle = float(np.clip(apply_angle, self.apply_angle_last - _cap,
                                                     self.apply_angle_last + _cap))
        self.apply_angle_last = apply_angle
        self.alert_vm_limit_frames = max(self.alert_vm_limit_frames - 2, 0)

      if not CC.latActive:
        self.apply_angle_last = float(np.clip(steer_angle_safe,
                                              -self.params.ANGLE_LIMITS.STEER_ANGLE_MAX,
                                               self.params.ANGLE_LIMITS.STEER_ANGLE_MAX))
        self.frames_since_apply_anchor = 0

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
    # Phase 14-1: offset-proof redesign. The v1-era thresholds (enter 30/60 Nm,
    # exit <30 Nm) all sit inside the +90..180 Nm column-torque offset band —
    # with Phase 11 opening latActive below 20 km/h this latch became the main
    # residual shake source on 0x2e-0x2f: exit <30 Nm is near-impossible while
    # moving (hands-off |tq| p10=24), locking gentle low-speed stretches 69%
    # passive, while the 30 Nm entry boundary + offset-dominated sign test
    # flapped plan<->wheel around the low-torque tail. Redesign to the 13a
    # standard: entry A keys on debounced steeringPressed, entry B (opposing
    # push) needs 260 Nm sustained 0.3 s so the sign is the driver's, and exit
    # is a sustained let-go (!pressed & <260 Nm for 0.5 s).
    if not bool(CC.latActive):
      self.angle_passive_active = False
      self.angle_passive_enter_frames = 0
      self.intent_disagree_frames = 0
      self.angle_passive_release_frames = 0
    elif self.angle_passive_active:
      # Phase 26/26b: exit runs with op passive, where hold_comp is gated to
      # 0 (driver_tq == raw) — the raw-domain 260 threshold is kept, so exit
      # semantics stay bit-identical to the validated 14-1 design.
      if (not self.driver_pressed) and \
         driver_tq < CarControllerParams.ANGLE_PASSIVE_EXIT_TORQUE_NM:
        self.angle_passive_release_frames += 1
      else:
        self.angle_passive_release_frames = 0
      if self.angle_passive_release_frames >= CarControllerParams.ANGLE_PASSIVE_EXIT_FRAMES:
        self.angle_passive_active = False
        self.angle_passive_enter_frames = 0
        self.intent_disagree_frames = 0
        self.angle_passive_release_frames = 0
    else:
      # Phase 6f-3 OR-arm, 14-1 hardened: the driver pushes >= 260 Nm
      # (clears the offset — the old 30 Nm test made the sign a coin flip)
      # opposite op's apply_angle_last (>= 5° off the wheel) at <= 30 km/h,
      # sustained 0.3 s.
      # Phase 26: magnitude test moves to driver_tq (raw 260 -> 160 driver-
      # domain); the SIGN stays on the raw reading — driver_tq is unsigned,
      # and when it is positive the excess beyond the hold estimate carries
      # the raw signal's sign. The opposing-sign test already protects this
      # arm structurally (op's own hold torque always points toward its own
      # tracking error, never against it). Comp-off note: this arm also
      # evaluates in passive frames (threshold then raw 160) — in parking /
      # passthrough the |apply - wheel| >= 5° gate is structurally unmeetable
      # (apply is wheel-clamped every frame); in fault/reverse passivity a
      # fire is a no-op (op already passive) and clears on the raw <260
      # let-go like any angle-passive episode.
      low_intent_disagree = (
        v_ego_safe <= CarControllerParams.INTENT_DISAGREE_VEGO_MS
        and driver_tq >= CarControllerParams.INTENT_DISAGREE_TQ_MIN_NM
        and abs(self.apply_angle_last - steer_angle_safe) >= CarControllerParams.INTENT_DISAGREE_DELTA_DEG
        and (np.sign(steer_torque_safe)
             * np.sign(self.apply_angle_last - steer_angle_safe)) < 0
      )
      self.intent_disagree_frames = self.intent_disagree_frames + 1 if low_intent_disagree else 0
      # Phase 26: driver_pressed replaces the raw EPS flag — a >= 40° curve at
      # speed generates enough hold torque to trip raw pressed hands-off
      # (geo-entry self-fire: 202 candidate episodes on 0x36-0x3d), which
      # would drop op passive mid-curve with nobody holding the wheel.
      geo_enter = (abs(steer_angle_safe) >= CarControllerParams.ANGLE_PASSIVE_ENTER_WHEEL_DEG
                   and self.driver_pressed)
      self.angle_passive_enter_frames = min(self.angle_passive_enter_frames + 1,
                                            CarControllerParams.ANGLE_PASSIVE_MIN_ENTER_FRAMES) if geo_enter else 0
      if (self.angle_passive_enter_frames >= CarControllerParams.ANGLE_PASSIVE_MIN_ENTER_FRAMES
          or self.intent_disagree_frames >= CarControllerParams.INTENT_DISAGREE_SUSTAIN_FRAMES):
        self.angle_passive_active = True
        self.angle_passive_release_frames = 0
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
    # in_passthrough (low-speed grip latch / 13a scenario gate) gets the same
    # treatment for the same reason: op is passive there with CC.latActive
    # still True, so without the anchor apply_angle_last keeps advancing on
    # the plan while the driver turns — the gate then re-engaged with a stale
    # angle tens of degrees off the wheel (sim measured a 40°+ first-frame
    # step at scenario-gate release). Anchored, resume starts at the wheel.
    if (self.angle_passive_active or heavy_grip_anchor or self.parking_mode_active
        or in_passthrough):
      self.apply_angle_last = float(np.clip(steer_angle_safe,
                                            -self.params.ANGLE_LIMITS.STEER_ANGLE_MAX,
                                             self.params.ANGLE_LIMITS.STEER_ANGLE_MAX))
      self.frames_since_apply_anchor = 0
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

    # Phase 27: surface the intentional-passive latches to the driver.
    # card.py mirrors this into carStateSP.lateralControlPaused and the UI
    # shows the MADS paused/override presentation while it is set — op looks
    # engaged on the cluster but is deliberately not tracking the plan here
    # (automation-surprise fix; parking-lot congestion exits measured up to
    # 398 s of silent passivity). Display ONLY: driving the MADS state
    # machine from these latches would drop CC.latActive, which resets the
    # latches themselves — an oscillation by construction. Fault paths
    # (cam_stale / LFA fault / reverse) are excluded: they carry their own
    # alerts. heavy_grip_anchor / blinker anchor are excluded: those are the
    # driver's own hands on the wheel, and flashing the icon during routine
    # overrides would train the driver to ignore it.
    # Phase 26b (review fix): vm_reject_persistent and was_in_reverse are
    # included after all — vm rejection has no alert below 8 m/s and the
    # post-reverse crawl has none at any speed, so both are silent passivity
    # in exactly the regime this flag exists for. cam_stale / LFA fault stay
    # excluded (they raise their own alerts).
    self.lat_passive_indicated = bool(CC.latActive) and ccnc_lka_alt and (
      self.parking_mode_active or in_passthrough or self.angle_passive_active
      or vm_reject_persistent or self.was_in_reverse)
    self.prev_eff_lat_active = bool(effective_lat_active)

    # ACIGain (reference 17-line compute_torque_reduction_gain).
    effective_aci_gain = None
    if ccnc_lka_alt and lkas_alt_cam_msg is not None:
      steering_error = self.apply_angle_last - steer_angle_safe
      # Phase 9: yield-by-authority reshapes the ACIGain torque curve to drop
      # harder on real grip (absorbing the removed command-blend) and suppresses
      # the error boost so MDPS does not fight the driver's divergence. Gated on
      # the DEBOUNCED steeringPressed flag (not the offset-corrupted torque/
      # override_factor): hands-off therefore stays bit-identical to legacy
      # (full authority + drift recovery), only real grip changes behaviour.
      # Phase 26: real_grip now keys on the hold-compensated driver_pressed —
      # the raw EPS flag self-triggered in hard curves (41% of pressed frames
      # at hold-model >= 300 Nm), silently flipping the yield curve to the
      # grip branch and suppressing the error boost exactly where sustained-
      # curve delivery needs it.
      real_grip = self.driver_pressed
      # Phase 22/31: driver_tq / hold_comp (compute_hold_torque model;
      # parked-car data proved the true sensor offset ~ 0) are now
      # computed once at the top of update() — Phase 26 — and shared with
      # every latch above.
      # Phase 23: the hands-off else-branch used to pass the LEGACY raw-domain
      # literals (350.0 / 0.19), silently overriding every hands-off band
      # change since Phase 21 (the function defaults were dead code on this
      # path) — sub-350 Nm firm grip therefore never got the softened yield.
      # Route through values.py constants in the Phase 22 driver-torque
      # domain, and speed-schedule the FLOORS down at highway speed: during
      # spirited driving the driver already grips past full-yield, so the
      # felt residual force IS the floor — relief goes exactly there while
      # city-speed floors (tracking authority) stay untouched.
      v_kph_aci = v_ego_safe * CV.MS_TO_KPH
      ho_floor = float(np.interp(v_kph_aci, CarControllerParams.ACIGAIN_FLOOR_SPEEDS_KPH,
                                 CarControllerParams.ACIGAIN_HANDSOFF_FLOOR_V))
      # Phase 35a: grip-side floor on its own schedule (0.03 from 60 km/h)
      gr_floor = float(np.interp(v_kph_aci, CarControllerParams.ACIGAIN_GRIP_FLOOR35_SPEEDS_KPH,
                                 CarControllerParams.ACIGAIN_GRIP_FLOOR35_V))
      # Phase 25: speed-scheduled full-yield points (see values.py).
      ho_full = float(np.interp(v_kph_aci, CarControllerParams.ACIGAIN_FULL_SPEEDS_KPH,
                                CarControllerParams.ACIGAIN_HANDSOFF_FULL_V))
      # Phase 35a: grip full-yield point 80 driver-Nm from 60 km/h (was 120)
      gr_full = float(np.interp(v_kph_aci, CarControllerParams.ACIGAIN_GRIP_FULL35_SPEEDS_KPH,
                                CarControllerParams.ACIGAIN_GRIP_FULL35_V))
      grip_rate_dn_floor = float(np.interp(v_kph_aci, CarControllerParams.ACIGAIN_GRIP_RATE_DN_SPEEDS_KPH,
                                           CarControllerParams.ACIGAIN_GRIP_RATE_DN_FLOOR_V))
      # Verification round (35c): "recently pinned" alone cannot tell a fresh
      # chase from a stale command when an INVISIBLE touch (driver_tq below
      # the arm level) let the arm decay so the one-shot could not fire while
      # the wheel walked away (closed-loop probe: anchor -> 1.2 s invisible
      # touch -> release delivered 73 -> 78 deg/s with the pin alone). A
      # VM-step bound (v1) was inert there — the stale divergence IS apply
      # advancing at the VM rate against a still wheel, which the bound
      # admits. The discriminator that works is the arm itself: while
      # reanchor_arm >= ARM_FRAMES a stale divergence would have been (or
      # will be) dumped by the one-shot, so the chase is fresh; once the arm
      # has decayed we are in the invisible-touch window by definition. Arm
      # decays 1/frame from 100 -> the fast tail survives ~0.7 s after a
      # clean release, longer than the 0.48 s the Hannam recovery needs.
      anchored_recovery = (CarControllerParams.ANCHORED_RECOVERY_FRAMES > 0
                           and self.frames_since_apply_anchor <= CarControllerParams.ANCHORED_RECOVERY_FRAMES
                           and v_kph_aci >= CarControllerParams.ANCHORED_RECOVERY_SPEED_KPH
                           and self.reanchor_arm >= CarControllerParams.REANCHOR_ARM_FRAMES)
      # Phase 33: blindspot-gated large-correction softening. When op is
      # about to close a LARGE error (a swing that moves the car laterally
      # toward one side), and the BSM radar reports that side occupied
      # (debounced 1 s hold), recover at the tapered rate with the boost
      # off — the correction still happens (rate_up floor = reference
      # 0.004, full authority in ~2.4 s; VM limits bound the move) but
      # never as a snap toward an occupied lane. NOTE the gate keys on the
      # SWING DIRECTION only, not predicted lane incursion — a leftward
      # centering correction with a car in the left blindspot is softened
      # the same as a leftward swing into that lane (separating them needs
      # lane position, which this gate does not consume; cost is bounded:
      # slower, never refused). Sign chain review-verified: positive angle
      # = left; error = apply - wheel > 0 pulls the wheel LEFT. Cars
      # without BSM: flags always False -> inert. Error threshold carries
      # 3.0/2.0 hysteresis — review-measured, a gate chattering at the
      # knee realizes only ~13% of the softening (the ungated 10x rate
      # frames dominate the ratchet); engage-and-hold makes it effective.
      if self.blind_caution_on:
        err_gate = abs(steering_error) >= CarControllerParams.BLIND_CORR_ERR_RELEASE_DEG
      else:
        err_gate = abs(steering_error) >= CarControllerParams.BLIND_CORR_ERR_DEG
      blind_caution = (err_gate
                       and ((self.blind_left_hold > 0) if steering_error > 0
                            else (self.blind_right_hold > 0)))
      self.blind_caution_on = blind_caution
      rate_up_cap_dry = float(np.interp(v_kph_aci, CarControllerParams.ACIGAIN_RATE_UP_CAP_SPEEDS_KPH,
                                        CarControllerParams.ACIGAIN_RATE_UP_CAP_V))
      rate_up_cap_rain = float(np.interp(v_kph_aci, CarControllerParams.ACIGAIN_RATE_UP_CAP_SPEEDS_KPH,
                                         CarControllerParams.ACIGAIN_RATE_UP_CAP_RAIN_V))
      effective_aci_gain = compute_torque_reduction_gain(
        driver_tq, v_kph_aci,
        effective_lat_active, self.aci_gain_last, steering_error,
        blinker_on=blinker_on,
        grip_full=(gr_full if real_grip else ho_full),
        grip_floor=(gr_floor if real_grip else ho_floor),
        # Phase 35a: fast descent only on grip evidence (see values.py GATE_NM)
        rate_dn_floor=(grip_rate_dn_floor if (real_grip or driver_tq >= CarControllerParams.ACIGAIN_GRIP_RATE_DN_GATE_NM)
                       else 0.0),
        curve_deg=self.curve_meas_lp,                # Phase 36
        # Phase 37a: speed-tapered rise cap, one step tighter in rain; rain
        # also moves the yield-curve start down so a firm hand wins sooner
        rate_up_cap=(rate_up_cap_dry + self.rain_w * (rate_up_cap_rain - rate_up_cap_dry)),
        grip_start=(CarControllerParams.ACIGAIN_GRIP_START_NM
                    + self.rain_w * (CarControllerParams.ACIGAIN_GRIP_START_RAIN_NM - CarControllerParams.ACIGAIN_GRIP_START_NM)),
        anchored_recovery=anchored_recovery,         # Phase 35b
        # Phase 28: boost held off only in the first ~0.25 s after an anchor
        # frame (the recovery transient) — G1 review: the full 2 s memory
        # suppressed 26.1% of hands-off drift-recovery frames, worse than
        # the rejected v1 arm gate (14.6%); the short window lands at ~7-9%.
        # Phase 33: also held off while a large correction points at an
        # occupied adjacent lane.
        suppress_error_boost=(real_grip or blind_caution or
                              self.anchor_recent_frames > (CarControllerParams.REANCHOR_RECENT_FRAMES -
                                                           CarControllerParams.BOOST_HOLDOFF_FRAMES)),
        # Phase 29: the recovery/boost tapers apply only with grip evidence
        # (see compute_torque_reduction_gain) — hands-off drift recovery in
        # deficit curves keeps the legacy fast path. Evidence = a recent
        # pressed-anchor episode OR the armed episode memory: the arm term
        # covers the sub-anchor 150-300 Nm release class (sweep: 70°/s
        # delivery at 90 km/h with the anchor-only gate). The arm is NOT a
        # touch guarantee — review-measured, of hands-off arm>=30 frames
        # 49.2% follow a press within 3 s, 31.4% had none for 10+ s, the
        # rest is the F2 identifiability limit (a sustained sub-350 hold
        # and an under-compensated hands-off curve are indistinguishable in
        # torque) — accepted deliberately: the collateral is a sub-second
        # recovery lag on climbing gain only, vs a 70°/s wheel delivery.
        post_grip=(self.anchor_recent_frames > 0
                   or self.reanchor_arm >= CarControllerParams.REANCHOR_ARM_FRAMES
                   # Phase 33: tapered recovery whenever the pending swing
                   # points at an occupied adjacent lane
                   or blind_caution),
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
