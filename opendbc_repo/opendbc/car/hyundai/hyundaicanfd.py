import copy
import numpy as np
from opendbc.car import CanBusBase
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.crc import CRC16_XMODEM
from opendbc.car.hyundai.values import CarControllerParams, HyundaiFlags
from opendbc.sunnypilot.car.hyundai.lead_data_ext import CanFdLeadData


class CanBus(CanBusBase):
  def __init__(self, CP, fingerprint=None, lka_steering=None) -> None:
    super().__init__(CP, fingerprint)

    if lka_steering is None:
      lka_steering = CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG.value if CP is not None else False

    # On the CAN-FD platforms, the LKAS camera is on both A-CAN and E-CAN. LKA steering cars
    # have a different harness than the LFA steering variants in order to split
    # a different bus, since the steering is done by different ECUs.
    self._a, self._e = 1, 0
    if lka_steering:
      self._a, self._e = 0, 1

    self._a += self.offset
    self._e += self.offset
    self._cam = 2 + self.offset

  @property
  def ECAN(self):
    return self._e

  @property
  def ACAN(self):
    return self._a

  @property
  def CAM(self):
    return self._cam


def create_steering_messages(packer, CP, CAN, enabled, lat_active, apply_torque, lkas_icon,
                             apply_angle=0.0, lkas_alt_cam_msg=None,
                             mads_lka_icon=None, effective_aci_gain=None,
                             mads_force_assist=False, cam_invalid=False,
                             lfa_sync_pulse=False):
  """
  Create LKAS_ALT message for the HDA2-ALT + CCNC angle-control platform
  (any Hyundai/Kia with `CCNC | CANFD_LKA_STEER_MSG_ALT` flags; Ioniq 6 N
  2026 is the first member).

  lat_active: carcontroller's `effective_lat_active`. Gated upstream on
    CC.latActive, apply_steer_req, in_passthrough, was_in_reverse, cam_stale,
    and fault_lfa — when False the frame is emitted in passive form.
  effective_aci_gain: ADAS_ACIAnglTqRedcGainVal, computed by
    `compute_torque_reduction_gain` in carcontroller (reference 17-line
    version: torque + v_ego_kph + steering_error → 0.0..1.0). The LKAS_ALT
    packer just forwards it.
  mads_lka_icon: MADS-driven LKA_ICON override (2 = green / 0 = off-but-visible /
    None = mirror camera).
  lfa_sync_pulse: short LFA_BUTTON high pulse emitted on every MADS
    enabled-state edge so the stock gateway toggles the cluster LFA icon.
  cam_invalid: cam_stale or fault_lfa from carcontroller — forces
    LKA_ASSIST=0 and LKAS_ANGLE_ACTIVE=1 even when MADS would otherwise
    keep the icon green.
  """
  ccnc_lka_alt = bool(CP.flags & HyundaiFlags.CCNC) and bool(CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT)

  if ccnc_lka_alt:
    # IMPORTANT: no separate passthrough code path. Earlier builds had an
    # early-return that emitted camera bytes verbatim. It was removed
    # because the two code paths produced *structurally different*
    # LKAS_ALT frames (field ordering from camera passthrough vs. the
    # dict-constructed active frame), and ADAS DRV flagged the format
    # switch at each passthrough→active transition as a fault (observed
    # on routes 3a / 32 / 34 on ccnc-port-prebuilt). Now the SAME dict
    # construction below is used for both active and passive frames,
    # toggled only via `steering_active`.

    # Always-active strategy: when lat_active=True, signal LKAS_ANGLE_ACTIVE=2
    # to MDPS. ACIGain (continuous 0.0..1.0) modulates effort while the
    # ACTIVE flag stays stable.
    steering_active = bool(lat_active)

    if mads_lka_icon is not None:
      icon_value = mads_lka_icon
    else:
      icon_value = 2 if lat_active else None

    if lkas_alt_cam_msg is not None:
      # Mirror camera values, override ADAS_StrAnglReqVal + activation signals.
      # ADAS_ACIAnglTqRedcGainVal reduces MDPS's own torque blending: higher =
      # pure-angle following (ADAS authoritative), lower = MDPS contributes its
      # natural assistance/smoothing. Stage 0 DBC decode on 1.24M frames showed
      # the factory camera itself ALWAYS commands gain = 0.000.
      # ACTUAL behaviour here (Phase 6h COMMIT 0 comment fix — the previous
      # "mirror camera with a 0.15 floor" text described an unimplemented
      # Stage 2b design): carcontroller's compute_torque_reduction_gain output
      # is forwarded as-is via effective_aci_gain — a speed-dependent hands-off
      # ceiling, torque-yield to 0.19 under driver grip, 0.0 when inactive.
      # The hands-off ceiling level itself is tuned in carcontroller
      # (Phase 6h-4 lowers it toward the stock operating point).
      cam_aci_gain = lkas_alt_cam_msg["ADAS_ACIAnglTqRedcGainVal"]
      # Gain pre-computed by carcontroller (with rate limit + quantization).
      # The CCNC carcontroller always passes a non-None effective_aci_gain
      # (compute_torque_reduction_gain output), so this fallback is only
      # reached if a caller violates the contract. Mirror the camera value
      # in that case — safer than fabricating one from undefined constants.
      if effective_aci_gain is None:
        effective_aci_gain = cam_aci_gain

      # Angle command: when not steering, mirror camera's advisory so ADAS
      # DRV sees no delta from op's side. This closes the remaining window
      # where `apply_angle` (set by the rate limiter to actual wheel angle
      # when inactive) could briefly disagree with the camera's frame.
      effective_angle = apply_angle if steering_active else lkas_alt_cam_msg.get("ADAS_StrAnglReqVal", apply_angle)

      # Suppress camera takeover-request signals while op is actively steering.
      # The stock camera raises LKA_WARNING and FCA_SYSWARN in moderate corners
      # — false positives when op has its own model-based plan.
      lka_warning_out = 0 if steering_active else lkas_alt_cam_msg["LKA_WARNING"]
      fca_syswarn_out = 0 if steering_active else lkas_alt_cam_msg["FCA_SYSWARN"]

      lkas_values = {
        "LKA_MODE":                  lkas_alt_cam_msg["LKA_MODE"],
        "LKA_AVAILABLE":             lkas_alt_cam_msg["LKA_AVAILABLE"],
        "LKA_WARNING":               lka_warning_out,
        # Force green icon (LKA_ICON=2) whenever op is actively steering,
        # regardless of MADS vs ACC-only engagement source. The previous
        # logic relied on `mads_lka_icon` which is 0 ("off-but-visible") for
        # ACC-only — leaving op steering visually indistinguishable from
        # off. icon_value still drives the off-state (e.g., MADS off but
        # cruise available → 0).
        "LKA_ICON":                  2 if steering_active else (icon_value if icon_value is not None else lkas_alt_cam_msg["LKA_ICON"]),
        "FCA_SYSWARN":               fca_syswarn_out,
        "TORQUE_REQUEST":            0,
        "STEER_REQ":                 0,
        # LFA_BUTTON drives the stock gateway ECU's internal LFA toggle which in
        # turn controls the cluster's LFA green icon on bus 1's CCNC_0x161/0x162
        # (we don't publish those addresses — see carcontroller's note on the
        # dual-publisher hazard, c6a33de). To keep cluster green in lockstep
        # with MADS, the carcontroller emits a synthetic one-frame `lfa_sync_pulse`
        # on every MADS enabled-state edge. When that pulse is active we force
        # the bit high; otherwise we mirror the camera's bit so any direct
        # press the camera still observes also propagates to the gateway.
        "LFA_BUTTON":                1 if lfa_sync_pulse else lkas_alt_cam_msg["LFA_BUTTON"],
        # Force LKA_ASSIST=1 when MADS is enabled, even during transient
        # passive states (in_passthrough, was_in_reverse, VM rate-limit
        # hold). Without this the cluster's green steering icon falls off
        # during these windows because the camera passthrough value
        # (LKA_ASSIST=0) takes over, even though MADS is still the active
        # assistance source. STEER_REQ and TORQUE_REQUEST stay 0 and
        # ACIGain follows effective_lat_active, so MDPS does not actually
        # pull the wheel — only the icon stays on.
        # When the camera is invalid (cam_stale or fault_lfa), drop the
        # icon to reflect actual MADS health: an unhealthy camera path
        # should not display "everything fine" green.
        "LKA_ASSIST":                0 if cam_invalid else (1 if (steering_active or mads_force_assist) else lkas_alt_cam_msg["LKA_ASSIST"]),
        "DAMP_FACTOR":               lkas_alt_cam_msg["DAMP_FACTOR"],
        "STEER_MODE":                lkas_alt_cam_msg["STEER_MODE"],
        "NEW_SIGNAL_2":              lkas_alt_cam_msg["NEW_SIGNAL_2"],
        "LKAS_BYTE9_HIDDEN":         lkas_alt_cam_msg["LKAS_BYTE9_HIDDEN"],
        # Override stale camera value with explicit passive (1) when the
        # camera is broken, so we never forward a frozen "active=2"
        # snapshot from a dead camera to MDPS.
        "LKAS_ANGLE_ACTIVE":         2 if steering_active else (1 if cam_invalid else lkas_alt_cam_msg["LKAS_ANGLE_ACTIVE"]),
        "HAS_LANE_SAFETY":           lkas_alt_cam_msg["HAS_LANE_SAFETY"],
        "ADAS_StrAnglReqVal":        effective_angle,
        "ADAS_ACIAnglTqRedcGainVal": effective_aci_gain,
        "LKAS_BYTE7_BITS4_5":        3 if steering_active else lkas_alt_cam_msg["LKAS_BYTE7_BITS4_5"],
        "LKAS_BYTE7_BIT7":           1 if steering_active else lkas_alt_cam_msg["LKAS_BYTE7_BIT7"],
        "LKAS_BYTE13":               lkas_alt_cam_msg["LKAS_BYTE13"] if lkas_alt_cam_msg["LKAS_BYTE13"] else (0x09 if steering_active else 0),
        "LKAS_BYTE28":               lkas_alt_cam_msg["LKAS_BYTE28"],
        "LKAS_BYTE29":               lkas_alt_cam_msg["LKAS_BYTE29"],
        "LKAS_BYTE30":               lkas_alt_cam_msg["LKAS_BYTE30"],
        "LKAS_BYTE31":               lkas_alt_cam_msg["LKAS_BYTE31"],
      }
    else:
      # Fallback (startup before camera message received). Same unified
      # gate — everything is either fully passive or fully active.
      # F13: Includes all fields that ADAS DRV may validate, even on boot.
      # steering_active is forced False here (no camera means no blend),
      # so every frame is fully passive = stock-compatible.
      # LKAS_BYTE28..31 = 0x00: drivelog scan of 35,853 LKAS_ALT frames
      # across 30 segments (/tmp/dlog/may09) showed bytes 28-31 always
      # 0x00 from the factory camera, so emitting 0x00 in fallback matches
      # what ADAS DRV expects on boot; the prior 0x92/0x01/0xFF/0xFF was
      # never observed in real captures.
      steering_active = False
      lkas_values = {
        "LKA_MODE": 0,
        "LKA_AVAILABLE": 0,
        "LKA_WARNING": 0,
        "LKA_ICON": 2 if lat_active else 1,
        "FCA_SYSWARN": 0,
        "TORQUE_REQUEST": 0,
        "STEER_REQ": 0,
        "LFA_BUTTON": 0,
        "LKA_ASSIST": 0,
        "DAMP_FACTOR": 0,
        "STEER_MODE": 0,
        "NEW_SIGNAL_2": 0,
        "HAS_LANE_SAFETY": 0,
        "LKAS_BYTE9_HIDDEN": 0x5,
        "LKAS_ANGLE_ACTIVE": 1,
        "ADAS_StrAnglReqVal": 0.0,
        "ADAS_ACIAnglTqRedcGainVal": 0,
        "LKAS_BYTE7_BITS4_5": 0,
        "LKAS_BYTE7_BIT7": 0,
        "LKAS_BYTE13": 0,
        "LKAS_BYTE28": 0x00,
        "LKAS_BYTE29": 0x00,
        "LKAS_BYTE30": 0x00,
        "LKAS_BYTE31": 0x00,
      }
    return [packer.make_can_msg("LKAS_ALT", CAN.ACAN, lkas_values)]

  common_values = {
    "LKA_MODE": 2,
    "LKA_ICON": lkas_icon,
    "TORQUE_REQUEST": apply_torque,
    "LKA_ASSIST": 0,
    "STEER_REQ": 1 if lat_active else 0,
    "STEER_MODE": 0,
    "HAS_LANE_SAFETY": 0,  # hide LKAS settings
    "NEW_SIGNAL_2": 0,
    "DAMP_FACTOR": 100,  # can potentially tuned for better perf [3, 200]
  }

  lkas_values = copy.copy(common_values)
  lkas_values["LKA_AVAILABLE"] = 0

  lfa_values = copy.copy(common_values)
  lfa_values["NEW_SIGNAL_1"] = 0

  ret = []
  if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG:
    lkas_msg = "LKAS_ALT" if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT else "LKAS"
    if CP.openpilotLongitudinalControl:
      ret.append(packer.make_can_msg("LFA", CAN.ECAN, lfa_values))
    ret.append(packer.make_can_msg(lkas_msg, CAN.ACAN, lkas_values))
  else:
    ret.append(packer.make_can_msg("LFA", CAN.ECAN, lfa_values))

  return ret


def create_suppress_lfa(packer, CAN, lfa_block_msg, lka_steering_alt, suppress_lanes=True,
                        override_counter=None, force_lanes=False):
  suppress_msg = "CAM_0x362" if lka_steering_alt else "CAM_0x2a4"
  msg_bytes = 32 if lka_steering_alt else 24

  values = {f"BYTE{i}": lfa_block_msg[f"BYTE{i}"] for i in range(3, msg_bytes) if i != 7}
  # Panda relay blocks the camera's original 0x362/0x2a4 on a_can (see
  # hyundai_canfd.h: check_relay=(a_can)==0), so ADAS DRV only observes
  # our TX stream. Forwarding the camera's native COUNTER through a
  # frame%5==0 downsample produces visible gaps (e.g. 0x1a→0x1c) whenever
  # our TX slot is skipped by one tick — an ECU with a +1-per-frame
  # continuity check treats that as frame corruption. Caller owns a
  # monotonic counter to keep the observed stream clean.
  values["COUNTER"] = (override_counter & 0xFF) if override_counter is not None \
                       else lfa_block_msg["COUNTER"]
  values["SET_ME_0"] = 0
  values["SET_ME_0_2"] = 0
  if suppress_lanes:
    values["LEFT_LANE_LINE"] = 0
    values["RIGHT_LANE_LINE"] = 0
  elif force_lanes:
    values["LEFT_LANE_LINE"] = 3
    values["RIGHT_LANE_LINE"] = 3
  else:
    values["LEFT_LANE_LINE"] = lfa_block_msg["LEFT_LANE_LINE"]
    values["RIGHT_LANE_LINE"] = lfa_block_msg["RIGHT_LANE_LINE"]
  return packer.make_can_msg(suppress_msg, CAN.ACAN, values)


def create_buttons(packer, CP, CAN, cnt, btn):
  values = {
    "COUNTER": cnt,
    "SET_ME_1": 1,
    "CRUISE_BUTTONS": btn,
  }

  bus = CAN.ECAN if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG else CAN.CAM
  return packer.make_can_msg("CRUISE_BUTTONS", bus, values)


def create_acc_cancel(packer, CP, CAN, cruise_info_copy):
  # CAN FD camera-based SCC requires additional signals to be preserved
  # verbatim from the previous SCC_CONTROL frame to avoid checksum or
  # state validation faults. Classic CAN SCC only validates a subset.
  if CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value:
    values = {s: cruise_info_copy[s] for s in [
      "COUNTER",
      "CHECKSUM",
      "NEW_SIGNAL_1",
      "MainMode_ACC",
      "ACCMode",
      "ZEROS_9",
      "CRUISE_STANDSTILL",
      "ZEROS_5",
      "DISTANCE_SETTING",
      "VSetDis",
    ]}
  else:
    values = {s: cruise_info_copy[s] for s in [
      "COUNTER",
      "CHECKSUM",
      "ACCMode",
      "VSetDis",
      "CRUISE_STANDSTILL",
    ]}
  values.update({
    "ACCMode": 4,
    "aReqRaw": 0.0,
    "aReqValue": 0.0,
  })
  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values)


def create_lfahda_cluster(packer, CAN, enabled, lfa_icon):
  values = {
    "HDA_ICON": 1 if enabled else 0,
    "LFA_ICON": lfa_icon,
  }
  return packer.make_can_msg("LFAHDA_CLUSTER", CAN.ECAN, values)


def create_ccnc(packer, CAN, openpilotLongitudinalControl, enabled, hud, leftBlinker, rightBlinker, msg_161, msg_162, msg_1b5,
                is_metric, out, main_cruise_enabled, lfa_icon, op_driving=False):
  """op_driving: True when openpilot is actively providing lateral control. Used
  to conditionally suppress hands-on / takeover / HDP deactivation alerts
  generated by the ADAS camera ECU — these are correct during stock LFA but
  spurious while openpilot is steering. When False, pass-through so stock UX
  (e.g. real takeover warnings during manual driving) is preserved.
  """
  for f in {"FAULT_LSS", "FAULT_HDA", "FAULT_DAS", "FAULT_LFA", "FAULT_DAW", "FAULT_ESS"}:
    msg_162[f] = 0
  if msg_161["ALERTS_2"] == 5:
    msg_161.update({"ALERTS_2": 0, "SOUNDS_2": 0})
  if msg_161["ALERTS_3"] == 17:
    msg_161["ALERTS_3"] = 0
  if msg_161["ALERTS_5"] in (2, 5):
    msg_161["ALERTS_5"] = 0
  if msg_161["SOUNDS_4"] == 2 and msg_161["LFA_ICON"] in (3, 0,):
    msg_161["SOUNDS_4"] = 0

  # Mode-separated suppression — op only. HDP takeover prompt (ALERTS_3=11) and
  # the hands-on variants (ALERTS_2 ∈ {1,2}, ALERTS_3=12) are camera-generated
  # warnings that should NOT be shown while openpilot is actively steering
  # (they cause the "takeover on gentle corner" anxiety). In stock LFA they're
  # correct, so we leave them alone unless op_driving is True.
  if op_driving:
    if msg_161["ALERTS_2"] in (1, 2):
      msg_161.update({"ALERTS_2": 0, "SOUNDS_2": 0})
    if msg_161["ALERTS_3"] in (11, 12):
      msg_161["ALERTS_3"] = 0

  LANE_CHANGE_SPEED_MIN = 8.9408
  anyBlinker = leftBlinker or rightBlinker
  curvature = {i: (31 if i == -1 else 13 - abs(i + 15)) if i < 0 else 15 + i for i in range(-15, 16)}

  msg_161.update({
    "DAW_ICON": 0,
    "LKA_ICON": 0,
    "LFA_ICON": 2 if lfa_icon else 0,
    "CENTERLINE": 1 if lfa_icon else 0,
    "LANELINE_CURVATURE": curvature.get(max(-15, min(int(out.steeringAngleDeg / 4.5), 15)), 14) if lfa_icon and not anyBlinker else 15,
    "LANELINE_LEFT": (0 if not lfa_icon else 1 if not hud.leftLaneVisible else 4 if hud.leftLaneDepart else 6 if anyBlinker else 2),
    "LANELINE_RIGHT": (0 if not lfa_icon else 1 if not hud.rightLaneVisible else 4 if hud.rightLaneDepart else 6 if anyBlinker else 2),
    "LCA_LEFT_ICON": (0 if not lfa_icon or out.vEgo < LANE_CHANGE_SPEED_MIN else 1 if out.leftBlindspot else 2 if anyBlinker else 4),
    "LCA_RIGHT_ICON": (0 if not lfa_icon or out.vEgo < LANE_CHANGE_SPEED_MIN else 1 if out.rightBlindspot else 2 if anyBlinker else 4),
    "LCA_LEFT_ARROW": 2 if leftBlinker else 0,
    "LCA_RIGHT_ARROW": 2 if rightBlinker else 0,
  })

  if lfa_icon and (leftBlinker or rightBlinker):
    leftlaneraw, rightlaneraw = msg_1b5["Info_LftLnPosVal"], msg_1b5["Info_RtLnPosVal"]

    scale_per_m = 15 / 1.7
    leftlane = abs(int(round(15 + (leftlaneraw - 1.7) * scale_per_m)))
    rightlane = abs(int(round(15 + (rightlaneraw - 1.7) * scale_per_m)))

    if msg_1b5["Info_LftLnQualSta"] not in (2, 3):
      leftlane = 0
    if msg_1b5["Info_RtLnQualSta"] not in (2, 3):
      rightlane = 0

    if leftlaneraw == -2.0248375:
      leftlane = 30 - rightlane
    if rightlaneraw == 2.0248375:
      rightlane = 30 - leftlane

    if leftlaneraw == rightlaneraw == 0:
      leftlane = rightlane = 15
    elif leftlaneraw == 0:
      leftlane = 30 - rightlane
    elif rightlaneraw == 0:
      rightlane = 30 - leftlane

    total = leftlane + rightlane
    if total == 0:
      leftlane = rightlane = 15
    else:
      leftlane = round((leftlane / total) * 30)
      rightlane = 30 - leftlane

    msg_161["LANELINE_LEFT_POSITION"] = leftlane
    msg_161["LANELINE_RIGHT_POSITION"] = rightlane

  if hud.leftLaneDepart or hud.rightLaneDepart:
    msg_162["VIBRATE"] = 1

  if openpilotLongitudinalControl:
    if msg_161["ALERTS_3"] in (1, 2, 3, 4, 7, 8, 9, 10):
      msg_161["ALERTS_3"] = 0
    if msg_161["ALERTS_5"] == 4:
      msg_161["ALERTS_5"] = 0
    if msg_161["SOUNDS_3"] == 5:
      msg_161["SOUNDS_3"] = 0

    msg_161.update({
      "SETSPEED": 3 if enabled else 1,
      "SETSPEED_HUD": 0 if not main_cruise_enabled else 2 if enabled else 1,
      "SETSPEED_SPEED": (
        255 if not main_cruise_enabled else
        (40 if is_metric else 25) if (s := round(out.vCruiseCluster * (1 if is_metric else CV.KPH_TO_MPH))) > (145 if is_metric else 90) else s
      ),
      "DISTANCE": hud.leadDistanceBars,
      "DISTANCE_SPACING": 0 if not main_cruise_enabled else 1 if enabled else 3,
      "DISTANCE_LEAD": 0 if not main_cruise_enabled else 2 if enabled and hud.leadVisible else 1 if hud.leadVisible else 0,
      "DISTANCE_CAR": 0 if not main_cruise_enabled else 2 if enabled else 1,
      "SLA_ICON": 0,
      "NAV_ICON": 0,
      "TARGET": 0,
    })

    msg_162["LEAD"] = 0 if not main_cruise_enabled else 2 if enabled else 1
    msg_162["LEAD_DISTANCE"] = msg_1b5["Longitudinal_Distance"]

  return [packer.make_can_msg(msg, CAN.ECAN, data) for msg, data in [("CCNC_0x161", msg_161), ("CCNC_0x162", msg_162)]]


def create_acc_control(packer, CAN, enabled, accel_last, accel, stopping, gas_override, set_speed, hud_control,
                       lead_data: CanFdLeadData, main_cruise_enabled, tuning, cruise_info=None):
  jerk = 5
  jn = jerk / 50
  if not enabled or gas_override:
    a_val, a_raw = 0, 0
  else:
    a_raw = accel  # noqa: F841
    a_val = np.clip(accel, accel_last - jn, accel_last + jn)  # noqa: F841

  values = {
    "ACCMode": 0 if not enabled else (2 if gas_override else 1),
    "MainMode_ACC": 1 if main_cruise_enabled else 0,
    "StopReq": 1 if tuning.stopping else 0,
    "aReqValue": tuning.actual_accel,
    "aReqRaw": tuning.actual_accel,
    "VSetDis": set_speed,
    "JerkLowerLimit": tuning.jerk_lower,
    "JerkUpperLimit": tuning.jerk_upper,

    "ACC_ObjDist": int(lead_data.lead_distance),
    "ACC_ObjRelSpd": lead_data.lead_rel_speed,
    "ObjValid": int(not lead_data.lead_visible),
    "SCC_ObjSta": 0 if not (enabled and lead_data.lead_visible) else (1 if gas_override else 2),
    "SET_ME_2": 0x4,
    "SET_ME_3": 0x3,
    "SET_ME_TMP_64": 0x64,
    "DISTANCE_SETTING": hud_control.leadDistanceBars,
  }
  if cruise_info:
    values.update({s: cruise_info[s] for s in ["ACC_ObjDist", "ACC_ObjRelSpd"]})

  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values)


def create_spas_messages(packer, CAN, left_blink, right_blink):
  ret = []

  values = {
  }
  ret.append(packer.make_can_msg("SPAS1", CAN.ECAN, values))

  blink = 0
  if left_blink:
    blink = 3
  elif right_blink:
    blink = 4
  values = {
    "BLINKER_CONTROL": blink,
  }
  ret.append(packer.make_can_msg("SPAS2", CAN.ECAN, values))

  return ret


def create_fca_warning_light(packer, CAN, frame):
  ret = []

  if frame % 2 == 0:
    values = {
      'AEB_SETTING': 0x1,  # show AEB disabled icon
      'SET_ME_2': 0x2,
      'SET_ME_FF': 0xff,
      'SET_ME_FC': 0xfc,
      'SET_ME_9': 0x9,
    }
    ret.append(packer.make_can_msg("ADRV_0x160", CAN.ECAN, values))
  return ret


def create_adrv_messages(packer, CAN, frame):
  # messages needed to car happy after disabling
  # the ADAS Driving ECU to do longitudinal control

  ret = []

  values = {
  }
  ret.append(packer.make_can_msg("ADRV_0x51", CAN.ACAN, values))

  ret.extend(create_fca_warning_light(packer, CAN, frame))

  if frame % 5 == 0:
    values = {
      'SET_ME_1C': 0x1c,
      'SET_ME_FF': 0xff,
      'SET_ME_TMP_F': 0xf,
      'SET_ME_TMP_F_2': 0xf,
    }
    ret.append(packer.make_can_msg("ADRV_0x1ea", CAN.ECAN, values))

    values = {
      'SET_ME_E1': 0xe1,
      'SET_ME_3A': 0x3a,
    }
    ret.append(packer.make_can_msg("ADRV_0x200", CAN.ECAN, values))

  if frame % 20 == 0:
    values = {
      'SET_ME_15': 0x15,
    }
    ret.append(packer.make_can_msg("ADRV_0x345", CAN.ECAN, values))

  if frame % 100 == 0:
    values = {
      'SET_ME_22': 0x22,
      'SET_ME_41': 0x41,
    }
    ret.append(packer.make_can_msg("ADRV_0x1da", CAN.ECAN, values))

  return ret


def hkg_can_fd_checksum(address: int, sig, d: bytearray) -> int:
  crc = 0
  for i in range(2, len(d)):
    crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ d[i]]) & 0xFFFF
  crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ ((address >> 0) & 0xFF)]) & 0xFFFF
  crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ ((address >> 8) & 0xFF)]) & 0xFFFF
  if len(d) == 8:
    crc ^= 0x5F29
  elif len(d) == 16:
    crc ^= 0x041D
  elif len(d) == 24:
    crc ^= 0x819D
  elif len(d) == 32:
    crc ^= 0x9F5B
  return crc

