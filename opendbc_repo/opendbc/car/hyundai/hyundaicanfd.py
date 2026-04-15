import copy
import numpy as np
from opendbc.car import CanBusBase
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.crc import CRC16_XMODEM
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.lead_data_ext import CanFdLeadData


class CanBus(CanBusBase):
  def __init__(self, CP, fingerprint=None, lka_steering=None) -> None:
    super().__init__(CP, fingerprint)

    if lka_steering is None:
      lka_steering = CP.flags & HyundaiFlags.CANFD_LKA_STEERING.value if CP is not None else False

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


def create_steering_messages(packer, CP, CAN, enabled, lat_active, apply_torque, lkas_icon, apply_angle=0.0, lkas_alt_cam_msg=None,
                             driver_torque_blend=1.0, blinker_on=False, speed_blend=1.0,
                             aci_active=None, aci_gain_ramp=1.0, in_passthrough=False):
  """
  Create LKAS_ALT message for the HDA2-ALT + CCNC angle-control platform
  (any Hyundai/Kia with `CCNC | CANFD_LKA_STEERING_ALT` flags; Ioniq 6 N
  2026 is the first member).

  driver_torque_blend: 1.0 = no driver input, 0.0 = driver fully overriding.
    Used for gradient ACI authority reduction (Toyota LTA TORQUE_WIND_DOWN style).
  blinker_on: reduce ACI authority during turn signals to avoid fighting lane changes.
  speed_blend: 0.0 below ~1 km/h, 1.0 above ~3 km/h, linear in between. Smooth
    replacement for the old binary `aci_speed_ok` 3 km/h gate.
  aci_active: latched ACI engagement state with hysteresis (enter at authority>=0.3,
    exit at authority<0.05) computed in carcontroller. Prevents binary flips of
    LKAS_ANGLE_ACTIVE / LKA_ASSIST at the authority threshold — the root cause of
    the residual low-speed "tick" felt when returning wheel to center. If None,
    falls back to single-threshold computation (legacy).
  aci_gain_ramp: 0.0→1.0 first-order ramp applied to `effective_aci_gain` when
    aci_active transitions False→True, smoothing the ACIGain discontinuity.
  in_passthrough: carcontroller already decided to forward camera values. If True,
    emit the camera passthrough frame (identical to the legacy early-return, but
    now driven by carcontroller's latched state rather than recomputed here).
  """
  ccnc_lka_alt = bool(CP.flags & HyundaiFlags.CCNC) and bool(CP.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT)

  if ccnc_lka_alt:
    # Pure camera passthrough — decided by carcontroller. Two triggers:
    #   (a) driver-relaxed passthrough: op inactive + driver holding wheel
    #   (b) low-speed creep passthrough: v < 2 km/h (emulate stock LFA's
    #       passive behaviour at creep speed; MDPS idle, no op/MDPS fight)
    # Either way we forward camera bytes verbatim so ADAS DRV runs stock LFA.
    # LKA_ICON is overridden to green (2) when op is nominally engaged so
    # the HUD still reflects op's lateral-guidance state to the driver.
    if in_passthrough and lkas_alt_cam_msg is not None:
      lkas_values = {key: lkas_alt_cam_msg[key] for key in lkas_alt_cam_msg if key not in ('CHECKSUM', 'COUNTER')}
      if lat_active:
        lkas_values["LKA_ICON"] = 2
      return [packer.make_can_msg("LKAS_ALT", CAN.ACAN, lkas_values)]

    # Gradient ACI authority: blend based on driver torque, turn signal, and
    # speed. driver_torque_blend=1.0 means full ACI; 0.0 means none (driver
    # fully in control). speed_blend smoothly fades authority to 0 as speed
    # approaches zero, avoiding the discontinuous kick at the 3 km/h boundary.
    authority = driver_torque_blend * speed_blend if lat_active else 0.0
    if blinker_on:
      authority *= 0.2  # large gain reduction during turn signals (natural lane change feel)

    # aci_active drives the angle-control signaling to ADAS DRV. With hysteresis
    # (passed in from carcontroller), LKAS_ANGLE_ACTIVE / LKA_ASSIST / LKAS_BYTE7
    # signals stop binary-flipping at the authority boundary — the ADAS ECU sees
    # a stable engagement state across small authority wiggles at creep speeds.
    if aci_active is None:
      # Legacy path: single threshold (no hysteresis). Kept for safety.
      aci_active = lat_active and authority > 0.05

    # LKA_ICON: stay green (2) whenever openpilot is providing lateral control,
    # independent of driver-torque blending or low-speed state. This matches
    # the operational indicator convention used by official openpilot and
    # prevents the icon from flickering white during minor driver input or
    # near a stop.
    icon_green = lat_active

    if lkas_alt_cam_msg is not None:
      # Mirror camera values, override ADAS_StrAnglReqVal + activation signals.
      # Stage 2b: ADAS_ACIAnglTqRedcGainVal reduces MDPS's own torque blending
      # so higher = pure-angle following (ADAS authoritative), lower = MDPS
      # contributes its natural assistance/smoothing. Stage 0 DBC decode on
      # 1.24M frames showed the camera itself ALWAYS commands gain = 0.000;
      # with cam = 0, the old `max(cam, authority*0.6)` collapsed to op-forced
      # 0.6-1.0 — MDPS in a more authoritative state than stock LFA ever uses,
      # a plausible contributor to the residual low-speed tick and the
      # op-vs-LFA accuracy gap. Mirror the camera's behaviour with a small
      # authority floor (0.15) so MDPS still recognises openpilot as the
      # reference source while doing its own smoothing.
      cam_aci_gain = lkas_alt_cam_msg["ADAS_ACIAnglTqRedcGainVal"]
      op_gain_floor = authority * 0.15 * aci_gain_ramp
      effective_aci_gain = max(cam_aci_gain, op_gain_floor) if aci_active else cam_aci_gain

      lkas_values = {
        "LKA_MODE":                  lkas_alt_cam_msg["LKA_MODE"],
        "LKA_AVAILABLE":             lkas_alt_cam_msg["LKA_AVAILABLE"],
        "LKA_WARNING":               lkas_alt_cam_msg["LKA_WARNING"],
        "LKA_ICON":                  2 if icon_green else lkas_alt_cam_msg["LKA_ICON"],
        "FCA_SYSWARN":               lkas_alt_cam_msg["FCA_SYSWARN"],
        "TORQUE_REQUEST":            0,
        "STEER_REQ":                 0,
        "LFA_BUTTON":                lkas_alt_cam_msg["LFA_BUTTON"],
        "LKA_ASSIST":                1 if aci_active else lkas_alt_cam_msg["LKA_ASSIST"],
        "DAMP_FACTOR":               lkas_alt_cam_msg["DAMP_FACTOR"],
        "STEER_MODE":                lkas_alt_cam_msg["STEER_MODE"],
        "NEW_SIGNAL_2":              lkas_alt_cam_msg["NEW_SIGNAL_2"],
        "LKAS_BYTE9_HIDDEN":         lkas_alt_cam_msg["LKAS_BYTE9_HIDDEN"],
        "LKAS_ANGLE_ACTIVE":         2 if aci_active else (1 if lat_active else lkas_alt_cam_msg["LKAS_ANGLE_ACTIVE"]),
        "HAS_LANE_SAFETY":           lkas_alt_cam_msg["HAS_LANE_SAFETY"],
        "ADAS_StrAnglReqVal":        apply_angle,
        "ADAS_ACIAnglTqRedcGainVal": effective_aci_gain,
        "LKAS_BYTE7_BITS4_5":        3 if aci_active else lkas_alt_cam_msg["LKAS_BYTE7_BITS4_5"],
        "LKAS_BYTE7_BIT7":           1 if aci_active else lkas_alt_cam_msg["LKAS_BYTE7_BIT7"],
        "LKAS_BYTE13":               lkas_alt_cam_msg["LKAS_BYTE13"] if lkas_alt_cam_msg["LKAS_BYTE13"] else (0x09 if aci_active else 0),
        "LKAS_BYTE28":               lkas_alt_cam_msg["LKAS_BYTE28"],
        "LKAS_BYTE29":               lkas_alt_cam_msg["LKAS_BYTE29"],
        "LKAS_BYTE30":               lkas_alt_cam_msg["LKAS_BYTE30"],
        "LKAS_BYTE31":               lkas_alt_cam_msg["LKAS_BYTE31"],
      }
    else:
      # Fallback (startup before camera message received)
      lkas_values = {
        "LKA_MODE": 0,
        "LKA_AVAILABLE": 0,
        "LKA_ICON": 2 if icon_green else 1,  # 2=green when openpilot steers
        "TORQUE_REQUEST": 0,
        "STEER_REQ": 0,
        "LKA_ASSIST": 1 if aci_active else 0,
        "DAMP_FACTOR": 0,
        "STEER_MODE": 0,
        "HAS_LANE_SAFETY": 0,
        "LKAS_BYTE9_HIDDEN": 0x5,
        "LKAS_ANGLE_ACTIVE": 2 if aci_active else 1,
        "ADAS_StrAnglReqVal": apply_angle,
        "ADAS_ACIAnglTqRedcGainVal": authority * 0.15 * aci_gain_ramp if aci_active else 0,
        "LKAS_BYTE7_BITS4_5": 3 if aci_active else 0,
        "LKAS_BYTE7_BIT7": 1 if aci_active else 0,
        "LKAS_BYTE13": 0x09 if aci_active else 0,
        "LKAS_BYTE28": 0x92,
        "LKAS_BYTE29": 0x01,
        "LKAS_BYTE30": 0xFF,
        "LKAS_BYTE31": 0xFF,
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
  if CP.flags & HyundaiFlags.CANFD_LKA_STEERING:
    lkas_msg = "LKAS_ALT" if CP.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT else "LKAS"
    if CP.openpilotLongitudinalControl:
      ret.append(packer.make_can_msg("LFA", CAN.ECAN, lfa_values))
    ret.append(packer.make_can_msg(lkas_msg, CAN.ACAN, lkas_values))
  else:
    ret.append(packer.make_can_msg("LFA", CAN.ECAN, lfa_values))

  return ret


def create_suppress_lfa(packer, CAN, lfa_block_msg, lka_steering_alt, suppress_lanes=True):
  suppress_msg = "CAM_0x362" if lka_steering_alt else "CAM_0x2a4"
  msg_bytes = 32 if lka_steering_alt else 24

  values = {f"BYTE{i}": lfa_block_msg[f"BYTE{i}"] for i in range(3, msg_bytes) if i != 7}
  values["COUNTER"] = lfa_block_msg["COUNTER"]
  values["SET_ME_0"] = 0
  values["SET_ME_0_2"] = 0
  if suppress_lanes:
    values["LEFT_LANE_LINE"] = 0
    values["RIGHT_LANE_LINE"] = 0
  else:
    # Forward camera's lane detection to ADAS DRV so it accepts LKAS_ALT commands
    values["LEFT_LANE_LINE"] = lfa_block_msg["LEFT_LANE_LINE"]
    values["RIGHT_LANE_LINE"] = lfa_block_msg["RIGHT_LANE_LINE"]
  return packer.make_can_msg(suppress_msg, CAN.ACAN, values)


def create_buttons(packer, CP, CAN, cnt, btn):
  values = {
    "COUNTER": cnt,
    "SET_ME_1": 1,
    "CRUISE_BUTTONS": btn,
  }

  bus = CAN.ECAN if CP.flags & HyundaiFlags.CANFD_LKA_STEERING else CAN.CAM
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
