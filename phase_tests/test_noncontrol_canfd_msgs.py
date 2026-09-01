"""CCNC LKAS_ALT frame construction (hyundaicanfd.py) packed through the REAL
CANPacker against the platform's real DBC, plus a CarState CANFD parse smoke
test — a misspelled/missing signal name would raise (parser) or emit a
warning-and-drop (packer) at runtime."""
import types

from phase_tests.harness import make_cp  # noqa: F401 (path setup + CP helper)

from opendbc.can import CANPacker, CANParser
from opendbc.car import structs, Bus
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.values import DBC

DBC_NAME = DBC['HYUNDAI_IONIQ_6_N'][Bus.pt]

CAM_MSG = {
  "COUNTER": 3, "LKA_MODE": 2, "LKA_AVAILABLE": 0, "LKA_WARNING": 1,
  "LKA_ICON": 1, "FCA_SYSWARN": 1, "LFA_BUTTON": 0, "LKA_ASSIST": 0,
  "DAMP_FACTOR": 0, "STEER_MODE": 0, "NEW_SIGNAL_2": 0,
  "LKAS_BYTE9_HIDDEN": 0x5, "LKAS_ANGLE_ACTIVE": 1, "HAS_LANE_SAFETY": 0,
  "ADAS_StrAnglReqVal": 1.2, "ADAS_ACIAnglTqRedcGainVal": 0.0,
  "LKAS_BYTE7_BITS4_5": 0, "LKAS_BYTE7_BIT7": 0, "LKAS_BYTE13": 0,
  "LKAS_BYTE28": 0, "LKAS_BYTE29": 0, "LKAS_BYTE30": 0, "LKAS_BYTE31": 0,
}


def real_env():
  CP = make_cp()
  packer = CANPacker(DBC_NAME)
  CAN = CanBus(CP)
  return CP, packer, CAN


def unpack(msg, name="LKAS_ALT", bus=0):
  """Round-trip a packed frame through the real parser."""
  addr, dat, _ = msg
  cp = CANParser(DBC_NAME, [], bus)
  cp.vl[name]  # lazy-register the message on the parser  # noqa: B018
  cp.update([(0, [(addr, dat, bus)])])
  return cp


class TestLkasAltConstruction:
  def test_active_mirror_frame_packs(self):
    CP, packer, CAN = real_env()
    msgs = hyundaicanfd.create_steering_messages(
      packer, CP, CAN, enabled=True, lat_active=True, apply_torque=0,
      lkas_icon=0, apply_angle=3.5, lkas_alt_cam_msg=dict(CAM_MSG),
      effective_aci_gain=0.42)
    assert len(msgs) == 1
    addr, dat, bus = msgs[0]
    assert addr == 0x110 and len(dat) == 32 and bus == CAN.ACAN
    vl = unpack(msgs[0]).vl["LKAS_ALT"]
    assert vl["LKAS_ANGLE_ACTIVE"] == 2
    assert vl["ADAS_StrAnglReqVal"] == 3.5
    assert abs(vl["ADAS_ACIAnglTqRedcGainVal"] - 0.42) < 0.01
    # camera takeover-request signals suppressed while op steers
    assert vl["LKA_WARNING"] == 0 and vl["FCA_SYSWARN"] == 0
    assert vl["LKA_ICON"] == 2
    assert vl["LKAS_BYTE13"] == 0x09  # cam 0 -> active pattern

  def test_passive_mirror_frame_packs(self):
    CP, packer, CAN = real_env()
    msgs = hyundaicanfd.create_steering_messages(
      packer, CP, CAN, enabled=False, lat_active=False, apply_torque=0,
      lkas_icon=0, apply_angle=7.0, lkas_alt_cam_msg=dict(CAM_MSG),
      effective_aci_gain=0.0, mads_lka_icon=0)
    vl = unpack(msgs[0]).vl["LKAS_ALT"]
    # passive mirrors camera activation + advisory angle, not apply_angle
    assert vl["LKAS_ANGLE_ACTIVE"] == CAM_MSG["LKAS_ANGLE_ACTIVE"]
    assert abs(vl["ADAS_StrAnglReqVal"] - CAM_MSG["ADAS_StrAnglReqVal"]) < 0.05
    assert vl["LKA_WARNING"] == CAM_MSG["LKA_WARNING"]

  def test_boot_fallback_frame_packs_fully_passive(self):
    CP, packer, CAN = real_env()
    # lat_active=True must still emit a passive frame with no camera msg
    msgs = hyundaicanfd.create_steering_messages(
      packer, CP, CAN, enabled=True, lat_active=True, apply_torque=0,
      lkas_icon=0, apply_angle=5.0, lkas_alt_cam_msg=None)
    vl = unpack(msgs[0]).vl["LKAS_ALT"]
    assert vl["LKAS_ANGLE_ACTIVE"] == 1  # never active without a camera frame
    assert vl["ADAS_StrAnglReqVal"] == 0.0
    assert vl["ADAS_ACIAnglTqRedcGainVal"] == 0.0
    assert vl["LKAS_BYTE28"] == 0 and vl["LKAS_BYTE31"] == 0

  def test_none_gain_contract_violation_mirrors_camera(self):
    CP, packer, CAN = real_env()
    cam = dict(CAM_MSG, ADAS_ACIAnglTqRedcGainVal=0.19)
    msgs = hyundaicanfd.create_steering_messages(
      packer, CP, CAN, enabled=True, lat_active=True, apply_torque=0,
      lkas_icon=0, apply_angle=0.0, lkas_alt_cam_msg=cam,
      effective_aci_gain=None)
    vl = unpack(msgs[0]).vl["LKAS_ALT"]
    assert abs(vl["ADAS_ACIAnglTqRedcGainVal"] - 0.19) < 0.01

  def test_cam_invalid_forces_passive_icon_off(self):
    CP, packer, CAN = real_env()
    cam = dict(CAM_MSG, LKAS_ANGLE_ACTIVE=2, LKA_ASSIST=1)  # frozen "active" snapshot
    msgs = hyundaicanfd.create_steering_messages(
      packer, CP, CAN, enabled=True, lat_active=False, apply_torque=0,
      lkas_icon=0, apply_angle=0.0, lkas_alt_cam_msg=cam,
      effective_aci_gain=0.0, cam_invalid=True, mads_force_assist=True)
    vl = unpack(msgs[0]).vl["LKAS_ALT"]
    assert vl["LKAS_ANGLE_ACTIVE"] == 1  # dead camera never forwarded as active
    assert vl["LKA_ASSIST"] == 0


class TestOtherCcncMsgs:
  def test_suppress_lfa_packs_and_counter_override(self):
    CP, packer, CAN = real_env()
    block = {f"BYTE{i}": i for i in range(3, 32) if i != 7}
    block.update({"COUNTER": 0x1A, "LEFT_LANE_LINE": 2, "RIGHT_LANE_LINE": 2})
    msg = hyundaicanfd.create_suppress_lfa(packer, CAN, block, lka_steering_alt=True,
                                           suppress_lanes=False, override_counter=0x105,
                                           force_lanes=True)
    addr, dat, bus = msg
    assert len(dat) == 32 and bus == CAN.ACAN
    vl = unpack(msg, name="CAM_0x362").vl["CAM_0x362"]
    assert vl["COUNTER"] == 0x05  # masked to 8 bit
    assert vl["LEFT_LANE_LINE"] == 3 and vl["RIGHT_LANE_LINE"] == 3

  def test_buttons_and_spas_pack(self):
    CP, packer, CAN = real_env()
    msg = hyundaicanfd.create_buttons(packer, CP, CAN, cnt=5, btn=1)
    assert msg[0] is not None and len(msg[1]) > 0
    spas = hyundaicanfd.create_spas_messages(packer, CAN, left_blink=True, right_blink=False)
    assert len(spas) == 2


class TestCarStateCanfdSmoke:
  def test_real_parser_roundtrip_all_signal_names(self):
    # instantiating the parsers + one full update_canfd sweep touches every
    # message/signal name the CCNC branch reads; a bad name raises here
    CP = make_cp()
    cs = CarState(CP, structs.CarParamsSP())
    parsers = cs.get_can_parsers(CP, structs.CarParamsSP())
    ret, ret_sp = cs.update(parsers)
    assert not ret.lowSpeedAlert  # CANFD path never sets it (minSteerSpeed=1.39)
    # mirror dict exposes every key create_steering_messages reads
    for k in CAM_MSG:
      assert k in cs.lkas_alt_cam_msg
