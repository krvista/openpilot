"""Test harness: drives the REAL opendbc hyundai CarController (ccnc angle path)
frame-by-frame with synthetic CS/CC, without the full openpilot stack.

Only CANPacker is stubbed (returns (name, bus, values) tuples); everything else
(CarControllerParams, VehicleModel, apply_steer_angle_limits_vm, all Phase 5..14
state machines) is the real code under test.
"""
import os
import sys
import types

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (os.path.join(REPO, 'opendbc_repo'), REPO):
  if p not in sys.path:
    sys.path.insert(0, p)

import numpy as np  # noqa: E402

from opendbc.car import structs, Bus  # noqa: E402
import opendbc.car.hyundai.carcontroller as ccmod  # noqa: E402
from opendbc.car.hyundai.interface import CarInterface  # noqa: E402
from opendbc.car.hyundai.values import HyundaiFlags, CarControllerParams  # noqa: E402

DT = 0.01  # 100 Hz


class FakePacker:
  def __init__(self, *a, **k):
    pass

  def make_can_msg(self, name, bus, values):
    return (name, bus, dict(values))


def make_cp():
  CP = CarInterface.get_non_essential_params('HYUNDAI_IONIQ_6_N')
  # fingerprint-driven flags (0x110 on cam bus) that get_non_essential_params can't see
  CP.flags |= HyundaiFlags.CANFD_LKA_STEERING.value | HyundaiFlags.CANFD_LKA_STEERING_ALT.value
  return CP


CAM_MSG_TEMPLATE = {
  "COUNTER": 0, "LKA_MODE": 2, "LKA_AVAILABLE": 0, "LKA_WARNING": 0,
  "LKA_ICON": 1, "FCA_SYSWARN": 0, "LFA_BUTTON": 0, "LKA_ASSIST": 0,
  "DAMP_FACTOR": 0, "STEER_MODE": 0, "NEW_SIGNAL_2": 0,
  "LKAS_BYTE9_HIDDEN": 0x5, "LKAS_ANGLE_ACTIVE": 1, "HAS_LANE_SAFETY": 0,
  "ADAS_StrAnglReqVal": 0.0, "ADAS_ACIAnglTqRedcGainVal": 0.0,
  "LKAS_BYTE7_BITS4_5": 0, "LKAS_BYTE7_BIT7": 0, "LKAS_BYTE13": 0,
  "LKAS_BYTE28": 0, "LKAS_BYTE29": 0, "LKAS_BYTE30": 0, "LKAS_BYTE31": 0,
}

# steeringPressed debounce as in hyundai carstate for this platform:
# |torque| > 350 sustained 5 frames.
PRESS_THRESHOLD_NM = 350.0
PRESS_DEBOUNCE_FRAMES = 5


class Sim:
  def __init__(self):
    self._orig_packer = ccmod.CANPacker
    ccmod.CANPacker = FakePacker
    try:
      CP = make_cp()
      CP_SP = structs.CarParamsSP()
      self.cc = ccmod.CarController({Bus.pt: 'hyundai_canfd_generated'}, CP, CP_SP)
    finally:
      ccmod.CANPacker = self._orig_packer
    self.cc._cc_sp = types.SimpleNamespace(mads=types.SimpleNamespace(enabled=False))
    self.cam_counter = 0
    self._press_frames = 0
    self.last_msgs = []

  # convenience state accessors
  @property
  def s(self):
    return self.cc

  def lkas_alt(self):
    for m in self.last_msgs:
      if m[0] == "LKAS_ALT":
        return m[2]
    return None

  def effective_lat_active(self):
    m = self.lkas_alt()
    return m is not None and m["LKAS_ANGLE_ACTIVE"] == 2

  def tx_angle(self):
    return self.lkas_alt()["ADAS_StrAnglReqVal"]

  def tx_gain(self):
    return self.lkas_alt()["ADAS_ACIAnglTqRedcGainVal"]

  def step(self, v=15.0, tq=0.0, wheel=0.0, cmd=0.0, lat_active=True,
           pressed=None, blinker=False, lead_dist=None, gear='drive',
           door=False, belt=False, standstill=None, cruise_available=True,
           v_raw=None, enabled=None, bs_l=False, bs_r=False, wheel_rate=0.0):
    """Run one 100 Hz control frame through the real create_canfd_msgs."""
    cc = self.cc
    out = structs.CarState()
    out.vEgoRaw = float(v if v_raw is None else v_raw)
    out.vEgo = float(v) if np.isfinite(v) else 0.0
    out.steeringTorque = float(tq)
    out.steeringAngleDeg = float(wheel)
    out.steeringRateDeg = float(wheel_rate)
    out.leftBlinker = bool(blinker)
    out.leftBlindspot = bool(bs_l)
    out.rightBlindspot = bool(bs_r)
    out.doorOpen = bool(door)
    out.seatbeltUnlatched = bool(belt)
    out.standstill = bool(standstill if standstill is not None else (np.isfinite(v) and v < 0.1))
    out.gearShifter = gear
    out.cruiseState.available = bool(cruise_available)

    # emulate carstate's debounced steeringPressed unless forced
    if pressed is None:
      if np.isfinite(tq) and abs(tq) > PRESS_THRESHOLD_NM:
        self._press_frames += 1
      else:
        self._press_frames = 0
      pressed = self._press_frames >= PRESS_DEBOUNCE_FRAMES
    out.steeringPressed = bool(pressed)

    cam = dict(CAM_MSG_TEMPLATE)
    cam["COUNTER"] = self.cam_counter
    self.cam_counter = (self.cam_counter + 1) % 256

    CS = types.SimpleNamespace(out=out, lkas_alt_cam_msg=cam, fault_lfa=0,
                               msg_161=None, lfa_block_msg=None, is_metric=True,
                               main_cruise_enabled=True)

    CC = structs.CarControl()
    CC.latActive = bool(lat_active)
    CC.enabled = bool(lat_active if enabled is None else enabled)
    CC.actuators.steeringAngleDeg = float(cmd)

    # lead data (LeadDataCarController outputs)
    cc.lead_visible = lead_dist is not None
    cc.lead_distance = float(lead_dist) if lead_dist is not None else 0.0

    self.last_msgs = cc.create_canfd_msgs(bool(lat_active), 0, 0.0, 0.0, False, None, CS, CC)
    cc.frame += 1
    return self.last_msgs


def count_transitions(seq):
  return sum(1 for a, b in zip(seq, seq[1:]) if bool(a) != bool(b))


def run_signal(sim, n, tq_fn=None, **fixed):
  """Step n frames; tq_fn(i)->Nm. Returns dict of traces."""
  traces = {k: [] for k in ('low_speed_cam_latched', 'low_speed_scen_ok', 'in_low_speed_zone',
                            'angle_passive_active', 'blinker_anchor_on', 'parking_mode_active',
                            'traffic_following', 'eff_active', 'apply', 'trim', 'gain')}
  for i in range(n):
    kw = dict(fixed)
    if tq_fn is not None:
      kw['tq'] = tq_fn(i)
    for k, v in list(kw.items()):
      if callable(v) and k != 'tq':
        kw[k] = v(i)
    sim.step(**kw)
    s = sim.s
    traces['low_speed_cam_latched'].append(s.low_speed_cam_latched)
    traces['low_speed_scen_ok'].append(s.low_speed_scen_ok)
    traces['in_low_speed_zone'].append(s.in_low_speed_zone)
    traces['angle_passive_active'].append(s.angle_passive_active)
    traces['blinker_anchor_on'].append(s.blinker_anchor_on)
    traces['parking_mode_active'].append(s.parking_mode_active)
    traces['traffic_following'].append(s.traffic_following)
    traces['eff_active'].append(sim.effective_lat_active())
    traces['apply'].append(s.apply_angle_last)
    traces['trim'].append(s.curve_trim)
    traces['gain'].append(s.aci_gain_last)
  return traces
