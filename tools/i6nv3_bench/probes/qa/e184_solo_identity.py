"""e184 (Phase 89): bit-identity record of the solo steering path, to compare two source trees.

Run it from the repo root of each tree (PYTHONPATH=$PWD:$PWD/opendbc_repo) with the same arguments; every run writes
one JSON of SHA-256 digests (plus coverage counters) and `cmp` lists the keys that differ.

  python e184_solo_identity.py cc   OUT.json [--fuzz N] [--seeds a,b,..|none] [--kill NAME|all] [--no-scripted] [--routes DIR --names b4,b5]
  python e184_solo_identity.py ctl  OUT.json [--fuzz N] [--seeds a,b,..] [--routes DIR --names b4,b5]
  python e184_solo_identity.py lagd OUT.json
  python e184_solo_identity.py cmp  A.json B.json
  python e184_solo_identity.py dump cc|ctl KEY OUT.npz ...   (per-frame record of one key, to find the first difference)

cc    the real hyundai CarController.create_canfd_msgs with the real CANPacker: every transmitted (address, bytes, bus)
      per 100 Hz frame, the telemetry words and the controller's own state (all plain attributes) go into the digest.
      Inputs: scripted scenarios, seeded closed-loop fuzz over every input, and (with --routes) logged drives open loop
      (DIR holds the route npz of the bench: rx/<route>.npz).
ctl   the real controlsd.Controls.state_control with the real LatControlAngle and VehicleModel: command, flags, the
      lateral log and every plain state attribute per frame. Inputs: seeded synthetic model / car streams and (with
      --routes) logged drives (DIR holds rx/<route>.npz, la/plan_<route>.npz, vm/<route>.npz).
lagd  the estimator settings lagd would start with, the cached-delay rule and a hash of the estimator's source.

Floats are compared by their bytes (NaN equals NaN, -0.0 differs from 0.0). Attributes that only ever belonged to the
layers removed in Phase 89 are left out of the state digest (DROPPED_*), so the same script runs on a tree from before
the removal (there it also selects the solo profile the old way)."""
import argparse
import collections
import hashlib
import inspect
import json
import math
import os
import struct
import sys
import types

import numpy as np

REPO = os.getcwd()
for _p in (os.path.join(REPO, 'opendbc_repo'), os.path.join(REPO, 'tinygrad_repo'), os.path.join(REPO, 'msgq_repo'), REPO):
  if _p not in sys.path:
    sys.path.insert(0, _p)

DT = 0.01
CHUNK = 20000   # frames per digest

# state that existed only for the layers removed in Phase 89 (prefix match)
DROPPED_CC = ("curve_trim", "trim_resid_lp", "eb_", "xl_", "mdps_comp_", "kick_", "stall_kick_deg", "prev_mads_enabled",
              "solo_mode")
DROPPED_CTL = ("_steer_legacy", "_steer_profile", "_turn_exit", "_lookahead_", "_lat_cmd_lp", "_klane_lp", "_absdc_slow",
               "_delay_match", "_delay_extra_eff")
DROPPED_LAC = ("_fb_integ", "_des_slow", "_fb_err_lp", "_trim_7a6", "direct")


def ser(h, v):
  """Feed a canonical byte form of a plain value into hash h. Returns False for anything that is not plain data."""
  if v is None:
    h.update(b"N")
  elif isinstance(v, (bool, np.bool_)):
    h.update(b"T" if v else b"F")
  elif isinstance(v, (int, np.integer)):
    h.update(b"i%d;" % int(v))
  elif isinstance(v, (float, np.floating)):
    h.update(b"f" + struct.pack("<d", float(v)))
  elif isinstance(v, str):
    h.update(b"s" + v.encode() + b";")
  elif isinstance(v, (bytes, bytearray)):
    h.update(b"b" + bytes(v) + b";")
  elif isinstance(v, (list, tuple, collections.deque)):
    h.update(b"[")
    for x in v:
      if not ser(h, x):
        h.update(b"?")
    h.update(b"]")
  elif isinstance(v, dict):
    h.update(b"{")
    for k in sorted(v, key=str):
      h.update(str(k).encode() + b":")
      if not ser(h, v[k]):
        h.update(b"?")
    h.update(b"}")
  elif isinstance(v, np.ndarray):
    h.update(b"a" + np.ascontiguousarray(v).tobytes())
  else:
    return False
  return True


def ser_vars(h, obj, dropped):
  for k in sorted(vars(obj)):
    if k.startswith(dropped):
      continue
    hk = hashlib.sha256()
    if ser(hk, vars(obj)[k]):
      h.update(k.encode() + b"=" + hk.digest())


class Rec:
  """Chunked digests + optional per-frame dump."""

  def __init__(self, out, key, dump=None):
    self.out, self.key, self.n, self.h = out, key, 0, hashlib.sha256()
    self.dump = dump

  def frame(self, fill, row=None):
    fill(self.h)
    self.n += 1
    if self.dump is not None:
      hh = hashlib.sha256(); fill(hh)
      self.dump.append((hh.hexdigest()[:16],) + tuple(row or ()))
    if self.n % CHUNK == 0:
      self.out[f"{self.key}/{self.n // CHUNK - 1:04d}"] = self.h.hexdigest(); self.h = hashlib.sha256()

  def close(self):
    if self.n % CHUNK:
      self.out[f"{self.key}/{self.n // CHUNK:04d}"] = self.h.hexdigest()
    self.out[f"{self.key}/frames"] = self.n


# ====================================================================================================== car controller
CAM_MSG_TEMPLATE = {
  "COUNTER": 0, "LKA_MODE": 2, "LKA_AVAILABLE": 0, "LKA_WARNING": 0,
  "LKA_ICON": 1, "FCA_SYSWARN": 0, "LFA_BUTTON": 0, "LKA_ASSIST": 0,
  "DAMP_FACTOR": 0, "STEER_MODE": 0, "NEW_SIGNAL_2": 0,
  "LKAS_BYTE9_HIDDEN": 0x5, "LKAS_ANGLE_ACTIVE": 1, "HAS_LANE_SAFETY": 0,
  "ADAS_StrAnglReqVal": 0.0, "ADAS_ACIAnglTqRedcGainVal": 0.0,
  "LKAS_BYTE7_BITS4_5": 0, "LKAS_BYTE7_BIT7": 0, "LKAS_BYTE13": 0,
  "LKAS_BYTE28": 0, "LKAS_BYTE29": 0, "LKAS_BYTE30": 0, "LKAS_BYTE31": 0,
}
CC_KILLS = {
  "PARKING_V2": False, "ENGAGE_TAPER_FRAMES": 0, "FIRST_FRAME_ANCHOR": False, "TX_GOVERNOR_MIRROR": False,
  "LAT_GRANT_GATE": False, "ACI_HOLD_ZERO_UNRELAYED": False, "ROAD_STOP_NO_PARKING": False,
  "PARKING_STALE_SIGNATURE_CLEAR": False, "PASSIVE_INDICATE_CAM_FAULT": False, "ACIGAIN_LOWSPEED_RAISE_V": [0.0, 0.0, 0.0],
  "GW_FOLLOW_BAND_DEG": 0.0, "LFA_HOLD_OFF_CLEAR_FRAMES": 0, "HAND_GAP_CAP_DEG": 0.0,
}


class Car:
  """The real CarController of the Ioniq 6 N driven one frame at a time (as phase_tests/harness.Sim does, with the real
  packer and with every input exposed)."""

  def __init__(self):
    import opendbc.car.hyundai.carcontroller as ccmod
    from opendbc.car import structs, Bus
    from opendbc.car.hyundai.interface import CarInterface
    from opendbc.car.hyundai.values import HyundaiFlags
    self.structs = structs
    CP = CarInterface.get_non_essential_params('HYUNDAI_IONIQ_6_N')
    CP.flags |= HyundaiFlags.CANFD_LKA_STEER_MSG.value | HyundaiFlags.CANFD_LKA_STEER_MSG_ALT.value
    self.cc = ccmod.CarController({Bus.pt: 'hyundai_canfd_generated'}, CP, structs.CarParamsSP())
    # stockLfaWithOp: read only by trees from before Phase 89, where False selects the solo profile
    self.cc._cc_sp = types.SimpleNamespace(mads=types.SimpleNamespace(enabled=False), stockLfaWithOp=False)
    self.cam_counter = 0
    self._press = 0
    self._mdps_can_prev = None
    self.msgs = []

  def step(self, v=15.0, tq=0.0, wheel=0.0, cmd=0.0, lat_active=True, pressed=None, blinker=False, lead_dist=None,
           gear='drive', door=False, belt=False, standstill=None, cruise_available=True, v_raw=None, enabled=None,
           bs_l=False, bs_r=False, wheel_rate=0.0, wiper=False, wiper_stale=False, blinker_right=False,
           cc_blinker_left=False, cc_blinker_right=False, cc_lc_active=False, mdps_angle_2=None, tx_rejected=False,
           mdps_step_can=None, a_ego=0.0, gas=False, lane_prob_min=0.0, lfa_state=-1, gw_req_angle=None, gw_req_lvl2=2,
           cam_lfa_button=0, lat_granted=True, cruise_enabled=False, gps_fix_seen=True, fault_lfa=0, mads=False,
           cam_frozen=False):
    cc, structs = self.cc, self.structs
    out = structs.CarState()
    out.vEgoRaw = float(v if v_raw is None else v_raw)
    out.vEgo = float(v) if np.isfinite(v) else 0.0
    out.steeringTorque = float(tq)
    out.steeringAngleDeg = float(wheel)
    out.steeringRateDeg = float(wheel_rate)
    out.aEgo = float(a_ego)
    out.gasPressed = bool(gas)
    out.leftBlinker = bool(blinker)
    out.rightBlinker = bool(blinker_right)
    out.leftBlindspot = bool(bs_l)
    out.rightBlindspot = bool(bs_r)
    out.doorOpen = bool(door)
    out.seatbeltUnlatched = bool(belt)
    out.standstill = bool(standstill if standstill is not None else (np.isfinite(v) and v < 0.1))
    out.gearShifter = gear
    out.cruiseState.available = bool(cruise_available)
    out.cruiseState.enabled = bool(cruise_enabled)
    if pressed is None:      # carstate's debounce: |torque| > 350 for 5 frames
      self._press = self._press + 1 if (np.isfinite(tq) and abs(tq) > 350.0) else 0
      pressed = self._press >= 5
    out.steeringPressed = bool(pressed)
    cam = dict(CAM_MSG_TEMPLATE)
    cam["LFA_BUTTON"] = int(cam_lfa_button)
    cam["COUNTER"] = self.cam_counter
    if not cam_frozen:
      self.cam_counter = (self.cam_counter + 1) % 256
    CS = types.SimpleNamespace(out=out, lkas_alt_cam_msg=cam, fault_lfa=int(fault_lfa), msg_161=None, lfa_block_msg=None,
                               is_metric=True, main_cruise_enabled=True, wiper_front_on=bool(wiper),
                               wiper_stale=bool(wiper_stale),
                               mdps_angle_2=float(mdps_angle_2) if mdps_angle_2 is not None else (float(wheel) if np.isfinite(wheel) else 0.0),
                               tx_rejected=bool(tx_rejected))
    _m = CS.mdps_angle_2 * 10.0
    _m_can = int(round(_m)) if np.isfinite(_m) else 0
    CS.mdps_angle_2_step_can = int(mdps_step_can) if mdps_step_can is not None else (abs(_m_can - self._mdps_can_prev) if self._mdps_can_prev is not None else 0)
    self._mdps_can_prev = _m_can
    CS.lfa_state = int(lfa_state)
    CS.gw_req_valid = gw_req_angle is not None
    CS.gw_req_angle = float(gw_req_angle) if gw_req_angle is not None else 0.0
    CS.gw_req_lvl2 = int(gw_req_lvl2)
    CC = structs.CarControl()
    CC.latActive = bool(lat_active)
    CC.enabled = bool(lat_active if enabled is None else enabled)
    CC.actuators.steeringAngleDeg = float(cmd)
    CC.leftBlinker = bool(cc_blinker_left)
    CC.rightBlinker = bool(cc_blinker_right)
    sp = cc._cc_sp
    sp.mads.enabled = bool(mads)
    sp.lateralLaneChangeActive = bool(cc_lc_active)
    sp.laneLineProbMin = float(lane_prob_min)
    sp.latGranted = bool(lat_granted)
    sp.gpsFixSeen = bool(gps_fix_seen)
    cc.lead_visible = lead_dist is not None
    cc.lead_distance = float(lead_dist) if lead_dist is not None else 0.0
    self.msgs = cc.create_canfd_msgs(bool(lat_active), 0, 0.0, 0.0, False, None, CS, CC)
    cc.frame += 1
    return self.msgs

  def fill(self, h):
    cc = self.cc
    for m in self.msgs:
      h.update(b"m%d,%d:" % (int(m[0]), int(m[2])) + bytes(m[1]))
    ser(h, [int(cc.steer_debug_flags), int(cc.gw_flags), int(getattr(cc, "lat_passive_reason", 0)),
            bool(cc.lat_passive_indicated), float(cc.tx_angle_last), float(cc.apply_angle_last), float(cc.aci_gain_last)])
    ser_vars(h, cc, DROPPED_CC)

  def row(self):
    cc = self.cc
    return (float(cc.tx_angle_last), float(cc.apply_angle_last), float(cc.aci_gain_last), int(cc.steer_debug_flags),
            int(cc.gw_flags), int(getattr(cc, "lat_passive_reason", 0)), b"|".join(bytes(m[1]) for m in self.msgs).hex())


class Cov:
  def __init__(self):
    self.sf = 0; self.gw = 0; self.pr = set(); self.wire = 0; self.n = 0; self.park = 0; self.tx_nz = 0; self.neg0 = 0

  def add(self, cc):
    self.n += 1
    self.sf |= int(cc.steer_debug_flags); self.gw |= int(cc.gw_flags)
    self.pr.add(int(getattr(cc, "lat_passive_reason", 0)))
    self.wire += bool(cc.steer_debug_flags & 2)
    self.park += bool(cc.parking_mode_active)
    self.tx_nz += abs(cc.tx_angle_last) > 0.5
    self.neg0 += (cc.tx_angle_last == 0.0 and math.copysign(1.0, cc.tx_angle_last) < 0) or \
                 (cc.apply_angle_last == 0.0 and math.copysign(1.0, cc.apply_angle_last) < 0)

  def as_dict(self):
    return dict(frames=self.n, sf_union=hex(self.sf), gw_union=hex(self.gw), passive_reasons=sorted(self.pr),
                wire_active=int(self.wire), parking=int(self.park), tx_beyond_half_deg=int(self.tx_nz), neg_zero=int(self.neg0))


class Plant:
  """The wheel and the factory LFA as the car would answer: the wheel follows the transmitted angle (first order,
  0.18 s, 0.95) in the chosen mode; an LFA button pulse toggles the factory LFA 0.1 s later."""

  def __init__(self):
    self.wheel = 0.0; self.lfa = 0; self.btn_prev = False; self.toggle_in = -1; self.tx_hist = collections.deque([0.0] * 4, maxlen=4)

  def update(self, car, mode, drv_target, pulse):
    cc = car.cc
    tx = float(cc.tx_angle_last) if math.isfinite(cc.tx_angle_last) else 0.0
    active = bool(cc.steer_debug_flags & 2)
    self.tx_hist.append(tx)
    if mode == "follow" and active:
      self.wheel += (0.95 * tx - self.wheel) * DT / 0.18
    elif mode == "lag" and active:
      self.wheel += (0.6 * tx - self.wheel) * DT / 0.9
    elif mode == "driver":
      self.wheel += (drv_target - self.wheel) * DT / 0.3
    elif mode == "free":
      self.wheel -= self.wheel * DT / 1.0
    # "stuck": unchanged
    if pulse and not self.btn_prev:
      self.toggle_in = 10
    self.btn_prev = pulse
    if self.toggle_in >= 0:
      if self.toggle_in == 0:
        self.lfa = 0 if self.lfa else 1
      self.toggle_in -= 1


def _btn(car):
  from opendbc.car.hyundai.carcontroller import GW_PULSE
  return bool(car.cc.gw_flags & GW_PULSE)


def cc_fuzz(car, rec, cov, seed, n):
  """Piecewise-constant random inputs over every argument of step(), wheel and factory LFA in closed loop."""
  r = np.random.RandomState(seed)
  pl = Plant()
  st = dict(v=0.0, v_t=0.0, acc=1.0, cmd=0.0, cmd_mode="hold", cmd_a=0.0, cmd_f=0.5, cmd_ph=0.0, wmode="follow",
            drv=0.0, tq_mode="rest", tq=0.0, lat=False, en=None, lb=False, rb=False, ccl=False, ccr=False, lc=False,
            bsl=False, bsr=False, lead=None, gear="drive", door=False, belt=False, cav=True, cru=False, wip=False,
            wst=False, rej=0, gas=False, lane=0.9, lfa_mode="plant", lfa_fix=0, gw_mode="relay", lvl2=2, cambtn=0,
            grant=True, gps=False, fault=0, mads=False, frozen=0, pressed=None, standstill=None, mdps_mode="same",
            step_can=None, special=0, vraw=None)
  t_next = {k: 0 for k in ("v", "cmd", "wheel", "tq", "lat", "blink", "misc", "lfa", "gw", "rare", "mads")}
  for i in range(n):
    if i >= t_next["v"]:
      c = r.rand()
      st["v_t"] = 0.0 if c < 0.12 else (r.uniform(0, 4) if c < 0.3 else (r.uniform(4, 15) if c < 0.6 else r.uniform(15, 38)))
      st["acc"] = r.uniform(0.3, 3.5); t_next["v"] = i + r.randint(300, 3000)
    dv = float(np.clip(st["v_t"] - st["v"], -st["acc"] * DT, st["acc"] * DT))
    st["v"] += dv
    v = st["v"]
    if i >= t_next["cmd"]:
      st["cmd_mode"] = r.choice(["hold", "sine", "step", "ramp", "wheel", "zero", "negzero"], p=[0.2, 0.3, 0.15, 0.2, 0.05, 0.05, 0.05])
      amp = 400.0 if v < 3 else (90.0 if v < 10 else (25.0 if v < 20 else 8.0))
      st["cmd_a"] = r.uniform(-amp, amp) * r.choice([1.0, 0.3, 0.05]); st["cmd_f"] = r.choice([0.1, 0.25, 0.5, 1.0, 2.5])
      st["cmd_ph"] = r.uniform(0, 6.28); t_next["cmd"] = i + r.randint(50, 1500)
      if st["cmd_mode"] == "step":
        st["cmd"] = st["cmd_a"]
    m = st["cmd_mode"]
    if m == "sine":
      st["cmd"] = st["cmd_a"] * math.sin(6.2832 * st["cmd_f"] * i * DT + st["cmd_ph"])
    elif m == "ramp":
      st["cmd"] += float(np.clip(st["cmd_a"] - st["cmd"], -0.2, 0.2))
    elif m == "wheel":
      st["cmd"] = pl.wheel
    elif m == "zero":
      st["cmd"] = 0.0
    elif m == "negzero":
      st["cmd"] = -0.0
    if i >= t_next["wheel"]:
      st["wmode"] = r.choice(["follow", "lag", "stuck", "driver", "free", "zero", "negzero"], p=[0.5, 0.1, 0.1, 0.15, 0.05, 0.05, 0.05])
      st["drv"] = st["cmd"] + r.uniform(-30, 30) * r.choice([1.0, 0.1])
      st["mdps_mode"] = r.choice(["same", "offset", "none"], p=[0.6, 0.2, 0.2])
      t_next["wheel"] = i + r.randint(100, 2500)
    if st["wmode"] == "zero":
      pl.wheel = 0.0
    elif st["wmode"] == "negzero":
      pl.wheel = -0.0
    if i >= t_next["tq"]:
      st["tq_mode"] = r.choice(["rest", "zero", "hand", "press", "shove", "oppose", "touch"], p=[0.4, 0.1, 0.15, 0.1, 0.05, 0.1, 0.1])
      st["tq"] = r.uniform(-1, 1); st["pressed"] = r.choice([None, None, None, True, False])
      t_next["tq"] = i + r.randint(20, 1200)
    tm = st["tq_mode"]
    sgn = 1.0 if st["tq"] >= 0 else -1.0
    tq = {"rest": r.normal(0, 60), "zero": 0.0, "hand": sgn * (150 + 100 * abs(st["tq"])) + r.normal(0, 30),
          "press": sgn * (400 + 300 * abs(st["tq"])), "shove": sgn * 900.0,
          "oppose": -math.copysign(250.0 + 200 * abs(st["tq"]), st["cmd"] if st["cmd"] else 1.0),
          "touch": (200.0 * sgn if (i // 15) % 8 == 0 else r.normal(0, 40))}[tm]
    if i >= t_next["lat"]:
      st["lat"] = not st["lat"] if r.rand() < 0.8 else st["lat"]
      st["en"] = r.choice([None, None, None, True, False])
      t_next["lat"] = i + (r.randint(500, 6000) if st["lat"] else r.randint(20, 500))
    if i >= t_next["blink"]:
      c = r.rand()
      st["lb"], st["rb"] = (c < 0.12), (0.12 <= c < 0.24) or c > 0.99
      st["ccl"], st["ccr"] = r.rand() < 0.05, r.rand() < 0.05
      st["lc"] = (st["lb"] or st["rb"]) and r.rand() < 0.5
      st["bsl"], st["bsr"] = r.rand() < 0.1, r.rand() < 0.1
      t_next["blink"] = i + r.randint(100, 900)
    if i >= t_next["misc"]:
      st["lead"] = None if r.rand() < 0.5 else r.uniform(3, 80)
      st["gear"] = r.choice(["drive", "reverse", "park", "neutral"], p=[0.9, 0.04, 0.04, 0.02]) if v < 4 else "drive"
      st["door"], st["belt"] = r.rand() < 0.03, r.rand() < 0.03
      st["cav"] = r.rand() < 0.95; st["cru"] = r.rand() < 0.5
      st["wip"], st["wst"] = r.rand() < 0.15, r.rand() < 0.03
      st["gas"] = r.rand() < 0.4; st["lane"] = float(r.choice([0.0, 0.1, 0.3, 0.45, 0.55, 0.8, 0.95]))
      st["grant"] = r.rand() < 0.93; st["gps"] = st["gps"] or r.rand() < 0.2
      st["standstill"] = r.choice([None, None, None, None, True, False])
      st["vraw"] = r.choice([None, None, None, 1.0]) if True else None
      t_next["misc"] = i + r.randint(100, 2000)
    if i >= t_next["lfa"]:
      st["lfa_mode"] = r.choice(["plant", "fix", "stubborn"], p=[0.6, 0.25, 0.15]); st["lfa_fix"] = int(r.choice([0, 1, 2, 3, -1]))
      if r.rand() < 0.3:
        pl.lfa = int(r.choice([0, 1, 2]))      # the car (or the driver) switched it
      t_next["lfa"] = i + r.randint(200, 4000)
    if i >= t_next["gw"]:
      st["gw_mode"] = r.choice(["relay", "none", "other", "wheel"], p=[0.6, 0.15, 0.15, 0.1]); st["lvl2"] = int(r.choice([2, 2, 2, 1, 0]))
      t_next["gw"] = i + r.randint(100, 3000)
    if i >= t_next["mads"]:
      st["mads"] = not st["mads"]; t_next["mads"] = i + r.randint(200, 8000)
    if i >= t_next["rare"]:
      st["special"] = int(r.choice([0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9]))
      st["rej"] = r.randint(1, 8) if st["special"] == 1 else 0
      st["frozen"] = r.randint(5, 80) if st["special"] == 2 else 0
      st["fault"] = 1 if st["special"] == 3 else 0
      st["cambtn"] = 1 if st["special"] == 4 else 0
      st["step_can"] = int(r.choice([3, 5, 9, 20])) if st["special"] == 5 else None
      t_next["rare"] = i + r.randint(30, 1500)
    sp = st["special"]; kw_v = v; kw_cmd = st["cmd"]; kw_wheel = pl.wheel; kw_tq = tq
    if sp == 6 and i % 97 < 3:
      kw_cmd = float(r.choice([float("nan"), float("inf"), -float("inf")]))
    if sp == 7 and i % 89 < 2:
      kw_wheel = float("nan")
    if sp == 8 and i % 83 < 2:
      kw_v = float("nan")
    if sp == 9 and i % 79 < 2:
      kw_tq = float("nan")
    lfa_state = pl.lfa if st["lfa_mode"] != "fix" else st["lfa_fix"]
    if st["lfa_mode"] == "stubborn" and pl.lfa == 0 and i % 250 == 0:
      pl.lfa = 1
    if st["lfa_mode"] == "plant" and pl.lfa == 1 and st["cru"] and v > 15 and i % 300 == 0:
      pl.lfa = 2                               # standby -> active
    gm = st["gw_mode"]
    gw = {"relay": pl.tx_hist[0], "none": None, "other": pl.wheel + 2.0, "wheel": pl.wheel}[gm]
    md = {"same": None, "offset": (pl.wheel + 0.3 if math.isfinite(pl.wheel) else 0.0), "none": None}[st["mdps_mode"]]
    rej = st["rej"] > 0
    if rej:
      st["rej"] -= 1
    frozen = st["frozen"] > 0
    if frozen:
      st["frozen"] -= 1
    car.step(v=kw_v, tq=kw_tq, wheel=kw_wheel, cmd=kw_cmd, lat_active=st["lat"], pressed=st["pressed"], blinker=st["lb"],
             lead_dist=st["lead"], gear=st["gear"], door=st["door"], belt=st["belt"], standstill=st["standstill"],
             cruise_available=st["cav"], v_raw=(None if st["vraw"] is None or not math.isfinite(kw_v) else kw_v * 1.01), enabled=st["en"],
             bs_l=st["bsl"], bs_r=st["bsr"], wheel_rate=0.0, wiper=st["wip"], wiper_stale=st["wst"], blinker_right=st["rb"],
             cc_blinker_left=st["ccl"], cc_blinker_right=st["ccr"], cc_lc_active=st["lc"], mdps_angle_2=md, tx_rejected=rej,
             mdps_step_can=st["step_can"], a_ego=dv / DT, gas=st["gas"], lane_prob_min=st["lane"], lfa_state=lfa_state,
             gw_req_angle=gw, gw_req_lvl2=st["lvl2"], cam_lfa_button=st["cambtn"], lat_granted=st["grant"],
             cruise_enabled=st["cru"], gps_fix_seen=st["gps"], fault_lfa=st["fault"], mads=st["mads"], cam_frozen=frozen)
    pl.update(car, st["wmode"], st["drv"], _btn(car))
    rec.frame(car.fill, car.row() if rec.dump is not None else None); cov.add(car.cc)


def _scripted():
  """name -> (frames, fn(i, plant) -> step kwargs, wheel mode)"""
  S = {}
  kmh = lambda x: x / 3.6

  def sweep(i, pl):          # 0 -> 130 km/h -> 0 with a slow weave, relayed, hands off
    t = i * DT
    v = kmh(130.0) * (t / 60.0 if t < 60 else max(0.0, (120.0 - t) / 60.0))
    return dict(v=v, cmd=6.0 * math.sin(0.5 * t) * (20.0 / max(v, 5.0)), lfa_state=0, gw_req_angle=pl.tx_hist[0], lane_prob_min=0.9,
                tq=40.0 * math.sin(3.1 * t))
  S["sweep"] = (12000, sweep, "follow")

  for name, v_kmh in (("curve40", 40.0), ("curve70", 70.0), ("curve110", 110.0)):
    def curve(i, pl, v_kmh=v_kmh):   # steps, S-curve and exits; wheel follows
      t = i * DT; a = 600.0 / v_kmh
      seg = int(t // 6) % 6
      cmd = [0.0, a, a, -a, 0.3 * a, 0.0][seg] + (0.4 * math.sin(6.28 * 1.0 * t) if seg == 4 else 0.0)
      return dict(v=kmh(v_kmh), cmd=cmd, lfa_state=0, gw_req_angle=pl.tx_hist[0], lane_prob_min=0.9, tq=30.0 * math.sin(2.3 * t))
    S[name] = (7200, curve, "follow")

  def stuck(i, pl):          # the wheel does not answer a growing request (what the removed kick / trim layers reacted to)
    t = i * DT
    return dict(v=kmh(60.0), cmd=min(t, 8.0) * (1 if int(t // 20) % 2 == 0 else -1), lfa_state=0, gw_req_angle=pl.tx_hist[0], tq=20.0)
  S["stuck"] = (6000, stuck, "stuck")

  def lagging(i, pl):
    t = i * DT
    return dict(v=kmh(80.0), cmd=5.0 * math.sin(0.4 * t), lfa_state=0, gw_req_angle=pl.tx_hist[0], tq=10.0 * math.sin(5 * t))
  S["lagging"] = (6000, lagging, "lag")

  def driver(i, pl):         # resting hand, light touches, press, shove, opposing push, release
    t = i * DT; ph = int(t // 5) % 8
    tq = [30.0, 120.0, 250.0, 420.0, 900.0, -300.0, 60.0, 0.0][ph] * (1.0 if int(t // 40) % 2 == 0 else -1.0)
    return dict(v=kmh([25.0, 50.0, 90.0][int(t // 40) % 3]), cmd=4.0 * math.sin(0.3 * t), tq=tq + 15.0 * math.sin(7 * t), lfa_state=0,
                gw_req_angle=pl.tx_hist[0], lane_prob_min=0.9)
  S["driver"] = (12000, driver, "follow")

  def blinkers(i, pl):
    t = i * DT; ph = int(t // 4) % 10
    return dict(v=kmh(70.0), cmd=2.0 * math.sin(0.5 * t), blinker=ph in (1, 2), blinker_right=ph in (5, 6), bs_l=ph == 2, bs_r=ph == 6,
                cc_lc_active=ph in (2, 6), cc_blinker_left=ph == 8, tq=(200.0 if ph in (1, 5) else 20.0), lfa_state=0,
                gw_req_angle=pl.tx_hist[0], lane_prob_min=0.9)
  S["blinkers"] = (8000, blinkers, "follow")

  def engage(i, pl):         # lateral and MADS on / off, cruise edges, grant windows
    t = i * DT
    return dict(v=kmh(55.0), cmd=3.0 + 2.0 * math.sin(0.7 * t), lat_active=(i % 700) < 520, mads=(i % 1400) < 1100,
                cruise_enabled=(i % 900) > 300, lat_granted=not (2000 < i % 3000 < 2200), lfa_state=pl.lfa, gw_req_angle=pl.tx_hist[0],
                lane_prob_min=0.9)
  S["engage"] = (9000, engage, "follow")

  def factory(i, pl):        # the factory LFA comes on, goes active, releases, is unknown; with and without relay
    t = i * DT; ph = int(t // 3) % 10
    lfa = [0, 1, 1, 2, 2, 3, 0, -1, 2, 0][ph]
    relay = ph not in (3, 4, 5, 8)
    return dict(v=kmh(75.0), cmd=4.0 * math.sin(0.6 * t), lfa_state=lfa, gw_req_angle=(pl.tx_hist[0] if relay else pl.wheel + 1.5),
                gw_req_lvl2=2 if ph != 9 else 1, mads=True, cruise_enabled=ph >= 2, cam_lfa_button=int(ph == 6 and i % 300 < 20),
                lane_prob_min=0.9)
  S["factory"] = (9000, factory, "follow")

  def parking(i, pl):        # low speed, big angles, reverse / park, door / belt, GPS not yet seen, creep, lead near and far
    t = i * DT; ph = int(t // 6) % 12
    v = [0.0, 1.0, 2.5, 4.0, 7.0, 9.5, 12.0, 6.0, 2.0, 0.0, 1.5, 5.0][ph]
    gear = "reverse" if ph == 1 else ("park" if ph in (0, 9) else "drive")
    return dict(v=v, cmd=[0.0, 200.0, 320.0, 90.0, 20.0, 5.0, 2.0, 30.0, 280.0, 0.0, 100.0, 10.0][ph] * math.sin(0.4 * t), gear=gear,
                door=ph == 9, belt=ph == 0, gps_fix_seen=t > 30.0, lead_dist=(6.0 if ph in (3, 4) else (40.0 if ph == 5 else None)),
                lane_prob_min=(0.9 if ph in (5, 6) else 0.2), lfa_state=0, gw_req_angle=pl.tx_hist[0], cruise_enabled=ph in (5, 6, 7),
                tq=50.0 * math.sin(1.3 * t))
  S["parking"] = (14400, parking, "follow")

  def faults(i, pl):         # panda rejects, wheel-step spikes, LFA fault, frozen camera counter, wiper, non-finite inputs, -0.0
    t = i * DT; ph = int(t // 3) % 12
    nan = float("nan")
    return dict(v=(nan if (ph == 7 and i % 50 == 0) else kmh(65.0)), cmd=(nan if (ph == 8 and i % 40 < 2) else (-0.0 if ph == 9 else 3.0 * math.sin(0.8 * t))),
                wheel_override=(nan if (ph == 10 and i % 60 == 0) else (-0.0 if ph == 9 else None)),
                tx_rejected=(ph == 1 and i % 100 < 4), mdps_step_can=(6 if (ph == 2 and i % 40 < 2) else None), fault_lfa=int(ph == 3),
                cam_frozen=(ph == 4 and i % 300 < 60), wiper=ph == 5, wiper_stale=ph == 6, tq=(nan if (ph == 11 and i % 70 == 0) else 25.0),
                lfa_state=0, gw_req_angle=pl.tx_hist[0], lane_prob_min=0.9)
  S["faults"] = (10800, faults, "follow")

  def zeros(i, pl):          # exact zeros of both signs on a straight, lateral toggling (sign of zero must be kept)
    ph = int(i // 200) % 6
    return dict(v=kmh([20.0, 50.0, 100.0][int(i // 1200) % 3]), cmd=[0.0, -0.0, 0.0, -0.0, 0.04, -0.04][ph],
                wheel_override=[0.0, -0.0, -0.0, 0.0, None, None][ph], lat_active=(i % 500) < 400, lfa_state=0, gw_req_angle=pl.tx_hist[0], tq=0.0)
  S["zeros"] = (7200, zeros, "follow")
  return S


def cc_scripted(car, rec, cov, name):
  n, fn, mode = _scripted()[name]
  pl = Plant()
  for i in range(n):
    kw = fn(i, pl)
    wo = kw.pop("wheel_override", None)
    kw.setdefault("wheel", pl.wheel if wo is None else wo)
    car.step(**kw)
    pl.update(car, mode, 0.0, _btn(car))
    if wo is not None and math.isfinite(wo):
      pl.wheel = wo
    rec.frame(car.fill, car.row() if rec.dump is not None else None); cov.add(car.cc)


GEAR = {1: "park", 2: "drive", 3: "neutral", 4: "reverse"}


def cc_route(car, rec, cov, path):
  """A logged drive open loop: the logged car / driver / controlsd signals through the car controller."""
  d = dict(np.load(path, allow_pickle=True))
  n = len(d["T"])
  has = lambda k: k in d and len(d[k]) == n  # noqa: E731
  fin = lambda x: x is not None and math.isfinite(x)  # noqa: E731
  for k in range(n):
    g = lambda name, default=None: (d[name][k] if has(name) else default)  # noqa: E731
    lfa = g("lfa_sym"); req = g("cb_req"); lvl = g("cb_lvl2"); md = g("mdps_ang"); ld = g("lead_d")
    car.step(v=float(d["v"][k]) / 3.6, tq=float(d["tq_s"][k]), wheel=float(d["ang"][k]), cmd=float(d["apply"][k]),
             lat_active=bool(d["lat"][k]), pressed=bool(d["pressed"][k]), blinker=bool(d["lb"][k]), blinker_right=bool(d["rb"][k]),
             a_ego=float(d["a_ego"][k]), gas=bool(d["gas"][k]), lane_prob_min=float(d["lane_min"][k]) if fin(float(d["lane_min"][k])) else 0.0,
             bs_l=bool(d["bsl"][k]), bs_r=bool(d["bsr"][k]), wheel_rate=float(d["rate"][k]),
             lfa_state=int(lfa) if fin(lfa) else -1, gw_req_angle=float(req) if fin(req) else None,
             gw_req_lvl2=int(lvl) if fin(lvl) else 0, mads=bool(g("mads_enabled", False)), cruise_enabled=bool(g("cruise", False)),
             cruise_available=bool(g("cruise_avail", True)), gear=GEAR.get(int(g("gear", 2)), "drive"),
             lat_granted=bool(g("lat_granted", True)), gps_fix_seen=bool(g("gps_seen", 1)),
             lead_dist=(float(ld) if (bool(g("lead", False)) and fin(ld)) else None), mdps_angle_2=float(md) if fin(md) else None,
             tx_rejected=bool(g("tx_reject", 0)), cc_lc_active=bool(g("lcs", 0) == 2))
    rec.frame(car.fill, car.row() if rec.dump is not None else None); cov.add(car.cc)


def cc_update_pass(out):
  """One pass through the whole CarController.update() (what card.py calls): the reported actuators and the frames."""
  from opendbc.car import structs
  car = Car(); cc = car.cc
  h = hashlib.sha256(); n = 0
  CC_SP = structs.CarControlSP()
  for i in range(3000):
    t = i * DT
    CS = types.SimpleNamespace()
    o = structs.CarState(); o.vEgoRaw = o.vEgo = 20.0; o.steeringAngleDeg = 0.9 * 4.0 * math.sin(0.6 * (t - 0.2)); o.gearShifter = "drive"
    o.cruiseState.available = True; o.steeringTorque = 30.0 * math.sin(3 * t)
    cam = dict(CAM_MSG_TEMPLATE); cam["COUNTER"] = i % 256
    CS = types.SimpleNamespace(out=o, lkas_alt_cam_msg=cam, fault_lfa=0, msg_161=None, lfa_block_msg=None, is_metric=True,
                               main_cruise_enabled=True, wiper_front_on=False, wiper_stale=False, mdps_angle_2=float(o.steeringAngleDeg),
                               tx_rejected=False, mdps_angle_2_step_can=0, lfa_state=0, gw_req_valid=True,
                               gw_req_angle=float(cc.tx_angle_last), gw_req_lvl2=2)
    CC = structs.CarControl(); CC.latActive = (i % 1000) > 100; CC.enabled = CC.latActive
    CC.actuators.steeringAngleDeg = 4.0 * math.sin(0.6 * t)
    try:
      act, sends = cc.update(CC.as_reader() if hasattr(CC, "as_reader") else CC, CC_SP, CS, int(t * 1e9))
    except Exception as e:  # noqa: BLE001 - the stub car state lacks something update() needs on this host
      out["cc/update/error"] = f"{type(e).__name__}: {e}"[:200]
      return
    for m in sends:
      h.update(b"m%d,%d:" % (int(m[0]), int(m[2])) + bytes(m[1]))
    ser(h, [float(act.steeringAngleDeg), float(act.torque), float(act.accel)])
    ser_vars(h, cc, DROPPED_CC)
    n += 1
  out["cc/update/0000"] = h.hexdigest(); out["cc/update/frames"] = n


def apply_kill(name):
  from opendbc.car.hyundai.values import CarControllerParams as P
  names = list(CC_KILLS) if name == "all" else ([] if name in (None, "", "none") else [name])
  for k in names:
    if not hasattr(P, k):
      raise SystemExit(f"no such switch {k}")
    setattr(P, k, CC_KILLS[k])


def run_cc(a, out, dump=None, only=None):
  apply_kill(a.kill)
  tag = f"cc[{a.kill or 'none'}]"
  jobs = []
  if not a.no_scripted:
    jobs += [(f"{tag}/s/{nm}", lambda car, rec, cov, nm=nm: cc_scripted(car, rec, cov, nm)) for nm in _scripted()]
  for sd in a.seeds:
    jobs.append((f"{tag}/fuzz/{sd}", lambda car, rec, cov, sd=sd: cc_fuzz(car, rec, cov, sd, a.fuzz)))
  if a.routes:
    for nm in a.names:
      jobs.append((f"{tag}/route/{nm}", lambda car, rec, cov, nm=nm: cc_route(car, rec, cov, os.path.join(a.routes, "rx", f"000000{nm}.npz"))))
  for key, fn in jobs:
    if only and key != only:
      continue
    car = Car(); rec = Rec(out, key, dump); cov = Cov()
    fn(car, rec, cov); rec.close()
    out[f"{key}/cov"] = cov.as_dict()
    print(key, rec.n, out[f"{key}/cov"], flush=True)
  if not only and not a.no_scripted and (a.kill in (None, "", "none")):
    cc_update_pass(out)


# ====================================================================================================== controlsd
def _stubs():
  import phase_tests.harness_noncontrol  # noqa: F401  (stub install: params / messaging leaf modules on a dev host)


class _SM(dict):
  def __init__(self, *a):
    super().__init__(*a)
    self.valid = {'driverAssistance': True}
    self.updated = {'extrinsicsCalibration': False, 'deviceMotion': False}
    self.logMonoTime = {}

  def all_checks(self, s=None):
    return True


class _LoC:
  def __init__(self, log):
    self.long_control_state = log.LongitudinalPersonality.standard

  def reset(self):
    pass

  def update(self, *a):
    return 0.0


class _Params:
  def __init__(self):
    self.toggle = True; self.cache = 0.3

  def get_bool(self, k, block=False):
    return self.toggle if k == "LagdToggle" else False

  def get(self, k, block=False, return_default=False):
    return self.cache if k == "LagdValueCache" else None


class _RouteVM:
  """curvature <-> wheel angle from the drives' own ratio per speed (as probes/qa/replay_state_control.VM)"""
  DEG_PER = ([40, 60, 80, 100], [2.85, 3.12, 3.45, 3.74])

  def update_params(self, x, sr):
    pass

  def calc_curvature(self, angle, v, roll):
    return math.degrees(angle) / float(np.interp(v * 3.6, *self.DEG_PER)) * 1e-3

  def get_steer_from_curvature(self, curv, v, roll):
    return math.radians(curv * 1e3 * float(np.interp(v * 3.6, *self.DEG_PER)))


def mk_controls(real_vm=True):
  _stubs()
  from opendbc.car.structs import car
  from opendbc.car.hyundai.interface import CarInterface
  from opendbc.car.vehicle_model import VehicleModel
  from openpilot.cereal import log
  import openpilot.selfdrive.controls.controlsd as cd
  import openpilot.selfdrive.controls.lib.latcontrol_angle as lca
  CPc = CarInterface.get_non_essential_params('HYUNDAI_IONIQ_6_N')
  s = cd.Controls.__new__(cd.Controls)
  s.sm = _SM({
    'carState': types.SimpleNamespace(vEgo=20.0, steeringAngleDeg=0.0, standstill=False, steerFaultTemporary=False,
                                      steerFaultPermanent=False, leftBlinker=False, rightBlinker=False, leftBlindspot=False,
                                      rightBlindspot=False, vCruise=100.0, vCruiseCluster=100.0, canValid=True,
                                      steeringPressed=False, steeringTorque=0.0),
    'vehicleParameters': types.SimpleNamespace(stiffnessFactor=1.0, steerRatio=float(CPc.steerRatio), angleOffsetDeg=0.0, roll=0.0),
    'lateralDelay': types.SimpleNamespace(lateralDelay=0.32),
    'longitudinalPlan': types.SimpleNamespace(aTarget=0.0, shouldStop=False, hasLead=False),
    'modelV2': None,
    'selfdriveState': types.SimpleNamespace(enabled=True, active=True, personality=None, alertHudVisual=None),
    'onroadEvents': [],
    'driverAssistance': types.SimpleNamespace(leftLaneDeparture=False, rightLaneDeparture=False),
  })
  s.CP = types.SimpleNamespace(steerControlType=car.CarParams.SteerControlType.angle, minSteerSpeed=float(CPc.minSteerSpeed),
                               steerAtStandstill=False, openpilotLongitudinalControl=False,
                               lateralTuning=types.SimpleNamespace(which=lambda: 'angle'))
  s.CP_SP = types.SimpleNamespace(pcmCruiseSpeed=True)
  s.CI = types.SimpleNamespace(get_pid_accel_limits=lambda *a: (-1.0, 1.0))
  s.params = _Params()
  s.VM = VehicleModel(CPc) if real_vm else _RouteVM()
  s.LoC = _LoC(log)
  s.LaC = lca.LatControlAngle(CPc, None, None, DT)
  s.get_lat_active = lambda sm: True
  s.get_request_paused = lambda sm: False
  s.calibrated_pose = None
  s.steer_limited_by_safety = False
  s.curvature = s.desired_curvature = s.predicted_lat_accel_ratio = 0.0
  s.bsm_guard = cd.BsmLaneGuard(DT)
  s.lowconf_cap = cd.LowConfRateCap(DT)
  s.lane_dropout = False; s.lane_dropout_clear_frames = 0; s.lane_dropout_frames = 0
  s.lane_dropout_low_frames = 0; s.lane_dropout_armed = True; s.lane_dropout_rearm_frames = 0
  s.lane_dropout_lc_holdoff = 0; s.lane_dropout_hold_k = 0.0
  s.lane_dropout_k_hist = collections.deque(maxlen=int(cd.LANE_DROPOUT_ENTRY_S / DT) + 1)
  s._turn_peak_buf = collections.deque(maxlen=int(cd.TURN_EXIT_S / 0.1)); s._turn_peak_cur = 0.0; s._turn_peak_n = 0
  s._mf = None
  s._model_nonfinite_frames = 0
  s._lc_p = s._lc_i = s._lc_e = 0.0
  s._lat_speed_ok = False
  s.steer_flags = 0
  # read only by trees from before Phase 89 (there: the solo profile, and state the removed layers kept)
  s._steer_legacy = False
  s._lat_cmd_lp = 0.0; s._klane_lp = 0.0; s._absdc_slow = 0.0; s._turn_exit = False
  return s, cd, log


def ctl_fill(s, CC, lg):
  def fill(h):
    a = CC.actuators
    ser(h, [float(a.curvature), float(a.steeringAngleDeg), float(a.torque), float(a.accel), float(s.desired_curvature),
            float(s.curvature), int(s.steer_flags), bool(s.lane_dropout), bool(CC.latActive), bool(CC.longActive),
            bool(CC.leftBlinker), bool(CC.rightBlinker), bool(lg.active), bool(lg.saturated), float(lg.steeringAngleDeg),
            float(lg.steeringAngleDesiredDeg), float(getattr(s.LaC, "_fb_integ", 0.0)), float(s.predicted_lat_accel_ratio)])
    ser_vars(h, s, DROPPED_CTL)
    ser_vars(h, s.LaC, DROPPED_LAC)
    ser_vars(h, s.lowconf_cap, ())
    ser_vars(h, s.bsm_guard, ())
  return fill


class CtlCov:
  def __init__(self):
    self.n = 0; self.flags = 0; self.active = 0; self.dropout = 0; self.sat = 0; self.nonfinite = 0; self.neg0 = 0; self.cmd_nz = 0

  def add(self, s, CC, lg):
    self.n += 1; self.flags |= int(s.steer_flags); self.active += bool(CC.latActive); self.dropout += bool(s.lane_dropout)
    self.sat += bool(lg.saturated); self.nonfinite += s._model_nonfinite_frames > 0
    a = float(CC.actuators.steeringAngleDeg); k = float(CC.actuators.curvature)
    self.neg0 += (a == 0.0 and math.copysign(1.0, a) < 0) or (k == 0.0 and math.copysign(1.0, k) < 0)
    self.cmd_nz += abs(k) > 1e-4

  def as_dict(self):
    return dict(frames=self.n, flags_union=hex(self.flags), lat_active=int(self.active), dropout=int(self.dropout),
                saturated=int(self.sat), nonfinite_model=int(self.nonfinite), neg_zero=int(self.neg0), cmd_beyond_1e4=int(self.cmd_nz))


_T = np.array([10.0 * (k / 32) ** 2 for k in range(33)])
_X = np.array([192.0 * (k / 32) ** 2 for k in range(33)])


def synth_model(r, log, st, v):
  """One modelV2-like message from the generator state st (road curvature, lane offset, confidences, faults)."""
  k = st["k"]; e = st["e"]; psi0 = st["psi"]
  xs = (_X * st["xscale"]).tolist()
  ys = [0.5 * k * x * x + psi0 * x + st["py"] for x in xs]
  kl = k * st["kl_ratio"]
  yc = [0.5 * kl * x * x + e for x in xs]
  hw = st["width"] / 2.0
  wid = [hw * (1.0 + st["wgrow"] * x / 100.0) for x in xs]
  yl = [c - w for c, w in zip(yc, wid)]; yr = [c + w for c, w in zip(yc, wid)]
  vv = max(v, 1.0)
  yaw = [(k + st["dk"] * t) * vv for t in _T]
  psi = [psi0 + (k * t + 0.5 * st["dk"] * t * t) * vv for t in _T]
  vx = [vv] * 33
  act = k + st["act_off"]
  fault = st["fault"]
  if fault == "nan":
    act = float("nan")
  elif fault == "inf":
    act = float("inf") * (1 if st["k"] >= 0 else -1)
  elif fault == "lane_nan":
    yl[int(st["fidx"])] = float("nan")
  elif fault == "pos_nan":
    ys[int(st["fidx"])] = float("nan")
  elif fault == "psi_nan":
    psi[int(st["fidx"])] = float("nan")
  elif fault == "negzero":
    act = -0.0
  elif fault == "zero":
    act = 0.0
  n = 33
  if fault == "short":
    n = 3
  ln = lambda Y: types.SimpleNamespace(x=xs[:n], y=Y[:n])  # noqa: E731
  lines = [ln([y - 3.5 for y in yl]), ln(yl), ln(yr), ln([y + 3.5 for y in yr])]
  if fault == "nolanes":
    lines = []
  lcs = [log.LaneChangeState.off, log.LaneChangeState.preLaneChange, log.LaneChangeState.laneChangeStarting,
         log.LaneChangeState.laneChangeFinishing][st["lcs"]]
  lcd = [log.LaneChangeDirection.none, log.LaneChangeDirection.left, log.LaneChangeDirection.right][st["lcd"]]
  probs = [0.5, st["pl"], st["pr"], 0.5]
  if fault == "prob_nan":
    probs[1] = float("nan")
  if fault == "noprobs":
    probs = []
  return types.SimpleNamespace(
    action=types.SimpleNamespace(desiredCurvature=act),
    position=types.SimpleNamespace(x=xs[:n], y=ys[:n], yStd=[st["ystd"]] * n),
    orientationRate=types.SimpleNamespace(z=yaw[:n]), velocity=types.SimpleNamespace(x=vx[:n]),
    orientation=types.SimpleNamespace(z=psi[:n]),
    laneLines=lines, laneLineProbs=probs,
    meta=types.SimpleNamespace(laneChangeState=lcs, laneChangeDirection=lcd))


def ctl_fuzz(rec, cov, seed, n):
  s, cd, log = mk_controls(real_vm=True)
  r = np.random.RandomState(seed)
  cs = s.sm['carState']; lp = s.sm['vehicleParameters']
  st = dict(k=0.0, k_t=0.0, k_rate=1e-3, dk=0.0, e=0.0, psi=0.0, py=0.0, kl_ratio=1.0, width=3.5, wgrow=0.0, xscale=1.0,
            act_off=0.0, fault=None, fidx=3, lcs=0, lcd=0, pl=0.95, pr=0.95, ystd=0.01)
  g = dict(v=20.0, v_t=20.0, acc=1.0, lat=True, wheel_mode="follow", wheel=0.0, tq=0.0, pressed=False, lb=False, rb=False,
           fault_left=0, paused=False, limited=False)
  nxt = {k: 0 for k in ("road", "lane", "conf", "v", "lat", "drv", "blink", "fault", "misc", "da", "delay", "lc")}
  model = None; mi = 0
  for i in range(n):
    if i >= nxt["v"]:
      c = r.rand()
      g["v_t"] = 0.0 if c < 0.06 else (r.uniform(0, 3) if c < 0.15 else (r.uniform(3, 14) if c < 0.45 else r.uniform(14, 38)))
      g["acc"] = r.uniform(0.3, 3.0); nxt["v"] = i + r.randint(300, 3000)
    g["v"] += float(np.clip(g["v_t"] - g["v"], -g["acc"] * DT, g["acc"] * DT))
    v = g["v"]
    if i >= nxt["road"]:
      kmax = min(0.03, 2.5 / max(v, 3.0) ** 2)
      st["k_t"] = float(r.choice([0.0, 0.0, 1.0, -1.0, 0.3, -0.3])) * r.uniform(0.2, 1.0) * kmax
      st["k_rate"] = r.uniform(0.2, 3.0) * kmax; st["kl_ratio"] = float(r.choice([1.0, 1.0, 0.8, 1.3, 0.0]))
      nxt["road"] = i + r.randint(150, 1500)
    if i >= nxt["lane"]:
      st["e"] = float(r.choice([0.0, 0.1, -0.1, 0.3, -0.3, 0.6, -0.8])) * r.uniform(0.5, 1.2)
      st["psi"] = r.uniform(-0.01, 0.01); st["py"] = r.uniform(-0.3, 0.3) * float(r.choice([0.0, 1.0]))
      st["width"] = float(r.choice([3.5, 3.5, 3.2, 2.4, 4.8])); st["wgrow"] = float(r.choice([0.0, 0.0, 0.0, 0.5, -0.4]))
      st["act_off"] = r.uniform(-1, 1) * float(r.choice([0.0, 2e-5, 2e-4])); st["xscale"] = float(r.choice([1.0, 1.0, 1.0, 0.2, 0.05]))
      nxt["lane"] = i + r.randint(100, 1200)
    if i >= nxt["conf"]:
      st["pl"] = float(r.choice([0.95, 0.95, 0.8, 0.6, 0.45, 0.28, 0.22, 0.14, 0.08, 0.02]))
      st["pr"] = float(r.choice([0.95, 0.95, 0.8, 0.6, 0.45, 0.28, 0.22, 0.14, 0.08, 0.02]))
      st["ystd"] = float(r.choice([0.01, 0.01, 0.04, 0.1, 0.2, 0.35]))
      nxt["conf"] = i + r.randint(20, 900)
    if i >= nxt["lc"]:
      st["lcs"] = int(r.choice([0, 0, 0, 0, 1, 2, 3])); st["lcd"] = int(r.choice([0, 1, 2])) if st["lcs"] else 0
      nxt["lc"] = i + (r.randint(100, 700) if st["lcs"] else r.randint(300, 3000))
    if i >= nxt["fault"]:
      st["fault"] = r.choice([None, None, None, None, "nan", "inf", "lane_nan", "pos_nan", "psi_nan", "short", "nolanes", "noprobs",
                              "prob_nan", "negzero", "zero"])
      st["fidx"] = int(r.randint(0, 33)); g["fault_left"] = int(r.choice([1, 5, 30, 300])) if st["fault"] else 0
      nxt["fault"] = i + g["fault_left"] + r.randint(100, 2500)
    if g["fault_left"] > 0:
      g["fault_left"] -= 1
      if g["fault_left"] == 0:
        st["fault"] = None
    if i >= nxt["lat"]:
      g["lat"] = not g["lat"] if r.rand() < 0.8 else g["lat"]
      nxt["lat"] = i + (r.randint(600, 8000) if g["lat"] else r.randint(10, 400))
    if i >= nxt["drv"]:
      g["wheel_mode"] = r.choice(["follow", "follow", "stuck", "random", "zero", "negzero"])
      g["pressed"] = r.rand() < 0.15; g["tq"] = float(r.choice([0.0, 50.0, -50.0, 250.0, -250.0, 500.0, -500.0]))
      nxt["drv"] = i + r.randint(50, 1500)
    if i >= nxt["blink"]:
      c = r.rand(); g["lb"], g["rb"] = c < 0.08, 0.08 <= c < 0.16
      cs.leftBlindspot, cs.rightBlindspot = bool(r.rand() < 0.1), bool(r.rand() < 0.1)
      nxt["blink"] = i + r.randint(100, 900)
    if i >= nxt["da"]:
      da = s.sm['driverAssistance']; c = r.rand()
      da.leftLaneDeparture, da.rightLaneDeparture = bool(c < 0.08), bool(0.08 <= c < 0.16)
      s.sm.valid['driverAssistance'] = bool(r.rand() < 0.95)
      nxt["da"] = i + r.randint(50, 1500)
    if i >= nxt["delay"]:
      s.sm['lateralDelay'].lateralDelay = float(r.choice([0.32, 0.32, 0.3, 0.45, 0.5, 0.2, 0.0, float("nan")]))
      s.params.toggle = bool(r.rand() < 0.85); s.params.cache = float(r.choice([0.3, 0.32, 0.5]))
      nxt["delay"] = i + r.randint(300, 6000)
    if i >= nxt["misc"]:
      cs.steerFaultTemporary = bool(r.rand() < 0.03); cs.steerFaultPermanent = bool(r.rand() < 0.01)
      cs.standstill = bool(r.rand() < 0.03)
      lp.roll = float(r.choice([0.0, 0.0, 0.02, -0.03])); lp.angleOffsetDeg = float(r.choice([0.0, 0.0, 0.4, -0.7]))
      lp.steerRatio = float(r.choice([13.0, 14.5, 16.0])); lp.stiffnessFactor = float(r.choice([1.0, 0.8, 1.2]))
      g["paused"] = bool(r.rand() < 0.1); g["limited"] = bool(r.rand() < 0.15)
      s.sm['selfdriveState'].enabled = bool(r.rand() < 0.9)
      nxt["misc"] = i + r.randint(50, 1500)
    st["k"] += float(np.clip(st["k_t"] - st["k"], -st["k_rate"] * DT, st["k_rate"] * DT))
    st["dk"] = (st["k_t"] - st["k"]) * 0.5
    if i % 5 == 0 or model is None:
      model = synth_model(r, log, st, v); mi += 1; s.sm.logMonoTime['modelV2'] = mi
    # the car: the wheel follows the previous command (first order), or not
    wm = g["wheel_mode"]
    want = math.degrees(s.VM.get_steer_from_curvature(-s.desired_curvature, max(v, 1.0), 0.0)) if math.isfinite(s.desired_curvature) else 0.0
    if wm == "follow":
      g["wheel"] += (0.95 * want - g["wheel"]) * DT / 0.18
    elif wm == "random":
      g["wheel"] += r.normal(0, 0.3)
    elif wm == "zero":
      g["wheel"] = 0.0
    elif wm == "negzero":
      g["wheel"] = -0.0
    cs.vEgo = float(v)
    # a non-finite wheel angle or road roll latches the command for good (nothing clears it): only at the very end
    tail = i >= n - 3000
    cs.steeringAngleDeg = float("nan") if (tail and i % 1000 == 500) else float(g["wheel"])
    if tail and i % 1000 == 900:
      lp.roll = float("nan")
    cs.steeringPressed = g["pressed"]; cs.steeringTorque = g["tq"]; cs.leftBlinker = g["lb"]; cs.rightBlinker = g["rb"]
    s.get_lat_active = (lambda sm, a=g["lat"]: a)
    s.get_request_paused = (lambda sm, a=g["paused"]: a)
    s.steer_limited_by_safety = g["limited"]
    s.sm['modelV2'] = model
    CC, lg = cd.Controls.state_control(s)
    rec.frame(ctl_fill(s, CC, lg), (float(CC.actuators.curvature), float(CC.actuators.steeringAngleDeg), int(s.steer_flags)) if rec.dump is not None else None)
    cov.add(s, CC, lg)


def ctl_route(rec, cov, S, route):
  """A logged drive: carState (100 Hz route npz) + modelV2 rebuilt from the plan npz, as probes/qa/e180 does, but with the
  real LatControlAngle and everything recorded in float64."""
  s, cd, log = mk_controls(real_vm=False)
  import openpilot.selfdrive.controls.lib.lowconf_rate_cap as lrc
  lrc.LOWCONF_CAP_OWN_STATE = True
  P = dict(np.load(f"{S}/la/plan_{route}.npz"))
  d = dict(np.load(f"{S}/rx/{route}.npz", allow_pickle=True))
  try:
    LD = np.load(f"{S}/vm/{route}.npz", allow_pickle=True)['ld']
    LD = LD if len(LD) > 10 else None
  except Exception:  # noqa: BLE001
    LD = None
  LCS = {0: log.LaneChangeState.off, 1: log.LaneChangeState.preLaneChange, 2: log.LaneChangeState.laneChangeStarting,
         3: log.LaneChangeState.laneChangeFinishing}

  def model_at(i):
    ln = lambda y: types.SimpleNamespace(x=P['LX'][i].tolist(), y=y)  # noqa: E731
    j = min(np.searchsorted(d['T'], P['T'][i]), len(d['T']) - 1)
    return types.SimpleNamespace(
      action=types.SimpleNamespace(desiredCurvature=float(P['K'][i])),
      position=types.SimpleNamespace(x=P['PX'][i].tolist(), y=P['PY'][i].tolist(), yStd=[0.01] * 33),
      orientationRate=types.SimpleNamespace(z=P['OR'][i].tolist()), velocity=types.SimpleNamespace(x=P['VX'][i].tolist()),
      orientation=types.SimpleNamespace(z=P['PSI'][i].tolist()),
      laneLines=[ln((P['L1'][i] - 3.5).tolist()), ln(P['L1'][i].tolist()), ln(P['L2'][i].tolist()), ln((P['L2'][i] + 3.5).tolist())],
      laneLineProbs=P['PR'][i].tolist(),
      meta=types.SimpleNamespace(laneChangeState=LCS[int(d['lcs'][j])], laneChangeDirection=log.LaneChangeDirection.none))

  Tr = d['T']; n = len(Tr)
  cs = s.sm['carState']; s.desired_curvature = float(d['k_ctrl'][0]); s._lat_cmd_lp = s.desired_curvature
  pi = np.maximum(np.searchsorted(P['T'], Tr, side='right') - 1, 0)
  ldi = np.maximum(np.searchsorted(LD[:, 0], Tr, side='right') - 1, 0) if LD is not None else None
  cur = -1; m = None
  for j in range(n):
    cs.vEgo = float(d['v'][j]) / 3.6; cs.steeringAngleDeg = float(d['ang'][j]); cs.steeringPressed = bool(d['pressed'][j])
    cs.steeringTorque = float(d['tq_s'][j]); cs.leftBlinker = bool(d['lb'][j]); cs.rightBlinker = bool(d['rb'][j])
    cs.leftBlindspot = bool(d['bsl'][j]); cs.rightBlindspot = bool(d['bsr'][j])
    lat = bool(d['lat'][j]); s.get_lat_active = (lambda sm, a=lat: a)
    s.sm['lateralDelay'].lateralDelay = float(LD[ldi[j], 1]) if LD is not None else 0.3
    i = pi[j]
    if i != cur:
      m = model_at(i); cur = i; s.sm.logMonoTime['modelV2'] = int(i)
    s.sm['modelV2'] = m
    CC, lg = cd.Controls.state_control(s)
    rec.frame(ctl_fill(s, CC, lg), (float(CC.actuators.curvature), float(CC.actuators.steeringAngleDeg), int(s.steer_flags)) if rec.dump is not None else None)
    cov.add(s, CC, lg)


def ctl_schema(out):
  """CarControlSP as controlsd writes it: the profile field left alone equals the field written False."""
  _stubs()
  from openpilot.cereal import custom
  a = custom.CarControlSP.new_message(); a.laneLineProbMin = 0.5; a.latGranted = True
  b = custom.CarControlSP.new_message(); b.laneLineProbMin = 0.5; b.latGranted = True
  if hasattr(b, "stockLfaWithOp"):
    b.stockLfaWithOp = False
  out["ctl/schema/carControlSP_default_equals_false"] = a.to_bytes() == b.to_bytes()
  out["ctl/schema/carControlSP_bytes"] = hashlib.sha256(b.to_bytes()).hexdigest()


def run_ctl(a, out, dump=None, only=None):
  jobs = [(f"ctl/fuzz/{sd}", lambda rec, cov, sd=sd: ctl_fuzz(rec, cov, sd, a.fuzz)) for sd in a.seeds]
  if a.routes:
    jobs += [(f"ctl/route/{nm}", lambda rec, cov, nm=nm: ctl_route(rec, cov, a.routes, f"000000{nm}")) for nm in a.names]
  for key, fn in jobs:
    if only and key != only:
      continue
    rec = Rec(out, key, dump); cov = CtlCov()
    fn(rec, cov); rec.close()
    out[f"{key}/cov"] = cov.as_dict()
    print(key, rec.n, out[f"{key}/cov"], flush=True)
  if not only:
    ctl_schema(out)


# ====================================================================================================== lagd
def run_lagd(a, out):
  _stubs()
  import openpilot.cereal.messaging as messaging
  from opendbc.car.structs import car
  import openpilot.selfdrive.locationd.lagd as lagd
  CP = types.SimpleNamespace(brand="hyundai", steerControlType="angle", steerActuatorDelay=0.1)
  try:
    prof = lagd.lagd_profile(CP, solo=True)     # trees from before Phase 89
  except TypeError:
    prof = lagd.lagd_profile(CP)
  out["lagd/profile"] = {k: (float(v) if v is not None else None) for k, v in sorted(prof.items())}
  out["lagd/profile_other_car"] = lagd.lagd_profile(types.SimpleNamespace(brand="toyota", steerControlType="torque"))
  est = lagd.LateralLagEstimator(CP, 0.05, **prof)
  h = hashlib.sha256(); ser_vars(h, est, ()); out["lagd/estimator_initial_state"] = h.hexdigest()
  out["lagd/estimator_source"] = hashlib.sha256(inspect.getsource(lagd.LateralLagEstimator).encode()).hexdigest()
  for lag in (0.32, 0.49, 0.50, 0.69):
    msg = messaging.new_message('lateralDelay')
    ld = msg.lateralDelay
    ld.lateralDelayEstimate = lag; ld.lateralDelay = lag; ld.validBlocks = 12; ld.status = 'estimated'; ld.version = lagd.VERSION
    CPm = car.CarParams.new_message(carFingerprint="HYUNDAI_IONIQ_6_N")
    store = {"LiveDelay": msg.to_bytes(), "CarParamsPrevRoute": CPm.to_bytes()}
    p = types.SimpleNamespace(get=lambda k, **kw: store.get(k), remove=lambda k: store.pop(k, None))
    try:
      r = lagd.retrieve_initial_lag(p, CPm, True)
    except TypeError:
      r = lagd.retrieve_initial_lag(p, CPm)
    out[f"lagd/cache/{lag}"] = [None if r is None else [struct.pack("<d", float(r[0])).hex(), int(r[1])], "LiveDelay" in store]
  # the published value for a learned delay beyond the cap, and the seed before anything is learned
  est.reset(0.62, est.min_valid_block_count)
  out["lagd/published_capped"] = struct.pack("<d", float(est.get_msg(True).lateralDelay.lateralDelay)).hex()
  est2 = lagd.LateralLagEstimator(CP, 0.05, **prof)
  out["lagd/published_seed"] = struct.pack("<d", float(est2.get_msg(True).lateralDelay.lateralDelay)).hex()


# ====================================================================================================== main
def cmp(a, b):
  A = json.load(open(a)); B = json.load(open(b))
  bad = sorted(k for k in set(A) | set(B) if A.get(k) != B.get(k))
  frames = sum(v for k, v in A.items() if k.endswith("/frames"))
  print(f"{len(A)} keys / {frames} frames in {a}; {len(B)} keys in {b}; {len(bad)} differ")
  for k in bad[:40]:
    print("  DIFF", k, str(A.get(k))[:70], "|", str(B.get(k))[:70])
  return 1 if bad else 0


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("what", choices=["cc", "ctl", "lagd", "cmp", "dump"])
  ap.add_argument("args", nargs="*")
  ap.add_argument("--fuzz", type=int, default=100000)
  ap.add_argument("--seeds", default="1")
  ap.add_argument("--kill", default=None)
  ap.add_argument("--routes", default=os.environ.get("REPLAY_SCRATCH"))
  ap.add_argument("--names", default="")
  ap.add_argument("--no-scripted", action="store_true")
  ap.add_argument("--only", default=None)
  a = ap.parse_args()
  a.seeds = [int(x) for x in a.seeds.split(",") if x and x != "none"]
  a.names = [x for x in a.names.split(",") if x]
  if not a.names:
    a.routes = None
  if a.what == "cmp":
    raise SystemExit(cmp(a.args[0], a.args[1]))
  out = {}
  if a.what == "dump":
    kind, key, path = a.args
    rows = []
    (run_cc if kind == "cc" else run_ctl)(a, out, dump=rows, only=key)
    np.save(path, np.array(rows, dtype=object), allow_pickle=True)
    print("dumped", len(rows), "frames to", path)
    return
  {"cc": run_cc, "ctl": run_ctl, "lagd": run_lagd}[a.what](a, out) if a.what == "lagd" else {"cc": run_cc, "ctl": run_ctl}[a.what](a, out, only=a.only)
  with open(a.args[0], "w") as f:
    json.dump(out, f, indent=0, sort_keys=True)
  print("wrote", a.args[0], len(out), "keys")


if __name__ == "__main__":
  main()
