#!/usr/bin/env python3
"""i6nv3 bench: panda firmware + safety-config check. Run ON THE DEVICE.

Offroad (desk bench 1a):  python3 tools/i6nv3_bench/panda_check.py
  -> firmware signature vs the branch's expected firmware, health, current
     safety mode/param (expect noOutput/elm327 when no car is fingerprinted).
Onroad  (car bench 1b):   python3 tools/i6nv3_bench/panda_check.py --onroad 20
  -> samples pandaStates for N s: safety model/param (decoded), model id from
     CarParamsSP, controlsAllowed(Lateral), rx-invalid / tx-blocked deltas.
Prints PASS/FAIL lines; nothing here transmits on the bus.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, "/data/openpilot")
sys.path.insert(0, "/data/openpilot/opendbc_repo")
sys.path.insert(0, "/data/openpilot/panda")

EXPECTED_MODEL_ID = 11   # HYUNDAI_IONIQ_6_N in hyundai_canfd_angle_models.h
EXPECTED_BITS = {"EV_GAS": 1, "CANFD_LKA_STEER_MSG": 16, "CANFD_ALT_BUTTONS": 32,
                 "CANFD_LKA_STEER_MSG_ALT": 128, "CCNC": 2048}
ALL_BITS = {1: "EV_GAS", 2: "HYBRID_GAS", 4: "LONG", 8: "CAMERA_SCC", 16: "CANFD_LKA_STEER_MSG",
            32: "CANFD_ALT_BUTTONS", 64: "ALT_LIMITS", 128: "CANFD_LKA_STEER_MSG_ALT", 256: "FCEV_GAS",
            512: "ALT_LIMITS_2", 1024: "CANFD_ANGLE_STEERING", 2048: "CCNC"}


def decode(param: int) -> str:
  return "|".join(n for b, n in ALL_BITS.items() if param & b) or "none"


def ok(cond, msg):
  print(("PASS " if cond else "FAIL ") + msg)
  return bool(cond)


def check_offroad():
  from panda import Panda
  from panda.python.constants import FW_PATH, McuType
  fn = os.path.join(FW_PATH, McuType.H7.config.app_fn)
  print(f"expected firmware file: {fn}  exists={os.path.exists(fn)}")
  if not os.path.exists(fn):
    ok(False, "no built firmware — boot build not finished or failed (check /tmp/launch_log)")
    return False
  exp = Panda.get_signature_from_firmware(fn)
  serials = Panda.list()
  if not ok(len(serials) >= 1, f"panda enumerated: {serials}"):
    return False
  p = Panda(serials[0])
  sig = b"" if p.bootstub else p.get_signature()
  print(f"panda type={p.get_type()!r} version={p.get_version()} bootstub={p.bootstub}")
  print(f"signature: panda={sig.hex()[:16]} expected={exp.hex()[:16]}")
  good = ok(sig == exp, "firmware signature matches this branch's build (pandad flashed it)")
  h = p.health()
  print(f"health: uptime={h['uptime']} v={h['voltage']}mV faults={h['faults']} fault_status={h['fault_status']} "
        f"harness={h['car_harness_status']} safety_mode={h['safety_mode']} param={h['safety_param']} "
        f"heartbeat_lost={h['heartbeat_lost']}")
  ok(h['fault_status'] == 0, "no panda fault")
  return good


def check_onroad(seconds: int):
  import openpilot.cereal.messaging as messaging
  from openpilot.common.params import Params
  from opendbc.car.structs import car
  try:
    from openpilot.cereal import custom
  except Exception:
    custom = None
  params = Params()
  cp_b = params.get("CarParams")
  exp_param = None; exp_sp = None
  if cp_b:
    CP = messaging.log_from_bytes(cp_b, car.CarParams)
    exp_param = CP.safetyConfigs[-1].safetyParam
    print(f"CarParams: {CP.carFingerprint} safetyConfigs={[(str(c.safetyModel), c.safetyParam) for c in CP.safetyConfigs]}")
    ok(exp_param & 2048, f"CarParams safetyParam has CCNC(2048): {decode(exp_param)}")
    ok(all(exp_param & b for b in EXPECTED_BITS.values()), f"CarParams safetyParam == expected i6n set ({sum(EXPECTED_BITS.values())})")
  else:
    ok(False, "CarParams not set yet (car not fingerprinted)")
  cpsp_b = params.get("CarParamsSP")
  if cpsp_b and custom is not None:
    CPSP = messaging.log_from_bytes(cpsp_b, custom.CarParamsSP)
    exp_sp = CPSP.safetyParam
    model_id = (exp_sp >> 4) & 0xF
    ok(model_id == EXPECTED_MODEL_ID, f"CarParamsSP angle model id = {model_id} (expected {EXPECTED_MODEL_ID}); raw sp={exp_sp}")
  else:
    ok(False, "CarParamsSP not readable — model id path unverified")
  sm = messaging.SubMaster(['pandaStates'])
  first = None; last = None; t0 = time.monotonic()
  while time.monotonic() - t0 < seconds:
    sm.update(500)
    if sm.updated['pandaStates'] and len(sm['pandaStates']):
      ps = sm['pandaStates'][0]
      snap = dict(model=str(ps.safetyModel), param=int(ps.safetyParam), ca=bool(ps.controlsAllowed),
                  cal=bool(ps.controlsAllowedLateral), rxinv=int(ps.safetyRxInvalid), txblk=int(ps.safetyTxBlocked),
                  rxchk=bool(ps.safetyRxChecksInvalid), ign=bool(ps.ignitionCan or ps.ignitionLine))
      first = first or snap; last = snap
  if not last:
    ok(False, "no pandaStates received — pandad not running?"); return
  print(f"pandaStates: model={last['model']} param={last['param']} ({decode(last['param'])}) ignition={last['ign']}")
  ok(last['model'] == 'hyundaiCanfd', "safety model is hyundaiCanfd")
  if exp_param is not None:
    ok(last['param'] == exp_param, "panda safetyParam == CarParams safetyParam")
  ok(not last['rxchk'], f"rx checks valid (safetyRxChecksInvalid={last['rxchk']}) — CCNC relaxed accel spec working")
  print(f"controlsAllowed={last['ca']} controlsAllowedLateral={last['cal']}")
  print(f"deltas over {seconds}s: rxInvalid +{last['rxinv']-first['rxinv']}  txBlocked +{last['txblk']-first['txblk']}")
  ok(last['rxinv'] - first['rxinv'] == 0, "no new rx-invalid during sample")


if __name__ == "__main__":
  ap = argparse.ArgumentParser()
  ap.add_argument("--onroad", type=int, metavar="SECONDS", help="sample pandaStates for N seconds (car connected)")
  a = ap.parse_args()
  if a.onroad:
    check_onroad(a.onroad)
  else:
    check_offroad()
