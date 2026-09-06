#!/usr/bin/env python3
"""Real `card` (CarInterface + CarController + CANPacker) replayed on a logged segment through process_replay.
The car's own carParams is seeded as CarParamsCache (REPLAY=1) so card fingerprints the same car (FW cache) and derives
the CCNC/LKA_STEER_MSG_ALT flags from the replayed CAN — the daemon path the device actually runs. Reports: carState/carOutput
output rates, sendcan LKAS_ALT (0x110) frames regenerated, and a parity summary of the regenerated angle/gain/active bytes
against the logged accepted echoes (src 128) at the same frame counter.
usage: PYTHONPATH=$PWD:$PWD/opendbc_repo python3 tools/i6nv3_bench/replay_card_parity.py <rlog.zst> [timeout_s]"""
import sys, os, collections, time, faulthandler
sys.path.insert(0, os.getcwd()); sys.path.insert(0, os.path.join(os.getcwd(), "opendbc_repo"))
os.environ["REPLAY"] = "1"
from openpilot.tools.lib.logreader import LogReader
from openpilot.common.params import Params
from openpilot.selfdrive.test.process_replay.process_replay import replay_process_with_name
f = sys.argv[1]; tmo = int(sys.argv[2]) if len(sys.argv) > 2 else 600
faulthandler.dump_traceback_later(tmo, exit=True)
lr = list(LogReader(f))
cp = next((m for m in lr if m.which() == "carParams"), None)
assert cp is not None, "carParams message not found"
p = Params(); p.put("CarParamsCache", cp.carParams.as_builder().to_bytes(), block=True)
p.remove("CarParams")
t0 = time.time()
out = replay_process_with_name(["card"], lr, disable_progress=True)
wall = time.time() - t0
c = collections.Counter(m.which() for m in out)
n_cs_in = sum(1 for m in lr if m.which() == "carState")
cp_out = next((m for m in out if m.which() == "carParams"), None)
print(f"CARD outputs {dict(c)} wall {wall:.0f}s")
if cp_out is not None:
  print(f"CARD fingerprint {cp_out.carParams.carFingerprint} steer {cp_out.carParams.steerControlType} safetyParam {cp_out.carParams.safetyConfigs[0].safetyParam} flags {cp_out.carParams.flags}")
  print(f"CARD log        {cp.carParams.carFingerprint} steer {cp.carParams.steerControlType} safetyParam {cp.carParams.safetyConfigs[0].safetyParam} flags {cp.carParams.flags}")
def lkas(dat):
  des = (dat[11] << 6) | (dat[10] >> 2); des = des - 16384 if des >= 8192 else des
  return des, dat[12], (dat[9] >> 4) & 3, dat[0] & 0xF if len(dat) > 0 else 0
gen = [lkas(bytes(cc.dat)) for m in out if m.which() == "sendcan" for cc in m.sendcan if cc.address == 0x110]
logd = [lkas(bytes(cc.dat)) for m in lr if m.which() == "can" for cc in m.can if cc.address == 0x110 and cc.src == 128]
print(f"CARD sendcan LKAS_ALT frames regenerated {len(gen)} vs logged accepted echoes {len(logd)}")
if gen and logd:
  import numpy as np
  n = min(len(gen), len(logd)); g = np.array(gen[:n]); l = np.array(logd[:n])
  act_g = (g[:, 2] == 2).mean(); act_l = (l[:, 2] == 2).mean()
  d = np.abs(g[:, 0] - l[:, 0]) / 10.0
  print(f"CARD active-frame share regenerated {act_g:.3f} vs logged {act_l:.3f}; |angle diff| p50/p90/max {np.median(d):.2f}/{np.percentile(d, 90):.2f}/{d.max():.2f} deg (open-loop replay: differences expected where the logged TX steered the car)")
ok = c.get("carState", 0) >= 0.8 * n_cs_in and len(gen) >= 0.8 * len(logd) and cp_out is not None and cp_out.carParams.steerControlType == cp.carParams.steerControlType
print("CARD " + ("GREEN" if ok else "FAILED"))
sys.exit(0 if ok else 1)
