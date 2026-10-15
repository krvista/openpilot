"""Per-frame wall time of the REAL controlsd.state_control (Python logic, stubs for messaging / VM) on a logged window,
per constant set: cubic (pre-55), logged 55b, new (55c + 56), and new with single features killed — what each costs.
Usage (repo root): PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/tools/i6nv3_bench/probes/qa python3 .../time_state_control.py <scratch> <route> <t0> <t1>"""
import sys
import time

import numpy as np

import replay_state_control as R
import openpilot.selfdrive.controls.controlsd as cd

S, r, t0, t1 = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
R.load(S, r)
KILLS = {"new": {}, "cubic": {}, "logged": {},
         "new-no7c": {"ENTRY_ASSIST_CAP": 0.0}, "new-noBSM": {"BSM_LANE_GUARD_M": 0.0}, "new-no41": {"LOWCONF_CAP_DPS": 0.0},
         "new-no39": {"LANE_DROPOUT_LATCH": False}}
base = {k: getattr(cd, k) for ks in KILLS.values() for k in ks}
for name, kills in KILLS.items():
  variant = name if name in ("cubic", "logged") else "new"
  for k, v in kills.items():
    setattr(cd, k, v)
  ts = []
  orig = cd.Controls.state_control
  def timed(self, _o=orig):
    a = time.perf_counter(); out = _o(self); ts.append(time.perf_counter() - a); return out
  cd.Controls.state_control = timed
  try:
    R.replay(variant, t0, t1)
  finally:
    cd.Controls.state_control = orig
    for k in kills:
      setattr(cd, k, base[k])
  ts = np.array(ts[200:]) * 1e6
  print(f"{name:10} frames {len(ts):6d}  state_control us/frame p50 {np.median(ts):6.1f} p90 {np.percentile(ts, 90):6.1f} p99 {np.percentile(ts, 99):6.1f} mean {ts.mean():6.1f}")
