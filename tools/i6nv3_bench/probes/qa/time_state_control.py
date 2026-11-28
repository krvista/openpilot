"""Per-frame wall time of the REAL controlsd.state_control (Python logic, stubs for messaging / VM) on a logged window,
with the shipped constants and with single features killed — what each costs. (The cubic / 55b constant sets went with
the lookahead lead in Phase 89.)
Usage (repo root): PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/tools/i6nv3_bench/probes/qa python3 .../time_state_control.py <scratch> <route> <t0> <t1>"""
import sys
import time

import numpy as np

import replay_state_control as R
import openpilot.selfdrive.controls.controlsd as cd

S, r, t0, t1 = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
R.load(S, r)
KILLS = {"new": {}, "new-noBSM": {"BSM_LANE_GUARD_M": 0.0}, "new-no41": {"LOWCONF_CAP_DPS": 0.0},
         "new-no39": {"LANE_DROPOUT_LATCH": False}, "new-noLC": {"LANE_CENTER_WEIGHT": 0.0}, "new-noEL": {"ENTRY_LEAD_S": 0.0}}
base = {k: getattr(cd, k) for ks in KILLS.values() for k in ks}
for name, kills in KILLS.items():
  variant = "new"
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
