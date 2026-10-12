"""controlsd tick cost: controlsState publish - the carState it consumed (last carState before it), split into the first
tick after a new modelV2 vs other ticks; plus carControl/sendcan of that tick."""
import sys
import numpy as np
from openpilot.tools.lib.logreader import LogReader
last_cs = None; new_model = False; first, other = [], []; cc_first, cc_other = [], []; last_ctl = None; mark = None
for f in sys.argv[1:]:
  for m in LogReader(f):
    w = m.which(); t = m.logMonoTime
    if w == "carState": last_cs = t
    elif w == "modelV2": new_model = True
    elif w == "controlsState" and last_cs is not None:
      d = (t - last_cs) * 1e-6
      (first if new_model else other).append(d); mark = new_model; new_model = False; last_ctl = t
    elif w == "carControl" and last_cs is not None and mark is not None:
      (cc_first if mark else cc_other).append((t - last_cs) * 1e-6); mark = None
for n, a in (("controlsState, tick after new modelV2", first), ("controlsState, other ticks", other),
             ("carControl, tick after new modelV2", cc_first), ("carControl, other ticks", cc_other)):
  a = np.array(a); print(f"  {n:40} n={len(a):6d} p50 {np.percentile(a,50):6.2f} p90 {np.percentile(a,90):6.2f} p99 {np.percentile(a,99):6.2f} max {a.max():6.2f} ms")
