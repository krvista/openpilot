"""Plan age at the controlsd tick (controlsState publish - modelV2.timestampEof of the newest modelV2 seen), the lagd
lateralDelay message, and the command step at the first tick after a new modelV2 vs other ticks (desiredCurvature)."""
import sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
age, step_new, step_old, ld = [], [], [], []
last_eof = None; new = False; prev_k = None
for f in sys.argv[1:]:
  for m in LogReader(f):
    w = m.which(); t = m.logMonoTime
    if w == "modelV2": last_eof = m.modelV2.timestampEof; new = True
    elif w == "controlsState":
      cs = m.controlsState
      if last_eof: age.append((t - last_eof) * 1e-6)
      if prev_k is not None and cs.latActive if hasattr(cs, 'latActive') else prev_k is not None:
        (step_new if new else step_old).append(abs(cs.desiredCurvature - prev_k))
      prev_k = cs.desiredCurvature; new = False
    elif w == "lateralDelay":
      x = m.lateralDelay; ld.append((x.lateralDelay, x.lateralDelayEstimate, x.validBlocks, x.status.raw if hasattr(x.status, 'raw') else x.status))
a = np.array(age); print(f"  plan age at controlsd tick: p10 {np.percentile(a,10):5.1f} p50 {np.percentile(a,50):5.1f} p90 {np.percentile(a,90):5.1f} max {a.max():5.1f} ms (n {len(a)})")
sn, so = np.array(step_new) * 1e3, np.array(step_old) * 1e3
print(f"  |dk| per tick (1e-3/m): first tick after new modelV2 p50 {np.median(sn):.3f} p90 {np.percentile(sn,90):.3f} | other ticks p50 {np.median(so):.3f} p90 {np.percentile(so,90):.3f}")
L = np.array([x[:3] for x in ld], float)
if len(L): print(f"  lateralDelay msg n {len(L)}: lateralDelay p50 {np.median(L[:,0]):.3f} (min {L[:,0].min():.3f} max {L[:,0].max():.3f}), estimate p50 {np.median(L[:,1]):.3f}, validBlocks p50 {np.median(L[:,2]):.0f}, status set {sorted(set(str(x[3]) for x in ld))}")
