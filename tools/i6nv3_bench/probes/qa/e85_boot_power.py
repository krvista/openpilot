"""E85: first 20 s of segment 0 per route — 12 V supply (peripheralState.voltage), panda uptime, device thermal/
cpu state, and road camera frame start, to compare the fault boots (7d, 7e) with normal ones."""
import sys, glob
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from openpilot.tools.lib.logreader import LogReader
D = sys.argv[1]
for r in sys.argv[2:]:
  f = glob.glob(f"{D}/*_{r}--*--0--rlog.zst")[0]
  t0 = None; volt = []; cam = None; dev = []
  for m in LogReader(f):
    t = m.logMonoTime * 1e-9
    if t0 is None: t0 = t
    w = m.which()
    if w == "peripheralState": volt.append((t - t0, m.peripheralState.voltage / 1000))
    elif w == "roadCameraState" and cam is None: cam = t - t0
    elif w == "deviceState": dev.append((t - t0, m.deviceState.maxTempC, m.deviceState.memoryUsagePercent))
  v = np.array(volt)
  first = v[v[:, 0] < 5, 1] if len(v) else []
  print(f"{r}: log mono0 {t0:7.1f} s | 12V first 5 s min {np.min(first) if len(first) else np.nan:5.2f} med {np.median(first) if len(first) else np.nan:5.2f} | 12V whole seg0 min {v[:,1].min():5.2f} | first road frame +{cam if cam is not None else np.nan:.2f} s | temp {dev[0][1] if dev else np.nan:.0f} C")
