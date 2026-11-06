"""E94: lagd progress per drive — accepted estimates (validBlocks * 100 + current block index, from calPerc) at the
start / end of each route, vs op minutes at >= 80 km/h and the subset with no steeringPressed for 2 s."""
import glob, sys
import numpy as np
from openpilot.tools.lib.logreader import LogReader
D = sys.argv[1]
for r in sys.argv[2:]:
  fs = sorted(glob.glob(f"{D}/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
  first = last = None; fast = ok = 0.0; v = 0.0; lat = False; pr_t = -1e9; t = 0.0; dts = []
  for f in fs:
    try:
      for m in LogReader(f):
        w = m.which()
        if w == "lateralDelay":
          x = m.lateralDelay; s = (x.validBlocks, x.calPerc, round(x.lateralDelayEstimate, 3), str(x.status))
          first = first or s; last = s
        elif w == "carState":
          v = m.carState.vEgo; t = m.logMonoTime * 1e-9
          if m.carState.steeringPressed:
            pr_t = t
        elif w == "carControl":
          lat = m.carControl.latActive
        elif w == "deviceMotion" and lat and v > 50 * 0.44704:
          fast += 0.05
          if t - pr_t >= 2.0:
            ok += 0.05
    except Exception:
      pass
  print(f"{r}: segs {len(fs)} | lagd start {first} end {last} | op >= 80 km/h {fast / 60:5.1f} min, of which no press for 2 s {ok / 60:5.1f} min", flush=True)
