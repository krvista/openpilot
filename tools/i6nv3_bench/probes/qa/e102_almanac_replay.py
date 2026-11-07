"""E102: replay Phase 65's AlmanacSaver on a route's gpsLocationExternal fix flag and carState speed (4 Hz, as pigeond
polls): when would the stopped-receiver backup have been made (time from log start, speed), and the standstills that
were available after the fix requirement was met."""
import glob, sys
from openpilot.tools.lib.logreader import LogReader
import openpilot.system.ubloxd.pigeond as pg
D = sys.argv[1]
for r in sys.argv[2:]:
  fs = sorted(glob.glob(f"{D}/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
  s = pg.AlmanacSaver(); t0 = None; fix = None; v = None; nxt = 0.0; asks = []; end = 0.0
  for f in fs:
    for m in LogReader(f):
      w = m.which(); t = m.logMonoTime * 1e-9; t0 = t if t0 is None else t0; tt = t - t0; end = tt
      if w == "gpsLocationExternal": fix = m.gpsLocationExternal.hasFix
      elif w == "carState": v = m.carState.vEgo
      if tt >= nxt:
        nxt = tt + 0.25
        if s.update(tt, fix, v):
          asks.append((round(tt, 1), round(v * 3.6, 1))); s.done(tt, True)   # assume it works -> once
        fix = None; v = None
  print(f"{r}: {end / 60:.1f} min drive -> backup at {asks if asks else 'never'}", flush=True)
