"""E108: device roll at standstill (localizer deviceMotion.orientation = device euler in NED, the same roll paramsd
observes as road roll; v < 0.1 m/s for >= 3 s): per stop the mean roll / pitch; distribution over the stops of a route. A device
mounted level shows a spread around the ground roll of the stops (~0 .. +1 deg on crowned roads); a constant offset is
the mount roll (calibrationd estimates pitch / yaw only and takes roll as 0, so it is never corrected)."""
import glob, sys
import numpy as np
from openpilot.tools.lib.logreader import LogReader
D = sys.argv[1]
for r in sys.argv[2:]:
  fs = sorted(glob.glob(f"{D}/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
  v = 1.0; still_t = None; buf = []; stops = []; t0 = None
  for f in fs:
    for m in LogReader(f):
      try:
        w = m.which(); t = m.logMonoTime * 1e-9; t0 = t if t0 is None else t0
        if w == "carState":
          v = m.carState.vEgo
          if v < 0.1:
            still_t = t if still_t is None else still_t
          else:
            if still_t is not None and t - still_t >= 3.0 and len(buf) > 20:
              a = np.mean(buf, 0); stops.append((round(still_t - t0), np.degrees(a[0]), np.degrees(a[1]), round(t - still_t)))
            still_t = None; buf = []
        elif w == "deviceMotion" and still_t is not None and t - still_t >= 1.0:
          o = m.deviceMotion.orientationNED
          buf.append([o.x, o.y])
      except Exception:  # noqa: BLE001
        pass
  if not stops:
    print(r, "no stops"); continue
  R = np.array([s[1] for s in stops]); P = np.array([s[2] for s in stops])
  print(f"{r}: {len(stops)} stops >= 3 s | device roll (localizer) p10/p25/p50/p75/p90 {np.percentile(R, 10):+.2f}/{np.percentile(R, 25):+.2f}/{np.median(R):+.2f}/{np.percentile(R, 75):+.2f}/{np.percentile(R, 90):+.2f} deg | pitch p50 {np.median(P):+.2f} | first stops (t, roll, pitch, dur): {[(s[0], round(s[1], 2), round(s[2], 2), s[3]) for s in stops[:5]]}", flush=True)
