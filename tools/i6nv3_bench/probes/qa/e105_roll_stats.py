"""E105: estimated road roll (vehicleParameters.roll, paramsd) per route: distribution overall, on straights (|yaw rate|
small) vs curves, by speed band, and the drive-to-drive consistency — a device mounted with a roll about the car's
longitudinal axis shows up as a constant offset (calibrationd estimates pitch / yaw only, roll is taken as 0)."""
import glob, sys
import numpy as np
from openpilot.tools.lib.logreader import LogReader
D = sys.argv[1]
for r in sys.argv[2:]:
  fs = sorted(glob.glob(f"{D}/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
  v = 0.0; yr = 0.0; rows = []
  for f in fs:
    for m in LogReader(f):
      try:
        w = m.which()
        if w == "carState": v = m.carState.vEgo * 3.6
        elif w == "deviceMotion": yr = m.deviceMotion.angularVelocityDevice.z if hasattr(m.deviceMotion, "angularVelocityDevice") else yr
        elif w == "vehicleParameters" and v > 15:
          rows.append((v, np.degrees(m.vehicleParameters.roll), abs(yr), m.vehicleParameters.angleOffsetDeg))
      except Exception:  # noqa: BLE001
        pass
  A = np.array(rows)
  if not len(A):
    print(r, "no data"); continue
  q = lambda x: "/".join(f"{np.percentile(x, p):+.2f}" for p in (5, 25, 50, 75, 95))
  s = A[:, 2] < 0.01; c = A[:, 2] > 0.03
  print(f"{r}: roll deg p5/25/50/75/95 all {q(A[:, 1])} | straights {q(A[s, 1])} | curves {q(A[c, 1])} | share roll > +1 deg {np.mean(A[:, 1] > 1):.0%}, < -1 deg {np.mean(A[:, 1] < -1):.0%}")
  for lo, hi in ((15, 40), (40, 70), (70, 200)):
    b = (A[:, 0] >= lo) & (A[:, 0] < hi)
    if b.sum() > 100:
      print(f"    {lo}-{hi} km/h: roll p50 {np.median(A[b, 1]):+.2f} (p25/p75 {np.percentile(A[b, 1], 25):+.2f}/{np.percentile(A[b, 1], 75):+.2f}) | angleOffset p50 {np.median(A[b, 3]):+.2f}")
