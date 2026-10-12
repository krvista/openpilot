"""liveLocationKalman yaw rate (deg/s, calibrated frame z) and its validity, T relative to the same t0 as route_extract."""
import glob, sys
import numpy as np
from openpilot.tools.lib.logreader import LogReader
r, out = sys.argv[1], sys.argv[2]
files = sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
t0 = None; rows = []
for f in files:
  try:
    for m in LogReader(f):
      t = m.logMonoTime * 1e-9
      if t0 is None: t0 = t
      if m.which() == "liveLocationKalman":
        k = m.liveLocationKalman; av = k.angularVelocityCalibrated
        rows.append((t - t0, np.degrees(av.value[2]) if len(av.value) >= 3 else np.nan, float(av.valid), float(k.posenetOK), k.velocityCalibrated.value[0] if len(k.velocityCalibrated.value) else np.nan))
  except Exception as e:  # noqa: BLE001
    print(f.split('/')[-1], e)
np.save(out, np.array(rows, float)); print(r, len(rows))
