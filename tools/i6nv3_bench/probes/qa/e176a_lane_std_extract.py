import glob, sys
import numpy as np
from openpilot.tools.lib.logreader import LogReader
r, out = sys.argv[1], sys.argv[2]
files = sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
t0 = None; T, SD, DS = [], [], []
for f in files:
  try:
    for m in LogReader(f):
      t = m.logMonoTime * 1e-9
      if t0 is None: t0 = t
      if m.which() != "modelV2": continue
      d = m.modelV2
      if len(d.position.x) != 33 or len(d.laneLines) < 4 or len(d.orientationRate.z) != 33: continue
      T.append(t - t0); SD.append(list(d.laneLineStds)); DS.append(int(d.meta.laneChangeState.raw))
  except Exception as e:
    print(f.split('/')[-1], type(e).__name__, e)
np.savez_compressed(out, T=np.array(T), SD=np.array(SD, np.float32), LCS=np.array(DS))
print(r, len(T), np.percentile(np.array(SD), [10, 50, 90], axis=0))
