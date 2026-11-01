"""VM/delay/model-name extractor: liveParameters, liveLocationKalman yaw rate, liveDelay, liveTorque-free; initData params
mentioning Model. T relative to the same t0 as route_extract (first message of the first rlog)."""
import glob
import sys
import numpy as np
from openpilot.tools.lib.logreader import LogReader
r, out = sys.argv[1], sys.argv[2]
files = sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
t0 = None; lp, lk, ld = [], [], []; params = {}; services = set()
for f in files:
  try:
    for m in LogReader(f):
      t = m.logMonoTime * 1e-9; w = m.which(); services.add(w)
      if t0 is None:
        t0 = t
      if w in ("liveParameters", "vehicleParameters"):
        p = getattr(m, w)
        lp.append((t - t0, p.steerRatio, p.stiffnessFactor, p.angleOffsetDeg, p.angleOffsetAverageDeg, p.roll, float(p.valid),
                   float(getattr(p, "sensorValid", True)), getattr(p, "steerRatioStd", np.nan), getattr(p, "angleOffsetFastStd", np.nan)))
      elif w == "liveLocationKalman":
        k = m.liveLocationKalman; av = k.angularVelocityCalibrated
        lk.append((t - t0, np.degrees(av.value[2]) if len(av.value) >= 3 else np.nan, float(av.valid), float(k.posenetOK)))
      elif w in ("liveDelay", "lateralDelay"):
        d = getattr(m, w)
        ld.append((t - t0, d.lateralDelay, int(d.status.raw) if hasattr(d.status, "raw") else -1, getattr(d, "validBlocks", -1),
                   getattr(d, "lateralDelayEstimate", np.nan)))
      elif w == "initData" and not params:
        for e in m.initData.params.entries:
          if "odel" in e.key or "Steer" in e.key or "Lag" in e.key or "Delay" in e.key:
            try:
              params[e.key] = bytes(e.value).decode(errors="replace")[:120]
            except Exception:  # noqa: BLE001
              params[e.key] = "?"
  except Exception as e:  # noqa: BLE001
    print(f.split('/')[-1], type(e).__name__, e)
np.savez_compressed(out, lp=np.array(lp, float), lk=np.array(lk, float), ld=np.array(ld, float), params=np.array(str(params)))
print(r, "lp", len(lp), "lk", len(lk), "ld", len(ld), "params", params, "| has livePose", "livePose" in services)
