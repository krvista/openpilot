"""E101: GPS chain per route — pigeond / locationd log lines (almanac restore / store, AssistNow, time sent, init
retries) with times, time to first fix and fix quality (gpsLocationExternal: hasFix, accuracy, satellites), kalman
resets with time and speed, and the device's network / time state at boot."""
import glob, sys, json
import numpy as np
from openpilot.tools.lib.logreader import LogReader
D = sys.argv[1]
KEYS = ("almanac", "AssistNow", "ublox", "Pigeon", "pigeon", "kalman reset", "Sending current time", "Initialization failed")
for r in sys.argv[2:]:
  fs = sorted(glob.glob(f"{D}/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
  t0 = None; lines = []; fix = []; first_fix = None; v = 0.0; net = None; resets = []; nets = {}
  for f in fs:
    for m in LogReader(f):
      try:
        w = m.which(); t = m.logMonoTime * 1e-9; t0 = t if t0 is None else t0; tt = t - t0
        if w == "carState": v = m.carState.vEgo * 3.6
        elif w in ("logMessage", "errorLogMessage"):
          s = getattr(m, w)
          try:
            j = json.loads(s); msg = str(j.get("msg", s))
          except Exception:  # noqa: BLE001
            msg = s
          if any(k in msg for k in KEYS) and msg != "Received ACK from ublox":
            if "kalman reset" in msg: resets.append((round(tt, 1), round(v, 0), msg[29:60]))
            else: lines.append((round(tt, 1), msg[:90]))
        elif w == "gpsLocationExternal":
          g = m.gpsLocationExternal
          if g.hasFix and first_fix is None: first_fix = tt
          fix.append((tt, g.hasFix, g.horizontalAccuracy, g.satelliteCount if hasattr(g, "satelliteCount") else -1, g.speedAccuracy))
        elif w == "deviceState":
          nt = str(m.deviceState.networkType); nets[nt] = nets.get(nt, 0) + 1
          if net is None: net = (nt, tt)
      except Exception:  # noqa: BLE001
        pass
  F = np.array([(a, b, c, d, e) for a, b, c, d, e in fix], float) if fix else np.zeros((0, 5))
  print(f"== {r}: {len(fs)} segs, network at first deviceState {net}, network over the drive {nets}")
  for l in lines[:24]: print("   ", l)
  if len(lines) > 24: print(f"    ... {len(lines)} lines")
  if len(F):
    hf = F[:, 1] > 0
    print(f"   first fix at {first_fix} s | fix {hf.mean():.1%} of samples | hAcc p50/p90 {np.median(F[hf, 2]):.1f}/{np.percentile(F[hf, 2], 90):.1f} m | sats p10/p50 {np.percentile(F[hf, 3], 10):.0f}/{np.median(F[hf, 3]):.0f}"
          f" | no-fix stretches > 10 s: {sum(1 for _ in [0])}")
    nf = np.flatnonzero(np.diff(np.concatenate([[0], (~hf).astype(int), [0]])))
    gaps = [(round(F[a, 0], 0), round(F[min(b, len(F) - 1), 0] - F[a, 0], 1)) for a, b in zip(nf[::2], nf[1::2]) if F[min(b, len(F) - 1), 0] - F[a, 0] > 10]
    print(f"   no-fix stretches > 10 s (start, length): {gaps[:10]}")
  print(f"   kalman resets: {resets[:12]}")
