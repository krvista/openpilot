"""E107: the car's real hands-on nag — cluster alert ALERTS_2 (CCNC 0x161: 1 KEEP_HANDS_ON_STEERING_WHEEL, 2 ..._RED)
onsets with speed / op lat / driver torque / HOD sensor state, and the HOD sensor (HOD_FD_01_100ms HOD_Dir_Status:
0 hands off, 1-2 touch, 3-4 grip) share of op-lat-active time per route, plus the time from hands-off to the nag."""
import glob, sys
from collections import Counter
import numpy as np
from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value
from openpilot.tools.lib.logreader import LogReader
D = sys.argv[1]
dbc = DBC("hyundai_canfd_generated")
m161 = next(m for m in dbc.msgs.values() if m.address == 353); a2 = m161.sigs["ALERTS_2"]; a3 = m161.sigs["ALERTS_3"]
mhod = next(m for m in dbc.msgs.values() if m.address == 687); hs = mhod.sigs["HOD_Dir_Status"]
for r in sys.argv[2:]:
  fs = sorted(glob.glob(f"{D}/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
  t0 = None; v = 0.0; tq = 0.0; lat = False; hod = None; hod_t = []; prev2 = 0; prev3 = 0; nags = []; hod_off_since = None; srcs = Counter(); a3c = Counter()
  for f in fs:
    for m in LogReader(f):
      try:
        w = m.which(); t = m.logMonoTime * 1e-9; t0 = t if t0 is None else t0; tt = t - t0
        if w == "carState": v = m.carState.vEgo * 3.6; tq = m.carState.steeringTorque
        elif w == "carControl": lat = m.carControl.latActive
        elif w == "can":
          for c in m.can:
            if c.address == 687:
              srcs[("hod", c.src)] += 1
              if c.src in (0, 1):
                hod = int(get_raw_value(bytes(c.dat), hs))
                if lat and v > 10: hod_t.append(hod)
                if hod == 0: hod_off_since = tt if hod_off_since is None else hod_off_since
                else: hod_off_since = None
            elif c.address == 353:
              srcs[("161", c.src)] += 1
              if c.src == 1:
                x2 = int(get_raw_value(bytes(c.dat), a2)); x3 = int(get_raw_value(bytes(c.dat), a3))
                if x2 in (1, 2) and prev2 not in (1, 2):
                  nags.append((round(tt, 1), round(v), int(lat), round(tq), hod, round(tt - hod_off_since, 1) if hod_off_since is not None else None, x2))
                if x3 != prev3 and x3: a3c[x3] += 1
                prev2, prev3 = x2, x3
      except Exception:  # noqa: BLE001
        pass
  H = Counter(hod_t); n = max(sum(H.values()), 1)
  print(f"== {r}: sources {dict(srcs)}")
  print(f"   HOD sensor while op lat (> 10 km/h): off {H[0] / n:.0%} touch {(H[1] + H[2]) / n:.0%} grip {(H[3] + H[4]) / n:.0%}")
  print(f"   hands-on nags (ALERTS_2 1/2): {len(nags)} | while op lat {sum(1 for x in nags if x[2])} | hands-off for (s) before nag p50 {np.median([x[5] for x in nags if x[5] is not None]) if any(x[5] is not None for x in nags) else None} | first 8 (t, v, lat, tq, hod, off_s, lvl): {nags[:8]}")
  print(f"   ALERTS_3 onsets: {dict(a3c)}")
