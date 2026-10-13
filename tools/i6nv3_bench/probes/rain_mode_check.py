"""Phase 37a rain mode reconstruction from rlog CAN: CCNC_WIPER (0x35c) byte 18 bit 0 per bus, the carcontroller debounce
(ON 500 frames = 5 s continuous, OFF 6000 = 60 s), and the resulting rain_active share. Also stalk pulse bit (150) and aux bit (147)."""
import glob, sys
import numpy as np
from openpilot.tools.lib.logreader import LogReader
r = sys.argv[1]
files = sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
t0 = None; rows = {}; v = []; 
for f in files:
  for m in LogReader(f):
    t = m.logMonoTime * 1e-9
    if t0 is None: t0 = t
    w = m.which()
    if w == "carState": v.append((t - t0, m.carState.vEgo * 3.6))
    elif w == "can":
      for c in m.can:
        if c.address == 0x35c and c.src < 128 and len(c.dat) >= 19:
          d = bytes(c.dat); rows.setdefault(c.src, []).append((t - t0, d[18] & 1, (d[18] >> 3) & 1, (d[18] >> 6) & 1))
V = np.array(v)
for src, rr in sorted(rows.items()):
  A = np.array(rr, float); T = A[:, 0]; on = A[:, 1]
  print(f"bus {src}: msgs {len(A)} ({len(A)/(T[-1]-T[0]+1e-9):.1f} Hz), FRONT_WIPER_ON duty {on.mean():.1%}, aux3 duty {A[:,2].mean():.1%}, stalk_pulse duty {A[:,3].mean():.1%}")
  # episodes of ON
  e = np.diff(np.concatenate([[0], on, [0]])); st = np.where(e == 1)[0]; en = np.where(e == -1)[0]
  dur = T[np.minimum(en, len(T) - 1)] - T[st]
  if len(st): print(f"   ON episodes {len(st)}: duration p10/p50/p90/max {np.percentile(dur,10):.1f}/{np.median(dur):.1f}/{np.percentile(dur,90):.1f}/{dur.max():.1f} s; first at {T[st[0]]:.0f} s, last end {T[en[-1]-1] if en[-1] > 0 else 0:.0f} s of {T[-1]:.0f}")
  # reconstruct debounce at 100 Hz
  g = np.arange(0, T[-1], 0.01); onf = np.interp(g, T, on) > 0.5
  act = np.zeros(len(g), bool); cnt_on = cnt_off = 0; a = False
  for i in range(len(g)):
    if onf[i]: cnt_on += 1; cnt_off = 0
    else: cnt_off += 1; cnt_on = 0
    if cnt_on >= 500: a = True
    elif cnt_off >= 6000: a = False
    act[i] = a
  vg = np.interp(g, V[:, 0], V[:, 1])
  print(f"   rain_active share {act.mean():.1%} of route, {act[vg > 5].mean():.1%} of moving time; active from {g[act][0] if act.any() else -1:.0f} s to {g[act][-1] if act.any() else -1:.0f} s")
  # minute-by-minute duty
  mins = int(T[-1] // 60) + 1; duty = [on[(T >= k*60) & (T < (k+1)*60)].mean() if ((T >= k*60) & (T < (k+1)*60)).any() else np.nan for k in range(mins)]
  print("   per-minute ON duty:", " ".join(f"{d*100:3.0f}" if not np.isnan(d) else "  -" for d in duty))
