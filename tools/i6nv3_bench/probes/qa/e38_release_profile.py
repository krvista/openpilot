"""E38: what precedes a yank? For every E37-style yank on the latest build: driver torque max / time of the last |tq| >= 60,
>= 150, >= 300 before onset, EPS steeringPressed in the 1 s before, carcontroller flags at onset (42c anchor hold,
35b anchored recovery, 29 post-grip, 39a city release, 12 fast floor, 51 HOD touch), and the wheel-command gap history:
|apply - ang| 1.0 / 0.5 / 0.1 s before. Classifies: 'release' (a grip/touch >= 150 ended within 0.5 s), 'light hand'
(only 60-150 in the 1 s before), 'no hand'."""
import glob
import sys
from collections import Counter
import numpy as np

S = sys.argv[1]; B = sys.argv[2] if len(sys.argv) > 2 else "98e912d3d"
CC = {8: "drvPressed", 9: "epsPressed", 10: "gripAnchor", 11: "anchorHold42c", 12: "fastFloor", 13: "tqArmFloor", 18: "cityRel39a",
      19: "anchRec35b", 21: "postGrip29", 25: "outrun", 27: "txSat", 28: "hodTouch51"}
rows = []; kinds = Counter(); fl = Counter(); hours = 0.0
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = np.load(f, allow_pickle=True)
  if str(d['commit']) != B:
    continue
  r = f[-12:-4]; T = d['T']; n = len(T); v = d['v']
  ang = d['ang']; ap = d['apply']; g = d['gain']; tq = np.abs(d['tq_s']); pr = d['pressed']; sc = d['sf_cc'].astype(np.int64)
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  c = lambda m: np.concatenate([[0], np.cumsum(m)]); idx = np.arange(n)
  busy = (c(bl)[np.minimum(idx + 300, n)] - c(bl)[np.maximum(idx - 300, 0)]) > 0
  act = d['lat'] & (v >= 30) & ~busy
  hours += act.sum() / 360000
  rate = np.zeros(n); rate[10:] = (ang[10:] - ang[:-10]) / np.maximum(T[10:] - T[:-10], 0.05)
  i = 100
  while i < n - 60:
    if act[i] and abs(rate[i]) >= 20 and abs(ang[min(i + 40, n - 1)] - ang[max(i - 10, 0)]) >= 3:
      a0 = max(i - 10, 0); a1 = min(i + 40, n - 1)
      if (tq[a0:a1] >= 150).any() or pr[a0:a1].any() or not act[a0:a1].all():
        i += 1; continue
      last = lambda thr: (T[a0] - T[np.flatnonzero(tq[:a0] >= thr)[-1]]) if (tq[:a0] >= thr).any() else 99.0
      l60, l150, l300 = last(60), last(150), last(300)
      kind = "release (>=150 ended < 0.5 s)" if l150 < 0.5 else ("light hand (60-150 < 1 s)" if l60 < 1.0 else "no hand")
      kinds[kind] += 1
      f_on = int(sc[a0])
      for bit, nm in CC.items():
        if f_on >> bit & 1:
          fl[(kind, nm)] += 1
      gaps = [abs(ap[max(a0 - k, 0)] - ang[max(a0 - k, 0)]) for k in (100, 50, 10)]
      sgn = np.sign(ang[a1] - ang[a0])
      toward_cmd = np.sign(ap[a0] - ang[a0]) == sgn
      rows.append((kind, abs(ang[a1] - ang[a0]), np.max(np.abs(rate[a0:a1])), gaps, l150, toward_cmd, r, T[i], v[i], tq[max(a0 - 100, 0):a0].max(), g[a0]))
      i = a1 + 50
    else:
      i += 1
print(f"build {B}: op active >= 30 km/h {hours * 60:.0f} min, yanks {len(rows)} ({len(rows) / max(hours, 1e-9):.1f}/h)")
for k, nn in kinds.most_common():
  rs = [x for x in rows if x[0] == k]
  print(f"  {k:30} {nn:3d} ({nn / hours:4.1f}/h) | size p50 {np.median([x[1] for x in rs]):4.1f} deg, rate p50 {np.median([x[2] for x in rs]):3.0f} deg/s | "
        f"gap 1.0/0.5/0.1 s before p50 {np.median([x[3][0] for x in rs]):4.1f}/{np.median([x[3][1] for x in rs]):4.1f}/{np.median([x[3][2] for x in rs]):4.1f} deg | "
        f"wheel moved toward the command {sum(1 for x in rs if x[5])}/{nn} | torque max 1 s before p50 {np.median([x[9] for x in rs]):3.0f}")
  print("      flags at onset: " + ", ".join(f"{nm} {cnt}" for (kk, nm), cnt in fl.most_common() if kk == k))
