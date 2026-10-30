"""E34: lateral bias on straights. Offset of the car from the lane centre at x = 0 (plan lane lines, + = right of centre),
straight road (|k_road| < 0.3e-3 now and 1 s ahead), both inner lines > 0.8, lane width 2.8-4.0 m, >= 40 km/h, no
blinker/LC +-3 s. Per build and mode: op hands-off / op hands-on / human (lat inactive). Per route means (stability),
by speed band, by lane width band, and the inner-line distances (left / right) to see whether one line is read off."""
import sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from common import routes

S = sys.argv[1]
rows = []
for r, P, d, T, f in routes(S):
  n = len(T); b = str(d['commit'])
  kr = f['k_road']; k1 = kr[np.minimum(np.searchsorted(T, T + 1.0), n - 1)]
  L1 = np.array([np.interp(0.0, P['LX'][i], P['L1'][i]) for i in range(n)])
  L2 = np.array([np.interp(0.0, P['LX'][i], P['L2'][i]) for i in range(n)])
  pr = P['PR']
  m = ((np.abs(kr) < 0.3e-3) & (np.abs(k1) < 0.3e-3) & (pr[:, 1] > 0.8) & (pr[:, 2] > 0.8) & (f['width'] > 2.8) & (f['width'] < 4.0)
       & (f['v'] * 3.6 >= 40) & ~f['busy'])
  mode = np.where(~f['lat'], 0, np.where(f['hands'], 2, 1))
  rows.append(np.stack([np.full(m.sum(), int(r, 16)), np.full(m.sum(), int(b[:7], 16)), mode[m], -f['c0'][m], f['v'][m] * 3.6,
                        f['width'][m], -L1[m], L2[m]], 1))
  print(r, b, flush=True)
A = np.concatenate(rows)
names = {0: "human", 1: "op hands-off", 2: "op hands-on"}
for bb in np.unique(A[:, 1]):
  B = A[A[:, 1] == bb]
  print(f"== build {int(bb):07x}: straight good-lane frames {len(B)} ({len(B) / 1200:.1f} min)")
  for md in (1, 2, 0):
    x = B[B[:, 2] == md]
    if len(x) < 200:
      continue
    per = [np.mean(x[x[:, 0] == rr, 3]) for rr in np.unique(x[:, 0]) if np.sum(x[:, 0] == rr) >= 200]
    print(f"   {names[md]:13} {len(x) / 1200:5.1f} min  mean {np.mean(x[:, 3]):+.3f} m  median {np.median(x[:, 3]):+.3f}  std {np.std(x[:, 3]):.3f} | "
          f"per-route means ({len(per)}) {np.min(per) if per else 0:+.3f} .. {np.max(per) if per else 0:+.3f}, sd {np.std(per) if per else 0:.3f} | "
          f"left line {np.median(x[:, 6]):.2f} right {np.median(x[:, 7]):.2f} m")
  x = B[B[:, 2] == 1]
  if len(x) >= 200:
    print("   op hands-off by speed: " + "  ".join(f"{lo}-{hi}: {np.mean(x[(x[:, 4] >= lo) & (x[:, 4] < hi), 3]):+.3f} (n {np.sum((x[:, 4] >= lo) & (x[:, 4] < hi))})"
                                               for lo, hi in ((40, 60), (60, 90), (90, 200)) if np.sum((x[:, 4] >= lo) & (x[:, 4] < hi)) >= 100))
    print("   op hands-off by width: " + "  ".join(f"{lo}-{hi}: {np.mean(x[(x[:, 5] >= lo) & (x[:, 5] < hi), 3]):+.3f} (n {np.sum((x[:, 5] >= lo) & (x[:, 5] < hi))})"
                                               for lo, hi in ((2.8, 3.1), (3.1, 3.4), (3.4, 4.0)) if np.sum((x[:, 5] >= lo) & (x[:, 5] < hi)) >= 100))
