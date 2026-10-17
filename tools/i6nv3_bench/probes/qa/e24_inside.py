"""E24: 'too far inside' — lateral offset toward the curve INSIDE (+ = inside) per build, hands-off, both lines > 0.7,
>= 30 km/h, no blinker/LC: by |k_road| band and by phase (entry / steady / exit), p10 / p50 / p90 of inside offset and
the share of frames more than 0.35 m inside (wheel near the inner line for a 3.3 m lane). Also the inside offset
vs the controlsd additions on that frame (7c entry-assist flag, 55 lead flag) from steerFlags."""
import sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from common import routes

S = sys.argv[1]
rows = {}
for r, P, d, T, f in routes(S):
  n = len(T); v = f['v']; kr = f['k_road']; b = str(d['commit'])
  j6 = np.minimum(np.searchsorted(T, T + 0.6), n - 1); k6 = kr[j6]
  ph = np.where(np.abs(k6) > np.abs(kr) + 0.7e-3, 1, np.where(np.abs(kr) > np.abs(k6) + 0.7e-3, 3, 2))
  sgn = np.sign(np.where(np.abs(kr) > np.abs(k6), kr, k6))
  inside = f['c0'] * sgn            # c0 = lane centre y at the car (+y right); car offset = -c0; inside = -c0 * -sgn? see below
  # offset of the car from the centre is -c0 (common.py); toward the curve inside = offset * sign(k) ... curvature > 0 bends
  # toward +y, inside is +y side: inside = (-c0) * sgn
  inside = (-f['c0']) * sgn
  sf = np.interp(T, d['T'], d['sf_ctl'].astype(float)).astype(int)
  m = f['good'] & f['lat'] & ~f['hands'] & (np.maximum(np.abs(kr), np.abs(k6)) >= 1e-3)
  rows.setdefault(b, []).append(np.stack([np.abs(kr[m]), inside[m], ph[m], (sf[m] >> 8) & 1, v[m] * 3.6, f['width'][m]], 1))
  print(r, b, flush=True)
for b, rr in rows.items():
  A = np.concatenate(rr)
  print(f"== build {b}: hands-off curve frames {len(A)} ({len(A) / 1200:.1f} min), lane width p50 {np.median(A[:, 5]):.2f}")
  for nm, m in (("all", np.ones(len(A), bool)), ("|k| 1-2.5e-3", A[:, 0] < 2.5e-3), ("|k| 2.5-5e-3", (A[:, 0] >= 2.5e-3) & (A[:, 0] < 5e-3)),
                ("|k| >= 5e-3", A[:, 0] >= 5e-3), ("entry", A[:, 2] == 1), ("steady", A[:, 2] == 2), ("exit", A[:, 2] == 3),
                ("7c assist on", A[:, 3] == 1), ("7c off", A[:, 3] == 0), ("v<50", A[:, 4] < 50), ("v>=50", A[:, 4] >= 50)):
    if m.sum() >= 50:
      x = A[m, 1]
      print(f"   {nm:14} n {m.sum():6d} inside p10/p50/p90 {np.percentile(x, 10):+.2f}/{np.median(x):+.2f}/{np.percentile(x, 90):+.2f} m  >0.35 m inside {np.mean(x > 0.35):5.1%}  >0.35 outside {np.mean(x < -0.35):5.1%}")
