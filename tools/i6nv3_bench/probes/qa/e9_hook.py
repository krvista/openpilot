"""E9: "corrections come in too hard" — 55b vs the cubic build, in steering-wheel degrees. Curvature -> wheel angle via
the per-speed median apply / k_ctrl ratio of the drives. After the controlsd LP (tau(v), emulated at 20 Hz). Per speed
band: added lead |cmd - K| p95/p99, command rate deg/s p95/p99/max, and onset events (rate > 5 deg/s) per hour,
on straights (|k_road| < 0.5e-3) and overall."""
import sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from common import routes, T_IDXS, bfit

S = sys.argv[1]
rows = []; ratio_rows = []
for r, P, d, T, f in routes(S):
  n = len(T); v = f['v']
  K = P['K'].astype(float)
  OR, VX = P['OR'].astype(float), P['VX'].astype(float)
  tau = np.interp(v, [8.0, 13.0, 18.0], [0.30, 0.12, 0.08])
  kt = lambda t: np.array([np.interp(t[i], T_IDXS, OR[i]) / max(np.interp(t[i], T_IDXS, VX[i]), 1.0) for i in range(n)])
  k9 = kt(0.9 + tau); k3 = kt(np.full(n, 0.3))
  C3 = bfit(P['PX'][:, :12].astype(float), P['PY'][:, :12].astype(float), 3)
  ab = np.abs(K)
  t_ah = np.minimum(np.interp(v, [5.6, 13.9, 27.8, 38.9], [0.08, 0.10, 0.13, 0.18]) + np.interp(ab, [0.0008, 0.005], [0.0, 0.20]), 0.32) + tau
  dkc = 0.7 * t_ah / np.maximum(v, 5.0) ** 2
  blend = np.clip((ab - 0.8e-3) / 0.7e-3, 0, 1) * (ab >= 0.8e-3)
  cub = K + blend * np.clip(6.0 * C3[:, 0] * np.minimum(v * t_ah, 10.0) + 2.0 * C3[:, 1] - K, -dkc, dkc)
  dk = 0.7 * (0.9 + tau) / np.maximum(v, 5.0) ** 2
  w = np.clip((v * 3.6 - 25) / 5, 0, 1) * ~f['busy']
  lead = 0.5 * np.clip(k9 - k3, -dk, dk)
  b55 = w * (K + lead) + (1 - w) * cub
  m = f['lat'] & (np.abs(f['k_ctrl']) > 1e-3) & (np.abs(f['apply']) > 1)
  ratio_rows.append(np.stack([v[m] * 3.6, f['apply'][m] / f['k_ctrl'][m]], 1))
  def lp(c):
    out = np.empty(n); y = c[0]
    for i in range(n):
      a = 1 - (1 - 0.01 / (tau[i] + 0.01)) ** 5
      y += a * (c[i] - y); out[i] = y
    return out
  lc, lb = lp(cub), lp(b55)
  cons = np.concatenate([np.diff(T) < 0.07, [False]])
  rate_c = np.concatenate([np.diff(lc) / np.maximum(np.diff(T), 0.04), [0]])
  rate_b = np.concatenate([np.diff(lb) / np.maximum(np.diff(T), 0.04), [0]])
  act = f['lat'] & (v * 3.6 >= 30) & cons
  rows.append(np.stack([v[act] * 3.6, np.abs(b55 - K)[act], np.abs(cub - K)[act], rate_b[act], rate_c[act],
                        np.abs(f['k_road'][act]) < 0.5e-3, np.abs(lead)[act] >= dk[act] * 0.5 - 1e-12], 1))
  print(r, flush=True)
R = np.concatenate(ratio_rows)
A = np.concatenate(rows)
BANDS = [(30, 50), (50, 70), (70, 90), (90, 200)]
deg = {}
for b in BANDS:
  s = (R[:, 0] >= b[0]) & (R[:, 0] < b[1])
  deg[b] = np.median(np.abs(R[s, 1])) if s.sum() > 100 else np.nan
print("wheel deg per 1e-3/m curvature: " + " ".join(f"{b[0]}-{b[1]} {deg[b] * 1e-3:.2f}" for b in BANDS))
for zone, zn in ((A[:, 5] > 0, "straight"), (np.ones(len(A), bool), "all")):
  print(f"== {zn}")
  for b in BANDS:
    s = zone & (A[:, 0] >= b[0]) & (A[:, 0] < b[1])
    if s.sum() < 200:
      continue
    g = deg[b]; hrs = s.sum() / 20 / 3600
    lb, lc = A[s, 1] * g, A[s, 2] * g
    rb, rc = np.abs(A[s, 3]) * g, np.abs(A[s, 4]) * g
    ev = lambda x: np.sum((x[1:] > 5) & (x[:-1] <= 5)) / hrs
    print(f"  {b[0]:3}-{b[1]:<3} {hrs * 60:5.1f} min | added lead deg p95/p99: 55b {np.percentile(lb,95):.2f}/{np.percentile(lb,99):.2f} "
          f"cubic {np.percentile(lc,95):.2f}/{np.percentile(lc,99):.2f} | cmd rate deg/s p95/p99/max: 55b {np.percentile(rb,95):.1f}/"
          f"{np.percentile(rb,99):.1f}/{rb.max():.0f} cubic {np.percentile(rc,95):.1f}/{np.percentile(rc,99):.1f}/{rc.max():.0f} | "
          f">5 deg/s onsets/h 55b {ev(rb):.0f} cubic {ev(rc):.0f} | lead at budget {np.mean(A[s,6] > 0):.1%}")
