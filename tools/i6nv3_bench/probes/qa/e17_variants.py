"""E17: lead variants on ALL plan routes (cubic-build 2e-38 and 55b 48-4f): per curve phase, the mean lead signed toward
the curve INSIDE (+ = more turn), its spread, and on straights its RMS / 20 Hz change; open-loop RMSE vs road at t+0.45;
centring slopes. Phases from road curvature now vs +0.6 s (|k| >= 1e-3 for curves), op active, both lines > 0.7,
>= 30 km/h, no blinker/LC. The real closed-loop offset change cubic -> 55b (§56) calibrates lead -> offset per phase."""
import sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from common import routes, T_IDXS, bfit

S = sys.argv[1]
V = ("cubic", "55b", "55c", "dz0.3", "dz0.5", "dz0.3g", "grow+exit")
acc = {v: {} for v in V}
cent = {v: [] for v in V}
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
  dk = 0.7 * (0.9 + tau) / np.maximum(v, 5.0) ** 2
  dl = np.clip(k9 - k3, -dk, dk)
  grow = (np.sign(dl) == np.sign(k9)) & (np.abs(k9) > np.abs(k3))
  b9 = np.clip((np.maximum(np.abs(k9), np.abs(k3)) - 0.8e-3) / 0.7e-3, 0, 1)
  dz = lambda z: np.sign(dl) * np.maximum(np.abs(dl) - z, 0.0)
  lead = {"cubic": blend * np.clip(6.0 * C3[:, 0] * np.minimum(v * t_ah, 10.0) + 2.0 * C3[:, 1] - K, -dkc, dkc),
          "55b": 0.5 * dl, "55c": 0.5 * dl * grow * np.clip((np.abs(k9) - 0.8e-3) / 0.7e-3, 0, 1),
          "dz0.3": 0.5 * dz(0.3e-3), "dz0.5": 0.5 * dz(0.5e-3), "dz0.3g": 0.5 * dz(0.3e-3) * b9,
          "grow+exit": 0.5 * dl * np.where(grow, np.clip((np.abs(k9) - 0.8e-3) / 0.7e-3, 0, 1), np.clip((np.abs(dl) - 0.5e-3) / 0.5e-3, 0, 1) * b9)}
  kr = f['k_road']; j6 = np.minimum(np.searchsorted(T, T + 0.6), n - 1); ka = kr[j6]
  base = f['good'] & f['lat']
  ph = np.where(np.abs(np.maximum(np.abs(kr), np.abs(ka))) < 0.5e-3, 0,
       np.where((np.abs(kr) >= 1e-3) | (np.abs(ka) >= 1e-3),
                np.where(np.abs(ka) > np.abs(kr) + 0.7e-3, 1, np.where(np.abs(kr) > np.abs(ka) + 0.7e-3, 3, 2)), -1))
  sgn = np.sign(np.where(np.abs(kr) > np.abs(ka), kr, ka))
  truth = kr.copy(); truth[~f['good']] = np.nan
  jj = np.minimum(np.searchsorted(T, T + 0.45), n - 1); tr = truth[jj]
  okt = base & (np.abs(T[jj] - (T + 0.45)) < 0.03) & ~np.isnan(tr)
  cons = np.concatenate([np.diff(T) < 0.07, [False]])
  for vn in V:
    L = lead[vn]
    for p in (0, 1, 2, 3):
      m = base & (ph == p)
      a = acc[vn].setdefault(p, dict(inside=[], err=[], chg=[]))
      a['inside'].append((L * sgn)[m] if p else L[m])
      a['err'].append((K + L - tr)[okt & (ph == p)])
      mc = m & cons
      a['chg'].append(np.diff(np.concatenate([L, [L[-1]]]))[mc])
    ms = base & (ph >= 0)
    cent[vn].append(np.stack([f['c0'][ms] * -1, (K + L - kr)[ms], (K - kr)[ms]], 1))
  print(r, flush=True)
names = {0: "straight", 1: "entry", 2: "steady", 3: "exit"}
rm = lambda x: np.sqrt(np.mean(x ** 2)) * 1e3
print("lead toward inside (1e-3/m) mean | p10/p90 | RMSE vs road | 20 Hz lead change std")
for p in (0, 1, 2, 3):
  print(f"== {names[p]} (n {len(np.concatenate(acc['cubic'][p]['inside']))})")
  for vn in V:
    a = acc[vn][p]; x = np.concatenate(a['inside'])
    print(f"   {vn:9} {'RMS ' + format(rm(x), '.3f') if p == 0 else format(np.mean(x) * 1e3, '+.3f')} | {np.percentile(x, 10) * 1e3:+.2f}/{np.percentile(x, 90) * 1e3:+.2f}"
          f" | {rm(np.concatenate(a['err'])):.3f} | {np.std(np.concatenate(a['chg'])) * 1e3:.4f}")
print("offset feedback of the command (slope of (cmd - k_road) on offset, rel. to action):")
for vn in V:
  A = np.concatenate(cent[vn])
  s1 = np.polyfit(A[:, 0], A[:, 1], 1)[0]; s0 = np.polyfit(A[:, 0], A[:, 2], 1)[0]
  print(f"   {vn:9} {s1 / s0:.2f}")
