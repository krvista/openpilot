"""E8: pre-drive checks of Phase 55b (cmd = K + 0.5 clip(k_or(0.9+tau) - k_or(0.3), +-dk)) vs the Phase 41-2 cubic build.
(a) reversal: frames where the command's sign differs from the action's (|K| >= 0.3e-3): how often the road 0.45 s
    later agrees with the command, and the wrong-side excess.
(b) low lane confidence: size of the added lead |cmd - K| by inner-lane min prob (now ungated on straights, the plan
    is least reliable there), lat active >= 30 km/h, all frames incl. blinker.
(c) 7c entry assist: fire rate and mean assist on top of each build (the rising detector reads the shaped command)."""
import sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from common import routes, T_IDXS

S = sys.argv[1]
acc = {k: [] for k in ("rev_c", "rev_b", "low", "ea")}
R = {"cubic": [0, 0, 0, []], "55b": [0, 0, 0, []]}
LOW = []; EA = {"cubic": [0, 0, 0.0], "55b": [0, 0, 0.0]}
for r, P, d, T, f in routes(S):
  n = len(T); v = f['v']
  K = P['K'].astype(float)
  OR, VX = P['OR'].astype(float), P['VX'].astype(float)
  tau = np.interp(v, [8.0, 13.0, 18.0], [0.30, 0.12, 0.08])
  kt = lambda t: np.array([np.interp(t[i], T_IDXS, OR[i]) / max(np.interp(t[i], T_IDXS, VX[i]), 1.0) for i in range(n)])
  k9 = kt(0.9 + tau); k3 = kt(np.full(n, 0.3))
  from common import bfit
  C3 = bfit(P['PX'][:, :12].astype(float), P['PY'][:, :12].astype(float), 3)
  ab = np.abs(K)
  t_ah = np.minimum(np.interp(v, [5.6, 13.9, 27.8, 38.9], [0.08, 0.10, 0.13, 0.18]) + np.interp(ab, [0.0008, 0.005], [0.0, 0.20]), 0.32) + tau
  dkc = 0.7 * t_ah / np.maximum(v, 5.0) ** 2
  blend = np.clip((ab - 0.8e-3) / 0.7e-3, 0, 1) * (ab >= 0.8e-3)
  cub = K + blend * np.clip(6.0 * C3[:, 0] * np.minimum(v * t_ah, 10.0) + 2.0 * C3[:, 1] - K, -dkc, dkc)
  dk = 0.7 * (0.9 + tau) / np.maximum(v, 5.0) ** 2
  w = np.clip((v * 3.6 - 25) / 5, 0, 1) * ~f['busy']
  b55 = w * (K + 0.5 * np.clip(k9 - k3, -dk, dk)) + (1 - w) * cub
  cmds = {"cubic": cub, "55b": b55}
  truth = f['k_road'].copy(); truth[~f['good']] = np.nan
  j = np.minimum(np.searchsorted(T, T + 0.45), n - 1)
  ok = f['good'] & f['lat'] & (np.abs(T[j] - (T + 0.45)) < 0.03) & ~np.isnan(truth[j]) & (ab >= 0.3e-3)
  tr = truth[j]
  for nm, c in cmds.items():
    flip = ok & (np.sign(c) != np.sign(K)) & (np.abs(c) > 0.2e-3)
    agree = flip & (np.sign(tr) == np.sign(c))
    over = np.where(np.sign(tr) == np.sign(c), np.maximum(np.abs(c) - np.abs(tr), 0), np.abs(c))
    R[nm][0] += int(ok.sum()); R[nm][1] += int(flip.sum()); R[nm][2] += int(agree.sum()); R[nm][3].append(over[flip])
  lm = np.minimum(P['PR'][:, 1], P['PR'][:, 2])
  act = f['lat'] & (v * 3.6 >= 30)
  LOW.append(np.stack([lm[act], np.abs(b55 - K)[act], np.abs(cub - K)[act], ab[act]], 1))
  # 7c on top (20 Hz approx), klane from 0/25/50 m
  LX = P['LX'].astype(float); LC = 0.5 * (P['L1'].astype(float) + P['L2'].astype(float))
  lc = lambda xq: np.array([np.interp(xq, LX[i], LC[i]) for i in range(n)])
  klane = (lc(0.0) - 2.0 * lc(25.0) + lc(50.0)) / 625.0
  for nm, c in cmds.items():
    kl = 0.0; slow = 0.0; fires = 0; tot = 0; asum = 0.0
    for i in range(n):
      if np.isfinite(klane[i]):
        kl += 0.05 / 0.35 * (klane[i] - kl)
      slow += 0.1 * (abs(c[i]) - slow)
      if not act[i]:
        continue
      tot += 1
      if (v[i] > 8.0 and lm[i] > 0.6 and not f['busy'][i] and abs(kl) > 0.003 and abs(c[i]) > slow * 1.02
          and np.sign(kl) == np.sign(c[i]) and abs(kl) > abs(c[i])):
        a = min(abs(kl) - abs(c[i]), 1e-3, 0.5 * abs(c[i]))
        if a > 0:
          fires += 1; asum += a
    EA[nm][0] += fires; EA[nm][1] += tot; EA[nm][2] += asum
  print(r, flush=True)

print("(a) reversal (|K| >= 0.3e-3, both lines > 0.7)")
for nm, (n_ok, n_f, n_a, ov) in R.items():
  ov = np.concatenate(ov)
  print(f"  {nm:6} frames {n_ok:6d} early flip {n_f:4d} ({n_f / n_ok:.2%}) road agrees {n_a / max(n_f, 1):.0%} "
        f"wrong-side excess p95 {np.percentile(ov, 95) * 1e3 if len(ov) else 0:.3f}e-3")
A = np.concatenate(LOW)
print("(b) added lead |cmd - K| (1e-3/m) by inner-lane min prob, lat active >= 30 km/h")
for lo, hi in ((0, 0.3), (0.3, 0.5), (0.5, 0.7), (0.7, 1.01)):
  s = (A[:, 0] >= lo) & (A[:, 0] < hi)
  print(f"  lane {lo:.1f}-{hi:.1f} share {s.mean():5.1%}  55b p50/p95/p99 {np.median(A[s,1])*1e3:.3f}/{np.percentile(A[s,1],95)*1e3:.3f}/"
        f"{np.percentile(A[s,1],99)*1e3:.3f}   cubic p95/p99 {np.percentile(A[s,2],95)*1e3:.3f}/{np.percentile(A[s,2],99)*1e3:.3f}")
print("(c) 7c entry assist on top")
for nm, (fi, tot, asum) in EA.items():
  print(f"  {nm:6} fires {fi / tot:.2%} of active frames, mean assist {asum / max(fi, 1) * 1e3:.3f}e-3")
