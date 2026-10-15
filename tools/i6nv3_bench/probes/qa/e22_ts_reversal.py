"""E22: 55c lead with sample time 0.9 vs 1.1 s — S-curve / reversal safety (as qa/e8) and open-loop RMSE vs road at
t + 0.45 s, all plan routes."""
import sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from common import routes, T_IDXS

S = sys.argv[1]
TS = (0.7, 0.9, 1.1, 1.3)
R = {t: [0, 0, 0, []] for t in TS}; E = {t: {} for t in TS}
for r, P, d, T, f in routes(S):
  n = len(T); v = f['v']; K = P['K'].astype(float)
  OR, VX = P['OR'].astype(float), P['VX'].astype(float)
  tau = np.interp(v, [8.0, 13.0, 18.0], [0.30, 0.12, 0.08])
  kt = lambda t: np.array([np.interp(t[i], T_IDXS, OR[i]) / max(np.interp(t[i], T_IDXS, VX[i]), 1.0) for i in range(n)])
  k3 = kt(np.full(n, 0.3))
  truth = f['k_road'].copy(); truth[~f['good']] = np.nan
  j = np.minimum(np.searchsorted(T, T + 0.45), n - 1); tr = truth[j]
  ok = f['good'] & f['lat'] & (np.abs(T[j] - (T + 0.45)) < 0.03) & ~np.isnan(tr)
  tr0 = truth
  reg = np.where((np.abs(tr) < 0.5e-3) & (np.abs(tr0) < 0.5e-3), "straight", np.where(np.abs(tr) > np.abs(tr0) + 0.7e-3, "entry",
                 np.where(np.abs(tr0) > np.abs(tr) + 0.7e-3, "exit", np.where(np.abs(tr) >= 1.5e-3, "curve", "mild"))))
  for ts in TS:
    k9 = kt(ts + tau)
    dk = 0.7 * (ts + tau) / np.maximum(v, 5.0) ** 2
    dl = np.clip(k9 - k3, -dk, dk)
    w = np.interp(np.abs(k9) - np.abs(k3), [0.0, 0.5e-3], [0.2, 0.5])
    g = np.clip((np.maximum(np.abs(k9), np.abs(k3)) - 0.8e-3) / 0.7e-3, 0, 1)
    c = K + w * g * np.sign(dl) * np.maximum(np.abs(dl) - 0.15e-3, 0.0)
    okr = ok & (np.abs(K) >= 0.3e-3)
    flip = okr & (np.sign(c) != np.sign(K)) & (np.abs(c) > 0.2e-3)
    agree = flip & (np.sign(tr) == np.sign(c))
    over = np.where(np.sign(tr) == np.sign(c), np.maximum(np.abs(c) - np.abs(tr), 0), np.abs(c))
    R[ts][0] += int(okr.sum()); R[ts][1] += int(flip.sum()); R[ts][2] += int(agree.sum()); R[ts][3].append(over[flip])
    for rg in ("straight", "entry", "curve", "exit"):
      E[ts].setdefault(rg, []).append((c - tr)[ok & (reg == rg)])
  print(r, flush=True)
rm = lambda e: np.sqrt(np.mean(e ** 2)) * 1e3
for ts in TS:
  n_ok, n_f, n_a, ov = R[ts]; ov = np.concatenate(ov)
  print(f"ts {ts}: early flip {n_f / n_ok:.2%} road agrees {n_a / max(n_f, 1):.0%} wrong-side p95 {np.percentile(ov, 95) * 1e3 if len(ov) else 0:.3f} | "
        f"RMSE " + " ".join(f"{rg} {rm(np.concatenate(E[ts][rg])):.3f}" for rg in ("straight", "entry", "curve", "exit")))
