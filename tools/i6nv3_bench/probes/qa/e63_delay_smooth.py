"""E63: make the per-band MDPS delay physically consistent. Three ways to set d per band, then refit (Kf, Kp, Kc) at that d and
compare route-fold cross-validated output-error RMS:
  free   : E59 (d free per band, grid 0.04 s)
  common : one d shared by all bands (grid 0.00-0.40 s, chosen on the training fold)
  linear : d = a + b * v (km/h), a/b on a grid, chosen on the training fold
Uses the E59 run cache (50 Hz, gain >= 0.95 hands-off)."""
import json
import pickle
import sys
import numpy as np
from numba import njit
from scipy.optimize import minimize
S = sys.argv[1]; DT = 0.02
runs = pickle.load(open(f"{S}/qa/e59_runs.pkl", "rb"))
BANDS = [(15, 30), (30, 40), (40, 55), (55, 70), (70, 90), (90, 200)]
CENT = {1: 35.0, 2: 47.5, 3: 62.5, 4: 80.0, 5: 100.0}
routes = sorted({R['route'] for R in runs}); fold = {r: i % 2 for i, r in enumerate(routes)}


@njit(cache=True)
def sim_m1(u, w0, h0, nd, kf, kp, kc):
  n = len(u); w = np.empty(n)
  for k in range(h0 + 1):
    w[k] = w0[k]
  for k in range(h0 + 1, n):
    i1 = k - 1 - nd; i2 = k - 2 - nd
    u1 = u[i1] if i1 >= 0 else u[0]; u2 = u[i2] if i2 >= 0 else u[0]
    w[k] = w[k - 1] + DT * (kf * (u1 - u2) / DT + kp * (u1 - w[k - 1]) - kc * w[k - 1])
  return w


def sse(p, P):
  nd = int(round(p[0] / DT)); se = 0.0; nn = 0
  for u, w0, h0 in P:
    w = sim_m1(u, w0, h0, nd, p[1], p[2], p[3]); se += np.sum((w[h0:] - w0[h0:]) ** 2); nn += len(w) - h0
  return se, nn


def fit_at(d0, P):
  f = lambda q: sse([d0, *np.clip(q, [0.05, 0.0, -0.3], [1.5, 5.0, 2.0])], P)[0]
  r = minimize(f, [0.55, 0.15, 0.1], method="Nelder-Mead", options=dict(maxiter=400, xatol=1e-3, fatol=1e-3))
  return [d0, *np.clip(r.x, [0.05, 0.0, -0.3], [1.5, 5.0, 2.0]).tolist()], r.fun


def packs(fo):
  return {b: [(R['wire'], R['ang'], R['h0']) for R in runs if R['band'] == b and (fo is None or fold[R['route']] == fo)] for b in range(1, 6)}


DG = np.arange(0.0, 0.42, 0.04)
res = {}
for tr, te in ((0, 1), (1, 0)):
  Ptr, Pte = packs(tr), packs(te)
  cache = {(b, round(d, 2)): fit_at(d, Ptr[b]) for b in range(1, 6) for d in DG}     # train fits at every d
  def eval_choice(dsel):
    se = nn = 0.0; per = {}
    for b in range(1, 6):
      p, _ = cache[(b, round(dsel[b], 2))]
      s, n = sse(p, Pte[b]); se += s; nn += n; per[b] = (np.sqrt(s / max(n, 1)), p)
    return np.sqrt(se / nn), per
  free = {b: min(DG, key=lambda d: cache[(b, round(d, 2))][1]) for b in range(1, 6)}
  common = min(DG, key=lambda d: sum(cache[(b, round(d, 2))][1] for b in range(1, 6)))
  best_lin = None
  for a in np.arange(0.0, 0.42, 0.04):
    for slope in np.arange(-0.004, 0.0041, 0.001):
      dsel = {b: DG[np.argmin(np.abs(DG - np.clip(a + slope * (CENT[b] - 35.0), 0.0, 0.4)))] for b in range(1, 6)}
      tot = sum(cache[(b, round(dsel[b], 2))][1] for b in range(1, 6))
      if best_lin is None or tot < best_lin[0]:
        best_lin = (tot, a, slope, dsel)
  for nm, dsel in (("free", free), ("common", {b: common for b in range(1, 6)}), ("linear", best_lin[3])):
    tot, per = eval_choice(dsel)
    res.setdefault(nm, []).append((tot, per, dsel))
    print(f"train fold {tr} -> test fold {te} | {nm:6}: test RMS {tot:.3f} | d " + " ".join(f"{dsel[b]:.2f}" for b in range(1, 6))
          + " | per band " + " ".join(f"{per[b][0]:.2f}" for b in range(1, 6)) + (f" | line d = {best_lin[1]:.2f} + {best_lin[2]:+.4f}*(v-35)" if nm == "linear" else ""), flush=True)
for nm, L in res.items():
  print(f"{nm:6}: mean test RMS {np.mean([x[0] for x in L]):.3f}")
# final fit on all data for the 'linear' and 'common' rules
Pall = packs(None)
for nm in ("linear", "common"):
  dsel = res[nm][0][2] if nm == "common" else None
out = {}
for nm in ("free", "common", "linear"):
  ds = [np.mean([L[2][b] for L in res[nm]]) for b in range(1, 6)]
  rows = [fit_at(float(DG[np.argmin(np.abs(DG - ds[b - 1]))]), Pall[b])[0] for b in range(1, 6)]
  out[nm] = rows
  print(f"ALL-DATA {nm:6}: " + " | ".join(f"{CENT[b]:.0f}: d {rows[b - 1][0]:.2f} Kf {rows[b - 1][1]:.3f} Kp {rows[b - 1][2]:.3f} Kc {rows[b - 1][3]:.3f}" for b in range(1, 6)))
json.dump(out, open(f"{S}/qa/e63_params.json", "w"), indent=1)
