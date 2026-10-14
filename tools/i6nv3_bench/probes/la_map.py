"""Lookahead study, step 3: (a) sample-time x target-lag map: RMSE of predictor sampled at tau_s from frame t vs road curvature
at the car at t + L, in curve/entry/exit frames — which tau_s best serves a given total delay L; (b) straight-road jitter:
std of frame-to-frame differences (20 Hz) of each predictor at tau_s (noise the command would carry), vs the road truth's own.
Usage: la_map.py <scratch dir>"""
import glob
import sys

import numpy as np

sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from la_fast import bfit, interp_rows, interp_tau  # noqa: E402

S = sys.argv[1]
TS = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0])
LS = np.array([0.3, 0.4, 0.5, 0.6])
acc = {}      # (pred, region, tau_s, L) -> list of errors
jit = {}      # (pred, tau_s) -> list of diffs on straights
for pf in sorted(glob.glob(f"{S}/la/plan_*.npz")):
  r = pf[-12:-4]
  P = np.load(pf)
  T = P['T']
  n = len(T)
  rf = glob.glob(f"{S}/ropm/{r}.npz") + glob.glob(f"{S}/r43/{r}.npz")
  d = np.load(rf[0], allow_pickle=True)
  v = np.interp(T, d['T'], d['v']) / 3.6
  busy = np.interp(T, d['T'], (d['lb'] | d['rb'] | (d['lcs'] > 0)).astype(float)) > 0
  PR = P['PR']
  good = (PR[:, 1] > 0.7) & (PR[:, 2] > 0.7) & ~busy & (v * 3.6 >= 30)
  K = P['K'].astype(float)
  PX, PY = P['PX'].astype(float), P['PY'].astype(float)
  LX = P['LX'].astype(float)
  LC = 0.5 * (P['L1'].astype(float) + P['L2'].astype(float))
  OR, VX = P['OR'].astype(float), P['VX'].astype(float)
  C3 = bfit(PX[:, :12], PY[:, :12], 3)
  xs0 = np.tile(np.linspace(0, 20, 9), (n, 1))
  truth = 2.0 * bfit(xs0, interp_rows(xs0, LX, LC), 2)[:, 0]
  truth[~good] = np.nan
  preds = {}
  for ts in TS:
    dd = v * ts
    xl = np.maximum(dd[:, None] + np.linspace(-10, 10, 9)[None, :], 0.0)
    preds[("cubic", ts)] = 6.0 * C3[:, 0] * dd + 2.0 * C3[:, 1]
    preds[("orient", ts)] = interp_tau(ts, OR) / np.maximum(interp_tau(ts, VX), 1.0)
    preds[("lane", ts)] = 2.0 * bfit(xl, interp_rows(xl, LX, LC), 2)[:, 0]
  preds[("action", 0.0)] = K
  # (a) map
  for L in LS:
    j = np.minimum(np.searchsorted(T, T + L), n - 1)
    ok = good & (np.abs(T[j] - (T + L)) < 0.03) & ~np.isnan(truth[j])
    tr, tr0 = truth[j], truth
    reg = np.where(np.abs(tr) > np.abs(tr0) + 0.7e-3, "entry", np.where(np.abs(tr0) > np.abs(tr) + 0.7e-3, "exit",
                   np.where(np.abs(tr) >= 1.5e-3, "curve", "other")))
    for (pn, ts), p in preds.items():
      e = (p - tr)[ok]
      for rg in ("entry", "curve", "exit"):
        acc.setdefault((pn, rg, ts, L), []).append(e[reg[ok] == rg])
  # (b) jitter on straights (truth small at t and t+0.05), consecutive frames
  st = good & (np.abs(np.nan_to_num(truth, nan=1.0)) < 0.5e-3)
  cons = st[1:] & st[:-1] & (np.diff(T) < 0.07)
  for (pn, ts), p in list(preds.items()) + [(("truth", 0.0), truth)]:
    jit.setdefault((pn, ts), []).append(np.diff(p)[cons])
  print(r, flush=True)

rm = lambda e: np.sqrt(np.mean(e ** 2)) * 1e3 if len(e) else float('nan')
for rg in ("entry", "curve", "exit"):
  print(f"== {rg}: RMSE (1e-3/m); rows = sample tau_s, cols = target lag L")
  for pn in ("cubic", "orient", "lane"):
    print(f"  {pn:7} tau_s " + "".join(f"  L={L:.1f}" for L in LS))
    for ts in TS:
      print(f"          {ts:4.1f} " + "".join(f"{rm(np.concatenate(acc[(pn, rg, ts, L)])):7.3f}" for L in LS))
  print("  action  (no tau) " + "".join(f"{rm(np.concatenate(acc[('action', rg, 0.0, L)])):7.3f}" for L in LS)
        + "   n " + " ".join(str(len(np.concatenate(acc[('cubic', rg, 0.1, L)]))) for L in LS))
print("== straight-road jitter: std of 20 Hz frame-to-frame change (1e-3/m)")
print("   road truth itself: %.3f" % (np.std(np.concatenate(jit[("truth", 0.0)])) * 1e3))
print("   action: %.3f" % (np.std(np.concatenate(jit[("action", 0.0)])) * 1e3))
for pn in ("cubic", "orient", "lane"):
  print(f"   {pn:7} " + " ".join(f"tau {ts:.1f}: {np.std(np.concatenate(jit[(pn, ts)])) * 1e3:.3f}" for ts in TS))
