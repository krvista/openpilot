"""Lookahead study, step 4: candidate lookahead sources pushed through the CURRENT command shaping (gate 0.8e-3, blend
0.8->1.5e-3, jerk budget dk_max = 0.7 t / v^2, bounded additive on the model action) — final command error vs road curvature
at t + L (L = 0.45 s: plan age 0.08 + actuator/chassis ~0.37), by regime and speed band, plus straight-road jitter and
command jerk proxy. Variants:
  current : cubic at t_ahead = min(base + boost, 0.32) + tau_lp, 10 m clamp
  cubic06 : cubic at 0.6 s + tau_lp, 25 m clamp
  orient09: plan yaw rate / speed at 0.9 s (+ tau_lp), budget from the same t_ahead
  orientB : as orient09 with the budget time fixed at the current t_ahead (no larger jerk allowance)
Usage: la_cmd.py <scratch dir>"""
import glob
import sys

import numpy as np

sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from la_fast import bfit, interp_rows, interp_tau  # noqa: E402

S = sys.argv[1]
L_TGT = 0.45
VARS = ("current", "cubic06", "orient09", "orientB", "action")
err = {}
jit = {}
for pf in sorted(glob.glob(f"{S}/la/plan_*.npz")):
  r = pf[-12:-4]
  P = np.load(pf)
  T = P['T']
  n = len(T)
  rf = glob.glob(f"{S}/ropm/{r}.npz") + glob.glob(f"{S}/r43/{r}.npz")
  d = np.load(rf[0], allow_pickle=True)
  v = np.interp(T, d['T'], d['v']) / 3.6
  lat = np.interp(T, d['T'], d['lat'].astype(float)) > 0.5
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
  ab = np.abs(K)
  tau_lp = np.interp(v, [8.0, 13.0, 18.0], [0.30, 0.12, 0.08])
  base = np.interp(v, [5.6, 13.9, 27.8, 38.9], [0.08, 0.10, 0.13, 0.18])
  boost = np.interp(ab, [0.0008, 0.005], [0.0, 0.20])
  blend = np.interp(ab, [0.0008, 0.0015], [0.0, 1.0]) * (ab >= 0.0008)
  vb = np.maximum(v, 5.0) ** 2

  def shaped(pred, t_budget):
    return K + blend * np.clip(pred - K, -0.7 * np.maximum(t_budget, 0.05) / vb, 0.7 * np.maximum(t_budget, 0.05) / vb)

  t_cur = np.minimum(base + boost, 0.32) + tau_lp
  d_cur = np.minimum(v * t_cur, 10.0)
  cmds = {"current": shaped(6.0 * C3[:, 0] * d_cur + 2.0 * C3[:, 1], t_cur)}
  t6 = 0.6 + tau_lp
  d6 = np.minimum(v * t6, 25.0)
  cmds["cubic06"] = shaped(6.0 * C3[:, 0] * d6 + 2.0 * C3[:, 1], t6)
  k_or = np.zeros(n)
  for i in range(n):
    tq = 0.9 + tau_lp[i]
    k_or[i] = np.interp(tq, interp_tau.__globals__['T_IDXS'], OR[i]) / max(np.interp(tq, interp_tau.__globals__['T_IDXS'], VX[i]), 1.0)
  cmds["orient09"] = shaped(k_or, 0.9 + tau_lp)
  cmds["orientB"] = shaped(k_or, t_cur)
  cmds["action"] = K.copy()
  j = np.minimum(np.searchsorted(T, T + L_TGT), n - 1)
  ok = good & lat & (np.abs(T[j] - (T + L_TGT)) < 0.03) & ~np.isnan(truth[j])
  tr, tr0 = truth[j], truth
  reg = np.where((np.abs(tr) < 0.5e-3) & (np.abs(tr0) < 0.5e-3), "straight", np.where(np.abs(tr) > np.abs(tr0) + 0.7e-3, "entry",
                 np.where(np.abs(tr0) > np.abs(tr) + 0.7e-3, "exit", np.where(np.abs(tr) >= 1.5e-3, "curve", "mild"))))
  band = np.where(v * 3.6 < 50, "30-50", np.where(v * 3.6 < 70, "50-70", "70+"))
  st = good & lat & (np.abs(np.nan_to_num(truth, nan=1.0)) < 0.5e-3)
  cons = st[1:] & st[:-1] & (np.diff(T) < 0.07)
  for vn, c in cmds.items():
    e = (c - tr)[ok]
    for rg in ("straight", "entry", "curve", "exit"):
      for bd in ("30-50", "50-70", "70+", "all"):
        m = (reg[ok] == rg) & ((band[ok] == bd) if bd != "all" else True)
        err.setdefault((vn, rg, bd), []).append(e[m])
    jit.setdefault(vn, []).append(np.diff(c)[cons])
  print(r, flush=True)

rm = lambda e: np.sqrt(np.mean(e ** 2)) * 1e3 if len(e) else float('nan')
print(f"== final command error vs road curvature at t + {L_TGT} s (RMSE 1e-3/m), lat active, both lines > 0.7")
for rg in ("straight", "entry", "curve", "exit"):
  print(f"  {rg}:")
  for bd in ("30-50", "50-70", "70+", "all"):
    n_ = len(np.concatenate(err[("current", rg, bd)]))
    print(f"    {bd:6} n {n_:6d} " + " ".join(f"{vn} {rm(np.concatenate(err[(vn, rg, bd)])):6.3f}" for vn in VARS))
print("== straight-road command jitter (std of 20 Hz change, 1e-3/m): " + " ".join(f"{vn} {np.std(np.concatenate(jit[vn])) * 1e3:.3f}" for vn in VARS))
