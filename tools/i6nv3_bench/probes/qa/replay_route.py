"""Whole-route replay of the REAL state_control (see replay_state_control.py): 55b-build constants ('old', = what was
driven) vs shipped ('new'), lat-active frames only. Reports: fidelity vs the logged command, dropout-hold frames,
command steps (|change| > 1e-3/m in 0.3 s, >= 40 km/h), |new - old| p95/p99, and the mean (new - old) toward the
curve inside by road phase (straight / entry / steady / late-steady / exit, lanes from the plan npz).
Usage (repo root): PYTHONPATH=$PWD:$PWD/opendbc_repo python3 .../replay_route.py <scratch> <route> [<route> ...]"""
import sys

import numpy as np

import replay_state_control as R  # noqa: E402
from common import bfit, interp_rows  # noqa: E402

S = sys.argv[1]; routes = sys.argv[2:]
import os
VARS = os.environ.get("VARIANTS", "logged,new").split(",")
agg = {v: {} for v in VARS}
for r in routes:
  R.load(S, r)
  P, d = R.P, R.d
  Tr = d['T']; lat = d['lat']
  t0, t1 = Tr[np.argmax(lat)] + 8.5, Tr[len(lat) - 1 - np.argmax(lat[::-1])]
  res = {v: R.replay(v, t0, t1) for v in VARS}
  T = np.array([o[0] for o in res["logged"]]); ko = np.array([o[1] for o in res["logged"]])
  j = np.searchsorted(Tr, T); act = lat[j] & (d['v'][j] >= 30)
  logged = d['k_ctrl'][j]
  n = len(P['T'])
  LX = P['LX'].astype(float); LC = 0.5 * (P['L1'].astype(float) + P['L2'].astype(float))
  xs = np.tile(np.linspace(0, 20, 9), (n, 1)); kr = 2 * bfit(xs, interp_rows(xs, LX, LC), 2)[:, 0]
  good = (P['PR'][:, 1] > 0.7) & (P['PR'][:, 2] > 0.7)
  i = np.clip(np.searchsorted(P['T'], T) - 1, 0, n - 1)
  i6 = np.clip(np.searchsorted(P['T'], T + 0.6) - 1, 0, n - 1); i15 = np.clip(np.searchsorted(P['T'], T + 1.5) - 1, 0, n - 1)
  k0, k6, k15 = kr[i], kr[i6], kr[i15]
  ph = np.where(np.maximum(np.abs(k0), np.abs(k6)) < 0.5e-3, 0, np.where(np.maximum(np.abs(k0), np.abs(k6)) >= 1e-3,
       np.where(np.abs(k6) > np.abs(k0) + 0.7e-3, 1, np.where(np.abs(k0) > np.abs(k6) + 0.7e-3, 3,
       np.where(np.abs(k15) < np.abs(k0) - 0.7e-3, 4, 2))), -1))
  sgn = np.sign(np.where(np.abs(k0) > np.abs(k6), k0, k6))
  g = act & good[i]
  for name in VARS:
    k = np.array([o[1] for o in res[name]]); dd = np.array([o[3] for o in res[name]])
    a = agg[name]
    a.setdefault("fid", []).append(np.abs(ko - logged)[act])
    a.setdefault("drop", []).append(dd[act])
    step = np.abs(k[30:] - k[:-30]) > 1e-3
    a.setdefault("steps", 0); a["steps"] += int(np.sum(step & act[30:] & (d['v'][j][30:] >= 40) & ~np.r_[False, step[:-1]]))
    a.setdefault("min", 0.0); a["min"] += act.sum() * 0.01 / 60
    a.setdefault("diff", []).append(np.abs(k - ko)[act])
    for p in (0, 1, 2, 4, 3):
      m = g & (ph == p)
      a.setdefault(p, []).append(((k - ko) * (sgn if p else 1))[m])
  print(r, "done", flush=True)
o = agg["logged"]
print(f"replay fidelity (driven constants vs logged k_ctrl): p50 {np.median(np.concatenate(o['fid'])) * 1e3:.3f} p95 "
      f"{np.percentile(np.concatenate(o['fid']), 95) * 1e3:.3f} e-3/m over {o['min']:.0f} min")
for v in VARS:
  a = agg[v]; dfx = np.concatenate(a['diff'])
  ph_txt = " ".join(f"{nm} {np.mean(np.concatenate(a[p])) * 1e3:+.3f}" for p, nm in ((1, "entry"), (2, "steady"), (4, "late"), (3, "exit")))
  print(f"{v:8} drop {np.concatenate(a['drop']).mean():.2%} steps {a['steps'] / a['min'] * 60:6.1f}/h | vs driven: |d| p95 "
        f"{np.percentile(dfx, 95) * 1e3:.3f} straight RMS {np.sqrt(np.mean(np.concatenate(a[0]) ** 2)) * 1e3:.3f} | toward inside: {ph_txt}")
