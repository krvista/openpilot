"""Whole-route replay of the REAL state_control (see replay_state_control.py): 55b-build constants ('old', = what was
driven) vs shipped ('new'), lat-active frames only. Reports: fidelity vs the logged command, dropout-hold frames,
command steps (|change| > 1e-3/m in 0.3 s, >= 40 km/h), |new - old| p95/p99, and the mean (new - old) toward the
curve inside by road phase (straight / entry / steady / late-steady / exit, lanes from the plan npz).
Usage (repo root): PYTHONPATH=$PWD:$PWD/opendbc_repo python3 .../replay_route.py <scratch> <route> [<route> ...]"""
import os
import sys

import numpy as np

import replay_state_control as R  # noqa: E402
from common import bfit, interp_rows  # noqa: E402

S = sys.argv[1]; routes = sys.argv[2:]
if os.environ.get("MERGE"):
  import pickle
  parts = [pickle.load(open(p, "rb")) for p in os.environ["MERGE"].split(",")]
  agg = parts[0]
  for pt in parts[1:]:
    for v, a in pt.items():
      for k, val in a.items():
        if isinstance(val, list):
          agg[v].setdefault(k, []).extend(val)
        elif isinstance(val, dict):
          for b, rs in val.items():
            agg[v].setdefault(k, {}).setdefault(b, []).extend(rs)
        else:
          agg[v][k] = agg[v].get(k, 0) + val
  routes = []
import os
VARS = os.environ.get("VARIANTS", "driven,new").split(",")
agg = {v: {} for v in VARS} if not os.environ.get('MERGE') else agg
for r in routes:
  R.load(S, r)
  P, d = R.P, R.d
  driven = R.DRIVEN.get(str(d['commit']), "cubic")
  Tr = d['T']; lat = d['lat']
  t0, t1 = Tr[np.argmax(lat)] + 8.5, Tr[len(lat) - 1 - np.argmax(lat[::-1])]
  res = {v: R.replay(v, t0, t1) for v in set(VARS) | {driven}}
  res["driven"] = res[driven]
  T = np.array([o[0] for o in res["driven"]]); ko = np.array([o[1] for o in res["driven"]])
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
    if name == "driven":
      a.setdefault("builds", {}).setdefault(str(d['commit']), []).append(r)
    a.setdefault("fid", []).append(np.abs(ko - logged)[act])
    a.setdefault("drop", []).append(dd[act])
    step = np.abs(k[30:] - k[:-30]) > 1e-3
    a.setdefault("steps", 0); a["steps"] += int(np.sum(step & act[30:] & (d['v'][j][30:] >= 40) & ~np.r_[False, step[:-1]]))
    a.setdefault("min", 0.0); a["min"] += act.sum() * 0.01 / 60
    a.setdefault("diff", []).append(np.abs(k - ko)[act])
    for p in (0, 1, 2, 4, 3):
      m = g & (ph == p)
      a.setdefault(p, []).append(((k - ko) * (sgn if p else 1))[m])
  per = {v: int(np.sum((np.abs(np.array([o[1] for o in res[v]])[30:] - np.array([o[1] for o in res[v]])[:-30]) > 1e-3) & act[30:]
                       & (d['v'][j][30:] >= 40))) for v in VARS}
  print(r, "done", f"{act.sum() / 6000:.0f} min", " ".join(f"{v}:{per[v]}" for v in VARS), flush=True)
import pickle
if os.environ.get("OUT"):
  pickle.dump(agg, open(os.environ["OUT"], "wb")); sys.exit(0)
o = agg["driven"]
print("builds:", {b: len(v) for b, v in o.get("builds", {}).items()})
print(f"replay fidelity (driven constants vs logged k_ctrl): p50 {np.median(np.concatenate(o['fid'])) * 1e3:.3f} p95 "
      f"{np.percentile(np.concatenate(o['fid']), 95) * 1e3:.3f} e-3/m over {o['min']:.0f} min")
for v in VARS:
  a = agg[v]; dfx = np.concatenate(a['diff'])
  ph_txt = " ".join(f"{nm} {np.mean(np.concatenate(a[p])) * 1e3:+.3f}" for p, nm in ((1, "entry"), (2, "steady"), (4, "late"), (3, "exit")))
  print(f"{v:8} drop {np.concatenate(a['drop']).mean():.2%} steps {a['steps'] / a['min'] * 60:6.1f}/h | vs driven: |d| p95 "
        f"{np.percentile(dfx, 95) * 1e3:.3f} straight RMS {np.sqrt(np.mean(np.concatenate(a[0]) ** 2)) * 1e3:.3f} | toward inside: {ph_txt}")
