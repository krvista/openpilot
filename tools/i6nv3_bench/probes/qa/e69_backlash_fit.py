"""E69: MDPS plant WITH BACKLASH (play), output-error fit. An internal target y follows the sent angle u (delayed d) through a
play of width B (y moves only once |u - y| > B, then y = u -+ B), and the wheel follows y:
   d w/dt = Kf * dy/dt + Kp * (y - w) - Kc * w
B = 0 reduces to E59's M1. Data: E59 run cache rebuilt to include the 31520b9 routes (76-79, where Phase 59 moved TX a
lot: the play is excited). 2-fold by route; per band report M1 vs backlash RMS on the other fold, and separately on the
31520b9 runs (the condition that failed)."""
import glob
import json
import os
import pickle
import sys
import numpy as np
from numba import njit
from scipy.optimize import minimize
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; DT = 0.02
BANDS = [(15, 30), (30, 40), (40, 55), (55, 70), (70, 90), (90, 200)]
cache = f"{S}/qa/e69_runs.pkl"
files = sorted(glob.glob(f"{S}/r50/0*.npz") + glob.glob(f"{S}/r_old/0*.npz"), key=lambda f: f[-12:-4])
if os.path.exists(cache) and pickle.load(open(cache + ".files", "rb")) == files:
  runs = pickle.load(open(cache, "rb"))
else:
  runs = []
  for f in files:
    d = np.load(f, allow_pickle=True)
    T = d['T']; n = len(T); v = d['v']
    if n < 3000 or 'gain' not in d:
      continue
    vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
    hold = np.array([compute_hold_torque(a, b) for a, b in zip(vm[::10], la[::10])]).repeat(10)[:n]
    dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
    ok = d['lat'] & (d['tx_active'] == 2) & (d['gain'] >= 0.95) & (dd < 40) & ~d['pressed'] & (v >= 15) & ~(d['lb'] | d['rb'])
    ok &= np.isfinite(d['wire']) & np.isfinite(d['ang'])
    c = np.concatenate([[0], np.cumsum(~ok)]); idx = np.arange(n)
    ok2 = ok & (c[idx + 1] - c[np.maximum(idx - 50, 0)] == 0)
    e = np.flatnonzero(np.diff(np.concatenate([[0], ok2.astype(int), [0]])))
    for a, b in zip(e[::2], e[1::2]):
      a0 = max(a - 50, 0)
      if T[b - 1] - T[a] < 3.0:
        continue
      tg = np.arange(T[a0], T[b - 1], DT)
      R = {k: np.interp(tg, T[a0:b], d[k][a0:b]) for k in ('wire', 'ang', 'v')}
      R['h0'] = int(round((T[a] - T[a0]) / DT)); R['route'] = f[-12:-4]; R['commit'] = str(d['commit'])[:7]
      vmed = np.median(R['v'][R['h0']:])
      R['band'] = next(i for i, (lo, hi) in enumerate(BANDS) if lo <= vmed < hi)
      runs.append(R)
  pickle.dump(runs, open(cache, "wb")); pickle.dump(files, open(cache + ".files", "wb"))
routes = sorted({R['route'] for R in runs}); fold = {r: i % 2 for i, r in enumerate(routes)}
new = [R for R in runs if R['commit'] == '31520b9']
print(f"runs {len(runs)} ({sum(len(R['ang']) - R['h0'] for R in runs) * DT / 60:.0f} min), of which 31520b9 {len(new)} ({sum(len(R['ang']) - R['h0'] for R in new) * DT / 60:.1f} min)", flush=True)


@njit(cache=True)
def sim_bl(u, w0, h0, nd, kf, kp, kc, B):
  n = len(u); w = np.empty(n)
  for k in range(h0 + 1):
    w[k] = w0[k]
  i0 = h0 - nd if h0 - nd >= 0 else 0
  y = u[i0]
  for k in range(h0 + 1, n):
    i1 = k - 1 - nd
    u1 = u[i1] if i1 >= 0 else u[0]
    yp = y
    if u1 - y > B:
      y = u1 - B
    elif y - u1 > B:
      y = u1 + B
    w[k] = w[k - 1] + DT * (kf * (y - yp) / DT + kp * (y - w[k - 1]) - kc * w[k - 1])
  return w


def sse(p, P):
  nd = int(round(p[0] / DT)); se = 0.0; nn = 0
  for u, w0, h0 in P:
    w = sim_bl(u, w0, h0, nd, p[1], p[2], p[3], p[4]); se += np.sum((w[h0:] - w0[h0:]) ** 2); nn += len(w) - h0
  return se, nn


LB = np.array([0.05, 0.0, -0.3, 0.0]); UB = np.array([1.5, 5.0, 2.0, 3.0])


def fit(P, with_B=True):
  best = None
  for d0 in (0.1, 0.2, 0.3):
    for b0 in ((0.0, 0.5, 1.0) if with_B else (0.0,)):
      def f(q):
        q = np.clip(q, LB, UB)
        if not with_B:
          q[3] = 0.0
        return sse([d0, *q], P)[0]
      r = minimize(f, [0.6, 0.15, 0.1, b0], method="Nelder-Mead", options=dict(maxiter=500, xatol=1e-3, fatol=1e-3))
      q = np.clip(r.x, LB, UB)
      if not with_B:
        q[3] = 0.0
      if best is None or r.fun < best[0]:
        best = (r.fun, [d0, *q.tolist()])
  return best[1]


def rms(p, P):
  s, n = sse(p, P)
  return np.sqrt(s / max(n, 1)) if n else np.nan


out = {}
pk = lambda rs: [(R['wire'], R['ang'], R['h0']) for R in rs]
for b, (lo, hi) in enumerate(BANDS):
  if b == 0:
    continue
  rb = [R for R in runs if R['band'] == b]
  if len(rb) < 10:
    continue
  A = pk([R for R in rb if fold[R['route']] == 0]); Bf = pk([R for R in rb if fold[R['route']] == 1]); N = pk([R for R in rb if R['commit'] == '31520b9'])
  res = {}
  for nm, wb in (("M1", False), ("BL", True)):
    pa, pb = fit(A, wb), fit(Bf, wb)
    pall = fit(pk(rb), wb)
    res[nm] = (np.sqrt((sse(pa, Bf)[0] + sse(pb, A)[0]) / (sse(pa, Bf)[1] + sse(pb, A)[1])), pall, pa, pb)
    out[f"{nm}{b}"] = pall; out[f"{nm}A{b}"] = pa; out[f"{nm}B{b}"] = pb
  print(f"[{lo}-{hi}] runs {len(rb)} (31520b9 {len(N)}) | cross-fold RMS M1 {res['M1'][0]:.2f} vs backlash {res['BL'][0]:.2f} deg | on 31520b9 runs (all-data fit) M1 {rms(res['M1'][1], N):.2f} vs backlash {rms(res['BL'][1], N):.2f} "
        f"| backlash all-data d {res['BL'][1][0]:.2f} Kf {res['BL'][1][1]:.3f} Kp {res['BL'][1][2]:.3f} Kc {res['BL'][1][3]:.3f} B {res['BL'][1][4]:.2f} deg", flush=True)
json.dump(out, open(f"{S}/qa/e69_params.json", "w"), indent=1)
