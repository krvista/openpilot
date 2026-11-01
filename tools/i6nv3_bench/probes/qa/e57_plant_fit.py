"""E57: MDPS plant identification by OUTPUT ERROR (simulate the wheel from the sent TX angle, minimise the simulated-wheel
RMS), per speed band, train / test split by route (odd / even route index). Data: op active, LKAS_ALT active, ACI gain
>= 0.95 for the last 0.5 s, hold-compensated driver torque < 40, not pressed, no blinker, >= 15 km/h; runs >= 3 s; 50 Hz.
Models (w = wheel, u = TX delayed by d):
  M0  w = u                                          (pure delay)
  M0b w' = (g*u - w) / tau                            (first-order lag, static gain g)
  M1  w' = Kf*u' + Kp*(u - w) - Kc*w                  (rate following + slow error closing + self-centring)
  M2  M1 with a deadband D on the error term and a rate threshold R0 below which u' is ignored
Saves the runs to qa/e57_runs.pkl and the fitted parameters to qa/e57_params.json."""
import glob
import json
import pickle
import sys
import numpy as np
from scipy.optimize import minimize
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]
DT = 0.02
BANDS = [(15, 40), (40, 70), (70, 200)]
runs = []
files = sorted(glob.glob(f"{S}/r50/0*.npz"))
for fi, f in enumerate(files):
  d = np.load(f, allow_pickle=True)
  T = d['T']; n = len(T); v = d['v']
  if n < 3000:
    continue
  vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, b) for a, b in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  ok = d['lat'] & (d['tx_active'] == 2) & (d['gain'] >= 0.95) & (dd < 40) & ~d['pressed'] & (v >= 15) & ~(d['lb'] | d['rb'])
  c = np.concatenate([[0], np.cumsum(~ok)]); idx = np.arange(n)
  ok2 = ok & (c[idx + 1] - c[np.maximum(idx - 50, 0)] == 0)
  e = np.flatnonzero(np.diff(np.concatenate([[0], ok2.astype(int), [0]])))
  for a, b in zip(e[::2], e[1::2]):
    a0 = max(a - 50, 0)                                   # 0.5 s of TX history before the run (for the delay)
    if T[b - 1] - T[a] < 3.0:
      continue
    tg = np.arange(T[a0], T[b - 1], DT)
    R = {k: np.interp(tg, T[a0:b], d[k][a0:b]) for k in ('wire', 'ang', 'v', 'apply', 'k_meas')}
    R['h0'] = int(round((T[a] - T[a0]) / DT)); R['route'] = f[-12:-4]; R['test'] = fi % 2 == 1
    R['band'] = next(i for i, (lo, hi) in enumerate(BANDS) if lo <= np.median(R['v']) < hi)
    runs.append(R)
pickle.dump(runs, open(f"{S}/qa/e57_runs.pkl", "wb"))
print(f"runs {len(runs)} ({sum(len(r['ang']) for r in runs) * DT / 60:.0f} min); per band train/test:",
      [(sum(1 for r in runs if r['band'] == b and not r['test']), sum(1 for r in runs if r['band'] == b and r['test'])) for b in range(3)])


def sim(model, p, R):
  u = R['wire']; w0 = R['ang']; h0 = R['h0']
  d = int(round(p[0] / DT))
  n = len(u); w = np.empty(n); w[:h0 + 1] = w0[:h0 + 1]
  ud = np.concatenate([np.full(d, u[0]), u[:n - d]]) if d > 0 else u
  ur = np.concatenate([[0.0], np.diff(ud) / DT])
  if model == "M0":
    return ud
  for k in range(h0 + 1, n):
    wk = w[k - 1]
    if model == "M0b":
      g, tau = p[1], p[2]
      w[k] = wk + DT * (g * ud[k - 1] - wk) / max(tau, 0.02)
    else:
      Kf, Kp, Kc = p[1], p[2], p[3]
      err = ud[k - 1] - wk; rate = ur[k - 1]
      if model == "M2":
        D, R0 = p[4], p[5]
        err = np.sign(err) * max(abs(err) - D, 0.0)
        rate = rate if abs(rate) >= R0 else 0.0
      w[k] = wk + DT * (Kf * rate + Kp * err - Kc * wk)
  return w


def cost(model, p, rs):
  se = 0.0; nn = 0
  for R in rs:
    w = sim(model, p, R); h0 = R['h0']
    se += np.sum((w[h0:] - R['ang'][h0:]) ** 2); nn += len(w) - h0
  return np.sqrt(se / max(nn, 1))


X0 = {"M0": [0.2], "M0b": [0.2, 0.8, 0.5], "M1": [0.2, 0.5, 0.1, 0.0], "M2": [0.2, 0.5, 0.1, 0.0, 0.3, 1.0]}
LB = {"M0": [0.0], "M0b": [0.0, 0.1, 0.02], "M1": [0.0, 0.0, 0.0, -0.5], "M2": [0.0, 0.0, 0.0, -0.5, 0.0, 0.0]}
UB = {"M0": [0.6], "M0b": [0.6, 1.5, 5.0], "M1": [0.6, 1.5, 5.0, 2.0], "M2": [0.6, 1.5, 5.0, 2.0, 3.0, 10.0]}
out = {}
for b, (lo, hi) in enumerate(BANDS):
  tr = [R for R in runs if R['band'] == b and not R['test']]; te = [R for R in runs if R['band'] == b and R['test']]
  if len(tr) < 5 or len(te) < 5:
    continue
  for model in ("M0", "M0b", "M1", "M2"):
    best = None
    for d0 in (0.05, 0.15, 0.3):
      x0 = list(X0[model]); x0[0] = d0
      f = lambda p: cost(model, np.clip(p, LB[model], UB[model]), tr)
      r = minimize(f, x0, method="Nelder-Mead", options=dict(maxiter=150 * len(x0), xatol=1e-3, fatol=1e-3))
      if best is None or r.fun < best[0]:
        best = (r.fun, np.clip(r.x, LB[model], UB[model]))
    p = best[1]
    out[f"{b}:{model}"] = p.tolist()
    print(f"[{lo}-{hi} km/h] {model:3} train RMS {best[0]:.2f} test RMS {cost(model, p, te):.2f} deg | params " + " ".join(f"{x:.3f}" for x in p), flush=True)
json.dump(out, open(f"{S}/qa/e57_params.json", "w"), indent=1)
