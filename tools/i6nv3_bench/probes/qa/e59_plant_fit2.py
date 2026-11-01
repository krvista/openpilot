"""E59: MDPS plant re-identification on ALL extracted routes (r50 + r_old), finer speed bands (30-40 separately; 15-30 for
information only — the cruise floor is 30 km/h), output-error fit with numba:
  d wheel/dt = Kf * du/dt + Kp * (u - wheel) - Kc * wheel,   u = TX delayed by d
Data: op active, LKAS_ALT active, ACI gain >= 0.95 (last 0.5 s), hold-compensated driver torque < 40, not pressed,
no blinker, runs >= 3 s, 50 Hz.
Outputs: (a) 2-fold by route (fold = route index parity): params fitted on each fold, RMS on the other fold;
(b) all-data params (qa/e59_params.json); (c) per-route fits (>= 60 s in the band) -> spread of Kf/Kp/Kc/d across
drives (is live adaptation worth it?) and their correlation with date order."""
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
S = sys.argv[1]
DT = 0.02
BANDS = [(15, 30), (30, 40), (40, 55), (55, 70), (70, 90), (90, 200)]
cache = f"{S}/qa/e59_runs.pkl"
files = sorted(glob.glob(f"{S}/r50/0*.npz") + glob.glob(f"{S}/r_old/0*.npz"), key=lambda f: f[-12:-4])
if os.path.exists(cache) and pickle.load(open(cache + ".files", "rb")) == files:
  runs = pickle.load(open(cache, "rb"))
else:
  runs = []
  for fi, f in enumerate(files):
    try:
      d = np.load(f, allow_pickle=True)
      T = d['T']; n = len(T); v = d['v']
      if n < 3000 or 'gain' not in d or 'wire' not in d:
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
        R = {k: np.interp(tg, T[a0:b], d[k][a0:b]) for k in ('wire', 'ang', 'v', 'apply')}
        R['h0'] = int(round((T[a] - T[a0]) / DT)); R['route'] = f[-12:-4]; R['commit'] = str(d['commit'])
        vmed = np.median(R['v'][R['h0']:])
        R['band'] = next(i for i, (lo, hi) in enumerate(BANDS) if lo <= vmed < hi)
        runs.append(R)
    except Exception as ex:  # noqa: BLE001
      print("skip", f[-12:-4], ex)
  pickle.dump(runs, open(cache, "wb")); pickle.dump(files, open(cache + ".files", "wb"))
routes = sorted({R['route'] for R in runs})
fold = {r: i % 2 for i, r in enumerate(routes)}
print(f"routes {len(routes)}, runs {len(runs)}, {sum(len(R['ang']) - R['h0'] for R in runs) * DT / 60:.0f} min usable")
for b, (lo, hi) in enumerate(BANDS):
  print(f"   {lo}-{hi} km/h: {sum(len(R['ang']) - R['h0'] for R in runs if R['band'] == b) * DT / 60:5.1f} min, {sum(1 for R in runs if R['band'] == b)} runs, {len({R['route'] for R in runs if R['band'] == b})} routes")


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


def pack(rs):
  return [(R['wire'], R['ang'], R['h0']) for R in rs]


def sse(p, P):
  nd = int(round(p[0] / DT)); se = 0.0; nn = 0
  for u, w0, h0 in P:
    w = sim_m1(u, w0, h0, nd, p[1], p[2], p[3])
    se += np.sum((w[h0:] - w0[h0:]) ** 2); nn += len(w) - h0
  return se, nn


def fit(P):
  best = None
  for d0 in np.arange(0.0, 0.42, 0.04):
    f = lambda q: np.sqrt(sse([d0, *np.clip(q, [0.05, 0.0, -0.3], [1.5, 5.0, 2.0])], P)[0] / max(sse([d0, 0.5, 0.1, 0.0], P)[1], 1))
    r = minimize(f, [0.55, 0.2, 0.1], method="Nelder-Mead", options=dict(maxiter=400, xatol=1e-3, fatol=1e-4))
    if best is None or r.fun < best[0]:
      best = (r.fun, [float(d0), *np.clip(r.x, [0.05, 0.0, -0.3], [1.5, 5.0, 2.0]).tolist()])
  return best


def rms(p, P):
  s, n = sse(p, P)
  return np.sqrt(s / max(n, 1))


out = {"bands": BANDS}
for b, (lo, hi) in enumerate(BANDS):
  rb = [R for R in runs if R['band'] == b]
  if len(rb) < 10:
    continue
  A = pack([R for R in rb if fold[R['route']] == 0]); B = pack([R for R in rb if fold[R['route']] == 1])
  fa = fit(A); fb = fit(B); fall = fit(pack(rb))
  base = lambda P: np.sqrt(sum(np.sum((np.concatenate([np.full(int(round(0.3 / DT)), u[0]), u[:len(u) - int(round(0.3 / DT))]])[h0:] - w0[h0:]) ** 2) for u, w0, h0 in P) / sum(len(w0) - h0 for u, w0, h0 in P))
  print(f"[{lo}-{hi}] fold A params {np.round(fa[1], 3).tolist()} -> RMS on B {rms(fa[1], B):.2f} (B's own fit {fb[0]:.2f}) | fold B params {np.round(fb[1], 3).tolist()} -> RMS on A {rms(fb[1], A):.2f} (A's own {fa[0]:.2f}) "
        f"| all {np.round(fall[1], 3).tolist()} RMS {fall[0]:.2f} | pure 0.3 s delay {base(pack(rb)):.2f}", flush=True)
  out[str(b)] = fall[1]; out[f"A{b}"] = fa[1]; out[f"B{b}"] = fb[1]
  # per-route spread
  per = []
  for r in sorted({R['route'] for R in rb}):
    rr = [R for R in rb if R['route'] == r]
    if sum(len(R['ang']) - R['h0'] for R in rr) * DT < 60:
      continue
    fr = fit(pack(rr))
    per.append((r, *fr[1], fr[0], rms(fall[1], pack(rr))))
  if per:
    Q = np.array([x[1:] for x in per])
    print(f"      per-route fits (n {len(per)}): d p10/p50/p90 {np.percentile(Q[:, 0], [10, 50, 90]).round(2).tolist()}, Kf {np.percentile(Q[:, 1], [10, 50, 90]).round(2).tolist()}, "
          f"Kp {np.percentile(Q[:, 2], [10, 50, 90]).round(2).tolist()}, Kc {np.percentile(Q[:, 3], [10, 50, 90]).round(2).tolist()} | own-fit RMS p50 {np.median(Q[:, 4]):.2f} vs all-data params {np.median(Q[:, 5]):.2f}", flush=True)
    out[f"per{b}"] = per
json.dump(out, open(f"{S}/qa/e59_params.json", "w"), indent=1, default=str)
