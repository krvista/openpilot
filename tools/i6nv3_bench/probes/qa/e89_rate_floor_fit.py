"""E89: MDPS plant with a REQUEST-RATE FLOOR (Phase 40 bench: the CCNC MDPS follows the request rate, not the error; the
wheel does not move while the request moves slower than ~1 deg/s, barely at 1-3 deg/s). Plant RF: an internal target y
integrates only the part of the request rate (0.1 s difference, delayed d) above r0:
    dy/dt = sign(du/dt) * max(0, |du/dt| - r0)
    dw/dt = Kf * dy/dt + Kp * (u - w) - Kc * w
r0 = 0 reduces to E59's M1. Output-error fit on hands-off runs (op, gain >= 0.95, no blinker), 2-fold by route, per band;
then each plant replays the recorded TX of every hands-off corner entry (e87 rule, all builds in r50) and the wheel onset
lag (wheel +1 deg after TX +1 deg) is compared with the measured one."""
import glob, json, os, pickle, sys
import numpy as np
from numba import njit
from scipy.optimize import minimize
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; DT = 0.02
BANDS = [(15, 30), (30, 40), (40, 55), (55, 70), (70, 90), (90, 200)]
cache = f"{S}/qa/e89_runs.pkl"
files = sorted(glob.glob(f"{S}/r50/0*.npz") + glob.glob(f"{S}/r_old/0*.npz"), key=lambda f: f[-12:-4])
if os.path.exists(cache):
  runs, entries = pickle.load(open(cache, "rb"))
else:
  runs = []; entries = []
  for f in files:
    d = dict(np.load(f, allow_pickle=True))
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
    # hands-off corner entries (e87 rule), stored as 50 Hz TX / wheel from K50 - 2 s to K50 + 2 s
    K = d['k_model']; aK = np.abs(K); kc = d['k_ctrl']
    m = d['lat'] & (np.abs(kc) > 0.5e-3)
    if m.sum() < 100:
      continue
    sg = np.sign(np.sum(d['apply'][m] * kc[m]))
    bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
    cs = np.concatenate([[0], np.cumsum(bl)])
    busy = (cs[np.minimum(idx + 300, n)] - cs[np.maximum(idx - 300, 0)]) > 0
    okE = d['lat'] & ~busy & (v >= 30); handw = (dd >= 40) | d['pressed']
    i = 300
    while i < n - 500:
      if aK[i] < 0.7e-3 and (aK[i - 100:i] < 0.7e-3).all():
        pk = i + int(np.argmax(aK[i:i + 300]))
        if aK[pk] >= 2e-3 and okE[i - 100:pk + 200].all() and handw[i:pk + 200].mean() < 0.1:
          s = np.sign(K[pk]); k50 = next((j for j in range(i, pk + 1) if K[j] * s >= 0.5 * aK[pk]), None)
          if k50 is not None and k50 - 200 >= 0 and k50 + 200 < n:
            sl = slice(k50 - 200, k50 + 200, 2)
            entries.append(dict(u=d['wire'][sl] * s * sg, w=d['ang'][sl] * s * sg, v=float(np.median(v[i:pk])), commit=str(d['commit'])[:7], route=f[-12:-4]))
        i = pk + 300; continue
      i += 10
  pickle.dump((runs, entries), open(cache, "wb"))
routes = sorted({R['route'] for R in runs}); fold = {r: i % 2 for i, r in enumerate(routes)}
print(f"runs {len(runs)} ({sum(len(R['ang']) - R['h0'] for R in runs) * DT / 60:.0f} min), hands-off entries {len(entries)}", flush=True)


@njit(cache=True)
def sim_rf(u, w0, h0, nd, kf, kp, kc, r0):
  n = len(u); w = np.empty(n)
  for k in range(h0 + 1):
    w[k] = w0[k]
  for k in range(h0 + 1, n):
    i1 = k - 1 - nd; i0 = i1 - 5
    u1 = u[i1] if i1 >= 0 else u[0]
    ua = u[i0] if i0 >= 0 else u[0]
    r = (u1 - ua) / (5 * DT)
    dy = 0.0
    if r > r0:
      dy = r - r0
    elif r < -r0:
      dy = r + r0
    w[k] = w[k - 1] + DT * (kf * dy + kp * (u1 - w[k - 1]) - kc * w[k - 1])
  return w


def sse(p, P):
  nd = int(round(p[0] / DT)); se = 0.0; nn = 0
  for u, w0, h0 in P:
    w = sim_rf(u, w0, h0, nd, p[1], p[2], p[3], p[4]); se += np.sum((w[h0:] - w0[h0:]) ** 2); nn += len(w) - h0
  return se, nn


LB = np.array([0.0, -0.3, 0.0, 0.0]); UB = np.array([5.0, 2.0, 3.0, 15.0])


def fit(P, with_r):
  best = None
  for d0 in (0.1, 0.2, 0.3, 0.4):
    for r00 in ((0.0, 1.0, 3.0) if with_r else (0.0,)):
      def f(q):
        q = np.clip(q, LB, UB)
        if not with_r:
          q[3] = 0.0
        return sse([d0, *q], P)[0]
      r = minimize(f, [0.6, 0.15, 0.1, r00], method="Nelder-Mead", options=dict(maxiter=400, xatol=1e-3, fatol=1e-3))
      q = np.clip(r.x, LB, UB)
      if not with_r:
        q[3] = 0.0
      if best is None or r.fun < best[0]:
        best = (r.fun, [d0, *q.tolist()])
  return best[1]


def onset(x, lev, start):
  j = next((k for k in range(start, len(x)) if x[k] - x[start] >= lev), None)
  return j


pk = lambda rs: [(R['wire'], R['ang'], R['h0']) for R in rs]
out = {}
for b, (lo, hi) in enumerate(BANDS):
  if b == 0:
    continue
  rb = [R for R in runs if R['band'] == b]
  if len(rb) < 10:
    continue
  A = pk([R for R in rb if fold[R['route']] == 0]); Bf = pk([R for R in rb if fold[R['route']] == 1])
  res = {}
  for nm, wr in (("M1", False), ("RF", True)):
    pa, pb = fit(A, wr), fit(Bf, wr)
    pall = fit(pk(rb), wr)
    res[nm] = (np.sqrt((sse(pa, Bf)[0] + sse(pb, A)[0]) / (sse(pa, Bf)[1] + sse(pb, A)[1])), pall)
    out[f"{nm}{b}"] = pall; out[f"{nm}A{b}"] = pa; out[f"{nm}B{b}"] = pb
  # entry onset replay in this band
  E = [e for e in entries if lo <= e['v'] < hi]
  lag = {"real": [], "M1": [], "RF": []}
  for e in E:
    u, w = e['u'], e['w']; s0 = 25   # K50 - 1.5 s
    t1 = onset(u, 1.0, s0)
    if t1 is None:
      continue
    wr_ = onset(w, 1.0, s0)
    lag["real"].append((wr_ - t1) * DT if wr_ is not None else np.nan)
    for nm in ("M1", "RF"):
      p = res[nm][1]; ws = sim_rf(u, w, s0, int(round(p[0] / DT)), p[1], p[2], p[3], p[4])
      j = onset(ws, 1.0, s0); lag[nm].append((j - t1) * DT if j is not None else np.nan)
  med = {k: (np.nanmedian(x) if len(x) else np.nan) for k, x in lag.items()}
  print(f"[{lo}-{hi}] runs {len(rb)} | cross-fold RMS M1 {res['M1'][0]:.3f} vs RF {res['RF'][0]:.3f} deg | RF all-data d {res['RF'][1][0]:.2f} Kf {res['RF'][1][1]:.3f} "
        f"Kp {res['RF'][1][2]:.3f} Kc {res['RF'][1][3]:.3f} r0 {res['RF'][1][4]:.2f} deg/s | M1 d {res['M1'][1][0]:.2f} Kf {res['M1'][1][1]:.3f} | entries {len(lag['real'])}: "
        f"wheel onset after TX real {med['real']:.2f} s, M1 {med['M1']:.2f}, RF {med['RF']:.2f}", flush=True)
json.dump(out, open(f"{S}/qa/e89_params.json", "w"), indent=1)
