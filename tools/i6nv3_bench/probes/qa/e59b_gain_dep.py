"""E59b: does the MDPS response scale with the ACI gain? Runs (>= 2 s, 50 Hz) with the sent gain inside a band for the whole
run, op active, LKAS_ALT active, hold-compensated driver torque < 40, not pressed, no blinker, 40-90 km/h. M1 fit per gain
band. If Kf / Kp scale ~ g the compensator should divide by the gain."""
import glob
import sys
import numpy as np
from numba import njit
from scipy.optimize import minimize
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; DT = 0.02
GB = [(0.95, 1.01), (0.85, 0.95), (0.7, 0.85), (0.5, 0.7), (0.25, 0.5)]
runs = {g: [] for g in GB}
for f in sorted(glob.glob(f"{S}/r50/0*.npz") + glob.glob(f"{S}/r_old/0*.npz")):
  d = dict(np.load(f, allow_pickle=True))
  T = d['T']; n = len(T); v = d['v']
  if n < 3000 or 'gain' not in d:
    continue
  vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, b) for a, b in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  base = d['lat'] & (d['tx_active'] == 2) & (dd < 40) & ~d['pressed'] & (v >= 40) & (v < 90) & ~(d['lb'] | d['rb']) & np.isfinite(d['wire'])
  for glo, ghi in GB:
    ok = base & (d['gain'] >= glo) & (d['gain'] < ghi)
    e = np.flatnonzero(np.diff(np.concatenate([[0], ok.astype(int), [0]])))
    for a, b in zip(e[::2], e[1::2]):
      if T[b - 1] - T[a] < 2.0 or a < 50:
        continue
      a0 = a - 50
      tg = np.arange(T[a0], T[b - 1], DT)
      runs[(glo, ghi)].append((np.interp(tg, T[a0:b], d['wire'][a0:b]), np.interp(tg, T[a0:b], d['ang'][a0:b]), int(round((T[a] - T[a0]) / DT))))


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


def cost(p, P):
  nd = int(round(p[0] / DT)); se = 0.0; nn = 0
  for u, w0, h0 in P:
    w = sim_m1(u, w0, h0, nd, p[1], p[2], p[3]); se += np.sum((w[h0:] - w0[h0:]) ** 2); nn += len(w) - h0
  return np.sqrt(se / max(nn, 1))


for g, P in runs.items():
  if len(P) < 10:
    print(g, "runs", len(P)); continue
  best = None
  for d0 in np.arange(0.0, 0.42, 0.04):
    r = minimize(lambda q: cost([d0, *np.clip(q, [0.0, 0.0, -0.3], [1.5, 5.0, 2.0])], P), [0.5, 0.15, 0.1], method="Nelder-Mead", options=dict(maxiter=400))
    if best is None or r.fun < best[0]:
      best = (r.fun, d0, np.clip(r.x, [0.0, 0.0, -0.3], [1.5, 5.0, 2.0]))
  mins = sum(len(u) - h0 for u, _, h0 in P) * DT / 60
  print(f"gain {g[0]:.2f}-{g[1]:.2f}: runs {len(P)} ({mins:.0f} min) | d {best[1]:.2f} Kf {best[2][0]:.3f} Kp {best[2][1]:.3f} Kc {best[2][2]:.3f} | RMS {best[0]:.2f}", flush=True)
