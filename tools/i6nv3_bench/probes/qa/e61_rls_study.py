"""E61: would LIVE adaptation help? Chronological replay over all hands-off runs (E59 cache, 50 Hz). Per speed band an RLS
estimator of (Kf, Kp, Kc) at the band's fixed delay, regression on 0.2 s differences:
   (w(t+0.2) - w(t)) / 0.2 = Kf * (u(t+0.2-d) - u(t-d)) / 0.2 + Kp * (u(t-d) - w(t)) - Kc * w(t)
with forgetting (time constant TAU of hands-off data) and bounds (+-BOUND x the offline value). Each run is first
PREDICTED (output-error RMS) with the current estimate, then used to update. Compared with fixed parameters from the
other route fold (fair) on the same runs. Reports by band, plus the estimate trajectory spread."""
import json
import pickle
import sys
import numpy as np
from numba import njit
S = sys.argv[1]; TAU = float(sys.argv[2]) if len(sys.argv) > 2 else 600.0; BOUND = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
DT = 0.02
runs = pickle.load(open(f"{S}/qa/e59_runs.pkl", "rb"))
PAR = json.load(open(f"{S}/qa/e59_params.json"))
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


def oe(p, R):
  u, w0, h0 = R['wire'], R['ang'], R['h0']
  w = sim_m1(u, w0, h0, int(round(p[0] / DT)), p[1], p[2], p[3])
  return np.sum((w[h0:] - w0[h0:]) ** 2), len(w) - h0


for b in range(1, 6):
  rb = sorted([R for R in runs if R['band'] == b], key=lambda R: R['route'])
  if len(rb) < 20:
    continue
  p_all = PAR[str(b)]; d = p_all[0]; nd = int(round(d / DT)); L = 10
  theta = np.array(p_all[1:4], float); P = np.eye(3) * 0.05
  lo = np.array(p_all[1:4]) - BOUND * np.maximum(np.abs(p_all[1:4]), 0.05); hi = np.array(p_all[1:4]) + BOUND * np.maximum(np.abs(p_all[1:4]), 0.05)
  lam = np.exp(-L * DT / TAU)
  se_a = se_f = se_o = 0.0; nn = 0; traj = []
  for R in rb:
    pf = PAR[f"{'B' if fold[R['route']] == 0 else 'A'}{b}"]          # fixed, from the other fold
    s1, n1 = oe([d, *theta], R); s2, _ = oe(pf, R); s3, _ = oe(p_all, R)
    se_a += s1; se_f += s2; se_o += s3; nn += n1
    traj.append(theta.copy())
    u, w = R['wire'], R['ang']
    for k in range(max(R['h0'], nd + L), len(w) - L, 2):
      y = (w[k + L] - w[k]) / (L * DT)
      x = np.array([(u[k + L - nd] - u[k - nd]) / (L * DT), u[k - nd] - w[k], -w[k]])
      Px = P @ x; g = Px / (lam + x @ Px)
      theta = theta + g * (y - x @ theta)
      P = (P - np.outer(g, Px)) / lam
      theta = np.clip(theta, lo, hi)
  T = np.array(traj)
  print(f"band {b} ({len(rb)} runs, {nn * DT / 60:.0f} min): predictive OE RMS adaptive {np.sqrt(se_a / nn):.2f} | fixed (other fold) {np.sqrt(se_f / nn):.2f} "
        f"| fixed (all data, optimistic) {np.sqrt(se_o / nn):.2f} deg | adaptive Kf p10/p90 {np.percentile(T[:, 0], 10):.2f}/{np.percentile(T[:, 0], 90):.2f}, "
        f"Kp {np.percentile(T[:, 1], 10):.2f}/{np.percentile(T[:, 1], 90):.2f}, Kc {np.percentile(T[:, 2], 10):.2f}/{np.percentile(T[:, 2], 90):.2f}", flush=True)
