"""Lookahead study (vectorized). Per OPM10V3 model frame, from route npz (v, lat, blinker, LC) + plan npz:
1) limits of the current controlsd lookahead (0.32 s cap, 10 m clamp, dk_max jerk budget) — how often they bind;
2) predictor accuracy vs ROAD curvature at the car at t+tau (lane-centre 2nd-order fit 0..20 m in the frame nearest t+tau),
   for cubic (current geometry), orient (plan yaw rate / speed at tau), action, lane (lane-centre y'' at d = v tau).
Usage: la_fast.py <scratch dir with la/plan_*.npz, ropm/*.npz, r43/*.npz>"""
import glob
import sys

import numpy as np

S = sys.argv[1]
T_IDXS = np.array([10.0 * (i / 32) ** 2 for i in range(33)])
TAUS = np.array([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8])


def bfit(X, Y, deg):
  """Batched least-squares polyfit: X, Y (n, m) -> coeffs (n, deg+1), highest power first (np.polyfit order)."""
  V = np.stack([X ** p for p in range(deg, -1, -1)], axis=-1)
  A = np.einsum('nmi,nmj->nij', V, V)
  b = np.einsum('nmi,nm->ni', V, Y)
  return np.linalg.solve(A + 1e-12 * np.eye(deg + 1), b[..., None])[..., 0]


def interp_rows(xq, X, Y):
  out = np.empty(xq.shape)
  for i in range(X.shape[0]):
    out[i] = np.interp(xq[i], X[i], Y[i])
  return out


def interp_tau(tau, Y):
  """Row-wise value at time tau on T_IDXS (same grid for all rows)."""
  j = np.searchsorted(T_IDXS, tau)
  j = min(max(j, 1), len(T_IDXS) - 1)
  w = (tau - T_IDXS[j - 1]) / (T_IDXS[j] - T_IDXS[j - 1])
  return Y[:, j - 1] * (1 - w) + Y[:, j] * w


ALL, LIM = [], []
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
  PX = P['PX'].astype(float)
  PY = P['PY'].astype(float)
  LX = P['LX'].astype(float)
  LC = 0.5 * (P['L1'].astype(float) + P['L2'].astype(float))
  OR = P['OR'].astype(float)
  VX = P['VX'].astype(float)
  C3 = bfit(PX[:, :12], PY[:, :12], 3)
  xs0 = np.tile(np.linspace(0, 20, 9), (n, 1))
  truth = 2.0 * bfit(xs0, interp_rows(xs0, LX, LC), 2)[:, 0]
  truth[~good] = np.nan

  # 1) limits of the current lookahead
  ab = np.abs(K)
  tau_lp = np.interp(v, [8.0, 13.0, 18.0], [0.30, 0.12, 0.08])
  base = np.interp(v, [5.6, 13.9, 27.8, 38.9], [0.08, 0.10, 0.13, 0.18])
  boost = np.interp(ab, [0.0008, 0.005], [0.0, 0.20])
  t_ahead = np.minimum(base + boost, 0.32) + tau_lp
  dist = np.minimum(v * t_ahead, 10.0)
  curv = 6.0 * C3[:, 0] * dist + 2.0 * C3[:, 1]
  dk_max = 0.7 * np.maximum(t_ahead, 0.05) / np.maximum(v, 5.0) ** 2
  blend = np.interp(ab, [0.0008, 0.0015], [0.0, 1.0])
  raw = curv - K
  gate = ab >= 0.0008
  cmd = K + blend * np.clip(raw, -dk_max, dk_max) * gate
  sel = lat & ~busy & (v * 3.6 >= 30)
  LIM.append(np.stack([v * 3.6, ab, (base + boost) > 0.32, v * t_ahead > 10.0, (np.abs(raw) > dk_max) & (blend > 0) & gate,
                       t_ahead, dist / np.maximum(v, 1), (cmd - K) * np.sign(K), raw * np.sign(K) * gate], 1)[sel])

  # 2) predictors vs truth at t + tau
  for tau in TAUS:
    j = np.minimum(np.searchsorted(T, T + tau), n - 1)
    ok = good & (np.abs(T[j] - (T + tau)) < 0.03) & ~np.isnan(truth[j])
    dd = v * tau
    k_cubic = 6.0 * C3[:, 0] * dd + 2.0 * C3[:, 1]
    k_orient = interp_tau(tau, OR) / np.maximum(interp_tau(tau, VX), 1.0)
    xl = np.maximum(dd[:, None] + np.linspace(-10, 10, 9)[None, :], 0.0)
    k_lane = 2.0 * bfit(xl, interp_rows(xl, LX, LC), 2)[:, 0]
    tr = truth[j]
    tr0 = truth
    reg = np.where((np.abs(tr) < 0.5e-3) & (np.abs(tr0) < 0.5e-3), 0,
          np.where(np.abs(tr) > np.abs(tr0) + 0.7e-3, 1,
          np.where(np.abs(tr0) > np.abs(tr) + 0.7e-3, 3,
          np.where(np.abs(tr) >= 1.5e-3, 2, 4))))
    ALL.append(np.stack([np.full(n, tau), v * 3.6, reg, tr, k_cubic, k_orient, K, k_lane], 1)[ok])
  print(r, n, flush=True)

A = np.concatenate(ALL)
L = np.concatenate(LIM)
np.save(f"{S}/la/A.npy", A)
np.save(f"{S}/la/L.npy", L)
names = ["cubic", "orient", "action", "lane"]
q = (A[:, 0] == 0.0) & (np.abs(A[:, 3]) > 1e-3)
for k, nm in enumerate(names):
  print(f"  sign/scale tau0 {nm:7}: corr {np.corrcoef(A[q, 4 + k], A[q, 3])[0, 1]:+.3f} slope {np.polyfit(A[q, 3], A[q, 4 + k], 1)[0]:+.2f}")


def rm(e):
  return np.sqrt(np.mean(e ** 2)) * 1e3


for reg, nm in ((None, "all"), (0, "straight"), (1, "entry"), (2, "curve"), (3, "exit")):
  print(f"== {nm}: RMSE (1e-3/m) predictor(t) vs road curvature at t+tau")
  print("   tau  " + "".join(f"{x:>9}" for x in names) + "      n")
  for tau in TAUS:
    m = (A[:, 0] == tau) & ((A[:, 2] == reg) if reg is not None else True)
    if m.sum() < 30:
      continue
    print(f"  {tau:4.1f}  " + "".join(f"{rm(A[m, 4 + k] - A[m, 3]):9.3f}" for k in range(4)) + f"  {int(m.sum()):6d}")

B = [(30, 50), (50, 70), (70, 90), (90, 200)]
print("== limits of the current lookahead (lat active, no blinker/LC, >= 30 km/h)")
print(f"{'band':8} {'frames':>7} {'|k|>=0.8e-3':>11} {'cap0.32':>8} {'10m clamp':>9} {'budget':>7} {'t_ahead p50':>11} {'eff p50':>8} {'lead p50/p90':>14} {'raw p90':>8}")
for b in B:
  m = (L[:, 0] >= b[0]) & (L[:, 0] < b[1])
  g = m & (L[:, 1] >= 0.0008)
  if g.sum() == 0:
    continue
  print(f"{b[0]:>3}-{b[1]:<4} {int(m.sum()):7d} {g.sum() / max(m.sum(), 1):11.0%} {L[g, 2].mean():8.0%} {L[g, 3].mean():9.0%} {L[g, 4].mean():7.0%} "
        f"{np.median(L[g, 5]):11.2f} {np.median(L[g, 6]):8.2f} {np.median(L[g, 7]) * 1e3:+6.3f}/{np.percentile(L[g, 7], 90) * 1e3:+.3f} {np.percentile(L[g, 8], 90) * 1e3:+8.3f}")
