"""Phase 55 S-curve / reversal check (companion to la_cmd.py, same frames and command shaping): on frames where the
command's sign differs from the model action's (|action| >= 0.8e-3, lateral active, both lines > 0.7, >= 30 km/h, no
blinker) — i.e. the lead crosses zero ahead of the plan — how often the road curvature 0.45 s later agrees with the
command's sign (anticipated reversal) vs the action's (premature flip), and by how much the command overshoots the road.
Usage: la_reversal.py <scratch dir>"""
import glob
import sys

import numpy as np

sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
T_IDXS = np.array([10.0 * (i / 32) ** 2 for i in range(33)])


def bfit(X, Y, deg):
  V = np.stack([X ** p for p in range(deg, -1, -1)], axis=-1)
  A = np.einsum('nmi,nmj->nij', V, V)
  b = np.einsum('nmi,nm->ni', V, Y)
  return np.linalg.solve(A + 1e-12 * np.eye(deg + 1), b[..., None])[..., 0]


def interp_rows(xq, X, Y):
  out = np.empty(xq.shape)
  for i in range(X.shape[0]):
    out[i] = np.interp(xq[i], X[i], Y[i])
  return out

S = sys.argv[1]
L_TGT = 0.45
VARS = ("current", "cubic06", "orient09", "orientB", "action")
rev = {}
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
    k_or[i] = np.interp(tq, T_IDXS, OR[i]) / max(np.interp(tq, T_IDXS, VX[i]), 1.0)
  cmds["orient09"] = shaped(k_or, 0.9 + tau_lp)
  cmds["orientB"] = shaped(k_or, t_cur)
  cmds["action"] = K.copy()
  j = np.minimum(np.searchsorted(T, T + L_TGT), n - 1)
  ok = good & lat & (np.abs(T[j] - (T + L_TGT)) < 0.03) & ~np.isnan(truth[j]) & (np.abs(K) >= 0.0008)
  tr = truth[j]
  for vn, c in cmds.items():
    flip = ok & (np.sign(c) != np.sign(K)) & (np.abs(c) > 0.2e-3)
    agree = flip & (np.sign(tr) == np.sign(c))
    over = np.maximum(np.abs(c) - np.abs(tr), 0.0) * (np.sign(tr) == np.sign(c)) + np.abs(c) * (np.sign(tr) != np.sign(c))
    acc = rev.setdefault(vn, [0, 0, 0, []])
    acc[0] += int(ok.sum()); acc[1] += int(flip.sum()); acc[2] += int(agree.sum()); acc[3].append(over[flip])
  print(r, flush=True)

for vn, (n_ok, n_flip, n_agree, ov) in rev.items():
  ov = np.concatenate(ov) if ov else np.zeros(0)
  print(f"{vn:9} frames {n_ok:6d}  early flip {n_flip:5d} ({n_flip / max(n_ok, 1):.2%})  road agrees {n_agree / max(n_flip, 1):.0%}"
        f"  wrong-side excess p50/p95 {np.median(ov) * 1e3 if len(ov) else 0:.3f}/{np.percentile(ov, 95) * 1e3 if len(ov) else 0:.3f} e-3/m")
