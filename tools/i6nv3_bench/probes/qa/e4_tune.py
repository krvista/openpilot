"""E4: Phase 55 parameter sweep on the 9 plan routes (vectorized replica of _lookahead_curvature + 7c entry assist + LP),
scored vs road curvature at t + L (L = 0.45 s, plus the LP's own lag where the LP is simulated).
Baseline = shipped Phase 55. Variants change one thing each."""
import sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from common import routes, T_IDXS

S = sys.argv[1]
L_TGT = 0.45
V = {
  "P55": {},
  "gate0.5": dict(gate=0.5e-3, b0=0.5e-3, b1=1.0e-3),
  "gate0.3": dict(gate=0.3e-3, b0=0.3e-3, b1=0.8e-3),
  "gate0": dict(gate=0.0, b0=0.0, b1=0.0),
  "ts0.7": dict(ts=0.7), "ts1.1": dict(ts=1.1),
  "tsV": dict(ts_v=([8.3, 16.7, 25.0], [1.1, 0.9, 0.8])),
  "J0.5": dict(J=0.5), "J1.0": dict(J=1.0),
  "no7c": dict(ea=False),
  "lp0.5": dict(lp=0.5),
}
OUT = {k: {} for k in V}
JIT = {k: [] for k in V}
for r, P, d, T, f in routes(S):
  n = len(T)
  v = f['v']
  K = P['K'].astype(float)
  OR, VX = P['OR'].astype(float), P['VX'].astype(float)
  LX = P['LX'].astype(float); LC = 0.5 * (P['L1'].astype(float) + P['L2'].astype(float))
  lc = lambda xq: np.array([np.interp(xq, LX[i], LC[i]) for i in range(n)])
  klane = (lc(0.0) - 2.0 * lc(25.0) + lc(50.0)) / 625.0
  lane_min = np.minimum(P['PR'][:, 1], P['PR'][:, 2])
  tau = np.interp(v, [8.0, 13.0, 18.0], [0.30, 0.12, 0.08])
  ab = np.abs(K)
  truth = f['k_road'].copy(); truth[~f['good']] = np.nan
  j = np.minimum(np.searchsorted(T, T + L_TGT), n - 1)
  okj = (np.abs(T[j] - (T + L_TGT)) < 0.03)
  for vn, p in V.items():
    gate, b0, b1 = p.get('gate', 0.8e-3), p.get('b0', 0.8e-3), p.get('b1', 1.5e-3)
    ts = np.interp(v, *p['ts_v']) if 'ts_v' in p else np.full(n, p.get('ts', 0.9))
    J = p.get('J', 0.7)
    tq = ts + tau
    k_or = np.array([np.interp(tq[i], T_IDXS, OR[i]) / max(np.interp(tq[i], T_IDXS, VX[i]), 1.0) for i in range(n)])
    blend = np.clip((ab - b0) / max(b1 - b0, 1e-9), 0, 1) if b1 > b0 else np.ones(n)
    blend = blend * (ab >= gate)
    dk = J * tq / np.maximum(v, 5.0) ** 2
    cmd = K + blend * np.clip(k_or - K, -dk, dk)
    # 7c entry assist (EMA 0.3 s on klane, rising detector 0.5 s), at 20 Hz
    if p.get('ea', True):
      kl = 0.0; slow = 0.0
      a_k = 0.05 / 0.35; a_s = 0.05 / 0.5
      out = cmd.copy()
      for i in range(n):
        if np.isfinite(klane[i]):
          kl += a_k * (klane[i] - kl)
        slow += a_s * (abs(cmd[i]) - slow)
        if (v[i] > 8.0 and lane_min[i] > 0.6 and not f['busy'][i] and abs(kl) > 0.003 and abs(cmd[i]) > slow * 1.02
            and np.sign(kl) == np.sign(cmd[i]) and abs(kl) > abs(cmd[i])):
          out[i] = cmd[i] + np.sign(cmd[i]) * min(abs(kl) - abs(cmd[i]), 1e-3, 0.5 * abs(cmd[i]))
      cmd = out
    # LP variant: scale tau, compare LP output vs truth at t + L - tau_scaled (the LP lag replaces part of the sample lead)
    sc = p.get('lp', 1.0)
    if sc != 1.0:
      # sample time keeps +tau_ship; LP lag shrinks -> effective lead grows by tau*(1-sc); emulate by scoring at L - tau*(1-sc)
      shift = tau * (1 - sc)
    else:
      shift = np.zeros(n)
    jj = np.minimum(np.searchsorted(T, T + L_TGT - shift), n - 1)
    tr = truth[jj]; tr0 = truth
    ok = f['good'] & f['lat'] & okj & ~np.isnan(tr)
    reg = np.where((np.abs(tr) < 0.5e-3) & (np.abs(tr0) < 0.5e-3), "straight", np.where(np.abs(tr) > np.abs(tr0) + 0.7e-3, "entry",
                   np.where(np.abs(tr0) > np.abs(tr) + 0.7e-3, "exit", np.where(np.abs(tr) >= 1.5e-3, "curve", "mild"))))
    e = (cmd - tr)[ok]
    for rg in ("straight", "mild", "entry", "curve", "exit"):
      OUT[vn].setdefault(rg, []).append(e[reg[ok] == rg])
    st = f['good'] & f['lat'] & (np.abs(np.nan_to_num(truth, nan=1.0)) < 0.5e-3)
    cons = st[1:] & st[:-1] & (np.diff(T) < 0.07)
    JIT[vn].append(np.diff(cmd)[cons])
  print(r, flush=True)

rm = lambda e: np.sqrt(np.mean(e ** 2)) * 1e3
print("RMSE 1e-3/m vs road at t+0.45 | straight-change std")
print(f"{'variant':9} " + " ".join(f"{rg:>8}" for rg in ("straight", "mild", "entry", "curve", "exit")) + "   jitter")
for vn in V:
  print(f"{vn:9} " + " ".join(f"{rm(np.concatenate(OUT[vn][rg])):8.3f}" for rg in ("straight", "mild", "entry", "curve", "exit"))
        + f"   {np.std(np.concatenate(JIT[vn])) * 1e3:.3f}")
print("n: " + " ".join(f"{rg} {len(np.concatenate(OUT['P55'][rg]))}" for rg in ("straight", "mild", "entry", "curve", "exit")))
