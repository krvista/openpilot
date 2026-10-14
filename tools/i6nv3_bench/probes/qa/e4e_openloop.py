"""E4c: Phase 55 (replace-form: K + clip(k_or(0.9+tau) - K)) vs a delta-form lead that keeps the model action's feedback
and adds only the plan's predicted curvature CHANGE: K + clip(k_or(0.9+tau) - k_or(t_a)), t_a = plan time that best
matches the action. Scores: (a) RMSE vs road at t+0.45 per regime, (b) centring/damping content of (cmd - k_road):
regression slopes on lateral offset and heading error, in curves (|k_road| 1-6e-3) and straights, relative to the action's."""
import sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from common import routes, T_IDXS

S = sys.argv[1]
TA = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
store = []
for r, P, d, T, f in routes(S):
  n = len(T); v = f['v']
  K = P['K'].astype(float)
  OR, VX = P['OR'].astype(float), P['VX'].astype(float)
  kor_t = lambda t: np.array([np.interp(t[i], T_IDXS, OR[i]) / max(np.interp(t[i], T_IDXS, VX[i]), 1.0) for i in range(n)])
  tau = np.interp(v, [8.0, 13.0, 18.0], [0.30, 0.12, 0.08])
  k9 = kor_t(0.9 + tau)
  ka = {ta: kor_t(np.full(n, ta)) for ta in TA}
  LX = P['LX'].astype(float); LC = 0.5 * (P['L1'].astype(float) + P['L2'].astype(float))
  head = np.array([(np.interp(5.0, LX[i], LC[i]) - np.interp(0.0, LX[i], LC[i])) / 5.0 for i in range(n)])
  truth = f['k_road'].copy(); truth[~f['good']] = np.nan
  j = np.minimum(np.searchsorted(T, T + 0.45), n - 1)
  store.append(dict(v=v, K=K, k9=k9, ka=ka, tau=tau, head=head, off=-f['c0'], kr=f['k_road'], tr=truth[j],
                    ok=f['good'] & f['lat'] & (np.abs(T[j] - (T + 0.45)) < 0.03) & ~np.isnan(truth[j]),
                    cons=np.concatenate([np.diff(T) < 0.07, [False]])))
  print(r, flush=True)
cat = lambda k: np.concatenate([s[k] for s in store])
v, K, k9, tau, head, off, kr, tr, ok = (cat(k) for k in ("v", "K", "k9", "tau", "head", "off", "kr", "tr", "ok"))
ka = {ta: np.concatenate([s['ka'][ta] for s in store]) for ta in TA}
sel = ok & (np.abs(K) > 0.5e-3)
print("action vs plan curvature at t_a (RMSE 1e-3/m):", " ".join(f"{ta}: {np.sqrt(np.mean((K - ka[ta])[sel] ** 2)) * 1e3:.3f}" for ta in TA))
ab = np.abs(K)
blend = np.clip((ab - 0.8e-3) / 0.7e-3, 0, 1) * (ab >= 0.8e-3)
dk = 0.7 * (0.9 + tau) / np.maximum(v, 5.0) ** 2
k3 = ka[0.3]
dlt = np.clip(k9 - k3, -dk, dk)
grow = (np.sign(dlt) == np.sign(k9)) & (np.abs(k9) > np.abs(k3))
b9 = np.clip((np.abs(k9) - 0.8e-3) / 0.7e-3, 0, 1)
t_ah = np.minimum(np.interp(v, [5.6, 13.9, 27.8, 38.9], [0.08, 0.10, 0.13, 0.18]) + np.interp(ab, [0.0008, 0.005], [0.0, 0.20]), 0.32) + tau
cmds = {"action": K, "55b": K + 0.5 * dlt, "55c": K + 0.5 * dlt * grow * b9, "55c-nogate": K + 0.5 * dlt * grow,
        "55c-0.7": K + 0.7 * dlt * grow * b9}
reg = np.where((np.abs(tr) < 0.5e-3) & (np.abs(kr) < 0.5e-3), "straight", np.where(np.abs(tr) > np.abs(kr) + 0.7e-3, "entry",
               np.where(np.abs(kr) > np.abs(tr) + 0.7e-3, "exit", np.where(np.abs(tr) >= 1.5e-3, "curve", "mild"))))
rm = lambda e: np.sqrt(np.mean(e ** 2)) * 1e3
print(f"{'cmd':16} " + " ".join(f"{g:>8}" for g in ("straight", "mild", "entry", "curve", "exit")) +
      "  | centring slopes rel. to action: curve offset / heading, straight offset / heading")
for nm, c in cmds.items():
  out = []
  for zone in ((np.abs(kr) >= 1e-3) & (np.abs(kr) < 6e-3), np.abs(kr) < 0.5e-3):
    s = ok & zone & (v >= 30 / 3.6)
    X = np.stack([off[s], head[s] * np.sign(1), np.ones(s.sum())], 1)
    co = np.linalg.lstsq(X, (c - kr)[s], rcond=None)[0]
    ca = np.linalg.lstsq(X, (K - kr)[s], rcond=None)[0]
    out.append(f"{co[0] / ca[0]:.2f}/{co[1] / ca[1]:.2f}")
  print(f"{nm:16} " + " ".join(f"{rm((c - tr)[ok & (reg == g)]):8.3f}" for g in ("straight", "mild", "entry", "curve", "exit")) + "  | " + "  ".join(out))
