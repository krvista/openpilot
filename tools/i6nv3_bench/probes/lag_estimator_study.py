"""Estimator study: per speed band, lag of (a) controlsd desired curvature -> wheel curvature (actuator L_act), (b) model action ->
wheel (residual after all compensation; the model assumes 0.375 s), using normalized cross-correlation on W-second windows
(lat active, no blinker/LC, hands off: pressed 0, |tq| mean < 50, gain mean >= GMIN), excitation: curvature range >= RNG.
Prints n windows, median / IQR, per-window std, and windows per route."""
import glob, sys, numpy as np
S = sys.argv[1]; W = float(sys.argv[2]); GMIN = float(sys.argv[3]); RNG = float(sys.argv[4])
B = [(20, 40), (40, 60), (60, 80), (80, 200)]
def ncc_lag(a, b, maxlag=100):
  a = a - a.mean(); b = b - b.mean(); na = np.linalg.norm(a); nb = np.linalg.norm(b)
  if na < 1e-9 or nb < 1e-9: return None, 0
  cs = np.array([np.dot(a[:len(a) - L], b[L:]) / (np.linalg.norm(a[:len(a) - L]) * np.linalg.norm(b[L:]) + 1e-12) for L in range(maxlag + 1)])
  L = int(np.argmax(cs)); return L, cs[L]
res = {b: dict(act=[], resid=[], routes={}) for b in B}
n = int(W * 100)
for f in sorted(glob.glob(f"{S}/0000*.npz")):
  d = np.load(f, allow_pickle=True); r = f[-6:-4]; v = d['v'] / 3.6
  kw = -np.radians(d['ang']); kc = d['k_ctrl']; km = d['k_model']
  base = d['lat'] & ~(d['lb'] | d['rb']) & (d['lcs'] == 0)
  i = 0
  while i + n <= len(v):
    s = slice(i, i + n)
    if base[s].mean() > 0.98 and d['pressed'][s].mean() == 0 and np.abs(d['tq_s'][s]).mean() < 50 and d['gain'][s].mean() >= GMIN and (kc[s].max() - kc[s].min()) >= RNG:
      vv = v[s].mean() * 3.6
      for b in B:
        if b[0] <= vv < b[1]:
          La, ca = ncc_lag(kc[s], kw[s]); Lr, cr = ncc_lag(km[s], kw[s])
          if La is not None and ca > 0.9 and cr > 0.9 and La < 100 and Lr < 100:
            res[b]['act'].append(La * 10); res[b]['resid'].append(Lr * 10); res[b]['routes'][r] = res[b]['routes'].get(r, 0) + 1
    i += n // 2
print(f"W={W:.0f}s gain>={GMIN} range>={RNG}")
for b in B:
  a = np.array(res[b]['act']); x = np.array(res[b]['resid'])
  if len(a) < 3: print(f"  {b[0]:>3}-{b[1]:<3}: n {len(a)}"); continue
  print(f"  {b[0]:>3}-{b[1]:<3}: n {len(a):3d} | actuator cmd->wheel p25/p50/p75 {np.percentile(a,25):4.0f}/{np.median(a):4.0f}/{np.percentile(a,75):4.0f} ms (std {a.std():4.0f}) | "
        f"residual action->wheel {np.percentile(x,25):4.0f}/{np.median(x):4.0f}/{np.percentile(x,75):4.0f} ms (std {x.std():4.0f}) | per route {res[b]['routes']}")
