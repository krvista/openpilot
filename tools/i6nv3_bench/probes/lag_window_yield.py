"""Learner data yield per route: 10 s windows, hands-off (pressed 0, |tq| mean < 50, gain mean >= 0.9), lat active, no blinker/LC,
excitation (controlsd curvature range >= 2e-3), speed within one band >= 95 % of the window, NCC(action -> wheel) > 0.9."""
import glob, sys, numpy as np
S = sys.argv[1]; B = [(20, 40), (40, 60), (60, 80), (80, 200)]
def ncc_lag(a, b, maxlag=120):
  a = a - a.mean(); b = b - b.mean()
  if np.linalg.norm(a) < 1e-9 or np.linalg.norm(b) < 1e-9: return None, 0
  cs = np.array([np.dot(a[:len(a) - L], b[L:]) / (np.linalg.norm(a[:len(a) - L]) * np.linalg.norm(b[L:]) + 1e-12) for L in range(maxlag + 1)])
  L = int(np.argmax(cs)); return L, cs[L]
print(f"{'route':6} {'min':>5} {'lat%':>5} " + " ".join(f"{b[0]}-{b[1]:<3}" for b in B) + "   (windows: candidates / accepted)")
tot = {b: [0, 0] for b in B}
for f in sorted(glob.glob(f"{S}/0000*.npz")):
  d = np.load(f, allow_pickle=True); v = d['v'] / 3.6 * 3.6
  kw = -np.radians(d['ang']); kc = d['k_ctrl']; km = d['k_model']
  base = d['lat'] & ~(d['lb'] | d['rb']) & (d['lcs'] == 0)
  cand = {b: 0 for b in B}; acc = {b: 0 for b in B}; i = 0; n = 1000
  while i + n <= len(v):
    s = slice(i, i + n)
    if base[s].mean() > 0.98 and d['pressed'][s].mean() == 0 and np.abs(d['tq_s'][s]).mean() < 50 and d['gain'][s].mean() >= 0.9 and (kc[s].max() - kc[s].min()) >= 2e-3:
      for b in B:
        inb = ((v[s] >= b[0]) & (v[s] < b[1])).mean()
        if inb >= 0.95:
          cand[b] += 1; L, c = ncc_lag(km[s], kw[s])
          if L is not None and c > 0.9 and 0 < L < 120: acc[b] += 1
    i += 500
  for b in B: tot[b][0] += cand[b]; tot[b][1] += acc[b]
  print(f"{f[-6:-4]:6} {len(v)/6000:5.1f} {d['lat'].mean()*100:5.0f} " + " ".join(f"{cand[b]:3d}/{acc[b]:<3d}" for b in B))
print("total        " + " ".join(f"{tot[b][0]:3d}/{tot[b][1]:<3d}" for b in B))
