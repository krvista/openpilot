"""E86: lane position in the REAL car frame. With sunnypilot modeld_v2 the CameraOffset param (p, m) shears the model
input, so model lane lines are in a virtual frame: y_real = y_model + p (y right-positive, as the onroad renderer draws
them). Routes 5d-73 ran p = 0.0, 76-79 p = -0.06, 7a-7f p = -0.08 (initData params). Straight offset (hands off, lines
>= 0.6, |K| < 0.5e-3; negative = car left of centre) and line approaches (car edge < 0.15 m held 0.3 s) per band, in
the model frame and corrected to the real frame."""
import glob, sys
import numpy as np
S = sys.argv[1]
P_OFF = {"98e912d": 0.0, "31520b9": -0.06, "bf36aa2": -0.08}
BANDS = [(15, 40), (40, 55), (55, 70), (70, 90), (90, 200)]
def starts(x):
  return np.flatnonzero(np.diff(np.concatenate([[0], x.astype(int)])) == 1)
A = {b: {i: dict(off=[], offr=[], n=0, nL=0, nr=0, nLr=0, mins=0.0) for i in range(len(BANDS))} for b in P_OFF}
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  r = f[-12:-4]
  d = dict(np.load(f, allow_pickle=True)); b = str(d['commit'])[:7]
  if b not in P_OFF:
    continue
  try:
    P = dict(np.load(f"{S}/la/plan_{r}.npz"))
  except Exception:
    continue
  p = P_OFF[b]; T = d['T']; v = d['v']; n = len(T)
  j = np.clip(np.searchsorted(P['T'], T), 0, len(P['T']) - 1)
  l1 = np.array([np.interp(0.0, x, y) for x, y in zip(P['LX'], P['L1'])])[j]
  l2 = np.array([np.interp(0.0, x, y) for x, y in zip(P['LX'], P['L2'])])[j]
  pr = np.minimum(P['PR'][:, 1], P['PR'][:, 2])[j]
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  cs = np.concatenate([[0], np.cumsum(bl)]); idx = np.arange(n)
  busy = (cs[np.minimum(idx + 300, n)] - cs[np.maximum(idx - 300, 0)]) > 0
  off = d['lat'] & ~busy & ~d['pressed'] & (np.abs(d['tq_s']) < 100)
  for bi, (lo, hi) in enumerate(BANDS):
    vb = (v >= lo) & (v < hi); m = off & vb
    if m.sum() < 300:
      continue
    R = A[b][bi]; R['mins'] += m.sum() / 6000
    st = m & (pr >= 0.6) & (np.abs(d['k_model']) < 0.5e-3)
    o = -(l1 + l2) / 2
    R['off'].append(o[st]); R['offr'].append(o[st] - p)
    for key, a1, a2 in (("n", l1, l2), ("nr", l1 + p, l2 + p)):
      edge = np.minimum(-a1, a2) - 0.95
      nr = np.convolve((m & (pr >= 0.6) & (edge < 0.15)).astype(float), np.ones(30), "same") >= 29
      s = starts(nr); R[key] += len(s); R[key[0] + "L" + key[1:]] += int(sum(1 for k in s if -a1[k] < a2[k]))
print("band        build    off-min | model frame: offset cm  approach/h (left) | real frame: offset cm  approach/h (left)")
for bi, (lo, hi) in enumerate(BANDS):
  for b in P_OFF:
    R = A[b][bi]
    if R['mins'] < 2:
      continue
    o = np.concatenate(R['off']); orr = np.concatenate(R['offr'])
    print(f"{lo:3d}-{hi:<3d}  {b}  {R['mins']:6.1f} | {np.mean(o) * 100:+6.1f}  {R['n'] / R['mins'] * 60:5.0f} ({R['nL'] / max(R['n'], 1):4.0%}) | "
          f"{np.mean(orr) * 100:+6.1f}  {R['nr'] / R['mins'] * 60:5.0f} ({R['nLr'] / max(R['nr'], 1):4.0%})")
