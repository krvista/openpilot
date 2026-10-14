"""E15: 'snap' events per build — measured wheel angle moves > 2.5 deg within 0.3 s while op is active, >= 40 km/h,
|wheel| < 20 deg, no blinker/LC within 3 s, driver NOT steering at that moment (|tq| < 150, not pressed). Attribution:
(a) authority recovery — ACI gain rose by >= 0.3 in the 0.5 s before (the command was already there, the MDPS got
    authority back), (b) command step — |apply| changed > 2 deg in the 0.5 s before with gain steady,
(c) other. Also how often a snap moves the car toward the nearer lane line is not available here (no lanes at 100 Hz)."""
import glob
import sys
import numpy as np

S = sys.argv[1]
res = {}
for f in sorted(glob.glob(f"{S}/ropm/0*.npz") + glob.glob(f"{S}/r3d/0*.npz") + glob.glob(f"{S}/r43/0*.npz") + glob.glob(f"{S}/r46/0*.npz")):
  d = np.load(f, allow_pickle=True); b = str(d['commit'])
  T = d['T']; ang = d['ang']; ap = d['apply']; g = d['gain']; tq = d['tq_s']; v = d['v']
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  cb = np.concatenate([[0], np.cumsum(bl)])
  idx = np.arange(len(T))
  busy = (cb[np.minimum(idx + 300, len(T))] - cb[np.maximum(idx - 300, 0)]) > 0
  ok = d['lat'] & (v >= 40) & (np.abs(ang) < 20) & ~busy
  st = res.setdefault(b, dict(min=0.0, a=0, b=0, c=0, mag=[]))
  st['min'] += ok.sum() * 0.01 / 60
  i = 50; n = len(T)
  while i < n - 30:
    if ok[i] and abs(ang[i + 30] - ang[i]) > 2.5 and np.max(np.abs(tq[i:i + 30])) < 150 and not d['pressed'][i:i + 30].any():
      dg = g[i] - np.min(g[i - 50:i + 1])
      da = abs(ap[i] - ap[i - 50])
      if dg >= 0.3:
        st['a'] += 1
      elif da > 2.0:
        st['b'] += 1
      else:
        st['c'] += 1
      st['mag'].append(abs(ang[i + 30] - ang[i]))
      i += 100
    else:
      i += 1
for b, st in res.items():
  h = st['min'] / 60
  tot = st['a'] + st['b'] + st['c']
  print(f"{b}: op-active >=40 km/h {st['min']:5.0f} min | snaps {tot / max(h, 1e-9):5.1f}/h  (authority recovery {st['a'] / max(h, 1e-9):4.1f}/h, "
        f"command step {st['b'] / max(h, 1e-9):4.1f}/h, other {st['c'] / max(h, 1e-9):4.1f}/h)  size p50 {np.median(st['mag']) if st['mag'] else 0:.1f} deg")
