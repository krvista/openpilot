"""E16: closed-loop lane-keeping quality per build from the real drives (model lane lines): op active, both inner lines
> 0.7, >= 30 km/h, no blinker/LC +-3 s, driver not steering (|tq| < 150 and not pressed within +-1 s). Offset from lane
centre (de-meaned per 20 s block to remove mount/lane-width bias): RMS on straights / curves, curve offset toward the
OUTSIDE (signed by road curvature), lateral weave power 0.1-0.6 Hz on straights, heading RMS (lane slope)."""
import glob
import sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from common import bfit, interp_rows

S = sys.argv[1]
res = {}
for pf in sorted(glob.glob(f"{S}/la/plan_*.npz")):
  r = pf[-12:-4]
  rf = glob.glob(f"{S}/ropm/{r}.npz") + glob.glob(f"{S}/r46/{r}.npz") + glob.glob(f"{S}/r3d/{r}.npz") + glob.glob(f"{S}/r43/{r}.npz") + glob.glob(f"{S}/rx/{r}.npz")
  if not rf:
    continue
  P = dict(np.load(pf)); d = dict(np.load(rf[0], allow_pickle=True))
  T = P['T']; n = len(T)
  if n < 2000:
    continue
  Tr = d['T']; b = str(d['commit'])
  v = np.interp(T, Tr, d['v']); lat = np.interp(T, Tr, d['lat'].astype(float)) > 0.5
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0); drv = d['pressed'] | (np.abs(d['tq_s']) >= 150)
  cs = lambda m, w: (np.concatenate([[0], np.cumsum(m)])[np.searchsorted(Tr, T + w)] - np.concatenate([[0], np.cumsum(m)])[np.searchsorted(Tr, T - w)]) > 0
  busy = cs(bl, 3.0); hand = cs(drv, 1.0)
  LX = P['LX'].astype(float); LC = 0.5 * (P['L1'].astype(float) + P['L2'].astype(float))
  xs = np.tile(np.linspace(0, 20, 9), (n, 1))
  cf = bfit(xs, interp_rows(xs, LX, LC), 2)
  kr = 2 * cf[:, 0]; e = -cf[:, 2]; head = cf[:, 1]
  good = lat & (P['PR'][:, 1] > 0.7) & (P['PR'][:, 2] > 0.7) & ~busy & (v >= 30)
  blk = (T // 20).astype(int)
  em = np.zeros(n)
  for k in np.unique(blk[good]):
    m = good & (blk == k)
    em[m] = e[m] - e[m].mean()
  st = res.setdefault(b, dict(rows=[], wv=[]))
  for hs, nm in ((~hand, "off"), (hand, "on")):
    m = good & hs
    j = np.minimum(np.searchsorted(T, T + 0.6), n - 1); k_ah = kr[j]
    ph = np.where(np.abs(k_ah) > np.abs(kr) + 0.7e-3, 1, np.where(np.abs(kr) > np.abs(k_ah) + 0.7e-3, 3, 2))
    st['rows'].append(np.stack([np.full(m.sum(), nm == "off"), np.abs(kr[m]), em[m], e[m] * -np.sign(kr[m]), head[m], ph[m], v[m]], 1))
  # weave on straight stretches >= 10 s
  m = good & (np.abs(kr) < 0.5e-3)
  i = 0
  while i < n:
    if m[i]:
      j = i
      while j + 1 < n and m[j + 1] and T[j + 1] - T[j] < 0.1:
        j += 1
      if T[j] - T[i] >= 10:
        seg = e[i:j + 1] - e[i:j + 1].mean()
        fr = np.fft.rfftfreq(len(seg), 0.05); Pw = np.abs(np.fft.rfft(seg * np.hanning(len(seg)))) ** 2
        tot = Pw[1:].sum()
        st['wv'].append(Pw[(fr >= 0.1) & (fr <= 0.6)].sum() / tot if tot > 0 else 0)
      i = j + 1
    else:
      i += 1
  print(r, b, flush=True)
for b, st in res.items():
  A = np.concatenate(st['rows'])
  out = [f"== {b}"]
  for ho, hn in ((1, "hands-off"), (0, "hands-on")):
    s = A[:, 0] == ho
    st_ = s & (A[:, 1] < 0.5e-3); cv = s & (A[:, 1] >= 1.5e-3)
    if st_.sum() < 200:
      continue
    out.append(f"  {hn:9} straight {st_.sum() / 1200:5.1f} min offset RMS {np.sqrt(np.mean(A[st_, 2] ** 2)):.3f} m, heading RMS "
               f"{np.sqrt(np.mean(A[st_, 4] ** 2)) * 1e3:.1f} mrad | curve {cv.sum() / 1200:4.1f} min offset RMS {np.sqrt(np.mean(A[cv, 2] ** 2)):.3f}, "
               f"toward outside mean {np.mean(A[cv, 3]):+.3f} p90 {np.percentile(A[cv, 3], 90) if cv.sum() else 0:+.3f} m")
  for ph, pn in ((1, "entry"), (2, "steady"), (3, "exit")):
    for ho in (1, 0):
      s = (A[:, 0] == ho) & (A[:, 1] >= 1.0e-3) & (A[:, 5] == ph)
      if s.sum() > 100:
        out.append(f"    {pn:6} {'off' if ho else 'on '} n {s.sum():5d} toward-outside mean {np.mean(A[s, 3]):+.3f} p90 {np.percentile(A[s, 3], 90):+.3f} m v {np.median(A[s, 6]):.0f}")
  out.append(f"  straight weave share 0.1-0.6 Hz: median {np.median(st['wv']) if st['wv'] else 0:.2f} (n {len(st['wv'])})")
  print("\n".join(out))
