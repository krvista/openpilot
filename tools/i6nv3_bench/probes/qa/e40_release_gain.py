"""E40: wheel behaviour right after the driver lets go, versus the MDPS authority (ACI gain) at that instant. Release edge
as in E39 (|tq| >= 150 for >= 0.3 s, then < 60 within 0.3 s, staying < 150 for 1.5 s), op active, >= 30 km/h, no blinker.
Per release: gain at the release instant and its time to reach 0.5; the wheel's peak rate in the next 0.6 s; whether that
motion was a FREE RETURN (|wheel| decreasing toward centre while the command did not move that way: |d apply| < 1/3 of the
wheel move); wheel excursion at the release. Binned by release gain and by |wheel| at release."""
import glob
import sys
import numpy as np

S = sys.argv[1]
R = []
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = np.load(f, allow_pickle=True); b = str(d['commit'])
  T = d['T']; n = len(T); v = d['v']
  if n < 3000:
    continue
  ang = d['ang']; ap = d['apply']; tq = np.abs(d['tq_s']); g = d['gain']
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  c = lambda m: np.concatenate([[0], np.cumsum(m)]); idx = np.arange(n)
  busy = (c(bl)[np.minimum(idx + 300, n)] - c(bl)[np.maximum(idx - 300, 0)]) > 0
  act = d['lat'] & (v >= 30) & ~busy
  hi = tq >= 150
  i = 30
  while i < n - 220:
    if hi[i - 1] and not hi[i] and hi[i - 30:i].all():
      j = i + int(np.argmax(tq[i:i + 30] < 60)) if (tq[i:i + 30] < 60).any() else None
      if j is not None and act[j:j + 200].all() and not hi[j:j + 150].any():
        w = slice(j, j + 60)
        rate = np.abs(np.diff(ang[j:j + 61:10])) / 0.1
        mv = ang[j + 60] - ang[j]; dap = ap[j + 60] - ap[j]
        free = abs(ang[j + 60]) < abs(ang[j]) - 1.0 and abs(dap) < abs(mv) / 3
        t50 = (np.argmax(g[j:j + 200] >= 0.5) * 0.01) if (g[j:j + 200] >= 0.5).any() else 2.0
        R.append((g[j], rate.max(), free, abs(ang[j]), t50, b, abs(mv)))
      i += 150
    else:
      i += 1
A = np.array([r[:5] for r in R], dtype=float); fr = A[:, 2] > 0
print(f"releases {len(A)}: free returns {fr.sum()} ({fr.mean():.0%}); peak wheel rate p50 {np.median(A[:, 1]):.0f} deg/s; time for gain to reach 0.5 p50 {np.median(A[:, 4]):.2f} s")
for lo, hi_ in ((0, 0.1), (0.1, 0.3), (0.3, 0.6), (0.6, 1.01)):
  m = (A[:, 0] >= lo) & (A[:, 0] < hi_)
  if m.sum():
    print(f"  gain at release {lo:.1f}-{hi_:.1f}: n {m.sum():3d} | free return {np.mean(fr[m]):4.0%} | peak rate p50 {np.median(A[m, 1]):3.0f} p90 {np.percentile(A[m, 1], 90):3.0f} deg/s | "
          f">= 20 deg/s {np.mean(A[m, 1] >= 20):4.0%} | |wheel| at release p50 {np.median(A[m, 3]):4.1f} deg | gain->0.5 p50 {np.median(A[m, 4]):.2f} s")
for lo, hi_ in ((0, 3), (3, 8), (8, 90)):
  m = (A[:, 3] >= lo) & (A[:, 3] < hi_)
  if m.sum():
    print(f"  |wheel| {lo}-{hi_} deg: n {m.sum():3d} | free return {np.mean(fr[m]):4.0%} | peak rate p50 {np.median(A[m, 1]):3.0f} p90 {np.percentile(A[m, 1], 90):3.0f} | gain at release p50 {np.median(A[m, 0]):.2f}")
