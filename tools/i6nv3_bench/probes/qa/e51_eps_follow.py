"""E51: how the MDPS follows the TX angle. Frames with ACI gain >= 0.95 for the last 0.3 s, op active >= 15 km/h, driver
|tq| < 60 over the last 0.3 s (hands quiet). Wheel rate (0.1 s) toward TX vs the gap (TX - wheel) * sign: binned medians and
p90 of the rate; and the TX rate itself. Also by speed. Shows whether the EPS follows proportionally (rate ~ gap) or saturates."""
import glob
import sys
import numpy as np
S = sys.argv[1]
rows = []
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = np.load(f, allow_pickle=True); b = str(d['commit'])
  if b != "98e912d3d":
    continue
  T = d['T']; n = len(T); v = d['v']
  if n < 3000:
    continue
  ang = d['ang']; tx = d['wire']; g = d['gain']; tq = np.abs(d['tq_s'])
  c = lambda m: np.concatenate([[0], np.cumsum(m)])
  okg = c(g < 0.95); okt = c(tq >= 60); idx = np.arange(n)
  sel = (idx >= 40) & (idx < n - 10)
  ii = idx[sel]
  m = (okg[ii + 1] - okg[ii - 30] == 0) & (okt[ii + 1] - okt[ii - 30] == 0) & d['lat'][ii] & (v[ii] >= 15)
  ii = ii[m][::5]
  gap = tx[ii - 5] - ang[ii - 5]
  wr = (ang[ii + 5] - ang[ii - 5]) / 0.1
  txr = (tx[ii + 5] - tx[ii - 5]) / 0.1
  rows.append(np.stack([gap, wr, txr, v[ii]], 1))
R = np.concatenate(rows)
s = np.sign(R[:, 0]); G = np.abs(R[:, 0]); W = R[:, 1] * s; X = R[:, 2] * s
print(f"frames {len(R)} (gain >= 0.95, hands quiet)")
for lo, hi in ((0, 1), (1, 2), (2, 3), (3, 5), (5, 8), (8, 12), (12, 20), (20, 99)):
  m = (G >= lo) & (G < hi)
  if m.sum() < 10:
    continue
  print(f"  gap {lo:2d}-{hi:2d} deg: n {m.sum():6d} | wheel rate toward TX p50 {np.median(W[m]):5.1f} p90 {np.percentile(W[m], 90):5.1f} max {W[m].max():5.0f} deg/s "
        f"| TX rate (same dir) p50 {np.median(X[m]):+5.1f} | wheel/gap p50 {np.median(W[m] / np.maximum(G[m], 0.5)):.2f} 1/s")
for lo, hi in ((15, 40), (40, 70), (70, 200)):
  m = (G >= 5) & (R[:, 3] >= lo) & (R[:, 3] < hi)
  if m.sum() > 10:
    print(f"  gap >= 5, {lo}-{hi} km/h: n {m.sum()} wheel rate p50 {np.median(W[m]):.1f} p90 {np.percentile(W[m], 90):.1f}")
