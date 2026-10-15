"""E21: why is the authority (ACI gain) below 0.9 while hands-off? Latest-build routes: hands-off frames (|tq| < 150,
not pressed, +-1 s), op active >= 25 km/h: split gain < 0.9 by time since the last touch (|tq| >= 150 or pressed),
by the carcontroller flags set, by speed band and by |command| — and the tracking cost: |apply - ang| at gain < 0.9
vs >= 0.9 in steady curves (|apply| > 3 deg)."""
import glob
import sys
import numpy as np

S = sys.argv[1]
CC = {15: "43b accel yield", 16: "boost suppressed", 17: "33 blind caution", 18: "39a city release", 19: "35b anchored rec",
      20: "blinker concession", 21: "29 post-grip", 27: "38 tx saturated", 28: "51 HOD touch", 5: "angle passive"}
rows = []
for f in sorted(glob.glob(f"{S}/r46/0*.npz")):
  d = np.load(f, allow_pickle=True)
  if str(d['commit']) != "8de98262a":
    continue
  v = d['v']; lat = d['lat']; g = d['gain']; tq = np.abs(d['tq_s']); pr = d['pressed']; sc = d['sf_cc'].astype(np.int64)
  n = len(v)
  touch = pr | (tq >= 150)
  last = np.full(n, -1e9); t_last = -1e9
  T = d['T']
  for i in range(n):
    if touch[i]:
      t_last = T[i]
    last[i] = T[i] - t_last
  hands = np.convolve(touch.astype(float), np.ones(201), 'same') > 0
  off = lat & (v >= 25) & ~hands
  rows.append(np.stack([g[off], last[off], v[off], np.abs(d['apply'][off]), np.abs(d['apply'][off] - d['ang'][off]), sc[off],
                        d['a_ego'][off], d['gas'][off].astype(float)], 1))
A = np.concatenate(rows)
low = A[:, 0] < 0.9
print(f"hands-off frames {len(A)} ({len(A) / 6000:.1f} min), gain < 0.9: {low.mean():.1%}")
print("by time since last touch:")
for lo, hi in ((0, 1), (1, 2), (2, 4), (4, 8), (8, 1e9)):
  s = (A[:, 1] >= lo) & (A[:, 1] < hi)
  print(f"   {lo:3.0f}-{hi if hi < 1e8 else 999:3.0f} s: frames {s.mean():5.1%}  gain<0.9 {low[s].mean():5.1%}  gain p50 {np.median(A[s, 0]):.2f}")
print("flags set while gain < 0.9 (share of those frames):")
for bit, nm in CC.items():
  x = ((A[:, 5].astype(np.int64) >> bit) & 1).astype(bool)
  print(f"   {nm:18} {np.mean(x[low]):6.1%}   (gain>=0.9 frames: {np.mean(x[~low]):5.1%})")
print("no flag of the above while gain<0.9:", f"{np.mean([not any(((int(x) >> b) & 1) for b in CC) for x in A[low, 5]]):.1%}")
print("by speed:")
for lo, hi in ((25, 45), (45, 65), (65, 200)):
  s = (A[:, 2] >= lo) & (A[:, 2] < hi)
  print(f"   {lo}-{hi} km/h: gain<0.9 {low[s].mean():5.1%}  gain p10/p50 {np.percentile(A[s, 0], 10):.2f}/{np.median(A[s, 0]):.2f}")
print("accel while gain<0.9: a_ego p50 %.2f, gas pressed %.0f%%  | gain>=0.9: a_ego p50 %.2f, gas %.0f%%" % (
  np.median(A[low, 6]), 100 * np.mean(A[low, 7]), np.median(A[~low, 6]), 100 * np.mean(A[~low, 7])))
cv = A[:, 3] > 3
print(f"tracking |apply-ang| in curves (|apply|>3): gain<0.9 p50 {np.median(A[cv & low, 4]):.2f} deg (n {np.sum(cv & low)}), gain>=0.9 p50 {np.median(A[cv & ~low, 4]):.2f} deg (n {np.sum(cv & ~low)})")
