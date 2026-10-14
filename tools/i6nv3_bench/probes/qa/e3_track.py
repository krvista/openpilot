"""E3: tracking in sustained curves (100 Hz). Frames where |k_ctrl| >= 1.5e-3 and its 1 s change < 15 %, lat active,
>= 25 km/h, no blinker/LC. Ratios: measured angle / commanded angle (MDPS), gyro curvature (yaw / v) / k_ctrl (vehicle),
and the angle error in deg, by speed band, hands-off vs on, per build. Also the lag: delay maximising corr(apply, ang)."""
import glob
import sys
import numpy as np

S = sys.argv[1]
rows = []
for f in sorted(glob.glob(f"{S}/ropm/0*.npz") + glob.glob(f"{S}/r3d/0*.npz") + glob.glob(f"{S}/r43/0*.npz")):
  d = np.load(f, allow_pickle=True)
  b = {"94b5b8454": 0, "d84d1a8f7": 1, "cb10ad784": 2}[str(d['commit'])]
  v = d['v'] / 3.6
  k = d['k_ctrl']
  k1 = np.concatenate([np.full(100, np.nan), k[:-100]])
  hands = np.convolve((d['pressed'] | (np.abs(d['tq_s']) > 60)).astype(float), np.ones(201), 'same') > 0
  ok = (d['lat'] & (v >= 25 / 3.6) & (np.abs(k) >= 1.5e-3) & (np.abs(k - k1) < 0.15 * np.abs(k)) & ~d['lb'] & ~d['rb']
        & (d['lcs'] == 0) & (np.abs(d['apply']) > 1.0))
  ky = d['yaw'] / np.maximum(v, 1.0)
  rows.append(np.stack([np.full(ok.sum(), b), d['v'][ok], np.abs(k[ok]), d['ang'][ok] / d['apply'][ok], ky[ok] / k[ok],
                        np.abs(d['apply'][ok]) - np.abs(d['ang'][ok]), hands[ok], d['k_meas'][ok] / k[ok]], 1))
  print(f[-12:-4], ok.sum(), flush=True)
A = np.concatenate(rows)
print("cols: n | ang/apply p50 | yaw-curv/k_ctrl p50 | k_meas/k_ctrl p50 | |apply|-|ang| p50/p90 deg")
for b, nm in ((0, "94b5b8454"), (1, "d84d1a8f7"), (2, "cb10ad784")):
  for h in (0, 1):
    for lo, hi in ((25, 50), (50, 70), (70, 90), (90, 200)):
      s = (A[:, 0] == b) & (A[:, 6] == h) & (A[:, 1] >= lo) & (A[:, 1] < hi)
      if s.sum() < 300:
        continue
      print(f"  {nm} hands {'on ' if h else 'off'} {lo:3}-{hi:<3} n {s.sum():6d}  {np.median(A[s,3]):.3f}  {np.median(A[s,4]):.3f}  "
            f"{np.median(A[s,7]):.3f}  {np.median(A[s,5]):+.2f}/{np.percentile(A[s,5],90):+.2f}")
for lo, hi in ((1.5e-3, 3e-3), (3e-3, 6e-3), (6e-3, 1.5e-2), (1.5e-2, 1)):
  s = (A[:, 6] == 0) & (A[:, 2] >= lo) & (A[:, 2] < hi)
  print(f"  hands-off |k| {lo*1e3:.1f}-{hi*1e3:.0f}e-3 n {s.sum():6d} ang/apply {np.median(A[s,3]):.3f} yaw/k {np.median(A[s,4]):.3f} v {np.median(A[s,1]):.0f}")
