"""E3b: steady-curve angle shortfall vs MDPS authority (gain) and lateral acceleration, hands-off only, all builds."""
import glob
import sys
import numpy as np

S = sys.argv[1]
rows = []
for f in sorted(glob.glob(f"{S}/ropm/0*.npz") + glob.glob(f"{S}/r3d/0*.npz") + glob.glob(f"{S}/r43/0*.npz")):
  d = np.load(f, allow_pickle=True)
  v = d['v'] / 3.6
  k = d['k_ctrl']
  k1 = np.concatenate([np.full(100, np.nan), k[:-100]])
  a1 = np.concatenate([np.full(50, np.nan), d['apply'][:-50]])
  hands = np.convolve((d['pressed'] | (np.abs(d['tq_s']) > 40)).astype(float), np.ones(201), 'same') > 0
  ok = (d['lat'] & (v >= 25 / 3.6) & (np.abs(k) >= 1.5e-3) & (np.abs(k - k1) < 0.15 * np.abs(k)) & ~d['lb'] & ~d['rb']
        & (d['lcs'] == 0) & (np.abs(d['apply']) > 1.0) & ~hands & (np.abs(d['apply'] - a1) < 0.1 * np.abs(d['apply'])))
  rows.append(np.stack([d['v'][ok], d['ang'][ok] / d['apply'][ok], d['gain'][ok], v[ok] ** 2 * np.abs(k[ok]), np.abs(d['apply'][ok]),
                        d['tq_eps'][ok] * np.sign(d['apply'][ok]), d['mdps_tq'][ok] * np.sign(d['apply'][ok])], 1))
A = np.concatenate(rows)
print("hands-off steady curve frames", len(A))
print("by gain:")
for lo, hi in ((0, 0.3), (0.3, 0.5), (0.5, 0.7), (0.7, 0.9), (0.9, 1.01)):
  s = (A[:, 2] >= lo) & (A[:, 2] < hi)
  if s.sum() > 100:
    print(f"  gain {lo:.1f}-{hi:.1f} n {s.sum():5d} ang/apply {np.median(A[s,1]):.3f} lat_acc {np.median(A[s,3]):.2f} |apply| {np.median(A[s,4]):.1f}")
print("by lateral accel:")
for lo, hi in ((0, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 2.5), (2.5, 5)):
  s = (A[:, 3] >= lo) & (A[:, 3] < hi)
  if s.sum() > 100:
    print(f"  a_lat {lo:.1f}-{hi:.1f} n {s.sum():5d} ang/apply {np.median(A[s,1]):.3f} gain p50 {np.median(A[s,2]):.2f} |apply| {np.median(A[s,4]):.1f} v {np.median(A[s,0]):.0f}")
print("by |apply| deg:")
for lo, hi in ((1, 3), (3, 6), (6, 12), (12, 30), (30, 200)):
  s = (A[:, 4] >= lo) & (A[:, 4] < hi)
  if s.sum() > 100:
    print(f"  |apply| {lo}-{hi} n {s.sum():5d} ang/apply {np.median(A[s,1]):.3f} short {np.median(A[s,4]*(1-A[s,1])):.2f} deg gain {np.median(A[s,2]):.2f} v {np.median(A[s,0]):.0f}")
