"""E20: MDPS-authority (ACI gain) behaviour while the driver is NOT holding the wheel (|tq| < 150, not pressed, +-1 s),
op active, >= 25 km/h: how often the gain is below 0.9, dips (drop >= 0.2 then recovery within 3 s), the 39a city-release
flag toggles, and the wheel movement that follows a gain recovery (|ang - apply| closing). Per build."""
import glob
import sys
import numpy as np

S = sys.argv[1]
for f in sorted(glob.glob(f"{S}/ropm/0*.npz") + glob.glob(f"{S}/r3d/0*.npz") + glob.glob(f"{S}/r43/0*.npz") + glob.glob(f"{S}/r46/0*.npz")):
  d = np.load(f, allow_pickle=True); b = str(d['commit']); r = f[-12:-4]
  v = d['v']; lat = d['lat']; g = d['gain']; tq = np.abs(d['tq_s']); pr = d['pressed']
  n = len(v)
  hands = np.convolve((pr | (tq >= 150)).astype(float), np.ones(201), 'same') > 0
  off = lat & (v >= 25) & ~hands
  if off.sum() < 3000:
    continue
  mins = off.sum() * 0.01 / 60
  low = (g < 0.9) & off
  # dips: gain falls by >= 0.2 within 0.5 s while hands-off, then rises back by >= 0.2 within 3 s
  dips = 0; rec = []
  i = 50
  while i < n - 300:
    if off[i] and g[i - 50] - g[i] >= 0.2 and off[i - 50:i + 300].all():
      j = i + np.argmax(g[i:i + 300] >= g[i] + 0.2) if (g[i:i + 300] >= g[i] + 0.2).any() else None
      if j is not None:
        dips += 1
        gap0 = abs(d['apply'][j] - d['ang'][j]); gap1 = abs(d['apply'][min(j + 50, n - 1)] - d['ang'][min(j + 50, n - 1)])
        rec.append(gap0 - gap1)
      i += 300
    else:
      i += 1
  cr = ((d['sf_cc'].astype(np.int64) >> 18) & 1)
  tog = int(np.sum((cr[1:] != cr[:-1]) & off[1:]))
  print(f"{r} {b}: hands-off {mins:5.1f} min | gain<0.9 {low.sum() / off.sum():6.1%} (p10 gain {np.percentile(g[off], 10):.2f}) | "
        f"dips {dips / mins * 60:5.1f}/h, wheel gap closed after recovery p50 {np.median(rec) if rec else 0:+.1f} deg | "
        f"city-release toggles {tog / mins * 60:6.0f}/h")
