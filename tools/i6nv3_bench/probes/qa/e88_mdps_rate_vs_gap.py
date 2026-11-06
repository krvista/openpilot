"""E88: MDPS wheel rate (toward the request, 0.1 s difference) as a function of the TX - wheel gap and speed, op active,
hands off (hold-compensated driver torque < 40, not pressed), gain >= 0.95. Is there a gap below which the wheel does not
move (breakaway) and a speed-dependent ceiling on how fast it moves (MDPS rate limit)? Also the same conditioned on the
wheel being at rest for the previous 0.3 s (stiction from rest) vs already moving."""
import glob, sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; BUILDS = sys.argv[2].split(",") if len(sys.argv) > 2 else ["bf36aa2"]
VB = [(15, 30), (30, 40), (40, 55), (55, 70), (70, 90), (90, 200)]
GB = [0.5, 1, 2, 3, 4, 6, 8, 12, 20]
acc = {i: [] for i in range(len(VB))}
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = dict(np.load(f, allow_pickle=True))
  if str(d['commit'])[:7] not in BUILDS:
    continue
  T = d['T']; n = len(T); v = d['v']; vm = v / 3.6
  la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  ok = d['lat'] & (dd < 40) & ~d['pressed'] & (d['gain'] >= 0.95) & (d['tx_active'] == 2)
  ang = d['ang']; wire = d['wire']
  # wheel move over the NEXT 0.1 s, toward the gap measured now (MDPS delay ~0.1-0.2 s is inside the rate window below)
  fut = np.zeros(n); fut[:-20] = (ang[20:] - ang[:-20]) / 0.2     # 0.2 s ahead
  gap = wire - ang
  sg = np.sign(gap)
  rate_tw = fut * sg
  past = np.zeros(n); past[30:] = np.abs(ang[30:] - ang[:-30])
  rest = past < 0.3
  for i, (lo, hi) in enumerate(VB):
    m = ok & (v >= lo) & (v < hi)
    acc[i].append(np.c_[np.abs(gap[m]), rate_tw[m], rest[m]])
for i, (lo, hi) in enumerate(VB):
  if not acc[i]:
    continue
  A = np.concatenate(acc[i])
  if len(A) < 1000:
    continue
  line = f"{lo:3d}-{hi:<3d} ({len(A)/6000:4.0f} min) "
  for g0, g1 in zip(GB[:-1], GB[1:]):
    m = (A[:, 0] >= g0) & (A[:, 0] < g1)
    if m.sum() < 50:
      line += f"| {g0:g}-{g1:g}:   -     "; continue
    mr = m & (A[:, 2] > 0)
    line += f"| {g0:g}-{g1:g}: {np.median(A[m, 1]):4.1f}/{np.percentile(A[m, 1], 90):4.1f} r{np.median(A[mr, 1]) if mr.sum() > 30 else np.nan:4.1f} "
  print(line)
print("cells: gap band deg: wheel rate toward TX p50/p90 deg/s, r = p50 when the wheel was at rest the previous 0.3 s")
