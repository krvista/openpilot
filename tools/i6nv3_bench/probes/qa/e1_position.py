"""E1: lateral position in lane. offset = car - lane centre (+ = toward +y = right). In curves: inside offset = offset
toward the curve centre. op hands-off vs human (lat inactive) at matched road curvature / speed."""
import sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from common import routes

S = sys.argv[1]
rows = []
for r, P, d, T, f in routes(S):
  off = -f['c0']
  kr = f['k_road']
  mode = np.where(f['lat'] & ~f['hands'], 1, np.where(~f['lat'], 0, 2))   # 1 op hands-off, 0 human, 2 op with hands
  rows.append(np.stack([np.full(len(T), int(r, 16)), f['v'] * 3.6, kr, off, off * np.sign(kr), mode, f['good'], f['width']], 1))
  print(r, flush=True)
A = np.concatenate(rows)
A = A[A[:, 6] > 0]
print(f"frames {len(A)}  op-handsoff {np.mean(A[:,5]==1):.0%} human {np.mean(A[:,5]==0):.0%}")
print("== straight (|k| < 0.3e-3): offset mean/std (m, + = right)")
for m, nm in ((1, "op hands-off"), (0, "human"), (2, "op hands-on")):
  for b in ((30, 60), (60, 90), (90, 200)):
    s = (A[:, 5] == m) & (np.abs(A[:, 2]) < 0.3e-3) & (A[:, 1] >= b[0]) & (A[:, 1] < b[1])
    if s.sum() > 200:
      print(f"  {nm:13} {b[0]:3}-{b[1]:<3} n {s.sum():6d} mean {np.mean(A[s,3]):+.3f} std {np.std(A[s,3]):.3f}  width {np.median(A[s,7]):.2f}")
print("== curves: inside offset (m, + = toward curve centre) by |k_road| and turn direction")
for m, nm in ((1, "op hands-off"), (0, "human")):
  for lo, hi in ((0.5e-3, 1.5e-3), (1.5e-3, 3e-3), (3e-3, 6e-3), (6e-3, 2e-2)):
    out = []
    for sg, dn in ((1, "+y/right"), (-1, "-y/left")):
      s = (A[:, 5] == m) & (np.abs(A[:, 2]) >= lo) & (np.abs(A[:, 2]) < hi) & (np.sign(A[:, 2]) == sg)
      out.append(f"{dn} n {s.sum():5d} {np.mean(A[s,4]) if s.sum() else np.nan:+.3f}")
    print(f"  {nm:13} |k| {lo*1e3:.1f}-{hi*1e3:.1f}e-3: " + "  ".join(out))
print("== per route straight op hands-off mean offset")
for rr in np.unique(A[:, 0]):
  s = (A[:, 0] == rr) & (A[:, 5] == 1) & (np.abs(A[:, 2]) < 0.3e-3)
  h = (A[:, 0] == rr) & (A[:, 5] == 0) & (np.abs(A[:, 2]) < 0.3e-3)
  print(f"  {int(rr):08x} op {np.mean(A[s,3]):+.3f} (n {s.sum()})  human {np.mean(A[h,3]) if h.sum() else np.nan:+.3f} (n {h.sum()})")
