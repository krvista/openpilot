"""E123 (report §91): does the ACI gain scale the MDPS's effort? Hands off (as E122), steady TX ramps 2-10 deg/s over 0.5 s:
the MDPS output-torque change in the ramp direction (peak within the ramp + 0.15 s, relative to the value at ramp start)
per deg/s of TX rate, by gain bin and speed band. A request moving twice as fast should need about twice the torque; if
the gain scales authority the torque per deg/s grows with the gain. argv: S"""
import glob, sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; W = 0.5; nw, nd, nh = 50, 15, 25
VB = [(15, 30), (30, 40), (40, 55), (55, 70), (70, 90), (90, 200)]; GB = [0.1, 0.35, 0.55, 0.75, 0.9, 0.97, 1.01]
A = {i: [] for i in range(len(VB))}
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = dict(np.load(f, allow_pickle=True)); n = len(d['T'])
  if n < 2000: continue
  v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  ok = d['lat'] & (dd < 40) & ~d['pressed'] & (d['tx_active'] == 2) & ~d['lb'] & ~d['rb']
  okc = np.concatenate([[0], np.cumsum(ok)]); wire, ang, g, tq = d['wire'], d['ang'], d['gain'], d['mdps_tq']
  t = np.arange(nw, n - nd, 10); allok = (okc[t + nd + 1] - okc[t - nw]) == (nw + nd + 1)
  r1 = (wire[t - nh] - wire[t - nw]) / (W / 2); r2 = (wire[t] - wire[t - nh]) / (W / 2); rtx = (wire[t] - wire[t - nw]) / W
  steady = (np.sign(r1) == np.sign(r2)) & (np.abs(r1) <= 2 * np.abs(r2) + 0.2) & (np.abs(r2) <= 2 * np.abs(r1) + 0.2)
  gm = np.minimum(g[t], g[t - nw]); gx = np.maximum(g[t], g[t - nw])
  m = allok & steady & (np.abs(rtx) >= 2.0) & (np.abs(rtx) < 10.0) & (gx - gm < 0.12)
  for k in np.flatnonzero(m):
    tt = t[k]; s = np.sign(rtx[k]); seg = (tq[tt - nw:tt + nd] - tq[tt - nw]) * s
    for i, (lo, hi) in enumerate(VB):
      if lo <= v[tt] < hi:
        A[i].append((gm[k], seg.max() / abs(rtx[k]), (ang[tt + nd] - ang[tt + nd - nw]) / W * s / abs(rtx[k]))); break
print("MDPS torque rise per deg/s of TX rate, p50 [p25-p75] (n) by gain bin")
print("  speed    " + "".join(f"| g {a:.2f}-{b:.2f}           " for a, b in zip(GB[:-1], GB[1:])))
for i, (lo, hi) in enumerate(VB):
  X = np.array(A[i]); l1 = f"  {lo:3d}-{hi:<3d}  "
  for a, b in zip(GB[:-1], GB[1:]):
    q = (X[:, 0] >= a) & (X[:, 0] < b)
    l1 += f"| {np.median(X[q, 1]):4.2f} [{np.percentile(X[q, 1], 25):4.2f}-{np.percentile(X[q, 1], 75):4.2f}] ({q.sum():4d}) " if q.sum() >= 15 else "|   -                   "
  print(l1)
