"""E119 (report §91): does the MDPS rate transfer depend on the lateral load? Same selection as E117-A with W = 1.0 s,
outward TX ramps only, by |TX rate| (slow 0.8-2.5 / fast 2.5-8 deg/s) and by lateral acceleration at the window start.
Also the MDPS output torque change over the window per degree of TX change. argv: S"""
import glob, sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; W, D = 1.0, 0.15; nw, nd, nh = 100, 15, 50
VB = [(30, 55), (55, 70), (70, 90), (90, 200)]; LB = [0, 0.3, 0.6, 1.0, 1.5, 6.0]; RBINS = [("slow 0.8-2.5", 0.8, 2.5), ("fast 2.5-8", 2.5, 8.0)]
A = {i: [] for i in range(len(VB))}
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = dict(np.load(f, allow_pickle=True)); n = len(d['T'])
  if n < 2000: continue
  v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  ok = d['lat'] & (dd < 40) & ~d['pressed'] & (d['gain'] >= 0.95) & (d['tx_active'] == 2) & ~d['lb'] & ~d['rb']
  okc = np.concatenate([[0], np.cumsum(ok)]); wire, ang, tq = d['wire'], d['ang'], d['mdps_tq']
  t = np.arange(nw, n - nd); allok = (okc[t + nd + 1] - okc[t - nw]) == (nw + nd + 1)
  r1 = (wire[t - nh] - wire[t - nw]) / (W / 2); r2 = (wire[t] - wire[t - nh]) / (W / 2); rtx = (wire[t] - wire[t - nw]) / W
  steady = (np.sign(r1) == np.sign(r2)) & (np.abs(r1) <= 2 * np.abs(r2) + 0.2) & (np.abs(r2) <= 2 * np.abs(r1) + 0.2)
  rw = (ang[t + nd] - ang[t + nd - nw]) / W; s = np.sign(rtx)
  outward = (s == np.sign(wire[t - nw])) & (np.abs(wire[t - nw]) >= 0.5) & (s == np.sign(ang[t - nw]))
  dtq = (tq[t] - tq[t - nw]) * s
  m = allok & steady & outward & (np.abs(rtx) >= 0.8)
  for i, (lo, hi) in enumerate(VB):
    mm = m & (v[t] >= lo) & (v[t] < hi)
    A[i].append(np.c_[np.abs(rtx[mm]), rw[mm] * s[mm], la[t - nw][mm], dtq[mm], np.abs(tq[t - nw][mm])][::10])
print("outward steady TX ramps, 1 s windows: wheel rate / TX rate p50 [p25-p75] (n) | MDPS torque change per deg of TX")
for i, (lo, hi) in enumerate(VB):
  X = np.concatenate(A[i]); print(f"[{lo}-{hi} km/h]")
  for nm, a, b in RBINS:
    line = f"   {nm:13s}"
    for l0, l1 in zip(LB[:-1], LB[1:]):
      q = (X[:, 0] >= a) & (X[:, 0] < b) & (X[:, 2] >= l0) & (X[:, 2] < l1)
      if q.sum() < 12: line += f"| la {l0:g}-{l1:g}: -              "; continue
      r = X[q, 1] / X[q, 0]
      line += f"| la {l0:g}-{l1:g}: {np.median(r):4.2f} [{np.percentile(r,25):4.2f}-{np.percentile(r,75):4.2f}] ({q.sum():3d}) dtq {np.median(X[q,3]/X[q,0]):5.2f} "
    print(line)
