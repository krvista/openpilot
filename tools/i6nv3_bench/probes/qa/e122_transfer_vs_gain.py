"""E122 (report §91): MDPS rate transfer vs the ACI gain op sends. Hands off (hold-compensated driver torque < 40, EPS
press flag off, no blinker), wire active, steady TX ramps of 2-10 deg/s over 0.5 s (as E117-A): wheel rate / TX rate by
gain bin and speed band. Also the quasi-steady wheel / TX ratio (|TX| >= 1.5 deg, both still for 1 s) by gain bin.
argv: S"""
import glob, sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; W, D = 0.5, 0.15; nw, nd, nh = 50, 15, 25
VB = [(15, 30), (30, 40), (40, 55), (55, 70), (70, 90), (90, 200)]; GB = [0.1, 0.35, 0.55, 0.75, 0.9, 0.97, 1.01]
A = {i: [] for i in range(len(VB))}; B = {i: [] for i in range(len(VB))}
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = dict(np.load(f, allow_pickle=True)); n = len(d['T'])
  if n < 2000: continue
  v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  ok = d['lat'] & (dd < 40) & ~d['pressed'] & (d['tx_active'] == 2) & ~d['lb'] & ~d['rb']
  okc = np.concatenate([[0], np.cumsum(ok)]); wire, ang, g = d['wire'], d['ang'], d['gain']
  t = np.arange(nw, n - nd); allok = (okc[t + nd + 1] - okc[t - nw]) == (nw + nd + 1)
  r1 = (wire[t - nh] - wire[t - nw]) / (W / 2); r2 = (wire[t] - wire[t - nh]) / (W / 2); rtx = (wire[t] - wire[t - nw]) / W
  steady = (np.sign(r1) == np.sign(r2)) & (np.abs(r1) <= 2 * np.abs(r2) + 0.2) & (np.abs(r2) <= 2 * np.abs(r1) + 0.2)
  rw = (ang[t + nd] - ang[t + nd - nw]) / W
  gm = np.minimum(g[t], g[t - nw]); gx = np.maximum(g[t], g[t - nw])
  m = allok & steady & (np.abs(rtx) >= 2.0) & (np.abs(rtx) < 10.0) & (gx - gm < 0.12)
  st = np.zeros(n, bool); st[100:] = (np.abs(wire[100:] - wire[:-100]) < 0.3) & (np.abs(ang[100:] - ang[:-100]) < 0.3)
  ok1 = np.zeros(n, bool); t1 = np.arange(100, n); ok1[100:] = (okc[t1 + 1] - okc[t1 - 100]) == 101
  ms = ok1 & st & (np.abs(wire) >= 1.5)
  for i, (lo, hi) in enumerate(VB):
    mm = m & (v[t] >= lo) & (v[t] < hi)
    A[i].append(np.c_[gm[mm], rw[mm] * np.sign(rtx[mm]) / np.abs(rtx[mm])][::5])
    mm = ms & (v >= lo) & (v < hi)
    B[i].append(np.c_[g[mm], ang[mm] / wire[mm]][::25])
print("fast-ramp transfer (wheel rate / TX rate) p50 (n) by gain bin        || steady wheel/TX p50 (n)")
print("  speed    " + "".join(f"| g {a:.2f}-{b:.2f}   " for a, b in zip(GB[:-1], GB[1:])))
for i, (lo, hi) in enumerate(VB):
  X = np.concatenate(A[i]); Y = np.concatenate(B[i]); l1 = f"  {lo:3d}-{hi:<3d}  "; l2 = "  steady   "
  for a, b in zip(GB[:-1], GB[1:]):
    q = (X[:, 0] >= a) & (X[:, 0] < b); l1 += f"| {np.median(X[q, 1]):4.2f} ({q.sum():4d})  " if q.sum() >= 15 else "|   -          "
    q = (Y[:, 0] >= a) & (Y[:, 0] < b); l2 += f"| {np.median(Y[q, 1]):4.2f} ({q.sum():4d})  " if q.sum() >= 15 else "|   -          "
  print(l1); print(l2)
