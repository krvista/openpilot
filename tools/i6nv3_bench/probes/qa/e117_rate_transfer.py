"""E117 (report §91): how the MDPS turns a moving angle request into wheel motion, measured directly (no plant fit).
Op active, hands off (hold-compensated driver torque < 40, not pressed), gain >= 0.95, wire active. All builds (the MDPS is
the same car).
 A) rate transfer: steady TX ramps (rate over W s, both halves the same sign and within 2x) -> wheel rate over the same W s
    delayed by D; ratio = wheel rate / TX rate, by |TX rate| bin and speed band, outward (away from centre) / inward.
 B) trim-like ramps: the command (carControl angle) steady within 0.3 deg over 1 s while TX moves >= 1 deg in that second
    (the slow TX-layer trim integrator): wheel change / TX change over the same second.
argv: S [W=0.5] [D=0.15]"""
import glob, sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; W = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5; D = float(sys.argv[3]) if len(sys.argv) > 3 else 0.15
VB = [(15, 30), (30, 40), (40, 55), (55, 70), (70, 90), (90, 200)]
RB = [0.3, 0.7, 1.2, 1.8, 2.5, 4.0, 7.0, 12.0, 25.0]
nw, nd, nh = int(W * 100), int(D * 100), int(W * 50)
A = {i: [] for i in range(len(VB))}; B = {i: [] for i in range(len(VB))}
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = dict(np.load(f, allow_pickle=True)); n = len(d['T'])
  if n < 2000: continue
  v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  ok = d['lat'] & (dd < 40) & ~d['pressed'] & (d['gain'] >= 0.95) & (d['tx_active'] == 2) & ~d['lb'] & ~d['rb']
  okc = np.concatenate([[0], np.cumsum(ok)])
  wire, ang, ap = d['wire'], d['ang'], d['apply']
  t = np.arange(nw, n - nd)
  allok = (okc[t + nd + 1] - okc[t - nw]) == (nw + nd + 1)
  r1 = (wire[t - nh] - wire[t - nw]) / (W / 2); r2 = (wire[t] - wire[t - nh]) / (W / 2); rtx = (wire[t] - wire[t - nw]) / W
  steady = (np.sign(r1) == np.sign(r2)) & (np.abs(r1) <= 2 * np.abs(r2) + 0.2) & (np.abs(r2) <= 2 * np.abs(r1) + 0.2)
  rw = (ang[t + nd] - ang[t + nd - nw]) / W
  mid = wire[t - nh]; outward = np.sign(rtx) == np.sign(mid)
  m = allok & steady & (np.abs(rtx) >= RB[0])
  # B: 1 s windows
  t1 = np.arange(100, n - nd)
  ok1 = (okc[t1 + nd + 1] - okc[t1 - 100]) == (100 + nd + 1)
  dap = np.abs(ap[t1] - ap[t1 - 100]); dap2 = np.abs(ap[t1 - 50] - ap[t1 - 100]); dtx = wire[t1] - wire[t1 - 100]; dwh = ang[t1 + nd] - ang[t1 + nd - 100]
  mb = ok1 & (dap < 0.3) & (dap2 < 0.3) & (np.abs(dtx) >= 1.0) & (np.sign(dtx) == np.sign(wire[t1 - 50]))
  for i, (lo, hi) in enumerate(VB):
    mm = m & (v[t] >= lo) & (v[t] < hi)
    A[i].append(np.c_[np.abs(rtx[mm]), rw[mm] * np.sign(rtx[mm]), outward[mm]][::5])      # decimate (overlapping windows)
    mm = mb & (v[t1] >= lo) & (v[t1] < hi)
    B[i].append(np.c_[np.abs(dtx[mm]), dwh[mm] * np.sign(dtx[mm])][::20])
print(f"A) wheel rate / TX rate (median), W {W} s, delay {D} s; cells 'out / in (n windows)'")
print("   speed     " + "".join(f"| {a:g}-{b:g} deg/s      " for a, b in zip(RB[:-1], RB[1:])))
for i, (lo, hi) in enumerate(VB):
  X = np.concatenate(A[i]) if A[i] else np.zeros((0, 3)); line = f"   {lo:3d}-{hi:<3d}   "
  for a, b in zip(RB[:-1], RB[1:]):
    m = (X[:, 0] >= a) & (X[:, 0] < b); mo = m & (X[:, 2] > 0); mi = m & (X[:, 2] == 0)
    f = lambda q: f"{np.median(X[q, 1] / X[q, 0]):4.2f}" if q.sum() >= 15 else "  - "
    line += f"| {f(mo)} / {f(mi)} ({m.sum():4d}) "
  print(line)
print("B) command steady (< 0.3 deg / s), TX moving >= 1 deg in 1 s outward (trim-like): wheel change / TX change")
for i, (lo, hi) in enumerate(VB):
  X = np.concatenate(B[i]) if B[i] else np.zeros((0, 2))
  if len(X) < 15:
    print(f"   {lo:3d}-{hi:<3d}   n={len(X)} -"); continue
  r = X[:, 1] / X[:, 0]
  print(f"   {lo:3d}-{hi:<3d}   n={len(X):4d}  TX change p50 {np.median(X[:, 0]):.2f} deg/s | wheel/TX p25 {np.percentile(r, 25):5.2f} p50 {np.median(r):5.2f} p75 {np.percentile(r, 75):5.2f} | wheel moved < 0.2 deg in {np.mean(X[:, 1] < 0.2) * 100:3.0f}%")
