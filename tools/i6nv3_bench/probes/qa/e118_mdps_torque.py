"""E118 (report §91): is the wheel's shortfall behind the request a torque limit? Hands off (as E117), quasi-steady
(TX and wheel each moved < 0.3 deg over the last 1 s). By speed band: MDPS output torque (0xEA) and torsion-bar torque vs
the TX - wheel gap and vs lateral acceleration; the wheel / TX ratio vs lateral acceleration. argv: S"""
import glob, sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]
VB = [(30, 40), (40, 55), (55, 70), (70, 90), (90, 200)]
GB = [0, 0.5, 1, 2, 3, 5, 8, 20]; LB = [0, 0.2, 0.5, 1.0, 1.5, 2.0, 3.0, 6.0]
R = {i: [] for i in range(len(VB))}
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = dict(np.load(f, allow_pickle=True)); n = len(d['T'])
  if n < 2000: continue
  v = d['v']; vm = v / 3.6; lat = vm ** 2 * d['k_meas']; la = np.abs(lat)
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  ok = d['lat'] & (dd < 40) & ~d['pressed'] & (d['gain'] >= 0.95) & (d['tx_active'] == 2) & ~d['lb'] & ~d['rb']
  wire, ang = d['wire'], d['ang']
  st = np.zeros(n, bool); st[100:] = (np.abs(wire[100:] - wire[:-100]) < 0.3) & (np.abs(ang[100:] - ang[:-100]) < 0.3) & (np.abs(wire[100:] - wire[50:-50]) < 0.3)
  okc = np.concatenate([[0], np.cumsum(ok)]); t = np.arange(100, n); allok = np.zeros(n, bool); allok[100:] = (okc[t + 1] - okc[t - 100]) == 101
  m = allok & st
  s = np.sign(wire)
  for i, (lo, hi) in enumerate(VB):
    mm = m & (v >= lo) & (v < hi)
    R[i].append(np.c_[(wire[mm] - ang[mm]) * s[mm], d['mdps_tq'][mm] * s[mm], d['tq_s'][mm] * s[mm], la[mm], np.abs(wire[mm]), np.abs(ang[mm]) * np.sign(ang[mm] * s[mm]), d['tq_eps'][mm] * s[mm]][::25])
for i, (lo, hi) in enumerate(VB):
  X = np.concatenate(R[i]); print(f"[{lo}-{hi}] steady hands-off samples {len(X)} (decimated 4 Hz): corr(mdps_tq, gap) {np.corrcoef(X[:,0], X[:,1])[0,1]:.2f}  corr(mdps_tq, lat acc) {np.corrcoef(X[:,3], X[:,1])[0,1]:.2f}  |mdps_tq| p50/p90/p99 {np.percentile(np.abs(X[:,1]),50):.1f}/{np.percentile(np.abs(X[:,1]),90):.1f}/{np.percentile(np.abs(X[:,1]),99):.1f}  tq_eps p99 {np.percentile(np.abs(X[:,6]),99):.0f}")
  line = "   by gap (TX-wheel, outward +):   "
  for a, b in zip(GB[:-1], GB[1:]):
    q = (X[:, 0] >= a) & (X[:, 0] < b)
    line += f"| {a:g}-{b:g}: tq {np.median(X[q,1]):5.1f} tb {np.median(X[q,2]):4.0f} la {np.median(X[q,3]):.2f} n{q.sum():4d} " if q.sum() >= 20 else f"| {a:g}-{b:g}: - "
  print(line)
  line = "   by lat acc (m/s2):              "
  for a, b in zip(LB[:-1], LB[1:]):
    q = (X[:, 3] >= a) & (X[:, 3] < b) & (X[:, 4] >= 1.0)
    line += f"| {a:g}-{b:g}: tq {np.median(X[q,1]):5.1f} wheel/TX {np.median(X[q,5]/X[q,4]):.2f} gap {np.median(X[q,0]):4.1f} n{q.sum():4d} " if q.sum() >= 20 else f"| {a:g}-{b:g}: - "
  print(line)
