"""E120 (report §91): does op lower its own MDPS authority in curves without a driver? Per build and speed band, op active,
no blinker, EPS press flag off: the sent ACI gain and the hold-compensated driver-domain torque, on straights vs in curves
(by lateral acceleration); share of time with gain < 0.9 / < 0.6; and the steady-curve delivery (yaw curvature / model
curvature) split by gain. argv: S builds_csv"""
import glob, sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; BUILDS = sys.argv[2].split(",")
VB = [(30, 40), (40, 55), (55, 70), (70, 90), (90, 200)]; LB = [(0, 0.3, "straight"), (0.3, 0.8, "la .3-.8"), (0.8, 1.5, "la .8-1.5"), (1.5, 6, "la 1.5+")]
for b in BUILDS:
  R = {(i, j): [] for i in range(len(VB)) for j in range(len(LB))}
  for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
    d = dict(np.load(f, allow_pickle=True)); n = len(d['T'])
    if str(d['commit'])[:7] != b or n < 2000: continue
    v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
    hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
    dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
    ok = d['lat'] & ~d['pressed'] & (d['tx_active'] == 2) & ~d['lb'] & ~d['rb']
    km = d['k_model']; kc = d['k_meas']
    for i, (lo, hi) in enumerate(VB):
      for j, (l0, l1, _) in enumerate(LB):
        m = ok & (v >= lo) & (v < hi) & (la >= l0) & (la < l1)
        if m.sum(): R[(i, j)].append(np.c_[d['gain'][m], dd[m], np.abs(d['tq_s'][m]), hold[m], kc[m] * np.sign(km[m]), np.abs(km[m])][::10])
  print(f"=== {b}")
  for i, (lo, hi) in enumerate(VB):
    for j, (_, _, nm) in enumerate(LB):
      if not R[(i, j)]: continue
      X = np.concatenate(R[(i, j)])
      if len(X) < 300: continue
      g = X[:, 0]; big = X[:, 5] >= 0.5e-3
      dl = lambda q: f"{np.median(X[q & big, 4] / X[q & big, 5]):.2f}" if (q & big).sum() >= 100 else " - "
      print(f"  [{lo:3d}-{hi:<3d}] {nm:10s} {len(X)/600:5.1f} min | gain p10/p50 {np.percentile(g,10):.2f}/{np.median(g):.2f}  <0.9 {np.mean(g<0.9)*100:3.0f}%  <0.6 {np.mean(g<0.6)*100:3.0f}% | driver-domain tq p50/p90 {np.median(X[:,1]):3.0f}/{np.percentile(X[:,1],90):3.0f}  raw |tq| p50 {np.median(X[:,2]):3.0f}  hold model p50 {np.median(X[:,3]):3.0f} | delivery gain>=.95 {dl(g>=0.95)}  .6-.95 {dl((g>=0.6)&(g<0.95))}  <.6 {dl(g<0.6)}")
