"""E127 (report §91): how the wheel falls behind in a corner entry. Hands-off entries (no EPS press and hold-compensated
driver torque < 40 from -1.5 s to +2.5 s, no blinker), aligned at the moment op's command has moved 2 deg outward from
its level 1 s earlier (t = 0) and keeps growing to >= 6 deg more within 2.5 s. Median trajectories of command, wire
request and wheel (all relative to their values at t = -1 s), and the wheel's shortfall behind the command.
argv: S builds_csv"""
import glob, sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; BUILDS = sys.argv[2].split(",")
VB = [(30, 55), (55, 70), (70, 200)]; TS = [-0.5, 0.0, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 2.5]
for b in BUILDS:
  E = {i: [] for i in range(len(VB))}
  for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
    d = dict(np.load(f, allow_pickle=True)); n = len(d['T'])
    if str(d['commit'])[:7] != b or n < 2000: continue
    v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
    hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
    dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
    bad = ~d['lat'] | d['pressed'] | (dd >= 40) | d['lb'] | d['rb'] | (d['tx_active'] != 2)
    bc = np.concatenate([[0], np.cumsum(bad)]); ap, wr, an = d['apply'], d['wire'], d['ang']
    k = 150
    while k < n - 260:
      mv = ap[k] - ap[k - 100]
      if abs(mv) >= 2.0 and abs(ap[k - 1] - ap[k - 101]) < 2.0 and abs(ap[k - 100] - ap[k - 150]) < 1.0:
        s = np.sign(mv)
        if (ap[k + 250] - ap[k - 100]) * s >= 0 and np.max((ap[k:k + 250] - ap[k - 100]) * s) >= 8.0 and bc[k + 251] - bc[k - 150] == 0 \
           and np.sign(ap[k]) == s:
          i = next((q for q, (lo, hi) in enumerate(VB) if lo <= v[k] < hi), None)
          if i is not None:
            row = []
            for t in TS:
              j = k + int(t * 100); row += [(ap[j] - ap[k - 100]) * s, (wr[j] - wr[k - 100]) * s, (an[j] - an[k - 100]) * s]
            E[i].append(row + [(wr[k - 100] - an[k - 100]) * s])
          k += 300; continue
      k += 1
  print(f"=== {b}")
  for i, (lo, hi) in enumerate(VB):
    X = np.array(E[i])
    if len(X) < 4: print(f"  [{lo}-{hi}] n={len(X)}"); continue
    print(f"  [{lo}-{hi}] hands-off entries n={len(X)}  (TX - wheel at t = -1 s: p50 {np.median(X[:, -1]):+.1f} deg)")
    for nm, c in (("command", 0), ("wire TX", 1), ("wheel  ", 2)):
      print("     " + nm + " " + " ".join(f"{t:+.2f}s:{np.median(X[:, 3 * q + c]):5.1f}" for q, t in enumerate(TS)))
    print("     shortfall (cmd - wheel) " + " ".join(f"{t:+.2f}s:{np.median(X[:, 3 * q] - X[:, 3 * q + 2]):5.1f}" for q, t in enumerate(TS)))
    print("     wheel / command         " + " ".join(f"{t:+.2f}s:{np.median(X[:, 3 * q + 2]) / max(np.median(X[:, 3 * q]), 0.1):5.2f}" for q, t in enumerate(TS)))
