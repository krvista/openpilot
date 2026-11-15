"""E132 (report §91): the mirror of E127 for corner exits. Hands-off exits (no press, driver-domain torque < 40, no
blinker from -1.5 to +2.5 s): op's command, steady in a curve (|command| >= 5 deg, moved < 1 deg in the previous 0.5 s),
has come back 2 deg toward centre within 1 s (t = 0) and unwinds >= 5 deg in total within 2.5 s. Median unwinding of
command, wire request and wheel (positive = toward centre, relative to t = -1 s), how much less the wheel has unwound than
the command, and the ABSOLUTE wheel - command difference (the wheel starts the exit SHORT of the command, so unwinding
less does not by itself put it beyond the command). All builds pooled. argv: S"""
import glob, sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]
VB = [(30, 55), (55, 70), (70, 200)]; TS = [-0.5, 0.0, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 2.5]
E = {i: [] for i in range(len(VB))}
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = dict(np.load(f, allow_pickle=True)); n = len(d['T'])
  if n < 2000: continue
  v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  bad = ~d['lat'] | d['pressed'] | (dd >= 40) | d['lb'] | d['rb'] | (d['tx_active'] != 2)
  bc = np.concatenate([[0], np.cumsum(bad)]); ap, wr, an = d['apply'], d['wire'], d['ang']
  k = 150
  while k < n - 260:
    s = np.sign(ap[k - 100])
    back = (ap[k - 100] - ap[k]) * s
    if abs(ap[k - 100]) >= 5.0 and back >= 2.0 and (ap[k - 101] - ap[k - 1]) * s < 2.0 and abs(ap[k - 100] - ap[k - 150]) < 1.0:
      if np.max((ap[k - 100] - ap[k:k + 250]) * s) >= 5.0 and bc[k + 251] - bc[k - 150] == 0:
        i = next((q for q, (lo, hi) in enumerate(VB) if lo <= v[k] < hi), None)
        if i is not None:
          row = []
          for t in TS:
            j = k + int(t * 100); row += [(ap[k - 100] - ap[j]) * s, (wr[k - 100] - wr[j]) * s, (an[k - 100] - an[j]) * s]
          E[i].append(row + [(an[k + int(t * 100)] - ap[k + int(t * 100)]) * s for t in TS] + [(wr[k - 100] - an[k - 100]) * s, abs(ap[k - 100])])
        k += 300; continue
    k += 1
for i, (lo, hi) in enumerate(VB):
  X = np.array(E[i])
  if len(X) < 4: print(f"[{lo}-{hi}] n={len(X)}"); continue
  print(f"[{lo}-{hi}] hands-off exits n={len(X)}  (|command| before {np.median(X[:, -1]):.1f} deg, TX - wheel before {np.median(X[:, -2]):+.1f} deg)")
  for nm, c in (("command unwound", 0), ("wire TX unwound", 1), ("wheel unwound  ", 2)):
    print("     " + nm + " " + " ".join(f"{t:+.2f}s:{np.median(X[:, 3 * q + c]):5.1f}" for q, t in enumerate(TS)))
  print("     unwound less than cmd  " + " ".join(f"{t:+.2f}s:{np.median(X[:, 3 * q] - X[:, 3 * q + 2]):5.1f}" for q, t in enumerate(TS)))
  nT = 3 * len(TS)
  print("     ABSOLUTE wheel - command (> 0: wheel more turned than the command) " + " ".join(f"{t:+.2f}s:{np.median(X[:, nT + q]):5.1f}" for q, t in enumerate(TS))
        + f" | wheel more turned than the command by >= 1 deg at +1 s in {np.mean(X[:, nT + 5] >= 1.0) * 100:.0f} %, at +2 s in {np.mean(X[:, nT + 7] >= 1.0) * 100:.0f} %")
