"""E129 (report §91): the wheel's response to a request that starts moving from rest, for all sizes of move. Hands off
(as E117). Onset = the wire request, quiet for 1 s (within +-0.3 deg, wheel within +-0.3 deg), has moved 0.5 deg (t = 0)
and goes on in the same direction. By speed band and by the size of the request move at t = 1 s: median wheel move and
median wheel / TX ratio at 0.25 ... 2.0 s, and the time until the wheel has moved 0.3 deg. For comparison the same for
onsets where the wheel was already moving the same way (>= 0.5 deg in the previous second). argv: S"""
import glob, sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]
VB = [(30, 55), (55, 70), (70, 90), (90, 200)]; SB = [(1.0, 2.5), (2.5, 6.0), (6.0, 30.0)]; TS = [0.25, 0.5, 0.75, 1.0, 1.5, 2.0]
R = {}
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = dict(np.load(f, allow_pickle=True)); n = len(d['T'])
  if n < 2000: continue
  v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  bad = ~d['lat'] | d['pressed'] | (dd >= 40) | d['lb'] | d['rb'] | (d['tx_active'] != 2)
  bc = np.concatenate([[0], np.cumsum(bad)]); wr, an = d['wire'], d['ang']
  k = 110
  while k < n - 210:
    mv = wr[k] - wr[k - 20]
    if abs(mv) >= 0.5 and abs(wr[k - 1] - wr[k - 21]) < 0.5:
      s = np.sign(mv); base = wr[k - 20]
      quiet_tx = np.max(np.abs(wr[k - 120:k - 20] - base)) <= 0.3
      wpre = (an[k - 20] - an[k - 120]) * s
      if quiet_tx and bc[k + 201] - bc[k - 120] == 0:
        size = (wr[k + 100] - base) * s
        if size >= 1.0 and np.min((wr[k:k + 100] - base) * s) >= 0.3:
          i = next((q for q, (lo, hi) in enumerate(VB) if lo <= v[k] < hi), None); j = next((q for q, (lo, hi) in enumerate(SB) if lo <= size < hi), None)
          cls = "rest" if abs(wpre) < 0.3 else ("moving" if wpre >= 0.5 else None)
          if i is not None and j is not None and cls:
            w = (an[k:k + 201] - an[k]) * s; t3 = np.flatnonzero(w >= 0.3)
            R.setdefault((i, j, cls), []).append([(wr[k + int(t * 100)] - base) * s for t in TS] + [w[int(t * 100)] for t in TS] + [t3[0] / 100.0 if len(t3) else 2.5])
        k += 150; continue
    k += 1
for cls in ("rest", "moving"):
  print(f"== request onset, wheel {'at rest' if cls == 'rest' else 'already moving the same way'}")
  for i, (lo, hi) in enumerate(VB):
    for j, (a, b) in enumerate(SB):
      X = np.array(R.get((i, j, cls), []))
      if len(X) < 8: continue
      print(f"  [{lo:3d}-{hi:<3d}] TX move {a:g}-{b:g} deg (n={len(X):3d}): TX p50 at 0.5/1/2 s {np.median(X[:,1]):4.1f}/{np.median(X[:,3]):4.1f}/{np.median(X[:,5]):4.1f} | wheel/TX "
            + " ".join(f"{t:.2f}s:{np.median(X[:, 6 + q]) / max(np.median(X[:, q]), 0.1):4.2f}" for q, t in enumerate(TS)) + f" | wheel +0.3 deg after {np.median(X[:, -1]):.2f} s [{np.percentile(X[:, -1], 25):.2f}-{np.percentile(X[:, -1], 75):.2f}]")
