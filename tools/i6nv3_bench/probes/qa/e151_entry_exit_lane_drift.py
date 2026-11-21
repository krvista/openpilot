"""E151 (report §95): is the car late into a corner, measured on the ROAD instead of against the model's timing - where
does it sit in the lane while the curve builds up and while it ends? Hands off from -1 s to +3 s, no blinker / lane
change within 3 s, both lane lines seen (prob >= 0.6), >= 30 km/h.
  entry: the model action |K| was < 0.5e-3 one second ago, is now >= 1.0e-3 (t = 0) and stays >= 1.0e-3 for 1.5 s
  exit:  |K| was >= 1.5e-3 one second ago (same sign for 1 s), is now <= 0.7e-3 (t = 0) and stays <= 0.7e-3 for 1.5 s
Lane position = lane-centre offset from the model's lane lines at the car, relative to its value at t = -1 s, counted
positive toward the OUTSIDE of the curve. Late into a corner = the car runs to the outside while the curve builds up;
early = it cuts to the inside. At an exit: unwinding late = it ends up on the inside of the curve it leaves.
Per build and speed band: n, median (and 25-75 %) position at 0 / 0.5 / 1.0 / 1.5 / 2.0 / 3.0 s and the peak outside
position within 3 s. argv: S builds_csv"""
import glob
import sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; BUILDS = sys.argv[2].split(",")
BANDS = [(30, 40), (40, 55), (55, 70), (70, 200)]; TS = [0, 50, 100, 150, 200, 300]
for b in BUILDS:
  E = {i: [] for i in range(len(BANDS))}; X = {i: [] for i in range(len(BANDS))}; PKE = {i: [] for i in range(len(BANDS))}
  for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
    d = dict(np.load(f, allow_pickle=True)); n = len(d['T']); r = f[-12:-4]
    if str(d['commit'])[:7] != b or n < 6000: continue
    try:
      P = dict(np.load(f"{S}/la/plan_{r}.npz"))
    except Exception:  # noqa: BLE001
      continue
    T = d['T']; v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
    hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
    dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
    bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
    cs = np.concatenate([[0], np.cumsum(bl)]); idx = np.arange(n)
    busy = (cs[np.minimum(idx + 300, n)] - cs[np.maximum(idx - 300, 0)]) > 0
    hand = (dd >= 40) | d['pressed'].astype(bool)
    j = np.clip(np.searchsorted(P['T'], T), 0, len(P['T']) - 1)
    l1 = np.array([np.interp(0.0, x, y) for x, y in zip(P['LX'], P['L1'])])[j]
    l2 = np.array([np.interp(0.0, x, y) for x, y in zip(P['LX'], P['L2'])])[j]
    pr = np.minimum(P['PR'][:, 1], P['PR'][:, 2])[j]
    offset = -(l1 + l2) / 2            # + = the car is left of the lane centre
    good = d['lat'].astype(bool) & ~busy & ~hand & (pr >= 0.6) & (v >= 30)
    K = d['k_model']; aK = np.abs(K)
    k = 150
    while k < n - 320:
      if good[k - 100:k + 300].all():
        bi = next((i for i, (lo, hi) in enumerate(BANDS) if lo <= v[k] < hi), None)
        if bi is not None and aK[k] >= 1.0e-3 and aK[k - 1] < 1.0e-3 and aK[k - 100] < 0.5e-3 and (aK[k:k + 150] >= 1.0e-3).all():
          s = np.sign(K[k])                                   # + = left curve: outside is the right = offset going negative
          out = -(offset[k - 100:k + 301] - offset[k - 100]) * s
          E[bi].append([out[100 + t] for t in TS]); PKE[bi].append(out[100:].max()); k += 300; continue
        if (bi is not None and aK[k] <= 0.7e-3 and aK[k - 1] > 0.7e-3 and aK[k - 100] >= 1.5e-3
            and np.all(np.sign(K[k - 200:k - 100]) == np.sign(K[k - 100])) and (aK[k:k + 150] <= 0.7e-3).all()):
          s = np.sign(K[k - 100])
          out = -(offset[k - 100:k + 301] - offset[k - 100]) * s
          X[bi].append([out[100 + t] for t in TS]); k += 300; continue
      k += 1
  print(f"=== {b}   lane position relative to t = -1 s, + = toward the outside of the curve (cm): median [25-75 %] at " + " / ".join(f"{t / 100:g}" for t in TS) + " s")
  for i, (lo, hi) in enumerate(BANDS):
    for nm, D in (("entry", E[i]), ("exit ", X[i])):
      if len(D) < 3:
        print(f"  [{lo}-{hi}] {nm} n={len(D)}"); continue
      A = np.array(D) * 100
      line = f"  [{lo}-{hi}] {nm} n={len(D):3d}: " + "  ".join(f"{np.median(A[:, q]):+5.1f} [{np.percentile(A[:, q], 25):+.0f} {np.percentile(A[:, q], 75):+.0f}]" for q in range(len(TS)))
      if nm == "entry":
        line += f"  | peak outside {np.median(np.array(PKE[i]) * 100):.1f} cm"
      print(line)
