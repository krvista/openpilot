"""E18: unsettling motions on the latest-build routes, driver not steering (|tq| < 150, not pressed, +-1 s), op active,
>= 30 km/h, no blinker/LC +-3 s. (1) command steps: |apply| change > 2 deg in 0.3 s; (2) near-line approaches:
centre < 0.85 m from an inner line with both lines > 0.5. For each, the controlsd steerFlags / carcontroller flags in
the preceding 0.5 s and the component that moved: model action K vs controlsd command k_ctrl."""
import sys
import numpy as np
from collections import Counter

S = sys.argv[1]
CN = {0: "LCgate", 1: "LCcap", 2: "LCrel", 3: "LAcap", 4: "blend", 5: "drop", 6: "dep", 7: "bsm", 8: "7c", 9: "orient", 10: "budget"}
cc_names = {8: "drvPressed", 9: "epsPressed", 12: "fastFloor", 13: "tqArmFloor", 16: "boostSupp", 18: "cityRel", 19: "anchRec",
            21: "postGrip", 25: "outrun", 27: "txSat", 28: "hodTouch"}
causes = Counter(); hrs = 0.0; rows = []
for r in sys.argv[2:]:
  import glob; d = np.load((glob.glob(f"{S}/r46/{r}.npz") + glob.glob(f"{S}/r50/{r}.npz"))[0], allow_pickle=True)
  T = d['T']; n = len(T); v = d['v']
  tq = np.abs(d['tq_s']); pr = d['pressed']
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  c = lambda m: np.concatenate([[0], np.cumsum(m)])
  idx = np.arange(n)
  win = lambda m, w: (c(m)[np.minimum(idx + w, n)] - c(m)[np.maximum(idx - w, 0)]) > 0
  drv = win(pr | (tq >= 150), 100); busy = win(bl, 300)
  ok = d['lat'] & (v >= 30) & ~drv & ~busy
  hrs += ok.sum() * 0.01 / 3600
  ap = d['apply']; K = d['k_model']; kc = d['k_ctrl']; sf = d['sf_ctl'].astype(int); sc = d['sf_cc'].astype(int)
  i = 50
  while i < n - 30:
    if ok[i] and ok[i + 30] and abs(ap[i + 30] - ap[i]) > 2.0:
      w = slice(i - 50, i + 30)
      f = int(np.bitwise_or.reduce(sf[w])); g = int(np.bitwise_or.reduce(sc[w]))
      dK = (K[i + 30] - K[i]) * 1e3; dC = (kc[i + 30] - kc[i]) * 1e3
      tag = [CN[b] for b in CN if f >> b & 1 and b != 9] + [cc_names[b] for b in cc_names if g >> b & 1]
      src = "model" if abs(dK) >= 0.6 * abs(dC) else "controlsd"
      key = src + ":" + ("+".join(sorted(set(tag))) if tag else "-")
      causes[key] += 1
      rows.append((abs(ap[i + 30] - ap[i]), r, T[i], v[i], ap[i + 30] - ap[i], dK, dC, d['lane_min'][i], key))
      i += 100
    else:
      i += 1
print(f"latest build, op active hands-off >= 30 km/h: {hrs * 60:.0f} min; command steps > 2 deg / 0.3 s: {sum(causes.values())} "
      f"({sum(causes.values()) / hrs:.0f}/h)")
for k, nn in causes.most_common(14):
  print(f"  {nn:3d}  {k}")
rows.sort(reverse=True)
print("largest:")
for rw in rows[:14]:
  print(f"  {rw[1]} t {rw[2]:7.1f} v {rw[3]:5.1f} d_apply {rw[4]:+5.1f} deg dK {rw[5]:+5.2f} dCmd {rw[6]:+5.2f}e-3 lane_min {rw[7]:.2f} {rw[8]}")
