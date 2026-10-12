"""§33 follow-up on CD210 routes: op-driven (lat active, not pressed, no blinker) curves >= 0.7 m/s^2 at >= 30 km/h:
distance to the INNER inner-lane line by phase (entry / steady / exit, from d|k_meas|/dt) vs straights."""
import glob, sys
import numpy as np
S = sys.argv[1]
acc = {k: [] for k in ("straight", "entry", "steady", "exit")}; sgn_chk = []
for f in sorted(glob.glob(f"{S}/0000*.npz")):
  d = np.load(f, allow_pickle=True); L = np.load(f.replace('/0000', '/lane_0000'))['L']
  T = d['T']; idx = np.clip(np.searchsorted(L[:, 0], T, side='right') - 1, 0, len(L) - 1)
  yl, yr = L[idx, 10], L[idx, 11]; yl30, yr30 = L[idx, 13], L[idx, 14]; pl, pr = L[idx, 2], L[idx, 3]
  v = d['v'] / 3.6; k = d['k_meas']; lacc = v * v * k
  ks = np.convolve(k, np.ones(50) / 50, mode='same'); dk = np.gradient(np.abs(ks)) * 100   # 1/m per s
  ok = d["lat"] & ~d["pressed"] & ~(d['lb'] | d['rb']) & (d['lcs'] == 0) & (d['v'] >= 30) & (np.minimum(pl, pr) > 0.5)
  # hands-off for the surrounding 1 s too
  ho = np.ones(len(ok), bool)
  m = ok & ho
  bend = ((yl30 - yl) + (yr30 - yr)) / 2          # + = lines move right ahead = road bends right
  sgn_chk.append(np.corrcoef(bend[m & (np.abs(lacc) > 0.7)], k[m & (np.abs(lacc) > 0.7)])[0, 1])
  for i in np.where(m)[0][::5]:
    if abs(lacc[i]) < 0.2 and abs(bend[i]) < 0.3: acc["straight"].append((yl[i], yr[i], 0)); continue
    if abs(lacc[i]) < 0.7: continue
    right = bend[i] > 0
    inner = yr[i] if right else -yl[i]             # distance to the inner line (m, positive)
    ph = "entry" if dk[i] > 1e-3 else ("exit" if dk[i] < -1e-3 else "steady")
    acc[ph].append((inner, (-yl[i] if right else yr[i]), 1))
print("corr(road bend right, k_meas) at >=0.7 m/s^2:", np.round(sgn_chk, 2))
st = np.array(acc["straight"]); st_in = np.concatenate([-st[:, 0], st[:, 1]])   # either side on straights
print(f"straight: n {len(st)} ({len(st)*0.05/60:.1f} min) line distance p50 {np.median(st_in):.2f} m, <= p10 {np.percentile(st_in,10):.2f}")
thr = np.percentile(st_in, 10)
for ph in ("entry", "steady", "exit"):
  a = np.array(acc[ph])
  if not len(a): print(ph, "none"); continue
  print(f"{ph:7}: n {len(a)} ({len(a)*0.05/60:.1f} min) inner p50 {np.median(a[:,0]):.2f} m (vs straight {np.median(a[:,0])-np.median(st_in):+.2f}), "
        f"outer p50 {np.median(a[:,1]):.2f}, inner closer than straight p10 ({thr:.2f} m): {np.mean(a[:,0] < thr):.1%} (straight 10%)")
