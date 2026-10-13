"""HOD touch analysis on route npz (100 Hz, lat active): steeringPressed episodes (duration, peak |tq|, gain drop), light
torque-only touches (|tq| 50-350 without pressed), intervals between touches, and hands-off window yield (10 s / 5 s) under
strict vs 'light touch ignored' rules (touch < TOUCH_S with peak |tq| < TOUCH_NM counts as hands-off)."""
import glob, sys
import numpy as np
S = sys.argv[1]; TOUCH_S, TOUCH_NM = 1.0, 250.0
for f in sorted(glob.glob(f"{S}/0000*.npz")):
  d = np.load(f, allow_pickle=True); r = f[-6:-4]; T = d['T']; lat = d['lat']; pr = d['pressed']; tq = np.abs(d['tq_s']); g = d['gain']; v = d['v']
  e = np.diff(np.concatenate([[0], pr.astype(int), [0]])); st = np.where(e == 1)[0]; en = np.where(e == -1)[0]
  ep = [(T[a], (b - a) / 100, tq[a:b].max(), g[max(a-5,0)], g[a:b].min(), lat[a:b].mean()) for a, b in zip(st, en) if lat[a:b].mean() > 0.5 and v[a] > 20]
  E = np.array(ep) if ep else np.zeros((0, 6))
  print(f"== {r}: lat-active {lat.sum()/6000:.1f} min, pressed episodes (>20 km/h, lat active) {len(E)}")
  if len(E):
    short = E[E[:,1] < TOUCH_S]
    print(f"   duration p25/p50/p75 {np.percentile(E[:,1],25):.2f}/{np.median(E[:,1]):.2f}/{np.percentile(E[:,1],75):.2f} s | peak|tq| p50 {np.median(E[:,2]):.0f} Nm | gain before->min p50 {np.median(E[:,3]):.2f}->{np.median(E[:,4]):.2f}")
    print(f"   short touches (< {TOUCH_S} s): {len(short)} ({len(short)/len(E):.0%}), their peak|tq| p50/p90 {np.median(short[:,2]) if len(short) else 0:.0f}/{np.percentile(short[:,2],90) if len(short) else 0:.0f} Nm, gain min p50 {np.median(short[:,4]) if len(short) else 0:.2f}")
    gaps = np.diff(E[:,0]); print(f"   interval between touch onsets p25/p50/p75 {np.percentile(gaps,25):.1f}/{np.median(gaps):.1f}/{np.percentile(gaps,75):.1f} s (HOD period hint)")
  # light torque-only touches
  lt = (tq > 50) & (tq < 350) & ~pr & lat & (v > 20)
  print(f"   torque-only touch (50-350 Nm, not pressed) share of lat-active >20 km/h: {lt[lat & (v > 20)].mean():.1%}")
  # window yields
  base = lat & ~(d['lb'] | d['rb']) & (d['lcs'] == 0) & (v >= 40)
  kc = d['k_ctrl']
  light_ok = np.ones(len(T), bool)
  for a, b in zip(st, en):
    if (b - a) / 100 >= TOUCH_S or tq[a:b].max() >= TOUCH_NM: light_ok[max(a-100,0):min(b+200,len(T))] = False   # a real grab (+2 s recovery)
  for WIN in (1000, 500):
    n_strict = n_light = 0; i = 0
    while i + WIN <= len(T):
      s = slice(i, i + WIN)
      if base[s].mean() > 0.98 and (kc[s].max() - kc[s].min()) >= 2e-3:
        if pr[s].mean() == 0 and tq[s].mean() < 50 and g[s].mean() >= 0.9: n_strict += 1
        if light_ok[s].all() and g[s].mean() >= 0.7: n_light += 1
      i += WIN // 2
    print(f"   windows {WIN/100:.0f} s (>=40 km/h, excitation): strict hands-off {n_strict} | light-touch-ignored (gain>=0.7) {n_light}")
