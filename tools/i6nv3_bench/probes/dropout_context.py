"""Usage: <dir with route_extract npz + lane_<route>.npz>; see ROUTE8_REPORT §39."""
import sys, numpy as np
S = sys.argv[1]
R = list(np.load(f"{S}/dropouts.npy", allow_pickle=True))
cache = {}
for x in R:
  if x['r'] not in cache: cache[x['r']] = np.load(f"{S}/000000{x['r']}.npz", allow_pickle=True)
  d = cache[x['r']]; T = d['T']; a = np.searchsorted(T, x['t'])
  # GPS frozen? (same fix for how long around the start)
  g = np.stack([d['gps_lat'], d['gps_lon']], 1); same = np.all(np.abs(g - g[a]) < 1e-6, 1)
  i0 = a; i1 = a
  while i0 > 0 and same[i0 - 1]: i0 -= 1
  while i1 < len(T) - 1 and same[i1 + 1]: i1 += 1
  frozen = (T[i1] - T[i0])
  pre_press = d['pressed'][max(a - 100, 0):a].mean()      # driver already holding in the 1 s before
  b = a + int(x['dur'] * 100)
  dur_press = d['pressed'][a:b + 1].mean()
  ang_pre = d['ang'][max(a - 100, 0)]; ang_end = d['ang'][min(b + 100, len(T) - 1)]
  x.update(frozen=frozen, pre_press=pre_press, dur_press=dur_press, ang_pre=ang_pre, ang_end=ang_end)
  print(f"{x['r']} t {x['t']:5.0f} v {x['v']:3.0f} dur {x['dur']:.1f} gps-frozen {frozen:5.0f} s | press 1s-before {pre_press:.0%} during {dur_press:.0%} | "
        f"ang -1s {ang_pre:6.1f} start {x['ang']:6.1f} +1s-after {ang_end:6.1f} | kHold {x['k_hold']*1e3:6.2f} kPlan {x['k_plan']*1e3:6.2f} kMeas {x['k_meas']*1e3:6.2f}")
np.save(f"{S}/dropouts.npy", R, allow_pickle=True)
