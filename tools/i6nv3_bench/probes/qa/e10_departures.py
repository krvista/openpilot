"""E10: lane departures on the new routes. From the model lane lines at the car (x = 0): left inner line y L1 (< 0),
right inner line y L2 (> 0) — +y = right. Wheel-on-line when the nearer line is within 0.95 m of the car centre (half
car width), crossing when the centre passes the line (|y| < 0.15 m or the inner pair re-indexes, centre jump > 1.8 m).
Episodes: lat active, op not in a lane change (no blinker within 3 s), >= 20 km/h, both lines > 0.5 just before.
For each: context from the 100 Hz route signals around onset (-3 .. +1 s): speed, lane probs, road curvature, action K,
controlsd command k_ctrl, 55b lead (plan), measured vs command angle, ACI gain, driver torque, steerFlags bits."""
import glob
import sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from common import T_IDXS, bfit, interp_rows

S = sys.argv[1]; RT = sys.argv[2:]
for r in RT:
  P = dict(np.load(f"{S}/la/plan_{r}.npz")); d = dict(np.load(f"{S}/r46/{r}.npz", allow_pickle=True))
  T = P['T']; n = len(T)
  if n == 0:
    continue
  Tr = d['T']
  at = lambda k: np.interp(T, Tr, d[k].astype(float))
  v = at('v'); lat = at('lat') > 0.5
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  c = np.concatenate([[0], np.cumsum(bl)])
  lo = np.searchsorted(Tr, T - 3.0); hi = np.searchsorted(Tr, T + 1.0)
  busy = (c[hi] - c[lo]) > 0
  L1 = np.array([np.interp(0.0, P['LX'][i], P['L1'][i]) for i in range(n)])
  L2 = np.array([np.interp(0.0, P['LX'][i], P['L2'][i]) for i in range(n)])
  pr = P['PR']
  near = np.minimum(-L1, L2)
  side = np.where(-L1 < L2, -1, 1)               # -1: left line nearer
  ok = lat & ~busy & (v >= 20)
  touch = ok & (near < 0.95)
  K = P['K'].astype(float)
  tau = np.interp(v / 3.6, [8.0, 13.0, 18.0], [0.30, 0.12, 0.08])
  OR, VX = P['OR'].astype(float), P['VX'].astype(float)
  kt = lambda t: np.array([np.interp(t[i], T_IDXS, OR[i]) / max(np.interp(t[i], T_IDXS, VX[i]), 1.0) for i in range(n)])
  lead = 0.5 * (kt(0.9 + tau) - kt(np.full(n, 0.3)))
  LC = 0.5 * (P['L1'].astype(float) + P['L2'].astype(float)); LX = P['LX'].astype(float)
  xs0 = np.tile(np.linspace(0, 20, 9), (n, 1))
  kroad = 2.0 * bfit(xs0, interp_rows(xs0, LX, LC), 2)[:, 0]
  # episodes
  i = 0; eps = []
  while i < n:
    if touch[i]:
      j = i
      while j + 1 < n and (touch[j + 1] or (ok[j + 1] and near[j + 1] < 1.05)):
        j += 1
      if T[j] - T[i] >= 0.3:
        eps.append((i, j))
      i = j + 1
    else:
      i += 1
  hrs = ok.sum() / 20 / 3600
  build = str(d['commit'])
  print(f"== {r} build {build} op-active>=20km/h {hrs * 60:.0f} min, wheel-on-line episodes >= 0.3 s: {len(eps)}")
  for (i, j) in eps:
    k = slice(max(i - 60, 0), i + 1)
    m = slice(i, j + 1)
    a0 = np.searchsorted(Tr, T[i] - 3); a1 = np.searchsorted(Tr, T[j])
    sf = np.bitwise_or.reduce(d['sf_ctl'][a0:a1]) if a1 > a0 else 0
    sfc = np.bitwise_or.reduce(d['sf_cc'][a0:a1]) if a1 > a0 else 0
    print(f"  t {T[i]:7.1f}-{T[j]:7.1f} ({T[j]-T[i]:4.1f}s) v {v[i]:5.1f} {'L' if side[i] < 0 else 'R'} min-gap {near[m].min():.2f} m "
          f"lane p {min(pr[i,1],pr[i,2]):.2f}/{max(pr[i,1],pr[i,2]):.2f} k_road {kroad[i]*1e3:+.2f} K {K[i]*1e3:+.2f} "
          f"lead p95|{np.percentile(np.abs(lead[k]),95)*1e3:.2f}| k_ctrl {at('k_ctrl')[i]*1e3:+.2f} "
          f"ang {at('ang')[i]:+.1f} apply {at('apply')[i]:+.1f} gain min {d['gain'][a0:a1].min() if a1>a0 else np.nan:.2f} "
          f"tq max {np.abs(d['tq_s'][a0:a1]).max() if a1>a0 else 0:.0f} sf_ctl {sf:#06x} sf_cc {sfc:#010x}")
