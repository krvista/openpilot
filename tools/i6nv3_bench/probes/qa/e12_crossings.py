"""E12: real crossings (car centre within 0.5 m of an inner line = ~0.45 m of the body over it), lat active, >= 25 km/h,
|wheel angle| < 30 deg at onset, no blinker/LC within 3 s. Approach = back from onset to the last frame with the
centre > 1.1 m from that line. Who initiated: driver torque toward the line (+ = toward, tq_s > 0 = left) and
EPS pressed during the approach vs op command toward the line: k_ctrl, action K, k_ctrl - K (signed toward the line),
and the 55b lead itself from the plan (0.5 (k_or(0.9+tau) - k_or(0.3))) — all averaged over the approach; ACI gain."""
import glob
import sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from common import T_IDXS

S = sys.argv[1]
for r in sys.argv[2:]:
  P = dict(np.load(f"{S}/la/plan_{r}.npz"))
  rf = (glob.glob(f"{S}/ropm/{r}.npz") + glob.glob(f"{S}/r43/{r}.npz") + glob.glob(f"{S}/r46/{r}.npz"))[0]
  d = dict(np.load(rf, allow_pickle=True))
  T = P['T']; n = len(T); Tr = d['T']
  at = lambda k: np.interp(T, Tr, d[k].astype(float))
  v = at('v'); lat = at('lat') > 0.5; ang = at('ang'); kc = at('k_ctrl'); tq = at('tq_s'); pr_ = at('pressed') > 0.3; gain = at('gain')
  K = P['K'].astype(float)
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  cb = np.concatenate([[0], np.cumsum(bl)])
  busy = (cb[np.searchsorted(Tr, T + 1.0)] - cb[np.searchsorted(Tr, T - 3.0)]) > 0
  L1 = np.array([np.interp(0.0, P['LX'][i], P['L1'][i]) for i in range(n)])
  L2 = np.array([np.interp(0.0, P['LX'][i], P['L2'][i]) for i in range(n)])
  pmin = np.minimum(P['PR'][:, 1], P['PR'][:, 2])
  tau = np.interp(v / 3.6, [8.0, 13.0, 18.0], [0.30, 0.12, 0.08])
  OR, VX = P['OR'].astype(float), P['VX'].astype(float)
  kt = lambda t: np.array([np.interp(t[i], T_IDXS, OR[i]) / max(np.interp(t[i], T_IDXS, VX[i]), 1.0) for i in range(n)])
  lead = 0.5 * (kt(0.9 + tau) - kt(np.full(n, 0.3)))
  sf = np.interp(T, Tr, d['sf_ctl'].astype(float)).astype(int)
  ok = lat & ~busy & (v >= 25)
  out = []
  for sd, dist in ((-1, -L1), (1, L2)):            # -1 left line (k < 0 toward it), +1 right line
    i = 0
    while i < n:
      if ok[i] and dist[i] < 0.5 and abs(ang[i]) < 30:
        j = i
        while j + 1 < n and dist[j + 1] < 0.5:
          j += 1
        a = i
        while a > 0 and dist[a - 1] <= 1.1 and T[i] - T[a - 1] < 6:
          a -= 1
        w = slice(a, i + 1)
        tq_tow = -sd * tq[w]                          # tq_s > 0 = left = toward the left line (sd = -1)
        out.append((T[a], T[i], T[j], v[i], sd, dist[i:j + 1].min(), pmin[a], np.max(tq_tow), np.min(tq_tow), pr_[w].any(),
                    np.mean(kc[w]) * sd * 1e3, np.mean(K[w]) * sd * 1e3, np.mean((kc - K)[w]) * sd * 1e3,
                    np.mean(lead[w]) * sd * 1e3, np.min(gain[w]), int(np.bitwise_or.reduce(sf[w])), np.mean(pmin[w])))
        i = j + 1
      else:
        i += 1
  out.sort()
  print(f"== {r} {str(d['commit'])}: crossings {len(out)}")
  for o in out:
    who = ("DRIVER" if o[7] > 200 else "op" if o[7] < 120 and not o[9] else "mixed")
    print(f"  approach {o[0]:7.1f} -> line {o[1]:7.1f} (+{o[2]-o[1]:.1f}s) v {o[3]:5.1f} {'R' if o[4] > 0 else 'L'} gap {o[5]:.2f} "
          f"lane_p {o[6]:.2f}/{o[16]:.2f} drv tq toward max {o[7]:+4.0f} away {o[8]:+4.0f} pressed {int(o[9])} | toward line: "
          f"k_ctrl {o[10]:+.2f} K {o[11]:+.2f} added {o[12]:+.2f} lead {o[13]:+.2f} e-3 | gain min {o[14]:.2f} sf {o[15]:#06x} -> {who}")
