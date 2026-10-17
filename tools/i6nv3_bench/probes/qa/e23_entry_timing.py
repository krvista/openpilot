"""E23: curve-entry timing consistency per build. Entry event = lane-centre road curvature |k_road| (0-20 m fit, at the car)
rising from < 0.5e-3 to a plateau >= 1.2e-3 within 2.5 s, op active, both lines > 0.6, >= 30 km/h, no blinker/LC +-3 s.
For each entry: lag = t(command |k_ctrl| crosses 50 % of the plateau) - t(road crosses 50 %), negative = early; the car's
measured curvature lag likewise (k_meas, wheel). Plateau = median |k_road| over the 1.5 s after the rise. Compound = another
entry (either sign) started within 4 s after the previous curve's exit. Per build: n, lag mean / std / p10 / p90, and
split by plateau (shallow < 2.5e-3 / tight), speed, first vs compound-second, 55c lead weight state (plan-emulated)."""
import sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from common import routes, T_IDXS

S = sys.argv[1]
EV = {}
for r, P, d, T, f in routes(S):
  n = len(T); v = f['v']; kr = f['k_road']; b = str(d['commit'])
  Tr = d['T']
  kc = np.interp(T, Tr, d['k_ctrl']); km = np.interp(T, Tr, d['k_meas'])
  pr = P['PR']; good = (pr[:, 1] > 0.6) & (pr[:, 2] > 0.6) & f['lat'] & ~f['busy'] & (v * 3.6 >= 30)
  # 55c weight emulation from the plan
  K = P['K'].astype(float); OR, VX = P['OR'].astype(float), P['VX'].astype(float)
  tau = np.interp(v, [8.0, 13.0, 18.0], [0.30, 0.12, 0.08])
  kt = lambda t: np.array([np.interp(t[i], T_IDXS, OR[i]) / max(np.interp(t[i], T_IDXS, VX[i]), 1.0) for i in range(n)])
  k9 = kt(0.9 + tau); k3 = kt(np.full(n, 0.3))
  w_t = np.interp(np.abs(k9) - np.abs(k3), [0.0, 0.5e-3], [0.2, 0.5])
  gate = np.clip((np.maximum(np.abs(k9), np.abs(k3)) - 0.8e-3) / 0.7e-3, 0, 1)
  ak = np.abs(kr)
  evs = EV.setdefault(b, [])
  i = 0; last_exit_t = -1e9; last_sign = 0
  while i < n - 60:
    if good[i] and ak[i] < 0.5e-3:
      # find rise to >= 1.2e-3 within 2.5 s
      j = i + 1
      while j < n and T[j] - T[i] <= 2.5 and ak[j] < 1.2e-3:
        j += 1
      if j < n and T[j] - T[i] <= 2.5 and ak[j] >= 1.2e-3 and good[i:j + 1].all():
        plat = np.median(ak[j:min(j + 30, n)])
        if plat >= 1.2e-3 and np.sign(kr[j]) != 0:
          sg = np.sign(kr[j]); half = 0.5 * plat
          # road half-crossing
          tr = T[i + np.argmax(ak[i:j + 1] >= half)]
          # command / wheel half-crossing (same sign) searched -2 .. +3 s around tr
          a0 = np.searchsorted(T, tr - 2.0); a1 = np.searchsorted(T, tr + 3.0)
          def xlag(sig):
            x = (ak[a0:a1] - ak[a0:a1].mean()); best = None
            for L in range(-30, 31):          # 20 Hz frames: -1.5 .. +1.5 s
              y = sig[a0 + L:a1 + L] * sg if a0 + L >= 0 and a1 + L <= n else None
              if y is None or len(y) != len(x):
                continue
              c_ = np.dot(x, y - y.mean())
              if best is None or c_ > best[0]:
                best = (c_, L)
            return -best[1] * 0.05 if best else np.nan       # + = command later than the road
          tc = tr + xlag(kc); tm = tr + xlag(km)
          compound = (tr - last_exit_t) < 4.0
          hands = f['hands'][i:j + 1].any()
          evs.append((tc - tr, tm - tr, plat, v[j] * 3.6, compound, sg != last_sign and compound, w_t[i:j + 1].mean(), gate[i:j + 1].mean(), hands,
                      min(pr[i:j + 1, 1].min(), pr[i:j + 1, 2].min()), r, tr))
          # move past this curve: find its exit (|k| < 0.6e-3 for 0.5 s)
          e = j
          while e < n - 10 and not (ak[e:e + 10] < 0.6e-3).all():
            e += 1
          last_exit_t = T[e]; last_sign = sg
          i = e + 1
          continue
      i = j + 1 if j > i else i + 1
    else:
      i += 1
  print(r, b, len(evs), flush=True)

def stats(x):
  x = np.array([a for a in x if np.isfinite(a)])
  return f"n {len(x):3d} mean {np.mean(x) * 1e3 if len(x) else 0:+5.0f} std {np.std(x) * 1e3 if len(x) else 0:4.0f} p10/p90 {np.percentile(x, 10) * 1e3 if len(x) else 0:+5.0f}/{np.percentile(x, 90) * 1e3 if len(x) else 0:+5.0f} ms"

for b, evs in EV.items():
  E = np.array([e[:10] for e in evs], dtype=float)
  if len(E) == 0:
    continue
  print(f"== build {b}: entries {len(E)} (hands-off {int(np.sum(E[:, 8] == 0))})")
  off = E[:, 8] == 0
  print(f"   command lag  all      : {stats(E[:, 0])}")
  print(f"   command lag  hands-off: {stats(E[off, 0])}")
  print(f"   wheel lag    hands-off: {stats(E[off, 1])}")
  for nm, m in (("shallow <2.5e-3", off & (E[:, 2] < 2.5e-3)), ("tight >=2.5e-3", off & (E[:, 2] >= 2.5e-3)),
                ("v<50", off & (E[:, 3] < 50)), ("v>=50", off & (E[:, 3] >= 50)),
                ("first/isolated", off & (E[:, 4] == 0)), ("compound 2nd", off & (E[:, 4] == 1)), ("compound 2nd reversal", off & (E[:, 5] == 1)),
                ("lane_min<0.8", off & (E[:, 9] < 0.8)), ("55c weight<0.35", off & (E[:, 6] < 0.35)), ("55c gate<0.7", off & (E[:, 7] < 0.7))):
    if m.sum() >= 3:
      print(f"     {nm:22}: cmd {stats(E[m, 0])} | wheel {stats(E[m, 1])}")
