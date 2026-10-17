"""E25 (v2): compound corners from the CAR's path. k_car = gyro yaw rate / speed (100 Hz, 0.3 s smoothed). A corner =
|k_car| >= 1.5e-3 for >= 0.8 s; compound = the next corner starts within 4 s of the previous one's end (reversal if the
sign flips). For the SECOND corner, over the window from 1.5 s before its start to its peak:
  hold lag  = cross-correlation lag of the controlsd command k_ctrl vs the model action K (how long controlsd held the
              model back; 0 = passed straight through), and the mean (k_ctrl - K) signed toward the 2nd corner,
  flags     = share of frames with the 41 cap / 6g blend / dropout, lane_min p10,
  55c       = plan-emulated lead weight (0.2 loosening .. 0.5 tightening) averaged over the approach,
  outcome   = max offset toward the OUTSIDE of the 2nd corner in its first 3 s (plan lanes, when both lines > 0.5).
Op active, >= 25 km/h, no blinker/LC +-3 s, hands-off (|tq| < 150) through the approach. Per build."""
import sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from common import routes, T_IDXS

S = sys.argv[1]
EV = {}
for r, P, d, T, f in routes(S):
  n = len(T); v = f['v']; b = str(d['commit']); Tr = d['T']
  kcar = np.interp(T, Tr, d['yaw']) / np.maximum(v, 1.0) * np.pi / 180.0 if np.abs(d['yaw']).max() > 1.0 else np.interp(T, Tr, d['k_meas'])
  kcar = np.convolve(kcar, np.ones(6) / 6, 'same')
  kc = np.interp(T, Tr, d['k_ctrl']); K = P['K'].astype(float)
  sf = np.interp(T, Tr, d['sf_ctl'].astype(float)).astype(int)
  pr = P['PR']; lm = np.minimum(pr[:, 1], pr[:, 2])
  ok = f['lat'] & ~f['busy'] & (v * 3.6 >= 25)
  OR, VX = P['OR'].astype(float), P['VX'].astype(float)
  tau = np.interp(v, [8.0, 13.0, 18.0], [0.30, 0.12, 0.08])
  kt = lambda t: np.array([np.interp(t[i], T_IDXS, OR[i]) / max(np.interp(t[i], T_IDXS, VX[i]), 1.0) for i in range(n)])
  k9 = kt(0.9 + tau); k3 = kt(np.full(n, 0.3))
  tight = (np.sign(k9 - k3) == np.sign(k9)) & (np.abs(k9) > np.abs(k3))
  w_t = np.where(tight, np.interp(np.abs(k9) - np.abs(k3), [0.0, 0.5e-3], [0.2, 0.5]), 0.2)
  ak = np.abs(kcar)
  curves = []; i = 0
  while i < n:
    if ak[i] >= 1.5e-3 and ok[i]:
      j = i
      while j + 1 < n and ak[j + 1] >= 0.8e-3 and ok[j + 1]:
        j += 1
      if T[j] - T[i] >= 0.8:
        curves.append((i, j, np.sign(np.median(kcar[i:j + 1]))))
      i = j + 1
    else:
      i += 1
  evs = EV.setdefault(b, [])
  for (i1, j1, s1), (i2, j2, s2) in zip(curves[:-1], curves[1:]):
    if T[i2] - T[j1] > 4.0:
      continue
    a0 = np.searchsorted(T, T[i2] - 1.5); pk = i2 + int(np.argmax(ak[i2:j2 + 1])); a1 = pk + 1
    if a1 - a0 < 20 or f['hands'][a0:a1].any():
      continue
    x = K[a0:a1] * s2; y = kc[a0:a1] * s2
    if np.std(x) < 3e-4:
      continue
    best = None
    for L in range(0, 31):                      # command lagging the model 0 .. 1.5 s
      xx = x[:len(x) - L] - x[:len(x) - L].mean(); yy = y[L:] - y[L:].mean()
      c_ = np.dot(xx, yy) / max(len(xx), 1)
      if best is None or c_ > best[0]:
        best = (c_, L)
    lag = best[1] * 0.05
    held = np.mean((kc - K)[a0:a1] * s2)
    good = (pr[:, 1] > 0.5) & (pr[:, 2] > 0.5)
    p1 = min(np.searchsorted(T, T[i2] + 3.0), n)
    out = ((-f['c0']) * -s2)[i2:p1][good[i2:p1]]
    evs.append((s1 != s2, lag, held, np.mean((sf[a0:a1] >> 1) & 1), np.mean((sf[a0:a1] >> 4) & 1), np.mean((sf[a0:a1] >> 5) & 1),
                np.percentile(lm[a0:a1], 10), w_t[a0:a1].mean(), out.max() if len(out) else np.nan, v[i2] * 3.6, ak[pk], T[i2] - T[j1], r, T[i2]))
  print(r, b, len(evs), flush=True)
for b, evs in EV.items():
  if not evs:
    continue
  E = np.array([e[:12] for e in evs], dtype=float)
  for nm, m in (("reversal (S)", E[:, 0] == 1), ("same-sign", E[:, 0] == 0)):
    if m.sum() == 0:
      continue
    x = E[m]
    print(f"== {b} {nm:12} n {m.sum():3d} | ctrl hold lag mean {np.mean(x[:, 1]) * 1e3:4.0f} ms p90 {np.percentile(x[:, 1], 90) * 1e3:4.0f} | "
          f"(cmd-model) toward 2nd {np.mean(x[:, 2]) * 1e3:+.2f}e-3 | cap {np.mean(x[:, 3]):4.1%} blend {np.mean(x[:, 4]):4.1%} drop {np.mean(x[:, 5]):4.1%} "
          f"lm p10 {np.median(x[:, 6]):.2f} | 55c w {np.mean(x[:, 7]):.2f} | outside max p50/p90 {np.nanmedian(x[:, 8]):+.2f}/{np.nanpercentile(x[:, 8], 90):+.2f} m | v {np.median(x[:, 9]):.0f} peak {np.median(x[:, 10]) * 1e3:.1f}e-3")
  if b in ("a0108a47b", "8de98262a"):
    for e in evs:
      if e[1] >= 0.4 or (np.isfinite(e[8]) and e[8] > 0.4):
        print(f"     {e[12]} t {e[13]:7.1f} {'S' if e[0] else 'same'} v {e[9]:3.0f} lag {e[1] * 1e3:4.0f} ms held {e[2] * 1e3:+.2f} cap {e[3]:.0%} blend {e[4]:.0%} drop {e[5]:.0%} lm {e[6]:.2f} w {e[7]:.2f} out {e[8]:+.2f}")
