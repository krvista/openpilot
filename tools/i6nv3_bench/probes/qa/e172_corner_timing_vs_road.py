"""E172 (report §108): when does the wheel turn relative to the ROAD - factory LFA vs openpilot solo, same yardstick.
Road curvature under the car and ahead comes from the model's inner lane lines (quadratic fit of the lane centre over
0-40 m of each 20 Hz frame; independent of who steers). Curve entries: |road curvature at the car| rises through
ENTRY_K with >= 2 s straighter than half of it before, peak >= PEAK_K within 4 s, steered hands-off throughout.
Per entry (t = 0 at the crossing): time of 20 % / 50 % / 80 % of the 3 s change for the request, the wheel, the
measured curvature; the wheel's peak rate; lane position at -1 / 0 / +1 / +2 s (cm toward the OUTSIDE of the curve);
lateral jerk rms. Whole-drive: lag of measured curvature behind road curvature (cross-correlation).
argv: S who:route [...]   who = F (factory LFA active), O (openpilot relayed, hands off) or M (the driver alone)"""
import sys
import numpy as np
S = sys.argv[1]
ENTRY_K, PEAK_K = 0.5e-3, 1.0e-3
BANDS = [(40, 60), (60, 80), (80, 130)]

def frac_time(x, t, f):
  """first time x has covered fraction f of its change between t = -1 and t = +3 s (None if the change is tiny)"""
  a = np.mean(x[(t >= -1.2) & (t <= -0.8)]); b = np.mean(x[(t >= 2.5) & (t <= 3.0)])
  if abs(b - a) < 1e-9: return np.nan
  y = (x - a) / (b - a); k = np.flatnonzero((y >= f) & (t >= -2.0))
  return t[k[0]] if len(k) else np.nan

rows = []
for arg in sys.argv[2:]:
  who, r = arg.split(":")
  d = dict(np.load(f"{S}/r50/{r}.npz", allow_pickle=True)); pl = dict(np.load(f"{S}/la/plan_{r}.npz"))
  T = d['T']; n = len(T); v = d['v']; Tp = pl['T']
  cen = (pl['L1'] + pl['L2']) / 2; X = pl['LX']; prob = np.minimum(pl['PR'][:, 1], pl['PR'][:, 2])
  kroad = np.full(len(Tp), np.nan); kahead = np.full(len(Tp), np.nan)
  for i in range(len(Tp)):
    m = X[i] <= 45.0
    if m.sum() >= 6 and prob[i] > 0.5:
      c = np.polyfit(X[i][m], cen[i][m], 3)
      kroad[i] = 2 * c[1]; kahead[i] = 2 * c[1] + 6 * c[0] * 25.0
  kr = np.interp(T, Tp, np.nan_to_num(kroad)); ok = np.interp(T, Tp, np.isfinite(kroad).astype(float)) > 0.99
  kr = np.convolve(kr, np.ones(40) / 40, 'same')                   # 0.4 s smoothing of the lane fit
  off = np.interp(T, Tp, -(pl['L1'][:, 0] + pl['L2'][:, 0]) / 2)
  pr = d['pressed'].astype(bool); bl = (d['lb'] | d['rb']).astype(bool)
  busy = np.convolve(pr | bl, np.ones(100), 'full')[:n] > 0
  if who == "F": st = (d['lfa_sym'].astype(int) == 2) & (d['cb_lvl2'].astype(int) == 2) & ~busy; reqsig = d['cb_req']
  elif who == "M":                                                 # the driver alone: nobody commands the MDPS, no blinker
    blk = np.convolve(bl, np.ones(100), 'full')[:n] > 0
    st = (d['cb_lvl2'].astype(int) != 2) & (d['mdps_active'].astype(int) != 2) & ~blk; reqsig = d['mdps_ang']
  else: st = (d['tx_active'].astype(int) == 2) & (d['cb_lvl2'].astype(int) == 2) & ~busy; reqsig = d['wire']
  kmeas = d['k_meas']
  # whole-drive lag
  for lo, hi in BANDS:
    m = st & ok & (v >= lo) & (v < hi); e = np.flatnonzero(np.diff(np.r_[0, m.astype(np.int8), 0])); ks = []
    for a, b in zip(e[::2], e[1::2]):
      if b - a >= 800 and np.ptp(kr[a:b]) > 1.5e-3:
        x = kr[a:b] - kr[a:b].mean(); best = (0, -2)
        for k in range(-150, 151, 5):
          y = kmeas[a + max(k, 0):b + min(k, 0)]; xx = x[max(-k, 0):len(x) - max(k, 0)]
          if len(y) == len(xx) and len(y) > 300:
            c = np.corrcoef(xx, y)[0, 1]
            if c > best[1]: best = (k, c)
        if best[1] > 0.8: ks.append(best[0] * 10)
    if ks: print(f"{who} {r} [{lo}-{hi}] measured curvature lags road curvature under the car by p50 {np.median(ks):+.0f} ms (n={len(ks)})")
  # entries
  s = np.sign(kr); ak = np.abs(kr)
  cross = np.flatnonzero((ak[1:] >= ENTRY_K) & (ak[:-1] < ENTRY_K)) + 1
  last = -10**9
  for i in cross:
    if i - last < 500 or i < 300 or i + 400 >= n: continue
    w = slice(i - 300, i + 400); t = (np.arange(700) - 300) / 100.0; sg = s[i]
    if not (st[w].all() and ok[w].all()) or ak[i - 250:i - 50].max() > ENTRY_K * 0.6 or (sg * kr[i:i + 400]).max() < PEAK_K or v[i] < 40: continue
    last = i
    wheel = -sg * d['mdps_ang'][w]; req = -sg * reqsig[w]; km = sg * kmeas[w]; road = sg * kr[w]     # all positive into the curve
    rate = np.gradient(wheel, 0.01); lat = km * (v[w] / 3.6) ** 2; jerk = np.gradient(np.convolve(lat, np.ones(20) / 20, 'same'), 0.01)
    o = -sg * off[w]                                               # + = outside
    rows.append((who, r, v[i], [frac_time(req, t, f) for f in (0.2, 0.5, 0.8)], [frac_time(wheel, t, f) for f in (0.2, 0.5, 0.8)],
                 [frac_time(km, t, f) for f in (0.2, 0.5, 0.8)], [frac_time(road, t, f) for f in (0.2, 0.5, 0.8)],
                 np.max(np.abs(rate[200:600])), [o[200] * 100, o[300] * 100, o[400] * 100, o[500] * 100], np.sqrt(np.mean(jerk[250:550] ** 2)),
                 (sg * kr[i:i + 400]).max()))
print("\nentries: t = 0 when the road curvature under the car passes 0.5e-3; times in s (median); offsets cm toward the outside relative to t = -1 s")
for who in ("F", "O", "M"):
  for lo, hi in BANDS:
    q = [x for x in rows if x[0] == who and lo <= x[2] < hi]
    if len(q) < 4: continue
    med = lambda j: np.nanmedian(np.array([x[j] for x in q]), axis=0)  # noqa: E731
    rq, wh, km, rd = med(3), med(4), med(5), med(6); o = np.array([x[8] for x in q]); o = o - o[:, :1]
    print(f"{who} [{lo}-{hi}] n={len(q):3d} peak road K {np.median([x[10] for x in q])*1e3:.1f}e-3 | road 20/50/80 % {rd[0]:+.2f} {rd[1]:+.2f} {rd[2]:+.2f} | request {rq[0]:+.2f} {rq[1]:+.2f} {rq[2]:+.2f} | wheel {wh[0]:+.2f} {wh[1]:+.2f} {wh[2]:+.2f} | curvature {km[0]:+.2f} {km[1]:+.2f} {km[2]:+.2f} "
          f"| wheel - road at 50 %: {np.nanmedian([x[4][1] - x[6][1] for x in q]):+.2f} s | peak wheel rate {np.median([x[7] for x in q]):.1f} deg/s | jerk rms {np.median([x[9] for x in q]):.2f} m/s3 | offset 0/+1/+2 s {np.median(o[:,1]):+.1f} {np.median(o[:,2]):+.1f} {np.median(o[:,3]):+.1f} cm")
