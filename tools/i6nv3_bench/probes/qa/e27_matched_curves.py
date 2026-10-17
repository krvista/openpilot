"""E27: GPS-matched curve comparison across builds. Curve event = road |k| (lane centre at the car) >= 1.2e-3 for >= 0.5 s,
op active, >= 25 km/h, no blinker/LC +-3 s (hands may be on; hands-off flagged). Keyed by the GPS cell (25 m) of the
curve's first frame and its sign. For every cell driven on >= 2 builds: entry lag (xcorr of |k_ctrl| vs |k_road|,
-2..+3 s), max inside / outside offset during the curve, the share of frames with the Phase 41 cap / 6g blend active
(steerFlags bits 1 / 4) and lane_min p10, the 55 lead flag, max |command rate|. Printed per build, matched cells only,
plus the per-cell table for cells on the newest build."""
import glob
import sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from common import routes

S = sys.argv[1]
NEW = "a0108a47b"
cells = {}
for r, P, d, T, f in routes(S):
  n = len(T); v = f['v']; kr = f['k_road']; b = str(d['commit']); Tr = d['T']
  kc = np.interp(T, Tr, d['k_ctrl']); ap = np.interp(T, Tr, d['apply'])
  la = np.interp(T, Tr, d['gps_lat']); lo = np.interp(T, Tr, d['gps_lon'])
  sf = np.interp(T, Tr, d['sf_ctl'].astype(float)).astype(int)
  pr = P['PR']; lm = np.minimum(pr[:, 1], pr[:, 2])
  good = (pr[:, 1] > 0.5) & (pr[:, 2] > 0.5) & f['lat'] & ~f['busy'] & (v * 3.6 >= 25) & (np.abs(la) > 1)
  ak = np.abs(kr)
  i = 0
  while i < n:
    if ak[i] >= 1.2e-3 and good[i]:
      j = i
      while j + 1 < n and ak[j + 1] >= 0.6e-3 and good[j + 1]:
        j += 1
      if T[j] - T[i] >= 0.5:
        sg = np.sign(np.median(kr[i:j + 1]))
        key = (int(la[i] * 111e3 / 25), int(lo[i] * 88e3 / 25), int(sg))
        plat = np.median(ak[i:j + 1]); half = 0.5 * plat
        a0 = max(i - 40, 0); a1 = min(j + 1, n)
        tr_i = a0 + int(np.argmax(ak[a0:a1] >= half))
        w0 = max(tr_i - 40, 0); w1 = min(tr_i + 60, n)
        x = ak[w0:w1] - ak[w0:w1].mean(); best = None
        for L in range(-30, 31):
          if w0 + L < 0 or w1 + L > n:
            continue
          y = kc[w0 + L:w1 + L] * sg; c_ = np.dot(x, y - y.mean())
          if best is None or c_ > best[0]:
            best = (c_, L)
        lag = -best[1] * 0.05 if best else np.nan
        inside = (-f['c0'][i:j + 1]) * sg
        rate = np.max(np.abs(np.diff(ap[max(i - 20, 0):j + 1]))) / 0.05 if j > i else 0
        cells.setdefault(key, []).append(dict(b=b, r=r, t=T[i], v=v[i] * 3.6, plat=plat, lag=lag, ins=inside.max(), out=-inside.min(),
                                              cap=np.mean((sf[i:j + 1] >> 1) & 1), blend=np.mean((sf[i:j + 1] >> 4) & 1),
                                              lm=np.percentile(lm[i:j + 1], 10), hands=bool(f['hands'][i:j + 1].any()), rate=rate,
                                              lead=np.mean((sf[i:j + 1] >> 9) & 1)))
      i = j + 1
    else:
      i += 1
  print(r, b, flush=True)
matched = {k: ev for k, ev in cells.items() if len({e['b'] for e in ev}) >= 2}
print(f"curve cells {len(cells)}, driven on >= 2 builds {len(matched)}, on the new build {sum(1 for ev in matched.values() if any(e['b'] == NEW for e in ev))}")
agg = {}
for ev in matched.values():
  for e in ev:
    agg.setdefault(e['b'], []).append(e)
print(f"{'build':10} {'n':>4} {'lag ms mean/sd':>15} {'inside max p50':>14} {'outside max p50/p90':>19} {'cap%':>5} {'blend%':>6} {'lm p10':>6} {'rate p90':>8}")
for b, ev in agg.items():
  lag = np.array([e['lag'] for e in ev]); lag = lag[np.isfinite(lag)]
  print(f"{b:10} {len(ev):4d} {np.mean(lag) * 1e3:+7.0f}/{np.std(lag) * 1e3:5.0f}   {np.median([e['ins'] for e in ev]):+.2f}          "
        f"{np.median([e['out'] for e in ev]):+.2f}/{np.percentile([e['out'] for e in ev], 90):+.2f}       {np.mean([e['cap'] for e in ev]):5.1%} {np.mean([e['blend'] for e in ev]):6.1%} "
        f"{np.median([e['lm'] for e in ev]):6.2f} {np.percentile([e['rate'] for e in ev], 90):8.1f}")
# new build vs the previous build at the SAME cells (paired)
pairs = []
for ev in matched.values():
  nw = [e for e in ev if e['b'] == NEW]; ol = [e for e in ev if e['b'] != NEW]
  if nw and ol:
    pairs.append((nw[0], ol))
print(f"\npaired cells (new vs older builds at the same place): {len(pairs)}")
print(" new-route  t      v  plat  | lag new / old (ms) | outside new / old | inside new / old | cap new / old | lm new / old | rate new / old")
for nw, ol in sorted(pairs, key=lambda p: (p[0]['r'], p[0]['t'])):
  o = ol[0]
  print(f" {nw['r']} {nw['t']:6.0f} {nw['v']:3.0f} {nw['plat'] * 1e3:4.1f} | {nw['lag'] * 1e3 if np.isfinite(nw['lag']) else 0:+5.0f} / {o['lag'] * 1e3 if np.isfinite(o['lag']) else 0:+5.0f} | "
        f"{nw['out']:+.2f} / {o['out']:+.2f} | {nw['ins']:+.2f} / {o['ins']:+.2f} | {nw['cap']:4.0%} / {o['cap']:4.0%} | {nw['lm']:.2f} / {o['lm']:.2f} | {nw['rate']:4.1f} / {o['rate']:4.1f} ({o['b'][:4]})")
# new-build cells overall: lag vs cap/blend share
ev = [e for evs in cells.values() for e in evs if e['b'] == NEW]
L = np.array([e['lag'] for e in ev]); C = np.array([e['cap'] + e['blend'] for e in ev]); ok = np.isfinite(L)
print(f"\nnew build all curves n {len(ev)}: lag mean {np.nanmean(L) * 1e3:+.0f} sd {np.nanstd(L) * 1e3:.0f} ms; curves with cap/blend > 20 % of frames: "
      f"{np.mean(C > 0.2):.0%}, their lag {np.nanmean(L[C > 0.2]) * 1e3:+.0f} ms vs {np.nanmean(L[C <= 0.2]) * 1e3:+.0f} ms for the rest")
