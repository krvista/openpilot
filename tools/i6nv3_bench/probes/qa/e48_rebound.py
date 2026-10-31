"""E48: post-release REBOUND. Release = driver held |tq| >= 150 for >= 0.3 s then < 60 within 0.3 s, op active >= 15 km/h,
no blinker / LC within 3 s, driver quiet (|tq| < 120, not pressed) from the release to +1.0 s (a later grab is allowed after).
held = wheel angle median over the last 0.3 s of the hold. Over release +0.2 .. +2.5 s, while ACI gain >= 0.4:
  excursion = max over time of (wheel - TX) * s, s = sign(held - TX at release) (wheel pushed back toward the held side)
  rebound   = excursion >= 3 deg and the wheel rate toward the held side >= 10 deg/s at that time, with the driver quiet
              at that instant (|tq| < 120 from 0.3 s before it).
Per build: releases, rebound share, size p50/p90, timing after the release, MDPS torque sign at the peak (+ toward held?),
the hold duration and |held - TX|, gain at the peak; whether a grab followed within 1.5 s and in which direction.
Controls: the same metric with s = -sign(...) (excursion to the OTHER side) to see if it is symmetric noise."""
import glob
import sys
from collections import defaultdict
import numpy as np
S = sys.argv[1]
agg = defaultdict(list)
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = np.load(f, allow_pickle=True); b = str(d['commit']); r = f[-12:-4]
  T = d['T']; n = len(T); v = d['v']
  if n < 3000:
    continue
  ang = d['ang']; tx = d['wire']; ap = d['apply']; g = d['gain']; tqs = d['tq_s']; tq = np.abs(tqs); pr = d['pressed']; mt = d['mdps_tq']
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  c = lambda m: np.concatenate([[0], np.cumsum(m)]); idx = np.arange(n)
  busy = (c(bl)[np.minimum(idx + 300, n)] - c(bl)[np.maximum(idx - 300, 0)]) > 0
  act = d['lat'] & (v >= 15) & ~busy
  loud = c((tq >= 120) | pr)
  rate = np.zeros(n); rate[10:] = (ang[10:] - ang[:-10]) / np.maximum(T[10:] - T[:-10], 0.05)
  hi = tq >= 150
  i = 100
  while i < n - 400:
    if hi[i - 1] and not hi[i] and hi[i - 30:i].all():
      k0 = i - 30
      while k0 > 0 and hi[k0 - 1]:
        k0 -= 1
      hold = (i - k0) / 100
      lo = tq[i:i + 30] < 60
      if lo.any():
        j = i + int(np.argmax(lo))
        if act[j - 50:j + 250].all() and loud[j + 100] - loud[j] == 0:
          held = np.median(ang[i - 30:i]); s = np.sign(held - tx[j + 20]) or 1.0
          w = np.arange(j + 20, j + 250)
          ok = g[w] >= 0.4
          # driver quiet at the instant (from 0.3 s before)
          quiet = np.array([loud[k + 1] - loud[k - 30] == 0 for k in w])
          for side, nm in ((s, "toward"), (-s, "away")):
            ex = (ang[w] - tx[w]) * side
            m = ok & quiet & (rate[w] * side >= 10)
            if m.any():
              kk = int(np.argmax(np.where(m, ex, -1e9))); peak = ex[kk]; kabs = w[kk]
            else:
              peak = 0.0; kabs = None
            reb = peak >= 3.0
            gr = tq[kabs:kabs + 150] >= 250 if kabs is not None else np.zeros(1, bool)
            agg[(b, nm)].append(dict(r=r, t=T[j], v=v[j], hold=hold, gap0=abs(held - tx[j + 20]), reb=reb, peak=peak,
                                     dt=(T[kabs] - T[j]) if kabs is not None else np.nan, mt=(mt[kabs] * side) if kabs is not None else np.nan,
                                     g=g[kabs] if kabs is not None else np.nan, grab=bool(reb and gr.any()),
                                     gagainst=bool(reb and gr.any() and np.sign(tqs[kabs + int(np.argmax(gr))]) == -side)))
      i += 100
    else:
      i += 1
for b in sorted({k[0] for k in agg}):
  for nm in ("toward", "away"):
    L = agg[(b, nm)]; R = [x for x in L if x['reb']]
    print(f"== {b} [{nm} the held side] releases {len(L)} | rebound >= 3 deg: {len(R)} ({len(R) / max(len(L), 1):.0%}) "
          + (f"size p50 {np.median([x['peak'] for x in R]):.1f} p90 {np.percentile([x['peak'] for x in R], 90):.1f} deg | at +{np.median([x['dt'] for x in R]):.2f} s "
             f"| MDPS torque pushing that way {sum(1 for x in R if x['mt'] > 0)}/{len(R)} | gain p50 {np.median([x['g'] for x in R]):.2f} | grabbed {sum(x['grab'] for x in R)} "
             f"(against the rebound {sum(x['gagainst'] for x in R)})" if R else ""))
    if nm == "toward" and R:
      H = np.array([x['hold'] for x in L]); G0 = np.array([x['gap0'] for x in L]); RB = np.array([x['reb'] for x in L])
      for lo_, hi_ in ((0, 1), (1, 3), (3, 1e9)):
        m = (H >= lo_) & (H < hi_)
        print(f"     hold {lo_}-{hi_ if hi_ < 1e8 else 'inf'} s: {m.sum():3d} releases, rebound {RB[m].mean() if m.sum() else 0:.0%}")
      for lo_, hi_ in ((0, 2), (2, 5), (5, 1e9)):
        m = (G0 >= lo_) & (G0 < hi_)
        print(f"     |held - TX| {lo_}-{hi_ if hi_ < 1e8 else 'inf'} deg: {m.sum():3d} releases, rebound {RB[m].mean() if m.sum() else 0:.0%}")
      for x in sorted(R, key=lambda x: -x['peak'])[:8]:
        print(f"       {x['r']} t {x['t']:7.1f} v {x['v']:3.0f} hold {x['hold']:4.1f}s |held-TX| {x['gap0']:4.1f} -> rebound {x['peak']:4.1f} deg at +{x['dt']:.2f}s gain {x['g']:.2f} mdps {x['mt']:+.0f} {'GRAB' if x['grab'] else ''}")
