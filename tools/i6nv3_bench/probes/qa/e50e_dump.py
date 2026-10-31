"""E50: HELD-BACK GAP -> SNAP. Gap episode: (TX - wheel) * s >= G deg (s = its sign) with ACI gain >= 0.7, op active >= 15 km/h,
no blinker / LC within 3 s, lasting >= 0.2 s. Holder: the hand if the median driver torque over the episode opposes s with
|tq| >= 50 ('hand'), else 'eps' (the EPS just lags). Close: the first time after the episode start the gap falls below 1 deg,
within 2 s; closing peak wheel rate (toward TX), overshoot beyond TX within 0.5 s after, lateral jerk; snap = peak closing
rate >= 40 deg/s. Then a grab (|tq| >= 250 within 1.5 s after the close) and its direction (with the snap = same sign as s).
Per build: episodes per hour, by holder; snap share by max gap; examples."""
import glob
import sys
from collections import defaultdict
import numpy as np
S = sys.argv[1]; G = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
agg = defaultdict(lambda: dict(h=0.0, ev=[]))
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = np.load(f, allow_pickle=True); b = str(d['commit']); r = f[-12:-4]
  T = d['T']; n = len(T); v = d['v']
  if n < 3000:
    continue
  ang = d['ang']; tx = d['wire']; g = d['gain']; tqs = d['tq_s']; tq = np.abs(tqs); km = d['k_meas']
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  c = lambda m: np.concatenate([[0], np.cumsum(m)]); idx = np.arange(n)
  busy = (c(bl)[np.minimum(idx + 300, n)] - c(bl)[np.maximum(idx - 300, 0)]) > 0
  act = d['lat'] & (v >= 15) & ~busy
  agg[b]['h'] += act.sum() / 360000
  gap = tx - ang
  rate = np.zeros(n); rate[10:] = (ang[10:] - ang[:-10]) / np.maximum(T[10:] - T[:-10], 0.05)
  a = (v / 3.6) ** 2 * km
  big = (np.abs(gap) >= G) & (g >= 0.7) & act
  i = 100
  while i < n - 400:
    if big[i] and big[i:i + 20].all() and np.ptp(np.sign(gap[i:i + 20])) == 0:
      s = np.sign(gap[i]); e = i
      while e < n - 1 and big[e + 1] and np.sign(gap[e + 1]) == s:
        e += 1
      cl = np.flatnonzero(gap[i:i + 200] * s < 1.0)
      if len(cl) == 0:
        i = e + 50; continue
      k = i + int(cl[0])
      mt = float(np.median(tqs[i:k + 1]))
      holder = "hand" if (mt * s <= -50) else "eps"
      w = slice(max(k - 40, i), k + 10)
      pk = float(np.max(rate[w] * s))
      over = float(np.max((ang[k:k + 50] - tx[k:k + 50]) * s))
      jk = float(np.max(np.abs(a[k - 15:k + 35] - a[k - 45:k + 5])) / 0.3)
      gg = tq[k:k + 150] >= 250
      grab = bool(gg.any()); gw = (np.sign(tqs[k + int(np.argmax(gg))]) == s) if grab else None
      agg[b]['ev'].append(dict(r=r, t=T[i], tc=T[k], v=v[i], s=s, gmax=float(np.max(gap[i:k + 1] * s)), dur=(k - i) / 100, holder=holder, mt=mt,
                               pk=pk, over=over, jk=jk, snap=pk >= 40, grab=grab, gw=gw, pre=float(np.median(tq[max(i - 50, 0):max(k - 40, i + 1)])), boost=float(np.median((tx[i:k + 1] - d['apply'][i:k + 1]) * s)), lagc=float(np.median((d['apply'][i:k + 1] - ang[i:k + 1]) * s)), txr=float(np.max(np.abs(np.diff(tx[i:k + 1:10]))) * 10) if k - i >= 20 else np.nan, tcl=float(np.median(tqs[max(k - 40, i):k + 1]) * s), tclmax=float(np.max(tqs[max(k - 40, i):k + 1] * s)), first=(T[i + 20 + int(np.argmax(tq[i + 20:k + 150] >= 250))] - T[k]) if (tq[i + 20:k + 150] >= 250).any() else np.nan))
      i = k + 100
    else:
      i += 1
for b, A in agg.items():
  E = A['ev']; h = A['h']
  print(f"== build {b} gap >= {G} deg under gain >= 0.7: {h * 60:.0f} min | episodes {len(E)} ({len(E) / h:.1f}/h), snaps {sum(e['snap'] for e in E)} ({sum(e['snap'] for e in E) / h:.1f}/h)")
  for hd in ("hand", "eps"):
    es = [e for e in E if e['holder'] == hd]; ss = [e for e in es if e['snap']]
    if not es:
      continue
    print(f"   held by {hd:4}: {len(es):3d} ({len(es) / h:4.1f}/h) max gap p50 {np.median([e['gmax'] for e in es]):4.1f} | snaps {len(ss)} ({len(ss) / h:.1f}/h) "
          f"close rate p50 {np.median([e['pk'] for e in ss]) if ss else 0:3.0f} overshoot p50 {np.median([e['over'] for e in ss]) if ss else 0:+.1f} jerk p50 {np.median([e['jk'] for e in ss]) if ss else 0:.1f} "
          f"| grab after snap {sum(e['grab'] for e in ss)} (with {sum(1 for e in ss if e['gw'])}, against {sum(1 for e in ss if e['grab'] and not e['gw'])})")
  ss = [e for e in E if e['snap']]
  q = [e for e in ss if e['tclmax'] < 100]; p = [e for e in ss if e['tclmax'] >= 100]
  print(f"   snaps with the driver quiet during the close (push toward TX < 100): {len(q)} ({len(q) / h:.1f}/h); driver pushing during the close: {len(p)} ({len(p) / h:.1f}/h)")
  print(f"     quiet-close snaps: grab after {sum(e['grab'] for e in q)} (with {sum(1 for e in q if e['gw'])}, against {sum(1 for e in q if e['grab'] and not e['gw'])}), close rate p50 {np.median([e['pk'] for e in q]) if q else 0:.0f}, jerk p50 {np.median([e['jk'] for e in q]) if q else 0:.1f}")
  print(f"     first big torque (>=250) relative to the close: pushing-close p50 {np.nanmedian([e['first'] for e in p]) if p else 0:+.2f} s; quiet-close p50 {np.nanmedian([e['first'] for e in q]) if q else 0:+.2f} s")
  for e in sorted(q, key=lambda e: -e['pk'])[:8]:
    print(f"       QUIET {e['r']} t {e['t']:7.1f}->{e['tc']:7.1f} v {e['v']:3.0f} gap {e['gmax']:4.1f} close @ {e['pk']:3.0f} over {e['over']:+.1f} jerk {e['jk']:.1f} {('GRAB ' + ('with' if e['gw'] else 'AGAINST')) if e['grab'] else ''}")
  for nm, es in (("all", E), ("snaps", [e for e in E if e['snap']]), ("big >= 8", [e for e in E if e['gmax'] >= 8])):
    print(f"   [{nm}] n {len(es)}: gap split p50 -> carcontroller boost (TX - apply) {np.median([e['boost'] for e in es]):+.1f}, command lead (apply - wheel) {np.median([e['lagc'] for e in es]):+.1f} deg; TX peak rate p50 {np.nanmedian([e['txr'] for e in es]):.0f} deg/s; boost >= 3 deg in {np.mean([e['boost'] >= 3 for e in es]):.0%}")
  for thr in (50, 80, 120):
    ss = [e for e in E if e['snap']]; on = [e for e in ss if e['pre'] >= thr]
    eo = [e for e in E if e['pre'] >= thr]
    print(f"   hand-on (|tq| median before the close >= {thr}): episodes {len(eo)}/{len(E)}, snaps {len(on)}/{len(ss)}; hand-on snaps with max gap >= 8: {sum(e['gmax'] >= 8 for e in on)}")
  for lo, hi in ((G, 8), (8, 12), (12, 1e9)):
    es = [e for e in E if lo <= e['gmax'] < hi]
    print(f"     max gap {lo:.0f}-{hi if hi < 1e8 else 'inf'}: {len(es):3d}, snap {np.mean([e['snap'] for e in es]) if es else 0:.0%}, overshoot p50 {np.median([e['over'] for e in es]) if es else 0:+.1f}")
  for e in sorted([e for e in E if e['snap']], key=lambda e: -e['pk'])[:12]:
    print(f"     {e['r']} t {e['t']:7.1f}->{e['tc']:7.1f} v {e['v']:3.0f} {e['holder']:4} tq {e['mt']:+4.0f} gap {e['gmax']:4.1f} -> close @ {e['pk']:3.0f} deg/s over {e['over']:+4.1f} jerk {e['jk']:.1f} "
          f"{('GRAB ' + ('with' if e['gw'] else 'AGAINST')) if e['grab'] else ''}")

import json
out = [dict(r=e['r'], t=float(e['t']), tc=float(e['tc']), snap=bool(e['snap']), gmax=float(e['gmax']), pre=float(e['pre']), tclmax=float(e['tclmax']))
       for e in agg['98e912d3d']['ev']]
json.dump(out, open(S + "/qa/e50_events.json", "w"))
