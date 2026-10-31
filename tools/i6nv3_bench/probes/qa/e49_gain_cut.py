"""E49: abrupt AUTHORITY CUTS. Cut = ACI gain goes from >= 0.7 to <= 0.15 within 0.15 s, op active >= 15 km/h, no blinker / LC
within 3 s. Driver torque at the cut (median over the 0.2 s before) and the flags that fired at the cut. After the cut
(0..0.6 s): wheel peak rate and travel (the power-assisted wheel moving under the hand that op was holding back), its
direction vs the driver torque, and the lateral jerk. 'snap' = travel >= 4 deg at >= 20 deg/s. Per build: cuts per hour,
snap share, torque at the cut; by trigger flag. Also the share of op-active time by |tq| band (how the hand rests)."""
import glob
import sys
from collections import Counter, defaultdict
import numpy as np
S = sys.argv[1]
CCN = {12: "fast", 13: "b13", 14: "b14", 15: "accelY", 16: "boostSup", 17: "blindC", 18: "city39a", 19: "anch35b", 20: "blinkC", 21: "post29",
       22: "b22", 23: "b23", 24: "b24", 25: "outrun", 26: "b26", 27: "txSat", 28: "hod51", 29: "b29", 30: "b30", 0: "c0", 1: "c1", 2: "c2", 3: "c3",
       4: "c4", 5: "c5", 6: "c6", 7: "c7", 8: "c8", 9: "c9", 10: "c10", 11: "c11"}
agg = defaultdict(lambda: dict(h=0.0, ev=[], tqb=np.zeros(4)))
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = np.load(f, allow_pickle=True); b = str(d['commit']); r = f[-12:-4]
  T = d['T']; n = len(T); v = d['v']
  if n < 3000:
    continue
  ang = d['ang']; g = d['gain']; tqs = d['tq_s']; tq = np.abs(tqs); sc = d['sf_cc'].astype(np.int64); km = d['k_meas']
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  c = lambda m: np.concatenate([[0], np.cumsum(m)]); idx = np.arange(n)
  busy = (c(bl)[np.minimum(idx + 300, n)] - c(bl)[np.maximum(idx - 300, 0)]) > 0
  act = d['lat'] & (v >= 15) & ~busy
  agg[b]['h'] += act.sum() / 360000
  agg[b]['tqb'] += np.array([(act & (tq < 50)).sum(), (act & (tq >= 50) & (tq < 100)).sum(), (act & (tq >= 100) & (tq < 200)).sum(), (act & (tq >= 200)).sum()])
  rate = np.zeros(n); rate[10:] = (ang[10:] - ang[:-10]) / np.maximum(T[10:] - T[:-10], 0.05)
  a = (v / 3.6) ** 2 * km
  i = 100
  while i < n - 100:
    if act[i] and g[i] <= 0.15 and g[i - 15:i].max() >= 0.7 and act[i - 50:i + 60].all():
      k = i - 15 + int(np.argmax(g[i - 15:i] >= 0.7))
      t0 = float(np.median(tqs[i - 20:i]))
      w = slice(i, i + 60)
      travel = ang[i + 60] - ang[i]
      pk = np.max(np.abs(rate[w]))
      snap = abs(travel) >= 4 and pk >= 20
      fl = int(np.bitwise_or.reduce(sc[i - 3:i + 3]))
      jk = np.max(np.abs(a[i + 30:i + 60] - a[i:i + 30])) / 0.3
      agg[b]['ev'].append(dict(r=r, t=T[i], v=v[i], tq0=t0, tqmax=tq[i - 20:i + 1].max(), travel=travel, pk=pk, snap=snap,
                               withtq=np.sign(travel) == np.sign(t0), fl=fl, jk=jk))
      i += 100
    else:
      i += 1
for b, A in agg.items():
  E = A['ev']; h = A['h']; S_ = [e for e in E if e['snap']]
  tb = A['tqb'] / A['tqb'].sum()
  print(f"== build {b}: {h * 60:.0f} min op >= 15 km/h | hand |tq| <50 {tb[0]:.0%}, 50-100 {tb[1]:.0%}, 100-200 {tb[2]:.0%}, >=200 {tb[3]:.0%}")
  print(f"   abrupt gain cuts {len(E)} ({len(E) / h:.1f}/h) | followed by a wheel snap {len(S_)} ({len(S_) / h:.1f}/h), snap in the hand's direction {sum(e['withtq'] for e in S_)}")
  for lo, hi in ((0, 150), (150, 250), (250, 1e9)):
    es = [e for e in E if lo <= e['tqmax'] < hi]; ss = [e for e in es if e['snap']]
    print(f"   hand torque at the cut {lo}-{hi if hi < 1e8 else 'inf'}: cuts {len(es)} ({len(es) / h:.1f}/h), snaps {len(ss)} ({len(ss) / max(len(es), 1):.0%}), "
          f"travel p50 {np.median([abs(e['travel']) for e in ss]) if ss else 0:.1f} deg, jerk p50 {np.median([e['jk'] for e in ss]) if ss else 0:.1f}")
  fc = Counter(); fs = Counter()
  for e in E:
    for bb, nm in CCN.items():
      if e['fl'] >> bb & 1:
        fc[nm] += 1; fs[nm] += e['snap']
  print("   flags at the cut (cuts/snaps): " + ", ".join(f"{k} {fc[k]}/{fs[k]}" for k, _ in fc.most_common(10)))
  for e in sorted([e for e in E if e['snap'] and e['tqmax'] < 250], key=lambda e: -abs(e['travel']))[:10]:
    fl = [nm for bb, nm in CCN.items() if e['fl'] >> bb & 1]
    print(f"     {e['r']} t {e['t']:7.1f} v {e['v']:3.0f} tq {e['tq0']:+4.0f} (max {e['tqmax']:3.0f}) -> wheel {e['travel']:+5.1f} deg @ {e['pk']:3.0f} deg/s jerk {e['jk']:.1f} {' '.join(fl)}")
