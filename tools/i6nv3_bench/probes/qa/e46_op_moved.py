"""E46: lateral jerk produced by op with nobody steering: jerk = d(v^2 k_meas)/dt over 0.3 s, |jerk| >= J for >= 0.15 s,
op active >= 15 km/h, no blinker / LC within 3 s, |driver tq| < 100 and not pressed over [onset-0.3 s, end], ACI gain >= 0.2
somewhere in the episode (op can move the wheel). Kind: 'regain' if the gain rose by >= 0.2 within 0.7 s before / during the
episode (authority coming back after a hand), else 'steady'. Cause road / correction as E45. Reaction grab: |tq| >= 250
starting >= 0.2 s after the onset and within 2 s. Per build and speed band, rates per op-hour, and the top cases."""
import glob
import sys
from collections import defaultdict
import numpy as np
S = sys.argv[1]; J = float(sys.argv[2]) if len(sys.argv) > 2 else 1.5
agg = defaultdict(lambda: dict(h=defaultdict(float), ev=[]))
BANDS = [(15, 30), (30, 50), (50, 70), (70, 90), (90, 200)]
band = lambda v: next(f"{lo}-{hi}" for lo, hi in BANDS if lo <= v < hi)
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = np.load(f, allow_pickle=True); b = str(d['commit']); r = f[-12:-4]
  T = d['T']; n = len(T); v = d['v']
  if n < 3000:
    continue
  P = np.load(f"{S}/la/plan_{r}.npz")
  g = d['gain']; tqs = d['tq_s']; tq = np.abs(tqs); pr = d['pressed']; km = d['k_meas']; K = d['k_model']; lm = d['lane_min']
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  c = lambda m: np.concatenate([[0], np.cumsum(m)]); idx = np.arange(n)
  busy = (c(bl)[np.minimum(idx + 300, n)] - c(bl)[np.maximum(idx - 300, 0)]) > 0
  act = d['lat'] & (v >= 15) & ~busy
  for lo, hi in BANDS:
    agg[b]['h'][f"{lo}-{hi}"] += (act & (v >= lo) & (v < hi)).sum() / 360000
  a = (v / 3.6) ** 2 * km
  jerk = np.zeros(n); jerk[15:-15] = (a[30:] - a[:-30]) / np.maximum(T[30:] - T[:-30], 0.1)
  hj = (np.abs(jerk) >= J) & act
  loud = c((tq >= 100) | pr)
  def kroad(t):
    j = min(max(int(np.searchsorted(P['T'], t)), 0), len(P['T']) - 1)
    ys = np.interp([10.0, 20.0, 30.0], P['LX'][j], 0.5 * (P['L1'][j] + P['L2'][j]))
    return 2 * (ys[2] - 2 * ys[1] + ys[0]) / 100.0
  i = 100
  while i < n - 250:
    if hj[i] and hj[i:i + 15].all():
      e = i
      while e < n - 1 and hj[e + 1]:
        e += 1
      if loud[e + 1] - loud[i - 30] == 0 and g[i:e + 1].max() >= 0.2:
        regain = (g[i - 70:e + 1].max() - g[i - 70:i + 1].min() >= 0.2) and np.argmax(g[i - 70:e + 1]) > np.argmin(g[i - 70:i + 1])
        dkm = abs(km[min(e + 50, n - 1)] - km[i - 50]); dkr = abs(kroad(T[min(e + 50, n - 1)]) - kroad(T[i - 50]))
        w = tq[i + 20:i + 200] >= 250
        grab = bool(w.any())
        agg[b]['ev'].append(dict(r=r, t=T[i], v=v[i], dur=(e - i) / 100, pk=np.max(np.abs(jerk[i:e + 1])), kind="regain" if regain else "steady",
                                 cause="road" if dkr >= 0.5 * dkm else "correction", grab=grab, dkm=dkm, dkr=dkr, lm=lm[i], g0=g[i - 30], g1=g[e], band=band(v[i])))
      i = e + 100
    else:
      i += 1
for b, A in agg.items():
  E = A['ev']; H = A['h']; h = sum(H.values())
  print(f"== build {b} J >= {J}: {h * 60:.0f} min op >= 15 km/h | op-made jerk episodes {len(E)} ({len(E) / h:.1f}/h), followed by a grab {sum(e['grab'] for e in E)}")
  for bd in H:
    es = [e for e in E if e['band'] == bd]
    if H[bd] < 0.05:
      continue
    f = lambda k, c_=None: [e for e in es if e['kind'] == k and (c_ is None or e['cause'] == c_)]
    print(f"   {bd:7} {H[bd] * 60:4.0f} min: {len(es):3d} ({len(es) / H[bd]:5.1f}/h) | regain {len(f('regain'))} ({len(f('regain')) / H[bd]:4.1f}/h) | steady road {len(f('steady', 'road'))} "
          f"correction {len(f('steady', 'correction'))} ({len(f('steady', 'correction')) / H[bd]:4.1f}/h) | grabbed {sum(e['grab'] for e in es)} | peak p50 {np.median([e['pk'] for e in es]) if es else 0:.1f}")
  for e in sorted(E, key=lambda e: -e['pk'])[:14]:
    print(f"     {e['r']} t {e['t']:7.1f} v {e['v']:3.0f} {e['kind']:6} {e['cause']:10} peak {e['pk']:4.1f} {e['dur']:.2f}s dk_car {e['dkm'] * 1e3:4.1f} road {e['dkr'] * 1e3:4.1f} "
          f"lm {e['lm']:.2f} gain {e['g0']:.2f}->{e['g1']:.2f} {'GRAB' if e['grab'] else ''}")
