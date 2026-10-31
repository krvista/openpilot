"""E45b (relaxed: any gain, only |tq|<150 at onset): felt lateral jerk under op authority. a_lat = v^2 * k_meas (from the wheel); jerk = (a(t+0.15)-a(t-0.15))/0.3.
Episode: |jerk| >= J for >= 0.2 s, op active >= 30 km/h, no blinker / LC within 3 s, ACI gain >= 0.5 at the onset, driver
quiet (|tq| < 150, not pressed) for 0.5 s before. Cause: road-driven if the plan-lane centre curvature (10-30 m) changed by
>= 50 % of the car's curvature change over the episode +-0.5 s, else 'correction / model swing'. Grab = |tq| >= 250 within
1.5 s; WITH / AGAINST the jerk direction. Rates per op-hour, by speed band; and examples."""
import glob
import sys
from collections import defaultdict
import numpy as np
S = sys.argv[1]; J = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
agg = defaultdict(lambda: dict(h=defaultdict(float), ev=[]))
BANDS = [(30, 50), (50, 70), (70, 90), (90, 200)]
band = lambda v: next(f"{lo}-{hi}" for lo, hi in BANDS if lo <= v < hi)
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = np.load(f, allow_pickle=True); b = str(d['commit']); r = f[-12:-4]
  T = d['T']; n = len(T); v = d['v']
  if n < 3000:
    continue
  P = np.load(f"{S}/la/plan_{r}.npz")
  g = d['gain']; tqs = d['tq_s']; tq = np.abs(tqs); pr = d['pressed']; km = d['k_meas']; kc = d['k_ctrl']; K = d['k_model']; lm = d['lane_min']
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  c = lambda m: np.concatenate([[0], np.cumsum(m)]); idx = np.arange(n)
  busy = (c(bl)[np.minimum(idx + 300, n)] - c(bl)[np.maximum(idx - 300, 0)]) > 0
  act = d['lat'] & (v >= 30) & ~busy
  for lo, hi in BANDS:
    agg[b]['h'][f"{lo}-{hi}"] += (act & (v >= lo) & (v < hi)).sum() / 360000
  vv = v / 3.6; a = vv ** 2 * km
  # 100 Hz nominal; use index offsets
  jerk = np.zeros(n); jerk[15:-15] = (a[30:] - a[:-30]) / np.maximum(T[30:] - T[:-30], 0.1)
  hj = (np.abs(jerk) >= J) & act
  quiet = c((tq >= 150) | pr)
  def kroad(t):
    j = min(max(int(np.searchsorted(P['T'], t)), 0), len(P['T']) - 1)
    ys = np.interp([10.0, 20.0, 30.0], P['LX'][j], 0.5 * (P['L1'][j] + P['L2'][j]))
    return 2 * (ys[2] - 2 * ys[1] + ys[0]) / 100.0
  i = 100
  while i < n - 200:
    if hj[i] and hj[i:i + 20].all() and tq[i] < 150:
      e = i
      while e < n - 1 and hj[e + 1]:
        e += 1
      sgn = np.sign(jerk[i])
      dkm = abs(km[min(e + 50, n - 1)] - km[i - 50]); dkr = abs(kroad(T[min(e + 50, n - 1)]) - kroad(T[i - 50]))
      ddK = abs(K[e] - K[i - 50])
      grab = bool((tq[i:i + 150] >= 250).any())
      gk = i + int(np.argmax(tq[i:i + 150] >= 250)) if grab else None
      # angle + = left = curvature +? k_meas sign: angle = -curvature in this fork -> torque + (left) pushes curvature -.
      gwith = (np.sign(-tqs[gk]) == sgn) if grab else None
      cause = "road" if dkr >= 0.5 * dkm else "correction"
      agg[b]['ev'].append(dict(r=r, t=T[i], v=v[i], dur=(e - i) / 100, pk=np.max(np.abs(jerk[i:e + 1])), cause=cause, grab=grab, gwith=gwith,
                               dkm=dkm, dkr=dkr, ddK=ddK, lm=lm[i], g=g[i], band=band(v[i])))
      i = e + 100
    else:
      i += 1
for b, A in agg.items():
  E = A['ev']; H = A['h']; h = sum(H.values())
  G = [e for e in E if e['grab']]
  print(f"== build {b} J >= {J} m/s3: {h * 60:.0f} min | episodes {len(E)} ({len(E) / h:.1f}/h), grabbed {len(G)} ({len(G) / h:.1f}/h; with {sum(1 for e in G if e['gwith'])}, against {sum(1 for e in G if e['gwith'] is False)})")
  for bd in H:
    es = [e for e in E if e['band'] == bd]
    if H[bd] < 0.05:
      continue
    rd = [e for e in es if e['cause'] == 'road']; co = [e for e in es if e['cause'] != 'road']
    print(f"   {bd:7} km/h {H[bd] * 60:4.0f} min: {len(es):3d} ({len(es) / H[bd]:5.1f}/h) road {len(rd)} / correction {len(co)} ({len(co) / H[bd]:4.1f}/h) | grabbed {sum(e['grab'] for e in es)} "
          f"(correction {sum(e['grab'] for e in co)}) | peak p50 {np.median([e['pk'] for e in es]) if es else 0:.1f}")
  for e in sorted([e for e in E], key=lambda e: -e["pk"])[:16]:
    print(f"     {e['r']} t {e['t']:7.1f} v {e['v']:3.0f} peak {e['pk']:4.1f} m/s3 {e['dur']:.2f}s dk_car {e['dkm'] * 1e3:4.1f} d_road {e['dkr'] * 1e3:4.1f} dK {e['ddK'] * 1e3:4.1f} lm {e['lm']:.2f} gain {e['g']:.2f} {e['cause']} "
          f"{'GRAB ' + ('with' if e['gwith'] else 'AGAINST') if e['grab'] else ''}")
