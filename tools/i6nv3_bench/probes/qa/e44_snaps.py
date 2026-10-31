"""E44: fast wheel motions while op HAS authority. Onset: |wheel rate| >= 30 deg/s (0.1 s diff), ACI gain >= 0.5 over the
0.3 s before, op active >= 30 km/h, no blinker / LC within 3 s, |driver tq| < 150 in the 0.3 s before the onset.
Split by what drives it:
  gap-close : |TX - wheel| >= 5 deg at 0.3 s before the onset and the TX itself moved < 50 % of the wheel move
              (the wheel was held back — hand / EPS saturation — and then caught up)
  command   : the TX moved >= 50 % of the wheel move (the command itself was fast)
For 'command': lateral jerk of the controlsd command v^2*|dk|/dt (over 0.3 s), whether the road (plan lanes) changed
curvature as much (road-driven) or not (correction / model swing), lane prob. Grab after: |tq| >= 250 within 1.5 s.
Also the hold-back phase before gap-close: driver torque level and sign vs TX direction (resisting?)."""
import glob
import sys
from collections import defaultdict
import numpy as np
S = sys.argv[1]
agg = defaultdict(lambda: dict(h=0.0, ev=[]))
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = np.load(f, allow_pickle=True); b = str(d['commit']); r = f[-12:-4]
  T = d['T']; n = len(T); v = d['v']
  if n < 3000:
    continue
  P = np.load(f"{S}/la/plan_{r}.npz")
  ang = d['ang']; wire = d['wire']; g = d['gain']; tqs = d['tq_s']; tq = np.abs(tqs); kc = d['k_ctrl']; K = d['k_model']; lm = d['lane_min']
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  c = lambda m: np.concatenate([[0], np.cumsum(m)]); idx = np.arange(n)
  busy = (c(bl)[np.minimum(idx + 300, n)] - c(bl)[np.maximum(idx - 300, 0)]) > 0
  act = d['lat'] & (v >= 30) & ~busy
  agg[b]['h'] += act.sum() / 360000
  rate = np.zeros(n); rate[10:] = (ang[10:] - ang[:-10]) / np.maximum(T[10:] - T[:-10], 0.05)
  def kroad(t):
    j = min(max(int(np.searchsorted(P['T'], t)), 0), len(P['T']) - 1)
    ys = np.interp([10.0, 20.0, 30.0], P['LX'][j], 0.5 * (P['L1'][j] + P['L2'][j]))
    return 2 * (ys[2] - 2 * ys[1] + ys[0]) / 100.0    # same sign convention as K (checked on route 65)
  i = 300
  while i < n - 160:
    if act[i] and abs(rate[i]) >= 30 and act[i - 300:i + 30].all() and (g[i - 30:i] >= 0.5).all() and (tq[i - 30:i] < 150).all():
      a0 = i - 10; a1 = i + 30; sgn = np.sign(ang[a1] - ang[a0]); dang = abs(ang[a1] - ang[a0])
      gap = (wire[i - 30] - ang[i - 30]) * sgn
      dwire = (wire[a1] - wire[a0]) * sgn
      grab = bool((tq[i:i + 150] >= 250).any())
      gk = i + int(np.argmax(tq[i:i + 150] >= 250)) if grab else None
      vv = v[i] / 3.6
      jerk = vv ** 2 * abs(kc[i + 20] - kc[i - 10]) / 0.3
      dkr = abs(kroad(T[i + 20]) - kroad(T[i - 30])); dK = abs(K[i + 20] - K[i - 30])
      hold_tq = np.median(tqs[i - 50:i]) * sgn       # < 0: hand resisting the direction the wheel then snapped
      kind = "gap-close" if (gap >= 5 and dwire < 0.5 * dang) else "command"
      agg[b]['ev'].append(dict(r=r, t=T[i], v=v[i], kind=kind, dang=dang, rate=np.max(np.abs(rate[a0:a1])), gap=gap, jerk=jerk, dkr=dkr, dK=dK,
                               lm=lm[i], grab=grab, gwith=(np.sign(tqs[gk]) == sgn) if grab else None, hold=hold_tq))
      i = a1 + 100
    else:
      i += 1
for b, a in agg.items():
  E = a['ev']; h = a['h']
  print(f"== build {b}: {h * 60:.0f} min op active >= 30 km/h | fast wheel moves under op authority {len(E)} ({len(E) / h:.1f}/h)")
  for kind in ("gap-close", "command"):
    es = [e for e in E if e['kind'] == kind]
    if not es:
      continue
    gs = [e for e in es if e['grab']]
    print(f"   {kind:9} {len(es):3d} ({len(es) / h:4.1f}/h) size p50 {np.median([e['dang'] for e in es]):4.1f} rate p50 {np.median([e['rate'] for e in es]):3.0f} max {max(e['rate'] for e in es):3.0f} deg/s "
          f"gap p50 {np.median([e['gap'] for e in es]):+5.1f} | grabbed {len(gs)} (with {sum(1 for e in gs if e['gwith'])}, against {sum(1 for e in gs if e['gwith'] is False)}) "
          f"| hand resisting before (median tq < -30) {sum(1 for e in es if e['hold'] < -30)} | cmd jerk p50 {np.median([e['jerk'] for e in es]):.1f} p90 {np.percentile([e['jerk'] for e in es], 90):.1f} m/s3 "
          f"| road-driven (d road k >= 0.5 dK) {sum(1 for e in es if e['dkr'] >= 0.5 * e['dK'])}")
  for e in sorted(E, key=lambda e: -e['rate'])[:14]:
    print(f"     {e['r']} t {e['t']:7.1f} v {e['v']:3.0f} {e['kind']:9} {e['dang']:4.1f} deg @ {e['rate']:3.0f} gap {e['gap']:+5.1f} hold-tq {e['hold']:+5.0f} jerk {e['jerk']:4.1f} "
          f"dK {e['dK'] * 1e3:4.1f} droad {e['dkr'] * 1e3:4.1f} lm {e['lm']:.2f} {'GRAB ' + ('with' if e['gwith'] else 'AGAINST') if e['grab'] else ''}")
