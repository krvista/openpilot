"""E47: the wheel turning fast in the driver's hands with op doing it: |wheel rate| >= R deg/s (0.1 s diff) for >= 0.1 s,
op active >= 15 km/h, no blinker / LC within 3 s, |tq| < 100 & not pressed from onset-0.5 s to onset+0.2 s, gain >= 0.2 near
the onset. Kind: 'tx-fast' (TX moved >= 60 % of the wheel move over the episode), 'gap-close' (|TX-wheel| >= 4 deg 0.3 s
before and the TX moved less), 'other'. Grab (>= 250) 0.2-2 s after the onset, and its direction vs the motion (+ torque =
+ angle). Per build and speed band; top cases with the command source (dK = model move, dcmd = controlsd command move)."""
import glob
import sys
from collections import defaultdict
import numpy as np
S = sys.argv[1]; R = float(sys.argv[2]) if len(sys.argv) > 2 else 30
agg = defaultdict(lambda: dict(h=defaultdict(float), ev=[]))
BANDS = [(15, 30), (30, 50), (50, 70), (70, 200)]
band = lambda v: next(f"{lo}-{hi}" for lo, hi in BANDS if lo <= v < hi)
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = np.load(f, allow_pickle=True); b = str(d['commit']); r = f[-12:-4]
  T = d['T']; n = len(T); v = d['v']
  if n < 3000:
    continue
  ang = d['ang']; wire = d['wire']; ap = d['apply']; g = d['gain']; tqs = d['tq_s']; tq = np.abs(tqs); pr = d['pressed']; K = d['k_model']; kc = d['k_ctrl']; lm = d['lane_min']
  sc = d['sf_cc'].astype(np.int64); sf = d['sf_ctl'].astype(np.int64)
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  c = lambda m: np.concatenate([[0], np.cumsum(m)]); idx = np.arange(n)
  busy = (c(bl)[np.minimum(idx + 300, n)] - c(bl)[np.maximum(idx - 300, 0)]) > 0
  act = d['lat'] & (v >= 15) & ~busy
  for lo, hi in BANDS:
    agg[b]['h'][f"{lo}-{hi}"] += (act & (v >= lo) & (v < hi)).sum() / 360000
  rate = np.zeros(n); rate[10:] = (ang[10:] - ang[:-10]) / np.maximum(T[10:] - T[:-10], 0.05)
  loud = c((tq >= 100) | pr)
  i = 100
  while i < n - 250:
    if act[i] and (np.abs(rate[i:i + 10]) >= R).all() and loud[i + 20] - loud[i - 50] == 0 and g[i - 20:i + 10].max() >= 0.2:
      a0 = i - 10; a1 = i + 30; sgn = np.sign(rate[i]); dang = abs(ang[a1] - ang[a0])
      dtx = (wire[a1] - wire[a0]) * sgn; gap = (wire[i - 30] - ang[i - 30]) * sgn
      kind = "tx-fast" if dtx >= 0.6 * dang else ("gap-close" if gap >= 4 else "other")
      w = tq[i + 20:i + 200] >= 250; grab = bool(w.any())
      gw = (np.sign(tqs[i + 20 + int(np.argmax(w))]) == sgn) if grab else None
      agg[b]['ev'].append(dict(r=r, t=T[i], v=v[i], kind=kind, dang=dang, rate=np.max(np.abs(rate[a0:a1])), gap=gap, grab=grab, gw=gw, g=g[i],
                               dK=abs(K[i + 20] - K[i - 50]), dkc=abs(kc[i + 20] - kc[i - 50]), lm=lm[i], band=band(v[i]),
                               cc=int(np.bitwise_or.reduce(sc[i - 50:i + 20])), ctl=int(np.bitwise_or.reduce(sf[i - 50:i + 20]))))
      i = a1 + 100
    else:
      i += 1
CCN = {12: "fast", 15: "accelY", 16: "boostSup", 17: "blindC", 18: "city39a", 19: "anch35b", 20: "blinkC", 21: "post29", 25: "outrun", 27: "txSat", 28: "hod51"}
SER = {0: "41gate", 1: "41cap", 2: "41rel", 4: "6g", 5: "drop", 6: "dep", 7: "bsm", 8: "7c", 10: "budget"}
for b, A in agg.items():
  E = A['ev']; H = A['h']; h = sum(H.values())
  print(f"== build {b} wheel >= {R:.0f} deg/s, op-made: {h * 60:.0f} min | {len(E)} ({len(E) / h:.1f}/h), grabbed {sum(e['grab'] for e in E)} "
        f"(with {sum(1 for e in E if e['gw'])}, against {sum(1 for e in E if e['gw'] is False)})")
  for bd in H:
    es = [e for e in E if e['band'] == bd]
    k = lambda kk: sum(1 for e in es if e['kind'] == kk)
    print(f"   {bd:7} {H[bd] * 60:4.0f} min: {len(es):3d} ({len(es) / max(H[bd], 1e-9):5.1f}/h) tx-fast {k('tx-fast')} gap-close {k('gap-close')} other {k('other')} | grabbed {sum(e['grab'] for e in es)}")
  for e in sorted(E, key=lambda e: -e['rate'])[:16]:
    fl = [nm for bb, nm in SER.items() if e['ctl'] >> bb & 1] + [nm for bb, nm in CCN.items() if e['cc'] >> bb & 1]
    print(f"     {e['r']} t {e['t']:7.1f} v {e['v']:3.0f} {e['kind']:9} {e['dang']:4.1f} deg @ {e['rate']:3.0f} gap {e['gap']:+5.1f} gain {e['g']:.2f} dK {e['dK'] * 1e3:4.1f} dcmd {e['dkc'] * 1e3:4.1f} "
          f"lm {e['lm']:.2f} {('GRAB ' + ('with' if e['gw'] else 'AGAINST')) if e['grab'] else '':12} {' '.join(fl)}")
