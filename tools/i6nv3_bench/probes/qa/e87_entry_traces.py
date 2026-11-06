"""E87: hands-off corner-entry traces (same entry rule as e68b/e82, v >= 55 km/h unless argv[2] = 'low' -> 30-55), aligned
on the model K reaching 50 % of its peak (t = 0). Everything is expressed as a share of the entry's K peak (angles divided by
the per-route angle-per-curvature scale): command (apply), TX (wire), wheel (ang), yaw curvature; plus TX - wheel gap (deg),
wheel rate (deg/s), ACI gain, MDPS torque and the driver torque sensor. Median over entries per build at -1.0 .. +1.5 s."""
import glob, sys
from collections import defaultdict
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; LOW = len(sys.argv) > 2 and sys.argv[2] == "low"
BUILDS = ("98e912d", "bf36aa2")
TR = defaultdict(list)
OFFS = np.arange(-100, 151, 10)
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  r = f[-12:-4]
  d = dict(np.load(f, allow_pickle=True)); b = str(d['commit'])[:7]
  if b not in BUILDS:
    continue
  try:
    V = dict(np.load(f"{S}/vm/{r}.npz", allow_pickle=True))
  except Exception:
    continue
  T = d['T']; n = len(T); v = d['v']; vm = v / 3.6
  ky = np.radians(np.interp(T, V['lk'][:, 0], V['lk'][:, 1])) / np.maximum(vm, 1.0)
  K = d['k_model']; kc = d['k_ctrl']
  m = d['lat'] & (np.abs(kc) > 0.5e-3)
  sc = np.sum(d['apply'][m] * kc[m]) / np.sum(kc[m] ** 2) if m.sum() > 100 else -3000.0
  la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  cs = lambda x: np.concatenate([[0], np.cumsum(x)]); idx = np.arange(n)
  busy = (cs(bl)[np.minimum(idx + 300, n)] - cs(bl)[np.maximum(idx - 300, 0)]) > 0
  ok = d['lat'] & ~busy & (v >= 30)
  handw = (dd >= 40) | d['pressed']
  rate = np.zeros(n); rate[5:] = (d['ang'][5:] - d['ang'][:-5]) / 0.05
  aK = np.abs(K); i = 200
  while i < n - 500:
    if aK[i] < 0.7e-3 and (aK[i - 100:i] < 0.7e-3).all():
      pk = i + int(np.argmax(aK[i:i + 300]))
      if aK[pk] >= 2e-3 and ok[i - 100:pk + 200].all():
        s = np.sign(K[pk]); kp = aK[pk]
        k50 = next((j for j in range(i, pk + 1) if K[j] * s >= 0.5 * kp), None)
        vb = np.median(v[i:pk])
        inband = (30 <= vb < 55) if LOW else (vb >= 55)
        if k50 is not None and inband and handw[i:pk + 200].mean() < 0.1 and k50 - 100 >= 0 and k50 + 151 < n:
          w = k50 + OFFS; a0 = d['apply'][k50 - 100]; t0 = d['wire'][k50 - 100]; g0 = d['ang'][k50 - 100]
          TR[b].append(dict(K=K[w] * s / kp, cmd=(d['apply'][w] - a0) / sc * s / kp, tx=(d['wire'][w] - t0) / sc * s / kp,
                            wh=(d['ang'][w] - g0) / sc * s / kp, yaw=ky[w] * s / kp, gap=(d['wire'][w] - d['ang'][w]) * s * np.sign(sc),
                            rate=rate[w] * s * np.sign(sc), gain=d['gain'][w], mtq=d['mdps_tq'][w] * s * np.sign(sc),
                            tqs=d['tq_s'][w] * s * np.sign(sc), v=vb, kp=kp, txdeg=abs(d['wire'][k50 + 50] - t0)))
          ss = s * np.sign(sc); seg = slice(k50 - 100, k50 + 151)
          txm = (d['wire'][seg] - t0) * ss; whm = (d['ang'][seg] - g0) * ss; Km = K[seg] * s
          on = lambda x, lev: next((j for j in range(len(x)) if x[j] >= lev), None)
          TR[b][-1].update(on_tx1=on(txm, 1.0), on_w1=on(whm, 1.0), on_tx3=on(txm, 3.0), on_w3=on(whm, 3.0), on_k=on(Km, 0.25 * kp),
                           txrate=np.max(np.diff(txm[::5])) / 0.05)
        i = pk + 300; continue
    i += 10
SPLIT = sys.argv[3] if len(sys.argv) > 3 else ''
for b in BUILDS:
  E = TR[b]
  if SPLIT:
    lo_, hi_ = [float(x) for x in SPLIT.split(':')]
    E = [e for e in E if lo_ <= np.max(np.abs(e['tqs'][:16])) < hi_]
  if not E:
    continue
  print(f"== {b}: {len(E)} hands-off entries, v p50 {np.median([e['v'] for e in E]):.0f} km/h, K peak p50 {np.median([e['kp'] for e in E])*1e3:.1f}e-3, TX move by +0.5 s p50 {np.median([e['txdeg'] for e in E]):.1f} deg")
  print("   t s |   K   cmd    TX  wheel   yaw | gap deg  wheel deg/s  gain  mdps_tq  drv_tq")
  for j, o in enumerate(OFFS):
    if o % 20:
      continue
    g = lambda k: np.median([e[k][j] for e in E])
    print(f"  {o/100:+4.1f} | {g('K'):5.2f} {g('cmd'):5.2f} {g('tx'):5.2f} {g('wh'):5.2f} {g('yaw'):5.2f} | {g('gap'):+6.2f}  {g('rate'):+8.1f}  {g('gain'):5.2f} {g('mtq'):+7.1f} {g('tqs'):+7.1f}")

print("per-entry onsets (s, relative to K = 50 % peak): K 25 %, TX +1 deg, wheel +1 deg, TX +3, wheel +3 | wheel1 - TX1, wheel3 - TX3")
for b in BUILDS:
  E = TR[b]
  if SPLIT:
    pass
  W1 = []; W3 = []; TXK = []
  for e in E:
    f = lambda k: (e[k] - 100) / 100 if e.get(k) is not None else np.nan
    W1.append(f('on_w1') - f('on_tx1')); W3.append(f('on_w3') - f('on_tx3')); TXK.append(f('on_tx1') - f('on_k'))
  print(f"  {b}: n {len(E)} | wheel+1 after TX+1 p25/p50/p75 {np.nanpercentile(W1, 25):.2f}/{np.nanmedian(W1):.2f}/{np.nanpercentile(W1, 75):.2f} s "
        f"| wheel+3 after TX+3 p50 {np.nanmedian(W3):.2f} s | TX+1 after K25 % p50 {np.nanmedian(TXK):+.2f} s | TX+1 vs K50 p50 {np.nanmedian([f2 for f2 in [(e['on_tx1'] - 100) / 100 for e in E if e['on_tx1'] is not None]]):+.2f} s")
  for e in E:
    f = lambda k: (e[k] - 100) / 100 if e.get(k) is not None else np.nan
    print(f"     v {e['v']:3.0f} K {e['kp']*1e3:.1f} | K25 {f('on_k'):+.2f} TX1 {f('on_tx1'):+.2f} W1 {f('on_w1'):+.2f} TX3 {f('on_tx3'):+.2f} W3 {f('on_w3'):+.2f} | TX max rate {e['txrate']:4.1f} deg/s")
