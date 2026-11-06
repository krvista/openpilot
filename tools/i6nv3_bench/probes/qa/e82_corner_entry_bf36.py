"""E82 (e68b + bf36aa2, plus a hands-off-only summary): corner ENTRY timing by build and speed band. Entry = model action |K| rising from < 0.7e-3 (held 1 s) to a peak
>= 2e-3 within 3 s, op active, no blinker / LC within 3 s, hands off over the entry (driver-domain < 40). Per entry:
lag of the car (yaw curvature) behind the model: time from K reaching 50 % of its peak to the yaw curvature reaching that
level (and 80 %); peak outside lane offset (lane-centre offset toward the outside, 0-4 s after the start, lines >= 0.6);
command lag (controlsd command reaching 50 %); and TX / wheel at the K peak time as a share of the command."""
import glob
import sys
from collections import defaultdict
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]
BANDS = [(30, 55), (55, 200)]
R = defaultdict(list)
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  r = f[-12:-4]
  d = dict(np.load(f, allow_pickle=True)); b = str(d['commit'])[:7]
  if b not in ("98e912d", "a0108a4", "31520b9", "bf36aa2"):
    continue
  try:
    V = np.load(f"{S}/vm/{r}.npz", allow_pickle=True); lk = V['lk']; P = np.load(f"{S}/la/plan_{r}.npz")
  except Exception:  # noqa: BLE001
    continue
  T = d['T']; n = len(T); v = d['v']; vm = v / 3.6
  ky = np.radians(np.interp(T, lk[:, 0], lk[:, 1])) / np.maximum(vm, 1.0)
  K = d['k_model']; kc = d['k_ctrl']
  m = d['lat'] & (np.abs(kc) > 0.5e-3)
  sc = np.sum(d['apply'][m] * kc[m]) / np.sum(kc[m] ** 2) if m.sum() > 100 else -3000.0
  kw = d['ang'] / sc; ktx = d['wire'] / sc
  lc = 0.5 * (P['L1'] + P['L2']); off20 = np.array([-np.interp(0.0, x, y) for x, y in zip(P['LX'], lc)])
  off = np.interp(T, P['T'], off20); lok = np.interp(T, P['T'], np.minimum(P['PR'][:, 1], P['PR'][:, 2])) > 0.6
  la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  cs = lambda x: np.concatenate([[0], np.cumsum(x)]); idx = np.arange(n)
  busy = (cs(bl)[np.minimum(idx + 300, n)] - cs(bl)[np.maximum(idx - 300, 0)]) > 0
  ok = d['lat'] & ~busy & (v >= 30)
  handw = (dd >= 40) | d['pressed']
  aK = np.abs(K)
  i = 200
  while i < n - 500:
    if aK[i] < 0.7e-3 and (aK[i - 100:i] < 0.7e-3).all():
      w = slice(i, i + 300)
      pk = i + int(np.argmax(aK[w]))
      if aK[pk] >= 2e-3 and ok[i - 100:pk + 200].all():
        s = np.sign(K[pk]); kp = aK[pk]
        t_of = lambda x, lev: next((j for j in range(i, min(pk + 300, n)) if x[j] * s >= lev), None)
        k50 = t_of(K, 0.5 * kp); c50 = t_of(kc, 0.5 * kp); y50 = t_of(ky, 0.5 * kp); y80 = t_of(ky, 0.8 * kp)
        if k50 is not None and y50 is not None:
          # outside = away from the curve centre: curvature s>0 -> left turn (convention: K>0 left?) use offset sign vs s
          o = off[i:i + 400] * (-s) if lok[i:i + 400].mean() > 0.8 else None
          bi = 0 if np.median(v[i:pk]) < 55 else 1
          R[(b, bi)].append(dict(lag50=(T[y50] - T[k50]), lag80=(T[y80] - T[t_of(K, 0.8 * kp)]) if (y80 and t_of(K, 0.8 * kp)) else np.nan,
                                 clag=(T[c50] - T[k50]) if c50 else np.nan, out=np.max(o) if o is not None else np.nan,
                                 hand=handw[i:pk + 200].mean(), grab_before=bool(handw[i:y50].any()) if y50 else False, tx=ktx[pk] * s / kp, wh=kw[pk] * s / kp, yw=ky[pk] * s / kp))
        i = pk + 300; continue
    i += 10
for bi, (lo, hi) in enumerate(BANDS):
  print(f"[{lo}-{hi} km/h]")
  for b in ("a0108a4", "98e912d", "31520b9", "bf36aa2"):
    E = R[(b, bi)]
    if not E:
      continue
    g = lambda k: np.nanmedian([e[k] for e in E])
    print(f"   {b}: entries {len(E):3d} | car behind model 50 %: p50 {g('lag50') * 1000:4.0f} ms (p75 {np.nanpercentile([e['lag50'] for e in E], 75) * 1000:4.0f}), 80 %: {g('lag80') * 1000:4.0f} ms "
          f"| hand share {g('hand'):.2f}, hand before the car reached 50 % {np.mean([e['grab_before'] for e in E]):.0%} | command behind model {g('clag') * 1000:4.0f} ms | at the model peak: TX {g('tx'):.2f} wheel {g('wh'):.2f} yaw {g('yw'):.2f} of K | outside offset max p50 {g('out') * 100:+5.1f} cm")
print("hands-off entries only (hand share < 0.1):")
for bi, (lo, hi) in enumerate(BANDS):
  for b in ("98e912d", "31520b9", "bf36aa2"):
    E = [e for e in R[(b, bi)] if e['hand'] < 0.1]
    if len(E) >= 3:
      print(f"  [{lo}-{hi}] {b}: n {len(E):3d} | lag50 p50 {np.nanmedian([e['lag50'] for e in E]) * 1000:4.0f} ms | yaw at peak {np.nanmedian([e['yw'] for e in E]):.2f} | wheel {np.nanmedian([e['wh'] for e in E]):.2f} | TX {np.nanmedian([e['tx'] for e in E]):.2f} | outside {np.nanmedian([e['out'] for e in E]) * 100:+.1f} cm")
