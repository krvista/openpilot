"""E55: lateral error budget, op driving hands-off (lat, gain >= 0.9, hold-compensated driver torque < 40, not pressed, no
blinker / LC within 3 s, both lane lines >= 0.7, >= 30 km/h). Per speed band:
  car offset          : lane-centre offset of the car (plan lanes at x = 0)                       [what the driver sees]
  model target offset : the model's planned path vs the lane centre at x = 10 / 20 m            [MODEL intention]
  model curve gain    : model action K / road curvature from the lanes (10-30 m) in steady curves [MODEL under/over turn]
  model jitter        : RMS of K minus its 1 s mean on straights, as wheel deg                   [MODEL churn -> TX churn]
  command - model     : k_ctrl - K RMS                                                            [controlsd shaping]
  car - command       : k_meas(t + 0.4 s) - k_ctrl(t) RMS; and in steady curves k_meas / k_ctrl   [ACTUATOR (MDPS + carctl)]"""
import glob
import sys
from collections import defaultdict
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]
BANDS = [(30, 60), (60, 90), (90, 200)]
DEG_PER = ([40, 60, 80, 100], [2.85, 3.12, 3.45, 3.74])
acc = defaultdict(lambda: defaultdict(list))
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = np.load(f, allow_pickle=True); r = f[-12:-4]
  T = d['T']; n = len(T); v = d['v']
  if n < 3000:
    continue
  P = np.load(f"{S}/la/plan_{r}.npz")
  vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, b) for a, b in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  c = lambda m: np.concatenate([[0], np.cumsum(m)]); idx = np.arange(n)
  busy = (c(bl)[np.minimum(idx + 300, n)] - c(bl)[np.maximum(idx - 300, 0)]) > 0
  ok = d['lat'] & (d['gain'] >= 0.9) & (dd < 40) & ~d['pressed'] & ~busy & (v >= 30)
  K = d['k_model']; kc = d['k_ctrl']; km = d['k_meas']
  # plan quantities at 20 Hz: vectorised over the plan frames
  j = np.arange(1, len(P['T']))
  i = np.searchsorted(T, P['T'][j]); keep = (i >= 200) & (i < n - 100)
  j = j[keep]; i = i[keep]
  keep = ok[i] & (np.minimum(P['PR'][j, 1], P['PR'][j, 2]) >= 0.7)
  j = j[keep]; i = i[keep]
  if not len(i):
    continue
  def at(x, xs, ys):
    return np.array([np.interp(x, a, b) for a, b in zip(xs, ys)])
  LX = P['LX'][j]; LC = 0.5 * (P['L1'][j] + P['L2'][j]); PX = P['PX'][j]; PY = P['PY'][j]
  c0 = at(0.0, LX, LC); c10 = at(10.0, LX, LC); c20 = at(20.0, LX, LC); c30 = at(30.0, LX, LC)
  kr = 2 * (c30 - 2 * c20 + c10) / 100.0
  cK = np.concatenate([[0], np.cumsum(K)]); Km = (cK[i + 50] - cK[i - 50]) / 100.0
  co = np.concatenate([[0], np.cumsum(~ok)]); ok1 = (co[i + 50] - co[i - 50]) == 0
  vals = dict(car_off=-c0, tgt10=at(10.0, PX, PY) - c10, tgt20=at(20.0, PX, PY) - c20, kroad=kr, K=K[i], kc=kc[i], km4=km[i + 40], v=v[i],
              Kjit=K[i] - Km, ok1s=ok1)
  for b, (lo, hi) in enumerate(BANDS):
    mb = (v[i] >= lo) & (v[i] < hi)
    for k, x in vals.items():
      acc[b][k].extend(x[mb].tolist())
for b, (lo, hi) in enumerate(BANDS):
  A = {k: np.array(x) for k, x in acc[b].items()}
  if not len(A.get('K', [])):
    continue
  dpk = np.interp(np.median(A['v']), *DEG_PER)       # wheel deg per 1e-3/m
  st = (np.abs(A['kroad']) < 0.3e-3) & A['ok1s']
  cv = np.abs(A['kroad']) >= 1.5e-3
  print(f"== {lo}-{hi} km/h: {len(A['K']) / 1200:.0f} min hands-off op (both lines >= 0.7)")
  print(f"   car offset            mean {np.mean(A['car_off']) * 100:+5.1f} cm, RMS {np.sqrt(np.mean(A['car_off'] ** 2)) * 100:4.1f} cm")
  print(f"   MODEL target offset   @10 m mean {np.mean(A['tgt10']) * 100:+5.1f} RMS {np.sqrt(np.mean(A['tgt10'] ** 2)) * 100:4.1f} cm | @20 m mean {np.mean(A['tgt20']) * 100:+5.1f} RMS {np.sqrt(np.mean(A['tgt20'] ** 2)) * 100:4.1f} cm")
  for k0, k1 in ((1.5e-3, 3e-3), (3e-3, 6e-3), (6e-3, 1)):
    m = (np.abs(A['kroad']) >= k0) & (np.abs(A['kroad']) < k1)
    if m.sum() >= 40:
      g = A['K'][m] / A['kroad'][m]; gc = A['km4'][m] / A['kc'][m]
      print(f"   curve |k| {k0 * 1e3:.1f}-{min(k1, 0.1) * 1e3:.0f}e-3 n {m.sum():5d}: MODEL K/road p50 {np.median(g):.2f} | command/model p50 {np.median(A['kc'][m] / A['K'][m]):.2f} | ACTUATOR car(+0.4 s)/command p50 {np.median(gc):.2f}")
  print(f"   MODEL jitter on straights (n {st.sum()}): RMS {np.sqrt(np.mean(A['Kjit'][st] ** 2)) * 1e3:.3f}e-3 = {np.sqrt(np.mean(A['Kjit'][st] ** 2)) * 1e3 * dpk:.2f} wheel deg")
  e2 = A['kc'] - A['K']; e3 = A['km4'] - A['kc']
  print(f"   RMS (1e-3/m): command - model {np.sqrt(np.mean(e2 ** 2)) * 1e3:.3f} | car(+0.4 s) - command {np.sqrt(np.mean(e3 ** 2)) * 1e3:.3f} "
        f"(curves {np.sqrt(np.mean(e3[cv] ** 2)) * 1e3:.3f}, straights {np.sqrt(np.mean(e3[st] ** 2)) * 1e3:.3f}) | model - road {np.sqrt(np.mean((A['K'] - A['kroad']) ** 2)) * 1e3:.3f}")
