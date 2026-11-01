"""E56: curvature <-> angle conversion and delays. Inputs: r50 npz (100 Hz) + vm npz (vehicleParameters 20 Hz,
liveLocationKalman yaw 20 Hz, lateralDelay 4 Hz, model bundle).
 1. Per route: model bundle, learned steerRatio / stiffness / angle offset (median, first -> last), lateralDelay status.
 2. Conversion check (hands-off and hands-on, >= 30 km/h, steady: |dk/dt| small): car curvature from yaw (yaw / v) vs
    k_meas (controlsState.curvature = VM(wheel - offset) with the learned params). Slope (through origin and with
    intercept), intercept, residual RMS; by speed band; implied steer-ratio correction = 1 / slope.
 3. Delays by cross-correlation (hands-off op, >= 30 km/h, 6 s windows with motion): model action K -> wheel curvature
    k_meas, k_meas -> yaw curvature, K -> yaw curvature. Medians per speed band vs the model's assumed 0.375 s."""
import ast
import glob
import sys
from collections import defaultdict
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]
BANDS = [(30, 60), (60, 90), (90, 200)]
conv = defaultdict(list); lags = defaultdict(lambda: defaultdict(list))
print("route  bundle      SR med (first->last)  stiff  offAvg   delay status/blocks/est")
for f in sorted(glob.glob(f"{S}/vm/0*.npz")):
  r = f[-12:-4]
  V = np.load(f, allow_pickle=True)
  try:
    pr = ast.literal_eval(str(V['params']))
  except Exception:  # noqa: BLE001
    pr = {}
  bund = pr.get('ModelManager_ActiveBundle', '')
  name = bund.split('"internalName": "')[1].split('"')[0] if 'internalName' in bund else 'default'
  lp = V['lp']; lk = V['lk']; ld = V['ld']
  okp = lp[:, 6] > 0.5 if len(lp) else np.zeros(0, bool)
  srm = np.median(lp[okp, 1]) if okp.any() else np.nan
  st = f"{int(np.median(ld[:, 2])) if len(ld) else -1}/{int(np.max(ld[:, 3])) if len(ld) else -1}/{np.median(ld[:, 4]) if len(ld) else np.nan:.3f}"
  print(f"{r} {name:10} {srm:6.3f} ({lp[okp, 1][0] if okp.any() else np.nan:.3f}->{lp[okp, 1][-1] if okp.any() else np.nan:.3f}) "
        f"{np.median(lp[okp, 2]) if okp.any() else np.nan:5.3f} {np.median(lp[okp, 4]) if okp.any() else np.nan:+6.2f}  {st}")
  try:
    d = np.load(f"{S}/r50/{r}.npz", allow_pickle=True)
  except FileNotFoundError:
    continue
  T = d['T']; n = len(T); v = d['v']
  if n < 3000 or len(lk) < 100:
    continue
  yaw = np.interp(T, lk[:, 0], lk[:, 1]); yv = np.interp(T, lk[:, 0], lk[:, 2]) > 0.5
  vm = v / 3.6
  ky = np.radians(yaw) / np.maximum(vm, 1.0)
  km = d['k_meas']; K = d['k_model']
  la = np.abs(vm ** 2 * km)
  hold = np.array([compute_hold_torque(a, b) for a, b in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  c = lambda m: np.concatenate([[0], np.cumsum(m)]); idx = np.arange(n)
  busy = (c(bl)[np.minimum(idx + 300, n)] - c(bl)[np.maximum(idx - 300, 0)]) > 0
  base = yv & (v >= 30) & ~busy
  # 2. conversion: steady frames
  dk = np.zeros(n); dk[50:-50] = np.abs(km[100:] - km[:-100])
  stdy = base & (dk < 0.5e-3)
  for i in range(0, n, 10):
    if stdy[i]:
      b = next(bi for bi, (lo, hi) in enumerate(BANDS) if lo <= v[i] < hi)
      conv[b].append((km[i], ky[i], d['lat'][i] and d['gain'][i] >= 0.9 and dd[i] < 40))
  # 3. delays
  hoff = base & d['lat'] & (d['gain'] >= 0.9) & (dd < 40) & ~d['pressed']
  W = 600
  for a in range(0, n - W - 150, W // 2):
    w = slice(a, a + W)
    if not hoff[w].all() or np.std(km[w]) < 0.3e-3:
      continue
    b = next(bi for bi, (lo, hi) in enumerate(BANDS) if lo <= np.median(v[w]) < hi)
    for nm, x, y in (("K->wheel", K, km), ("wheel->yaw", km, ky), ("K->yaw", K, ky)):
      xs = x[a:a + W] - np.mean(x[a:a + W])
      best = (-2, 0)
      for L in range(0, 151, 5):
        ys = y[a + L:a + L + W] - np.mean(y[a + L:a + L + W])
        cc = np.dot(xs, ys) / (np.linalg.norm(xs) * np.linalg.norm(ys) + 1e-12)
        if cc > best[0]:
          best = (cc, L)
      if best[0] > 0.7:
        lags[b][nm].append(best[1] * 10)
print()
for b, (lo, hi) in enumerate(BANDS):
  A = np.array(conv[b], float)
  if len(A) < 50:
    continue
  x, y = A[:, 0], A[:, 1]
  s0 = np.sum(x * y) / np.sum(x * x); s1, i1 = np.polyfit(x, y, 1)
  res = y - (s1 * x + i1)
  h = A[:, 2] > 0.5
  sh = np.sum(x[h] * y[h]) / np.sum(x[h] * x[h]) if h.sum() > 50 else np.nan
  print(f"[conversion {lo}-{hi}] n {len(A)} | yaw-curvature vs wheel-curvature: slope {s0:.3f} (hands-off {sh:.3f}), fit slope {s1:.3f} intercept {i1 * 1e3:+.3f}e-3 "
        f"(= {i1 * 1e3 * 3.1:+.2f} wheel deg) | residual RMS {np.sqrt(np.mean(res ** 2)) * 1e3:.3f}e-3")
  for k0, k1 in ((0, 1e-3), (1e-3, 3e-3), (3e-3, 1)):
    m = (np.abs(x) >= k0) & (np.abs(x) < k1)
    if m.sum() > 50:
      print(f"      |k| {k0 * 1e3:.0f}-{min(k1, 0.1) * 1e3:.0f}e-3: n {m.sum()} slope {np.sum(x[m] * y[m]) / np.sum(x[m] ** 2):.3f}, mean (yaw - wheel) {np.mean(y[m] - x[m]) * 1e3:+.3f}e-3")
  L = lags[b]
  print(f"   delays (median ms, n): " + ", ".join(f"{k} {np.median(L[k]):.0f} (p25 {np.percentile(L[k], 25):.0f} p75 {np.percentile(L[k], 75):.0f}, n {len(L[k])})" for k in ("K->wheel", "wheel->yaw", "K->yaw") if len(L[k]) >= 3))
