"""E56b: where the delay sits. Hands-off op windows (6 s, 50 % overlap, >= 30 km/h, std of the wheel curvature >= 0.2e-3),
cross-correlation lag and amplitude ratio (std(y)/std(x) at the best lag) for each link:
  K (model action) -> kc (controlsd command) -> TX (sent angle, as curvature via apply/kc scale) -> wheel -> yaw."""
import glob
import sys
from collections import defaultdict
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]
BANDS = [(30, 60), (60, 90), (90, 200)]
L = defaultdict(lambda: defaultdict(list))
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  r = f[-12:-4]
  d = np.load(f, allow_pickle=True); V = np.load(f"{S}/vm/{r}.npz", allow_pickle=True)
  T = d['T']; n = len(T); v = d['v']; lk = V['lk']
  if n < 3000 or len(lk) < 100:
    continue
  vm = v / 3.6
  ky = np.radians(np.interp(T, lk[:, 0], lk[:, 1])) / np.maximum(vm, 1.0)
  km = d['k_meas']; K = d['k_model']; kc = d['k_ctrl']
  # TX and wheel angle -> curvature with the same per-route scale as apply ~ -kc (fit)
  m = d['lat'] & (np.abs(kc) > 0.5e-3)
  sc = np.sum(d['apply'][m] * kc[m]) / np.sum(kc[m] ** 2) if m.sum() > 100 else -3000.0
  ktx = d['wire'] / sc; kw = d['ang'] / sc
  la = np.abs(vm ** 2 * km)
  hold = np.array([compute_hold_torque(a, b) for a, b in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  c = lambda mm: np.concatenate([[0], np.cumsum(mm)]); idx = np.arange(n)
  busy = (c(bl)[np.minimum(idx + 300, n)] - c(bl)[np.maximum(idx - 300, 0)]) > 0
  hoff = d['lat'] & (d['gain'] >= 0.9) & (dd < 40) & ~d['pressed'] & ~busy & (v >= 30)
  W = 600
  for a in range(0, n - W - 150, W // 2):
    if not hoff[a:a + W].all() or np.std(km[a:a + W]) < 0.2e-3:
      continue
    b = next(bi for bi, (lo, hi) in enumerate(BANDS) if lo <= np.median(v[a:a + W]) < hi)
    for nm, x, y in (("K->cmd", K, kc), ("cmd->TX", kc, ktx), ("TX->wheel", ktx, kw), ("wheel->yaw", km, ky), ("K->wheel", K, km)):
      xs = x[a:a + W] - np.mean(x[a:a + W]); best = (-2, 0, 0)
      for Lg in range(-50, 151, 5):
        if a + Lg < 0:
          continue
        ys = y[a + Lg:a + Lg + W] - np.mean(y[a + Lg:a + Lg + W])
        cc = np.dot(xs, ys) / (np.linalg.norm(xs) * np.linalg.norm(ys) + 1e-12)
        if cc > best[0]:
          best = (cc, Lg, np.std(ys) / (np.std(xs) + 1e-12))
      if best[0] > 0.7:
        L[b][nm].append((best[1] * 10, best[2]))
for b, (lo, hi) in enumerate(BANDS):
  print(f"[{lo}-{hi} km/h] " + " | ".join(f"{k}: lag {np.median([x[0] for x in L[b][k]]):+.0f} ms (p25 {np.percentile([x[0] for x in L[b][k]], 25):+.0f} p75 {np.percentile([x[0] for x in L[b][k]], 75):+.0f}) amp {np.median([x[1] for x in L[b][k]]):.2f} n {len(L[b][k])}"
                                        for k in ("K->cmd", "cmd->TX", "TX->wheel", "wheel->yaw", "K->wheel") if len(L[b][k]) >= 3))
