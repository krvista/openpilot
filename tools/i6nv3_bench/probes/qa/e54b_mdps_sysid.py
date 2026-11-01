"""E54: MDPS angle-request response, identified from normal driving (all 30 routes). Frames: op active, LKAS_ALT active (2),
ACI gain >= 0.95 for the last 0.5 s, raw |driver torque| < 100 and not pressed for the last 0.5 s, >= 15 km/h, no blinker.
Contiguous runs >= 3 s, resampled to 100 Hz.
 1. FIR (20 Hz, 1.5 s) from TX rate to wheel rate per speed band: delay (first tap >= 10 % of peak, centroid) and rate gain.
 2. Steady state: 1.5 s windows with TX and wheel each within 0.5 deg, |TX| >= 2 deg: wheel / TX and the gap, by speed and
    lateral acceleration (v^2 * k_meas).
 3. Small moves: TX steps of 0.5-3 deg completed within 0.3 s from rest: wheel travel after 0.5 / 1.0 s as a fraction.
 4. Rate following: TX ramps held >= 0.4 s at a rate band: wheel rate over the same window (lagged 0.15 s).
 5. Compact plant fit per speed band:  dwheel/dt = clip(Kp*(TX(t-d) - wheel) + Kf*dTX(t-d)/dt, +-Rmax)
    (grid over d, least squares Kp/Kf, Rmax from the p99 wheel rate); open-loop simulated RMS on held-out runs vs a
    'wheel = TX(t-d)' baseline."""
import glob
import sys
from collections import defaultdict
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]
BANDS = [(15, 40), (40, 70), (70, 200)]
runs = []
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = np.load(f, allow_pickle=True)
  T = d['T']; n = len(T); v = d['v']
  if n < 3000:
    continue
  vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, b) for a, b in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  ok = d['lat'] & (d['tx_active'] == 2) & (d['gain'] >= 0.95) & (dd < 40) & ~d['pressed'] & (v >= 15) & ~(d['lb'] | d['rb'])
  c = np.concatenate([[0], np.cumsum(~ok)]); idx = np.arange(n)
  ok2 = ok & (c[idx + 1] - c[np.maximum(idx - 50, 0)] == 0)
  e = np.flatnonzero(np.diff(np.concatenate([[0], ok2.astype(int), [0]])))
  for a, b in zip(e[::2], e[1::2]):
    if T[b - 1] - T[a] < 3.0:
      continue
    tg = np.arange(T[a], T[b - 1], 0.01)
    R = {k: np.interp(tg, T[a:b], d[k][a:b]) for k in ('wire', 'ang', 'v', 'k_meas')}
    R['r'] = f[-12:-4]
    runs.append(R)
print(f"runs {len(runs)}, {sum(len(r['ang']) for r in runs) / 6000:.0f} min")


def band_of(v):
  return next(i for i, (lo, hi) in enumerate(BANDS) if lo <= v < hi)


# 1. FIR on 20 Hz rates
NT = 30
for bi, (lo, hi) in enumerate(BANDS):
  X, Y = [], []
  for R in runs:
    tx = R['wire'][::5]; wh = R['ang'][::5]; vv = R['v'][::5]
    xr = np.diff(tx) / 0.05; yr = np.diff(wh) / 0.05
    for k in range(NT, len(xr)):
      if lo <= vv[k] < hi:
        X.append(xr[k - NT + 1:k + 1][::-1]); Y.append(yr[k])
  X = np.array(X); Y = np.array(Y)
  h = np.linalg.solve(X.T @ X + 50.0 * np.eye(NT), X.T @ Y)
  pk = np.argmax(h); first = np.argmax(h >= 0.1 * h.max()); cen = np.sum(np.arange(NT) * np.clip(h, 0, None)) / np.sum(np.clip(h, 0, None))
  r2 = 1 - np.var(Y - X @ h) / np.var(Y)
  print(f"[FIR {lo}-{hi} km/h] n {len(Y)} | delay first {first * 50} ms, peak {pk * 50} ms, centroid {cen * 50:.0f} ms | rate gain (sum) {h.sum():.2f} | R2 {r2:.2f}")
  print("   taps(50ms): " + " ".join(f"{x:+.2f}" for x in h[:16]))

# 2. steady state
ss = defaultdict(list)
for R in runs:
  tx = R['wire']; wh = R['ang']; vv = R['v']; la = (vv / 3.6) ** 2 * R['k_meas']
  for k in range(150, len(tx), 50):
    w = slice(k - 150, k)
    if np.ptp(tx[w]) < 0.5 and np.ptp(wh[w]) < 0.5 and abs(tx[k - 1]) >= 2:
      ss[band_of(vv[k - 1])].append((tx[k - 1], wh[k - 1], abs(la[k - 1])))
for bi, (lo, hi) in enumerate(BANDS):
  A = np.array(ss[bi])
  if not len(A):
    continue
  ratio = A[:, 1] / A[:, 0]; gap = (A[:, 0] - A[:, 1]) * np.sign(A[:, 0])
  line = f"[steady {lo}-{hi}] n {len(A)} wheel/TX p50 {np.median(ratio):.2f} (p25 {np.percentile(ratio, 25):.2f} p75 {np.percentile(ratio, 75):.2f}), gap p50 {np.median(gap):+.1f} deg"
  for la0, la1 in ((0, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 9)):
    m = (A[:, 2] >= la0) & (A[:, 2] < la1)
    if m.sum() >= 5:
      line += f" | a_lat {la0}-{la1}: n {m.sum()} ratio {np.median(ratio[m]):.2f} gap {np.median(gap[m]):+.1f}"
  for t0, t1 in ((2, 5), (5, 10), (10, 30), (30, 400)):
    m = (np.abs(A[:, 0]) >= t0) & (np.abs(A[:, 0]) < t1)
    if m.sum() >= 5:
      line += f" | |TX| {t0}-{t1}: ratio {np.median(ratio[m]):.2f}"
  print(line)

# 3. small moves from rest
sm = defaultdict(list)
for R in runs:
  tx = R['wire']; wh = R['ang']; vv = R['v']
  k = 100
  while k < len(tx) - 110:
    if np.ptp(tx[k - 50:k]) < 0.2 and np.ptp(wh[k - 50:k]) < 0.3:
      step = tx[k + 30] - tx[k]
      if 0.5 <= abs(step) <= 3 and np.ptp(tx[k + 30:k + 100]) < 0.3:
        s = np.sign(step)
        sm[band_of(vv[k])].append((abs(step), (wh[k + 50] - wh[k]) * s / abs(step), (wh[k + 100] - wh[k]) * s / abs(step)))
        k += 100; continue
    k += 5
for bi, (lo, hi) in enumerate(BANDS):
  A = np.array(sm[bi])
  if len(A) < 5:
    continue
  line = f"[small steps {lo}-{hi}] n {len(A)}"
  for s0, s1 in ((0.5, 1.0), (1.0, 2.0), (2.0, 3.01)):
    m = (A[:, 0] >= s0) & (A[:, 0] < s1)
    if m.sum() >= 3:
      line += f" | step {s0}-{s1}: n {m.sum()} wheel after 0.5 s {np.median(A[m, 1]):.2f}, 1.0 s {np.median(A[m, 2]):.2f} of the step, none (<10 %) {np.mean(A[m, 2] < 0.1):.0%}"
  print(line)

# 4. rate following
rf = defaultdict(list)
for R in runs:
  tx = R['wire']; wh = R['ang']; vv = R['v']
  for k in range(0, len(tx) - 70, 20):
    r1 = (tx[k + 40] - tx[k]) / 0.4; mid = (tx[k + 20] - tx[k]) / 0.2
    if abs(r1) >= 2 and abs(mid - r1) < 0.3 * abs(r1):
      wr = (wh[k + 55] - wh[k + 15]) / 0.4
      rf[band_of(vv[k])].append((abs(r1), wr * np.sign(r1)))
for bi, (lo, hi) in enumerate(BANDS):
  A = np.array(rf[bi])
  if len(A) < 5:
    continue
  line = f"[ramps {lo}-{hi}] n {len(A)}"
  for r0, r1 in ((2, 5), (5, 10), (10, 20), (20, 40), (40, 400)):
    m = (A[:, 0] >= r0) & (A[:, 0] < r1)
    if m.sum() >= 3:
      line += f" | TX {r0}-{r1} deg/s: n {m.sum()} wheel/TX rate {np.median(A[m, 1] / A[m, 0]):.2f}"
  print(line)

# 5. compact plant fit
rng = np.random.default_rng(0); idx = rng.permutation(len(runs)); half = set(idx[: len(runs) // 2].tolist())
for bi, (lo, hi) in enumerate(BANDS):
  best = None
  for dly in range(0, 40, 2):           # 10 ms units
    X, Y = [], []
    for j, R in enumerate(runs):
      if j not in half:
        continue
      tx = R['wire']; wh = R['ang']; vv = R['v']
      m = (vv[dly + 1:-1] >= lo) & (vv[dly + 1:-1] < hi)
      e = (tx[1:-1 - dly] - wh[dly + 1:-1])[m] if dly else (tx[1:-1] - wh[1:-1])[m]
      txr = ((tx[1:len(tx) - 1 - dly] - tx[0:len(tx) - 2 - dly]) / 0.01)[m] if dly else ((tx[1:-1] - tx[:-2]) / 0.01)[m]
      y = ((wh[dly + 2:] - wh[dly + 1:-1]) / 0.01)[m]
      wc = wh[dly + 1:-1][m]
      X.append(np.stack([e, txr, wc], 1)); Y.append(y)
    X = np.concatenate(X); Y = np.concatenate(Y)
    if len(Y) < 1000:
      break
    # smooth targets: wheel rate is quantised; fit on 0.1 s averaged rates
    kk = np.ones(10) / 10
    Ys = np.convolve(Y, kk, 'same'); Xs = np.stack([np.convolve(X[:, j], kk, 'same') for j in range(3)], 1)
    th, *_ = np.linalg.lstsq(Xs, Ys, rcond=None)
    res = np.mean((Ys - Xs @ th) ** 2)
    if best is None or res < best[0]:
      best = (res, dly, th)
  if best is None:
    continue
  _, dly, (kp, kf, kc) = best
  # Rmax and held-out simulation
  allr = np.concatenate([np.abs(np.diff(R['ang'][::10])) / 0.1 for R in runs])
  rmax = 200.0
  se_m, se_b, nn = 0.0, 0.0, 0
  for j, R in enumerate(runs):
    if j in half:
      continue
    tx = R['wire']; wh = R['ang']; vv = R['v']
    if not (lo <= np.median(vv) < hi):
      continue
    sim = np.empty_like(wh); sim[:dly + 1] = wh[:dly + 1]
    for k in range(dly + 1, len(wh)):
      txr = (tx[k - 1 - dly] - tx[k - 2 - dly]) / 0.01 if k - 2 - dly >= 0 else 0.0
      rate = np.clip(kp * (tx[k - 1 - dly] - sim[k - 1]) + kf * txr + kc * sim[k - 1], -rmax, rmax)
      sim[k] = sim[k - 1] + rate * 0.01
    base = np.concatenate([wh[:dly], tx[:len(tx) - dly]]) if dly else tx.copy()
    se_m += np.sum((sim - wh) ** 2); se_b += np.sum((base - wh) ** 2); nn += len(wh)
  print(f"[plant {lo}-{hi}] delay {dly * 10} ms, Kp {kp:.2f} 1/s (time constant {1 / max(kp, 1e-3):.2f} s), Kf {kf:.2f}, centring {kc:+.3f} 1/s "
        f"| held-out sim RMS {np.sqrt(se_m / max(nn, 1)):.2f} deg vs 'wheel = TX(t-d)' {np.sqrt(se_b / max(nn, 1)):.2f} deg (n {nn / 6000:.0f} min)")
