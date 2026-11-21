"""E150 (report §95): the chain sim's yardstick measured on the REAL drives, for builds whose logged action was computed
for different delays. Reference A0 = the action re-timed to T = 0.375 s (K + (2 / v) (psi(0.375) / 0.375 - psi(T) / T),
T = lagd's published delay + 0.075; equal to the logged action on builds told 0.30 s) - so 'car after model' has the same
meaning as in E64 / E133 / E136: 375 ms = on time. Hands-off stretches as the sim's windows (>= 10 s, <= 60 s chunks).
Per build and speed band: intent error RMS(car K(t) - A0(t - 0.375)), lag of the car behind A0 (cross-correlation per
stretch, 0-1.2 s), and E136's entries / exits on A0. argv: S builds_csv"""
import glob
import sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; BUILDS = sys.argv[2].split(",")
BANDS = [(30, 55), (55, 70), (70, 90), (90, 200)]
TI = np.array([10.0 * (i / 32) ** 2 for i in range(33)])
for b in BUILDS:
  M = {i: dict(e=[], lag=[], el=[], pk=[], xl=[], xh=[]) for i in range(len(BANDS))}; mins = 0.0
  for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
    d = dict(np.load(f, allow_pickle=True)); n = len(d['T']); r = f[-12:-4]
    if str(d['commit'])[:7] != b or n < 2000: continue
    T = d['T']; v = d['v']; vm = v / 3.6; K = d['k_model'].astype(float)
    try:
      P = np.load(f"{S}/la/plan_{r}.npz"); ld = np.load(f"{S}/vm/{r}.npz", allow_pickle=True)['ld']
    except Exception:  # noqa: BLE001
      continue
    told = np.interp(T, ld[:, 0], ld[:, 1]) if len(ld) > 10 else np.full(n, 0.3)
    if np.nanmax(np.abs(told - 0.3)) > 1e-3:
      if 'PSI' not in P.files: continue
      j = np.clip(np.searchsorted(P['T'], T, side="right") - 1, 0, len(P['T']) - 1)
      toldf = np.array([ld[max(np.searchsorted(ld[:, 0], t, side='right') - 1, 0), 1] for t in P['T']])
      tt = toldf + 0.075; vf = np.maximum(np.interp(P['T'], T, vm), 1.0)
      psi_t = np.array([np.interp(t_, TI, p_) for t_, p_ in zip(tt, P['PSI'])]); psi_0 = np.array([np.interp(0.375, TI, p_) for p_ in P['PSI']])
      K = (P['K'] + (2.0 / vf) * (psi_0 / 0.375 - psi_t / tt))[j]
    la = np.abs(vm ** 2 * d['k_meas'])
    hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
    dd = np.maximum(0.0, np.abs(d['tq_s']) - hold); bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
    ok = d['lat'] & (v >= 30) & (dd < 40) & ~d['pressed'] & ~bl; ok[:1200] = False
    e = np.flatnonzero(np.diff(np.concatenate([[0], ok.astype(int), [0]])))
    for a0, b0 in zip(e[::2], e[1::2]):
      if b0 - a0 < 1000: continue
      Kw = K[a0:b0]; kcar = d['k_meas'][a0:b0]; vv = v[a0:b0]; m = len(Kw); aK = np.abs(Kw); mins += m / 6000
      err = np.full(m, np.nan); err[38:] = kcar[38:] - Kw[:-38]
      for i, (lo, hi) in enumerate(BANDS):
        sel = (vv >= lo) & (vv < hi) & np.isfinite(err); M[i]['e'].append(err[sel])
      for c0 in range(0, m - 999, 3000):                      # lag per <= 60 s chunk, as the sim's summary
        c1 = min(c0 + 6000, m); x = Kw[c0:c1]; y = kcar[c0:c1]
        if c1 - c0 < 1000 or np.std(x) < 0.3e-3: continue
        xs = x - x.mean(); best = (-2, 0)
        for L in range(0, 120, 2):
          ys = y[L:] - y[L:].mean(); xx = xs[:len(ys)]
          cc = np.dot(xx, ys) / (np.linalg.norm(xx) * np.linalg.norm(ys) + 1e-12)
          if cc > best[0]: best = (cc, L)
        bi = next((i for i, (lo, hi) in enumerate(BANDS) if lo <= np.median(vv[c0:c1]) < hi), None)
        if best[0] > 0.7 and bi is not None: M[bi]['lag'].append(best[1] * 10)
      k = 100
      while k < m - 300:
        if aK[k] < 0.7e-3 and (aK[k - 100:k] < 0.7e-3).all():
          pk = k + int(np.argmax(aK[k:k + 300]))
          if aK[pk] >= 2e-3 and pk + 150 < m:
            s = np.sign(Kw[pk]); kp = aK[pk]
            k50 = next((t for t in range(k, pk + 1) if Kw[t] * s >= 0.5 * kp), None)
            y50 = next((t for t in range(k, min(pk + 300, m)) if kcar[t] * s >= 0.5 * kp), None)
            bi = next((i for i, (lo, hi) in enumerate(BANDS) if lo <= np.median(vv[k:pk + 1]) < hi), None)
            if k50 is not None and bi is not None:
              M[bi]['el'].append((y50 - k50) * 10 if y50 is not None else np.nan); M[bi]['pk'].append(kcar[pk] * s / kp)
          k = pk + 300; continue
        k += 10
      k = 100
      while k < m - 450:
        if aK[k] >= 2e-3 and (aK[k - 100:k] >= 2e-3).all() and np.all(np.sign(Kw[k - 100:k + 1]) == np.sign(Kw[k])):
          s = np.sign(Kw[k]); k0 = np.median(aK[k - 100:k]); seg = Kw[k:k + 300] * s; lo_i = np.flatnonzero(seg <= 0.7e-3)
          if len(lo_i) and seg[:lo_i[0]].max() <= 1.25 * k0:
            end = k + lo_i[0]
            m50 = next((t for t in range(k, end + 1) if Kw[t] * s <= 0.5 * k0), None); c0 = np.median(kcar[k - 100:k] * s)
            y50 = next((t for t in range(k, min(end + 300, m)) if kcar[t] * s <= 0.5 * c0), None) if c0 > 0.5e-3 else None
            bi = next((i for i, (lo, hi) in enumerate(BANDS) if lo <= np.median(vv[k:end + 1]) < hi), None)
            if m50 is not None and bi is not None and c0 > 0.5e-3:
              M[bi]['xl'].append((y50 - m50) * 10 if y50 is not None else np.nan); M[bi]['xh'].append(kcar[min(m50 + 38, m - 1)] * s / c0)
            k = end + 200; continue
        k += 10
  print(f"=== {b}: hands-off stretches {mins:.0f} min   (375 ms = on time)")
  for i, (lo, hi) in enumerate(BANDS):
    o = M[i]; md = lambda k: np.nanmedian(o[k]) if o[k] else np.nan
    ee = np.concatenate(o['e']) if o['e'] else np.array([np.nan])
    print(f"  [{lo}-{hi}] intent err {np.sqrt(np.nanmean(ee ** 2)) * 1e3:.3f}e-3 | car after A0 (xcorr, {len(o['lag'])} stretches) {md('lag'):4.0f} ms | "
          f"entries {len(o['el']):2d}: {md('el'):4.0f} ms, car/model at peak {md('pk'):.2f} | exits {len(o['xl']):2d}: {md('xl'):4.0f} ms, still held {md('xh'):.2f}")
