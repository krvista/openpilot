"""E136 (report §92): the entry / exit timing metric of E133, on the REAL drives instead of the chain sim: model action
curvature vs the car's measured curvature in hands-off windows (the chain sim's window rule), per build and speed band.
Entries: model |K| from < 0.7e-3 (held 1 s) to a peak >= 2e-3 within 3 s; exits: model |K| >= 2e-3 held 1 s falling to
<= 0.7e-3 within 3 s. 'car after model' = when the car reaches the half-way point minus when the model does (the action
is meant for 375 ms later: 375 = on time); entry: car / model at the model's peak; exit: share of the curve the car
still holds 0.38 s after the model is half-way out (0.5 = on the model). argv: S builds_csv"""
import glob, sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; BUILDS = sys.argv[2].split(",")
BANDS = [(30, 55), (55, 70), (70, 90), (90, 200)]
for b in BUILDS:
  M = {i: dict(lag=[], pk=[], xlag=[], xhold=[]) for i in range(len(BANDS))}; mins = 0.0
  for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
    d = dict(np.load(f, allow_pickle=True)); n = len(d['T'])
    if str(d['commit'])[:7] != b or n < 2000: continue
    v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
    hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
    dd = np.maximum(0.0, np.abs(d['tq_s']) - hold); bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
    ok = d['lat'] & (v >= 30) & (dd < 40) & ~d['pressed'] & ~bl; ok[:1200] = False
    e = np.flatnonzero(np.diff(np.concatenate([[0], ok.astype(int), [0]])))
    for a0, b0 in zip(e[::2], e[1::2]):
      if b0 - a0 < 1000: continue
      K = d['k_model'][a0:b0]; kcar = d['k_meas'][a0:b0]; vv = v[a0:b0]; m = len(K); aK = np.abs(K); mins += m / 6000
      k = 100
      while k < m - 300:
        if aK[k] < 0.7e-3 and (aK[k - 100:k] < 0.7e-3).all():
          pk = k + int(np.argmax(aK[k:k + 300]))
          if aK[pk] >= 2e-3 and pk + 150 < m:
            s = np.sign(K[pk]); kp = aK[pk]
            k50 = next((t for t in range(k, pk + 1) if K[t] * s >= 0.5 * kp), None)
            y50 = next((t for t in range(k, min(pk + 300, m)) if kcar[t] * s >= 0.5 * kp), None)
            bi = next((i for i, (lo, hi) in enumerate(BANDS) if lo <= np.median(vv[k:pk + 1]) < hi), None)
            if k50 is not None and bi is not None:
              M[bi]['lag'].append((y50 - k50) * 0.01 if y50 is not None else np.nan); M[bi]['pk'].append(kcar[pk] * s / kp)
          k = pk + 300; continue
        k += 10
      k = 100
      while k < m - 450:
        if aK[k] >= 2e-3 and (aK[k - 100:k] >= 2e-3).all() and np.all(np.sign(K[k - 100:k + 1]) == np.sign(K[k])):
          s = np.sign(K[k]); k0 = np.median(aK[k - 100:k]); seg = K[k:k + 300] * s; lo_i = np.flatnonzero(seg <= 0.7e-3)
          if len(lo_i) and seg[:lo_i[0]].max() <= 1.25 * k0:
            end = k + lo_i[0]
            m50 = next((t for t in range(k, end + 1) if K[t] * s <= 0.5 * k0), None); c0 = np.median(kcar[k - 100:k] * s)
            y50 = next((t for t in range(k, min(end + 300, m)) if kcar[t] * s <= 0.5 * c0), None) if c0 > 0.5e-3 else None
            bi = next((i for i, (lo, hi) in enumerate(BANDS) if lo <= np.median(vv[k:end + 1]) < hi), None)
            if m50 is not None and bi is not None and c0 > 0.5e-3:
              M[bi]['xlag'].append((y50 - m50) * 0.01 if y50 is not None else np.nan); t = min(m50 + 38, m - 1); M[bi]['xhold'].append(kcar[t] * s / c0)
            k = end + 200; continue
        k += 10
  print(f"=== {b}: hands-off windows {mins:.0f} min")
  for i, (lo, hi) in enumerate(BANDS):
    o = M[i]; md = lambda key: np.nanmedian(o[key]) if o[key] else np.nan
    q = lambda key, p: np.nanpercentile(o[key], p) if o[key] else np.nan
    print(f"  [{lo}-{hi}] entries {len(o['lag']):3d}: car after model {md('lag') * 1e3:4.0f} ms [{q('lag', 25) * 1e3:.0f}-{q('lag', 75) * 1e3:.0f}], car/model at peak {md('pk'):.2f} | "
          f"exits {len(o['xlag']):3d}: car after model {md('xlag') * 1e3:4.0f} ms [{q('xlag', 25) * 1e3:.0f}-{q('xlag', 75) * 1e3:.0f}], curve still held {md('xhold'):.2f}")
