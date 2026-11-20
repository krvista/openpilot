"""E140 (report §94): E136's entry / exit timing on the real drives, measured against what the model was actually told.
The model's action is the planned curvature for lat_action_t = lateralDelay + ~0.075 s later (modeld: frame + action delay),
and lateralDelay is no longer a constant: 0.30 s up to bf36aa2, the lagd seed 0.65 s and then the learned value from Phase
66 / 68 on. So 'car after model' is on time when it EQUALS lat_action_t, and builds are only comparable by the difference:
  late = (car reaches half-way) - (model action reaches half-way) - lat_action_t        (0 = on the plan, + = the car is late)
Same hands-off windows and entry / exit rules as E136 (entries: model |K| from < 0.7e-3 held 1 s to a peak >= 2e-3 within
3 s; exits: >= 2e-3 held 1 s falling to <= 0.7e-3 within 3 s). Also per band: the intent error RMS(car K(t) - action
K(t - lat_action_t)) with the same reference, and how far the wire request ran ahead of controlsd's command.
argv: S builds_csv"""
import glob
import sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; BUILDS = sys.argv[2].split(",")
BANDS = [(30, 55), (55, 70), (70, 90), (90, 200)]
EXTRA = 0.075   # frame + action delay added to lateralDelay by modeld
for b in BUILDS:
  M = {i: dict(lag=[], late=[], tgt=[], pk=[], due=[], xlag=[], xlate=[], xhold=[], ie=[]) for i in range(len(BANDS))}; mins = 0.0; dl = []
  for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
    d = dict(np.load(f, allow_pickle=True)); n = len(d['T']); r = f[-12:-4]
    if str(d['commit'])[:7] != b or n < 2000: continue
    try:
      ld = np.load(f"{S}/vm/{r}.npz", allow_pickle=True)['ld']
    except Exception:  # noqa: BLE001
      continue
    if len(ld) < 10: continue
    T = d['T']
    tgt_all = np.interp(T, ld[:, 0], ld[:, 1]) + EXTRA
    v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
    hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
    dd = np.maximum(0.0, np.abs(d['tq_s']) - hold); bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
    ok = d['lat'] & (v >= 30) & (dd < 40) & ~d['pressed'] & ~bl; ok[:1200] = False
    # intent error with the per-sample reference
    sh = np.round(tgt_all * 100).astype(int); idx = np.arange(n) - sh
    ie = np.where(idx >= 0, d['k_meas'] - d['k_model'][np.clip(idx, 0, n - 1)], np.nan)
    e = np.flatnonzero(np.diff(np.concatenate([[0], ok.astype(int), [0]])))
    for a0, b0 in zip(e[::2], e[1::2]):
      if b0 - a0 < 1000: continue
      K = d['k_model'][a0:b0]; kcar = d['k_meas'][a0:b0]; vv = v[a0:b0]; m = len(K); aK = np.abs(K); mins += m / 6000
      tg = tgt_all[a0:b0]; dl.append(tg)
      for i, (lo, hi) in enumerate(BANDS):
        sel = (vv >= lo) & (vv < hi)
        M[i]['ie'].append(ie[a0:b0][sel])
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
              lag = (y50 - k50) * 0.01 if y50 is not None else np.nan
              M[bi]['lag'].append(lag); M[bi]['tgt'].append(tg[k50]); M[bi]['late'].append(lag - tg[k50]); M[bi]['pk'].append(kcar[pk] * s / kp)
              due = min(pk + int(round(tg[pk] * 100)), m - 1)          # when the peak is due on the road
              M[bi]['due'].append(kcar[due] * s / kp)
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
              lag = (y50 - m50) * 0.01 if y50 is not None else np.nan
              M[bi]['xlag'].append(lag); M[bi]['xlate'].append(lag - tg[m50])
              t = min(m50 + int(round(tg[m50] * 100)), m - 1); M[bi]['xhold'].append(kcar[t] * s / c0)
            k = end + 200; continue
        k += 10
  dl = np.concatenate(dl) if dl else np.array([np.nan])
  print(f"=== {b}: hands-off windows {mins:.0f} min; model action due after {np.nanmedian(dl) * 1e3:.0f} ms (p5-p95 {np.nanpercentile(dl, 5) * 1e3:.0f}-{np.nanpercentile(dl, 95) * 1e3:.0f})")
  for i, (lo, hi) in enumerate(BANDS):
    o = M[i]; md = lambda key: np.nanmedian(o[key]) if o[key] else np.nan
    q = lambda key, p: np.nanpercentile(o[key], p) if o[key] else np.nan
    ie = np.concatenate(o['ie']) if o['ie'] else np.array([np.nan])
    print(f"  [{lo}-{hi}] entries {len(o['lag']):3d}: car after model {md('lag') * 1e3:4.0f} ms, due {md('tgt') * 1e3:.0f} -> late {md('late') * 1e3:+5.0f} ms [{q('late', 25) * 1e3:+.0f} {q('late', 75) * 1e3:+.0f}], "
          f"car/model when the peak is due {md('due'):.2f} | exits {len(o['xlag']):3d}: car after model {md('xlag') * 1e3:4.0f} ms -> late {md('xlate') * 1e3:+5.0f} ms [{q('xlate', 25) * 1e3:+.0f} {q('xlate', 75) * 1e3:+.0f}], "
          f"curve still held when half-out is due {md('xhold'):.2f} | intent err {np.sqrt(np.nanmean(ie ** 2)) * 1e3:.3f}e-3")
