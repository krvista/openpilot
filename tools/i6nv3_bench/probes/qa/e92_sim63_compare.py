"""E92: Phase 63 sim-matrix summary from e64 rows (e91_run63.sh): per variant, by speed band — intent error RMS
(car curvature vs the model action 0.375 s earlier), lateral jerk p95, 0.2-1 Hz lateral-acceleration weave RMS on
straights (|K| < 0.5e-3), steady-curve delivery (car / model curvature, |K| >= 0.5e-3 held 1 s, after 1.5 s), and
corner ENTRY (same rule as e68b: K from < 0.7e-3 held 1 s to a peak >= 2e-3 within 3 s): car behind model at 50 %
and car / model at the model peak. argv: S plant tag1 tag2 ... (first tag = reference)."""
import sys, pickle
import numpy as np
S, PL = sys.argv[1], sys.argv[2]; TAGS = sys.argv[3:]
BANDS = [(30, 55), (55, 70), (70, 90), (90, 200)]
def bpf(x, lo, hi):
  X = np.fft.rfft(x - x.mean()); f = np.fft.rfftfreq(len(x), 0.01); X[(f < lo) | (f >= hi)] = 0; return np.fft.irfft(X, len(x))
def summ(rows):
  M = {i: dict(e=[], j=[], w=[], dl=[], lag=[], pk=[]) for i in range(len(BANDS))}
  for r, a, fold, K, v, res in rows:
    kcmd, kcar = res["Ce+0.0"]
    vm = v / 3.6; lat = vm ** 2 * kcar; n = len(K)
    bi = np.full(n, -1)
    for i, (lo, hi) in enumerate(BANDS):
      bi[(v >= lo) & (v < hi)] = i
    e = np.full(n, np.nan); e[38:] = kcar[38:] - K[:-38]
    j = np.zeros(n); j[10:] = np.abs(lat[10:] - lat[:-10]) / 0.1
    wv = bpf(lat, 0.2, 1.0)
    big = np.abs(K) >= 0.5e-3; cs = np.concatenate([[0], np.cumsum(big)]); idx = np.arange(n)
    held = (cs[idx + 1] - cs[np.maximum(idx - 149, 0)]) >= np.minimum(idx + 1, 150)
    for i in range(len(BANDS)):
      m = bi == i
      M[i]['e'].append(e[m & np.isfinite(e)]); M[i]['j'].append(j[m])
      M[i]['w'].append(wv[m & (np.abs(K) < 0.5e-3)])
      q = m & held & (np.sign(kcar) == np.sign(K))
      M[i]['dl'].append(kcar[q] / K[q])
    aK = np.abs(K); k = 100
    while k < n - 300:
      if aK[k] < 0.7e-3 and (aK[k - 100:k] < 0.7e-3).all():
        pk = k + int(np.argmax(aK[k:k + 300]))
        if aK[pk] >= 2e-3 and pk + 150 < n:
          s = np.sign(K[pk]); kp = aK[pk]
          k50 = next((t for t in range(k, pk + 1) if K[t] * s >= 0.5 * kp), None)
          y50 = next((t for t in range(k, min(pk + 300, n)) if kcar[t] * s >= 0.5 * kp), None)
          b = next((i for i, (lo, hi) in enumerate(BANDS) if lo <= np.median(v[k:pk + 1]) < hi), None)
          if k50 is not None and b is not None:
            M[b]['lag'].append((y50 - k50) * 0.01 if y50 is not None else np.nan); M[b]['pk'].append(kcar[pk] * s / kp)
        k = pk + 300; continue
      k += 10
  out = {}
  for i in range(len(BANDS)):
    c = lambda key: np.concatenate(M[i][key]) if M[i][key] else np.array([np.nan])
    out[i] = dict(e=np.sqrt(np.nanmean(c('e') ** 2)) * 1e3, j=np.nanpercentile(c('j'), 95), w=np.sqrt(np.nanmean(c('w') ** 2)),
                  dl=np.nanmedian(c('dl')), n=len(M[i]['lag']), lag=np.nanmedian(M[i]['lag']) * 1e3 if M[i]['lag'] else np.nan,
                  pk=np.nanmedian(M[i]['pk']) if M[i]['pk'] else np.nan)
  return out
R = {t: summ(pickle.load(open(f"{S}/qa/e64_rows_63{PL}_{t}.pkl", "rb"))) for t in TAGS}
print(f"plant {PL}: intent err e-3 | jerk p95 | straight weave | curve delivery | entries n, car behind model ms, car/model at peak")
for i, (lo, hi) in enumerate(BANDS):
  print(f"[{lo}-{hi}]")
  ref = R[TAGS[0]][i]
  for t in TAGS:
    o = R[t][i]
    print(f"  {t:5s} err {o['e']:.3f} | jerk {o['j']:.2f} ({o['j'] / ref['j']:.2f}x) | weave {o['w']:.3f} ({o['w'] / ref['w']:.2f}x) | deliv {o['dl']:.2f} | entries {o['n']:3d} lag {o['lag']:4.0f} ms peak {o['pk']:.2f}")
