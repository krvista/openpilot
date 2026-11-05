"""E72: per-band comparison of the E64 chain-sim rows: intent error RMS, lateral jerk p95 and 0.2-1 Hz lateral-acceleration
weave RMS, vs the Phase 61 reference (B0e+0.3 in e64_rows_b0.pkl). Pass = every band's jerk and weave <= 1.05 x Phase 61."""
import glob
import pickle
import sys
import numpy as np
S = sys.argv[1]
BANDS = [(30, 40), (40, 55), (55, 70), (70, 90), (90, 200)]


def metrics(rows, var):
  J = {i: [] for i in range(5)}; Wv = {i: [] for i in range(5)}; E = {i: [] for i in range(5)}; lags = []
  for r, a, fold, K, v, res in rows:
    if var not in res:
      continue
    kcmd, kcar = res[var]
    j = np.abs(np.diff(((v / 3.6) ** 2 * kcar)[::10])) / 0.1; vv = v[::10][1:]
    X = np.fft.rfft(kcar - kcar.mean()); f = np.fft.rfftfreq(len(kcar), 0.01); X[(f < 0.2) | (f >= 1.0)] = 0; wb = np.fft.irfft(X, len(kcar))
    e = kcar[38:] - K[:-38]; ve = v[38:]
    for i, (lo, hi) in enumerate(BANDS):
      m = (vv >= lo) & (vv < hi); m2 = (v >= lo) & (v < hi); m3 = (ve >= lo) & (ve < hi)
      if m.any():
        J[i].append(j[m])
      if m2.any():
        Wv[i].append((v[m2] / 3.6) ** 2 * wb[m2])
      if m3.any():
        E[i].append(e[m3])
    if len(K) >= 1000 and np.std(K) > 0.3e-3:
      x = K - K.mean(); best = (-2, 0)
      for L in range(0, 100, 2):
        y = kcar[L:] - kcar[L:].mean(); xx = x[:len(y)]
        cc = np.dot(xx, y) / (np.linalg.norm(xx) * np.linalg.norm(y) + 1e-12)
        if cc > best[0]:
          best = (cc, L)
      if best[0] > 0.7:
        lags.append(best[1] * 10)
  cat = lambda D: [np.concatenate(D[i]) if D[i] else np.zeros(1) for i in range(5)]
  Jc, Wc, Ec = cat(J), cat(Wv), cat(E)
  allE = np.concatenate(Ec)
  return dict(rms=np.sqrt(np.mean(allE ** 2)), band_rms=[np.sqrt(np.mean(x ** 2)) for x in Ec], jerk=[np.percentile(x, 95) for x in Jc],
              weave=[np.sqrt(np.mean(x ** 2)) for x in Wc], lag=np.median(lags) if lags else np.nan)


ref = metrics(pickle.load(open(f"{S}/qa/e64_rows_b0.pkl", "rb")), "B0e+0.3")
fmt = lambda L, f="{:.3f}": " ".join(f.format(x) for x in L)
print(f"{'P61 (ref)':10} intent {ref['rms'] * 1e3:.3f} | lag {ref['lag']:.0f} | band RMS {fmt([x * 1e3 for x in ref['band_rms']])} | jerk {fmt(ref['jerk'], '{:.2f}')} | weave {fmt(ref['weave'])}")
for f in sorted(glob.glob(f"{S}/qa/e64_rows_C*.pkl")):
  tag = f.split("e64_rows_")[1][:-4]
  m = metrics(pickle.load(open(f, "rb")), "Ce+0.0")
  ok = all(m['jerk'][i] <= 1.05 * ref['jerk'][i] and m['weave'][i] <= 1.05 * ref['weave'][i] for i in range(5))
  worst = max(max(m['jerk'][i] / ref['jerk'][i], m['weave'][i] / ref['weave'][i]) for i in range(5))
  print(f"{tag:10} intent {m['rms'] * 1e3:.3f} | lag {m['lag']:.0f} | band RMS {fmt([x * 1e3 for x in m['band_rms']])} | jerk {fmt(m['jerk'], '{:.2f}')} | weave {fmt(m['weave'])} "
        f"| {'PASS' if ok else 'fail'} (worst band x{worst:.2f})")
