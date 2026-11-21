"""E154 (report §95): the chain sim's own timing by speed band, to hold against the real drives (E150 / E143). From E64 rows
(qa/e64_rows_63<plant>_n_<tag>.pkl, reference = action re-timed to 0.375 s): per window the lag of the simulated car behind
the reference (cross-correlation, 0-1.2 s; 375 ms = on time) and behind the replayed command, and the intent error.
The difference to the real car's figures on the same drives is how far the identified plant is off.
argv: S plant tag [tag ...]"""
import pickle
import sys
import numpy as np
S = sys.argv[1]; PL = sys.argv[2]
BANDS = [(30, 55), (55, 70), (70, 90), (90, 200)]
for tag in sys.argv[3:]:
  rows = pickle.load(open(f"{S}/qa/e64_rows_63{PL}_n_{tag}.pkl", "rb"))
  M = {i: dict(lag=[], e=[], cl=[]) for i in range(len(BANDS))}
  for r, a, fold, K, v, res in rows:
    kcmd, kcar = res["Ce+0.0"]; n = len(K)
    err = np.full(n, np.nan); err[38:] = kcar[38:] - K[:-38]
    for i, (lo, hi) in enumerate(BANDS):
      M[i]['e'].append(err[(v >= lo) & (v < hi) & np.isfinite(err)])
    bi = next((i for i, (lo, hi) in enumerate(BANDS) if lo <= np.median(v) < hi), None)
    if bi is None or n < 1000 or np.std(K) < 0.3e-3: continue
    def lag(x, y, hi=120):
      xs = x - x.mean(); best = (-2, 0)
      for L in range(0, hi, 2):
        ys = y[L:] - y[L:].mean(); xx = xs[:len(ys)]
        cc = np.dot(xx, ys) / (np.linalg.norm(xx) * np.linalg.norm(ys) + 1e-12)
        if cc > best[0]: best = (cc, L)
      return best
    c, L = lag(K, kcar)
    if c > 0.7: M[bi]['lag'].append(L * 10)
    c, L = lag(kcmd, kcar)
    if c > 0.7: M[bi]['cl'].append(L * 10)
  print(f"{PL} {tag:6s} " + " | ".join(f"[{lo}-{hi}] car after A0 {np.median(M[i]['lag']) if M[i]['lag'] else np.nan:4.0f} ms ({len(M[i]['lag'])}), car behind command {np.median(M[i]['cl']) if M[i]['cl'] else np.nan:4.0f}, err {np.sqrt(np.nanmean(np.concatenate(M[i]['e']) ** 2)) * 1e3:.3f}" for i, (lo, hi) in enumerate(BANDS)))
