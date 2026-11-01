"""E64b: route-fold cross-validated choice of the per-band delay-match E (Phase 60) from the E64 rows, for Phase 59 preview off
(P0) and on (P1): choose E per band on one fold's windows, score on the other fold; compare with base / p59."""
import pickle
import sys
import numpy as np
S = sys.argv[1]
sys.path.insert(0, S + "/qa")
rows = pickle.load(open(f"{S}/qa/e64_rows.pkl", "rb"))
BANDS = [(30, 40), (40, 55), (55, 70), (70, 90), (90, 200)]
EG = sorted({float(v[3:]) for v in rows[0][5] if v.startswith("P")})


def band_err(rs, var):
  se = np.zeros(5); n = np.zeros(5)
  for r, a, fold, K, v, res in rs:
    e = res[var][1][38:] - K[:-38]; vv = v[38:]
    for i, (lo, hi) in enumerate(BANDS):
      m = (vv >= lo) & (vv < hi); se[i] += np.sum(e[m] ** 2); n[i] += m.sum()
  return se, n


for P in ("P0", "P1"):
  tot_se = np.zeros(5); tot_n = np.zeros(5); picks = []
  for tr in (0, 1):
    A = [x for x in rows if x[2] == tr]; B = [x for x in rows if x[2] != tr]
    choice = []
    for i in range(5):
      best = min(EG, key=lambda e: (band_err(A, f"{P}e{e:+.1f}")[0][i] / max(band_err(A, f"{P}e{e:+.1f}")[1][i], 1)))
      choice.append(best)
    picks.append(choice)
    for i in range(5):
      se, n = band_err(B, f"{P}e{choice[i]:+.1f}"); tot_se[i] += se[i]; tot_n[i] += n[i]
  cv = np.sqrt(tot_se / np.maximum(tot_n, 1))
  print(f"{P} cross-validated: picks fold0 {picks[0]} fold1 {picks[1]} | band RMS " + " ".join(f"{x * 1e3:.3f}" for x in cv)
        + f" | all {np.sqrt(tot_se.sum() / tot_n.sum()) * 1e3:.3f}e-3")
  # all-data choice for shipping
  ch = [min(EG, key=lambda e: band_err(rows, f"{P}e{e:+.1f}")[0][i] / max(band_err(rows, f"{P}e{e:+.1f}")[1][i], 1)) for i in range(5)]
  print(f"   all-data picks {ch}")
for var in ("base", "p59"):
  se, n = band_err(rows, var)
  print(f"{var:5}: band RMS " + " ".join(f"{x * 1e3:.3f}" for x in np.sqrt(se / np.maximum(n, 1))) + f" | all {np.sqrt(se.sum() / n.sum()) * 1e3:.3f}e-3")
