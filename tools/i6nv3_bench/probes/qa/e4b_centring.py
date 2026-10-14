"""E4b: does the 0.9 s plan yaw-rate sample keep the model's lane-centring correction? On straights / mild roads
(|k_road| < 1e-3, both lines > 0.7, lat active, >= 30 km/h): regress (predictor - road curvature) on the car's lateral
offset from lane centre and on its heading error (lane-centre slope at the car). A centring controller has negative slopes
(offset to +y -> curvature toward -y). Compare the model action, the cubic lead and the orient lead; and the future offset:
does the plan's own 2 s position (PY at 2 s minus lane centre at that x) still close the offset?"""
import sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from common import routes, T_IDXS

S = sys.argv[1]
rows = []
for r, P, d, T, f in routes(S):
  n = len(T); v = f['v']
  K = P['K'].astype(float)
  OR, VX = P['OR'].astype(float), P['VX'].astype(float)
  tau = np.interp(v, [8.0, 13.0, 18.0], [0.30, 0.12, 0.08])
  k_or = np.array([np.interp(0.9 + tau[i], T_IDXS, OR[i]) / max(np.interp(0.9 + tau[i], T_IDXS, VX[i]), 1.0) for i in range(n)])
  LX = P['LX'].astype(float); LC = 0.5 * (P['L1'].astype(float) + P['L2'].astype(float))
  head = np.array([(np.interp(5.0, LX[i], LC[i]) - np.interp(0.0, LX[i], LC[i])) / 5.0 for i in range(n)])   # lane-centre slope
  off = -f['c0']
  # plan's lateral position relative to lane centre at 2 s
  PX, PY = P['PX'].astype(float), P['PY'].astype(float)
  x2 = PX[:, 13] if True else None   # T_IDXS[13] = 1.65 s
  p_off2 = PY[:, 13] - np.array([np.interp(x2[i], LX[i], LC[i]) for i in range(n)])
  s = f['good'] & f['lat'] & (np.abs(f['k_road']) < 1e-3)
  rows.append(np.stack([v[s] * 3.6, off[s], head[s], (K - f['k_road'])[s], (k_or - f['k_road'])[s], p_off2[s]], 1))
  print(r, flush=True)
A = np.concatenate(rows)
print("frames", len(A))
for lo, hi in ((30, 60), (60, 90), (90, 200)):
  s = (A[:, 0] >= lo) & (A[:, 0] < hi)
  X = np.stack([A[s, 1], A[s, 2], np.ones(s.sum())], 1)
  for c, nm in ((3, "action"), (4, "orient0.9")):
    co, *_ = np.linalg.lstsq(X, A[s, c], rcond=None)
    pred = X @ co
    r2 = 1 - np.var(A[s, c] - pred) / np.var(A[s, c])
    print(f"  {lo:3}-{hi:<3} n {s.sum():6d} {nm:9}: d(k)/d(offset) {co[0]*1e3:+.3f}e-3/m per m, d(k)/d(heading) {co[1]*1e3:+.2f}e-3 per rad, R2 {r2:.2f}")
  co = np.polyfit(A[s, 1], A[s, 5], 1)
  print(f"           plan offset at 1.65 s vs offset now: slope {co[0]:+.2f} (1 = no correction, 0 = fully centred)")
