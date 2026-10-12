"""§43 delay budget on route_extract npz (100 Hz): cross-correlation lags in 2 s windows (lat active, no blinker / lane change,
controlsd curvature band >= 1e-3), split by hand state — strict hands-off (gain >= 0.9, |tq| < 50, never pressed), resting
hand (gain 0.5-0.9, pressed < 20 %), driver holding (pressed > 80 %). Lags: transmitted wire angle -> wheel (MDPS), model
action curvature (logged ~50 ms after frame EOF) -> wheel. The model assumes action_t = lat_delay + 0.075 s.
Usage: lag_budget.py <dir with 0000*.npz>"""
import glob, sys
import numpy as np
S = sys.argv[1]
def xlag(a, b, maxlag=120):
  a = a - a.mean(); b = b - b.mean()
  if a.std() < 1e-9 or b.std() < 1e-9: return None, 0
  cs = [np.corrcoef(a[:len(a) - L], b[L:])[0, 1] for L in range(maxlag + 1)]
  L = int(np.argmax(cs)); return L, cs[L]
rows = []
for f in sorted(glob.glob(f"{S}/0000*.npz")):
  d = np.load(f, allow_pickle=True); v = d['v'] / 3.6
  ang = -np.radians(d['ang']); kc = d['k_ctrl']; km = d['k_model']; wire = -np.radians(d['wire'])
  base = d['lat'] & ~(d['lb'] | d['rb']) & (d['lcs'] == 0)
  n = len(v); i = 0
  while i + 200 <= n:
    s = slice(i, i + 200)
    if base[s].mean() > 0.95 and (kc[s].max() - kc[s].min()) >= 1e-3 and v[s].mean() * 3.6 >= 20:
      g = d['gain'][s].mean(); tq = np.abs(d['tq_s'][s]).mean(); pr = d['pressed'][s].mean(); vv = v[s].mean() * 3.6
      Lw, cw = xlag(wire[s], ang[s]); Lp, cp = xlag(km[s], ang[s])
      if Lw is not None and cw > 0.7 and cp > 0.7: rows.append((vv, g, tq, pr, Lw * 10, Lp * 10))
    i += 100
R = np.array(rows); print(f"windows {len(R)}")
for name, m in (("hands-off strict: gain>=0.9, |tq|<50, pressed 0", (R[:,1] >= 0.9) & (R[:,2] < 50) & (R[:,3] == 0)),
                ("resting hand: gain 0.5-0.9, pressed<0.2", (R[:,1] >= 0.5) & (R[:,1] < 0.9) & (R[:,3] < 0.2)),
                ("driver holding: pressed>0.8", R[:,3] > 0.8)):
  for lo, hi in ((20, 40), (40, 60), (60, 200)):
    q = m & (R[:,0] >= lo) & (R[:,0] < hi)
    if q.sum(): print(f"  {name:48} {lo:>3}-{hi:<3}: n {q.sum():3d} | wire->wheel p50 {np.median(R[q,4]):4.0f} ms | plan->wheel p50 {np.median(R[q,5]):4.0f} ms | gain p50 {np.median(R[q,1]):.2f}")
