"""E33: who puts the car on the INSIDE of curves? Hands-off, op active, both lines > 0.7, >= 30 km/h, no blinker/LC,
road |k| >= 2e-3 (plan lanes). Per build and phase (entry / steady / exit) and per tightness: the car's inside offset, the
command's excess over the model toward the inside ((k_ctrl - K) * sign), and the model's own excess over the road toward
the inside ((K - k_road) * sign) — which layer is cutting the corner. Also split by the 7c entry-assist flag and the 55
lead-weight state, and the MDPS ratio wheel/command (an over-tracking MDPS also cuts)."""
import sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from common import routes

S = sys.argv[1]
rows = {}
for r, P, d, T, f in routes(S):
  n = len(T); b = str(d['commit']); Tr = d['T']; kr = f['k_road']
  j6 = np.minimum(np.searchsorted(T, T + 0.6), n - 1); k6 = kr[j6]
  ph = np.where(np.abs(k6) > np.abs(kr) + 0.7e-3, 1, np.where(np.abs(kr) > np.abs(k6) + 0.7e-3, 3, 2))
  sgn = np.sign(np.where(np.abs(kr) > np.abs(k6), kr, k6))
  kc = np.interp(T, Tr, d['k_ctrl']); K = P['K'].astype(float)
  ap = np.interp(T, Tr, d['apply']); an = np.interp(T, Tr, d['ang'])
  sf = np.interp(T, Tr, d['sf_ctl'].astype(float)).astype(int)
  m = f['good'] & f['lat'] & ~f['hands'] & (np.maximum(np.abs(kr), np.abs(k6)) >= 2e-3)
  ratio = np.abs(an) / np.maximum(np.abs(ap), 0.5)
  rows.setdefault(b, []).append(np.stack([np.abs(kr[m]), ph[m], (-f['c0'] * sgn)[m], ((kc - K) * sgn)[m], ((K - kr) * sgn)[m],
                                          (sf[m] >> 8) & 1, ratio[m], f['v'][m] * 3.6], 1))
  print(r, b, flush=True)
for b, rr in rows.items():
  A = np.concatenate(rr)
  print(f"== build {b}: hands-off curve frames {len(A)} ({len(A) / 1200:.1f} min)")
  print(f"   {'subset':18} {'n':>6} {'inside p50/p90 m':>17} {'cmd-model +in e-3':>18} {'model-road +in e-3':>19} {'wheel/cmd':>9}")
  for nm, m in (("all", np.ones(len(A), bool)), ("entry", A[:, 1] == 1), ("steady", A[:, 1] == 2), ("exit", A[:, 1] == 3),
                ("|k| 2-4e-3", A[:, 0] < 4e-3), ("|k| >= 4e-3", A[:, 0] >= 4e-3), ("7c on", A[:, 5] == 1),
                ("v < 50", A[:, 7] < 50), ("v >= 50", A[:, 7] >= 50)):
    if m.sum() < 60:
      continue
    x = A[m]
    print(f"   {nm:18} {m.sum():6d} {np.median(x[:, 2]):+6.2f}/{np.percentile(x[:, 2], 90):+5.2f}      {np.mean(x[:, 3]) * 1e3:+7.3f}            {np.mean(x[:, 4]) * 1e3:+7.3f}        {np.median(x[:, 6]):6.2f}")
