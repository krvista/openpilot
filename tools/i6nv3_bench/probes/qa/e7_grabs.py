"""E7: corrective grabs. Onset = EPS steeringPressed rising edge (or |driver tq| >= 225) after >= 2 s with |tq| < 130 (resting hand allowed), lat
active, >= 30 km/h, no blinker/LC within 2 s. Direction of the driver torque vs op command (more turn / less turn / on a
straight: toward which side), the road regime at onset (lane-centre curvature now vs 0.5 s later), lane prob, offset."""
import sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from common import routes

S = sys.argv[1]
ev = []
tot_min = 0.0
for r, P, d, T, f in routes(S):
  Tr = d['T']; tq = d['tq_s']; v = d['v']
  strong = d['pressed'] | (np.abs(tq) >= 225)
  quiet = np.abs(tq) < 130
  qc = np.concatenate([[0], np.cumsum(quiet)])
  lc = d['lb'] | d['rb'] | (d['lcs'] > 0)
  lcc = np.concatenate([[0], np.cumsum(lc)])
  tot_min += np.sum(d['lat'] & (v >= 30)) * 0.01 / 60
  on = np.flatnonzero(strong[1:] & ~strong[:-1]) + 1
  last = -1e9
  for i in on:
    if i < 300 or Tr[i] - last < 3.0:
      continue
    if not (d['lat'][i] and v[i] >= 30):
      continue
    if qc[i] - qc[i - 200] < 195 or lcc[min(i + 200, len(lc))] - lcc[max(i - 200, 0)] > 0:
      continue
    last = Tr[i]
    j = np.searchsorted(T, Tr[i])
    if j >= len(T) - 12 or abs(T[j] - Tr[i]) > 0.1:
      continue
    k_now, k_ahead = f['k_road'][j], f['k_road'][min(j + 10, len(T) - 1)]
    kc = d['k_ctrl'][i]
    sgn_t = np.sign(tq[i])                                   # driver torque sign (+ = left in the angle convention)
    sgn_k = np.sign(d['apply'][i]) if abs(d['apply'][i]) > 1.0 else 0
    reg = ("straight" if max(abs(k_now), abs(k_ahead)) < 0.7e-3 else
           "entry" if abs(k_ahead) > abs(k_now) + 0.5e-3 else "exit" if abs(k_now) > abs(k_ahead) + 0.5e-3 else "curve")
    lane = min(P['PR'][j, 1], P['PR'][j, 2])
    ev.append((r, Tr[i], v[i], reg, sgn_t, sgn_k, lane, -f['c0'][j], d['apply'][i] - d['ang'][i], tq[i], d['ang'][i]))
  print(r, flush=True)
print(f"op-active >= 30 km/h: {tot_min:.0f} min, corrective-grab onsets after 3 s quiet: {len(ev)} ({len(ev) / tot_min * 60:.1f}/h)")
from collections import Counter
print("regime:", Counter(e[3] for e in ev))
for rg in ("entry", "curve", "exit"):
  es = [e for e in ev if e[3] == rg and e[5] != 0]
  more = sum(1 for e in es if e[4] == e[5]); less = sum(1 for e in es if e[4] == -e[5])
  print(f"  {rg:6}: driver adds turn {more}, removes turn {less}")
es = [e for e in ev if e[3] == "straight"]
print(f"  straight: n {len(es)}, lane_min<0.5 {sum(e[6] < 0.5 for e in es)}, |offset|>0.3 m {sum(abs(e[7]) > 0.3 for e in es)}, "
      f"torque toward lane centre {sum(1 for e in es if np.sign(-e[7]) == -e[4])} / away {sum(1 for e in es if np.sign(-e[7]) == e[4])}")
print("low lane prob (<0.5) share all:", f"{np.mean([e[6] < 0.5 for e in ev]):.0%}")
for e in ev[:60]:
  print(f"  {e[0]} t {e[1]:7.1f} v {e[2]:5.1f} {e[3]:8} tq {e[9]:+6.0f} apply-ang {e[8]:+5.1f} ang {e[10]:+6.1f} lane {e[6]:.2f} off {e[7]:+.2f}")
