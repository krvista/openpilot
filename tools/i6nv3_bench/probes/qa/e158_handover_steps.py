"""E158 (report §96): what happens to the wheel when the factory LFA drops out while openpilot lateral is active - the
ADAS controller switches from its own request to relaying openpilot's (route npz v4).
Lists every mixed -> op transition (v >= 20 km/h): speed, why the factory LFA left (blinker within 1 s, driver torque
>= 150 within 1.5 s before, camera lane lines not 3 / 3, otherwise '?'), openpilot's request minus the wheel at that
moment (the wound-up gap), the largest step of the MDPS request within 1 s (deg in 100 ms), the wheel's movement after
0.3 / 0.6 / 1.0 s, the peak wheel rate and the peak lateral-acceleration change within 1.5 s.
Summary: share of transitions with |gap| >= 1 / 3 / 5 deg, and the same for hands-off, no-blinker transitions.
argv: S route [route ...]"""
import sys
from collections import Counter
import numpy as np
S = sys.argv[1]
G = []
for r in sys.argv[2:]:
  d = dict(np.load(f"{S}/r50/{r}.npz", allow_pickle=True)); T = d['T']; n = len(T)
  if 'cb_req' not in d:
    continue
  v = d['v']; lat = d['lat'].astype(bool); sym = d['lfa_sym'].astype(int)
  cmd = (d['cb_lvl2'].astype(int) == 2) & (d['mdps_active'].astype(int) == 2)
  cb = d['cb_req']; wire = d['wire']; wh = d['mdps_ang']; bl = d['lb'] | d['rb']
  mixed = cmd & lat & (sym == 2); op = cmd & lat & (sym != 2)
  tr = [i for i in range(150, n - 300) if mixed[i - 1] and op[i] and v[i] >= 20]
  if not tr:
    continue
  print(f"\n{r} ({str(d['commit'])[:7]}): {len(tr)} transitions (shown: |gap| >= 2 deg)")
  print("       t    v  cause          gap   | max MDPS-request step /0.1 s | wheel +0.3 / +0.6 / +1.0 s | peak wheel rate | lat-acc change m/s2")
  for i in tr:
    gap = wire[i] - wh[i]
    tq = np.abs(d['tq_s'][i - 150:i + 20]).max(); blk = bool(bl[i - 100:i + 150].any())
    lanes = bool(((d['lane_l'][i - 100:i + 20] != 3) | (d['lane_r'][i - 100:i + 20] != 3)).any())
    cause = "blinker" if blk else ("driver torque" if tq >= 150 or d['pressed'][i - 150:i + 20].any() else ("lanes" if lanes else "?"))
    step = np.abs(cb[i + 10:i + 110] - cb[i:i + 100]).max()
    rate = (np.abs(wh[i + 10:i + 150] - wh[i:i + 140]) / 0.1).max()
    vm = v[i] / 3.6; la = vm ** 2 * d['k_meas'][i:i + 150]; dla = np.ptp(la)
    G.append((abs(gap), cause, step, rate, dla, v[i]))
    if abs(gap) >= 2:
      print(f"  {T[i]:7.1f} {v[i]:4.0f}  {cause:13s} {gap:+5.1f}  | {step:5.1f}                        | {wh[i + 30] - wh[i]:+5.1f} / {wh[i + 60] - wh[i]:+5.1f} / "
            f"{wh[i + 100] - wh[i]:+5.1f}      | {rate:5.1f} deg/s     | {dla:4.2f}")
if G:
  g = np.array([x[0] for x in G]); print(f"\nall: {len(G)} transitions; causes {dict(Counter(x[1] for x in G))}")
  print(f"  |gap| >= 1 deg {np.mean(g >= 1):.0%}, >= 3 deg {np.mean(g >= 3):.0%}, >= 5 deg {np.mean(g >= 5):.0%}; median {np.median(g):.1f}, p90 {np.percentile(g, 90):.1f}")
  q = [x for x in G if x[1] in ("?", "lanes")]
  if q:
    print(f"  hands off, no blinker: {len(q)}; |gap| " + ", ".join(f"{x[0]:.1f}" for x in q) + "; MDPS-request step " + ", ".join(f"{x[2]:.1f}" for x in q))
