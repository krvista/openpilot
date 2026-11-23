"""E166 (report §103): exact accounting of who commanded the MDPS, per 10 ms sample (route npz v5).
Classes while moving (v >= 1 km/h):
  R  openpilot relayed   - openpilot requests, the controller commands, its request equals one of the last 8 angles
                           openpilot transmitted (0.5 deg), factory LFA icon not 'active'
  F  factory LFA         - icon 'active' and the controller commands
  C  controller's own    - openpilot requests, the controller commands something else, icon not 'active'
                           (the factory LFA's start / release edges, relay timing)
  P  commanded, op passive
  X  op requests, not commanded
  N1 nobody, lateral on  - split by the car controller's steerFlags
  N0 nobody, lateral off
Then: every episode of F / C longer than 50 ms, the share of each class by speed band, how long the transmitted angle
was rate-limited behind the command (TX governor), and gain / delivery at low speed.
argv: S route [route ...]"""
import re
import sys
from collections import Counter
import numpy as np
S = sys.argv[1]
src = open("opendbc_repo/opendbc/car/hyundai/carcontroller.py").read()
SF = {m.group(1): int(m.group(2)) for m in re.finditer(r"^(SF_[A-Z_]+)\s*=\s*1 << (\d+)", src, re.M)}
WHY = [("grip", "SF_LOWSPEED_LATCH"), ("scenario", "SF_SCENARIO_BLOCK"), ("parking", "SF_PARKING"), ("driver-turn", "SF_ANGLE_PASSIVE"),
       ("reverse", "SF_REVERSE"), ("vm-reject", "SF_VM_REJECT"), ("camera", "SF_CAM_FAULT"), ("wheel-outrun", "SF_WHEEL_OUTRUN")]
BANDS = [(1, 5), (5, 15), (15, 25), (25, 35), (35, 45), (45, 60), (60, 90), (90, 200)]
ORDER = ["R", "F", "C", "P", "X", "N1", "N0"]

def runs(m):
  e = np.flatnonzero(np.diff(np.r_[0, m.astype(np.int8), 0]))
  return list(zip(e[::2], e[1::2]))

for r in sys.argv[2:]:
  d = dict(np.load(f"{S}/r50/{r}.npz", allow_pickle=True)); T = d['T']; n = len(T); v = d['v']
  tx = d['tx_active'].astype(int); l2 = d['cb_lvl2'].astype(int); sym = d['lfa_sym'].astype(int)
  cb = d['cb_req']; wire = d['wire']; lat = d['lat'].astype(bool); sf = d['sf_cc'].astype(np.int64)
  ok = np.zeros(n, bool)
  for k in range(0, 9):
    ok |= np.abs(cb - np.r_[np.full(k, wire[0]), wire[:n - k]]) <= 0.5
  cls = np.full(n, "?", dtype=object)
  req = tx == 2; cmd = l2 == 2
  cls[req & cmd & ok] = "R"; cls[req & cmd & ~ok] = "C"; cls[~req & cmd] = "P"; cls[req & ~cmd] = "X"
  cls[~req & ~cmd & lat] = "N1"; cls[~req & ~cmd & ~lat] = "N0"; cls[(sym == 2) & cmd] = "F"
  mv = v >= 1.0
  print(f"== {r} (commit {str(d['commit'])[:8]}): moving {mv.sum()/6000:.1f} min, lateral on {lat[mv].mean()*100:.1f} % of it")
  for c in ORDER:
    m = (cls == c) & mv
    print(f"  {c:3s} {m.sum()/100:8.2f} s  {m.sum()/mv.sum()*100:7.3f} % of moving  {(m & lat).sum()/max((mv & lat).sum(), 1)*100:7.3f} % of lateral-on")
  print("  share (%) of each speed band's moving time\n  km/h      min |" + "".join(f"{c:>8s}" for c in ORDER))
  for lo, hi in BANDS:
    b = mv & (v >= lo) & (v < hi)
    if b.sum():
      print(f"  {lo:3d}-{hi:3d} {b.sum()/6000:6.1f} |" + "".join(f"{((cls == c) & b).mean()/b.mean()*100:8.2f}" for c in ORDER))
  print("  not openpilot's request although commanded, lateral on (F / C runs >= 50 ms, first 40):")
  for a, b in runs(((cls == "F") | (cls == "C")) & mv & lat)[:40]:     # lateral-on only (a capture drive is all 'F')
    if b - a >= 5:
      w = d['mdps_ang'][a:b]
      print(f"    t={T[a]:8.2f} s {(b-a)*10:5d} ms  {v[a]:5.1f} km/h  icon {sorted(set(sym[max(a-10,0):b+10].tolist()))}  "
            f"max |ctrl - ours| {np.max(np.abs(cb[a:b]-wire[a:b])):.2f} deg  wheel moved {w.max()-w.min():.2f} deg")
  n1 = (cls == "N1") & mv
  combo = Counter("+".join(nm for nm, k in WHY if (sf[i] >> SF[k]) & 1) or "none" for i in np.flatnonzero(n1))
  print("  N1 by reason: " + ", ".join(f"{k} {c/100:.1f} s" for k, c in combo.most_common(8)))
  # transmitted angle behind the command (rate limit) while relayed and hands off
  beh = np.abs(d['apply'] - wire); ho = ~d['pressed'].astype(bool)
  for lo, hi in ((30, 60), (60, 67), (67, 200)):
    m = (cls == "R") & ho & (v >= lo) & (v < hi)
    if m.sum():
      rate = np.abs(np.gradient(wire, 0.01))
      print(f"  relayed, hands off, {lo}-{hi} km/h: {m.sum()/100:7.1f} s | wire > 1 deg behind the command {(m & (beh > 1)).sum()/100:6.1f} s, "
            f"max {beh[m].max():.1f} deg | wire rate p99 {np.percentile(rate[m], 99):.1f} deg/s, max {rate[m].max():.1f}")
  for lo, hi in ((5, 20), (20, 30), (30, 45), (45, 200)):
    m = (cls == "R") & ho & (v >= lo) & (v < hi)
    if m.sum() > 200:
      e = np.abs(wire - d['mdps_ang'])[m]
      print(f"  relayed, hands off, {lo}-{hi} km/h: gain sent p50 {np.median(d['gain'][m]):.2f} | |request - wheel| p50 {np.median(e):.2f} p90 {np.percentile(e, 90):.2f} deg")
