"""E19: which steering guards actually act, per build, from controlsState.steerFlags (bits 0-10) and carStateSP.steerFlags
(bits 0-28) on all Phase 46+ routes (100 Hz npz dirs r3d, r43, r46 + any extra). Per flag: share of op-active frames
(>= 30 km/h, hands-off / all), onsets per hour. Dead or near-dead guards are complexity-reduction candidates."""
import glob
import sys
import numpy as np

S = sys.argv[1]
CN = {0: "41 lowconf gate", 1: "41 lowconf CAPPED", 2: "41 release ramp", 3: "41-2 lookahead cap", 4: "6g conf blend",
      5: "39 dropout hold", 6: "departure hold", 7: "37b BSM hold", 8: "7c entry assist", 9: "55 orient lead", 10: "55 budget clip"}
CC = {2: "passthrough", 3: "lowspeed latch", 4: "scenario block", 5: "angle passive", 6: "parking", 7: "parking releasing",
      8: "driver pressed", 9: "eps pressed", 10: "heavy grip anchor", 11: "42c anchor hold", 12: "42a/44a fast floor",
      13: "35a tq-arm floor", 14: "42b shove", 15: "43b accel yield", 16: "boost suppressed", 17: "33 blind caution",
      18: "39a city release", 19: "35b anchored recovery", 20: "blinker concession", 21: "29 post-grip", 22: "VM reject",
      23: "post-reverse", 24: "cam fault", 25: "38-3 wheel outrun", 26: "tx rejected", 27: "38 tx saturated", 28: "51 HOD touch"}
agg = {}
for f in sorted(glob.glob(f"{S}/r3d/0*.npz") + glob.glob(f"{S}/r43/0*.npz") + glob.glob(f"{S}/r46/0*.npz") + glob.glob(f"{S}/rx/0*.npz")):
  d = np.load(f, allow_pickle=True); b = str(d['commit'])
  v = d['v']; lat = d['lat']; sf = d['sf_ctl'].astype(np.int64); sc = d['sf_cc'].astype(np.int64)
  if not (sf.any() or sc.any()):
    continue
  hands = np.convolve((d['pressed'] | (np.abs(d['tq_s']) > 150)).astype(float), np.ones(201), 'same') > 0
  act = lat & (v >= 30); off = act & ~hands
  a = agg.setdefault(b, dict(act=0, off=0, ctl={}, cc={}, routes=[]))
  a['act'] += act.sum(); a['off'] += off.sum(); a['routes'].append(f[-12:-4])
  for bits, src, store in ((CN, sf, a['ctl']), (CC, sc, a['cc'])):
    for bit in bits:
      x = (src >> bit) & 1
      st = store.setdefault(bit, [0, 0, 0])
      st[0] += int((x & act).sum()); st[1] += int((x & off).sum()); st[2] += int(np.sum((x[1:] == 1) & (x[:-1] == 0) & act[1:]))
for b, a in agg.items():
  h = a['act'] * 0.01 / 3600
  print(f"== build {b} ({', '.join(a['routes'])}): op-active >= 30 km/h {h * 60:.0f} min, hands-off {a['off'] * 0.01 / 60:.0f} min")
  print("   controlsState.steerFlags            all%   hands-off%   onsets/h")
  for bit, nm in CN.items():
    st = a['ctl'][bit]
    print(f"   {bit:2d} {nm:22} {st[0] / max(a['act'], 1):7.2%} {st[1] / max(a['off'], 1):9.2%} {st[2] / max(h, 1e-9):9.1f}")
  print("   carStateSP.steerFlags")
  for bit, nm in CC.items():
    st = a['cc'][bit]
    if st[0] == 0:
      print(f"   {bit:2d} {nm:22}    never")
    else:
      print(f"   {bit:2d} {nm:22} {st[0] / max(a['act'], 1):7.2%} {st[1] / max(a['off'], 1):9.2%} {st[2] / max(h, 1e-9):9.1f}")
