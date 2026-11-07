"""E109 (v2): find the capacitive hands-on sensor on the CAN buses. State A = 13 .. 3 s before each cluster
"keep hands on" nag (the car itself judged hands off then; the last 3 s, where the driver reacts, are excluded); state B = stretches with |driver torque| > 250 for >= 2 s — hands on.
For every (address, bus, byte) collect the value sets in A and B; report bytes whose sets are disjoint and small
(status-like, not counters / checksums). Also: seconds since the driver torque last exceeded 50 before each nag."""
import glob, sys
from collections import defaultdict, deque
import numpy as np
from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value
from openpilot.tools.lib.logreader import LogReader
D, r = sys.argv[1], sys.argv[2]
dbc = DBC("hyundai_canfd_generated"); m161 = next(m for m in dbc.msgs.values() if m.address == 353); a2 = m161.sigs["ALERTS_2"]
fs = sorted(glob.glob(f"{D}/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
ring = deque(); t0 = None; tq = 0.0; v = 0.0; prev2 = 0; last_tq = None; hi_since = None; lo_since = None; lat = False; nags = []
A = defaultdict(set); B = defaultdict(set); nB = 0; nA = 0
def snap(dst, t_from, t_to):
  for (t, a, s, d) in ring:
    if t_from <= t <= t_to:
      for i, b in enumerate(d):
        dst[(a, s, i)].add(b)
for f in fs:
  for m in LogReader(f):
    try:
      w = m.which(); t = m.logMonoTime * 1e-9; t0 = t if t0 is None else t0; tt = t - t0
      if w == "carState":
        tq = m.carState.steeringTorque; v = m.carState.vEgo * 3.6
        if abs(tq) > 50: last_tq = tt

        if abs(tq) > 250:
          hi_since = tt if hi_since is None else hi_since
          if tt - hi_since >= 2.0 and nB < 12 and v > 20:
            snap(B, tt - 2.0, tt); nB += 1; hi_since = None
        else:
          hi_since = None
      elif w == "carControl": lat = m.carControl.latActive
      elif w == "can":
        for c in m.can:
          d = bytes(c.dat)
          if len(d) <= 64:
            ring.append((tt, c.address, c.src, d))
          if c.address == 353 and c.src == 1:
            x2 = int(get_raw_value(d, a2))
            if x2 in (1, 2) and prev2 not in (1, 2) and v > 20:
              nags.append((round(tt, 1), round(v), round(tt - last_tq, 1) if last_tq is not None else None))
              snap(A, tt - 13.0, tt - 3.0); nA += 1
            prev2 = x2
        while ring and tt - ring[0][0] > 14.0:
          ring.popleft()
    except Exception:  # noqa: BLE001
      pass
print(f"== {r}: nags {len(nags)} (t, v, s since |tq|>50): {nags}")
print(f"   windows: hands-off {nA}, hands-on {nB}")
cands = []
for k in set(A) & set(B):
  if A[k] and B[k] and (max(A[k]) < min(B[k]) or max(B[k]) < min(A[k])):   # ranges disjoint (status or level signal)
    cands.append((hex(k[0]), k[1], k[2], sorted(A[k]), sorted(B[k])))
cands.sort()
print(f"   status-like bytes that differ (addr, bus, byte, values hands-off, values hands-on): {len(cands)}")
import pickle
pickle.dump(cands, open(f"{sys.argv[3]}/hodcand_{r}.pkl", "wb"))
for c in cands[:40]:
  nm = next((mm.name for mm in dbc.msgs.values() if mm.address == int(c[0], 16)), "?")
  print(f"     {c[0]} ({nm}) bus {c[1]} byte {c[2]}: off {c[3]} on {c[4]}")
