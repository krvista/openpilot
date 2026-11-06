"""E80: the last seconds of a route (ignition off): per 0.5 s, CAN messages per bus/src (0 car side, 2 camera side, 128+
TX echo, 192+ rejected), panda safety model / ignition / controlsAllowed, LKAS_ALT (0x110) frames sent and their active bit,
camera fault signals; and the last 0x110 sent vs the last camera 0x110 seen."""
import glob
import sys
from collections import Counter, defaultdict
from openpilot.tools.lib.logreader import LogReader
from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value
dbc = DBC("hyundai_canfd_generated"); L = dbc.name_to_msg["LKAS_ALT"]; C2 = dbc.name_to_msg["CCNC_0x162"]
r, seg = sys.argv[1].split(":"); TAIL = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
f = glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--{seg}--rlog.zst")[0]
ev = []
for m in LogReader(f):
  ev.append(m.as_builder() if False else m)
tend = ev[-1].logMonoTime * 1e-9
bins = defaultdict(Counter); ps = {}; sent = defaultdict(list); flt = {}
for m in ev:
  t = m.logMonoTime * 1e-9
  if t < tend - TAIL:
    continue
  b = round((t - tend) * 2) / 2
  w = m.which()
  if w == "can":
    for c in m.can:
      bins[b][c.src] += 1
      if c.src == 1 and c.address == C2.address:
        flt[b] = get_raw_value(bytes(c.dat), C2.sigs["FAULT_FCA"])
  elif w == "sendcan":
    for c in m.sendcan:
      if c.address == L.address:
        sent[b].append(get_raw_value(bytes(c.dat), L.sigs["LKAS_ANGLE_ACTIVE"]))
  elif w == "pandaStates" and len(m.pandaStates):
    p = m.pandaStates[0]; ps[b] = (str(p.safetyModel), p.ignitionLine, p.controlsAllowed, p.safetyTxBlocked)
print(f"== {r} seg {seg}: last {TAIL} s before the log ends")
for b in sorted(bins):
  print(f"  {b:5.1f} s | " + " ".join(f"{k}:{v}" for k, v in sorted(bins[b].items())) + f" | sent 0x110 {len(sent.get(b, []))} active {sorted(set(sent.get(b, [])))} | panda {ps.get(b)} | FAULT_FCA {flt.get(b)}")
