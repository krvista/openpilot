"""E78: decode the hyundai_canfd DBC signals in the stock boot window (t 2-9 s) and compare a warning boot with normal
boots: report signals whose value set in the bad boot does not overlap any good boot's, ranked fault/warning-like names
first (FLT, FAULT, FAIL, WARN, ERR, SYS, STAT, AVAIL, INHIBIT, LAMP, DTC, MALF, IND)."""
import glob
import re
import sys
from collections import defaultdict
from openpilot.tools.lib.logreader import LogReader
from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value
BAD = sys.argv[1]; GOOD = sys.argv[2].split(","); T0 = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0; T1 = float(sys.argv[4]) if len(sys.argv) > 4 else 9.0
dbc = DBC("hyundai_canfd_generated")
FAULTY = re.compile(r"FLT|FAULT|FAIL|WARN|ERR|SYS|STAT|AVAIL|INHIBIT|LAMP|DTC|MALF|IND|ICON|BLOCK|DISP|MSG|ALERT|OFF|ON_", re.I)


def collect(r):
  f = glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--0--rlog.zst")[0]
  t0 = None; vals = defaultdict(set)
  for m in LogReader(f):
    t = m.logMonoTime * 1e-9
    if t0 is None:
      t0 = t
    if t - t0 > T1:
      break
    if m.which() == "can" and t - t0 >= T0:
      for c in m.can:
        if c.src >= 3:
          continue
        msg = dbc.addr_to_msg.get(c.address)
        if msg is None:
          continue
        dat = bytes(c.dat)
        for sname, sig in msg.sigs.items():
          if sname in ("CHECKSUM", "COUNTER") or "CRC" in sname or "COUNTER" in sname or "Cnt" in sname or "Crc" in sname:
            continue
          try:
            vals[(c.src, msg.name, sname)].add(get_raw_value(dat, sig))
          except Exception:  # noqa: BLE001
            pass
  return vals


bad = collect(BAD); goods = [collect(g) for g in GOOD]
rows = []
for key, bv in bad.items():
  gv = [g.get(key) for g in goods]
  if any(x is None for x in gv):
    continue
  if all(not (bv & x) for x in gv):          # bad values never seen in any good boot
    rows.append((0 if FAULTY.search(key[2]) else 1, key, sorted(bv)[:6], [sorted(x)[:6] for x in gv]))
rows.sort()
print(f"signals whose values in {BAD} never occur in {GOOD} (t {T0}-{T1} s): {len(rows)}")
for pri, key, bv, gv in rows[:70]:
  print(f"  {'*' if pri == 0 else ' '} bus {key[0]} {key[1]:28} {key[2]:34} bad {bv}  good {gv}")
