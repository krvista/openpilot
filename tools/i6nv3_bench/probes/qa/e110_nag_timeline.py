"""E110: every signal change in the cluster messages CCNC 0x161 / 0x162 (bus 1) from -25 s to +5 s around the first N
hands-on nags (ALERTS_2 1/2 onset) of a route, with the driver torque at each change — to find the stage-1 prompt."""
import glob, sys
from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value
from openpilot.tools.lib.logreader import LogReader
D, r, N = sys.argv[1], sys.argv[2], int(sys.argv[3])
dbc = DBC("hyundai_canfd_generated"); M = {a: next(m for m in dbc.msgs.values() if m.address == a) for a in (353, 354)}
M[dbc.name_to_msg["LFA"].address] = dbc.name_to_msg["LFA"]
fs = sorted(glob.glob(f"{D}/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
t0 = None; tq = 0.0; v = 0.0; state = {}; log = []; nags = []
for f in fs:
  for m in LogReader(f):
    try:
      w = m.which(); t = m.logMonoTime * 1e-9; t0 = t if t0 is None else t0; tt = t - t0
      if w == "carState": tq = m.carState.steeringTorque; v = m.carState.vEgo * 3.6
      elif w == "can":
        for c in m.can:
          if c.src == 1 and c.address in M:
            d = bytes(c.dat)
            for sn, sg in M[c.address].sigs.items():
              x = get_raw_value(d, sg)
              if state.get((c.address, sn)) != x:
                if (c.address, sn) in state:
                  log.append((tt, c.address, sn, state[(c.address, sn)], x, tq, v))
                  if sn == "ALERTS_2" and x in (1, 2) and state[(c.address, sn)] not in (1, 2) and v > 20:
                    nags.append(tt)
                state[(c.address, sn)] = x
    except Exception:  # noqa: BLE001
      pass
  if len(nags) >= N and log and log[-1][0] > nags[N - 1] + 5:
    break
SKIP = ("COUNTER", "CHECKSUM")
from collections import Counter
for tn in nags[:N]:
  print(f"== {r} nag at {tn:.1f} s")
  win = [x for x in log if tn - 25 <= x[0] <= tn + 5 and not any(s in x[2] for s in SKIP)]
  nch = Counter((x[1], x[2]) for x in win)
  for (tt, a, sn, old, new, q, vv) in win:
    if nch[(a, sn)] <= 4:
      print(f"   {tt - tn:+6.1f} s | {hex(a)} {sn:28s} {old} -> {new} | tq {q:+5.0f} v {vv:4.0f}")
  print(f"   (flapping signals skipped: {[k[1] for k, n in nch.items() if n > 4]})")
