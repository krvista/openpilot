"""E79: timeline of the front camera fault signals (CCNC_0x162 FAULT_*, LFA ELK_SysFlrSta / LKA_SysWrn, FR_CMR DAW_SysSta)
plus panda safety model, op lat state and ignition, for given route segments (0.5 s steps); first / last time each fault
was set."""
import glob
import sys
from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value
from openpilot.tools.lib.logreader import LogReader
dbc = DBC("hyundai_canfd_generated")
WATCH = [("CCNC_0x162", "FAULT_FCA"), ("CCNC_0x162", "FAULT_LSS"), ("CCNC_0x162", "FAULT_DAW"), ("LFA", "ELK_SysFlrSta"),
         ("LFA", "LKA_SysWrn"), ("FR_CMR_01_10ms", "DAW_SysSta")]
for spec in sys.argv[1:]:
  r, seg = spec.split(":")
  f = glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--{seg}--rlog.zst")[0]
  t0 = None; state = {}; model = "?"; lat = None; ign = None; last_print = -1; rows = []; first = {}; lastseen = {}
  msgs = {dbc.name_to_msg[m].address: dbc.name_to_msg[m] for m, _ in WATCH}
  for m in LogReader(f):
    t = m.logMonoTime * 1e-9; w = m.which()
    if t0 is None:
      t0 = t
    dt = t - t0
    if w == "can":
      for c in m.can:
        if c.src == 1 and c.address in msgs:
          msg = msgs[c.address]
          for mn, sn in WATCH:
            if mn == msg.name:
              v = get_raw_value(bytes(c.dat), msg.sigs[sn]); state[sn] = v
              bad = v not in (0, 5) if sn == "DAW_SysSta" else v != 0
              if bad:
                first.setdefault(sn, dt); lastseen[sn] = dt
    elif w == "pandaStates" and len(m.pandaStates):
      model = str(m.pandaStates[0].safetyModel); ign = m.pandaStates[0].ignitionLine
    elif w == "carControl":
      lat = m.carControl.latActive
    if int(dt * 2) != last_print and state:
      last_print = int(dt * 2)
      rows.append(f"    {dt:6.1f} {model:12} ign {ign} lat {lat} | " + " ".join(f"{k}={v}" for k, v in state.items()))
  print(f"== {r} seg {seg}: duration {dt:.1f} s | fault first/last seen: " + ", ".join(f"{k} {first[k]:.1f}-{lastseen[k]:.1f}" for k in first) if first else f"== {r} seg {seg}: duration {dt:.1f} s | no camera fault")
  sel = rows[:6] + (["    ..."] if len(rows) > 12 else []) + rows[-6:]
  for x in sel:
    print(x)
