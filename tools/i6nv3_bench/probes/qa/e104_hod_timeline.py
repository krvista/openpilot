"""E104: timeline around the car's hands-on warning onsets (LFA LKA_SysWrn -> 4): -4 .. +2 s at 0.25 s, driver raw
torque, EPS pressed flag, op lat active, wheel angle, TX request, speed, and the LKA_SysWrn / LFA_SysSta values."""
import glob, sys
from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value
from openpilot.tools.lib.logreader import LogReader
D, r, N = sys.argv[1], sys.argv[2], int(sys.argv[3])
dbc = DBC("hyundai_canfd_generated"); lfa = dbc.name_to_msg["LFA"]
sigs = {k: lfa.sigs[k] for k in ("LKA_SysWrn", "LFA_SysSta") if k in lfa.sigs}
fs = sorted(glob.glob(f"{D}/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
rows = []; t0 = None; st = dict(v=0, tq=0, pr=0, lat=0, ang=0, tx=0)
for f in fs:
  for m in LogReader(f):
    try:
      w = m.which(); t = m.logMonoTime * 1e-9; t0 = t if t0 is None else t0; tt = t - t0
      if w == "carState":
        c = m.carState; st.update(v=c.vEgo * 3.6, tq=c.steeringTorque, pr=int(c.steeringPressed), ang=c.steeringAngleDeg)
      elif w == "carControl": st.update(lat=int(m.carControl.latActive), tx=m.carControl.actuators.steeringAngleDeg)
      elif w == "can":
        for c in m.can:
          if c.src == 1 and c.address == lfa.address:
            vals = {k: get_raw_value(bytes(c.dat), s) for k, s in sigs.items()}
            rows.append((tt, dict(st), vals))
    except Exception:  # noqa: BLE001
      pass
on = [i for i in range(1, len(rows)) if rows[i][2]["LKA_SysWrn"] in (4, 5, 6) and rows[i - 1][2]["LKA_SysWrn"] not in (4, 5, 6)]
print(f"== {r}: {len(on)} onsets; showing the first {N} (t rel. to onset | v | drv tq | pressed | lat | wheel | TX | SysWrn LFA_SysSta)")
for k in on[:N]:
  tk = rows[k][0]; last = -9
  print(f"  onset at {tk:7.1f} s")
  for i in range(max(k - 400, 0), min(k + 200, len(rows))):
    tt = rows[i][0] - tk
    if tt - last >= 0.5:
      last = tt; s = rows[i][1]; vv = rows[i][2]
      print(f"    {tt:+5.1f} | {s['v']:5.1f} | {s['tq']:+5.0f} | {s['pr']} | {s['lat']} | {s['ang']:+6.1f} | {s['tx']:+6.1f} | {vv.get('LKA_SysWrn')} {vv.get('LFA_SysSta')}")
