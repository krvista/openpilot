"""Audible alerts in a route: selfdriveState.alertSound transitions (non-none), with text, time, speed."""
import glob, sys
from openpilot.tools.lib.logreader import LogReader
r = sys.argv[1]
files = sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
t0 = None; prev = None; v = 0.0
for f in files:
  seg = int(f.split("--")[-2])
  for m in LogReader(f):
    t = m.logMonoTime * 1e-9
    if t0 is None: t0 = t
    if m.which() == "carState": v = m.carState.vEgo * 3.6
    elif m.which() == "selfdriveState":
      s = m.selfdriveState; snd = str(s.alertSound)
      if snd != prev:
        if snd != "none": print(f"  seg {seg:2d} t {t - t0:7.1f} s  v {v:5.1f} km/h  sound {snd:18} status {s.alertStatus} size {s.alertSize}  '{s.alertText1}' / '{s.alertText2}'")
        prev = snd
