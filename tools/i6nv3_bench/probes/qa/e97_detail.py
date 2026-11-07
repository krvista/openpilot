"""E97: details behind e96 flags for a route: FCW frames (time, speed, lead), controlsState gaps > 30 ms (time since
log start), panda spiErrorCount steps, driver-monitoring alert onsets, calibration / live parameters, disk free."""
import glob, sys
import numpy as np
from openpilot.tools.lib.logreader import LogReader
D, r = sys.argv[1], sys.argv[2]
fs = sorted(glob.glob(f"{D}/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
t0 = None; v = 0.0; last_cs = None; spi = None; lead = None; cal = []; lp = []; dm_prev = set(); out = []; free = []; ev_prev = set()
for f in fs:
  for m in LogReader(f):
    try:
      w = m.which(); t = m.logMonoTime * 1e-9
      t0 = t if t0 is None else t0; tt = t - t0
      if w == "carState": v = m.carState.vEgo * 3.6
      elif w == "radarState":
        l = m.radarState.leadOne; lead = (l.status, round(l.dRel, 1), round(l.vRel * 3.6, 1), round(l.aLeadK, 1))
      elif w == "longitudinalPlan" and m.longitudinalPlan.fcw:
        out.append(f"  FCW t {tt:7.1f} v {v:5.1f} lead(status,d,vrel kph,a) {lead}")
      elif w == "controlsState":
        if last_cs is not None and t - last_cs > 0.03 and t - last_cs < 5:
          out.append(f"  controls gap {(t - last_cs) * 1e3:4.0f} ms at t {tt:7.1f} v {v:5.1f}")
        last_cs = t
      elif w == "pandaStates":
        for p in m.pandaStates:
          if spi is not None and p.spiErrorCount != spi:
            out.append(f"  spi errors {spi} -> {p.spiErrorCount} at t {tt:7.1f}")
          spi = p.spiErrorCount
      elif w == "onroadEvents":
        cur = {str(e.name) for e in m.onroadEvents if str(e.name).startswith(("driver", "fcw", "stockFcw", "stockAeb"))}
        for e in cur - ev_prev:
          out.append(f"  event {e} at t {tt:7.1f} v {v:5.1f}")
        ev_prev = cur
      elif w == "liveCalibration":
        lc = m.liveCalibration; cal.append((tt, str(lc.calStatus), lc.calPerc, [round(np.degrees(x), 2) for x in lc.rpyCalib]))
      elif w == "liveParameters":
        x = m.liveParameters; lp.append((tt, x.steerRatio, x.stiffnessFactor, x.angleOffsetAverageDeg, x.angleOffsetDeg, x.valid, x.roll))
      elif w == "deviceState":
        free.append(m.deviceState.freeSpacePercent)
    except Exception as e:  # noqa: BLE001
      pass
print(f"== {r}")
seen = {}
for o in out:
  k = o.split(" at")[0][:22]
  seen[k] = seen.get(k, 0) + 1
  if seen[k] <= 6:
    print(o)
for k, n in seen.items():
  if n > 6: print(f"  ... {k}: {n} total")
if cal: print(f"  calib first {cal[0]} last {cal[-1]}")
if lp:
  a = np.array([x[1:5] for x in lp]); print(f"  liveParams SR {a[0,0]:.2f}->{a[-1,0]:.2f} stiff {a[-1,1]:.2f} angleOffsetAvg {a[0,2]:.2f}->{a[-1,2]:.2f} inst p5/p95 {np.percentile(a[:,3],5):.2f}/{np.percentile(a[:,3],95):.2f} valid {np.mean([x[5] for x in lp]):.0%}")
if free: print(f"  disk free {free[0]:.1f}% -> {free[-1]:.1f}%")
