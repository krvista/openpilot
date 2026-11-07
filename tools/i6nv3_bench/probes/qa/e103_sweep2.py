"""E103: second whole-drive sweep (areas not covered by e96): camera calibration (status, rpy, height), learned vehicle
parameters (angle offset, steer ratio, stiffness), the car's own hands-on warnings (LFA LKA_SysWrn 4/5/6) and the
driver torque around them, lateral drop-outs (latActive falling while moving: cause events), alert onsets with speed
(laneTurn*, e2eChime, silentLkasDisable, laneDropout), model lane-line confidence and lane width on straights, and the
map-data speed-limit availability."""
import glob, sys, json
from collections import Counter, defaultdict
import numpy as np
from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value
from openpilot.tools.lib.logreader import LogReader
D, r = sys.argv[1], sys.argv[2]
dbc = DBC("hyundai_canfd_generated"); lfa = dbc.name_to_msg["LFA"]; sig = lfa.sigs["LKA_SysWrn"]
fs = sorted(glob.glob(f"{D}/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
t0 = None; v = 0.0; tq = 0.0; lat = False; pressed = False
cal = []; vp = []; hod = []; hod_prev = 0; drops = []; ev = set(); alerts = []; prev_alert = None
ll = []; lw = []; sl = Counter(); lane_prob = None
for f in fs:
  for m in LogReader(f):
    try:
      w = m.which(); t = m.logMonoTime * 1e-9; t0 = t if t0 is None else t0; tt = t - t0
      if w == "carState":
        c = m.carState; v = c.vEgo * 3.6; tq = c.steeringTorque; pressed = c.steeringPressed
      elif w == "carControl":
        la = m.carControl.latActive
        if lat and not la and v > 10:
          drops.append((round(tt, 1), round(v), sorted(ev)))
        lat = la
      elif w == "onroadEvents":
        ev = {str(e.name) for e in m.onroadEvents}
      elif w == "selfdriveState":
        a = m.selfdriveState.alertType
        if a and a != prev_alert and a.split("/")[0] in ("laneTurnLeft", "laneTurnRight", "e2eChime", "silentLkasDisable", "laneDropout", "silentLkasEnable", "lkasEnable"):
          alerts.append((a.split("/")[0], round(tt, 1), round(v)))
        prev_alert = a
      elif w == "can":
        for c in m.can:
          if c.src == 1 and c.address == lfa.address:
            x = get_raw_value(bytes(c.dat), sig)
            if x in (4, 5, 6) and hod_prev not in (4, 5, 6):
              hod.append((round(tt, 1), round(v), int(x), round(tq), lat))
            hod_prev = x
      elif w == "extrinsicsCalibration":
        e = m.extrinsicsCalibration
        cal.append((tt, str(e.calStatus), e.calPerc, [round(np.degrees(x), 2) for x in e.rpyCalib], round(e.height[0], 3) if len(e.height) else None))
      elif w == "vehicleParameters":
        p = m.vehicleParameters
        vp.append((tt, p.angleOffsetDeg, p.angleOffsetAverageDeg, p.steerRatio, p.stiffnessFactor, p.valid, p.angleOffsetAverageStd, p.steerRatioStd, p.roll))
      elif w == "modelV2" and v > 40:
        md = m.modelV2
        pr = list(md.laneLineProbs)
        if len(pr) >= 4:
          ll.append((pr[1], pr[2]))
          if pr[1] > 0.6 and pr[2] > 0.6 and len(md.laneLines) >= 3:
            lw.append(md.laneLines[2].y[0] - md.laneLines[1].y[0])
      elif w == "liveMapDataSP":
        sl["valid" if m.liveMapDataSP.speedLimitValid else "invalid"] += 1
    except Exception:  # noqa: BLE001
      pass
print(f"== {r}")
if cal:
  print(f"  calib: first {cal[0]} | last {cal[-1]} | statuses {Counter(c[1] for c in cal)}")
if vp:
  a = np.array([x[1:5] for x in vp], float); st = np.array([x[6:8] for x in vp], float)
  print(f"  vehicleParams: angleOffset inst p5/p50/p95 {np.percentile(a[:,0],5):+.2f}/{np.median(a[:,0]):+.2f}/{np.percentile(a[:,0],95):+.2f} deg, average {a[0,1]:+.2f} -> {a[-1,1]:+.2f} (std {st[-1,0]:.3f}) | "
        f"steerRatio {a[0,2]:.2f} -> {a[-1,2]:.2f} (p5/p95 {np.percentile(a[:,2],5):.2f}/{np.percentile(a[:,2],95):.2f}, std {st[-1,1]:.3f}) | stiffness {a[-1,3]:.2f} | valid {np.mean([x[5] for x in vp]):.0%} | roll p50 {np.degrees(np.median([x[8] for x in vp])):+.2f} deg")
print(f"  car HOD warnings: {len(hod)} onsets | by level {Counter(h[2] for h in hod)} | while op lat {sum(1 for h in hod if h[4])} | raw torque at onset p50 {np.median([abs(h[3]) for h in hod]) if hod else 0:.0f} | first 8: {hod[:8]}")
print(f"  lat drop-outs while moving: {len(drops)} | causes {Counter(e for d in drops for e in d[2])} | first 6: {drops[:6]}")
print(f"  alerts: {Counter(a[0] for a in alerts)} | laneTurn/e2e detail: {[a for a in alerts if a[0] in ('laneTurnLeft','laneTurnRight','e2eChime')][:12]}")
if ll:
  L = np.array(ll); print(f"  lane lines (> 40 km/h): left prob p10/p50 {np.percentile(L[:,0],10):.2f}/{np.median(L[:,0]):.2f}, right {np.percentile(L[:,1],10):.2f}/{np.median(L[:,1]):.2f}, both > 0.6 {np.mean((L[:,0]>0.6)&(L[:,1]>0.6)):.0%} | lane width p10/p50/p90 {np.percentile(lw,10):.2f}/{np.median(lw):.2f}/{np.percentile(lw,90):.2f} m")
print(f"  map speed limit: {dict(sl)}")
