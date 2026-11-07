"""E99: driver monitoring per route — share of time (> 20 km/h) with a face detected, distracted, on the wheel-touch policy,
awareness minimum and time below 0.5, alert onsets with speed, and DM model confidence / face position stats; plus the
car's own hands-on (HOD) state if available."""
import glob, sys
import numpy as np
from openpilot.tools.lib.logreader import LogReader
D = sys.argv[1]
for r in sys.argv[2:]:
  fs = sorted(glob.glob(f"{D}/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
  face = []; dist = []; low = []; aw = []; v = 0.0; en = False; prob = []; pos = []; sg = []; ev = {}
  for f in fs:
    for m in LogReader(f):
      try:
        w = m.which()
        if w == "carState": v = m.carState.vEgo * 3.6
        elif w == "selfdriveState": en = m.selfdriveState.enabled or m.selfdriveState.active
        elif w == "carControl": en = en or m.carControl.latActive
        elif w == "driverMonitoringState" and v > 20:
          s = m.driverMonitoringState; vp = s.visionPolicyState
          face.append(vp.faceDetected); dist.append(vp.isDistracted); low.append(str(s.activePolicy) == "wheeltouch")
          aw.append(min(vp.awarenessPercent, s.wheeltouchPolicyState.awarenessPercent) / 100.0)
          ev[str(s.alertLevel)] = ev.get(str(s.alertLevel), 0) + 1
          pos.append((vp.pose.uncertainty, vp.pose.calibrated, vp.wheeltouchFallbackPercent))
        elif w == "driverStateV2" and v > 20:
          ds = m.driverStateV2; d = ds.leftDriverData if not getattr(ds, "isRHD", False) else ds.rightDriverData
          prob.append(d.faceProb); sg.append(d.sunglassesProb)
      except Exception:  # noqa: BLE001
        pass
  if not face:
    print(r, "no DM data"); continue
  face = np.array(face); aw = np.array(aw); prob = np.array(prob); sg = np.array(sg)
  print(f"{r}: DM frames {len(face)} (> 20 km/h) | face detected {face.mean():.1%}, distracted {np.mean(dist):.1%}, low-std {np.mean(low):.1%} | "
        f"awareness min {aw.min():.2f}, < 0.5 {np.mean(aw < 0.5):.2%} | faceProb p10/p50 {np.percentile(prob, 10):.2f}/{np.median(prob):.2f}, sunglasses p50 {np.median(sg):.2f} | "
        f"policy wheeltouch {np.mean(low):.1%} (vision otherwise), pose calibrated {np.mean([p[1] for p in pos]):.0%}, uncertainty p50 {np.median([p[0] for p in pos]):.2f}, "
        f"wheeltouch fallback p95 {np.percentile([p[2] for p in pos], 95):.0f}% | alert levels {ev}", flush=True)
