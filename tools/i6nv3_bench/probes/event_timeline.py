import glob, sys
from openpilot.tools.lib.logreader import LogReader
r, seg, t_lo, t_hi = sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
f0 = sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--0--rlog.zst"))[0]
t0 = next(iter(LogReader(f0))).logMonoTime * 1e-9
f = glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--{seg}--rlog.zst")[0]
last = 0
st = {}
for m in LogReader(f):
  t = m.logMonoTime * 1e-9 - t0; w = m.which()
  if t < t_lo or t > t_hi: continue
  if w == "carState":
    c = m.carState; st.update(v=c.vEgo * 3.6, a=c.aEgo, brake=c.brakePressed, gas=c.gasPressed, stockFcw=c.stockFcw, stockAeb=c.stockAeb, ang=c.steeringAngleDeg, tq=c.steeringTorque)
  elif w == "radarState":
    l = m.radarState.leadOne; st.update(dRel=l.dRel, vRel=l.vRel, vLead=l.vLead * 3.6, aLead=l.aLeadK)
  elif w == "longitudinalPlan":
    st.update(planfcw=m.longitudinalPlan.fcw)
  elif w == "modelV2":
    st.update(hardBrake=m.modelV2.meta.hardBrakePredicted, lead_prob=(m.modelV2.leadsV3[0].prob if len(m.modelV2.leadsV3) else -1))
  elif w == "onroadEvents":
    n = [str(e.name) for e in m.onroadEvents if str(e.name) not in ("steerOverride", "gasPressedOverride")]
    if n: st["ev"] = ",".join(n)
    else: st.pop("ev", None)
  if t - last >= 0.25 and st:
    last = t; print(f"{t:7.2f} " + " ".join(f"{k}={v:.2f}" if isinstance(v, float) else f"{k}={v}" for k, v in st.items()))
