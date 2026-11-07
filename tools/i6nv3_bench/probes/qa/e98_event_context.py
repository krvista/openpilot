"""E98: 0.25 s timeline around given times of a route: speed / accel, brake / gas, op lateral state, model lead
(prob, distance, speed), longitudinal-plan FCW, car's own FCA/AEB signals (onroadEvents stockFcw/stockAeb, carState
stockFcw / stockAeb), steering wheel / torque. argv: route t_start t_end [t_start t_end ...] (seconds from log start)."""
import glob, sys
from openpilot.tools.lib.logreader import LogReader
D, r = sys.argv[1], sys.argv[2]; W = [(float(a), float(b)) for a, b in zip(sys.argv[3::2], sys.argv[4::2])]
fs = sorted(glob.glob(f"{D}/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
t0 = None; st = {}; nxt = {i: a for i, (a, b) in enumerate(W)}
seg0 = int(min(a for a, b in W) // 60) - 1
for f in fs:
  sidx = int(f.split("--")[-2])
  if sidx < max(seg0, 0) or sidx > int(max(b for a, b in W) // 60) + 1:
    if t0 is None:
      for m in LogReader(f):
        t0 = m.logMonoTime * 1e-9; break
    continue
  for m in LogReader(f):
    try:
      w = m.which(); t = m.logMonoTime * 1e-9; tt = t - t0
      if w == "carState":
        c = m.carState; st.update(v=c.vEgo * 3.6, a=c.aEgo, brake=c.brakePressed, gas=c.gasPressed, ang=c.steeringAngleDeg, tq=c.steeringTorque,
                                  sfcw=c.stockFcw, saeb=c.stockAeb)
      elif w == "modelV2":
        L = m.modelV2.leadsV3
        st.update(lead=(round(L[0].prob, 2), round(L[0].x[0], 1), round(L[0].v[0] * 3.6, 1)) if len(L) else None)
      elif w == "longitudinalPlan": st.update(fcw=m.longitudinalPlan.fcw)
      elif w == "carControl": st.update(lat=m.carControl.latActive, long=m.carControl.longActive)
      for i, (a, b) in enumerate(W):
        if a <= tt <= b and tt >= nxt[i] and 'v' in st:
          nxt[i] = tt + 0.25
          print(f"{tt:7.2f} v {st['v']:5.1f} a {st['a']:+5.1f} brake {int(st['brake'])} gas {int(st['gas'])} lat {int(st.get('lat', 0))} | "
                f"model lead {st.get('lead')} fcw {int(st.get('fcw', 0))} stockFcw {int(st['sfcw'])} stockAeb {int(st['saeb'])} | wheel {st['ang']:+6.1f} tq {st['tq']:+5.0f}")
    except Exception:  # noqa: BLE001
      pass
