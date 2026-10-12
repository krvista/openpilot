"""One segment -> roll summary json. Usage: segstat.py <rlog.zst> <out.json>"""
import sys, json, numpy as np
from openpilot.tools.lib.logreader import LogReader
v=0.; ang=0.; dm=np.nan; vp=np.nan; off=np.nan; cal=[np.nan]*3; sr=15.3
S=dict(st=[], stop=[], vis=[], vp=[], off=[], cal=[])
for m in LogReader(sys.argv[1]):
  w=m.which()
  if w=="carState": v=m.carState.vEgo; ang=m.carState.steeringAngleDeg
  elif w in ("deviceMotion","livePose"): dm=getattr(m,w).orientationNED.x
  elif w in ("vehicleParameters","liveParameters"):
    p=getattr(m,w); vp=p.roll; off=p.angleOffsetAverageDeg; sr=p.steerRatio or 15.3
  elif w in ("extrinsicsCalibration","liveCalibration"):
    c=list(getattr(m,w).rpyCalib)
    if len(c)==3: cal=c
  elif w=="modelV2":
    d=m.modelV2; L=list(d.laneLines); pr=list(d.laneLineProbs)
    ay=v*v*np.radians(ang-(0 if np.isnan(off) else off))/(sr*2.965)
    if np.isfinite(dm):
      if abs(v)<0.3: S['stop'].append(dm)
      elif v>30/3.6 and abs(ay)<0.3:
        S['st'].append(dm); S['vp'].append(vp)
        if len(L)==4 and len(pr)==4 and pr[1]>0.7 and pr[2]>0.7: S['vis'].append((L[1].z[0]-L[2].z[0])/(L[2].y[0]-L[1].y[0]))
    S['off'].append(off); S['cal'].append(cal)
med=lambda x: float(np.degrees(np.nanmedian(x))) if len(x) else None
c=np.array(S['cal'],float) if S['cal'] else np.full((1,3),np.nan)
json.dump(dict(n_st=len(S['st']), n_stop=len(S['stop']), st=med(S['st']), stop=med(S['stop']), vis=med(S['vis']), vp=med(S['vp']),
               off=float(np.nanmedian(S['off'])) if S['off'] else None, cal_p=float(np.degrees(np.nanmedian(c[:,1]))), cal_y=float(np.degrees(np.nanmedian(c[:,2]))),
               cal_r=float(np.degrees(np.nanmedian(c[:,0])))), open(sys.argv[2],"w"))
