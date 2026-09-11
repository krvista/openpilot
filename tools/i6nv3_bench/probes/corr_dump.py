"""Dump the 2 s before / 1 s after each hands-off driver grab at 30-45 km/h (route arg), 0.2 s sampling."""
import glob, sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
r=sys.argv[1]; maxn=int(sys.argv[2]) if len(sys.argv)>2 else 10
files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))[1:]
v=0; press=0; tq=0; wheel=0; blk=0; lat=0; cmd=0; I=0; gap=0; lc="off"; y0=0.0; kd=0; kact=0; pcurv=0
hist=[]; prev_press=0; pend=[]; n=0
for f in files:
    seg=int(f.split("--")[-2])
    for m in LogReader(f):
        w=m.which(); t=m.logMonoTime/1e9
        if w=="carState":
            c=m.carState; v=c.vEgo; press=c.steeringPressed; tq=c.steeringTorque; wheel=c.steeringAngleDeg; blk=c.leftBlinker or c.rightBlinker
            hist.append((t,v*3.6,cmd,wheel,gap,I*1e3,tq,int(press),y0,kd*1e3,kact*1e3,lat))
            if len(hist)>300: hist.pop(0)
            if press and not prev_press and lat and not blk and lc=="off" and 30<=v*3.6<45 and n<maxn:
                pre=[h for h in hist if t-2.0<=h[0]<t and h[7]==0 and h[11]]
                if len(pre)>=150:
                    n+=1; pend.append((t, seg, list(hist[-200:])))
            prev_press=press
            for p in pend:
                if t-p[0]<=1.0: p[2].append((t,v*3.6,cmd,wheel,gap,I*1e3,tq,int(press),y0,kd*1e3,kact*1e3,lat))
        elif w=="carControl": lat=m.carControl.latActive; cmd=m.carControl.actuators.steeringAngleDeg
        elif w=="controlsState": I=m.controlsState.angleFbInteg; gap=m.controlsState.steerCmdGapDeg; kd=m.controlsState.desiredCurvature; kact=m.controlsState.curvature
        elif w=="modelV2":
            md=m.modelV2; lc=str(md.meta.laneChangeState)
            if len(md.laneLines)>=4 and min(md.laneLineProbs[1],md.laneLineProbs[2])>0.5: y0=(md.laneLines[1].y[0]+md.laneLines[2].y[0])/2
    done=[p for p in pend if hist and hist[-1][0]-p[0]>1.0]
    for p in done:
        t0=p[0]; print(f"== {r} seg {p[1]} grab at {t0:.1f}:  (t-rel, v, cmd, wheel, gap, integ e-3, driver tq, pressed, lane-centre y0, kDes e-3, kAct e-3)")
        last=-9
        for h in p[2]:
            if h[0]-t0<-2.0 or h[0]-last<0.2: continue
            last=h[0]; print(f"   {h[0]-t0:+.1f} v={h[1]:4.0f} cmd={h[2]:+6.1f} wheel={h[3]:+6.1f} gap={h[4]:+5.1f} I={h[5]:+.2f} tq={h[6]:+5.0f} P={h[7]} y0={h[8]:+.2f} kD={h[9]:+.2f} kA={h[10]:+.2f}")
        pend.remove(p)
    if n>=maxn and not pend: break
