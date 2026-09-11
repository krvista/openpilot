"""Driver-correction onsets: what did the controller look like in the 1.5 s before the driver grabbed the wheel?"""
import glob, sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
r=sys.argv[1]; files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))[1:]
hist=[]  # (t, v, gap, integ, cmd, wheel, tq, pressed, lat, blk, lc, y0)
v=0; press=0; tq=0; wheel=0; blk=0; lat=0; cmd=0; I=0; gap=0; lc="off"; y0=None; kd=0
eps=[]; sign_hits=[]; prev_press=0
for f in files:
    for m in LogReader(f):
        w=m.which(); t=m.logMonoTime/1e9
        if w=="carState":
            c=m.carState; v=c.vEgo; press=c.steeringPressed; tq=c.steeringTorque; wheel=c.steeringAngleDeg; blk=c.leftBlinker or c.rightBlinker
            hist.append((t,v,gap,I,cmd,wheel,tq,press,lat,blk,lc,y0 if y0 is not None else 0.0,kd))
            if len(hist)>400: hist.pop(0)
            if press and not prev_press and lat and not blk and lc=="off" and v*3.6>=25:
                pre=[h for h in hist if t-1.5<=h[0]<t and h[7]==0 and h[8]]
                if len(pre)>=60:
                    g=np.array([h[2] for h in pre]); cm=np.array([h[4] for h in pre]); i0=np.array([h[3] for h in pre])
                    eps.append(dict(t=t, v=v*3.6, gap_pre=float(np.mean(g)), absgap_pre=float(np.median(np.abs(g))), gap_last=float(g[-10:].mean()), integ_pre=float(np.mean(np.abs(i0))*1e3), cmd_rate=float(np.abs(np.diff(cm)).sum()), y0=float(pre[-1][11]), kd=float(pre[-1][12]), onset_i=len(eps)))
                    eps[-1]["tq_on"]=[]; eps[-1]["dwheel"]=[]; eps[-1]["t_on"]=t; eps[-1]["w_on"]=wheel
            # collect torque during the first 0.4 s after any onset
            for e in eps[-3:]:
                if t-e["t_on"]<=0.4: e["tq_on"].append(tq)
                elif t-e["t_on"]<=1.0: e["dwheel"].append(wheel-e["w_on"])
            prev_press=press
        elif w=="carControl": lat=m.carControl.latActive; cmd=m.carControl.actuators.steeringAngleDeg
        elif w=="controlsState": I=m.controlsState.angleFbInteg; gap=m.controlsState.steerCmdGapDeg; kd=m.controlsState.desiredCurvature
        elif w=="modelV2":
            md=m.modelV2; lc=str(md.meta.laneChangeState)
            if len(md.laneLines)>=4 and min(md.laneLineProbs[1],md.laneLineProbs[2])>0.5: y0=(md.laneLines[1].y[0]+md.laneLines[2].y[0])/2
# torque sign convention: sign of mean onset torque vs sign of wheel motion in the following 0.6 s
conv=[np.sign(np.mean(e["tq_on"]))*np.sign(np.mean(e["dwheel"])) for e in eps if len(e["tq_on"])>5 and len(e["dwheel"])>5 and abs(np.mean(e["tq_on"]))>30 and abs(np.mean(e["dwheel"]))>0.5]
s=np.sign(np.mean(conv)) if conv else 1.0
print(f"== {r}: onsets {len(eps)} (latActive, hands-off>=1.5 s before, no blinker/LC, >=25 km/h); torque sign vs wheel motion agreement {np.mean(np.array(conv)>0)*100 if conv else 0:.0f}% (n={len(conv)}) -> torque sign factor {s:+.0f}")
def report(sel, tag):
    if len(sel)<8: return
    g3=np.mean([e["absgap_pre"]>3 for e in sel])*100
    helping=[]; centring=[]; still=[]
    for e in sel:
        if len(e["tq_on"])<5: continue
        tqs=s*np.sign(np.mean(e["tq_on"]))
        if abs(e["gap_last"])>1.5: helping.append(tqs==np.sign(e["gap_last"]))
        if abs(e["y0"])>0.1: centring.append(tqs==np.sign(e["y0"]))
        still.append(e["cmd_rate"]<3.0)
    print(f"   {tag:14s} n={len(sel):3d} | pre |gap| median {np.median([e['absgap_pre'] for e in sel]):.2f} deg, >3 deg in {g3:.0f}% | driver pushes toward command (|gap|>1.5) {np.mean(helping)*100 if helping else 0:.0f}% of {len(helping)} | pushes toward lane centre (|y0|>0.1 m) {np.mean(centring)*100 if centring else 0:.0f}% of {len(centring)} | command quiet {np.mean(still)*100:.0f}% | |y0| median {np.median([abs(e['y0']) for e in sel]):.2f} m | |integ| median {np.median([e['integ_pre'] for e in sel]):.2f} e-3")
report(eps,"all")
for lo,hi in ((25,30),(30,45),(45,60),(60,130)): report([e for e in eps if lo<=e["v"]<hi], f"{lo}-{hi} km/h")
