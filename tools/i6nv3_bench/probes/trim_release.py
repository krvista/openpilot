"""Stored-trim release events: transmitted angle falling TOWARD the wheel fast (<= -6 deg/s over 0.3 s) while op's own command
falls slowly (< 1.5 deg / 0.3 s) -> the drop is trim bleed, not the plan. Hands light (< 100 Nm), latActive, 25-70 km/h.
Measures the wheel motion in the direction of the drop (away from the curve) and the lane-centre offset change over 1 s."""
import glob, sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
r=sys.argv[1]; files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))[1:]
v=0; press=0; tq=0; blk=0; lat=0; cmd=0; wire=None; rh=[]; ch=[]; lc="off"; y0=0.0; ev=[]; last_t=-9; kd=0
for f in files:
    for m in LogReader(f):
        w=m.which(); t=m.logMonoTime/1e9
        if w=="sendcan":
            for c in m.sendcan:
                if c.address==272 and len(c.dat)>=13:
                    d=bytes(c.dat); raw=((d[10]>>2)|(d[11]<<6))&0x3FFF
                    if raw&0x2000: raw-=0x4000
                    wire=raw*0.1; rh.append((t,wire)); rh=[h for h in rh if t-h[0]<=0.35]
        elif w=="carControl": lat=m.carControl.latActive; cmd=m.carControl.actuators.steeringAngleDeg; ch.append((t,cmd)); ch=[h for h in ch if t-h[0]<=0.35]
        elif w=="controlsState": kd=m.controlsState.desiredCurvature*1e3
        elif w=="modelV2":
            md=m.modelV2; lc=str(md.meta.laneChangeState)
            if len(md.laneLines)>=4 and min(md.laneLineProbs[1],md.laneLineProbs[2])>0.5: y0=(md.laneLines[1].y[0]+md.laneLines[2].y[0])/2
        elif w=="carState":
            c=m.carState; v=c.vEgo*3.6; press=c.steeringPressed; tq=abs(c.steeringTorque); blk=c.leftBlinker or c.rightBlinker; wheel=c.steeringAngleDeg
            if wire is None or len(rh)<25 or len(ch)<5: continue
            dw=(wire-rh[0][1])/max(t-rh[0][0],1e-3); dc=cmd-ch[0][1]; gap=wire-wheel
            if lat and not press and not blk and lc=="off" and tq<100 and 25<=v<70 and abs(dw)>=6 and abs(dc)<1.5 and np.sign(dw)==-np.sign(gap) and abs(gap)>1.5 and t-last_t>1.5:
                ev.append(dict(t=t,v=v,s=np.sign(dw),w0=wheel,y00=y0,wire0=wire,gap0=abs(gap),traj={},dy=None,tqmax=0.0)); last_t=t
            for e in ev[-3:]:
                dt=t-e["t"]
                if dt<=1.0: e["tqmax"]=max(e["tqmax"],tq)
                for tt in (0.3,0.5,1.0):
                    if tt not in e["traj"] and dt>=tt: e["traj"][tt]=(wheel-e["w0"])*e["s"]
                if e["dy"] is None and dt>=1.0: e["dy"]=(y0-e["y00"])*e["s"]   # lane centre moving in the drop direction = car drifting the other way
done=[e for e in ev if 1.0 in e["traj"] and e["dy"] is not None]
print(f"== {r}: trim-release drops {len(done)} | gap before p50 {np.median([e['gap0'] for e in done]) if done else 0:.1f} deg")
if done:
    for tag,sel in (("all",done),("hands light (<100 Nm after)",[e for e in done if e["tqmax"]<100])):
        if len(sel)<3: continue
        w=[np.median([e["traj"][tt] for e in sel]) for tt in (0.3,0.5,1.0)]
        print(f"   {tag:30s} n={len(sel):3d} | wheel in the drop direction (away from the curve) median 0.3/0.5/1.0 s: {w[0]:+.2f}/{w[1]:+.2f}/{w[2]:+.2f} deg | >=1 deg @1 s {np.mean([e['traj'][1.0]>=1 for e in sel])*100:.0f}% | lane-centre shift @1 s median {np.median([e['dy'] for e in sel]):+.3f} m, >0.1 m {np.mean([e['dy']>0.1 for e in sel])*100:.0f}%")
