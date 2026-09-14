"""Hands-off-only evaluation from existing logs. Events = transmitted-angle rises away from the wheel (>= 1.2 deg over 0.15 s),
split into KICK (op command quiet, < 0.5 deg / 0.3 s) and NATURAL (op command itself rising >= 1.0 deg / 0.3 s).
Strict hands-off: |column torque| < 30 Nm for the whole 2 s after the event start, no press. Also report the request height
reached in 0.6 s and the wheel trajectory. Aggregates over all routes given."""
import glob, sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
T=[0.2,0.3,0.5,0.8,1.0,1.5,2.0]
allev=[]
for r in sys.argv[1:]:
    files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))[1:]
    v=0; press=0; tq=0; blk=0; lat=0; cmd=0; wire=None; rh=[]; ch=[]; lc="off"; ev=[]; last_t=-9; kd=0
    for f in files:
        seg=int(f.split("--")[-2])
        for m in LogReader(f):
            w=m.which(); t=m.logMonoTime/1e9
            if w=="sendcan":
                for c in m.sendcan:
                    if c.address==272 and len(c.dat)>=13:
                        d=bytes(c.dat); raw=((d[10]>>2)|(d[11]<<6))&0x3FFF
                        if raw&0x2000: raw-=0x4000
                        wire=raw*0.1; rh.append((t,wire)); rh=[h for h in rh if t-h[0]<=0.7]
            elif w=="carControl": lat=m.carControl.latActive; cmd=m.carControl.actuators.steeringAngleDeg; ch.append((t,cmd)); ch=[h for h in ch if t-h[0]<=0.35]
            elif w=="controlsState": kd=abs(m.controlsState.desiredCurvature)*1e3
            elif w=="modelV2": lc=str(m.modelV2.meta.laneChangeState)
            elif w=="carState":
                c=m.carState; v=c.vEgo*3.6; press=c.steeringPressed; tq=abs(c.steeringTorque); blk=c.leftBlinker or c.rightBlinker; wheel=c.steeringAngleDeg
                if wire is None or len(rh)<20: continue
                r15=[h for h in rh if t-h[0]<=0.16]
                if lat and not press and not blk and lc=="off" and tq<50 and 25<=v<60 and len(r15)>=12:
                    dw=wire-r15[0][1]; dc=(cmd-ch[0][1]) if len(ch)>3 else 0.0; gap0=r15[0][1]-wheel
                    if abs(dw)>=1.2 and np.sign(dw)==np.sign(gap0) and abs(gap0)>2.0 and t-last_t>1.0:
                        kind="kick" if abs(dc)<0.5 else ("natural" if abs(dc)>=1.0 else "mixed")
                        ev.append(dict(route=r,seg=seg,t=t,v=v,kind=kind,sign=np.sign(gap0),w0=wheel,wire0=r15[0][1],gap0=abs(gap0),kd=kd,traj={},height=0.0,tqmax=0.0,pressed=False)); last_t=t
                for e in ev[-4:]:
                    dt=t-e["t"]
                    if dt<=2.0:
                        e["tqmax"]=max(e["tqmax"],tq); e["pressed"]=e["pressed"] or bool(press)
                    if dt<=0.6 and wire is not None: e["height"]=max(e["height"],(wire-e["wire0"])*e["sign"])
                    for tt in T:
                        if tt not in e["traj"] and dt>=tt: e["traj"][tt]=(wheel-e["w0"])*e["sign"]
    allev+=[e for e in ev if 2.0 in e["traj"]]
def line(lst, tag):
    if len(lst)<3: print(f"   {tag}: n={len(lst)} (too few)"); return
    print(f"   {tag:44s} n={len(lst):3d} | height p50 {np.median([e['height'] for e in lst]):.1f} | kDes p50 {np.median([e['kd'] for e in lst]):.2f}e-3 | wheel toward request: " + " ".join(f"{tt}s:{np.median([e['traj'][tt] for e in lst]):+.2f}" for tt in T) + f" | >=1 deg @1 s {np.mean([e['traj'][1.0]>=1 for e in lst])*100:.0f}%")
print(f"== routes {sys.argv[1:]}: events {len(allev)}")
for kind in ("kick","natural"):
    k=[e for e in allev if e["kind"]==kind]
    if not k: continue
    tqm=np.array([e["tqmax"] for e in k])
    print(f"   {kind}: n={len(k)} | max |column tq| in the 2 s after start: p10/p25/p50/p75 {np.percentile(tqm,10):.0f}/{np.percentile(tqm,25):.0f}/{np.median(tqm):.0f}/{np.percentile(tqm,75):.0f} Nm | pressed within 2 s {np.mean([e['pressed'] for e in k])*100:.0f}%")
    line(k, f"{kind} all")
    for lo,hi in ((0,60),(60,100),(100,150),(150,9999)):
        b=[e for e in k if lo<=e["tqmax"]<hi and not e["pressed"]]
        line(b, f"{kind} tqmax {lo}-{hi} Nm, no press")
    b=[e for e in k if e["tqmax"]<100 and not e["pressed"]]
    line([e for e in b if e["v"]<45], f"{kind} tqmax<100 25-45 km/h")
    line([e for e in b if e["v"]>=45], f"{kind} tqmax<100 45-60 km/h")
    line([e for e in b if e["height"]>=3.0], f"{kind} tqmax<100 height>=3")
    line([e for e in b if e["height"]<3.0], f"{kind} tqmax<100 height<3")
