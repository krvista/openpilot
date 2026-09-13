"""Kick events (fast wire step away from the wheel, op cmd quiet): median wheel-toward-request trajectory over 1.5 s,
split by speed and by first vs later step; driver grab within 2 s; and the natural comparison: plan-driven sustained fast
rises (wire rate >= 6 deg/s for >= 0.3 s, no kick) -> wheel motion over 0.5 s."""
import glob, sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
r=sys.argv[1]; STEP=1.2
files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))[1:]
v=0; press=0; tq=0; blk=0; lat=0; cmd=0; wire=None; rh=[]; ch=[]; lc="off"; kd=0
ev=[]; nat=[]; nat_start=None; last_ev_t=-9; wh=[]
T=[0.1,0.2,0.3,0.5,0.8,1.0,1.5]
for f in files:
    seg=int(f.split("--")[-2])
    for m in LogReader(f):
        w=m.which(); t=m.logMonoTime/1e9
        if w=="sendcan":
            for c in m.sendcan:
                if c.address==272 and len(c.dat)>=13:
                    d=bytes(c.dat); raw=((d[10]>>2)|(d[11]<<6))&0x3FFF
                    if raw&0x2000: raw-=0x4000
                    wire=raw*0.1; rh.append((t,wire)); rh=[h for h in rh if t-h[0]<=0.35]
        elif w=="carControl": lat=m.carControl.latActive; cmd=m.carControl.actuators.steeringAngleDeg; ch.append((t,cmd)); ch=[h for h in ch if t-h[0]<=0.35]
        elif w=="modelV2": lc=str(m.modelV2.meta.laneChangeState)
        elif w=="carState":
            c=m.carState; v=c.vEgo*3.6; press=c.steeringPressed; tq=abs(c.steeringTorque); blk=c.leftBlinker or c.rightBlinker; wheel=c.steeringAngleDeg
            if wire is None or len(rh)<12: continue
            r10=[h for h in rh if t-h[0]<=0.11]; c10=[h for h in ch if t-h[0]<=0.11]
            ok = lat and not press and not blk and lc=="off" and tq<50 and 25<=v<60
            if ok and len(r10)>=8:
                dwire=wire-r10[0][1]; dcmd=(cmd-c10[0][1]) if len(c10)>2 else 0.0; gap0=r10[0][1]-wheel
                if abs(dwire)>=STEP and abs(dcmd)<0.5 and np.sign(dwire)==np.sign(gap0) and abs(gap0)>2.0 and t-last_ev_t>0.3:
                    ev.append(dict(t=t,v=v,seg=seg,sign=np.sign(gap0),w0=wheel,gap0=abs(gap0),first=(t-last_ev_t>3.0),traj={},grab=False,dw=abs(dwire))); last_ev_t=t
                # natural sustained rise: wire rate over 0.3 s >= 6 deg/s, op cmd moving too (plan-driven), not within 1 s of a kick
                r30=(wire-rh[0][1])/max(t-rh[0][0],1e-3); c30=(cmd-ch[0][1])/max(t-ch[0][0],1e-3) if len(ch)>2 else 0
                if abs(r30)>=6 and abs(c30)>=4 and np.sign(r30)==np.sign(gap0) and abs(gap0)>2.0 and t-last_ev_t>1.0 and (nat_start is None or t-nat_start>1.0):
                    nat.append(dict(t=t,v=v,w0=wheel,sign=np.sign(gap0),traj={},dw=abs(dwire))); nat_start=t
            for e in ev[-6:]:
                dt=t-e["t"]
                for tt in T:
                    if tt not in e["traj"] and dt>=tt: e["traj"][tt]=(wheel-e["w0"])*e["sign"]
                if press and dt<=2.0: e["grab"]=True
            for e in nat[-4:]:
                dt=t-e["t"]
                for tt in T:
                    if tt not in e["traj"] and dt>=tt: e["traj"][tt]=(wheel-e["w0"])*e["sign"]
def traj_line(lst, tag):
    lst=[e for e in lst if 1.5 in e["traj"]]
    if not lst: print(f"   {tag}: none"); return
    print(f"   {tag} n={len(lst)}: wheel toward request (deg) median at " + " ".join(f"{tt}s:{np.median([e['traj'][tt] for e in lst]):+.2f}" for tt in T) + f" | >=0.5 deg at 0.5 s {np.mean([e['traj'][0.5]>=0.5 for e in lst])*100:.0f}%")
print(f"== {r}: kick events {len(ev)}, natural sustained rises {len(nat)}")
traj_line(ev, "kick all")
traj_line([e for e in ev if e["first"]], "kick first-in-episode")
traj_line([e for e in ev if not e["first"]], "kick follow-up")
traj_line([e for e in ev if e["v"]<45], "kick 25-45 km/h")
traj_line([e for e in ev if e["v"]>=45], "kick 45-60 km/h")
traj_line(nat, "natural plan rise >=6 deg/s x0.3 s")
traj_line([e for e in nat if e["v"]<45], "natural rise 25-45 km/h")
traj_line([e for e in nat if e["v"]>=45], "natural rise 45-60 km/h")
traj_line([e for e in ev if e["dw"]<1.8], "kick step 1.2-1.8 deg")
traj_line([e for e in ev if e["dw"]>=1.8], "kick step >=1.8 deg")
if ev: print(f"   driver grabbed within 2 s of a kick: {np.mean([e['grab'] for e in ev])*100:.0f}% | kicks per hands-off 25-60 km/h minute: see release metrics")
