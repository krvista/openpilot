"""40b ramp events: transmitted angle rising >= 1.6 deg over 0.2 s (>= 8 deg/s) away from the wheel with op's command quiet
(< 0.5 deg over 0.3 s), hands-off (< 50 Nm), 25-60 km/h. Per event: ramp height reached within 0.6 s, wheel trajectory toward
the request, driver reaction (pressed within 2 s, |tq| > 100 Nm within 1 s), gap before -> 1.5 s later."""
import glob, sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
r=sys.argv[1]
files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))[1:]
v=0; press=0; tq=0; blk=0; lat=0; cmd=0; wire=None; rh=[]; ch=[]; lc="off"; ev=[]; last_t=-9
T=[0.2,0.3,0.5,0.8,1.0,1.5,2.0]
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
        elif w=="modelV2": lc=str(m.modelV2.meta.laneChangeState)
        elif w=="carState":
            c=m.carState; v=c.vEgo*3.6; press=c.steeringPressed; tq=abs(c.steeringTorque); blk=c.leftBlinker or c.rightBlinker; wheel=c.steeringAngleDeg
            if wire is None or len(rh)<25: continue
            r20=[h for h in rh if t-h[0]<=0.21]
            if lat and not press and not blk and lc=="off" and tq<50 and 25<=v<60 and len(r20)>=15:
                dw=wire-r20[0][1]; dc=(cmd-ch[0][1]) if len(ch)>3 else 0.0; gap0=r20[0][1]-wheel
                if abs(dw)>=1.6 and abs(dc)<0.5 and np.sign(dw)==np.sign(gap0) and abs(gap0)>2.0 and t-last_t>1.0:
                    ev.append(dict(seg=seg,t=t,v=v,sign=np.sign(gap0),w0=wheel,wire0=r20[0][1],gap0=abs(gap0),traj={},height=0.0,press2=False,tq1=False,gap15=None)); last_t=t
            for e in ev[-4:]:
                dt=t-e["t"]
                if dt<=0.6 and wire is not None: e["height"]=max(e["height"], (wire-e["wire0"])*e["sign"])
                for tt in T:
                    if tt not in e["traj"] and dt>=tt: e["traj"][tt]=(wheel-e["w0"])*e["sign"]
                if press and dt<=2.0: e["press2"]=True
                if tq>100 and dt<=1.0: e["tq1"]=True
                if e["gap15"] is None and dt>=1.5 and wire is not None: e["gap15"]=abs(wire-wheel)
done=[e for e in ev if 2.0 in e["traj"]]
print(f"== {r}: ramp events {len(done)}")
if done:
    print(f"   ramp height within 0.6 s p50/p90 {np.median([e['height'] for e in done]):.1f}/{np.percentile([e['height'] for e in done],90):.1f} deg | gap before p50 {np.median([e['gap0'] for e in done]):.1f} -> 1.5 s later {np.median([e['gap15'] for e in done]):.1f} deg")
    print("   wheel toward request (deg) median at " + " ".join(f"{tt}s:{np.median([e['traj'][tt] for e in done]):+.2f}" for tt in T) + f" | >=1 deg at 1.0 s {np.mean([e['traj'][1.0]>=1 for e in done])*100:.0f}%")
    no=[e for e in done if not e["press2"] and not e["tq1"]]
    if no: print(f"   ... driver did NOT react (n={len(no)}): " + " ".join(f"{tt}s:{np.median([e['traj'][tt] for e in no]):+.2f}" for tt in T))
    print(f"   driver pressed within 2 s: {np.mean([e['press2'] for e in done])*100:.0f}% | |tq|>100 Nm within 1 s: {np.mean([e['tq1'] for e in done])*100:.0f}%")
    for lo,hi in ((25,45),(45,60)):
        b=[e for e in done if lo<=e["v"]<hi]
        if b: print(f"   {lo}-{hi} km/h n={len(b)}: " + " ".join(f"{tt}s:{np.median([e['traj'][tt] for e in b]):+.2f}" for tt in T) + f" | pressed 2 s {np.mean([e['press2'] for e in b])*100:.0f}%")
    for e in done[:10]: print(f"     seg {e['seg']:2d} t={e['t']:.1f} v={e['v']:.0f} gap0={e['gap0']:.1f} height={e['height']:.1f} wheel@0.5={e['traj'][0.5]:+.1f} @1.0={e['traj'][1.0]:+.1f} @2.0={e['traj'][2.0]:+.1f} press2={e['press2']} gap15={e['gap15']:.1f}")
