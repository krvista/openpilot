"""Phase 40 kick events from the logs: a fast step of the transmitted angle (>= STEP_DEG within 10 frames, >= 12 deg/s)
away from the wheel while op's own command moved < 0.5 deg — then the wheel response in the next 0.5 s and the gap 1.5 s later.
Also the 1-s transmitted-angle swing p95 with the plan quiet (oscillation watch) for hands-off frames."""
import glob, sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
r=sys.argv[1]; STEP=1.2
files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))[1:]
v=0; press=0; tq=0; blk=0; lat=0; cmd=0; wire=None; wh=[]; ch=[]; rh=[]; ev=[]; pend=[]; swings=[]; lc="off"; kd=0; kh=[]
for f in files:
    seg=int(f.split("--")[-2])
    for m in LogReader(f):
        w=m.which(); t=m.logMonoTime/1e9
        if w=="sendcan":
            for c in m.sendcan:
                if c.address==272 and len(c.dat)>=13:
                    d=bytes(c.dat); raw=((d[10]>>2)|(d[11]<<6))&0x3FFF
                    if raw&0x2000: raw-=0x4000
                    wire=raw*0.1; rh.append((t,wire)); rh=[h for h in rh if t-h[0]<=1.05]
        elif w=="carControl": lat=m.carControl.latActive; cmd=m.carControl.actuators.steeringAngleDeg; ch.append((t,cmd)); ch=[h for h in ch if t-h[0]<=0.12]
        elif w=="controlsState": kd=m.controlsState.desiredCurvature; kh.append((t,kd)); kh=[h for h in kh if t-h[0]<=1.05]
        elif w=="modelV2": lc=str(m.modelV2.meta.laneChangeState)
        elif w=="carState":
            c=m.carState; v=c.vEgo*3.6; press=c.steeringPressed; tq=abs(c.steeringTorque); blk=c.leftBlinker or c.rightBlinker; wheel=c.steeringAngleDeg
            wh.append((t,wheel)); wh=[h for h in wh if t-h[0]<=2.0]
            if wire is None or len(rh)<12: continue
            r10=[h for h in rh if t-h[0]<=0.11]
            if lat and not press and not blk and lc=="off" and tq<50 and 25<=v<60 and len(r10)>=8:
                dwire=wire-r10[0][1]; dcmd=(cmd-ch[0][1]) if len(ch)>2 else 0.0; gap0=r10[0][1]-wheel
                if abs(dwire)>=STEP and abs(dcmd)<0.5 and np.sign(dwire)==np.sign(gap0) and abs(gap0)>2.0 and (not ev or t-ev[-1]["t"]>0.3):
                    ev.append(dict(seg=seg,t=t,dwire=dwire,gap0=gap0,wheel0=wheel,sign=np.sign(gap0),w05=None,gap15=None))
                # oscillation watch: 1-s wire swing while the plan is quiet
                if len(kh)>50 and abs(kh[-1][1]-kh[0][1])<0.3e-3 and abs(kd)<0.5e-3:
                    swings.append(max(h[1] for h in rh)-min(h[1] for h in rh))
            for e in ev[-5:]:
                if e["w05"] is None and t-e["t"]>=0.5: e["w05"]=(wheel-e["wheel0"])*e["sign"]
                if e["gap15"] is None and t-e["t"]>=1.5: e["gap15"]=abs(wire-wheel)
done=[e for e in ev if e["gap15"] is not None]
print(f"== {r}: kick-like steps {len(ev)} (hands-off, 25-60 km/h, wire step >= {STEP} deg in 0.1 s away from the wheel, op cmd quiet)")
if done:
    w=np.array([e["w05"] for e in done]); g0=np.array([abs(e["gap0"]) for e in done]); g1=np.array([e["gap15"] for e in done])
    print(f"   wheel moved toward the request in 0.5 s: p25/p50/p75 {np.percentile(w,25):+.2f}/{np.median(w):+.2f}/{np.percentile(w,75):+.2f} deg | >= 0.5 deg in {np.mean(w>=0.5)*100:.0f}% | gap before p50 {np.median(g0):.1f} -> 1.5 s later {np.median(g1):.1f} deg | gap closed by >= 1 deg in {np.mean(g0-g1>=1)*100:.0f}%")
    for e in done[:8]: print(f"     seg {e['seg']:2d} t={e['t']:.1f} step={e['dwire']:+.1f} gap0={e['gap0']:+.1f} wheel+0.5s={e['w05']:+.2f} gap+1.5s={e['gap15']:.1f}")
if swings: print(f"   plan-quiet 1-s wire swing p50/p95/p99 {np.median(swings):.2f}/{np.percentile(swings,95):.2f}/{np.percentile(swings,99):.2f} deg (n={len(swings)})")
