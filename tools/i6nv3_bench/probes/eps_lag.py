"""Runs of 'pure EPS lag': hands really off (<50 Nm), >3 s since press, sent gain >= 0.9, command quiet, |gap| > 3 deg, 30-45 km/h.
What do they look like: duration, gap sign vs kDes, wheel motion during the run, whether the gap closes."""
import glob, sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
r=sys.argv[1]; files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))[1:]
v=0; press=0; tq=0; wheel=0; blk=0; lat=0; gap=0; gain=None; last_press=-1e9; cmd=0; kd=0; I=0; hist=[]; lc="off"
runs=[]; cur=None; dumps=[]
for f in files:
    seg=int(f.split("--")[-2])
    for m in LogReader(f):
        w=m.which(); t=m.logMonoTime/1e9
        if w=="sendcan":
            for c in m.sendcan:
                if c.address==272 and len(c.dat)>=13: gain=bytes(c.dat)[12]*0.004
        elif w=="carControl": lat=m.carControl.latActive; cmd=m.carControl.actuators.steeringAngleDeg
        elif w=="controlsState": gap=m.controlsState.steerCmdGapDeg; kd=m.controlsState.desiredCurvature; I=m.controlsState.angleFbInteg
        elif w=="modelV2": lc=str(m.modelV2.meta.laneChangeState)
        elif w=="carState":
            c=m.carState; v=c.vEgo*3.6; press=c.steeringPressed; tq=abs(c.steeringTorque); wheel=c.steeringAngleDeg; blk=c.leftBlinker or c.rightBlinker
            if press: last_press=t
            hist.append((t,cmd)); hist=[h for h in hist if t-h[0]<=0.5]
            quiet=abs(cmd-hist[0][1])<2.0
            ok = gain is not None and lat and not press and not blk and lc=="off" and 30<=v<45 and tq<50 and t-last_press>3 and gain>=0.9 and quiet and abs(gap)>3
            if ok:
                if cur is None: cur=dict(seg=seg,t0=t,gaps=[],wheels=[],cmds=[],kds=[],Is=[],vs=[])
                cur["gaps"].append(gap); cur["wheels"].append(wheel); cur["cmds"].append(cmd); cur["kds"].append(kd*1e3); cur["Is"].append(I*1e3); cur["vs"].append(v)
                cur["t1"]=t
            elif cur is not None:
                cur["dur"]=cur["t1"]-cur["t0"]; runs.append(cur); cur=None
d=np.array([x["dur"] for x in runs]); long=[x for x in runs if x["dur"]>=0.5]
print(f"== {r}: pure-lag runs {len(runs)}, total {d.sum():.0f} s; >=0.5 s: {len(long)} runs, {sum(x['dur'] for x in long):.0f} s, dur p50 {np.median([x['dur'] for x in long]) if long else 0:.2f} p90 {np.percentile([x['dur'] for x in long],90) if long else 0:.2f} s")
if long:
    g=np.array([np.median(np.abs(x["gaps"])) for x in long]); mv=np.array([abs(x["wheels"][-1]-x["wheels"][0]) for x in long]); kk=np.array([np.median(np.abs(x["kds"])) for x in long])
    same=np.mean([np.sign(np.median(x["gaps"]))==np.sign(np.median(x["cmds"])) for x in long])
    print(f"   |gap| median of runs p50 {np.median(g):.2f} p90 {np.percentile(g,90):.2f} deg | wheel moved during run p50 {np.median(mv):.2f} deg | |kDes| p50 {np.median(kk):.2f} e-3 | gap same sign as command (under-delivery, not overshoot) {same*100:.0f}% | |cmd| p50 {np.median([np.median(np.abs(x['cmds'])) for x in long]):.1f} deg | |I| p50 {np.median([np.median(np.abs(x['Is'])) for x in long]):.2f} e-3")
    print("   longest 6 runs (seg, t0, dur, |gap| med, cmd med, wheel first->last, kDes med, v):")
    for x in sorted(long, key=lambda x:-x["dur"])[:6]:
        print(f"     seg {x['seg']:2d} t={x['t0']:.1f} dur={x['dur']:.1f} s gap={np.median(x['gaps']):+.1f} cmd={np.median(x['cmds']):+.1f} wheel {x['wheels'][0]:+.1f}->{x['wheels'][-1]:+.1f} kD={np.median(x['kds']):+.2f} v={np.median(x['vs']):.0f}")
