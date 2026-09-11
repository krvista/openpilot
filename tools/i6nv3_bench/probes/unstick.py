"""How do stalls end? For each hands-off stall (gain>=0.9, |gap|>2.5 deg, wheel static >=0.3 s, 25-60 km/h), at the first frame the wheel
starts moving (>=0.3 deg within 0.2 s): did the request move in the preceding 0.3 s (|dcmd|>0.5) and in which direction (toward / away from the wheel)?"""
import glob, sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
r=sys.argv[1]; files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))[1:]
v=0; press=0; tq=0; blk=0; lat=0; gap=0; gain=None; last_press=-1e9; lc="off"; cmd=0
ch=[]; wh=[]; stall_t=None; ends=[]; stalls=0
for f in files:
    for m in LogReader(f):
        w=m.which(); t=m.logMonoTime/1e9
        if w=="sendcan":
            for c in m.sendcan:
                if c.address==272 and len(c.dat)>=13: gain=bytes(c.dat)[12]*0.004
        elif w=="carControl": lat=m.carControl.latActive; cmd=m.carControl.actuators.steeringAngleDeg
        elif w=="controlsState": gap=m.controlsState.steerCmdGapDeg
        elif w=="modelV2": lc=str(m.modelV2.meta.laneChangeState)
        elif w=="carState":
            c=m.carState; v=c.vEgo*3.6; press=c.steeringPressed; tq=abs(c.steeringTorque); blk=c.leftBlinker or c.rightBlinker; wheel=c.steeringAngleDeg
            if press: last_press=t
            ch.append((t,cmd)); ch=[h for h in ch if t-h[0]<=0.35]; wh.append((t,wheel)); wh=[h for h in wh if t-h[0]<=0.35]
            ok = gain is not None and lat and not press and not blk and lc=="off" and 25<=v<60 and tq<50 and t-last_press>3 and gain>=0.9
            w02=[h for h in wh if t-h[0]<=0.2]; moved = len(w02)>2 and abs(wheel-w02[0][1])>=0.3
            static03 = len(wh)>5 and max(abs(h[1]-wheel) for h in wh)<0.3
            if ok and abs(gap)>2.5 and static03 and stall_t is None: stall_t=t; stalls+=1
            if stall_t is not None:
                if not ok or abs(gap)<=1.5: stall_t=None
                elif moved:
                    c03=[h for h in ch if t-h[0]<=0.35]; dcmd=cmd-c03[0][1]; toward_wheel = np.sign(dcmd)==-np.sign(gap)
                    ends.append((t-stall_t, abs(dcmd), toward_wheel, np.sign(wheel-w02[0][1])==np.sign(gap))); stall_t=None
e=np.array(ends, dtype=float)
print(f"== {r}: stalls {stalls}, ended by wheel motion {len(e)} | stall length before motion p50 {np.median(e[:,0]):.2f} s")
mv=e[:,1]>0.5
print(f"   request moved (>0.5 deg / 0.35 s) right before the wheel moved: {np.mean(mv)*100:.0f}% | of those, request moved TOWARD the wheel (retreat) {np.mean(e[mv,2])*100:.0f}%, away {100-np.mean(e[mv,2])*100:.0f}% | wheel moved toward the request {np.mean(e[:,3])*100:.0f}%")
print(f"   request quiet at unstick: {np.mean(~mv)*100:.0f}% (wheel moved toward request {np.mean(e[~mv,3])*100 if (~mv).any() else 0:.0f}%)")
