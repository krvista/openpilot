"""39a/39b on-road metrics at 30-45 km/h: post-release gap, re-grab interval, truly-hands-off gap, sent gain after release."""
import glob, sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
r=sys.argv[1]; files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))[1:]
v=0; press=0; tq=0; blk=0; lat=0; gap=0; gain=None; last_press=-1e9; last_rel=None; prev=0; lc="off"
rows=[]; regrab=[]
for f in files:
    for m in LogReader(f):
        w=m.which(); t=m.logMonoTime/1e9
        if w=="sendcan":
            for c in m.sendcan:
                if c.address==272 and len(c.dat)>=13: gain=bytes(c.dat)[12]*0.004
        elif w=="carState":
            c=m.carState; v=c.vEgo*3.6; press=c.steeringPressed; tq=abs(c.steeringTorque); blk=c.leftBlinker or c.rightBlinker
            if press and not prev and lat and not blk and lc=="off" and 30<=v<45 and last_rel is not None: regrab.append(t-last_rel)
            if not press and prev: last_rel=t
            if press: last_press=t
            prev=press
            if gain is not None and lat and not press and not blk and lc=="off" and 30<=v<45:
                rows.append((t-last_press, abs(gap), gain, tq))
        elif w=="carControl": lat=m.carControl.latActive
        elif w=="controlsState": gap=m.controlsState.steerCmdGapDeg
        elif w=="modelV2": lc=str(m.modelV2.meta.laneChangeState)
a=np.array(rows); g=np.array(regrab)
print(f"== {r}: 30-45 km/h hands-off latActive frames {len(a)}, grabs with prior release {len(g)}")
for lo,hi in ((0,2),(2,3),(3,10),(10,1e9)):
    b=a[(a[:,0]>=lo)&(a[:,0]<hi)]
    if len(b)>50: print(f"   {lo:.0f}-{hi if hi<1e8 else 999:.0f} s since press: {len(b)/len(a)*100:4.1f}% of frames | gain p50 {np.median(b[:,2]):.2f} | |gap|>3 {np.mean(b[:,1]>3)*100:.0f}%")
ok=a[(a[:,3]<50)&(a[:,0]>=3)]
print(f"   truly hands-off (<50 Nm, >3 s since press): {len(ok)/len(a)*100:.0f}% of frames, |gap|>3 {np.mean(ok[:,1]>3)*100:.0f}%, gain p50 {np.median(ok[:,2]):.2f}")
hand=a[(a[:,3]>=50)&(a[:,3]<100)]
if len(hand)>50: print(f"   50-100 Nm hand frames {len(hand)/len(a)*100:.0f}%: gain p50 {np.median(hand[:,2]):.2f}, |gap|>3 {np.mean(hand[:,1]>3)*100:.0f}%")
if len(g): print(f"   re-grab within 3 s {np.mean(g<3)*100:.0f}%, 5 s {np.mean(g<5)*100:.0f}% | median interval {np.median(g):.1f} s | grabs/min proxy n={len(g)}")
