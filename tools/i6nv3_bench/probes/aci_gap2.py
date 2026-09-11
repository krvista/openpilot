"""Hands-off latActive frames with |cmd-wheel| > 3 deg: split by raw column torque, ACI gain, time since press, command motion."""
import glob, sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
r=sys.argv[1]; files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))[1:]
v=0; press=0; tq=0; wheel=0; blk=0; lat=0; gap=0; gain=None; last_press_t=-1e9; cmd=0; cmd_hist=[]
rows=[]
for f in files:
    for m in LogReader(f):
        w=m.which(); t=m.logMonoTime/1e9
        if w=="sendcan":
            for c in m.sendcan:
                if c.address==272 and len(c.dat)>=13: gain=bytes(c.dat)[12]*0.004
        elif w=="carState":
            c=m.carState; v=c.vEgo; press=c.steeringPressed; tq=c.steeringTorque; wheel=c.steeringAngleDeg; blk=c.leftBlinker or c.rightBlinker
            if press: last_press_t=t
            cmd_hist.append((t,cmd)); cmd_hist=[h for h in cmd_hist if t-h[0]<=0.5]
            if gain is not None and lat and not press and not blk and v*3.6>=22:
                rows.append((v*3.6, abs(gap), gain, t-last_press_t, abs(tq), abs(cmd-cmd_hist[0][1])))
        elif w=="carControl": lat=m.carControl.latActive; cmd=m.carControl.actuators.steeringAngleDeg
        elif w=="controlsState": gap=m.controlsState.steerCmdGapDeg
a=np.array(rows)
print(f"== {r}: hands-off latActive frames {len(a)}")
for lo,hi in ((22,30),(30,45),(45,60),(60,130)):
    b=a[(a[:,0]>=lo)&(a[:,0]<hi)]
    if len(b)<200: continue
    big=b[b[:,1]>3]
    hand=big[:,4]>=100; light=(big[:,4]>=50)&(big[:,4]<100); nohand=big[:,4]<50
    recent=big[:,3]<3
    fullg=big[:,2]>=0.9
    moving=big[:,5]>2.0
    pure=nohand & ~recent & fullg & ~moving
    print(f"   {lo:3d}-{hi:3d} km/h: frames {len(b):6d}, resting-hand (|tq|>=100 Nm) share of ALL hands-off frames {np.mean(b[:,4]>=100)*100:.0f}% | |gap|>3 frames {len(big)} ({len(big)/len(b)*100:.1f}%) of which:")
    print(f"        |tq|>=100 Nm {np.mean(hand)*100:.0f}% | 50-100 {np.mean(light)*100:.0f}% | <50 {np.mean(nohand)*100:.0f}% || <3 s since press {np.mean(recent)*100:.0f}% | gain<0.5 {np.mean(big[:,2]<0.5)*100:.0f}% | gain>=0.9 {np.mean(fullg)*100:.0f}% | cmd moved >2 deg in 0.5 s {np.mean(moving)*100:.0f}%")
    print(f"        'pure EPS lag' (no hand <50 Nm, >3 s since press, gain>=0.9, command quiet): {np.mean(pure)*100:.0f}% of |gap|>3 frames = {np.mean(pure)*len(big)/len(b)*100:.1f}% of all hands-off frames in the bin")
    ok=b[(b[:,4]<50)&(b[:,3]>=3)]
    if len(ok)>100: print(f"        truly hands-off (<50 Nm, >3 s since press) frames {len(ok)} ({len(ok)/len(b)*100:.0f}%): |gap|>3 {np.mean(ok[:,1]>3)*100:.0f}%, gain p50 {np.median(ok[:,2]):.2f}")
