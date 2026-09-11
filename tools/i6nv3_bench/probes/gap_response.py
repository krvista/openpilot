"""MDPS response vs request distance: hands-off (<50 Nm, >3 s since press), sent gain >= 0.9, latActive, 30-60 km/h.
Bin |cmd-wheel| and look at the wheel rate TOWARD the command and the EPS torque. Deadband => response grows with gap.
Clamp ('too far -> ignore') => response drops to ~0 above some gap."""
import glob, sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
r=sys.argv[1]; files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))[1:]
v=0; press=0; tq=0; blk=0; lat=0; gap=0; gain=None; last_press=-1e9; lc="off"; cmd=0; hist=[]; whist=[]
rows=[]
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
            c=m.carState; v=c.vEgo*3.6; press=c.steeringPressed; tq=abs(c.steeringTorque); blk=c.leftBlinker or c.rightBlinker
            if press: last_press=t
            hist.append((t,cmd)); hist=[h for h in hist if t-h[0]<=0.3]
            whist.append((t,c.steeringAngleDeg)); whist=[h for h in whist if t-h[0]<=0.2]
            drate=(c.steeringAngleDeg-whist[0][1])/max(t-whist[0][0],1e-3) if len(whist)>3 else 0.0
            if gain is not None and lat and not press and not blk and lc=="off" and 30<=v<60 and tq<50 and t-last_press>3 and gain>=0.9:
                s=np.sign(gap) if gap!=0 else 1.0
                rows.append((abs(gap), drate*s, abs(c.steeringTorqueEps), abs(cmd-hist[0][1]), abs(c.steeringAngleDeg)))
a=np.array(rows)
print(f"== {r}: frames {len(a)} (hands-off, gain>=0.9, 30-60 km/h)")
print("   |gap| bin | n | wheel rate toward cmd: p25 / p50 / p75 (deg/s) | rate toward >5 deg/s % | |EPS tq| p50 | cmd moving (>1 deg/0.3 s) % | |wheel| p50")
for lo,hi in ((0,0.5),(0.5,1),(1,2),(2,3),(3,4),(4,5),(5,7),(7,10),(10,90)):
    b=a[(a[:,0]>=lo)&(a[:,0]<hi)]
    if len(b)<100: continue
    print(f"   {lo:4.1f}-{hi:4.1f} | {len(b):6d} | {np.percentile(b[:,1],25):+5.1f} / {np.median(b[:,1]):+5.1f} / {np.percentile(b[:,1],75):+5.1f} | {np.mean(b[:,1]>5)*100:4.0f}% | {np.median(b[:,2]):5.0f} | {np.mean(b[:,3]>1)*100:3.0f}% | {np.median(b[:,4]):4.1f}")
# quiet-command subset
q=a[a[:,3]<=1.0]
print("   -- command quiet subset (|dcmd| <= 1 deg per 0.3 s):")
for lo,hi in ((1,2),(2,3),(3,4),(4,5),(5,7),(7,90)):
    b=q[(q[:,0]>=lo)&(q[:,0]<hi)]
    if len(b)<100: continue
    print(f"   {lo:4.1f}-{hi:4.1f} | {len(b):6d} | {np.percentile(b[:,1],25):+5.1f} / {np.median(b[:,1]):+5.1f} / {np.percentile(b[:,1],75):+5.1f} | {np.mean(b[:,1]>5)*100:4.0f}% | {np.median(b[:,2]):5.0f} | wheel static (|rate|<1) {np.mean(np.abs(b[:,1])<1)*100:3.0f}%")
