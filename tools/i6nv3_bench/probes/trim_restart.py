"""Lane-centre offset in the 30-45 km/h band split by time since the last upward crossing of 8.3 m/s."""
import glob, sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
r=sys.argv[1]; files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))[1:]
v=0; press=0; lat=0; blk=0; I=0; gap=0; t_cross=None; prev_v=0; rows=[]
for f in files:
    for m in LogReader(f):
        w=m.which(); t=m.logMonoTime/1e9
        if w=="carState":
            c=m.carState; v=c.vEgo; press=c.steeringPressed; blk=c.leftBlinker or c.rightBlinker
            if prev_v<8.3<=v: t_cross=t
            prev_v=v
        elif w=="carControl": lat=m.carControl.latActive
        elif w=="controlsState": I=m.controlsState.angleFbInteg; gap=m.controlsState.steerCmdGapDeg
        elif w=="modelV2":
            md=m.modelV2
            if len(md.laneLines)<4 or str(md.meta.laneChangeState)!="off": continue
            yl=md.laneLines[1].y[0]; yr=md.laneLines[2].y[0]; p=min(md.laneLineProbs[1],md.laneLineProbs[2])
            if not(lat and not press and not blk and p>0.5 and 2.7<yr-yl<4.0 and 30<=v*3.6<45): continue
            since = (t-t_cross) if t_cross is not None else 999
            rows.append((since, (yl+yr)/2, abs(I)*1e3, abs(gap)))
a=np.array(rows)
print(f"== {r}: hands-off 30-45 km/h good-lines frames {len(a)}")
for lo,hi in ((0,3),(3,6),(6,10),(10,20),(20,1e9)):
    b=a[(a[:,0]>=lo)&(a[:,0]<hi)]
    if len(b)<50: continue
    print(f"   {lo:3.0f}-{hi if hi<1e8 else 999:3.0f} s after crossing 30 km/h: n={len(b):5d} | |offset| median {np.median(np.abs(b[:,1])):.3f} p90 {np.percentile(np.abs(b[:,1]),90):.3f} | signed mean {b[:,1].mean():+.3f} | |integ| median {np.median(b[:,2]):.2f} e-3 | |gap| median {np.median(b[:,3]):.2f}")
