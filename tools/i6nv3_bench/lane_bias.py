"""Lane-centre bias: realized (x=0) vs model-intended (path y at 15 m relative to lane centre there)."""
import glob, sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
r=sys.argv[1]; files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))[1:]
v=0; press=0; lat=0; kD=0; blk=0; rows=[]
for f in files:
    for m in LogReader(f):
        w=m.which()
        if w=="carState": c=m.carState; v=c.vEgo; press=c.steeringPressed; blk=c.leftBlinker or c.rightBlinker
        elif w=="carControl": lat=m.carControl.latActive
        elif w=="controlsState": kD=m.controlsState.desiredCurvature
        elif w=="modelV2":
            md=m.modelV2
            if len(md.laneLines)<4 or len(md.position.x)<20 or str(md.meta.laneChangeState)!="off": continue
            xs=np.array(md.laneLines[1].x); yl=np.array(md.laneLines[1].y); yr=np.array(md.laneLines[2].y)
            px=np.array(md.position.x); py=np.array(md.position.y)
            pl=md.laneLineProbs[1]; pr=md.laneLineProbs[2]
            c0=(yl[0]+yr[0])/2; w0=yr[0]-yl[0]
            i15=int(np.argmin(np.abs(xs-15))); c15=(yl[i15]+yr[i15])/2; w15=yr[i15]-yl[i15]
            p15=float(np.interp(15.0, px, py))
            rows.append((v*3.6, int(lat), int(press), int(blk), min(pl,pr), c0, w0, c15, w15, p15, kD*1e3))
a=np.array(rows)
ok=(a[:,1]==1)&(a[:,2]==0)&(a[:,3]==0)&(a[:,4]>0.5)&(a[:,6]>2.7)&(a[:,6]<4.0)&(a[:,0]>=30)
print(f"== {r}: model frames {len(a)}, usable (latActive, hands-off, no blinker/LC, probs>0.5, width 2.7-4.0, >=30 km/h): {ok.sum()}")
print("   convention: laneLines[1]=left has y<0 here, so lane-centre y>0 means the lane centre is to the RIGHT of the camera axis (car sits LEFT of centre)")
def stat(mask, tag):
    b=a[mask]
    if len(b)<200: return
    print(f"   {tag:34s} n={len(b):6d} | realized centre y0: mean {b[:,5].mean():+.3f} med {np.median(b[:,5]):+.3f} | intended (path15 - centre15): mean {(b[:,9]-b[:,7]).mean():+.3f} med {np.median(b[:,9]-b[:,7]):+.3f} | width {np.median(b[:,6]):.2f}")
stat(ok,"all")
stat(ok&(np.abs(a[:,10])<0.3),"straight |kD|<0.3e-3")
stat(ok&(a[:,10]>0.5),"right-ish curve kD>0.5")
stat(ok&(a[:,10]<-0.5),"left-ish curve kD<-0.5")
for lo,hi in ((30,45),(45,60),(60,80),(80,130)): stat(ok&(a[:,0]>=lo)&(a[:,0]<hi)&(np.abs(a[:,10])<0.3), f"straight {lo}-{hi} km/h")
for lo,hi in ((2.7,3.1),(3.1,3.4),(3.4,4.0)): stat(ok&(a[:,6]>=lo)&(a[:,6]<hi)&(np.abs(a[:,10])<0.3), f"straight width {lo}-{hi} m")
stat(ok&(a[:,4]>0.9)&(np.abs(a[:,10])<0.3),"straight probs>0.9")
hi=(a[:,1]==1)&(a[:,2]==1)&(a[:,3]==0)&(a[:,4]>0.5)&(a[:,6]>2.7)&(a[:,6]<4.0)&(a[:,0]>=30)&(np.abs(a[:,10])<0.3)
stat(hi,"straight, driver HANDS ON (reference)")
