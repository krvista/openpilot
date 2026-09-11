"""Lane-line confidence and model path uncertainty: night route vs day baseline (latActive frames, >=30 km/h)."""
import glob, sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
r=sys.argv[1]; files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))[1:]
lat=0; v=0; rows=[]
for f in files:
    for m in LogReader(f):
        w=m.which()
        if w=="carControl": lat=m.carControl.latActive
        elif w=="carState": v=m.carState.vEgo*3.6
        elif w=="modelV2" and lat and v>=30:
            md=m.modelV2
            if len(md.laneLineProbs)>=4:
                rows.append((md.laneLineProbs[1], md.laneLineProbs[2], (md.laneLineStds[1] if len(md.laneLineStds)>=4 else 0), md.position.yStd[10] if len(md.position.yStd)>10 else 0, md.meta.laneChangeState==0))
a=np.array(rows, dtype=float)
p=np.minimum(a[:,0],a[:,1])
print(f"== {r}: model frames {len(a)} | min(left,right) prob p10/p50 {np.percentile(p,10):.2f}/{np.median(p):.2f} | both>0.5 {np.mean(p>0.5)*100:.0f}% | both>0.8 {np.mean(p>0.8)*100:.0f}% | either<0.3 {np.mean(p<0.3)*100:.0f}% | lane std y0 p50 {np.median(a[:,2]):.2f} | path yStd@~30m p50/p90 {np.median(a[:,3]):.2f}/{np.percentile(a[:,3],90):.2f}")
