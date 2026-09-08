#!/usr/bin/env python3
"""A/B lane-keeping comparison of two routes under MATCHED conditions: hands-off, no lane change / blinker, both
inner lane lines confident, per speed bin, plus GPS+speed-matched location pairs (same spot within 25 m, speed
within 10 km/h). Built after route 00000008 showed how a raw comparison is confounded by driving style
(hands-off share, speed, lane changes). usage: PYTHONPATH=$PWD:$PWD/opendbc_repo python3 tools/i6nv3_bench/ab_lane_compare.py <routeA> <nA> <routeB> <nB>"""
import sys, glob, numpy as np
from openpilot.tools.lib.logreader import LogReader
def load(r, n):
    rows=[]; lc=0; blink=0; tot=0; ho=0
    for f in sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))[:n]:
        cc=None; cs=None; gps=None; lcs="off"
        for m in LogReader(f):
            w=m.which()
            if w=="carControl": cc=m.carControl
            elif w=="carState": cs=m.carState
            elif w in ("gpsLocationExternal","gpsLocation"):
                g=getattr(m,w)
                if g.hasFix: gps=(g.latitude,g.longitude)
            elif w=="onroadEvents":
                if any(str(e.name)=="laneChange" for e in m.onroadEvents): lc+=1
            elif w=="modelV2" and cc is not None and cs is not None:
                mv=m.modelV2; lcs=str(mv.meta.laneChangeState); p=list(mv.laneLineProbs)
                if cc.latActive and cs.vEgo*3.6>30:
                    tot+=1
                    handsoff = abs(cs.steeringTorque)<40 and not cs.steeringPressed
                    if handsoff: ho+=1
                    if handsoff and lcs=="off" and not (cs.leftBlinker or cs.rightBlinker) and len(p)>=4 and p[1]>0.5 and p[2]>0.5:
                        off=(mv.laneLines[1].y[0]+mv.laneLines[2].y[0])/2
                        rows.append((cs.vEgo*3.6, abs(off), gps[0] if gps else np.nan, gps[1] if gps else np.nan, abs(cs.steeringAngleDeg)))
    return np.array(rows), lc, tot, ho
A,lcA,totA,hoA=load(sys.argv[1],int(sys.argv[2])); B,lcB,totB,hoB=load(sys.argv[3],int(sys.argv[4]))
print(f"route A: latActive>30 frames {totA}, hands-off {100*hoA/max(totA,1):.0f} %, lane-change frames {lcA}, speed median {np.median(A[:,0]):.0f} km/h | route B: {totB}, hands-off {100*hoB/max(totB,1):.0f} %, LC frames {lcB}, speed median {np.median(B[:,0]):.0f}")
print("hands-off, no LC/blinker, good lines — lane-centre |offset| by speed bin (median / p90, n):")
for lo,hi in ((30,45),(45,60),(60,80),(80,130)):
    a=A[(A[:,0]>=lo)&(A[:,0]<hi)]; b=B[(B[:,0]>=lo)&(B[:,0]<hi)]
    if len(a)>50 and len(b)>50: print(f"   {lo}-{hi} km/h: route A {np.median(a[:,1]):.2f} / {np.percentile(a[:,1],90):.2f} (n={len(a)}) | route B {np.median(b[:,1]):.2f} / {np.percentile(b[:,1],90):.2f} (n={len(b)})")
# GPS-matched pairs: route-8 frame -> nearest route-6 frame within 25 m, speed within 10 km/h
ga=A[~np.isnan(A[:,2])]; gb=B[~np.isnan(B[:,2])]
print(f"frames with GPS: route A {len(ga)}, route B {len(gb)}")
if len(ga)>100 and len(gb)>100:
    la=np.radians(ga[:,2]); lo_a=np.radians(ga[:,3]); pairs=[]
    for row in gb[::3]:
        d=6371000*np.sqrt(((np.radians(row[2])-la)*1)**2+((np.radians(row[3])-lo_a)*np.cos(la))**2)
        j=np.argmin(d)
        if d[j]<25 and abs(ga[j,0]-row[0])<10: pairs.append((ga[j,1], row[1], row[0]))
    P=np.array(pairs)
    if len(P): print(f"GPS+speed-matched pairs: {len(P)} | |offset| median route A {np.median(P[:,0]):.2f} m vs route B {np.median(P[:,1]):.2f} m | route B worse in {100*np.mean(P[:,1]>P[:,0]):.0f} % of pairs | mean speed of pairs {P[:,2].mean():.0f} km/h")
