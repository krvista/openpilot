import glob, sys, math, numpy as np
from openpilot.tools.lib.logreader import LogReader
r=sys.argv[1]; MINGAP=60
files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))
# pass 1: find first GPS gap > MINGAP
last=None; gap=None
for f in files:
    for m in LogReader(f):
        w=m.which()
        if w in("gpsLocationExternal","gpsLocation"):
            g=getattr(m,w)
            if not g.hasFix or g.unixTimestampMillis==0: continue
            t=m.logMonoTime/1e9
            if last and t-last>MINGAP: gap=(last,t); break
            last=t
    if gap: break
if not gap: print(r,"no tunnel gap"); sys.exit()
t0,t1=gap; print(f"== {r}: GPS gap {t0:.1f} -> {t1:.1f} s ({t1-t0:.0f} s)")
# pass 2: collect samples in [t0-20, t1+20]
segs=[f for f in files if any(abs(int(f.split('--')[-2])*60 - x) < 130 for x in (t0,t1)) or t0 < int(f.split('--')[-2])*60 < t1]
rows=[]; v=0; tq=0; press=0; ang=0; angcmd=None; lat=0; gap_deg=0; integ=0; kdes=0; drop=0; lc=0; cs_t=0
last_v_t=None; s=0.0
for f in segs:
    for m in LogReader(f):
        t=m.logMonoTime/1e9
        if t<t0-20 or t>t1+20: continue
        w=m.which()
        if w=="carState":
            c=m.carState; v=c.vEgo; tq=c.steeringTorque; press=c.steeringPressed; ang=c.steeringAngleDeg
            if last_v_t is not None and t>=t0: s+=v*(t-last_v_t)
            last_v_t=t
        elif w=="carControl": angcmd=m.carControl.actuators.steeringAngleDeg; lat=m.carControl.latActive
        elif w=="controlsState":
            c=m.controlsState; gap_deg=c.steerCmdGapDeg; integ=c.angleFbInteg; kdes=c.curvature; drop=c.laneDropout
        elif w=="modelV2":
            md=m.modelV2
            if len(md.laneLines)<4: continue
            yl=md.laneLines[1].y[0]; yr=md.laneLines[2].y[0]; pl=md.laneLineProbs[1]; pr=md.laneLineProbs[2]
            lc=str(md.meta.laneChangeState)
            rows.append((t, s if t>=t0 else -1, v*3.6, (yl+yr)/2, yr-yl, min(pl,pr), gap_deg, integ*1e3, kdes*1e3, lat, press, abs(tq), ang, angcmd if angcmd is not None else ang, drop, lc!="off"))
a=np.array(rows, dtype=float)
inn=a[(a[:,1]>=0)]
print(f"   in-gap frames {len(inn)}, distance {inn[:,1].max():.0f} m, speed median {np.median(inn[:,2]):.0f} km/h (p10 {np.percentile(inn[:,2],10):.0f}), latActive {inn[:,9].mean()*100:.0f} %, hands-on {inn[:,10].mean()*100:.0f} %, laneDropout {inn[:,14].mean()*100:.1f} %, LC {inn[:,15].mean()*100:.1f} %")
ho=inn[(inn[:,9]==1)&(inn[:,10]==0)&(inn[:,15]==0)&(inn[:,5]>0.5)]
print(f"   hands-off good-lines frames {len(ho)}: |offset| median {np.median(np.abs(ho[:,3])):.3f} p90 {np.percentile(np.abs(ho[:,3]),90):.3f} m (signed mean {ho[:,3].mean():+.3f}) | lane width {np.median(ho[:,4]):.2f} m | |gap| median {np.median(np.abs(ho[:,6])):.2f} p90 {np.percentile(np.abs(ho[:,6]),90):.2f} deg | integ median {np.median(ho[:,7]):+.2f} e-3 | |k_des| median {np.median(np.abs(ho[:,8])):.2f} e-3 | driver tq p90 {np.percentile(inn[:,11],90):.0f}")
print("   per 200 m: s | v km/h | |off| med | off mean | lane_p min | |gap| med | integ | hands-on % | tq p90")
for lo in range(0, int(inn[:,1].max())+1, 200):
    b=inn[(inn[:,1]>=lo)&(inn[:,1]<lo+200)]
    if len(b)<5: continue
    print(f"   {lo:5d} | {np.median(b[:,2]):4.0f} | {np.median(np.abs(b[:,3])):.3f} | {b[:,3].mean():+.3f} | {b[:,5].min():.2f} | {np.median(np.abs(b[:,6])):.2f} | {np.median(b[:,7]):+.2f} | {b[:,10].mean()*100:3.0f} | {np.percentile(b[:,11],90):4.0f}")
