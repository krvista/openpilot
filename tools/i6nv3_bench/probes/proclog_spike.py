import glob, sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
r=sys.argv[1]; segs=[int(x) for x in sys.argv[2].split(",")]
ts=[]; v=[]; core2=[]; pss_n=[]
for s in segs:
    f=glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--{s}--rlog.zst")[0]
    lr=list(LogReader(f)); pl=[m for m in lr if m.which()=="procLog"]
    for a,b in zip(pl[:-1],pl[1:]):
        pa={p.pid:(p.cpuUser+p.cpuSystem) for p in a.procLog.procs}
        for p in b.procLog.procs:
            if "proclogd" in " ".join(p.cmdline) and p.pid in pa:
                dt=(b.logMonoTime-a.logMonoTime)/1e9; ts.append(b.logMonoTime/1e9); v.append(100*(p.cpuUser+p.cpuSystem-pa[p.pid])/dt)
                pss_n.append(sum(1 for q in b.procLog.procs if q.memPss>0))
    for m in lr:
        if m.which()=="deviceState": core2.append((m.logMonoTime/1e9, m.deviceState.cpuUsagePercent[2]))
v=np.array(v); ts=np.array(ts)
spk=ts[v>30]
print(f"{r} segs {segs}: proclogd windows {len(v)}, >30% {int((v>30).sum())}, >15% {int((v>15).sum())}; spike intervals (s): {np.round(np.diff(spk),1)[:12]}")
print(f"   value pct: p50 {np.percentile(v,50):.1f} p90 {np.percentile(v,90):.1f} max {v.max():.1f}; procs with PSS>0 per msg: {min(pss_n)}..{max(pss_n)}; total procs per msg ~{len(pl[-1].procLog.procs)}")
c2=np.array(core2); hi=c2[c2[:,1]>=95][:,0]
print(f"   core2>=95% samples {len(hi)}; nearest proclogd spike distance (s) for first 8: {[round(float(np.min(np.abs(spk-t))),1) for t in hi[:8]] if len(spk) else 'n/a'}")
