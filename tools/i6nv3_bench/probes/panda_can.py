import glob, sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
r=sys.argv[1]; files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))
first_fault=None; out=[]
for f in files:
    seg=int(f.split("--")[-2]); irq=[[],[],[]]; rx=[[],[],[]]; lost=[0,0,0]; load=[]; faults=set(); fs=set(); prev=None
    for m in LogReader(f):
        if m.which()!="pandaStates" or len(m.pandaStates)==0: continue
        ps=m.pandaStates[0]; t=m.logMonoTime/1e9
        load.append(ps.interruptLoad); fs.add(str(ps.faultStatus))
        for x in ps.faults: faults.add(str(x))
        cs=[ps.canState0, ps.canState1, ps.canState2]
        for i,c in enumerate(cs):
            irq[i].append(c.irq0CallRate+c.irq1CallRate+c.irq2CallRate); lost[i]=max(lost[i], c.totalRxLostCnt)
            if prev is not None and t>prev[0]: rx[i].append((c.totalRxCnt-prev[1][i])/(t-prev[0]))
        prev=(t,[c.totalRxCnt for c in cs])
        if faults and first_fault is None: first_fault=(seg, round(t - m.logMonoTime/1e9 + t,1))
    if not load: continue
    out.append(f"seg {seg:2d}: irqLoad mean {np.mean(load):.2f} max {np.max(load):.2f} | irq/s bus0 p50/max {np.median(irq[0]):.0f}/{np.max(irq[0]):.0f} bus1 {np.median(irq[1]):.0f}/{np.max(irq[1]):.0f} bus2 {np.median(irq[2]):.0f}/{np.max(irq[2]):.0f} | rx/s bus0/1/2 {np.median(rx[0]) if rx[0] else 0:.0f}/{np.median(rx[1]) if rx[1] else 0:.0f}/{np.median(rx[2]) if rx[2] else 0:.0f} | rxLost {lost} | faultStatus {sorted(fs)} faults {sorted(faults)}")
print(f"== {r}: first fault at {first_fault}")
print("\n".join(out))
