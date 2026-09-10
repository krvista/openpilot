import glob, sys
from openpilot.tools.lib.logreader import LogReader
for r in sys.argv[1:]:
    f=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--0--rlog.zst"))
    if not f: continue
    first=None; done=False
    for m in LogReader(f[0]):
        if first is None: first=m.logMonoTime
        if m.which()=="procLog" and not done:
            rows=[]
            for p in m.procLog.procs:
                cl=" ".join(p.cmdline)[:60]
                if any(k in cl or k in p.name for k in ("launch","build.py","manager","scons","updated","hardwared","pandad","ui")):
                    rows.append((p.startTime, p.pid, p.name, cl))
            print(f"== {r}: logger start {first/1e9:.1f} s; procLog at {m.logMonoTime/1e9:.1f} s")
            for st,pid,n,cl in sorted(rows)[:14]: print(f"   start {st:7.1f} s  pid {pid:5d}  {n:14s} {cl}")
            done=True
        if done: break
