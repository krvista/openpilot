"""Phase 47 first-boot check: uptime at logger start (cold vs warm), gpio udev helper processes in procLog (count, '{} +' vs '{} \;'),
core 0-5 load in the first 60 s of uptime, panda faults, tx rejects at boot, selfdriveInitializing duration."""
import glob, sys
import numpy as np
from openpilot.tools.lib.logreader import LogReader
for r in sys.argv[1:]:
  f = sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--0--rlog.zst"))
  if not f: continue
  first = None; gp = {}; prev = None; loads = []; faults = set(); procs_seen = 0
  for m in LogReader(f[0]):
    t = m.logMonoTime * 1e-9
    if first is None: first = t
    w = m.which()
    if w == "procLog":
      pl = m.procLog
      for p in pl.procs:
        cl = " ".join(p.cmdline)
        if "gpio" in cl and ("find" in cl or p.name in ("sh", "find", "chown", "chmod")):
          gp.setdefault(p.pid, (p.startTime, p.name, cl[:110]))
      ct = {c.cpuNum: (c.user + c.nice + c.system + c.irq + c.softirq, c.idle + c.iowait) for c in pl.cpuTimes}
      if prev is not None and t < 90:
        row = []
        for k in range(6):
          if k in ct and k in prev[1]:
            b = ct[k][0] - prev[1][k][0]; i = ct[k][1] - prev[1][k][1]; row.append(b / max(b + i, 1e-9))
        if row: loads.append((t, np.mean(row), max(row)))
      prev = (t, ct)
    elif w == "pandaStates":
      for ps in m.pandaStates:
        for fl in ps.faults: faults.add(str(fl))
  plus = sum(1 for v in gp.values() if "{} +" in v[2] or v[2].rstrip().endswith("+"))
  semi = sum(1 for v in gp.values() if ";" in v[2])
  L = np.array(loads) if loads else np.zeros((0, 3))
  b = L[(L[:, 0] >= 5) & (L[:, 0] <= 30)] if len(L) else L
  print(f"== {r}: logger start at uptime {first:.1f} s ({'COLD' if first < 40 else 'warm'}) | gpio helper procs {len(gp)} ('+' form {plus}, '\;' form {semi}) | "
        f"cores0-5 mean/peak 5-30 s {b[:,1].mean()*100 if len(b) else float('nan'):.0f}%/{b[:,2].max()*100 if len(b) else float('nan'):.0f}% | panda faults {sorted(faults) or 0}")
  for pid, (st, n, cl) in sorted(gp.items(), key=lambda x: x[1][0])[:4]:
    print(f"     pid {pid} start {st:.1f} {n}: {cl}")
