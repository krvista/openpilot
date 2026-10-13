"""Around a time window: controlsd loop gaps (controlsState interval), card gaps (carState interval), onroadEvents, procLog per-process
CPU deltas (top), deviceState cpu/mem/thermal, selfdriveState alert. Usage: lagprobe.py <route> <seg_lo> <seg_hi> <t_lo> <t_hi>"""
import glob, sys
import numpy as np
from collections import defaultdict
from openpilot.tools.lib.logreader import LogReader
r, slo, shi, tlo, thi = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])
f0 = sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--0--rlog.zst"))[0]
t0 = next(iter(LogReader(f0))).logMonoTime * 1e-9
files = [f for f in sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2])) if slo <= int(f.split("--")[-2]) <= shi]
last = {}; gaps = defaultdict(list); prev_proc = {}; ev_prev = set()
for f in files:
  for m in LogReader(f):
    t = m.logMonoTime * 1e-9 - t0; w = m.which()
    if w in ("controlsState", "carState", "modelV2", "carControl", "sendcan"):
      if w in last and tlo <= t <= thi: gaps[w].append((t, t - last[w]))
      last[w] = t
    if not (tlo <= t <= thi): continue
    if w == "onroadEvents":
      cur = {str(e.name) for e in m.onroadEvents}
      new = cur - ev_prev
      if new - {"steerOverride", "gasPressedOverride"}: print(f"  {t:8.2f} events +{sorted(new)}")
      ev_prev = cur
    elif w == "selfdriveState":
      s = m.selfdriveState
      if s.alertText1 and "Lag" in s.alertText1 + s.alertText2: pass
    elif w == "deviceState":
      d = m.deviceState
      print(f"  {t:8.2f} deviceState cpu% {list(d.cpuUsagePercent)} mem {d.memoryUsagePercent}% thermal {d.thermalStatus} cpuTemp {max(d.cpuTempC) if len(d.cpuTempC) else 0:.0f}")
    elif w == "procLog":
      cur = {}
      for p in m.procLog.procs:
        cur[p.pid] = (p.cpuUser + p.cpuSystem, " ".join(p.cmdline)[:50] or p.name)
      if prev_proc:
        dt = t - prev_t
        rows = sorted(((cur[pid][0] - prev_proc[pid][0]) / dt, cur[pid][1]) for pid in cur if pid in prev_proc)
        print(f"  {t:8.2f} procLog top cpu (cores): " + ", ".join(f"{n}={c:.2f}" for c, n in rows[::-1][:7] if c > 0.05))
      prev_proc = cur; prev_t = t
    elif w == "logMessage" and ("lag" in m.logMessage.lower() or "drop" in m.logMessage.lower() or "slow" in m.logMessage.lower()):
      print(f"  {t:8.2f} log: {m.logMessage[:200]}")
for w, g in gaps.items():
  G = np.array(g)
  big = G[G[:,1] > 0.03]
  print(f"  {w:14} intervals n {len(G)} p99 {np.percentile(G[:,1],99)*1e3:.0f} ms max {G[:,1].max()*1e3:.0f} ms; >30 ms at: " + ", ".join(f"{t:.2f}s({d*1e3:.0f})" for t, d in big[:12]))
