"""Pipeline timing from rlogs: camera EOF -> modelV2 publish latency, modelExecutionTime, frame drops, plannerd solver time,
modelV2 -> controlsState publish gap, and per-process CPU (procLog) for the model/plan/control chain."""
import sys
import numpy as np
from collections import defaultdict
from openpilot.tools.lib.logreader import LogReader
files = sys.argv[1:]
lat, exe, drop, solv, gaps, cpu = [], [], [], [], [], defaultdict(list)
last_md = None; prev = {}
for f in files:
  for m in LogReader(f):
    w = m.which(); t = m.logMonoTime
    if w == "modelV2":
      md = m.modelV2
      if md.timestampEof: lat.append((t - md.timestampEof) * 1e-6)
      exe.append(md.modelExecutionTime * 1e3); drop.append(md.frameDropPerc); last_md = t
    elif w == "longitudinalPlan":
      solv.append(m.longitudinalPlan.solverExecutionTime * 1e3)
    elif w == "controlsState" and last_md is not None:
      gaps.append((t - last_md) * 1e-6)
    elif w == "procLog":
      for p in m.procLog.procs:
        name = " ".join(p.cmdline)[:80]
        key = next((k for k in ("modeld_v2", "selfdrive.modeld.modeld", "modeld", "plannerd", "controlsd", "card", "selfdrived", "radard", "locationd", "paramsd", "lagd", "torqued") if k in name), None)
        if key is None: continue
        tot = p.cpuUser + p.cpuSystem
        if (p.pid) in prev:
          dt = (t - prev[p.pid][1]) * 1e-9
          if dt > 0: cpu[key].append((tot - prev[p.pid][0]) / dt * 100 / 100)   # cpu times are in ticks (1/100 s)
        prev[p.pid] = (tot, t)
def s(a, name, unit):
  a = np.array(a)
  if len(a): print(f"  {name:38} n={len(a):6d} p50 {np.percentile(a,50):7.2f} p90 {np.percentile(a,90):7.2f} p99 {np.percentile(a,99):7.2f} max {a.max():7.2f} {unit}")
s(lat, "camera EOF -> modelV2 publish", "ms"); s(exe, "modelExecutionTime", "ms"); s(drop, "frameDropPerc", "%")
s(solv, "longitudinal solverExecutionTime", "ms"); s(gaps, "modelV2 -> next controlsState", "ms")
for k, v in sorted(cpu.items()): s([x * 100 for x in v], f"CPU {k}", "% of a core")
