"""Scan a route: build info, alert texts (selfdriveState), onroadEvents names (+SP), counts & first times."""
import glob, sys
from collections import Counter
from openpilot.tools.lib.logreader import LogReader
r = sys.argv[1]
files = sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
txt = Counter(); ev = Counter(); first = {}; t0 = None; prev_a = None; prev_e = set(); info = {}
for f in files:
  seg = int(f.split("--")[-2])
  try:
    for m in LogReader(f):
      w = m.which(); t = m.logMonoTime * 1e-9
      if t0 is None: t0 = t
      if w == "initData":
        i = m.initData; info = dict(commit=i.gitCommit[:8], branch=i.gitBranch, dirty=i.dirty)
      elif w == "selfdriveState":
        s = m.selfdriveState; a = (s.alertText1, s.alertText2, str(s.alertStatus))
        if a != prev_a and (a[0] or a[1]):
          txt[a] += 1; first.setdefault(a, (seg, round(t - t0)))
        prev_a = a
      elif w in ("onroadEvents", "onroadEventsSP"):
        evs = m.onroadEvents if w == "onroadEvents" else m.onroadEventsSP.events
        names = {str(e.name) for e in evs}
        for n in names - prev_e:
          ev[n] += 1
        prev_e = names if w == "onroadEvents" else prev_e
  except Exception as e:
    print("ERR", f, e)
print(r, info, "segs", len(files))
for a, c in txt.most_common(): print(f"  ALERT {c:4d} {a}  first seg/t {first[a]}")
print("  EVENTS", dict(ev.most_common()))
