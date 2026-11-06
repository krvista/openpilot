"""E76: boot-time scan of a route's first segment: pandad / manager log lines about the panda (flash, firmware, relay,
safety, fault, harness), pandaStates evolution (safety model, faults, harness, ignition, controlsAllowed, rx-check
validity), onroadEvents, and the CAN traffic picture (messages per bus per second on the first seconds)."""
import glob
import json
import sys
from collections import Counter, defaultdict
from openpilot.tools.lib.logreader import LogReader
r = sys.argv[1]; seg = sys.argv[2] if len(sys.argv) > 2 else "0"
f = glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--{seg}--rlog.zst")[0]
t0 = None; lines = []; ps = []; evs = []; bus = defaultdict(Counter); last_ev = set()
for m in LogReader(f):
  t = m.logMonoTime * 1e-9; w = m.which()
  if t0 is None:
    t0 = t
  dt = t - t0
  if w in ("logMessage", "errorLogMessage"):
    txt = getattr(m, w)
    try:
      j = json.loads(txt); msg = str(j.get("msg", txt)); src = j.get("ctx", {}).get("daemon", "") if isinstance(j.get("ctx"), dict) else ""
    except Exception:  # noqa: BLE001
      msg, src = str(txt), ""
    low = msg.lower()
    if src != "manager" and any(k in low for k in ("panda", "flash", "firmware", "relay", "safety", "fault", "harness", "bootstub", "dfu", "mismatch", "ignition", "can ", "signature", "update")):
      lines.append((dt, src, msg[:220]))
  elif w == "pandaStates" and len(m.pandaStates):
    p = m.pandaStates[0]
    ps.append((dt, str(p.safetyModel), int(p.faultStatus.raw) if hasattr(p.faultStatus, "raw") else str(p.faultStatus), [str(x) for x in p.faults],
               str(p.harnessStatus), p.ignitionLine, p.ignitionCan, p.controlsAllowed, getattr(p, "controlsAllowedLateral", None),
               p.safetyRxChecksInvalid, p.safetyTxBlocked, p.rxBufferOverflow, p.txBufferOverflow, str(p.pandaType)))
  elif w == "onroadEvents":
    cur = {str(e.name) for e in m.onroadEvents}
    for nm in cur - last_ev:
      evs.append((dt, nm))
    last_ev = cur
  elif w == "can" and dt < 40:
    for c in m.can:
      bus[int(dt)][c.src] += 1
print(f"== {r} seg {seg}")
print("-- log lines:")
for x in lines[:60]:
  print(f"  {x[0]:7.2f} {x[1]:10} {x[2]}")
print("-- pandaStates changes:")
prev = None
for x in ps:
  key = x[1:]
  if key != prev:
    print(f"  {x[0]:7.2f} model {x[1]} faultStatus {x[2]} faults {x[3]} harness {x[4]} ignLine {x[5]} ignCan {x[6]} allowed {x[7]}/{x[8]} rxInvalid {x[9]} txBlocked {x[10]} ovf {x[11]}/{x[12]} type {x[13]}")
    prev = key
print("-- events:", " ".join(f"{t:.1f}:{n}" for t, n in evs[:40]))
print("-- CAN msgs per bus (src) per second, first 15 s:")
for s_ in range(0, 15):
  if s_ in bus:
    print(f"  t{s_:2d}: " + " ".join(f"{k}:{v}" for k, v in sorted(bus[s_].items())))
