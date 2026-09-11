import glob, re, sys
from openpilot.tools.lib.logreader import LogReader
r=sys.argv[1]; f=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--0--rlog.zst"))[0]
t0=None; last=None; msgs=[]; faults=set(); irqmax=0; cp=None
for m in LogReader(f):
    if t0 is None: t0=m.logMonoTime
    t=(m.logMonoTime-t0)/1e9; w=m.which()
    if w in ("logMessage","errorLogMessage"):
        s=str(getattr(m,w))
        mm=re.search(r'"msg": "([^"]{0,120})', s)
        if mm and re.search(r"cached CarParams|Getting VIN|Finished FW query|vin query|fingerprinted|Using cached", mm.group(1)): msgs.append((round(t,2), mm.group(1)))
    elif w=="pandaStates" and len(m.pandaStates):
        ps=m.pandaStates[0]; key=(str(ps.safetyModel), ps.safetyParam, ps.controlsAllowedLat if hasattr(ps,'controlsAllowedLat') else None)
        for x in ps.faults: faults.add(str(x))
        irqmax=max(irqmax, ps.canState1.irq0CallRate+ps.canState1.irq1CallRate)
        if key!=last: print(f"  {t:6.2f} panda safety={key[0]} param={key[1]}"); last=key
    elif w=="carParams" and cp is None:
        cp=m.carParams; print(f"  carParams: fingerprint={cp.carFingerprint} source={cp.fingerprintSource} vin={cp.carVin} fw={len(cp.carFw)}")
    if t>60: break
print("  log msgs:", msgs[:8]); print("  panda faults:", sorted(faults), "| bus1 irq/s max:", irqmax)
