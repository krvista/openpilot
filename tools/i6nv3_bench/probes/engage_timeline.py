import glob, sys
from openpilot.tools.lib.logreader import LogReader
f=glob.glob("/home/user/drivelog/drivelog/*_0000000b--*--0--rlog.zst")[0]
t0=None; rows=[]
for m in LogReader(f):
    if t0 is None: t0=m.logMonoTime
    t=(m.logMonoTime-t0)/1e9
    if t<8 or t>30: continue
    w=m.which()
    if w=="pandaStates":
        ps=m.pandaStates
        rows.append((t,"panda"," ".join(f"[{p.safetyModel} prm={p.safetyParam} ca={int(p.controlsAllowed)} caLat={int(p.controlsAllowedLateral)} ign={int(p.ignitionLine or p.ignitionCan)} hb={int(p.heartbeatLost)}]" for p in ps)))
    elif w=="selfdriveState":
        s=m.selfdriveState; rows.append((t,"sds",f"en={int(s.enabled)} act={int(s.active)} st={s.state} alert='{s.alertText1[:24]}'"))
    elif w=="selfdriveStateSP":
        s=m.selfdriveStateSP; rows.append((t,"sdsSP",f"mads en={int(s.mads.enabled)} act={int(s.mads.active)} avail={int(s.mads.available)} st={s.mads.state}"))
    elif w=="carControl":
        c=m.carControl; rows.append((t,"cc",f"en={int(c.enabled)} lat={int(c.latActive)} long={int(c.longActive)}"))
    elif w=="carState":
        c=m.carState; rows.append((t,"cs",f"v={c.vEgo*3.6:.0f} press={int(c.steeringPressed)} cruise={int(c.cruiseState.enabled)} avail={int(c.cruiseState.available)} btn={[str(b.type) for b in c.buttonEvents]}"))
    elif w=="carStateSP":
        c=m.carStateSP; rows.append((t,"csSP",f"{[a for a in dir(c) if not a.startswith('_')][:0]} madsEnabled={getattr(c,'madsEnabled','?')} btn={[str(b.type) for b in c.buttonEvents] if hasattr(c,'buttonEvents') else '?'}"))
    elif w=="onroadEvents":
        rows.append((t,"ev",str([str(e.name) for e in m.onroadEvents])))
    elif w=="onroadEventsSP":
        rows.append((t,"evSP",str([str(e.name) for e in m.onroadEventsSP.events]) if hasattr(m.onroadEventsSP,"events") else str(m.onroadEventsSP)[:80]))
# de-duplicate consecutive identical lines per kind
last={}; out=[]
for t,k,v in sorted(rows):
    if last.get(k)!=v or k in("ev","evSP"): out.append(f"{t:6.2f} {k:6s} {v}"); last[k]=v
print("\n".join(out[:260]))
