#!/usr/bin/env python3
"""Context around steer faults in rlogs/qlogs: for every carState.steerFaultTemporary/Permanent
frame group and every steer*Unavailable event, print speed, lateral active, applied torque,
EPS motor torque, driver override, and the time since the last sub-engage-speed assist,
the last lateral deactivation and the last lateral (re)activation — to tell whether a fault
follows low-speed torque, a fast re-enable, or a driver override.

  python3 fault_context.py <dir> [--route 0000001d] [--min-steer 17.5] [--window 4]
"""
import argparse, collections, glob, os, re, sys
ap = argparse.ArgumentParser()
ap.add_argument("paths", nargs="+"); ap.add_argument("--route", action="append", default=[])
ap.add_argument("--min-steer", type=float, default=17.5); ap.add_argument("--window", type=float, default=4.0)
a = ap.parse_args()

def _reader():
    if os.environ.get("NNLC_CEREAL_DIR"):
        try:
            from nnlc_tools.logreader import LogReader
            return lambda fn: LogReader(fn)
        except Exception:
            pass
    sys.path.insert(0, os.environ.get("OPENPILOT_DIR", "/home/user/openpilot"))
    import zstandard
    from cereal import log as capnp_log
    def rd(fn):
        data = zstandard.ZstdDecompressor().decompress(open(fn, "rb").read(), max_output_size=2**31)
        return capnp_log.Event.read_multiple_bytes(data)
    return rd
read_log = _reader()

files = []
for p in a.paths:
    files += glob.glob(os.path.join(p, "**", "*log.zst"), recursive=True) if os.path.isdir(p) else glob.glob(p)
def rs(path):
    name = os.path.basename(os.path.dirname(path)) if os.path.basename(path).startswith(("rlog", "qlog")) else os.path.basename(path)
    m = re.search(r"([0-9a-f]+--[0-9a-f]+)--(\d+)", name); return (m.group(1), int(m.group(2))) if m else (name, 0)
by_route = collections.defaultdict(list)
for f in sorted(set(files)):
    r, s = rs(f)
    if a.route and not any(x in r for x in a.route): continue
    by_route[r].append((s, f))
FAULT_EV = ("steerTempUnavailable", "steerTempUnavailableSilent", "steerUnavailable", "steerTimeLimit")
ms = a.min_steer

for route, segs in sorted(by_route.items()):
    segs.sort(); rows = []; events = []; t0 = None
    st = dict(v=0.0, eps=0.0, applied=float("nan"), pressed=False, fT=False, fP=False, active=False, out=0.0)
    last_hold = last_off = last_on = None; prev_active = False
    for seg, fn in segs:
        try:
            for m in read_log(fn):
                w = m.which(); t = m.logMonoTime / 1e9
                if t0 is None: t0 = t
                if w == "carState":
                    cs = m.carState; st.update(v=cs.vEgo, eps=cs.steeringTorqueEps, pressed=cs.steeringPressed, fT=cs.steerFaultTemporary, fP=cs.steerFaultPermanent)
                elif w == "carOutput":
                    st["applied"] = m.carOutput.actuatorsOutput.torque
                elif w == "onroadEvents":
                    for e in m.onroadEvents:
                        if str(e.name) in FAULT_EV: events.append((t, str(e.name)))
                elif w == "controlsState":
                    lcs = m.controlsState.lateralControlState; k = lcs.which()
                    if k not in ("torqueState", "pidState"): continue
                    s_ = getattr(lcs, k); st["active"] = bool(s_.active); st["out"] = float(s_.output)
                    if st["active"] and st["v"] < ms: last_hold = t
                    if prev_active and not st["active"]: last_off = t
                    if st["active"] and not prev_active: last_on = t
                    prev_active = st["active"]
                    rows.append((t, st["v"], st["active"], st["out"], st["applied"], st["eps"], st["pressed"], st["fT"], st["fP"], last_hold, last_off, last_on))
        except Exception as ex:
            print(f"  partial {os.path.basename(os.path.dirname(fn))}: {str(ex)[:50]}")
    if not rows: continue
    fault_ts = [r[0] for r in rows if r[7] or r[8]]
    marks = sorted({round(t, 1) for t in fault_ts} | {round(t, 1) for t, _ in events})
    # merge marks closer than 2 s into one incident
    incidents = []
    for t in marks:
        if incidents and t - incidents[-1][-1] < 2.0: incidents[-1].append(t)
        else: incidents.append([t])
    print(f"\n### {route}: fault frames={len(fault_ts)}  events={[(round(t - t0, 1), n) for t, n in events]}  incidents={len(incidents)}")
    for inc in incidents:
        tf = inc[0]
        print(f"--- incident at t={tf - t0:.1f}s (route-relative)  [{'/'.join(sorted({n for t, n in events if abs(t - tf) < 2}))}]")
        print(f"{'t-rel':>7} {'km/h':>5} {'act':>3} {'out':>6} {'applied':>7} {'eps':>6} {'prs':>3} {'fT':>2} | since hold / off / on (s)")
        last_shown = -1e9
        for r in rows:
            if tf - a.window <= r[0] <= tf + a.window and (r[0] - last_shown >= 0.5 or r[7] or r[8]):
                last_shown = r[0]
                sh = f"{r[0]-r[9]:5.1f}" if r[9] else "  -  "; so = f"{r[0]-r[10]:5.1f}" if r[10] else "  -  "; sn = f"{r[0]-r[11]:5.1f}" if r[11] else "  -  "
                print(f"{r[0]-tf:+7.1f} {r[1]*3.6:5.0f} {int(r[2]):3d} {r[3]:6.2f} {r[4]:7.2f} {r[5]:6.0f} {int(r[6]):3d} {int(r[7]):2d} | {sh} / {so} / {sn}")
