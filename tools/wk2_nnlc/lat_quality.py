#!/usr/bin/env python3
"""Steering-quality summary from qlogs or rlogs (torqueState or pidState).

Per route and aggregate: build commit, controller type, lateral-active time, tracking
error |desired-actual| lateral accel (torqueState only), by |desired| band and speed band,
saturation, steeringPressed share and driver column-torque p50/p90 while active, output
sign-flip rate (dithering), live torque params / liveDelay readout, and the events of
interest (processNotRunning, controlsMismatch, commIssue cluster, steer faults).

  OPENPILOT_DIR=<checkout> python3 lat_quality.py <dir-or-files...> [--type auto|qlog|rlog]
  (set NNLC_CEREAL_DIR to use the patched nnlc_tools reader instead)
"""
import argparse, collections, glob, os, re, sys
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("paths", nargs="+"); ap.add_argument("--type", choices=["auto", "qlog", "rlog"], default="auto")
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
files = sorted(set(files))
kinds = {"rlog" if "rlog" in os.path.basename(f) else "qlog" for f in files}
use = a.type if a.type != "auto" else ("qlog" if "qlog" in kinds else "rlog")
files = [f for f in files if use in os.path.basename(f)]
if not files:
    sys.exit("no log files")
def route_seg(path):
    name = os.path.basename(os.path.dirname(path)) if os.path.basename(path).startswith(("rlog", "qlog")) else os.path.basename(path)
    m = re.search(r"([0-9a-f]+--[0-9a-f]+)--(\d+)", name); return (m.group(1), int(m.group(2))) if m else (name, 0)
by_route = collections.defaultdict(list)
for f in files:
    r, s = route_seg(f); by_route[r].append((s, f))
EV = ("processNotRunning", "controlsMismatch", "commIssue", "commIssueAvgFreq", "posenetInvalid", "locationdTemporaryError",
      "paramsdTemporaryError", "steerTempUnavailable", "steerTempUnavailableSilent", "steerUnavailable", "steerSaturated")

agg = dict(err=[], dla=[], v=[], grip=[], act_s=0.0, press_s=0.0, sat=0, n=0, flips=0, ev=collections.Counter())
print(f"reading {len(files)} {use} files, {len(by_route)} routes\n")
for route, segs in sorted(by_route.items()):
    segs.sort(); commit = "-"; ctrl = collections.Counter(); ev = collections.Counter()
    err, dla, vv, grip = [], [], [], []; act_s = press_s = 0.0; sat = n = flips = 0; prev_sign = 0; t_prev = None
    ltp_first = ltp_last = ld = None
    st = dict(v=0.0, pressed=False, st=0.0)
    for seg, fn in segs:
        try:
            for m in read_log(fn):
                w = m.which()
                if w == "initData" and seg == 0: commit = str(m.initData.gitCommit)[:9]
                elif w == "carState":
                    cs = m.carState; st.update(v=cs.vEgo, pressed=cs.steeringPressed, st=cs.steeringTorque)
                elif w == "onroadEvents":
                    for e in m.onroadEvents:
                        nme = str(e.name)
                        if nme in EV: ev[nme] += 1
                elif w == "liveTorqueParameters":
                    p = m.liveTorqueParameters
                    ltp_last = (round(p.latAccelFactorFiltered, 3), round(p.frictionCoefficientFiltered, 4), int(p.totalBucketPoints), bool(p.useParams))
                    ltp_first = ltp_first or ltp_last
                elif w == "liveDelay":
                    ld = (round(m.liveDelay.lateralDelay, 3), str(m.liveDelay.status))
                elif w == "controlsState":
                    lcs = m.controlsState.lateralControlState; k = lcs.which(); ctrl[k] += 1
                    if k not in ("torqueState", "pidState"): continue
                    s_ = getattr(lcs, k); t = m.logMonoTime / 1e9
                    dt = (t - t_prev) if t_prev is not None and 0 < t - t_prev < 1.0 else 0.0
                    t_prev = t
                    if not s_.active:
                        prev_sign = 0; continue
                    n += 1; act_s += dt
                    if st["pressed"]: press_s += dt
                    grip.append(abs(st["st"])); vv.append(st["v"])
                    if s_.saturated: sat += 1
                    if k == "torqueState":
                        err.append(abs(s_.desiredLateralAccel - s_.actualLateralAccel)); dla.append(abs(s_.desiredLateralAccel))
                    sg = 1 if s_.output > 0.02 else (-1 if s_.output < -0.02 else 0)
                    if sg and prev_sign and sg != prev_sign: flips += 1
                    prev_sign = sg
        except Exception as ex:
            print(f"  partial {os.path.basename(os.path.dirname(fn)) or os.path.basename(fn)}: {str(ex)[:50]}")
    ctrl_name = max(ctrl, key=ctrl.get) if ctrl else "-"
    print(f"### {route}  build={commit}  ctrl={ctrl_name}  active={act_s/60:.1f} min  events={dict(ev) or 'NONE'}")
    if n:
        g = np.array(grip)
        print(f"  pressed {press_s/max(act_s,1e-9)*100:.1f}% of active | driver torque p50={np.percentile(g,50):.0f} p90={np.percentile(g,90):.0f} | saturation {sat/n*100:.2f}% | sign flips {flips/max(act_s/60,1e-9):.1f}/min")
        if err:
            e = np.array(err); print(f"  |dla-ala| mean={e.mean():.3f} p50={np.percentile(e,50):.3f} p90={np.percentile(e,90):.3f} p99={np.percentile(e,99):.3f}")
    print(f"  liveTorque first={ltp_first} last={ltp_last} | liveDelay={ld}")
    agg["err"] += err; agg["dla"] += dla; agg["v"] += vv; agg["grip"] += grip; agg["act_s"] += act_s; agg["press_s"] += press_s
    agg["sat"] += sat; agg["n"] += n; agg["flips"] += flips; agg["ev"].update(ev)
print("\n=== AGGREGATE ===")
n = agg["n"]
if n:
    g = np.array(agg["grip"]); print(f"active {agg['act_s']/3600:.2f} h | pressed {agg['press_s']/max(agg['act_s'],1e-9)*100:.1f}% | driver torque p50={np.percentile(g,50):.0f} p90={np.percentile(g,90):.0f} | saturation {agg['sat']/n*100:.2f}% | sign flips {agg['flips']/max(agg['act_s']/60,1e-9):.1f}/min")
    if agg["err"]:
        E, D, V = map(np.array, (agg["err"], agg["dla"], agg["v"]))
        print(f"|dla-ala| mean={E.mean():.3f} p50={np.percentile(E,50):.3f} p90={np.percentile(E,90):.3f} p99={np.percentile(E,99):.3f}   (reference: v7 converged 0.075 / fresh install 0.132)")
        for lo, hi in [(0, .5), (.5, 1), (1, 1.5), (1.5, 9)]:
            mk = (D >= lo) & (D < hi)
            if mk.sum() > 20: print(f"  |dla| [{lo},{hi}): n={mk.sum():6d} err mean={E[mk].mean():.3f} p90={np.percentile(E[mk],90):.3f}")
        for lo, hi in [(14, 17.5), (17.5, 22.2), (22.2, 27.8), (27.8, 40)]:
            mk = (V >= lo) & (V < hi)
            if mk.sum() > 20: print(f"  v [{int(lo*3.6)}-{int(hi*3.6)} km/h): n={mk.sum():6d} err mean={E[mk].mean():.3f} p90={np.percentile(E[mk],90):.3f}")
print(f"events: {dict(agg['ev']) or 'NONE'}")
