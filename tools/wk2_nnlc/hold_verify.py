#!/usr/bin/env python3
"""Verify the min-steer-speed hold fix (controlsd hysteresis + chrysler wind-down)
from rlogs or qlogs. Controller-agnostic: works on pidState and torqueState logs.

Reports per route and in aggregate:
  - build commit (segment-0 initData)
  - lateral-active frames by speed band: >=min, hold window [min-2.5, min), ramp band
    [min-3.0, min-2.5) (LKAS bit still up, latActive already off), < min-3.0
  - EPS acceptance below the engage speed: EPS motor torque response to the applied
    command in the hold window and in the ramp band vs. the >=min reference band
  - deactivation cuts (active->inactive with |controller output| > 0.10) by speed band,
    split driver-override vs. not
  - wind-down: applied torque samples until |torque| < 0.02 after a non-override cut
  - steer faults: carState steerFaultTemporary/Permanent frames and steer*Unavailable events,
    and whether any occur within 10 s after time spent below the engage speed

  python3 hold_verify.py <dir-or-files...> [--type auto|rlog|qlog] [--min-steer 17.5]
Reader: nnlc_tools.logreader if importable (set NNLC_CEREAL_DIR), else openpilot cereal
(set OPENPILOT_DIR, default /home/user/openpilot).
"""
import argparse, collections, glob, os, re, sys
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("paths", nargs="+")
ap.add_argument("--type", choices=["auto", "rlog", "qlog"], default="auto")
ap.add_argument("--min-steer", type=float, default=None, help="override minSteerSpeed (m/s); default from carParams or 17.5")
a = ap.parse_args()

# ---- reader ---------------------------------------------------------------
def _reader():
    # The bundled nnlc_tools schema does not match sunnypilot logs (union members differ),
    # so only use its LogReader when NNLC_CEREAL_DIR points at the fork's cereal.
    if os.environ.get("NNLC_CEREAL_DIR"):
        try:
            from nnlc_tools.logreader import LogReader
            return lambda fn: LogReader(fn)
        except Exception:
            pass
    if True:
        sys.path.insert(0, os.environ.get("OPENPILOT_DIR", "/home/user/openpilot"))
        import zstandard
        from cereal import log as capnp_log
        def rd(fn):
            data = zstandard.ZstdDecompressor().decompress(open(fn, "rb").read(), max_output_size=2**31)
            return capnp_log.Event.read_multiple_bytes(data)
        return rd
read_log = _reader()

# ---- discover files -------------------------------------------------------
files = []
for p in a.paths:
    if os.path.isdir(p):
        for pat in ("**/rlog.zst", "**/qlog.zst", "**/*--rlog.zst", "**/*--qlog.zst"):
            files += glob.glob(os.path.join(p, pat), recursive=True)
    else:
        files += glob.glob(p)
files = sorted(set(files))
kinds = {"rlog" if "rlog" in os.path.basename(f) else "qlog" for f in files}
use = a.type if a.type != "auto" else ("rlog" if "rlog" in kinds else "qlog")
files = [f for f in files if use in os.path.basename(f)]
if not files:
    sys.exit("no log files found")

def route_seg(path):
    name = os.path.basename(os.path.dirname(path)) if os.path.basename(path).startswith(("rlog", "qlog")) else os.path.basename(path)
    m = re.search(r"([0-9a-f]+--[0-9a-f]+)--(\d+)", name)
    return (m.group(1), int(m.group(2))) if m else (name, 0)

by_route = collections.defaultdict(list)
for f in files:
    r, s = route_seg(f)
    by_route[r].append((s, f))

STEER_FAULT_EVENTS = ("steerTempUnavailable", "steerTempUnavailableSilent", "steerUnavailable", "steerTimeLimit")

# ---- per-route scan -------------------------------------------------------
agg = collections.Counter()
agg_pairs = {"ref": [], "hold": [], "ramp": []}   # (applied, eps)
agg_cuts = []                                      # (v, pressed, out, samples_to_zero)
agg_faults = collections.Counter()
print(f"reading {len(files)} {use} files, {len(by_route)} routes\n")
hdr = f"# cuts@min / cuts@hold = non-override deactivation cuts at the engage speed / at the hold floor\n{'route':26} {'commit':9} {'ctrl':11} {'active':>7} {'>=min':>6} {'hold':>6} {'ramp':>6} {'<ramp':>6}  {'cuts@min':>8} {'cuts@hold':>9}  faults"
print(hdr)
for route, segs in sorted(by_route.items()):
    segs.sort()
    commit = "-"; min_steer = a.min_steer
    frames = []          # (t, active, out, v, eps, applied, pressed, faultT, faultP)
    fault_events = 0
    ctrl_types = collections.Counter()
    st = dict(v=0.0, eps=0.0, applied=None, pressed=False, fT=False, fP=False, hold_seen_t=-1e9)
    fault_after_hold = 0
    for seg, fn in segs:
        try:
            msgs = read_log(fn)
        except Exception as ex:
            print(f"  skip {os.path.basename(fn)}: {str(ex)[:60]}"); continue
        try:
            for m in msgs:
                w = m.which()
                if w == "initData" and seg == 0:
                    commit = str(m.initData.gitCommit)[:9]
                elif w == "carParams" and a.min_steer is None:
                    min_steer = float(m.carParams.minSteerSpeed) or 17.5
                elif w == "carState":
                    cs = m.carState
                    st["v"] = cs.vEgo; st["eps"] = cs.steeringTorqueEps; st["pressed"] = cs.steeringPressed
                    st["fT"] = cs.steerFaultTemporary; st["fP"] = cs.steerFaultPermanent
                elif w == "carOutput":
                    st["applied"] = m.carOutput.actuatorsOutput.torque
                elif w == "onroadEvents":
                    t = m.logMonoTime / 1e9
                    for e in m.onroadEvents:
                        if str(e.name) in STEER_FAULT_EVENTS:
                            fault_events += 1
                            if t - st["hold_seen_t"] < 10.0:
                                fault_after_hold += 1
                elif w == "controlsState":
                    lcs = m.controlsState.lateralControlState
                    kind = lcs.which()
                    ctrl_types[kind] += 1
                    if kind not in ("torqueState", "pidState"):
                        continue
                    s_ = getattr(lcs, kind)
                    t = m.logMonoTime / 1e9
                    ms = min_steer or 17.5
                    if s_.active and st["v"] < ms:
                        st["hold_seen_t"] = t
                    frames.append((t, bool(s_.active), float(s_.output), st["v"], st["eps"],
                                   st["applied"] if st["applied"] is not None else float("nan"),
                                   bool(st["pressed"]), bool(st["fT"]), bool(st["fP"])))
        except Exception as ex:
            # truncated segment (ignition off mid-write): keep what was parsed, move on
            print(f"  partial {os.path.basename(os.path.dirname(fn)) or os.path.basename(fn)}: {str(ex)[:60]}")
            continue
    ms = min_steer or 17.5
    hold_lo, ramp_lo = ms - 2.5, ms - 3.0
    if not frames:
        print(f"{route:26} {commit:9} {'no frames':11}"); continue
    F = np.array([(f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8]) for f in frames], dtype=float)
    act, out, v, eps, applied, pressed, fT, fP = F.T
    act = act.astype(bool); pressed = pressed.astype(bool)
    n_act = int(act.sum())
    b_ref = act & (v >= ms); b_hold = act & (v >= hold_lo) & (v < ms); b_ramp = act & (v >= ramp_lo) & (v < hold_lo); b_low = act & (v < ramp_lo)
    # EPS response pairs (applied command vs EPS motor torque)
    ok_ap = ~np.isnan(applied)
    for key, mask in (("ref", b_ref), ("hold", b_hold)):
        sel = mask & ok_ap & (np.abs(applied) > 0.05)
        agg_pairs[key] += list(zip(applied[sel], eps[sel]))
    ramp_sel = (~act) & ok_ap & (np.abs(applied) > 0.05) & (v >= ramp_lo) & (v < hold_lo)
    agg_pairs["ramp"] += list(zip(applied[ramp_sel], eps[ramp_sel]))
    # cuts + wind-down
    cuts_min = cuts_hold = 0
    for i in range(1, len(frames)):
        if act[i-1] and not act[i] and abs(out[i-1]) > 0.10:
            t_zero = 0.0
            for j in range(i, len(frames)):
                if frames[j][0] - frames[i][0] > 3.0 or np.isnan(applied[j]) or abs(applied[j]) < 0.02:
                    t_zero = frames[j][0] - frames[i-1][0]
                    break
            agg_cuts.append((v[i-1], bool(pressed[i]), abs(out[i-1]), t_zero))
            if abs(v[i-1] - ms) < 0.6 and not pressed[i]: cuts_min += 1
            if abs(v[i-1] - hold_lo) < 0.6 and not pressed[i]: cuts_hold += 1
    nf = int(fT.sum()); npf = int(fP.sum())
    agg.update(dict(active=n_act, ref=int(b_ref.sum()), hold=int(b_hold.sum()), ramp=int(b_ramp.sum()), low=int(b_low.sum()),
                    faultT=nf, faultP=npf, fault_events=fault_events, fault_after_hold=fault_after_hold, cuts_min=cuts_min, cuts_hold=cuts_hold))
    ctrl = max(ctrl_types, key=ctrl_types.get) if ctrl_types else "-"
    print(f"{route:26} {commit:9} {ctrl:11} {n_act:7d} {int(b_ref.sum()):6d} {int(b_hold.sum()):6d} {int(b_ramp.sum()):6d} {int(b_low.sum()):6d}  "
          f"{cuts_min:8d} {cuts_hold:9d}  T={nf} P={npf} ev={fault_events}(after-hold {fault_after_hold})")

# ---- aggregate + verdicts -------------------------------------------------
ms = a.min_steer or 17.5
print("\n=== AGGREGATE ===")
A = agg
print(f"active frames: {A['active']}  |  >= {ms*3.6:.0f}km/h: {A['ref']}  hold[{(ms-2.5)*3.6:.0f},{ms*3.6:.0f}): {A['hold']}  ramp-band: {A['ramp']}  below: {A['low']}")
def resp(key):
    P = np.array(agg_pairs[key])
    if len(P) < 30:
        return None
    ap_, ep_ = P[:, 0], P[:, 1]
    corr = abs(float(np.corrcoef(ap_, ep_)[0, 1])) if ap_.std() > 0 and ep_.std() > 0 else float("nan")
    ratio = float(np.median(np.abs(ep_)) / max(np.median(np.abs(ap_)), 1e-6))
    return len(P), corr, ratio
R = {k: resp(k) for k in ("ref", "hold", "ramp")}
for k, lab in (("ref", f"reference >= {ms*3.6:.0f}km/h"), ("hold", "hold window (latActive on)"), ("ramp", "ramp band (latActive off, LKAS bit up)")):
    print(f"EPS response {lab:40}: " + (f"n={R[k][0]} |corr(applied,eps)|={R[k][1]:.2f} median|eps|/|applied|={R[k][2]:.1f}" if R[k] else "n<30 (no data)"))
C = agg_cuts
if C:
    c = np.array([(x[0], x[1], x[2], x[3]) for x in C], dtype=float)
    np_ = c[:, 1] == 0
    at_min = np.abs(c[:, 0] - ms) < 0.6
    at_hold = np.abs(c[:, 0] - (ms - 2.5)) < 0.6
    print(f"cuts |out|>0.10: {len(c)}  not-pressed: {int(np_.sum())}  at {ms*3.6:.0f}km/h not-pressed: {int((at_min & np_).sum())}  "
          f"at hold floor ({(ms-2.5)*3.6:.0f}km/h) not-pressed: {int((at_hold & np_).sum())}")
    wd = c[np_ & (c[:, 2] > 0.2), 3]
    if len(wd):
        print(f"wind-down after not-pressed cuts with |out|>0.2: time until applied torque ~0: median={np.median(wd):.2f}s p90={np.percentile(wd,90):.2f}s "
              f"(<=0.1s = one-step cut; STEER_DELTA_DOWN ramp from |out| 0.3 takes ~0.5s, from saturation ~1.3s)")
print(f"steer faults: carState temp={A['faultT']} perm={A['faultP']} frames; steer*Unavailable events={A['fault_events']} (within 10s after sub-{ms*3.6:.0f} assist: {A['fault_after_hold']})")

print("\n=== VERDICT ===")
hold_on = A["hold"] >= max(100, 0.01 * max(A["active"], 1))
print(f"[{'PASS' if hold_on else 'FAIL'}] hold active below {ms*3.6:.0f}km/h: {A['hold']} frames ({A['hold']/max(A['active'],1)*100:.1f}% of active)")
if R["ref"] and R["hold"]:
    ok = R["hold"][1] > 0.5 and R["hold"][2] > 0.5 * R["ref"][2]
    print(f"[{'PASS' if ok else 'FAIL'}] EPS accepts torque in hold window: corr {R['hold'][1]:.2f} vs ref {R['ref'][1]:.2f}, gain ratio {R['hold'][2]:.1f} vs ref {R['ref'][2]:.1f}")
else:
    print("[N/A ] EPS acceptance in hold window: insufficient data")
if R["ramp"]:
    ok = R["ramp"][1] > 0.5 and R["ref"] and R["ramp"][2] > 0.5 * R["ref"][2]
    print(f"[{'PASS' if ok else 'WEAK'}] EPS still responds in ramp band [{(ms-3.0)*3.6:.0f},{(ms-2.5)*3.6:.0f})km/h: corr {R['ramp'][1]:.2f} gain {R['ramp'][2]:.1f} -> {'52 km/h extension has hardware support' if ok else 'no evidence for extending below the hold floor'}")
else:
    print("[N/A ] ramp-band EPS response: no data (needs non-override deactivations while decelerating)")
print(f"[{'PASS' if A['cuts_min']==0 else 'FAIL'}] non-override cuts at {ms*3.6:.0f}km/h: {A['cuts_min']}")
print(f"[{'PASS' if A['fault_events']==0 and A['faultP']==0 else 'CHECK'}] steer faults: events={A['fault_events']} permanent-frames={A['faultP']}")
