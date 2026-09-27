#!/usr/bin/env python3
"""Driver-release snap from rlogs (100 Hz controlsState): for every steeringPressed
True->False while lateral is active, the largest change of the controller's requested
torque within the next 300 ms, and the error term at release. Used to verify the
release blend-in (latcontrol_torque RELEASE_BLEND_SECONDS).

  OPENPILOT_DIR=<checkout> python3 release_snap.py <dir-or-files>
"""
import glob, os, re, sys, collections
import numpy as np
sys.path.insert(0, os.environ.get("OPENPILOT_DIR", "/home/user/openpilot"))
import zstandard
from cereal import log as capnp_log

files = []
for p in sys.argv[1:]:
    files += glob.glob(os.path.join(p, "**", "*rlog.zst"), recursive=True) if os.path.isdir(p) else glob.glob(p)
def route_of(path):
    name = os.path.basename(os.path.dirname(path)) if os.path.basename(path).startswith("rlog") else os.path.basename(path)
    m = re.search(r"([0-9a-f]+--[0-9a-f]+)--\d+", name); return m.group(1) if m else name
by_route = collections.defaultdict(list)
for f in sorted(set(files)): by_route[route_of(f)].append(f)

agg = []
print(f"{'route':24} {'build':10} {'releases':>8} {'step p50':>9} {'step p90':>9} {'max':>6}")
for route, fl in sorted(by_route.items()):
    commit = "-"; rows = []; pressed = False
    for fn in sorted(fl, key=lambda p: int(re.search(r"--(\d+)/", p + "/").group(1)) if re.search(r"--(\d+)/", p + "/") else 0):
        try:
            data = zstandard.ZstdDecompressor().decompress(open(fn, "rb").read(), max_output_size=2**31)
            for m in capnp_log.Event.read_multiple_bytes(data):
                w = m.which()
                if w == "initData": commit = str(m.initData.gitCommit)[:9]
                elif w == "carState": pressed = m.carState.steeringPressed
                elif w == "controlsState":
                    lcs = m.controlsState.lateralControlState; k = lcs.which()
                    if k not in ("torqueState", "pidState"): continue
                    s_ = getattr(lcs, k)
                    rows.append((m.logMonoTime / 1e9, bool(s_.active), float(s_.output), bool(pressed)))
        except Exception:
            pass
    steps = []
    for i in range(1, len(rows)):
        t0, a0, o0, p0 = rows[i - 1]; t1, a1, o1, p1 = rows[i]
        if a0 and a1 and p0 and not p1:
            fut = [abs(r[2] - o0) for r in rows[i:i + 40] if r[0] - t0 <= 0.35]
            if fut: steps.append(max(fut))
    if steps:
        st = np.array(steps); agg += steps
        print(f"{route:24} {commit:10} {len(st):8d} {np.percentile(st,50):9.3f} {np.percentile(st,90):9.3f} {st.max():6.2f}")
    else:
        print(f"{route:24} {commit:10} {0:8d}")
if agg:
    A = np.array(agg)
    print(f"\nAGGREGATE releases={len(A)} requested-torque step within 300 ms: p50={np.percentile(A,50):.3f} p90={np.percentile(A,90):.3f} (pre-blend reference: p50 0.157, p90 0.522)")
