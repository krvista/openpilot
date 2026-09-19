#!/usr/bin/env python3
"""Lane-position bias from rlogs (modelV2 is not in qlogs).

For every modelV2 frame with both ego lane lines confident, take the ego lane center
at x=0: c = (y_left + y_right) / 2 in the model frame (y > 0 = left). c > 0 means the
lane center is to the LEFT of the car, i.e. the car sits RIGHT of center by c metres.
Reported separately for lateral-active and manual driving, straight roads only, plus
the calibration yaw, learned steering-angle offset and torqued lateral-accel offset.

  python3 lane_bias.py <dir-or-files> [--min-speed 15] [--max-curv 0.002]
Reader: nnlc_tools.logreader if NNLC_CEREAL_DIR is set, else openpilot cereal (OPENPILOT_DIR).
"""
import argparse, collections, glob, math, os, re, sys
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("paths", nargs="+")
ap.add_argument("--min-speed", type=float, default=15.0)
ap.add_argument("--max-curv", type=float, default=0.002, help="|desired curvature| limit for 'straight' (1/m)")
ap.add_argument("--min-prob", type=float, default=0.5)
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
    files += glob.glob(os.path.join(p, "**", "*rlog.zst"), recursive=True) if os.path.isdir(p) else glob.glob(p)
files = sorted(set(files))
if not files:
    sys.exit("no rlogs found (modelV2 needs rlogs)")

def route_of(path):
    name = os.path.basename(os.path.dirname(path)) if os.path.basename(path).startswith("rlog") else os.path.basename(path)
    m = re.search(r"([0-9a-f]+--[0-9a-f]+)--\d+", name)
    return m.group(1) if m else name

per_route = collections.defaultdict(lambda: {"act": [], "man": [], "width": [], "sign": [], "yaw": [], "aoff": [], "laoff": [], "commit": "-", "n": 0})
for fn in files:
    r = per_route[route_of(fn)]
    st = dict(v=0.0, active=False, curv=0.0)
    try:
        for m in read_log(fn):
            w = m.which()
            if w == "initData":
                r["commit"] = str(m.initData.gitCommit)[:9]
            elif w == "carState":
                st["v"] = m.carState.vEgo
            elif w == "controlsState":
                lcs = m.controlsState.lateralControlState; k = lcs.which()
                if k in ("torqueState", "pidState"):
                    st["active"] = bool(getattr(lcs, k).active)
                st["curv"] = m.controlsState.desiredCurvature
            elif w == "liveCalibration":
                if m.liveCalibration.rpyCalib:
                    r["yaw"].append(math.degrees(m.liveCalibration.rpyCalib[2]))
            elif w == "liveParameters":
                if m.liveParameters.valid:
                    r["aoff"].append(m.liveParameters.angleOffsetAverageDeg)
            elif w == "liveTorqueParameters":
                r["laoff"].append(m.liveTorqueParameters.latAccelOffsetFiltered)
            elif w == "modelV2":
                mv = m.modelV2
                if st["v"] < a.min_speed or abs(st["curv"]) > a.max_curv:
                    continue
                probs = list(mv.laneLineProbs)
                if len(probs) < 4 or len(mv.laneLines) < 4:
                    continue
                if probs[1] < a.min_prob or probs[2] < a.min_prob:
                    continue
                yl = mv.laneLines[1].y[0]; yr = mv.laneLines[2].y[0]
                width = abs(yl - yr)
                if not (2.6 <= width <= 4.6):
                    continue
                # Frame convention is detected from the data: if the right line has the larger y,
                # +y is RIGHT (observed on this build), else +y is LEFT. car_right = how far the
                # car sits to the right of the lane centre, positive = right.
                sign_right = 1.0 if yr > yl else -1.0
                c = (yl + yr) / 2.0                 # lane centre relative to the car, in model frame
                car_right = -c * sign_right
                r["width"].append(width); r["sign"].append(sign_right)
                (r["act"] if st["active"] else r["man"]).append(car_right)
                r["n"] += 1
    except Exception as ex:
        print(f"  partial {os.path.basename(os.path.dirname(fn)) or fn}: {str(ex)[:50]}")

def stats(x):
    x = np.array(x)
    return f"n={len(x):5d} mean={x.mean():+.3f} med={np.median(x):+.3f} p10={np.percentile(x,10):+.3f} p90={np.percentile(x,90):+.3f}" if len(x) else "n=    0"
print("car offset from ego-lane centre at x=0 [m], positive = car sits RIGHT of centre. Straight road, v>=%.0f m/s, both lane lines prob>=%.2f" % (a.min_speed, a.min_prob))
all_act, all_man = [], []
for route, r in sorted(per_route.items()):
    conv = ("+y=RIGHT" if np.mean(r['sign']) > 0 else "+y=LEFT") if r['sign'] else "?"
    print(f"\n### {route}  build={r['commit']}  frames={r['n']}  lane width mean={np.mean(r['width']) if r['width'] else float('nan'):.2f} m  model frame {conv}")
    print(f"  lateral ACTIVE : {stats(r['act'])}")
    print(f"  manual driving : {stats(r['man'])}")
    if r["yaw"]:  print(f"  calib yaw: mean={np.mean(r['yaw']):+.2f} deg (p10 {np.percentile(r['yaw'],10):+.2f}, p90 {np.percentile(r['yaw'],90):+.2f})")
    if r["aoff"]: print(f"  steering angleOffsetAverage: median={np.median(r['aoff']):+.2f} deg")
    if r["laoff"]: print(f"  torqued latAccelOffset: last={r['laoff'][-1]:+.3f} m/s^2")
    all_act += r["act"]; all_man += r["man"]
print("\n=== AGGREGATE ===")
print(f"ACTIVE : {stats(all_act)}")
print(f"MANUAL : {stats(all_man)}")
if all_act:
    m = float(np.mean(all_act))
    print(f"-> openpilot keeps the car {abs(m)*100:.0f} cm {'RIGHT' if m > 0 else 'LEFT'} of lane center on average"
          + (f" (driver, when steering manually: {abs(np.mean(all_man))*100:.0f} cm {'RIGHT' if np.mean(all_man) > 0 else 'LEFT'})" if all_man else ""))
    print(f"-> CameraOffset candidate (setting: + moves the model centre LEFT, so use +{abs(m):.2f} for a RIGHT bias): {m:+.2f} m  (apply half first, re-measure)")
