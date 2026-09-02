#!/usr/bin/env python3
"""Collection-readiness check from one or more qlogs (10 Hz). Reports:
 - build commit/branch (must be the wk2-fixes-release-mici tip)
 - NNLC on/off inference: torqueState.f vs analytic linear FF (desired lat accel / latAccelFactor)
 - liveTorqueParameters / liveDelay convergence
 - assist below minSteerSpeed (hold fix evidence) and grip stats while active
  python3 collect_check.py <qlog.zst ...>   (set OPENPILOT_DIR if not /home/user/openpilot)
"""
import os, sys, glob
import numpy as np
sys.path.insert(0, os.environ.get("OPENPILOT_DIR", "/home/user/openpilot"))
import zstandard
from cereal import log as capnp_log

files = [f for a in sys.argv[1:] for f in sorted(glob.glob(a))]
commit = branch = ""; laf = None
f_vals, lin_vals = [], []
ltp = None; ld = None; min_steer = None
active_v, grip = [], []
for fn in files:
    try:
        data = zstandard.ZstdDecompressor().decompress(open(fn, "rb").read(), max_output_size=2**30)
    except Exception as ex:
        print(f"skip {fn}: {ex}"); continue
    v = 0.0; st = 0.0
    for e in capnp_log.Event.read_multiple_bytes(data):
        w = e.which()
        if w == "initData":
            commit, branch = str(e.initData.gitCommit)[:9], str(e.initData.gitBranch)
        elif w == "carParams":
            laf = e.carParams.lateralTuning.torque.latAccelFactor; min_steer = e.carParams.minSteerSpeed
        elif w == "carState":
            v = e.carState.vEgo; st = e.carState.steeringTorque
        elif w == "liveTorqueParameters":
            p = e.liveTorqueParameters
            ltp = (p.latAccelFactorFiltered, p.frictionCoefficientFiltered, p.totalBucketPoints, p.useParams)
            if p.useParams and p.latAccelFactorFiltered > 0: laf = p.latAccelFactorFiltered
        elif w == "liveDelay":
            ld = (e.liveDelay.lateralDelay, str(e.liveDelay.status))
        elif w == "controlsState":
            lcs = e.controlsState.lateralControlState
            if lcs.which() != "torqueState": continue
            ts = lcs.torqueState
            if ts.active:
                active_v.append(v); grip.append(abs(st))
                if abs(ts.desiredLateralAccel) > 0.3:
                    f_vals.append(ts.f); lin_vals.append(ts.desiredLateralAccel)
if ltp and (laf is None) and ltp[0] > 0:
    laf = ltp[0]
print(f"build: {commit or '(no initData in these files - include segment 0)'} ({branch})")
if f_vals:
    f, l = np.array(f_vals), np.array(lin_vals) / (laf if laf else 1.95)
    k = float(np.dot(f, l) / max(np.dot(l, l), 1e-9)); r = float(np.corrcoef(f, l)[0, 1])
    verdict = "LINEAR (NNLC OFF)" if r > 0.98 and 0.8 < abs(k) < 1.25 else "NN (NNLC ON) or mismatch"
    print(f"feedforward vs linear: corr={r:.3f} gain={k:.2f} -> {verdict}  (n={len(f)})")
else:
    print("feedforward: no active frames with |dla|>0.3 -> cannot infer NNLC state")
print(f"liveTorque(filtered laf, friction, points, useParams): {ltp}")
print(f"liveDelay: {ld}")
if active_v:
    av = np.array(active_v); g = np.array(grip)
    below = (av < (min_steer or 17.5)).mean() * 100 if min_steer else float('nan')
    print(f"active frames={len(av)}  active below minSteerSpeed={below:.1f}%  (hold fix evidence)")
    print(f"grip while active |steeringTorque|: p50={np.percentile(g,50):.0f} p90={np.percentile(g,90):.0f}  >40: {(g>40).mean()*100:.1f}%  (target: p90 well under 40)")
