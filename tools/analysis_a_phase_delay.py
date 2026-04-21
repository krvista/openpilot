#!/usr/bin/env python3
"""Analysis A: Phase delay — cross-correlation of desired vs actual angle.
Measures the actual time lag between op commanding an angle and EPS following.
If lag is consistently 50-100ms, a feedforward/lead compensation could help."""
import sys, os, glob, math
import numpy as np
sys.path.insert(0, '/home/user/openpilot')
sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG = "/home/user/openpilot/drivelog"
DT = 0.01

def get_all_segs():
    files = sorted(glob.glob(os.path.join(DRIVELOG, "*rlog.zst")))
    segs = []
    for f in files:
        segs.append(f)
    return segs

def extract_angles(path):
    des, act, v_arr = [], [], []
    last_v = 0.0
    for msg in LogReader(path):
        w = msg.which()
        if w == 'carState':
            last_v = msg.carState.vEgo
        elif w == 'controlsState':
            lat = msg.controlsState.lateralControlState
            if lat.which() == 'angleState':
                ang = lat.angleState
                if ang.active and last_v > 5.0:
                    des.append(float(ang.steeringAngleDesiredDeg))
                    act.append(float(msg.controlsState.steeringAngleDeg) if hasattr(msg.controlsState, 'steeringAngleDeg') else 0.0)
                    v_arr.append(last_v)
    # fallback: use carState steeringAngleDeg
    if not act or all(a == 0.0 for a in act):
        des2, act2, v2 = [], [], []
        last_ang = 0.0
        last_v = 0.0
        for msg in LogReader(path):
            w = msg.which()
            if w == 'carState':
                last_v = msg.carState.vEgo
                last_ang = msg.carState.steeringAngleDeg
            elif w == 'controlsState':
                lat = msg.controlsState.lateralControlState
                if lat.which() == 'angleState' and lat.angleState.active and last_v > 5.0:
                    des2.append(float(lat.angleState.steeringAngleDesiredDeg))
                    act2.append(last_ang)
                    v2.append(last_v)
        if des2:
            return np.array(des2), np.array(act2), np.array(v2)
    return np.array(des), np.array(act), np.array(v_arr)

def cross_correlate(des, act, max_lag_frames=30):
    """Compute normalized cross-correlation for lags 0..max_lag_frames.
    Returns array of correlations and best lag in frames."""
    n = len(des)
    if n < 100: return None, None
    des_z = des - des.mean()
    act_z = act - act.mean()
    norm = np.sqrt(np.sum(des_z**2) * np.sum(act_z**2))
    if norm < 1e-6: return None, None
    corrs = []
    for lag in range(max_lag_frames + 1):
        if lag == 0:
            c = np.sum(des_z * act_z) / norm
        else:
            c = np.sum(des_z[:-lag] * act_z[lag:]) / norm
        corrs.append(c)
    corrs = np.array(corrs)
    best_lag = int(np.argmax(corrs))
    return corrs, best_lag

segs = get_all_segs()
print(f"Phase Delay Analysis: {len(segs)} segments")

all_lags = []
speed_lags = {'low': [], 'mid': [], 'high': []}  # <30, 30-70, >70 km/h
done = 0

for path in segs:
    done += 1
    try:
        des, act, v = extract_angles(path)
    except Exception:
        continue
    if len(des) < 200: continue

    corrs, best_lag = cross_correlate(des, act)
    if corrs is None: continue
    all_lags.append(best_lag)

    # Speed-binned
    v_kmh = v * 3.6
    for label, lo, hi in [('low', 0, 30), ('mid', 30, 70), ('high', 70, 200)]:
        mask = (v_kmh >= lo) & (v_kmh < hi)
        if mask.sum() > 100:
            c, lag = cross_correlate(des[mask], act[mask])
            if c is not None:
                speed_lags[label].append(lag)

    if done % 50 == 0:
        print(f"  [{done}/{len(segs)}] processed")

print(f"\n{'='*60}")
print(f"PHASE DELAY RESULTS (100 Hz sampling → 1 frame = 10ms)")
print(f"{'='*60}")
if all_lags:
    lags = np.array(all_lags)
    print(f"\nOverall: N={len(lags)} segments")
    print(f"  Mean lag: {lags.mean():.1f} frames = {lags.mean()*10:.0f} ms")
    print(f"  Median lag: {np.median(lags):.0f} frames = {np.median(lags)*10:.0f} ms")
    print(f"  p25-p75: [{np.percentile(lags,25):.0f}, {np.percentile(lags,75):.0f}] frames")
    print(f"  Distribution: {dict(zip(*np.unique(lags, return_counts=True)))}")

for label in ['low', 'mid', 'high']:
    if speed_lags[label]:
        arr = np.array(speed_lags[label])
        print(f"\n  {label} speed: N={len(arr)}  mean={arr.mean():.1f} frames ({arr.mean()*10:.0f}ms)  median={np.median(arr):.0f}")
