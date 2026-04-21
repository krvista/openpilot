#!/usr/bin/env python3
"""Analysis C: Curvature-rate feedforward potential.
If we add d(curvature)/dt as a feedforward term, does tracking improve?
This simulates adding a derivative kick to anticipate steering changes."""
import sys, os, glob, math
import numpy as np
sys.path.insert(0, '/home/user/openpilot')
sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG = "/home/user/openpilot/drivelog"

def get_all_segs():
    return sorted(glob.glob(os.path.join(DRIVELOG, "*rlog.zst")))

def extract_series(path):
    des, act, v_arr, t_arr = [], [], [], []
    last_v = 0.0
    last_ang = 0.0
    init_t = None
    for msg in LogReader(path):
        w = msg.which()
        if w == 'carState':
            last_v = msg.carState.vEgo
            last_ang = msg.carState.steeringAngleDeg
        elif w == 'controlsState':
            lat = msg.controlsState.lateralControlState
            if lat.which() == 'angleState':
                ang = lat.angleState
                if ang.active and last_v > 3.0:
                    ts = msg.logMonoTime * 1e-9
                    if init_t is None: init_t = ts
                    des.append(float(ang.steeringAngleDesiredDeg))
                    act.append(last_ang)
                    v_arr.append(last_v)
                    t_arr.append(ts - init_t)
    return np.array(des), np.array(act), np.array(v_arr), np.array(t_arr)

segs = get_all_segs()
print(f"Curvature-Rate Feedforward Analysis: {len(segs)} segments")

# For each segment: compute d(desired)/dt, simulate adding ff_gain * d(desired)/dt
# to the desired angle (with various gains and lead times), measure MAE

ff_gains = [0.0, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.15]
results = {g: [] for g in ff_gains}
done = 0

for path in segs:
    done += 1
    try:
        des, act, v, t = extract_series(path)
    except Exception:
        continue
    if len(des) < 200:
        continue

    # Compute derivative of desired angle (central difference)
    dt = np.diff(t)
    dt[dt < 1e-4] = 0.01
    d_des = np.zeros_like(des)
    d_des[1:-1] = (des[2:] - des[:-2]) / (t[2:] - t[:-2] + 1e-6)
    d_des[0] = d_des[1]
    d_des[-1] = d_des[-2]

    for g in ff_gains:
        # Simulate: desired_ff = desired + g * d_desired (feedforward kick)
        des_ff = des + g * d_des
        err_ff = np.abs(des_ff - act)
        results[g].append(err_ff.mean())

    if done % 50 == 0:
        print(f"  [{done}/{len(segs)}]")

print(f"\n{'='*60}")
print(f"CURVATURE-RATE FEEDFORWARD RESULTS")
print(f"{'='*60}")
print(f"\nFF gain sweep (desired_ff = desired + gain * d(desired)/dt):")
print(f"{'Gain':>6s} | {'MAE (°)':>8s} | {'Δ vs 0':>8s} | {'%':>6s}")
print(f"{'-'*6}-+-{'-'*8}-+-{'-'*8}-+-{'-'*6}")
baseline = np.mean(results[0.0]) if results[0.0] else 0
for g in ff_gains:
    if results[g]:
        mae = np.mean(results[g])
        delta = mae - baseline
        pct = 100 * delta / baseline if baseline > 0 else 0
        marker = " ←best" if g > 0 and delta == min(np.mean(results[gg]) - baseline for gg in ff_gains if gg > 0 and results[gg]) else ""
        print(f"{g:6.3f} | {mae:8.3f} | {delta:+8.3f} | {pct:+5.1f}%{marker}")

# Also test: shift desired by N frames forward (use future desired as current)
print(f"\n\nTime-shift analysis (use desired[t+shift] at time t):")
print(f"{'Shift':>8s} | {'MAE (°)':>8s} | {'Δ vs 0':>8s}")
print(f"{'-'*8}-+-{'-'*8}-+-{'-'*8}")
shift_results = {}
for path in segs[:100]:  # subset for speed
    try:
        des, act, v, t = extract_series(path)
    except Exception:
        continue
    if len(des) < 200: continue
    for shift in [0, 1, 2, 3, 4, 5, 7, 10]:
        if shift == 0:
            err = np.abs(des - act).mean()
        else:
            err = np.abs(des[shift:] - act[:-shift]).mean()
        shift_results.setdefault(shift, []).append(err)

base_shift = np.mean(shift_results.get(0, [0]))
for shift in [0, 1, 2, 3, 4, 5, 7, 10]:
    if shift in shift_results and shift_results[shift]:
        mae = np.mean(shift_results[shift])
        delta = mae - base_shift
        print(f"{shift*10:5d} ms | {mae:8.3f} | {delta:+8.3f}")
