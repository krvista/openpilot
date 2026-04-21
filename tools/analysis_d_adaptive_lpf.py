#!/usr/bin/env python3
"""Analysis D: Adaptive LPF tau — find optimal tau per speed/curvature regime.
Current Phase 6 uses fixed tau=0.20s. Maybe faster tau in curves (less lag)
and slower tau on straights (more smoothing) would be better."""
import sys, os, glob, math
import numpy as np
sys.path.insert(0, '/home/user/openpilot')
sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
from openpilot.tools.lib.logreader import LogReader
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.structs import CarParams
from opendbc.car.hyundai.values import CAR as HCAR

DRIVELOG = "/home/user/openpilot/drivelog"
DT = 0.01
LPF_DT = 0.02  # 50 Hz

def build_vm():
    specs = HCAR.HYUNDAI_IONIQ_6_N.config.specs
    cp = CarParams.new_message()
    cp.mass = specs.mass; cp.wheelbase = specs.wheelbase
    cp.steerRatio = specs.steerRatio
    cp.centerToFront = specs.wheelbase * specs.centerToFrontRatio
    cp.steerRatioRear = 0.0; cp.tireStiffnessFactor = specs.tireStiffnessFactor
    cp.rotationalInertia = specs.mass * specs.wheelbase**2 * 0.35
    cp.tireStiffnessFront = 1.9e5 * specs.tireStiffnessFactor
    cp.tireStiffnessRear = 1.9e5 * specs.tireStiffnessFactor
    return VehicleModel(cp)

def get_all_segs():
    return sorted(glob.glob(os.path.join(DRIVELOG, "*rlog.zst")))

def extract(path):
    des, act, v_arr = [], [], []
    last_v, last_ang = 0.0, 0.0
    for msg in LogReader(path):
        w = msg.which()
        if w == 'carState':
            last_v = msg.carState.vEgo
            last_ang = msg.carState.steeringAngleDeg
        elif w == 'controlsState':
            lat = msg.controlsState.lateralControlState
            if lat.which() == 'angleState' and lat.angleState.active and last_v > 3.0:
                des.append(float(lat.angleState.steeringAngleDesiredDeg))
                act.append(last_ang)
                v_arr.append(last_v)
    return np.array(des), np.array(act), np.array(v_arr)

def sim_lpf(des, tau):
    if tau < 0.001: return des.copy()
    alpha = LPF_DT / (tau + LPF_DT)
    out = np.zeros_like(des)
    out[0] = des[0]
    for i in range(1, len(des)):
        out[i] = alpha * des[i] + (1-alpha) * out[i-1]
    return out

def sim_adaptive_lpf(des, v, tau_straight, tau_curve, curv_thresh=3.0):
    """Adaptive: use tau_curve when |desired| > curv_thresh, tau_straight otherwise."""
    out = np.zeros_like(des)
    out[0] = des[0]
    for i in range(1, len(des)):
        tau = tau_curve if abs(des[i]) > curv_thresh else tau_straight
        alpha = LPF_DT / (tau + LPF_DT)
        out[i] = alpha * des[i] + (1-alpha) * out[i-1]
    return out

segs = get_all_segs()
print(f"Adaptive LPF Analysis: {len(segs)} segments")

# Test configurations
configs = [
    ('fixed_0.00', lambda d, v: d),
    ('fixed_0.10', lambda d, v: sim_lpf(d, 0.10)),
    ('fixed_0.15', lambda d, v: sim_lpf(d, 0.15)),
    ('fixed_0.20', lambda d, v: sim_lpf(d, 0.20)),
    ('fixed_0.25', lambda d, v: sim_lpf(d, 0.25)),
    ('fixed_0.30', lambda d, v: sim_lpf(d, 0.30)),
    ('adapt_0.25/0.08', lambda d, v: sim_adaptive_lpf(d, v, 0.25, 0.08)),
    ('adapt_0.20/0.05', lambda d, v: sim_adaptive_lpf(d, v, 0.20, 0.05)),
    ('adapt_0.20/0.10', lambda d, v: sim_adaptive_lpf(d, v, 0.20, 0.10)),
    ('adapt_0.15/0.05', lambda d, v: sim_adaptive_lpf(d, v, 0.15, 0.05)),
]

results = {name: {'all': [], 'curve': [], 'straight': []} for name, _ in configs}
done = 0

for path in segs:
    done += 1
    try:
        des, act, v = extract(path)
    except Exception:
        continue
    if len(des) < 100: continue

    curve_mask = np.abs(des) > 3.0
    straight_mask = ~curve_mask

    for name, fn in configs:
        filtered = fn(des, v)
        err = np.abs(filtered - act)
        results[name]['all'].append(err.mean())
        if curve_mask.sum() > 20:
            results[name]['curve'].append(err[curve_mask].mean())
        if straight_mask.sum() > 20:
            results[name]['straight'].append(err[straight_mask].mean())

    if done % 50 == 0:
        print(f"  [{done}/{len(segs)}]")

print(f"\n{'='*60}")
print(f"ADAPTIVE LPF RESULTS")
print(f"{'='*60}")
print(f"\n{'Config':<20s} | {'MAE_all':>8s} | {'MAE_curve':>9s} | {'MAE_str':>8s} | {'Δ_all':>7s}")
print(f"{'-'*20}-+-{'-'*8}-+-{'-'*9}-+-{'-'*8}-+-{'-'*7}")
baseline_all = np.mean(results['fixed_0.20']['all'])
for name, _ in configs:
    r = results[name]
    mae_all = np.mean(r['all']) if r['all'] else 0
    mae_curve = np.mean(r['curve']) if r['curve'] else 0
    mae_str = np.mean(r['straight']) if r['straight'] else 0
    delta = mae_all - baseline_all
    print(f"{name:<20s} | {mae_all:8.3f} | {mae_curve:9.3f} | {mae_str:8.3f} | {delta:+7.3f}")
