#!/usr/bin/env python3
"""Simulate the 4 improvements (rate limit, quantize, dual VM, 4-bp torque factor)
on existing drivelogs and compare before/after ACIGain behavior."""
import sys, os, glob
import numpy as np
sys.path.insert(0, '/home/user/openpilot')
sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG = '/home/user/openpilot/drivelog'
segs = sorted(glob.glob(os.path.join(DRIVELOG, '*rlog.zst')))

# --- Replicate the new code logic ---
def compute_driver_torque_factor(steering_torque, v_ego, lat_active):
    if not lat_active:
        return 0.0
    bp1 = float(np.interp(v_ego, [3., 8., 15., 22.], [200., 220., 175., 80.]))
    bp2 = float(np.interp(v_ego, [3., 8., 15., 22.], [300., 280., 260., 200.]))
    bp3 = float(np.interp(v_ego, [3., 8., 15., 22.], [380., 330., 290., 290.]))
    bp4 = float(np.interp(v_ego, [3., 8., 15., 22.], [500., 470., 420., 400.]))
    ceiling = 1.0
    shelf = float(np.interp(v_ego, [3., 22.], [0.65, 0.80]))
    floor = float(np.interp(v_ego, [3., 22.], [0.15, 0.35]))
    return float(np.interp(abs(steering_torque), [bp1, bp2, bp3, bp4], [ceiling, shelf, shelf, floor]))

def old_driver_torque_blend(torque, v_ego):
    """Old linear ramp (before this change)."""
    DZ = 100.0
    full_lo = 300.0
    full_hi = 500.0
    full = float(np.interp(v_ego, [8.0, 15.0], [full_lo, full_hi]))
    override = float(np.clip((abs(torque) - DZ) / max(full - DZ, 1.0), 0.0, 1.0))
    return 1.0 - override

ACI_GAIN_RATE_DOWN = -0.028
ACI_GAIN_RATE_UP = 0.008
ACI_GAIN_QUANT = 0.004
LON_COMFORT_BP = [1.0, 2.5, 4.0]
LON_COMFORT_V = [1.0, 0.7, 0.5]

def rate_limit(x, x_last, down, up):
    return max(x_last + down, min(x, x_last + up))

def quantize(g):
    return round(g / ACI_GAIN_QUANT) * ACI_GAIN_QUANT

# Collect frames
v_all, tq_all, pressed_all, lat_active_all, aego_all = [], [], [], [], []
for i, path in enumerate(segs):
    try:
        last_v, last_tq, last_sp, last_aego = 0.0, 0.0, False, 0.0
        for msg in LogReader(path):
            w = msg.which()
            if w == 'carState':
                cs = msg.carState
                last_v = cs.vEgo
                last_tq = cs.steeringTorque
                last_sp = cs.steeringPressed
                last_aego = cs.aEgo
            elif w == 'controlsState':
                lat = msg.controlsState.lateralControlState
                if lat.which() == 'angleState':
                    active = bool(lat.angleState.active)
                    v_all.append(last_v)
                    tq_all.append(last_tq)
                    pressed_all.append(last_sp)
                    lat_active_all.append(active)
                    aego_all.append(last_aego)
    except Exception:
        pass
    if (i+1) % 10 == 0:
        print(f'  processed {i+1}/{len(segs)}', file=sys.stderr)

v = np.array(v_all)
tq = np.array(tq_all)
pressed = np.array(pressed_all)
lat_active = np.array(lat_active_all)
aego = np.array(aego_all)

print(f'Total frames: {len(v)}, latActive: {lat_active.sum()}')
print()

# --- Simulate old vs new on latActive frames ---
mask = lat_active
n = mask.sum()

# OLD: simple DTB -> raw_gain (no rate limit, no quantize)
old_dtb = np.array([old_driver_torque_blend(tq[i], v[i]) for i in range(len(v)) if mask[i]])
old_speeds = v[mask]
old_tqs = tq[mask]
old_aegos = aego[mask]
old_pressed = pressed[mask]

# Assume speed_blend=1, aci_gain_ramp=1 for steady-state comparison
old_lon_comfort = np.array([float(np.interp(abs(a), LON_COMFORT_BP, LON_COMFORT_V)) for a in old_aegos])
old_raw_gain = np.maximum(old_dtb * old_lon_comfort, 0.10)

# NEW: 4-bp torque factor + rate limit + quantize
new_dtf = np.array([compute_driver_torque_factor(old_tqs[i], old_speeds[i], True) for i in range(n)])
new_raw_gain = np.maximum(new_dtf * old_lon_comfort, 0.10)

# Simulate rate limit + quantize frame-by-frame
new_rl_gain = np.zeros(n)
last_g = 0.0
for i in range(n):
    target = new_raw_gain[i]
    g = rate_limit(target, last_g, ACI_GAIN_RATE_DOWN, ACI_GAIN_RATE_UP)
    g = quantize(g)
    new_rl_gain[i] = g
    last_g = g

print('=== OLD (linear DTB, no rate limit) vs NEW (4-bp, rate limit, quantize) ===')
print(f'{"Metric":>35} {"OLD":>10} {"NEW":>10} {"delta":>10}')

for desc, old_arr, new_arr in [
    ('mean ACIGain (latActive)', old_raw_gain, new_rl_gain),
]:
    print(f'{desc:>35} {old_arr.mean():>10.4f} {new_arr.mean():>10.4f} {(new_arr.mean()-old_arr.mean()):>+10.4f}')

print()
# Distribution comparison
for p in [10, 25, 50, 75, 90, 95, 99]:
    old_p = np.percentile(old_raw_gain, p)
    new_p = np.percentile(new_rl_gain, p)
    print(f'  p{p:02d}: old={old_p:.4f}  new={new_p:.4f}  delta={new_p-old_p:+.4f}')

print()
# NOT-pressed vs PRESSED breakdown
for label, sub_mask in [('NOT-pressed', ~old_pressed), ('PRESSED', old_pressed)]:
    if sub_mask.sum() == 0: continue
    old_sub = old_raw_gain[sub_mask]
    new_sub = new_rl_gain[sub_mask]
    print(f'{label} ({sub_mask.sum()} frames):')
    print(f'  mean:  old={old_sub.mean():.4f}  new={new_sub.mean():.4f}  delta={new_sub.mean()-old_sub.mean():+.4f}')
    print(f'  p50:   old={np.percentile(old_sub,50):.4f}  new={np.percentile(new_sub,50):.4f}')
    print(f'  p10:   old={np.percentile(old_sub,10):.4f}  new={np.percentile(new_sub,10):.4f}')

# Frame-over-frame delta analysis (rate limit effectiveness)
print()
old_deltas = np.diff(old_raw_gain)
new_deltas = np.diff(new_rl_gain)
print(f'Frame-over-frame |delta| (gain jitter):')
print(f'  OLD: mean={np.abs(old_deltas).mean():.6f}  max={np.abs(old_deltas).max():.4f}  p99={np.percentile(np.abs(old_deltas),99):.4f}')
print(f'  NEW: mean={np.abs(new_deltas).mean():.6f}  max={np.abs(new_deltas).max():.4f}  p99={np.percentile(np.abs(new_deltas),99):.4f}')

# Speed-bucket comparison
print()
print('=== Speed-bucket gain comparison ===')
speed_bins = [(0, 3, '0-11 km/h'), (3, 8, '11-29 km/h'), (8, 15, '29-54 km/h'),
              (15, 22, '54-79 km/h'), (22, 35, '79+ km/h')]
for lo, hi, label in speed_bins:
    sm = (old_speeds >= lo) & (old_speeds < hi)
    if sm.sum() < 100: continue
    print(f'{label}: old_mean={old_raw_gain[sm].mean():.4f}  new_mean={new_rl_gain[sm].mean():.4f}  '
          f'old_p50={np.percentile(old_raw_gain[sm],50):.4f}  new_p50={np.percentile(new_rl_gain[sm],50):.4f}')
