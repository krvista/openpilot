#!/usr/bin/env python3
"""Analyze steeringTorque distribution by speed bin and steeringPressed state."""
import sys, os, glob
import numpy as np
sys.path.insert(0, '/home/user/openpilot')
sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG = '/home/user/openpilot/drivelog'
segs = sorted(glob.glob(os.path.join(DRIVELOG, '*rlog.zst')))

v_all, tq_all, pressed_all = [], [], []

for i, path in enumerate(segs):
    try:
        last_v, last_tq, last_sp = 0.0, 0.0, False
        for msg in LogReader(path):
            w = msg.which()
            if w == 'carState':
                cs = msg.carState
                last_v = cs.vEgo
                last_tq = cs.steeringTorque
                last_sp = cs.steeringPressed
            elif w == 'controlsState':
                lat = msg.controlsState.lateralControlState
                if lat.which() == 'angleState':
                    lat_active = bool(lat.angleState.active)
                    if lat_active:
                        v_all.append(last_v)
                        tq_all.append(abs(last_tq))
                        pressed_all.append(last_sp)
    except Exception:
        pass
    if (i+1) % 5 == 0:
        print(f'  processed {i+1}/{len(segs)}', file=sys.stderr)

v = np.array(v_all)
tq = np.array(tq_all)
pressed = np.array(pressed_all)

print(f'Total latActive frames: {len(v)}')
print(f'steeringPressed=False: {(~pressed).sum()}, True: {pressed.sum()}')
print()

speed_bins = [(0, 3, '0-11 km/h'), (3, 8, '11-29 km/h'), (8, 15, '29-54 km/h'),
              (15, 22, '54-79 km/h'), (22, 35, '79+ km/h')]

hdr = f"{'Speed':>15} {'count':>7} {'p10':>7} {'p25':>7} {'p50':>7} {'p75':>7} {'p90':>7} {'p95':>7} {'p99':>7} {'max':>7}"

print('=== steeringPressed=False (light grip / no override) ===')
print(hdr)
for lo, hi, label in speed_bins:
    mask = (~pressed) & (v >= lo) & (v < hi)
    n = mask.sum()
    if n < 10:
        print(f'{label:>15} {n:>7} -- too few')
        continue
    t = tq[mask]
    print(f'{label:>15} {n:>7} {np.percentile(t,10):>7.0f} {np.percentile(t,25):>7.0f} {np.percentile(t,50):>7.0f} {np.percentile(t,75):>7.0f} {np.percentile(t,90):>7.0f} {np.percentile(t,95):>7.0f} {np.percentile(t,99):>7.0f} {t.max():>7.0f}')

print()
print('=== steeringPressed=True (driver actively steering) ===')
print(hdr)
for lo, hi, label in speed_bins:
    mask = pressed & (v >= lo) & (v < hi)
    n = mask.sum()
    if n < 10:
        print(f'{label:>15} {n:>7} -- too few')
        continue
    t = tq[mask]
    print(f'{label:>15} {n:>7} {np.percentile(t,10):>7.0f} {np.percentile(t,25):>7.0f} {np.percentile(t,50):>7.0f} {np.percentile(t,75):>7.0f} {np.percentile(t,90):>7.0f} {np.percentile(t,95):>7.0f} {np.percentile(t,99):>7.0f} {t.max():>7.0f}')

print()
print('=== ALL latActive ===')
print(hdr)
for lo, hi, label in speed_bins:
    mask = (v >= lo) & (v < hi)
    n = mask.sum()
    if n < 10:
        continue
    t = tq[mask]
    print(f'{label:>15} {n:>7} {np.percentile(t,10):>7.0f} {np.percentile(t,25):>7.0f} {np.percentile(t,50):>7.0f} {np.percentile(t,75):>7.0f} {np.percentile(t,90):>7.0f} {np.percentile(t,95):>7.0f} {np.percentile(t,99):>7.0f} {t.max():>7.0f}')

# Also compute: what fraction of NOT-pressed frames fall in various Nm ranges?
print()
print('=== NOT-pressed torque CDF (0 to 400 Nm, step 25) ===')
not_pressed_tq = tq[~pressed]
if len(not_pressed_tq) > 0:
    for thresh in range(0, 425, 25):
        pct = (not_pressed_tq <= thresh).sum() / len(not_pressed_tq) * 100
        print(f'  |tq| <= {thresh:>3} Nm: {pct:>6.1f}%')

# And pressed
print()
print('=== PRESSED torque CDF (0 to 900 Nm, step 50) ===')
pressed_tq = tq[pressed]
if len(pressed_tq) > 0:
    for thresh in range(0, 950, 50):
        pct = (pressed_tq <= thresh).sum() / len(pressed_tq) * 100
        print(f'  |tq| <= {thresh:>3} Nm: {pct:>6.1f}%')
