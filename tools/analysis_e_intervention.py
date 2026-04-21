#!/usr/bin/env python3
"""Analysis E: Intervention onset — what happens 1-3s BEFORE driver grabs wheel?
Identifies precursor signals that could trigger preemptive corrections."""
import sys, os, glob, math
import numpy as np
sys.path.insert(0, '/home/user/openpilot')
sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG = "/home/user/openpilot/drivelog"
DT = 0.01

def get_all_segs():
    return sorted(glob.glob(os.path.join(DRIVELOG, "*rlog.zst")))

def extract_full(path):
    t, des, act, v, pressed, torque, lane_probs, curv = [], [], [], [], [], [], [], []
    last_v, last_ang, last_sp, last_st = 0.0, 0.0, False, 0.0
    last_lp = [0.5, 0.5, 0.5, 0.5]
    last_curv = 0.0
    last_active = False
    init_t = None

    for msg in LogReader(path):
        w = msg.which()
        if w == 'carState':
            cs = msg.carState
            last_v = cs.vEgo
            last_ang = cs.steeringAngleDeg
            last_sp = cs.steeringPressed
            last_st = cs.steeringTorque
        elif w == 'modelV2':
            try:
                last_lp = list(msg.modelV2.laneLineProbs)
                last_curv = float(msg.modelV2.meta.desiredCurvature)
            except: pass
        elif w == 'controlsState':
            lat = msg.controlsState.lateralControlState
            if lat.which() == 'angleState':
                active = bool(lat.angleState.active)
                if active and last_v > 3.0:
                    ts = msg.logMonoTime * 1e-9
                    if init_t is None: init_t = ts
                    t.append(ts - init_t)
                    des.append(float(lat.angleState.steeringAngleDesiredDeg))
                    act.append(last_ang)
                    v.append(last_v)
                    pressed.append(last_sp)
                    torque.append(last_st)
                    inner_lp = (last_lp[1] + last_lp[2]) / 2.0 if len(last_lp) >= 4 else 0.5
                    lane_probs.append(inner_lp)
                    curv.append(last_curv)
                last_active = active

    return {k: np.array(v) for k, v in [
        ('t', t), ('des', des), ('act', act), ('v', v),
        ('pressed', pressed), ('torque', torque),
        ('lane_probs', lane_probs), ('curv', curv)
    ]}

segs = get_all_segs()
print(f"Intervention Analysis: {len(segs)} segments")

# Find intervention onsets: steeringPressed transitions False→True
# For each onset, capture the 3s window BEFORE
WINDOW_S = 3.0
WINDOW_F = int(WINDOW_S / DT)

pre_intervention = {
    'tracking_err': [],      # |desired - actual| trend
    'curv_magnitude': [],    # absolute curvature
    'curv_change_rate': [],  # d|curv|/dt in the pre-window
    'lane_prob': [],         # lane line confidence
    'speed': [],
    'err_slope': [],         # is error growing?
}
total_interventions = 0
done = 0

for path in segs:
    done += 1
    try:
        d = extract_full(path)
    except Exception:
        continue
    if len(d['t']) < 500: continue

    # Find pressed onsets
    pressed = d['pressed'].astype(bool)
    onsets = np.where(np.diff(pressed.astype(int)) == 1)[0]

    for onset in onsets:
        if onset < WINDOW_F: continue
        total_interventions += 1
        sl = slice(onset - WINDOW_F, onset)

        err = np.abs(d['des'][sl] - d['act'][sl])
        pre_intervention['tracking_err'].append(err.mean())
        pre_intervention['curv_magnitude'].append(np.abs(d['curv'][sl]).mean())
        pre_intervention['lane_prob'].append(d['lane_probs'][sl].mean())
        pre_intervention['speed'].append(d['v'][sl].mean())

        # Error trend: linear fit slope
        x = np.arange(len(err))
        if len(err) > 10:
            slope = np.polyfit(x, err, 1)[0]
            pre_intervention['err_slope'].append(slope)

        # Curvature change rate
        curv_abs = np.abs(d['curv'][sl])
        if len(curv_abs) > 10:
            curv_slope = np.polyfit(np.arange(len(curv_abs)), curv_abs, 1)[0]
            pre_intervention['curv_change_rate'].append(curv_slope)

    if done % 50 == 0:
        print(f"  [{done}/{len(segs)}] interventions: {total_interventions}")

print(f"\n{'='*60}")
print(f"INTERVENTION ONSET ANALYSIS")
print(f"{'='*60}")
print(f"\nTotal interventions (pressed onset): {total_interventions}")

if total_interventions < 5:
    print("Not enough interventions for analysis")
    sys.exit(0)

for key, label in [
    ('tracking_err', 'Tracking error (3s pre)'),
    ('curv_magnitude', 'Curvature magnitude'),
    ('lane_prob', 'Lane line confidence'),
    ('speed', 'Speed (km/h)'),
    ('err_slope', 'Error slope (growing?)'),
    ('curv_change_rate', 'Curvature change rate'),
]:
    vals = np.array(pre_intervention[key])
    if len(vals) == 0: continue
    if key == 'speed': vals *= 3.6
    print(f"\n  {label}:")
    print(f"    mean={vals.mean():.4f}  median={np.median(vals):.4f}  "
          f"p25={np.percentile(vals,25):.4f}  p75={np.percentile(vals,75):.4f}")

# Classification: what fraction of interventions have each precursor?
err_vals = np.array(pre_intervention['tracking_err'])
lp_vals = np.array(pre_intervention['lane_prob'])
slope_vals = np.array(pre_intervention['err_slope']) if pre_intervention['err_slope'] else np.array([])

print(f"\n\nIntervention precursor classification:")
if len(err_vals) > 0:
    high_err = (err_vals > 3.0).sum()
    print(f"  High tracking error (>3°): {high_err}/{len(err_vals)} ({100*high_err/len(err_vals):.0f}%)")
if len(lp_vals) > 0:
    low_lane = (lp_vals < 0.3).sum()
    print(f"  Low lane confidence (<0.3): {low_lane}/{len(lp_vals)} ({100*low_lane/len(lp_vals):.0f}%)")
if len(slope_vals) > 0:
    growing = (slope_vals > 0.001).sum()
    print(f"  Error growing (slope>0): {growing}/{len(slope_vals)} ({100*growing/len(slope_vals):.0f}%)")
