#!/usr/bin/env python3
"""Analysis B: Model look-ahead benefit.
modelV2 outputs position predictions at future timesteps (0.0-10.0s).
If we use the model's curvature prediction at t+50ms or t+100ms instead of t+0,
would tracking error decrease? This measures the "anticipatory" potential."""
import sys, os, glob, math
import numpy as np
sys.path.insert(0, '/home/user/openpilot')
sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG = "/home/user/openpilot/drivelog"

def get_all_segs():
    return sorted(glob.glob(os.path.join(DRIVELOG, "*rlog.zst")))

def extract_model_curvature_series(path):
    """Extract time-aligned series of:
    - modelV2.desiredCurvature (t+0 prediction)
    - modelV2 position x/y at various horizons → curvature at t+dt
    - actual steeringAngleDeg (ground truth)
    """
    desired_curvs = []
    actual_angs = []
    speeds = []
    model_positions = []  # list of (x_arr, y_arr) at model timesteps

    last_v = 0.0
    last_ang = 0.0
    last_active = False

    for msg in LogReader(path):
        w = msg.which()
        if w == 'carState':
            last_v = msg.carState.vEgo
            last_ang = msg.carState.steeringAngleDeg
        elif w == 'controlsState':
            lat = msg.controlsState.lateralControlState
            if lat.which() == 'angleState':
                last_active = bool(lat.angleState.active)
        elif w == 'modelV2':
            if not last_active or last_v < 5.0:
                continue
            try:
                md = msg.modelV2
                # desiredCurvature from model
                dc = float(md.meta.desiredCurvature)
                # position predictions (x=forward distance, y=lateral offset)
                pos_x = list(md.position.x)
                pos_y = list(md.position.y)
                if len(pos_x) < 10:
                    continue
                desired_curvs.append(dc)
                actual_angs.append(last_ang)
                speeds.append(last_v)
                model_positions.append((pos_x, pos_y))
            except Exception:
                continue

    return (np.array(desired_curvs), np.array(actual_angs),
            np.array(speeds), model_positions)

def curvature_from_positions(x, y, idx):
    """Estimate curvature at point idx using 3-point formula."""
    if idx < 1 or idx >= len(x) - 1:
        return 0.0
    x0, y0 = x[idx-1], y[idx-1]
    x1, y1 = x[idx], y[idx]
    x2, y2 = x[idx+1], y[idx+1]
    dx1, dy1 = x1-x0, y1-y0
    dx2, dy2 = x2-x1, y2-y1
    ds1 = math.sqrt(dx1**2 + dy1**2)
    ds2 = math.sqrt(dx2**2 + dy2**2)
    if ds1 < 0.01 or ds2 < 0.01:
        return 0.0
    # curvature ≈ 2*(dx1*dy2 - dy1*dx2) / (ds1*ds2*(ds1+ds2))
    cross = dx1*dy2 - dy1*dx2
    return 2.0 * cross / (ds1 * ds2 * (ds1 + ds2))

segs = get_all_segs()
print(f"Model Look-ahead Analysis: {len(segs)} segments")

# For each model frame, compute:
# - curvature at horizon 0 (= desiredCurvature)
# - curvature at ~5m ahead (≈ 50-100ms at typical speed)
# - curvature at ~10m ahead
# Then shift the series and measure MAE reduction

# Collect: (desired_curv_now, curv_at_5m, curv_at_10m, actual_ang, speed)
all_data = []
done = 0

for path in segs:
    done += 1
    try:
        curvs, angs, speeds, positions = extract_model_curvature_series(path)
    except Exception:
        continue
    if len(curvs) < 50:
        continue

    for i in range(len(curvs)):
        px, py = positions[i]
        v = speeds[i]
        # Find index in position array corresponding to ~50ms and ~100ms ahead
        # at speed v: distance = v * dt
        d_50ms = v * 0.05
        d_100ms = v * 0.10
        d_150ms = v * 0.15

        # position x values are forward distances from ego
        px_arr = np.array(px)
        idx_50 = np.searchsorted(px_arr, d_50ms)
        idx_100 = np.searchsorted(px_arr, d_100ms)
        idx_150 = np.searchsorted(px_arr, d_150ms)

        c_50 = curvature_from_positions(px, py, min(idx_50, len(px)-2))
        c_100 = curvature_from_positions(px, py, min(idx_100, len(px)-2))
        c_150 = curvature_from_positions(px, py, min(idx_150, len(px)-2))

        all_data.append((curvs[i], c_50, c_100, c_150, angs[i], v))

    if done % 50 == 0:
        print(f"  [{done}/{len(segs)}]  samples: {len(all_data)}")

print(f"\nTotal samples: {len(all_data)}")

if len(all_data) < 100:
    print("Not enough data")
    sys.exit(0)

data = np.array(all_data)
curv_now = data[:, 0]
curv_50 = data[:, 1]
curv_100 = data[:, 2]
curv_150 = data[:, 3]
ang_actual = data[:, 4]
speed = data[:, 5]

# Convert curvature to approximate angle for comparison
# angle ≈ degrees(arctan(curv * wheelbase * steer_ratio))
WB = 2.87  # Ioniq 6N wheelbase
SR = 13.6  # steer ratio
def curv_to_angle(c):
    return np.degrees(np.arctan(c * WB)) * SR

ang_from_curv_now = curv_to_angle(curv_now)
ang_from_curv_50 = curv_to_angle(curv_50)
ang_from_curv_100 = curv_to_angle(curv_100)
ang_from_curv_150 = curv_to_angle(curv_150)

# Shift analysis: if we use future curvature NOW, does it match actual better?
# But actual angle is what EPS is CURRENTLY at — it should match desired with a delay.
# So the question is: does curv_at_future match what actual_angle becomes LATER?

# Better approach: use the time-series shift.
# desired_curvature[t] → actual_angle[t + lag]
# If we apply curv_at_50ms[t] instead of curv_now[t], does actual follow sooner?

# Simpler metric: temporal autocorrelation of desired_curvature changes
# and cross-correlation with actual angle changes

# Just measure: how much does curvature change between t+0 and t+50ms/100ms/150ms?
delta_50 = np.abs(ang_from_curv_50 - ang_from_curv_now)
delta_100 = np.abs(ang_from_curv_100 - ang_from_curv_now)
delta_150 = np.abs(ang_from_curv_150 - ang_from_curv_now)

print(f"\n{'='*60}")
print(f"MODEL LOOK-AHEAD ANALYSIS")
print(f"{'='*60}")
print(f"\nAngle difference between t+0 and future predictions:")
print(f"  t+50ms:  mean={delta_50.mean():.3f}°  p50={np.median(delta_50):.3f}°  p95={np.percentile(delta_50, 95):.3f}°")
print(f"  t+100ms: mean={delta_100.mean():.3f}°  p50={np.median(delta_100):.3f}°  p95={np.percentile(delta_100, 95):.3f}°")
print(f"  t+150ms: mean={delta_150.mean():.3f}°  p50={np.median(delta_150):.3f}°  p95={np.percentile(delta_150, 95):.3f}°")

# Does the future curvature better predict current actual angle?
err_now = np.abs(ang_from_curv_now - ang_actual)
err_50 = np.abs(ang_from_curv_50 - ang_actual)
err_100 = np.abs(ang_from_curv_100 - ang_actual)
err_150 = np.abs(ang_from_curv_150 - ang_actual)

print(f"\nMAE: model curvature (converted to angle) vs actual steering angle:")
print(f"  Using t+0ms:   {err_now.mean():.3f}°")
print(f"  Using t+50ms:  {err_50.mean():.3f}°  (Δ={err_50.mean()-err_now.mean():+.3f}°)")
print(f"  Using t+100ms: {err_100.mean():.3f}°  (Δ={err_100.mean()-err_now.mean():+.3f}°)")
print(f"  Using t+150ms: {err_150.mean():.3f}°  (Δ={err_150.mean()-err_now.mean():+.3f}°)")

# Speed-binned
print(f"\nSpeed-binned look-ahead benefit (t+100ms vs t+0):")
for label, lo, hi in [('20-40 km/h', 5.6, 11.1), ('40-70 km/h', 11.1, 19.4), ('70-120 km/h', 19.4, 33.3)]:
    mask = (speed >= lo) & (speed < hi)
    if mask.sum() > 100:
        e0 = err_now[mask].mean()
        e100 = err_100[mask].mean()
        print(f"  {label}: N={mask.sum()}  MAE_now={e0:.3f}°  MAE_100ms={e100:.3f}°  Δ={e100-e0:+.3f}°")

# Curvature magnitude binned
print(f"\nCurvature-binned look-ahead (|curv| > 0.002 = curve vs straight):")
curv_abs = np.abs(curv_now)
for label, lo, hi in [('straight |c|<0.001', 0, 0.001), ('gentle 0.001-0.005', 0.001, 0.005), ('curve >0.005', 0.005, 1.0)]:
    mask = (curv_abs >= lo) & (curv_abs < hi)
    if mask.sum() > 50:
        e0 = err_now[mask].mean()
        e100 = err_100[mask].mean()
        print(f"  {label}: N={mask.sum()}  MAE_now={e0:.3f}°  MAE_100ms={e100:.3f}°  Δ={e100-e0:+.3f}°")
