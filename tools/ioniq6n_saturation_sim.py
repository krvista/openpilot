#!/usr/bin/env python3
"""Simulate saturation logic: old (False) vs new (steer_limited_by_safety).

Extracts 4 signals per frame:
  - desired: LatControlAngle's steeringAngleDesiredDeg
  - actual: actual wheel angle (CS.steeringAngleDeg)
  - clipped: carOutput.actuatorsOutput.steeringAngleDeg (after VM limiting)
  - v, pressed

Old logic: saturated = |desired - actual| > 2.5°, suppression = False
New logic: saturated = |desired - actual| > 2.5°, suppression = |desired - clipped| > 2.5°
"""
import sys, os, glob
import numpy as np

sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG = "/home/user/openpilot/drivelog"
SAT_THRESHOLD = 2.5
SAT_LIMIT = 0.4  # Hyundai steerLimitTimer
SAT_MIN_SPEED = 5.0
DT = 0.01


def find_segments(route_hex):
  rh = route_hex.strip().lower().zfill(8)
  pattern = os.path.join(DRIVELOG, f"*_{rh}--*--rlog.zst")
  files = sorted(glob.glob(pattern))
  segs = {}
  for f in files:
    parts = os.path.basename(f).split('--')
    if len(parts) >= 3:
      segs[int(parts[-2])] = f
  return segs


def sim_segment(path):
  desired_arr = []
  actual_arr = []
  clipped_arr = []
  v_arr = []
  pressed_arr = []

  last_v = 0.0
  last_pressed = False
  last_clipped = 0.0

  for msg in LogReader(path):
    w = msg.which()
    try:
      if w == 'carState':
        last_v = msg.carState.vEgo
        last_pressed = msg.carState.steeringPressed
      elif w == 'carOutput':
        last_clipped = msg.carOutput.actuatorsOutput.steeringAngleDeg
      elif w == 'controlsState':
        cs = msg.controlsState
        lat = cs.lateralControlState
        if lat.which() == 'angleState':
          ang = lat.angleState
          if ang.active:
            desired_arr.append(float(ang.steeringAngleDesiredDeg))
            actual_arr.append(float(ang.steeringAngleDeg))
            clipped_arr.append(float(last_clipped))
            v_arr.append(float(last_v))
            pressed_arr.append(bool(last_pressed))
    except Exception:
      pass

  if not desired_arr:
    return None

  desired = np.array(desired_arr)
  actual = np.array(actual_arr)
  clipped = np.array(clipped_arr)
  v = np.array(v_arr)
  pressed = np.array(pressed_arr)

  err_desired_actual = np.abs(desired - actual)
  err_desired_clipped = np.abs(desired - clipped)

  old_saturated = err_desired_actual > SAT_THRESHOLD  # old: |desired - actual_wheel|
  new_saturated = err_desired_clipped > SAT_THRESHOLD  # new: use_steer_limited = |desired - clipped|
  steer_limited = err_desired_clipped > SAT_THRESHOLD

  old_sat = 0.0
  new_sat = 0.0
  old_alerts = 0
  new_alerts = 0
  old_sat_events = 0
  new_sat_events = 0
  old_was_alert = False
  new_was_alert = False

  for i in range(len(desired)):
    # OLD: saturated = |desired - actual| > 2.5, suppression = False
    if old_saturated[i] and v[i] > SAT_MIN_SPEED and not pressed[i]:
      old_sat += DT
    else:
      old_sat -= DT
    old_sat = np.clip(old_sat, 0.0, SAT_LIMIT)
    old_alert_now = old_sat > SAT_LIMIT - 1e-3
    if old_alert_now:
      old_alerts += 1
    if old_alert_now and not old_was_alert:
      old_sat_events += 1
    old_was_alert = old_alert_now

    # NEW: saturated = |desired - clipped| > 2.5 (use_steer_limited=True)
    #      suppression = steer_limited (same signal → always cancels)
    #      → only curvature_limited can trigger, which we don't have in log
    #      So new_alerts should be ~0
    if new_saturated[i] and v[i] > SAT_MIN_SPEED and not steer_limited[i] and not pressed[i]:
      new_sat += DT
    else:
      new_sat -= DT
    new_sat = np.clip(new_sat, 0.0, SAT_LIMIT)
    new_alert_now = new_sat > SAT_LIMIT - 1e-3
    if new_alert_now:
      new_alerts += 1
    if new_alert_now and not new_was_alert:
      new_sat_events += 1
    new_was_alert = new_alert_now

  n_hi_speed_sat = int(np.sum(old_saturated & (v > SAT_MIN_SPEED) & ~pressed))
  n_genuine = int(np.sum(old_saturated & (v > SAT_MIN_SPEED) & ~steer_limited & ~pressed))

  return {
    'frames': len(desired),
    'old_alerts': old_alerts,
    'new_alerts': new_alerts,
    'old_events': old_sat_events,
    'new_events': new_sat_events,
    'hi_speed_sat': n_hi_speed_sat,
    'genuine_sat': n_genuine,
    'v_mean': float(np.mean(v)),
  }


def main(route_hexes):
  for rh in route_hexes:
    segs = find_segments(rh)
    if not segs:
      print(f"Route {rh}: not found")
      continue
    print(f"\n{'='*80}")
    print(f"  Route 0x{rh.zfill(8)[-2:]} — saturation simulation (old vs new)")
    print(f"{'='*80}")
    tot_old_a = 0; tot_new_a = 0
    tot_old_e = 0; tot_new_e = 0
    tot_frames = 0; tot_hi = 0; tot_genuine = 0
    for sn in sorted(segs.keys()):
      try:
        r = sim_segment(segs[sn])
      except Exception as e:
        print(f"  seg {sn}: ERROR {e}")
        continue
      if r is None:
        continue
      tot_old_a += r['old_alerts']
      tot_new_a += r['new_alerts']
      tot_old_e += r['old_events']
      tot_new_e += r['new_events']
      tot_frames += r['frames']
      tot_hi += r['hi_speed_sat']
      tot_genuine += r['genuine_sat']
      if r['old_alerts'] > 0 or r['new_alerts'] > 0:
        print(f"  seg {sn:3d}  v={r['v_mean']:5.1f}  "
              f"OLD: {r['old_alerts']:5d} frames / {r['old_events']:2d} events   "
              f"NEW: {r['new_alerts']:5d} frames / {r['new_events']:2d} events   "
              f"genuine_sat={r['genuine_sat']}")

    pct_a = 100 * (1 - tot_new_a / max(tot_old_a, 1))
    pct_e = 100 * (1 - tot_new_e / max(tot_old_e, 1))
    print(f"\n  TOTALS ({tot_frames} active frames):")
    print(f"    hi-speed saturated: {tot_hi} (|desired-actual|>2.5° && v>18km/h && !pressed)")
    print(f"    genuine (not rate-limited): {tot_genuine}")
    print(f"    OLD alerts: {tot_old_a} frames / {tot_old_e} events")
    print(f"    NEW alerts: {tot_new_a} frames / {tot_new_e} events")
    print(f"    Reduction: frames {pct_a:.0f}%, events {pct_e:.0f}%")


if __name__ == "__main__":
  routes = sys.argv[1:] if len(sys.argv) > 1 else ['3f', '40']
  main(routes)
