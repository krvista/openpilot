#!/usr/bin/env python3
"""Analyze parking lot driving patterns from route 36 (last segments
where the user descended B1→B6) and the failed routes (32-39, all in
parking lots). Extract:

1. Speed distribution during parking maneuvers
2. Steering angle change rate per speed bucket (how fast the wheel
   needs to move in tight maneuvers)
3. Steering angle magnitude per speed bucket (how much lock is used)

Goal: tune speed_blend ramp to match natural driving patterns.
"""
import glob
import sys
import numpy as np
from collections import defaultdict

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG_DIR = '/home/user/openpilot/drivelog'


def analyze_parking(route_pattern, label, last_n_segs=None):
  segs = sorted(glob.glob(f'{DRIVELOG_DIR}/*{route_pattern}*rlog.zst'),
                key=lambda p: int(p.split('--')[-2]))
  if last_n_segs:
    segs = segs[-last_n_segs:]

  print(f"\n=== {label} ({len(segs)} segs) ===")

  # Buckets
  BUCKETS = [(0, 2), (2, 5), (5, 7), (7, 10), (10, 15), (15, 20),
             (20, 30), (30, 50), (50, 100)]
  bkt_speed_time = defaultdict(int)        # frame count per bucket (proxy for time)
  bkt_steer_angle = defaultdict(list)      # |steering angle| per bucket
  bkt_steer_rate = defaultdict(list)       # |angle delta per 100ms| per bucket
  bkt_driver_torque = defaultdict(list)    # |driver torque| per bucket

  speed_kmh = 0.0
  prev_steer = None
  prev_t = None
  prev_steer_t = None

  for p in segs:
    for m in LogReader(p):
      try:
        w = m.which()
      except:
        continue
      t_ns = m.logMonoTime

      if w == 'carState':
        cs = m.carState
        speed_kmh = cs.vEgoRaw * 3.6
        steer_angle = cs.steeringAngleDeg
        driver_torque = abs(cs.steeringTorque)

        bkt = None
        for lo, hi in BUCKETS:
          if lo <= speed_kmh < hi:
            bkt = (lo, hi)
            break
        if bkt is None:
          continue

        bkt_speed_time[bkt] += 1
        bkt_steer_angle[bkt].append(abs(steer_angle))
        bkt_driver_torque[bkt].append(driver_torque)

        if prev_steer is not None and prev_steer_t is not None:
          dt = (t_ns - prev_steer_t) / 1e9
          if 0.05 < dt < 0.15:  # ~100ms window
            rate_dps = abs(steer_angle - prev_steer) / dt
            bkt_steer_rate[bkt].append(rate_dps)
        prev_steer = steer_angle
        prev_steer_t = t_ns

  # Report
  total_frames = sum(bkt_speed_time.values())
  if total_frames == 0:
    print("  No carState frames found!")
    return

  print(f"  Total time samples: {total_frames}")
  print(f"  {'Speed':>10} {'time%':>6} {'|angle| p50':>12} {'p95':>8} "
        f"{'rate p50':>10} {'p95':>8} {'driver_tq p95':>14}")

  for lo, hi in BUCKETS:
    bkt = (lo, hi)
    n = bkt_speed_time[bkt]
    if n == 0:
      continue
    pct = n / total_frames * 100

    angles = bkt_steer_angle[bkt]
    rates = bkt_steer_rate[bkt]
    torques = bkt_driver_torque[bkt]

    a_p50 = np.percentile(angles, 50) if angles else 0
    a_p95 = np.percentile(angles, 95) if angles else 0
    r_p50 = np.percentile(rates, 50) if rates else 0
    r_p95 = np.percentile(rates, 95) if rates else 0
    t_p95 = np.percentile(torques, 95) if torques else 0

    marker = ""
    if hi <= 7:
      marker = "  ← parking range"
    elif hi <= 20:
      marker = "  ← transition range"

    print(f"  {lo:>2}-{hi:<3} km/h {pct:>5.1f}% "
          f"{a_p50:>10.1f}° {a_p95:>7.1f}° "
          f"{r_p50:>8.1f}°/s {r_p95:>6.1f}°/s "
          f"{t_p95:>13.1f}{marker}")


# Analyze the parking sections
# Route 36 last 5 segments (underground descent B1→B6)
analyze_parking('00000036', 'Route 36 last 5 segs (underground B1→B6)', last_n_segs=5)
# Route 35-39 (all parking lot)
for r in ['00000035', '00000037', '00000038', '00000039']:
  analyze_parking(r, f'Route {r[-2:]} (parking, with errors)')
