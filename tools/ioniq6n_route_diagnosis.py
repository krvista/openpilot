#!/usr/bin/env python3
"""Comprehensive drivelog analysis — takeover alerts, saturation, ADAS faults.

Usage: python3 tools/ioniq6n_route_diagnosis.py [route_hex ...]
       e.g.: python3 tools/ioniq6n_route_diagnosis.py 3f 40
"""
import sys, os, glob
import numpy as np
from collections import defaultdict

sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG = "/home/user/openpilot/drivelog"


def find_segments(route_hex):
  rh = route_hex.strip().lower().zfill(8)
  pattern = os.path.join(DRIVELOG, f"*_{rh}--*--rlog.zst")
  files = sorted(glob.glob(pattern))
  segs = {}
  for f in files:
    base = os.path.basename(f)
    parts = base.split('--')
    if len(parts) >= 3:
      seg_num = int(parts[-2])
      segs[seg_num] = f
  return segs


def analyze_segment(path):
  s = {
    'n_frames': 0, 'n_lat_active': 0, 'n_saturated': 0,
    'n_steer_required': 0, 'card_crashes': 0,
    'speed_min': 999, 'speed_max': 0,
    'angle_errs': [], 'desired_angles': [], 'actual_angles': [],
    'speeds': [], 'sat_events': [],
    'process_issues': [], 'adas_warnings': [],
    'dur': 0, 'alert_types': defaultdict(int),
    'sat_pct_by_speed': defaultdict(lambda: [0, 0]),
  }
  t0 = None
  last_saturated = False
  sat_start_t = None

  for msg in LogReader(path):
    t = msg.logMonoTime / 1e9
    if t0 is None:
      t0 = t
    rel_t = t - t0
    w = msg.which()

    if w == 'controlsState':
      cs = msg.controlsState
      s['n_frames'] += 1
      s['dur'] = rel_t

      lat = cs.lateralControlState
      lat_w = lat.which()
      if lat_w == 'angleState':
        ang = lat.angleState
        if ang.active:
          s['n_lat_active'] += 1
          desired = ang.steeringAngleDesiredDeg
          actual = ang.steeringAngleDeg
          err = abs(desired - actual)
          s['angle_errs'].append(err)
          s['desired_angles'].append(desired)
          s['actual_angles'].append(actual)

          v_kmh = cs.vEgo * 3.6 if hasattr(cs, 'vEgo') else 0
          s['speeds'].append(v_kmh)
          s['speed_min'] = min(s['speed_min'], v_kmh)
          s['speed_max'] = max(s['speed_max'], v_kmh)

          spd_bin = int(v_kmh // 20) * 20
          s['sat_pct_by_speed'][spd_bin][1] += 1

          if ang.saturated:
            s['n_saturated'] += 1
            s['sat_pct_by_speed'][spd_bin][0] += 1
            if not last_saturated:
              sat_start_t = rel_t
            last_saturated = True
          else:
            if last_saturated and sat_start_t is not None:
              s['sat_events'].append((sat_start_t, rel_t, v_kmh))
            last_saturated = False

      at = str(cs.alertType) if hasattr(cs, 'alertType') else ''
      if at:
        s['alert_types'][at] += 1
      if 'steerRequired' in at or 'promptDistracted' in at:
        s['n_steer_required'] += 1

    elif w == 'managerState':
      ms = msg.managerState
      for p in ms.processes:
        if p.name in ('card', 'selfdrived', 'controlsd') and not p.running:
          s['process_issues'].append((rel_t, str(p.name)))

    elif w == 'logMessage':
      m = str(msg.logMessage) if hasattr(msg, 'logMessage') else ''
      if 'crash' in m.lower() or 'exception' in m.lower():
        s['card_crashes'] += 1
      if 'ADAS' in m or 'fault' in m.lower():
        s['adas_warnings'].append((rel_t, m[:150]))

  return s


def main(route_hexes):
  for route_hex in route_hexes:
    rh = route_hex.strip().lower().zfill(8)
    segs = find_segments(rh)
    if not segs:
      print(f"  ⚠ Route {route_hex}: no segments found")
      continue

    print(f"\n{'='*90}")
    print(f"  Route 0x{rh[-2:]} — {len(segs)} segments")
    print(f"{'='*90}")

    total = defaultdict(int)
    all_errs = []
    all_sat_events = []
    all_process = []
    all_adas = []
    all_alerts = defaultdict(int)
    all_sat_by_speed = defaultdict(lambda: [0, 0])
    total_time = 0.0
    problem_segs = []

    for seg_num in sorted(segs.keys()):
      path = segs[seg_num]
      try:
        s = analyze_segment(path)
      except Exception as e:
        print(f"  seg {seg_num:3d}: ERROR {e}")
        continue

      dur = s['dur']
      total_time += dur
      total['frames'] += s['n_frames']
      total['lat_active'] += s['n_lat_active']
      total['saturated'] += s['n_saturated']
      total['steer_req'] += s['n_steer_required']
      total['crashes'] += s['card_crashes']
      all_errs.extend(s['angle_errs'])
      all_sat_events.extend([(seg_num, t0, t1, v) for t0, t1, v in s['sat_events']])
      all_process.extend([(seg_num, t, n) for t, n in s['process_issues']])
      all_adas.extend([(seg_num, t, m) for t, m in s['adas_warnings']])
      for k, v in s['alert_types'].items():
        all_alerts[k] += v
      for spd, (sat, tot) in s['sat_pct_by_speed'].items():
        all_sat_by_speed[spd][0] += sat
        all_sat_by_speed[spd][1] += tot

      flag = ""
      if s['n_steer_required'] > 0 or s['n_saturated'] > 20:
        flag = "⚠ "
        problem_segs.append(seg_num)
      if s['card_crashes'] > 0 or s['process_issues']:
        flag = "❌ "

      v_range = f"{s['speed_min']:.0f}-{s['speed_max']:.0f}" if s['speed_min'] < 999 else "?"
      sat_pct = 100 * s['n_saturated'] / max(s['n_lat_active'], 1)
      avg_err = np.mean(s['angle_errs']) if s['angle_errs'] else 0
      print(f"  {flag}seg {seg_num:3d}  {dur:5.0f}s  v={v_range:>10}km/h  "
            f"active={s['n_lat_active']:5d}  sat={s['n_saturated']:4d}({sat_pct:4.1f}%)  "
            f"steerReq={s['n_steer_required']:3d}  err={avg_err:.2f}°  crash={s['card_crashes']}")

    print(f"\n  {'─'*80}")
    print(f"  ROUTE TOTALS  ({total_time/60:.1f} min, {total_time:.0f}s)")
    print(f"    Frames:           {total['frames']}")
    print(f"    Lat active:       {total['lat_active']}")
    sat_pct = 100 * total['saturated'] / max(total['lat_active'], 1)
    print(f"    Saturated frames: {total['saturated']} ({sat_pct:.1f}% of active)")
    print(f"    SteerRequired:    {total['steer_req']}")
    print(f"    Card crashes:     {total['crashes']}")

    if all_errs:
      errs = np.array(all_errs)
      print(f"\n    Angle error (|desired - actual|):")
      print(f"      mean={np.mean(errs):.2f}°  p50={np.median(errs):.2f}°  "
            f"p95={np.percentile(errs,95):.2f}°  p99={np.percentile(errs,99):.2f}°  "
            f"max={np.max(errs):.2f}°")
      over = [(2.5, '>2.5°'), (5.0, '>5°'), (10.0, '>10°')]
      for thr, lbl in over:
        n = (errs > thr).sum()
        print(f"      {lbl}: {n} frames ({100*n/len(errs):.1f}%)")

    if all_sat_by_speed:
      print(f"\n    Saturation by speed band:")
      for spd in sorted(all_sat_by_speed.keys()):
        sat, tot = all_sat_by_speed[spd]
        if tot > 0:
          print(f"      {spd:3d}-{spd+20:3d} km/h:  {sat:5d}/{tot:5d} = {100*sat/tot:.1f}%")

    if all_alerts:
      print(f"\n    Alert types (top 10):")
      for k, v in sorted(all_alerts.items(), key=lambda x: -x[1])[:10]:
        print(f"      {v:6d}  {k}")

    if problem_segs:
      print(f"\n    Problem segs: {problem_segs}")

    if all_sat_events:
      print(f"\n    Saturation episodes: {len(all_sat_events)} total")
      long_evts = sorted(all_sat_events, key=lambda x: -(x[2]-x[1]))
      for seg, t0, t1, v in long_evts[:8]:
        print(f"      seg {seg:3d} t={t0:.1f}-{t1:.1f}s ({t1-t0:.1f}s) v≈{v:.0f}km/h")

    if all_process:
      print(f"\n    Process not running: {len(all_process)}")
      for seg, t, name in all_process[:10]:
        print(f"      seg {seg} t={t:.1f}s: {name}")

    if all_adas:
      print(f"\n    ADAS/fault warnings: {len(all_adas)}")
      for seg, t, m in all_adas[:10]:
        print(f"      seg {seg} t={t:.1f}s: {m[:100]}")

  return total


if __name__ == "__main__":
  routes = sys.argv[1:] if len(sys.argv) > 1 else ['3f', '40']
  main(routes)
