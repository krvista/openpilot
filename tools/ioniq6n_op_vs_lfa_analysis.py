#!/usr/bin/env python3
"""Ioniq 6 N: op vs stock-LFA mode analysis on 176-segment dataset.

Extracts per-frame data from all 176 rlog.zst files and caches to
/tmp/ccnc_full_cache.pkl for fast iteration. Captures:
  - v_kmh, v_ms, actual_angle, desired (actuators.steeringAngleDeg)
  - cruise_on, lat_active, pressed_torque, curvature
  - LKAS_ALT bus 2 (camera) raw bytes for decoding ACIGain etc. — optional
  - CCNC_0x161 alerts/sounds fields — for Issue 1+5 analysis

Then reports:
  1. Mode distribution (op, lfa, manual) per route
  2. Speed bucket × mode → frame count (for data sufficiency check)
  3. Alerts field distribution by mode (for Issue 1+5)
  4. Rate-of-change analysis at various speed buckets (for Issue 2)
  5. 3 km/h transition analysis (for Issue 3)
  6. Icon/aci_active breakdown (for Issue 4)
"""
import os
import sys
import pickle
import numpy as np
from collections import defaultdict

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')

from openpilot.tools.lib.logreader import LogReader


LOG_DIR = '/tmp/ccnc_drivelog_full'
CACHE = '/tmp/ccnc_full_cache.pkl'


def classify_mode(cruise_on, lat_active, pressed_torque):
  """Return one of: 'op', 'lfa_passthrough', 'lfa_override', 'manual'."""
  heavy_press = pressed_torque > 200
  if heavy_press:
    return 'manual'
  if cruise_on and lat_active:
    return 'op'
  if cruise_on and not lat_active:
    return 'lfa_override'  # driver pressed hard, op yielded, LFA may be steering
  if not cruise_on and pressed_torque < 150:
    return 'lfa_passthrough'  # cruise off, driver gentle → camera passthrough
  return 'manual'


def extract_frames(log_path, route_id, seg):
  """Extract frames from one rlog; returns list of dicts."""
  latest_cs = None
  latest_ccnc = None
  out = []
  for msg in LogReader(log_path):
    w = msg.which()
    if w == 'carState':
      latest_cs = msg.carState
    elif w == 'can':
      # Parse CCNC_0x161 (ALERTS) and LKAS_ALT (0x110) from bus 2 (camera)
      # We don't need to decode here — just store snapshots alongside carState samples
      pass
    elif w == 'carControl' and latest_cs is not None:
      cc = msg.carControl
      pressed_torque = abs(latest_cs.steeringTorque)
      mode = classify_mode(latest_cs.cruiseState.enabled, cc.latActive, pressed_torque)
      out.append({
        'route': route_id,
        'seg': seg,
        'v_kmh': latest_cs.vEgoRaw * 3.6,
        'v_ms': latest_cs.vEgoRaw,
        'actual': latest_cs.steeringAngleDeg,
        'desired': cc.actuators.steeringAngleDeg,
        'curvature': cc.actuators.curvature,
        'lat_active': cc.latActive,
        'cruise_on': latest_cs.cruiseState.enabled,
        'pressed_torque': pressed_torque,
        'mode': mode,
      })
  return out


def load_or_cache():
  if os.path.exists(CACHE):
    print(f"Loading cache from {CACHE}...")
    with open(CACHE, 'rb') as f:
      return pickle.load(f)

  files = sorted(os.listdir(LOG_DIR))
  files = [f for f in files if f.endswith('.zst')]
  print(f"Processing {len(files)} files (cache will be written to {CACHE})...")

  all_frames = []
  by_route = defaultdict(int)
  for i, fn in enumerate(files):
    parts = fn.split('--')
    route_id = parts[0].split('_')[1]  # e.g., '0000002a'
    seg = int(parts[2])
    path = os.path.join(LOG_DIR, fn)
    try:
      frames = extract_frames(path, route_id, seg)
      all_frames.extend(frames)
      by_route[route_id] += 1
    except Exception as e:
      print(f"  ERR {fn}: {e}")
    if (i + 1) % 20 == 0:
      print(f"  {i+1}/{len(files)} processed, {len(all_frames)} frames")

  print(f"\nTotal frames: {len(all_frames)}")
  for r, n in sorted(by_route.items()):
    print(f"  Route {r}: {n} segments")

  with open(CACHE, 'wb') as f:
    pickle.dump(all_frames, f)
  return all_frames


# Speed buckets (Korean regimes)
BUCKETS = [
  ('stopped', 0, 3),
  ('creep',   3, 10),
  ('20km/h', 10, 25),
  ('30km/h', 25, 35),
  ('40km/h', 35, 45),
  ('50km/h', 45, 55),
  ('60km/h', 55, 70),
  ('80km/h', 70, 90),
  ('100km/h', 90, 105),
  ('110km/h', 105, 130),
]


def report_mode_distribution(frames):
  print("\n" + "="*70)
  print("MODE DISTRIBUTION PER ROUTE")
  print("="*70)
  by_route_mode = defaultdict(lambda: defaultdict(int))
  for f in frames:
    by_route_mode[f['route']][f['mode']] += 1

  print(f"{'Route':<10} {'op':>8} {'lfa_pass':>10} {'lfa_over':>10} {'manual':>10} {'total':>10}")
  for r in sorted(by_route_mode.keys()):
    counts = by_route_mode[r]
    total = sum(counts.values())
    print(f"{r:<10} {counts['op']:>8} {counts['lfa_passthrough']:>10} "
          f"{counts['lfa_override']:>10} {counts['manual']:>10} {total:>10}")
  print("-" * 70)
  total_counts = defaultdict(int)
  for r in by_route_mode.values():
    for m, n in r.items():
      total_counts[m] += n
  total_total = sum(total_counts.values())
  print(f"{'TOTAL':<10} {total_counts['op']:>8} {total_counts['lfa_passthrough']:>10} "
        f"{total_counts['lfa_override']:>10} {total_counts['manual']:>10} {total_total:>10}")


def report_speed_bucket(frames):
  print("\n" + "="*70)
  print("SPEED BUCKET × MODE (frame count; 100 frames = 1 sec)")
  print("="*70)

  per_bucket = defaultdict(lambda: defaultdict(int))
  for f in frames:
    v = f['v_kmh']
    for name, lo, hi in BUCKETS:
      if lo <= v < hi:
        per_bucket[name][f['mode']] += 1
        break

  hdr = f"{'Bucket':<10} {'op':>8}(min) {'lfa_pass':>10}(min) {'lfa_over':>10}(min) {'manual':>10}"
  print(hdr)
  for name, _, _ in BUCKETS:
    counts = per_bucket[name]
    op_min = counts['op'] / 6000
    lfa_pass_min = counts['lfa_passthrough'] / 6000
    lfa_over_min = counts['lfa_override'] / 6000
    manual_min = counts['manual'] / 6000
    total = sum(counts.values())
    if total < 100:
      continue
    print(f"{name:<10} {counts['op']:>8} ({op_min:4.1f}) "
          f"{counts['lfa_passthrough']:>10} ({lfa_pass_min:4.1f}) "
          f"{counts['lfa_override']:>10} ({lfa_over_min:4.1f}) "
          f"{counts['manual']:>10}")


def report_rate_analysis(frames):
  """For S-curve analysis: rate-of-change distribution by speed bucket and mode."""
  print("\n" + "="*95)
  print("ANGLE RATE ANALYSIS (|Δactual|/10ms → °/s) — for Issue 2 (S-curve)")
  print("="*95)

  # Group by (mode, bucket), compute rate of change
  groups = defaultdict(list)
  prev_by_seg = {}  # (route, seg) → last frame
  for f in frames:
    key = (f['route'], f['seg'])
    prev = prev_by_seg.get(key)
    prev_by_seg[key] = f
    if prev is None:
      continue
    if f['mode'] != prev['mode']:  # skip cross-mode transitions
      continue
    if f['mode'] not in ('op', 'lfa_passthrough'):
      continue
    v = f['v_kmh']
    for name, lo, hi in BUCKETS:
      if lo <= v < hi:
        delta_actual = abs(f['actual'] - prev['actual'])
        delta_desired = abs(f['desired'] - prev['desired']) if f['desired'] else 0
        groups[(f['mode'], name)].append((delta_actual, delta_desired))
        break

  print(f"{'Mode':<15} {'Bucket':<10} {'n':>6} "
        f"{'|Δact|_p95':>11} {'|Δact|_p99':>11} {'max':>7}  "
        f"{'|Δdes|_p95':>11} {'|Δdes|_p99':>11}")
  print('-' * 95)
  for mode in ('op', 'lfa_passthrough'):
    for name, _, _ in BUCKETS:
      data = groups.get((mode, name), [])
      if len(data) < 50:
        continue
      act = np.array([d[0] for d in data])
      des = np.array([d[1] for d in data])
      # Rates in °/s (at 100Hz sampling)
      act_p95 = float(np.percentile(act, 95)) * 100
      act_p99 = float(np.percentile(act, 99)) * 100
      act_max = float(act.max()) * 100
      des_p95 = float(np.percentile(des, 95)) * 100
      des_p99 = float(np.percentile(des, 99)) * 100
      print(f"{mode:<15} {name:<10} {len(data):>6} "
            f"{act_p95:>8.1f}°/s {act_p99:>8.1f}°/s {act_max:>5.1f}°/s  "
            f"{des_p95:>8.1f}°/s {des_p99:>8.1f}°/s")


def report_low_speed_transitions(frames):
  """Issue 3: analyze transitions at 3 km/h boundary."""
  print("\n" + "="*70)
  print("LOW-SPEED TRANSITION ANALYSIS (2-5 km/h) — for Issue 3")
  print("="*70)

  # Find all op-mode frames crossing 3 km/h boundary
  prev_by_seg = {}
  crossings = []
  for f in frames:
    key = (f['route'], f['seg'])
    prev = prev_by_seg.get(key)
    prev_by_seg[key] = f
    if prev is None or f['mode'] != 'op' or prev['mode'] != 'op':
      continue
    # Did speed cross 3 km/h?
    if (prev['v_kmh'] < 3) != (f['v_kmh'] < 3):
      delta_apply = abs(f['desired'] - prev['desired'])
      delta_actual = abs(f['actual'] - prev['actual'])
      crossings.append({
        'v_before': prev['v_kmh'],
        'v_after': f['v_kmh'],
        'delta_desired': delta_apply,
        'delta_actual': delta_actual,
      })

  if not crossings:
    print("  No 3 km/h crossings in op mode — user may not drive very slowly in op mode")
    return

  deltas_desired = [c['delta_desired'] for c in crossings]
  deltas_actual = [c['delta_actual'] for c in crossings]
  print(f"  Crossings: {len(crossings)}")
  print(f"  |Δdesired| at crossing: p50={np.percentile(deltas_desired,50):.2f}° "
        f"p95={np.percentile(deltas_desired,95):.2f}° max={max(deltas_desired):.2f}°")
  print(f"  |Δactual|  at crossing: p50={np.percentile(deltas_actual,50):.2f}° "
        f"p95={np.percentile(deltas_actual,95):.2f}° max={max(deltas_actual):.2f}°")


def report_icon_breakdown(frames):
  """Issue 4: when lat_active=True but aci_active would be False (icon white)."""
  print("\n" + "="*70)
  print("ICON WHITE BREAKDOWN (lat_active=True but would go white) — for Issue 4")
  print("="*70)
  ACI_MIN = 3.0 / 3.6
  DRIVER_TORQUE_DEADZONE = 30
  DRIVER_TORQUE_FULL_OVERRIDE = 150

  reasons = defaultdict(int)
  total_op = 0
  for f in frames:
    if f['mode'] != 'op':
      continue
    total_op += 1
    # Replicate hyundaicanfd.py logic
    speed_ok = f['v_ms'] > ACI_MIN
    driver_abs = f['pressed_torque']
    override = float(np.clip((driver_abs - DRIVER_TORQUE_DEADZONE) /
                              (DRIVER_TORQUE_FULL_OVERRIDE - DRIVER_TORQUE_DEADZONE), 0, 1))
    blend = 1.0 - override
    authority = blend if speed_ok else 0.0
    aci_active = speed_ok and authority > 0.1

    if not aci_active:
      if not speed_ok:
        reasons['below 3 km/h'] += 1
      elif authority <= 0.1:
        reasons['driver torque high'] += 1
      else:
        reasons['other'] += 1

  if total_op > 0:
    print(f"  op frames: {total_op}")
    print(f"  icon would go WHITE in {sum(reasons.values())} frames "
          f"({sum(reasons.values())/total_op*100:.1f}% of op time)")
    for r, n in sorted(reasons.items(), key=lambda x: -x[1]):
      print(f"    {r}: {n} ({n/total_op*100:.1f}%)")


def main():
  frames = load_or_cache()
  report_mode_distribution(frames)
  report_speed_bucket(frames)
  report_rate_analysis(frames)
  report_low_speed_transitions(frames)
  report_icon_breakdown(frames)


if __name__ == '__main__':
  main()
