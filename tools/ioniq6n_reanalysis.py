#!/usr/bin/env python3
"""Objective re-analysis of all 214 segs across routes 00000028-0000002d.

Goals (post-81c451f master-plan re-evaluation):
  1. Mode mix (op / lfa_passthrough / lfa_override / manual) per route
  2. Tracking accuracy: MAE, p95 |desired-actual| per speed bucket × mode
  3. Low-speed tick: |Δdesired|>0.3° frame-to-frame rate in op mode <5 km/h
  4. Takeover alert rate: parse CCNC_0x161 ALERTS_2/3 from bus 2 (camera)
  5. Rate-limit clipping: fraction of frames where our rate cap bound the commanded angle
  6. Oscillation: zero-crossings/s of (desired - low_passed_desired)
  7. Stock-LFA baseline: same metrics for lfa_passthrough frames

Caches raw extract to /tmp/reanalysis_cache.pkl.
"""
import os
import sys
import glob
import pickle
import time
import numpy as np
import zstandard as zstd
from collections import defaultdict

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')

from cereal import log

DRIVELOG_DIR = '/home/user/openpilot/drivelog'
CACHE = '/tmp/reanalysis_cache.pkl'

# Bus 2 = camera on HDA2-ALT (same bus the openpilot-injected LKAS_ALT mirrors)
# Addresses we care about (from opendbc dbc and commit logs):
#   0x110 (272)  LKAS_ALT         — camera's angle request (ADAS_StrAnglReqVal, ACIAnglTqRedcGainVal)
#   0x161 (353)  CCNC_0x161       — alerts (ALERTS_2, ALERTS_3, ALERTS_5, SOUNDS_2/4)
ADDR_LKAS_ALT = 0x110
ADDR_CCNC_161 = 0x161

# Hyundai CAN FD LKAS_ALT signal offsets (from opendbc DBC, 32-byte message)
# Populated by reverse-engineering — we only need a rough decode for alerts.


def _decode_u16_le(data: bytes, start: int) -> int:
  return data[start] | (data[start + 1] << 8)


def _decode_adas_str_angle_req(data: bytes) -> float:
  """ADAS_StrAnglReqVal: signal @ byte 8-9, little-endian, 0.1 factor, offset -3276.8"""
  raw = _decode_u16_le(data, 8)
  if raw & 0x8000:
    raw -= 0x10000
  return raw * 0.1


def _decode_aci_gain(data: bytes) -> float:
  """ADAS_ACIAnglTqRedcGainVal: signal @ byte 6 low nibble, factor 1/15, range 0..1"""
  return (data[6] & 0x0F) / 15.0


def extract_frames(path):
  """Extract per-100Hz snapshot: carState+carControl, plus camera-side bus2 LKAS_ALT."""
  with open(path, 'rb') as f:
    raw = zstd.ZstdDecompressor().decompress(f.read(), max_output_size=500 * 1024 * 1024)

  # We sync on carControl (100 Hz). For each CC, we attach the most recent
  # carState and the most recent bus-2 LKAS_ALT / CCNC_0x161 decodes.
  latest_cs = None
  latest_cam_angle = None
  latest_cam_aci = None
  latest_alerts2 = 0
  latest_alerts3 = 0
  git_commit = None
  git_branch = None

  frames = []
  for msg in log.Event.read_multiple_bytes(raw):
    w = msg.which()
    if w == 'initData':
      git_commit = msg.initData.gitCommit
      git_branch = msg.initData.gitBranch
    elif w == 'carState':
      latest_cs = msg.carState
    elif w == 'can':
      for c in msg.can:
        if c.src != 2:
          continue
        if c.address == ADDR_LKAS_ALT:
          d = bytes(c.dat)
          if len(d) >= 10:
            latest_cam_angle = _decode_adas_str_angle_req(d)
            latest_cam_aci = _decode_aci_gain(d)
        elif c.address == ADDR_CCNC_161:
          d = bytes(c.dat)
          # ALERTS_2 and ALERTS_3 are nibble-packed in byte 3-5 range on most variants.
          # We conservatively scan for the values we care about.
          # The real decode relies on DBC but we only need to count non-zero alerts.
          if len(d) >= 8:
            # Heuristic: lower nibble of byte 3 = ALERTS_2, upper = ALERTS_3 (per 611a505 analysis)
            latest_alerts2 = d[3] & 0x0F
            latest_alerts3 = (d[3] >> 4) & 0x0F
    elif w == 'carControl' and latest_cs is not None:
      cc = msg.carControl
      frames.append({
        'v_kmh': latest_cs.vEgoRaw * 3.6,
        'v_ms': latest_cs.vEgoRaw,
        'actual': latest_cs.steeringAngleDeg,
        'desired': cc.actuators.steeringAngleDeg,
        'curvature': cc.actuators.curvature,
        'lat_active': cc.latActive,
        'cruise_on': latest_cs.cruiseState.enabled,
        'pressed_torque': abs(latest_cs.steeringTorque),
        'steering_pressed': latest_cs.steeringPressed,
        'blinker_l': latest_cs.leftBlinker,
        'blinker_r': latest_cs.rightBlinker,
        'cam_angle': latest_cam_angle,
        'cam_aci': latest_cam_aci,
        'alert2': latest_alerts2,
        'alert3': latest_alerts3,
      })
  return frames, git_commit, git_branch


def load_or_cache():
  if os.path.exists(CACHE):
    print(f"Loading cache from {CACHE}…")
    with open(CACHE, 'rb') as f:
      return pickle.load(f)

  files = sorted(glob.glob(os.path.join(DRIVELOG_DIR, '*rlog.zst')))
  print(f"Processing {len(files)} segments…")
  all_frames = []
  by_route = defaultdict(lambda: {'segs': 0, 'commit': None, 'branch': None})
  t0 = time.time()
  for i, path in enumerate(files):
    fn = os.path.basename(path)
    parts = fn.split('--')
    route_id = parts[0].split('_')[1]
    seg = int(parts[2])
    try:
      frames, git_commit, git_branch = extract_frames(path)
    except Exception as e:
      print(f"  ERR {fn}: {e}")
      continue
    for f in frames:
      f['route'] = route_id
      f['seg'] = seg
    all_frames.extend(frames)
    by_route[route_id]['segs'] += 1
    if git_commit:
      by_route[route_id]['commit'] = git_commit[:7]
      by_route[route_id]['branch'] = git_branch
    if (i + 1) % 20 == 0:
      dt = time.time() - t0
      print(f"  {i+1}/{len(files)}  {len(all_frames):,} frames  {dt:.1f}s")

  print(f"\nTotal: {len(all_frames):,} frames in {time.time()-t0:.1f}s")
  for r, info in sorted(by_route.items()):
    print(f"  Route {r}: {info['segs']} segs  commit={info['commit']}  branch={info['branch']}")

  with open(CACHE, 'wb') as f:
    pickle.dump((all_frames, dict(by_route)), f)
  return all_frames, dict(by_route)


# ──────────── Analysis helpers ────────────

BUCKETS = [
  ('parking', 0, 10),
  ('20km/h', 10, 25),
  ('30km/h', 25, 35),
  ('40km/h', 35, 45),
  ('50km/h', 45, 55),
  ('60-70', 55, 75),
  ('80-90', 75, 95),
  ('100+', 95, 200),
]


def classify_mode(f):
  if f['pressed_torque'] > 200:
    return 'manual'
  if f['cruise_on'] and f['lat_active']:
    return 'op'
  if f['cruise_on'] and not f['lat_active']:
    return 'lfa_override'
  if not f['cruise_on'] and f['pressed_torque'] < 150:
    return 'lfa_passthrough'
  return 'manual'


def bucket_of(v_kmh):
  for name, lo, hi in BUCKETS:
    if lo <= v_kmh < hi:
      return name
  return None


def report_mode_mix(frames, by_route):
  print("\n=== 1. Mode distribution per route ===")
  print(f"{'Route':<10} {'commit':<8} {'segs':>5} {'op%':>6} {'lfa_pass%':>10} "
        f"{'lfa_over%':>10} {'manual%':>8}  frames")
  per_route = defaultdict(lambda: defaultdict(int))
  for f in frames:
    per_route[f['route']][classify_mode(f)] += 1
  for r, d in sorted(per_route.items()):
    total = sum(d.values())
    commit = by_route[r]['commit']
    segs = by_route[r]['segs']
    print(f"{r:<10} {commit:<8} {segs:>5} "
          f"{100*d['op']/total:>5.1f}% "
          f"{100*d['lfa_passthrough']/total:>9.1f}% "
          f"{100*d['lfa_override']/total:>9.1f}% "
          f"{100*d['manual']/total:>7.1f}% "
          f"{total:>8,}")


def report_speed_coverage(frames):
  print("\n=== 2. Speed × mode coverage (minutes) ===")
  per_bucket = defaultdict(lambda: defaultdict(int))
  for f in frames:
    b = bucket_of(f['v_kmh'])
    if b is None:
      continue
    per_bucket[b][classify_mode(f)] += 1
  print(f"{'Bucket':<9} {'op_min':>7} {'lfa_min':>7} {'manual':>7} {'total':>7}")
  for name, _, _ in BUCKETS:
    d = per_bucket[name]
    op_min = d['op'] / 6000
    lfa_min = d['lfa_passthrough'] / 6000
    man_min = d['manual'] / 6000
    total_min = sum(d.values()) / 6000
    if total_min < 0.5:
      continue
    print(f"{name:<9} {op_min:>6.1f} {lfa_min:>6.1f} {man_min:>6.1f} {total_min:>6.1f}")


def report_tracking_accuracy(frames):
  print("\n=== 3. Tracking accuracy (|desired-actual|) per bucket × mode ===")
  g = defaultdict(list)
  for f in frames:
    b = bucket_of(f['v_kmh'])
    if b is None:
      continue
    mode = classify_mode(f)
    if mode not in ('op', 'lfa_passthrough'):
      continue
    err = abs(f['desired'] - f['actual']) if f['desired'] is not None else None
    if err is None:
      continue
    # For lfa_passthrough, the "desired" from cc.actuators is still openpilot's
    # planner output — reflects what op WOULD command, not what's actually sent.
    # Use cam_angle when available for LFA accuracy (camera commanded vs actual).
    if mode == 'lfa_passthrough' and f['cam_angle'] is not None:
      err_lfa = abs(f['cam_angle'] - f['actual'])
      g[('lfa_cam', b)].append(err_lfa)
    g[(mode, b)].append(err)
  print(f"{'Mode':<12} {'Bucket':<9} {'n':>7} {'MAE':>6} {'p50':>6} {'p95':>6} {'p99':>6} {'max':>6}")
  for mode in ('op', 'lfa_passthrough', 'lfa_cam'):
    for name, _, _ in BUCKETS:
      e = np.array(g.get((mode, name), []))
      if len(e) < 500:
        continue
      print(f"{mode:<12} {name:<9} {len(e):>7,} "
            f"{e.mean():>5.2f}° {np.median(e):>5.2f}° "
            f"{np.percentile(e,95):>5.2f}° {np.percentile(e,99):>5.2f}° "
            f"{e.max():>5.1f}°")


def report_low_speed_tick(frames):
  print("\n=== 4. Low-speed tick rate: |Δdesired|>THR in op mode <5 km/h ===")
  # Replicate 81c451f commit-body metric: 14.3% of op<5 km/h frames had |Δdes|>0.3°
  prev_by_seg = {}
  buckets = {'<2 km/h': (0, 2), '2-5 km/h': (2, 5), '5-10 km/h': (5, 10)}
  results = defaultdict(lambda: defaultdict(int))
  for f in frames:
    key = (f['route'], f['seg'])
    prev = prev_by_seg.get(key)
    prev_by_seg[key] = f
    if prev is None:
      continue
    if classify_mode(f) != 'op' or classify_mode(prev) != 'op':
      continue
    v = f['v_kmh']
    for name, (lo, hi) in buckets.items():
      if lo <= v < hi:
        delta = abs((f['desired'] or 0) - (prev['desired'] or 0))
        results[f['route']]['_count_' + name] += 1
        if delta > 0.3:
          results[f['route']]['gt0_3_' + name] += 1
        if delta > 1.0:
          results[f['route']]['gt1_0_' + name] += 1
        break

  print(f"{'Route':<10} {'bucket':<10} {'n':>8} {'|Δ|>0.3°':>10} {'|Δ|>1.0°':>10}")
  for r in sorted(results.keys()):
    for name in buckets:
      n = results[r].get('_count_' + name, 0)
      if n < 50:
        continue
      g0 = results[r].get('gt0_3_' + name, 0)
      g1 = results[r].get('gt1_0_' + name, 0)
      print(f"{r:<10} {name:<10} {n:>8,} "
            f"{100*g0/n:>8.1f}% {100*g1/n:>8.1f}%")


def report_takeover_alerts(frames):
  print("\n=== 5. Takeover alert rate (CCNC_0x161 ALERTS_2/3, bus 2 heuristic decode) ===")
  per_route_mode = defaultdict(lambda: defaultdict(lambda: {'total': 0, 'a2_hands': 0, 'a3_hdp': 0}))
  for f in frames:
    mode = classify_mode(f)
    per_route_mode[f['route']][mode]['total'] += 1
    if f['alert2'] in (1, 2):
      per_route_mode[f['route']][mode]['a2_hands'] += 1
    if f['alert3'] in (11, 12):
      per_route_mode[f['route']][mode]['a3_hdp'] += 1

  print(f"{'Route':<10} {'mode':<16} {'frames':>8} "
        f"{'ALERTS_2{1,2}%':>14} {'ALERTS_3{11,12}%':>16}")
  for r in sorted(per_route_mode.keys()):
    for mode in ('op', 'lfa_passthrough', 'manual'):
      d = per_route_mode[r][mode]
      if d['total'] < 500:
        continue
      print(f"{r:<10} {mode:<16} {d['total']:>8,} "
            f"{100*d['a2_hands']/d['total']:>12.2f}% "
            f"{100*d['a3_hdp']/d['total']:>14.2f}%")


def report_oscillation(frames):
  print("\n=== 6. Oscillation: direction-change rate of apply_angle (op mode only) ===")
  # Per-segment contiguous op-mode sequences; count desired-angle sign changes of Δ
  g = defaultdict(lambda: {'changes': 0, 'frames': 0})
  prev = {}
  prev_delta = {}
  for f in frames:
    key = (f['route'], f['seg'])
    m = classify_mode(f)
    p = prev.get(key)
    prev[key] = f
    if p is None or m != 'op' or classify_mode(p) != 'op':
      prev_delta[key] = None
      continue
    d = (f['desired'] or 0) - (p['desired'] or 0)
    pd = prev_delta.get(key)
    prev_delta[key] = d
    if pd is None:
      continue
    if pd * d < 0:
      g[f['route']]['changes'] += 1
    g[f['route']]['frames'] += 1

  print(f"{'Route':<10} {'op_frames':>10} {'dir_changes':>12} {'rate_Hz':>8}")
  for r in sorted(g.keys()):
    d = g[r]
    if d['frames'] < 500:
      continue
    # Each frame = 10 ms; direction change rate in Hz
    rate = d['changes'] / (d['frames'] * 0.01)
    print(f"{r:<10} {d['frames']:>10,} {d['changes']:>12,} {rate:>7.1f}")


def report_rate_clip(frames):
  print("\n=== 7. Rate-limit clipping: does our cap bind commanded angle? ===")
  # Simulate: if unclipped |Δdesired| exceeds our ANGLE_LIMITS table, flag as clipped.
  # (We use the actual table from values.py.)
  ANGLE_LIMITS_UP = ([0., 3., 7., 12., 18., 25., 30.], [0.6, 0.9, 1.3, 1.0, 0.6, 0.4, 0.25])
  ANGLE_LIMITS_DN = ([0., 3., 7., 12., 18., 25., 30.], [0.8, 1.1, 1.5, 1.2, 0.75, 0.55, 0.35])

  def interp(v, xs, ys):
    return float(np.interp(v, xs, ys))

  g = defaultdict(lambda: defaultdict(int))
  prev_by_seg = {}
  for f in frames:
    key = (f['route'], f['seg'])
    p = prev_by_seg.get(key)
    prev_by_seg[key] = f
    if p is None or classify_mode(f) != 'op' or classify_mode(p) != 'op':
      continue
    b = bucket_of(f['v_kmh'])
    if b is None:
      continue
    delta = (f['desired'] or 0) - (p['desired'] or 0)
    limit = interp(f['v_ms'], *ANGLE_LIMITS_UP) if delta > 0 else interp(f['v_ms'], *ANGLE_LIMITS_DN)
    # Message sent every 2 frames (50 Hz), so per-20ms limit. Our 10 ms Δ should be ≤ limit/2.
    per_frame_limit = limit / 2
    if abs(delta) > per_frame_limit * 0.99:
      g[b]['clipped'] += 1
    g[b]['total'] += 1

  print(f"{'Bucket':<9} {'op_frames':>10} {'clipped':>8} {'clip%':>6}")
  for name, _, _ in BUCKETS:
    d = g[name]
    if d['total'] < 200:
      continue
    print(f"{name:<9} {d['total']:>10,} {d['clipped']:>8,} {100*d['clipped']/d['total']:>5.1f}%")


def report_stock_lfa_low_speed(frames):
  print("\n=== 8. Stock LFA low-speed behavior (cam_aci gain, below 15 km/h) ===")
  # What does stock LFA (lfa_passthrough) do at low speed? Does it engage? Back off?
  # cam_aci = ADAS_ACIAnglTqRedcGainVal as commanded by the camera (0..1)
  g = defaultdict(list)
  for f in frames:
    m = classify_mode(f)
    if m != 'lfa_passthrough' or f['cam_aci'] is None:
      continue
    v = f['v_kmh']
    key = 'parking' if v < 3 else ('creep' if v < 10 else ('15' if v < 15 else 'normal'))
    g[key].append(f['cam_aci'])
  print(f"{'bucket':<10} {'n':>8} {'aci_mean':>9} {'aci_p50':>8} {'aci_p95':>8} {'aci_max':>8}")
  for k in ('parking', 'creep', '15', 'normal'):
    arr = np.array(g.get(k, []))
    if len(arr) < 500:
      continue
    print(f"{k:<10} {len(arr):>8,} {arr.mean():>9.3f} "
          f"{np.percentile(arr,50):>8.3f} {np.percentile(arr,95):>8.3f} {arr.max():>8.3f}")


def main():
  frames, by_route = load_or_cache()
  report_mode_mix(frames, by_route)
  report_speed_coverage(frames)
  report_tracking_accuracy(frames)
  report_low_speed_tick(frames)
  report_takeover_alerts(frames)
  report_oscillation(frames)
  report_rate_clip(frames)
  report_stock_lfa_low_speed(frames)


if __name__ == '__main__':
  main()
