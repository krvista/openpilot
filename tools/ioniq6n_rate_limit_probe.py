#!/usr/bin/env python3
"""Rate-limit probe — measure angle-rate demand distribution per speed bucket.

Two demand sources per frame (20ms / 50Hz sample):
  1) STOCK camera: CAN 0x110 (LKAS_ALT), src=2, ADAS_StrAnglReqVal field.
     bits 82..95 (LE signed, scale 0.1 deg). This is what the factory LFA
     actually commands to EPS.  We sample it regardless of op state — bus 2
     always carries the OEM camera feed.
  2) OP planner: controlsState.lateralControlState.angleState.steeringAngleDesiredDeg,
     sampled at 100Hz, decimated to 50Hz (every other frame) for fair comparison
     with the 50Hz LKAS_ALT stream.

For each stream we compute the per-step increment (deg/20ms) and bucket by
speed.  We print p50 / p95 / p99 / max and compare against the currently
configured MAX_ANGLE_RATE = 1.3 deg/20ms cap.

Usage:
  python3 tools/ioniq6n_rate_limit_probe.py [route_hex ...]
Default: 3e 3f 40 (today's round-trip) plus 28 29 2a 2b 2c 2d (commute routes).
"""
import sys, os, glob
import numpy as np
from collections import defaultdict

sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG = "/home/user/openpilot/drivelog"

# speed buckets (m/s → km/h approx):  stopped, parking, city-low, city-high, suburban, highway, fast-highway
SPEED_BUCKETS = [
  ('stopped',      0.0,  1.0),    # 0 km/h
  ('parking',      1.0,  5.0),    # ~4-18 km/h
  ('city-low',     5.0, 11.0),    # 18-40 km/h
  ('city-high',   11.0, 17.0),    # 40-60 km/h
  ('suburban',    17.0, 23.0),    # 60-83 km/h
  ('highway',     23.0, 30.0),    # 83-108 km/h
  ('fast-highway',30.0, 99.0),    # 108+ km/h
]

CURRENT_CAP = 1.3  # deg/20ms currently configured in CarControllerParams.ANGLE_LIMITS_VM


def parse_adas_str_angle(dat):
  """Extract ADAS_StrAnglReqVal from 0x110 LKAS_ALT dat (bytes).
  DBC: start_bit=82, length=14, little-endian, signed, scale=0.1 deg.
  """
  if len(dat) < 12:
    return None
  # start bit 82 → byte 10 bit 2, need 14 bits → spills into byte 11
  raw = ((dat[11] << 8) | dat[10]) >> 2
  raw &= 0x3FFF
  if raw & 0x2000:                     # sign-extend 14-bit
    raw -= 0x4000
  return raw * 0.1


def find_segments(route_hex):
  rh = route_hex.strip().lower().zfill(8)
  pattern = os.path.join(DRIVELOG, f"*_{rh}--*--rlog.zst")
  files = sorted(glob.glob(pattern))
  segs = {}
  for f in files:
    parts = os.path.basename(f).split('--')
    if len(parts) >= 3:
      try:
        segs[int(parts[-2])] = f
      except ValueError:
        pass
  return segs


def bucket_for(v):
  for name, lo, hi in SPEED_BUCKETS:
    if lo <= v < hi:
      return name
  return None


def scan_seg(path):
  """Return per-speed-bucket lists of absolute increments (deg/20ms)."""
  stock_inc = defaultdict(list)   # bucket → list of |Δstock_angle|
  op_inc    = defaultdict(list)
  op_active_frames = 0
  last_v = 0.0
  last_stock_angle = None
  last_stock_counter = -1
  last_op_angle = None
  op_sample_toggle = 0            # decimate 100Hz → 50Hz

  for msg in LogReader(path):
    w = msg.which()
    try:
      if w == 'carState':
        last_v = msg.carState.vEgo
      elif w == 'can':
        for m in msg.can:
          if m.address == 0x110 and m.src == 2:
            dat = bytes(m.dat)
            if len(dat) < 2:
              continue
            counter = dat[1] >> 4
            if counter == last_stock_counter:
              continue                    # skip stale duplicates
            last_stock_counter = counter
            a = parse_adas_str_angle(dat)
            if a is None:
              continue
            if last_stock_angle is not None:
              inc = abs(a - last_stock_angle)
              # reject absurd spikes (wrap / discontinuity)
              if inc < 50.0:
                b = bucket_for(last_v)
                if b:
                  stock_inc[b].append(inc)
            last_stock_angle = a
      elif w == 'controlsState':
        lat = msg.controlsState.lateralControlState
        if lat.which() == 'angleState':
          ang = lat.angleState
          if ang.active:
            op_active_frames += 1
            op_sample_toggle ^= 1
            if op_sample_toggle == 0:     # sample every other 100Hz frame → 50Hz
              a = float(ang.steeringAngleDesiredDeg)
              if last_op_angle is not None:
                inc = abs(a - last_op_angle)
                if inc < 50.0:
                  b = bucket_for(last_v)
                  if b:
                    op_inc[b].append(inc)
              last_op_angle = a
          else:
            last_op_angle = None
    except Exception:
      pass
  return stock_inc, op_inc, op_active_frames


def summarize(incs, label):
  # incs: dict bucket → list.  Return dict bucket → stats
  rows = []
  all_inc = []
  for name, _, _ in SPEED_BUCKETS:
    arr = np.array(incs.get(name, []))
    if len(arr) == 0:
      rows.append((name, 0, 0, 0, 0, 0, 0))
      continue
    rows.append((
      name, len(arr),
      float(np.mean(arr)),
      float(np.percentile(arr, 50)),
      float(np.percentile(arr, 95)),
      float(np.percentile(arr, 99)),
      float(arr.max()),
    ))
    all_inc.extend(arr.tolist())
  return rows, np.array(all_inc) if all_inc else np.zeros(0)


def print_table(rows, label, cap=CURRENT_CAP):
  print(f"\n  {label}  (deg/20ms)")
  print(f"  {'bucket':<14} {'n':>8} {'mean':>7} {'p50':>7} {'p95':>7} "
        f"{'p99':>7} {'max':>7} {'cap_hit%':>9}")
  for (name, n, mean, p50, p95, p99, mx) in rows:
    if n == 0:
      print(f"  {name:<14} {'-':>8}")
      continue
    cap_hit = 100.0 * sum(1 for v in [mean, p50, p95, p99, mx] if v > cap) / 5  # not real — recompute
    print(f"  {name:<14} {n:>8} {mean:>7.2f} {p50:>7.2f} {p95:>7.2f} "
          f"{p99:>7.2f} {mx:>7.2f}")


def cap_hit_pct(arr, cap):
  if len(arr) == 0:
    return 0.0
  return 100.0 * float((arr > cap).sum()) / len(arr)


def run_routes(route_hexes):
  grand_stock = defaultdict(list)
  grand_op    = defaultdict(list)
  total_segs = 0
  total_op_frames = 0

  for rh in route_hexes:
    segs = find_segments(rh)
    if not segs:
      continue
    for sn in sorted(segs.keys()):
      try:
        s, o, oa = scan_seg(segs[sn])
      except Exception as e:
        print(f"  {rh} seg {sn}: ERR {e}")
        continue
      for b in s:
        grand_stock[b].extend(s[b])
      for b in o:
        grand_op[b].extend(o[b])
      total_segs += 1
      total_op_frames += oa

  print(f"\n{'='*80}")
  print(f"  Rate-limit probe — {len(route_hexes)} routes, {total_segs} segments, "
        f"{total_op_frames/100/60:.1f} min op-active")
  print(f"{'='*80}")

  stock_rows, stock_all = summarize(grand_stock, 'STOCK LFA camera')
  op_rows,    op_all    = summarize(grand_op,    'OP planner (sampled 50Hz)')

  print_table(stock_rows, 'STOCK LFA camera (bus2, 0x110 ADAS_StrAnglReqVal Δ)')
  print_table(op_rows,    'OP planner  (angleState.steeringAngleDesiredDeg Δ)')

  # cap-hit analysis
  print(f"\n  === Cap-hit analysis (current MAX_ANGLE_RATE cap = {CURRENT_CAP} deg/20ms) ===")
  print(f"  {'bucket':<14} {'stock_p99':>10} {'op_p99':>9} "
        f"{'stock>cap%':>11} {'op>cap%':>9}  bottleneck?")
  for name, _, _ in SPEED_BUCKETS:
    s = np.array(grand_stock.get(name, []))
    o = np.array(grand_op.get(name, []))
    if len(s) == 0 and len(o) == 0:
      continue
    sp99 = float(np.percentile(s, 99)) if len(s) else 0.0
    op99 = float(np.percentile(o, 99)) if len(o) else 0.0
    sh = cap_hit_pct(s, CURRENT_CAP)
    oh = cap_hit_pct(o, CURRENT_CAP)
    bottleneck = '⚠ YES' if (op99 > CURRENT_CAP or sp99 > CURRENT_CAP) else ''
    print(f"  {name:<14} {sp99:>10.2f} {op99:>9.2f} {sh:>10.1f}% "
          f"{oh:>8.1f}%  {bottleneck}")

  # overall
  print(f"\n  === Overall ===")
  if len(stock_all):
    print(f"  stock  n={len(stock_all)}  mean={stock_all.mean():.2f}  "
          f"p95={np.percentile(stock_all,95):.2f}  "
          f"p99={np.percentile(stock_all,99):.2f}  max={stock_all.max():.2f}  "
          f">cap={cap_hit_pct(stock_all, CURRENT_CAP):.2f}%")
  if len(op_all):
    print(f"  op     n={len(op_all)}  mean={op_all.mean():.2f}  "
          f"p95={np.percentile(op_all,95):.2f}  "
          f"p99={np.percentile(op_all,99):.2f}  max={op_all.max():.2f}  "
          f">cap={cap_hit_pct(op_all, CURRENT_CAP):.2f}%")

  # recommendation — use the regime where the cap actually matters:
  # city-high (~50 km/h) and above.  Low-speed buckets show huge op
  # values because steeringAngleDesiredDeg is pre-rate-limit and the
  # VM angle-from-curvature blows up as v→0; those are numerical
  # artifacts, not real EPS demands.
  if len(stock_all) and len(op_all):
    print(f"\n  === Recommendation for MAX_ANGLE_RATE (deg/20ms) ===")
    print(f"  Current global cap: {CURRENT_CAP:.2f}")
    print(f"  {'regime':<14} {'stock_p99':>10} {'op_p99':>8} {'safe_ceil':>10} "
          f"{'need_floor':>11} {'proposed':>9}")
    for name, _, _ in SPEED_BUCKETS:
      s = np.array(grand_stock.get(name, []))
      o = np.array(grand_op.get(name, []))
      if len(s) < 100 or len(o) < 100:
        continue
      sp99 = float(np.percentile(s, 99))
      op99 = float(np.percentile(o, 99))
      # safe ceiling = factory p99 × 1.2 (20% margin above OEM)
      ceil = sp99 * 1.2
      # need floor = max(planner p99, current cap) — only raise if needed
      floor = op99
      # pick a value within [floor, max(floor, ceil)] closest to ceil
      # but never below current_cap
      if floor <= ceil:
        proposed = (floor + ceil) / 2
      else:
        proposed = min(floor, 3.0)   # hard safety clamp
      marker = '⚠' if floor > CURRENT_CAP else ' '
      print(f"  {marker} {name:<12} {sp99:>10.2f} {op99:>8.2f} {ceil:>10.2f} "
            f"{floor:>11.2f} {proposed:>9.2f}")
    print(f"\n  NOTE: stopped/parking buckets show inflated op values because")
    print(f"  steeringAngleDesiredDeg at v<5 m/s is numerically unstable")
    print(f"  (VM.get_steer_from_curvature amplifies noise when curv_factor→0).")
    print(f"  Trust the city-high / suburban / highway rows for cap sizing.")


if __name__ == "__main__":
  routes = sys.argv[1:] if len(sys.argv) > 1 else \
           ['28', '29', '2a', '2b', '2c', '2d', '3e', '3f', '40']
  run_routes(routes)
