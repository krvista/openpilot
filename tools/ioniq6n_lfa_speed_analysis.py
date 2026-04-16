#!/usr/bin/env python3
"""Analyze route 36 (stock LFA commute) to determine:
1. Camera's LKAS_ALT active/passive behavior at each speed
2. Optimal passthrough↔active transition speed thresholds
3. Steering quality (angle change rate, smoothness) per speed bucket

This route had NO openpilot ACC — pure stock LFA with camera passthrough.
The camera's own LKAS_ANGLE_ACTIVE and ACI bit patterns reveal exactly
at which speeds stock LFA activates its steering, giving us the ground
truth for our passthrough exit threshold.
"""
import glob
import sys
import numpy as np
from collections import defaultdict

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

ROUTE = '00000036--47cb870a03'
DRIVELOG_DIR = '/home/user/openpilot/drivelog'


def main():
  segs = sorted(glob.glob(f'{DRIVELOG_DIR}/*{ROUTE}*rlog.zst'),
                key=lambda p: int(p.split('--')[-2]))
  print(f"Route 36: {len(segs)} segments")

  t0 = None
  # Track camera's 0x110 on bus 2 (raw camera output)
  # and our sendcan 0x110 on bus 0 (what we forwarded)
  # Also track speed from WHEEL_SPEEDS (0xa0) on bus 1

  speed_kmh = 0.0
  # Per-speed bucket: track camera's LKAS behavior
  # Buckets: 0-2, 2-5, 5-7, 7-10, 10-15, 15-20, 20-30, 30-50, 50-70, 70-100
  BUCKETS = [(0, 2), (2, 5), (5, 7), (7, 10), (10, 15), (15, 20),
             (20, 30), (30, 50), (50, 70), (70, 100)]
  bucket_cam_active = defaultdict(int)   # bucket -> count of active frames
  bucket_cam_passive = defaultdict(int)  # bucket -> count of passive frames
  bucket_cam_angle_delta = defaultdict(list)  # bucket -> list of |angle change| per frame
  bucket_steer_angle = defaultdict(list)  # bucket -> steering angles

  prev_cam_angle = None
  frame_count = 0

  for p in segs:
    lr = LogReader(p)
    for m in lr:
      try:
        w = m.which()
      except:
        continue
      if t0 is None:
        t0 = m.logMonoTime

      if w == 'can':
        for c in m.can:
          # Speed from WHEEL_SPEEDS (0xa0) on bus 1
          if c.src == 1 and c.address == 0xa0:
            dat = bytes(c.dat)
            if len(dat) >= 10:
              raw = int.from_bytes(dat, 'little')
              fl = ((raw >> 64) & 0x3fff) * 0.03125
              speed_kmh = fl

          # Camera's LKAS_ALT on bus 2
          if c.src == 2 and c.address == 0x110:
            dat = bytes(c.dat)
            if len(dat) < 14:
              continue
            frame_count += 1

            # Decode key fields
            # LKAS_BYTE13 at byte 13
            byte13 = dat[13]
            # LKAS_BYTE7 at byte 7
            byte7 = dat[7]
            # ADAS_StrAnglReqVal: 14-bit signed at bit 32 (byte 4-5)
            # DBC: SG_ ADAS_StrAnglReqVal : 32|14@1- (0.1,0)
            raw_angle = int.from_bytes(dat[4:6], 'little') & 0x3fff
            if raw_angle >= 0x2000:
              raw_angle -= 0x4000
            angle_deg = raw_angle * 0.1

            is_active = byte13 != 0 or byte7 != 0

            # Find bucket
            bkt = None
            for lo, hi in BUCKETS:
              if lo <= speed_kmh < hi:
                bkt = (lo, hi)
                break
            if bkt is None and speed_kmh >= 100:
              bkt = (70, 100)
            if bkt is None:
              continue

            if is_active:
              bucket_cam_active[bkt] += 1
            else:
              bucket_cam_passive[bkt] += 1

            if prev_cam_angle is not None:
              delta = abs(angle_deg - prev_cam_angle)
              bucket_cam_angle_delta[bkt].append(delta)
            prev_cam_angle = angle_deg
            bucket_steer_angle[bkt].append(angle_deg)

  print(f"\nTotal camera LKAS_ALT frames: {frame_count}")

  # Report
  print(f"\n{'='*80}")
  print(f"Camera LFA Active/Passive by speed bucket (route 36, stock LFA)")
  print(f"{'='*80}")
  print(f"{'Speed':>10} {'Active':>8} {'Passive':>8} {'Active%':>8}  "
        f"{'|Δangle| p50':>12} {'p95':>8} {'|angle| p50':>12} {'p95':>8}")

  for lo, hi in BUCKETS:
    bkt = (lo, hi)
    act = bucket_cam_active[bkt]
    pas = bucket_cam_passive[bkt]
    total = act + pas
    if total == 0:
      continue
    pct = act / total * 100

    deltas = bucket_cam_angle_delta[bkt]
    angles = bucket_steer_angle[bkt]
    if deltas:
      d_arr = np.array(deltas)
      dp50 = np.percentile(d_arr, 50)
      dp95 = np.percentile(d_arr, 95)
    else:
      dp50 = dp95 = 0

    if angles:
      a_arr = np.abs(angles)
      ap50 = np.percentile(a_arr, 50)
      ap95 = np.percentile(a_arr, 95)
    else:
      ap50 = ap95 = 0

    print(f"  {lo:>2}-{hi:<3} km/h {act:>8} {pas:>8} {pct:>7.1f}%  "
          f"{dp50:>10.3f}° {dp95:>7.3f}°  {ap50:>10.2f}° {ap95:>7.2f}°")

  # Also show the transition zone detail (2-10 km/h at 1 km/h resolution)
  print(f"\n{'='*80}")
  print(f"Fine-grained transition zone (1 km/h buckets, 0-15 km/h)")
  print(f"{'='*80}")

  fine_active = defaultdict(int)
  fine_passive = defaultdict(int)

  # Re-scan for fine buckets
  prev_cam_angle = None
  for p in segs:
    lr = LogReader(p)
    for m in lr:
      try:
        w = m.which()
      except:
        continue
      if w == 'can':
        for c in m.can:
          if c.src == 1 and c.address == 0xa0:
            dat = bytes(c.dat)
            if len(dat) >= 10:
              raw = int.from_bytes(dat, 'little')
              speed_kmh = ((raw >> 64) & 0x3fff) * 0.03125
          if c.src == 2 and c.address == 0x110:
            dat = bytes(c.dat)
            if len(dat) < 14: continue
            byte13 = dat[13]
            byte7 = dat[7]
            is_active = byte13 != 0 or byte7 != 0
            spd_int = int(speed_kmh)
            if spd_int > 15: continue
            if is_active:
              fine_active[spd_int] += 1
            else:
              fine_passive[spd_int] += 1

  print(f"{'Speed':>8} {'Active':>8} {'Passive':>8} {'Active%':>8}")
  for spd in range(16):
    act = fine_active[spd]
    pas = fine_passive[spd]
    total = act + pas
    if total == 0: continue
    pct = act / total * 100
    marker = ""
    if spd == 5: marker = "  ← current ENTER threshold"
    if spd == 7: marker = "  ← current EXIT threshold"
    print(f"  {spd:>3} km/h {act:>8} {pas:>8} {pct:>7.1f}%{marker}")


if __name__ == '__main__':
  main()
