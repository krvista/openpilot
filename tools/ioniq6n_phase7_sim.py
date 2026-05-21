#!/usr/bin/env python3
"""Phase 6c (N7) metric extraction + closed-form prediction sim.

Reads rlog.zst segments from a directory (default: /tmp/drive1f) and
produces five metric tables that mirror the Phase 6c plan pass-gates,
plus a best-effort closed-form prediction of how the N7a / N7b / N7c
hooks would reshape the per-frame ACIGain ceiling, B1 blend ratio and
sp_smooth_angle alpha for the same input frames.

Closed-form caveat: the sim re-applies the new hooks to the *observed*
apply_angle / wheel / torque / vEgo trace. It cannot re-run the lateral
planner or the VM rate limiter — op_curv and modelV2 desiredCurvature
would have to come from rebuilt model output. Treat the variant numbers
as upper bounds on the hook's per-frame effect; the on-vehicle drivelog
of the b6e5842 build is the ground truth.

Usage:
  # extract drivelog (read-only, only operates on the local git blob)
  mkdir -p /tmp/drive1f
  for n in $(seq 0 11); do
    git show "origin/ccnc-drivelog:drivelog/99b215d21bbf8735_0000001f--c6a312398c--${n}--rlog.zst" \
      > /tmp/drive1f/${n}--rlog.zst
  done
  .venv/bin/python tools/ioniq6n_phase7_sim.py /tmp/drive1f
"""
import argparse
import glob
import os
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, '/home/user/openpilot')
sys.path.insert(0, '/home/user/openpilot/opendbc_repo')

from openpilot.tools.lib.logreader import LogReader  # noqa: E402

DT_CTRL = 0.01
MS_TO_KPH = 3.6

# Constants mirrored from carcontroller.py / values.py at b6e5842 (post-Phase-6c).
DRIVER_TORQUE_DEADZONE_ANGLE              = 100.0
DRIVER_TORQUE_FULL_OVERRIDE_LOW_V_ANGLE   = 180.0
DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V_ANGLE  = 350.0
DRIVER_TORQUE_DEADZONE_ANGLE_BLINKER      = 70.0
DRIVER_TORQUE_FULL_OVERRIDE_LOW_V_BLINKER = 130.0
DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V_BLINKER= 220.0
DRIVER_TORQUE_LOW_V_SPEED  = 8.0
DRIVER_TORQUE_HIGH_V_SPEED = 15.0

# alpha curve: N6a baseline vs N7c
ALPHA_VEGO_BP   = [0, 8.5, 11, 13.8, 18]
ALPHA_V_N6A     = [0.05, 0.1, 0.3, 0.6, 1]
ALPHA_V_N7C     = [0.05, 0.05, 0.15, 0.4, 1]

SPEED_BUCKETS = [(0, 20), (20, 30), (30, 40), (40, 60), (60, 90), (90, 200)]


def override_factor(abs_tq: float, v_ms: float, blinker_on: bool) -> float:
  if blinker_on:
    dz, lo, hi = (DRIVER_TORQUE_DEADZONE_ANGLE_BLINKER,
                  DRIVER_TORQUE_FULL_OVERRIDE_LOW_V_BLINKER,
                  DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V_BLINKER)
  else:
    dz, lo, hi = (DRIVER_TORQUE_DEADZONE_ANGLE,
                  DRIVER_TORQUE_FULL_OVERRIDE_LOW_V_ANGLE,
                  DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V_ANGLE)
  full = float(np.interp(v_ms, [DRIVER_TORQUE_LOW_V_SPEED, DRIVER_TORQUE_HIGH_V_SPEED], [lo, hi]))
  return float(np.clip((abs_tq - dz) / max(full - dz, 1.0), 0.0, 1.0))


def b1_blend(of: float, divisor: float) -> float:
  if of <= 0.1:
    return 0.0
  return min((of - 0.1) / divisor, 1.0)


def aci_ceiling(steering_torque: float, v_ego_kph: float, steering_error: float,
                blinker_on: bool, *, n7b: bool) -> float:
  """dynamic_ceiling computed with or without N7b error_mult suppression."""
  base = float(np.interp(v_ego_kph, [0, 20, 40, 120], [0.4, 0.62, 0.85, 1.0]))
  err_start = float(np.interp(v_ego_kph, [0, 20, 40, 120], [1.25, 0.5, 0.3, 0.2]))
  raw = float(np.interp(abs(steering_error), [err_start, err_start * 2], [1.0, 2.0]))
  if n7b:
    suppress = float(np.interp(abs(steering_torque), [100, 180], [1.0, 0.0]))
    mult = 1.0 + (raw - 1.0) * suppress
  else:
    mult = raw
  ceil = min(1.0, base * mult)
  if blinker_on:
    ceil = min(ceil, 0.45)
  return ceil


def aci_target(steering_torque: float, ceiling: float) -> float:
  return float(np.interp(abs(steering_torque), [100, 350], [ceiling, 0.19]))


def alpha_for(v_ms: float, *, n7c: bool) -> float:
  curve = ALPHA_V_N7C if n7c else ALPHA_V_N6A
  return float(np.interp(abs(v_ms), ALPHA_VEGO_BP, curve))


def percentile(arr, p):
  if not arr:
    return float('nan')
  return float(np.percentile(arr, p))


def speed_bucket(v_kph: float):
  for lo, hi in SPEED_BUCKETS:
    if lo <= v_kph < hi:
      return (lo, hi)
  return None


def collect_frames(paths):
  """Stream rlog segments and yield per-frame tuples."""
  cc_cache = {'apply': 0.0, 'lat_active': False}
  for path in paths:
    cs_buf = None
    for msg in LogReader(path):
      which = msg.which()
      if which == 'carState':
        cs_buf = msg.carState
      elif which == 'carControl' and cs_buf is not None:
        cc = msg.carControl
        apply_angle = float(cc.actuators.steeringAngleDeg)
        lat_active = bool(cc.latActive)
        v_ego = float(cs_buf.vEgo)
        wheel = float(cs_buf.steeringAngleDeg)
        tq = float(cs_buf.steeringTorque)
        blinker = bool(cs_buf.leftBlinker or cs_buf.rightBlinker)
        yield {
          'v_ego': v_ego,
          'v_kph': v_ego * MS_TO_KPH,
          'wheel': wheel,
          'tq': tq,
          'abs_tq': abs(tq),
          'blinker': blinker,
          'lat_active': lat_active,
          'apply': apply_angle,
          'apply_prev': cc_cache['apply'],
          'mismatch': apply_angle - wheel,
        }
        cc_cache['apply'] = apply_angle
        cc_cache['lat_active'] = lat_active


def main(argv=None):
  ap = argparse.ArgumentParser()
  ap.add_argument('directory', nargs='?', default='/tmp/drive1f',
                  help='directory containing rlog.zst segments (extracted via git show origin/ccnc-drivelog)')
  ap.add_argument('--limit', type=int, default=12, help='max segments to read')
  args = ap.parse_args(argv)

  paths = sorted(glob.glob(os.path.join(args.directory, '*rlog.zst')))
  if not paths:
    # tolerate the original filename convention from the branch
    paths = sorted(glob.glob(os.path.join(args.directory, '*--rlog.zst')))
  if not paths:
    print(f'no rlog.zst under {args.directory}')
    return 1
  paths = paths[:args.limit]
  print(f'reading {len(paths)} segments from {args.directory}')

  total = 0
  # Concern 1 — grip distribution + B1 full blend rate
  tq_samples = []
  grip_frames = []          # frames with abs_tq > 100
  high_grip_frames = []     # frames with abs_tq > 150
  mismatch_samples = []
  light_grip_apply_delta = []

  # Concern 2 — |Δapply| jitter by speed bucket
  jitter_by_bucket = defaultdict(list)

  # ACIGain estimate by speed bucket
  aci_n6a_by_bucket = defaultdict(list)
  aci_n7_by_bucket  = defaultdict(list)

  for f in collect_frames(paths):
    total += 1
    abs_tq = f['abs_tq']
    tq_samples.append(abs_tq)
    mismatch_samples.append(abs(f['mismatch']))

    of = override_factor(abs_tq, f['v_ego'], f['blinker'])
    b1_n6a = b1_blend(of, 0.9)
    b1_n7  = b1_blend(of, 0.4)

    if abs_tq > 100:
      grip_frames.append({
        'of': of, 'mismatch': abs(f['mismatch']), 'tq': abs_tq,
        'b1_n6a': b1_n6a, 'b1_n7': b1_n7, 'v_kph': f['v_kph'],
      })
    if abs_tq > 150:
      high_grip_frames.append(of)

    if 60 <= abs_tq <= 80:
      light_grip_apply_delta.append(abs(f['apply'] - f['apply_prev']))

    bucket = speed_bucket(f['v_kph'])
    if bucket is not None:
      jitter_by_bucket[bucket].append(abs(f['apply'] - f['apply_prev']))
      # ACIGain target (approximation): error_mult uses steering_error proxy = mismatch
      err = abs(f['mismatch'])
      ceil_n6a = aci_ceiling(abs_tq, f['v_kph'], err, f['blinker'], n7b=False)
      ceil_n7  = aci_ceiling(abs_tq, f['v_kph'], err, f['blinker'], n7b=True)
      if f['lat_active']:
        aci_n6a_by_bucket[bucket].append(aci_target(abs_tq, ceil_n6a))
        aci_n7_by_bucket[bucket].append(aci_target(abs_tq, ceil_n7))

  print(f'\n=== Phase 6c sim — drivelog metric over {total} frames ===')
  print('\n--- driver_torque distribution (all frames) ---')
  print(f'  p50={percentile(tq_samples, 50):6.1f}  p90={percentile(tq_samples, 90):6.1f}  '
        f'p95={percentile(tq_samples, 95):6.1f}  p99={percentile(tq_samples, 99):6.1f}  '
        f'max={max(tq_samples):6.1f}  >100 Nm frames: {len([t for t in tq_samples if t>100])}/{total} '
        f'({100*len([t for t in tq_samples if t>100])/total:.1f}%)')

  print('\n--- mismatch (|apply_angle - wheel|) all frames ---')
  print(f'  p50={percentile(mismatch_samples, 50):6.2f}  p95={percentile(mismatch_samples, 95):6.2f}  '
        f'p99={percentile(mismatch_samples, 99):6.2f}')

  print('\n--- B1 full blend (≥0.95) at sustained grip (>100 Nm) ---')
  if grip_frames:
    n_n6a = sum(1 for g in grip_frames if g['b1_n6a'] >= 0.95)
    n_n7  = sum(1 for g in grip_frames if g['b1_n7']  >= 0.95)
    print(f'  N6a (divisor 0.9): {n_n6a}/{len(grip_frames)} = {100*n_n6a/len(grip_frames):.1f}%')
    print(f'  N7a (divisor 0.4): {n_n7 }/{len(grip_frames)} = {100*n_n7 /len(grip_frames):.1f}%')

  print('\n--- |Δapply| (°/frame) by speed bucket ---')
  print('  bucket          p50    p95    p99   frames')
  for bucket in SPEED_BUCKETS:
    samples = jitter_by_bucket.get(bucket, [])
    if not samples:
      continue
    print(f'  {bucket[0]:3d}-{bucket[1]:3d} kph   '
          f'{percentile(samples, 50):5.2f}  {percentile(samples, 95):5.2f}  '
          f'{percentile(samples, 99):5.2f}  {len(samples):6d}')

  print('\n--- ACIGain target by speed bucket (latActive only) — closed-form N7b comparison ---')
  print('  bucket          mean(N6a)   mean(N7b)   delta')
  for bucket in SPEED_BUCKETS:
    n6a = aci_n6a_by_bucket.get(bucket, [])
    n7  = aci_n7_by_bucket.get(bucket, [])
    if not n6a:
      continue
    m_n6a = float(np.mean(n6a))
    m_n7  = float(np.mean(n7))
    print(f'  {bucket[0]:3d}-{bucket[1]:3d} kph   {m_n6a:7.3f}    {m_n7:7.3f}    {m_n7 - m_n6a:+.3f}')

  print('\n--- ACIGain target at sustained grip (>200 Nm) — closed-form N7b ---')
  high_n6a = []
  high_n7  = []
  for f in collect_frames(paths):
    if not (f['lat_active'] and f['abs_tq'] > 200):
      continue
    err = abs(f['mismatch'])
    ceil_n6a = aci_ceiling(f['abs_tq'], f['v_kph'], err, f['blinker'], n7b=False)
    ceil_n7  = aci_ceiling(f['abs_tq'], f['v_kph'], err, f['blinker'], n7b=True)
    high_n6a.append(aci_target(f['abs_tq'], ceil_n6a))
    high_n7.append (aci_target(f['abs_tq'], ceil_n7))
  if high_n6a:
    m_n6a = float(np.mean(high_n6a))
    m_n7  = float(np.mean(high_n7))
    print(f'  N6a mean ACI: {m_n6a:.3f}   N7b mean ACI: {m_n7:.3f}   '
          f'reduction: {100*(1 - m_n7/m_n6a):.1f}%   n={len(high_n6a)}')

  print('\n--- N7c sp_smooth_angle alpha shift summary ---')
  print('  v(kph)   alpha_N6a   alpha_N7c   ratio')
  for v_ms in [0.0, 8.5, 11.0, 13.8, 18.0]:
    a6 = alpha_for(v_ms, n7c=False)
    a7 = alpha_for(v_ms, n7c=True)
    print(f'  {v_ms*MS_TO_KPH:5.1f}   {a6:7.3f}     {a7:7.3f}     {a7/max(a6, 1e-6):.2f}')

  print('\n--- light-grip (60-80 Nm) false-positive check ---')
  if light_grip_apply_delta:
    print(f'  |Δapply| in light-grip frames: p50={percentile(light_grip_apply_delta, 50):.3f}  '
          f'p95={percentile(light_grip_apply_delta, 95):.3f}  '
          f'p99={percentile(light_grip_apply_delta, 99):.3f}  n={len(light_grip_apply_delta)}')
    print('  (N7a leaves override_factor≤0.1 untouched → no change expected on this metric)')

  print('\n--- Phase 6d angle-aware passive latch (40°/60 Nm enter, 30 Nm exit) ---')
  ap_enter_deg = 40.0
  ap_enter_tq  = 60.0
  ap_exit_tq   = 30.0
  ap_min_enter_frames = 5  # Phase 6e-1
  entry_zone_frames = 0
  exit_zone_frames = 0
  latch_6d_frames = 0  # Phase 6d (no transient filter)
  latch_6e_frames = 0  # Phase 6e-1 (5-frame transient filter)
  total_lat_active = 0
  latch_6d = False
  latch_6e = False
  enter_cnt = 0
  dwells_6d = []
  dwells_6e = []
  run_6d = 0
  run_6e = 0
  for f in collect_frames(paths):
    if not f['lat_active']:
      if latch_6d:
        dwells_6d.append(run_6d)
      if latch_6e:
        dwells_6e.append(run_6e)
      run_6d = 0
      run_6e = 0
      latch_6d = False
      latch_6e = False
      enter_cnt = 0
      continue
    total_lat_active += 1
    abs_wheel = abs(f['wheel'])
    abs_tq = f['abs_tq']
    entry_now = (abs_wheel >= ap_enter_deg and abs_tq >= ap_enter_tq)
    if entry_now:
      entry_zone_frames += 1
    if abs_tq < ap_exit_tq:
      exit_zone_frames += 1
    if latch_6d:
      if abs_tq < ap_exit_tq:
        latch_6d = False
        dwells_6d.append(run_6d)
        run_6d = 0
    else:
      if entry_now:
        latch_6d = True
    if latch_6d:
      latch_6d_frames += 1
      run_6d += 1
    if latch_6e:
      if abs_tq < ap_exit_tq:
        latch_6e = False
        enter_cnt = 0
        dwells_6e.append(run_6e)
        run_6e = 0
    else:
      if entry_now:
        enter_cnt = min(enter_cnt + 1, ap_min_enter_frames)
        if enter_cnt >= ap_min_enter_frames:
          latch_6e = True
      else:
        enter_cnt = 0
    if latch_6e:
      latch_6e_frames += 1
      run_6e += 1
  if latch_6d:
    dwells_6d.append(run_6d)
  if latch_6e:
    dwells_6e.append(run_6e)
  if total_lat_active:
    pct = lambda n: 100.0 * n / total_lat_active
    print(f'  latActive frames analyzed: {total_lat_active}')
    print(f'  entry-zone (|wheel|>=40 AND |tq|>=60):  {entry_zone_frames:6d}  ({pct(entry_zone_frames):5.2f}%)')
    print(f'  exit-zone  (|tq|<30):                    {exit_zone_frames:6d}  ({pct(exit_zone_frames):5.2f}%)')
    print(f'  Phase 6d latch active (STEER_REQ=0):     {latch_6d_frames:6d}  ({pct(latch_6d_frames):5.2f}%)')
    print(f'  Phase 6e-1 latch active (5-frame filter):{latch_6e_frames:6d}  ({pct(latch_6e_frames):5.2f}%)')

  def dwell_summary(label, ds):
    short  = sum(1 for d in ds if d < 5)
    medium = sum(1 for d in ds if 5 <= d < 50)
    long_  = sum(1 for d in ds if d >= 50)
    print(f'  {label}: events={len(ds)}  <5fr(transient)={short}  5-50fr={medium}  >=50fr={long_}', end='')
    if ds:
      print(f'   p50={percentile(ds,50):.0f}  p95={percentile(ds,95):.0f}  max={max(ds)}')
    else:
      print()
  dwell_summary('  dwell 6d ', dwells_6d)
  dwell_summary('  dwell 6e-1', dwells_6e)
  print('  (transient events filtered out by Phase 6e-1 = 6d_<5fr - 6e-1_<5fr)')

  return 0


if __name__ == '__main__':
  sys.exit(main())
