#!/usr/bin/env python3
"""Steering quality comparison across rate-limiter variants on Ioniq 6 N drivelogs.

Simulates four rate-limiter configurations on every available drivelog:
  1. OURS       - apply_std_steer_angle_limits with our Tesla-inspired table
  2. TOYOTA     - apply_std_steer_angle_limits with Toyota LTA's actual table
  3. TESLA      - apply_steer_angle_limits_vm with Tesla's VM-based limits
  4. PREVIOUS   - the earlier VM-based Ioniq 6 N config (JERK=3.0, 100Hz)

Each variant is fed the same desired-angle trajectory (derived from
`carControl.actuators.curvature` via VehicleModel) and the same vEgo /
steeringAngleDeg measurements from the log.

Active gating mirrors the real code: requires curvature non-trivial AND
vEgoRaw > 3 km/h (the aci_speed_ok gate).

Metrics (per active frame):
  - Max / p99 |Δangle| per TX step (spike detection)
  - Direction-change frequency (oscillation rate, Hz)
  - Tracking error vs desired angle (mae, p95)
  - Per-speed-bucket tracking MAE

Run:
    python3 tools/ioniq6n_rate_comparison.py [--detailed]
"""
import argparse
import math
import os
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'opendbc_repo'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from openpilot.tools.lib.logreader import LogReader
from opendbc.car.lateral import (
  AngleSteeringLimits,
  apply_std_steer_angle_limits,
  apply_steer_angle_limits_vm,
)
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car import DT_CTRL
from opendbc.car.hyundai.values import CarControllerParams as HyundaiParams


LOG_DIR = '/tmp/ioniq6n_logs'
ACI_MIN_SPEED_MS = 3.0 / 3.6   # 3 km/h — matches our carcontroller


# --- Rate-limiter configurations ---
# OURS pulls the actual production table directly from values.py so this tool
# stays in sync automatically.
OURS_LIMITS = HyundaiParams.ANGLE_LIMITS

# Toyota LTA actual table (from toyota/values.py)
TOYOTA_LIMITS = AngleSteeringLimits(
  94.9461,
  ([5, 25], [0.3, 0.15]),
  ([5, 25], [0.36, 0.26]),
)


class TeslaParamsShim:
  STEER_STEP = 2
  ANGLE_LIMITS = AngleSteeringLimits(
    360,
    ([], []),
    ([], []),
    MAX_LATERAL_ACCEL=3.6,
    MAX_LATERAL_JERK=3.6,
    MAX_ANGLE_RATE=5,
  )


class PrevParamsShim:
  STEER_STEP = 1  # 100 Hz back then
  ANGLE_LIMITS = AngleSteeringLimits(
    176.7,
    ([], []),
    ([], []),
    MAX_LATERAL_ACCEL=3.0,
    MAX_LATERAL_JERK=3.0,
    MAX_ANGLE_RATE=1.0,
  )


SPEED_BUCKETS = [(0, 3), (3, 6), (6, 10), (10, 15), (15, 20), (20, 25), (25, 40)]


def load_cp_vm():
  for f in sorted(os.listdir(LOG_DIR)):
    try:
      for msg in LogReader(os.path.join(LOG_DIR, f)):
        if msg.which() == 'carParams':
          return VehicleModel(msg.carParams)
    except Exception:
      continue
  return None


def collect_frames(path):
  latest_cs = None
  out = []
  for msg in LogReader(path):
    w = msg.which()
    if w == 'carState':
      latest_cs = msg.carState
    elif w == 'carControl' and latest_cs is not None:
      cc = msg.carControl
      out.append({
        'curvature': cc.actuators.curvature,
        'v_ego': max(latest_cs.vEgoRaw, 0.0),
        'actual_angle': latest_cs.steeringAngleDeg,
        'lat_active_log': cc.latActive,
      })
  return out


def simulate_variant(frames, VM, variant, tx_every_frame=2):
  last_angle = 0.0
  out = []

  for i, f in enumerate(frames):
    # Convert curvature → angle (what LatControlAngle would output).
    # Use a sane minimum speed floor to prevent VM from exploding near 0.
    v_sim = max(f['v_ego'], 1.0)
    desired_angle = math.degrees(VM.get_steer_from_curvature(-f['curvature'], v_sim, 0.0))

    # Active gate mirrors production carcontroller: curvature non-trivial AND
    # vEgoRaw above 3 km/h. If not active, the std helper returns the actual
    # wheel angle, so tracking metrics must not count these frames.
    simulated_active = (
      (f['lat_active_log'] or abs(f['curvature']) > 5e-5)
      and f['v_ego'] > ACI_MIN_SPEED_MS
    )

    if i % tx_every_frame == 0:
      if variant == 'OURS':
        last_angle = apply_std_steer_angle_limits(
          desired_angle, last_angle, f['v_ego'], f['actual_angle'],
          simulated_active, OURS_LIMITS,
        )
      elif variant == 'TOYOTA':
        last_angle = apply_std_steer_angle_limits(
          desired_angle, last_angle, f['v_ego'], f['actual_angle'],
          simulated_active, TOYOTA_LIMITS,
        )
      elif variant == 'TESLA':
        last_angle = apply_steer_angle_limits_vm(
          desired_angle, last_angle, f['v_ego'], f['actual_angle'],
          simulated_active, TeslaParamsShim, VM,
        )
      elif variant == 'PREVIOUS':
        last_angle = apply_steer_angle_limits_vm(
          desired_angle, last_angle, f['v_ego'], f['actual_angle'],
          simulated_active, PrevParamsShim, VM,
        )
      else:
        raise ValueError(variant)

    out.append({
      'desired': desired_angle,
      'last': last_angle,
      'v_ego': f['v_ego'],
      'active': simulated_active,
    })
  return out


def compute_metrics(sim, tx_every=2):
  """Compute metrics from simulation output."""
  angles = np.array([s['last'] for s in sim])
  desired = np.array([s['desired'] for s in sim])
  v_ego = np.array([s['v_ego'] for s in sim])
  active = np.array([s['active'] for s in sim])

  if active.sum() < 10:
    return None

  # Extract TX-rate samples (one per 20 ms or per 10 ms depending on tx_every).
  tx_angles = angles[::tx_every]
  tx_active = active[::tx_every]
  tx_v = v_ego[::tx_every]
  tx_desired = desired[::tx_every]

  tx_diff = np.diff(tx_angles)
  act_pair = tx_active[1:] & tx_active[:-1]
  if act_pair.sum() < 5:
    return None

  diffs = np.abs(tx_diff[act_pair])
  tx_dt = DT_CTRL * tx_every

  # Direction changes — count at TX cadence
  sign_prod = np.sign(tx_diff[:-1]) * np.sign(tx_diff[1:])
  act_trip = tx_active[2:] & tx_active[1:-1] & tx_active[:-2]
  dir_changes = ((sign_prod < 0) & act_trip).sum()
  active_time = act_pair.sum() * tx_dt
  osc_hz = dir_changes / active_time if active_time > 0 else 0.0

  # Tracking error
  err = tx_angles[tx_active] - tx_desired[tx_active]

  # Per-speed bucket MAE
  bucket_mae = {}
  for lo, hi in SPEED_BUCKETS:
    mask = tx_active & (tx_v >= lo) & (tx_v < hi)
    if mask.sum() > 5:
      bucket_mae[(lo, hi)] = float(np.mean(np.abs(tx_angles[mask] - tx_desired[mask])))

  # Spike detection — frames where angle changed by more than typical max
  spike_threshold_deg = 1.0  # anything > 1° per 20ms step is large
  high_spikes = (diffs > spike_threshold_deg).sum()

  return {
    'n_active_frames': int(active.sum()),
    'n_tx_steps': int(act_pair.sum()),
    'rate_max': float(diffs.max()) / tx_dt,   # °/s
    'rate_p99': float(np.percentile(diffs, 99)) / tx_dt,
    'rate_p50': float(np.percentile(diffs, 50)) / tx_dt,
    'spikes_gt1deg': int(high_spikes),
    'osc_hz': osc_hz,
    'mae': float(np.mean(np.abs(err))),
    'rmse': float(np.sqrt(np.mean(err * err))),
    'p95_err': float(np.percentile(np.abs(err), 95)),
    'bucket_mae': bucket_mae,
  }


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--logs-dir', default=LOG_DIR)
  parser.add_argument('--variants', default='OURS,TOYOTA,TESLA,PREVIOUS')
  parser.add_argument('--detailed', action='store_true')
  args = parser.parse_args()

  VM = load_cp_vm()
  if VM is None:
    print(f"ERROR: no carParams in {args.logs_dir}")
    return 1

  variants = args.variants.split(',')
  log_files = sorted(f for f in os.listdir(args.logs_dir) if f.endswith('.zst'))
  print(f"Loaded VM from drivelog. Platform: Ioniq 6 N")
  print(f"Processing {len(log_files)} logs from {args.logs_dir}")
  print(f"Gate: curvature>5e-5 AND vEgo>3km/h (mirrors production code)")
  print()

  all_frames = []
  per_file = defaultdict(dict)

  for fn in log_files:
    path = os.path.join(args.logs_dir, fn)
    try:
      frames = collect_frames(path)
    except Exception as e:
      print(f"  ERR {fn}: {e}")
      continue
    if not frames:
      continue
    all_frames.extend(frames)

  print(f"Total frames: {len(all_frames)}")
  print()

  # Aggregate metrics on concatenated frames
  print("=" * 86)
  print("AGGREGATE METRICS (all 8 logs concatenated, 3km/h+ active only)")
  print("=" * 86)
  header = (f"{'variant':<10} {'act_fr':>7} {'rate_max':>10} {'rate_p99':>10}"
            f" {'spikes>1°':>9} {'osc_Hz':>7} {'MAE':>6} {'p95_err':>8}")
  print(header)
  print('-' * len(header))

  results = {}
  for v in variants:
    tx_every = 1 if v == 'PREVIOUS' else 2
    sim = simulate_variant(all_frames, VM, v, tx_every_frame=tx_every)
    m = compute_metrics(sim, tx_every=tx_every)
    results[v] = m
    if m is None:
      print(f"  {v}: no active frames")
      continue
    print(f"{v:<10} {m['n_active_frames']:>7}"
          f" {m['rate_max']:>7.1f}°/s {m['rate_p99']:>7.1f}°/s"
          f" {m['spikes_gt1deg']:>9}"
          f" {m['osc_hz']:>5.2f}Hz"
          f" {m['mae']:>4.2f}° {m['p95_err']:>6.2f}°")

  print()
  print("=" * 86)
  print("PER-SPEED-BUCKET TRACKING MAE (deg) — lower = better tracking of desired")
  print("=" * 86)
  bucket_hdr = f"{'variant':<10}" + ''.join(f"{lo}-{hi}m/s".rjust(12) for lo, hi in SPEED_BUCKETS)
  print(bucket_hdr)
  print('-' * len(bucket_hdr))
  for v in variants:
    m = results[v]
    if m is None:
      continue
    row = f"{v:<10}"
    for b in SPEED_BUCKETS:
      mae = m['bucket_mae'].get(b)
      row += (f"{mae:12.2f}" if mae is not None else f"{'-':>12}")
    print(row)

  if args.detailed:
    print()
    print("=" * 86)
    print("PER-FILE METRICS (active subset only)")
    print("=" * 86)
    for fn in log_files:
      path = os.path.join(args.logs_dir, fn)
      try:
        frames = collect_frames(path)
      except Exception:
        continue
      if not frames:
        continue
      n_act = sum(1 for f in frames if f['v_ego'] > ACI_MIN_SPEED_MS and abs(f['curvature']) > 5e-5)
      print(f"\n{fn} ({n_act} active frames, {len(frames)} total)")
      if n_act < 20:
        print("  (skip: not enough active frames)")
        continue
      for v in variants:
        tx_every = 1 if v == 'PREVIOUS' else 2
        sim = simulate_variant(frames, VM, v, tx_every_frame=tx_every)
        m = compute_metrics(sim, tx_every=tx_every)
        if m is None:
          print(f"  {v}: skip")
          continue
        print(f"  {v:<9} max={m['rate_max']:5.1f}°/s  p99={m['rate_p99']:5.1f}°/s"
              f"  spikes>1°={m['spikes_gt1deg']:4d}  osc={m['osc_hz']:4.2f}Hz"
              f"  MAE={m['mae']:4.2f}°  p95={m['p95_err']:5.2f}°")


if __name__ == '__main__':
  main()
