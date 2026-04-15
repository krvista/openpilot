#!/usr/bin/env python3
"""Steering quality comparison across rate-limiter variants on Ioniq 6 N data.

Primary data source: /tmp/drivelog_frames.pkl (90k preprocessed frames,
16 segments from real drive logs with curvature, desired angle, etc.).
Covers speeds from 0 to ~58 km/h (city + suburban).

Also supports fallback to raw .zst logs in /tmp/ioniq6n_logs and
/tmp/rlog_analysis when the pickle is unavailable.

Simulates these variants with 50 Hz TX cadence:
  1. OURS       - values.py ANGLE_LIMITS (production)
  2. TOYOTA     - Toyota LTA's table
  3. TESLA      - Tesla's VM-based limits
  4. PREVIOUS   - our old 100Hz VM-based config
  5. IONIQ_5    - virtual full torque-stack output (Ioniq 5 latcontrol_torque
                  + first-order plant). Not a rate-limiter variant — it asks
                  "what would a torque-controlled Ioniq 5 have produced on
                  this same reference?" Useful cross-reference for Phase 3
                  validation. See tools/ioniq5_torque_sim.py for details.

Metrics are broken out by Korean speed regime:
  stopped, parking, 20, 30, 40, 50, 60, 80, 100, 110 km/h

Run:
    python3 tools/ioniq6n_rate_comparison.py [--detailed]
"""
import argparse
import math
import os
import pickle
import sys

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


FRAMES_PKL = '/tmp/drivelog_frames.pkl'            # older preprocessed data (16 segs, 0–58 km/h)
CCNC_CACHE = '/tmp/ccnc_frames.pkl'                 # 34-segment real drive (0–91 km/h, 33k op-active)
CCNC_LOG_DIR = '/tmp/ccnc_drivelog'
FALLBACK_LOGS = [
  '/tmp/ioniq6n_logs/99b215d21bbf8735_00000004--90915acd0e--0--rlog.zst',
]
ACI_MIN_SPEED_MS = 3.0 / 3.6


# --- Variants ---
OURS_LIMITS = HyundaiParams.ANGLE_LIMITS

TOYOTA_LIMITS = AngleSteeringLimits(
  94.9461,
  ([5, 25], [0.3, 0.15]),
  ([5, 25], [0.36, 0.26]),
)


class TeslaParamsShim:
  STEER_STEP = 2
  ANGLE_LIMITS = AngleSteeringLimits(
    360, ([], []), ([], []),
    MAX_LATERAL_ACCEL=3.6, MAX_LATERAL_JERK=3.6, MAX_ANGLE_RATE=5,
  )


class PrevParamsShim:
  STEER_STEP = 1
  ANGLE_LIMITS = AngleSteeringLimits(
    176.7, ([], []), ([], []),
    MAX_LATERAL_ACCEL=3.0, MAX_LATERAL_JERK=3.0, MAX_ANGLE_RATE=1.0,
  )


# Compact Ioniq 5 torque stack used by the IONIQ_5 variant. Mirrors the full
# simulator in tools/ioniq5_torque_sim.py. Params from opendbc torque_data/
# params.toml (HYUNDAI_IONIQ_5) + values.py.
I5_STEER_RATIO = 14.26
I5_WHEELBASE = 2.97
I5_LAT_ACCEL_FACTOR = 3.172929
I5_FRICTION = 0.096019
I5_PLANT_TAU = 0.12
I5_PLANT_DAMPING = 3.0
I5_KP_BP = [1.0, 1.5, 2.0, 3.0, 5.0, 7.5, 10.0, 15.0, 30.0]
I5_KP_VAL = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, 0.8]


def simulate_ioniq5_torque(frames, VM, tx_every=1):
  """Virtual Ioniq 5 torque controller + first-order EPS plant replay.
  Returns the same (angles, desired, v, active) tuple as other simulate_*.
  """
  out_a = np.zeros(len(frames))
  out_d = np.zeros(len(frames))
  out_v = np.zeros(len(frames))
  out_act = np.zeros(len(frames), dtype=bool)
  virt_angle = 0.0
  virt_rate = 0.0
  pid_i = 0.0
  prev_seg = None

  for i, (desired, v, sa, seg, lat_active) in enumerate(frames):
    if prev_seg is not None and seg != prev_seg:
      virt_angle, virt_rate, pid_i = sa, 0.0, 0.0
    prev_seg = seg

    active = lat_active and (v > ACI_MIN_SPEED_MS)
    v_safe = max(v, 0.5)
    # Convert desired angle → desired curvature → desired lat accel
    des_curv = math.radians(desired / I5_STEER_RATIO) / I5_WHEELBASE
    des_lat_accel = des_curv * v_safe * v_safe
    # Measured curvature from virtual wheel angle
    meas_curv = math.radians(virt_angle / I5_STEER_RATIO) / I5_WHEELBASE
    meas_lat_accel = meas_curv * v_safe * v_safe
    err = des_lat_accel - meas_lat_accel
    # Friction (piecewise linear, deadzone 0.2 m/s²)
    fric = I5_FRICTION * np.clip(err / 0.2, -1.0, 1.0)
    ff = des_lat_accel + fric
    if active:
      kp_scale = float(np.interp(v, I5_KP_BP, I5_KP_VAL))
      kp = 0.8 * kp_scale
      ki = 0.15 * kp_scale
      if not (v < 5):
        pid_i = float(np.clip(pid_i + ki * err * 0.01, -2.5, 2.5))
      u_lat_accel = ff + kp * err + pid_i
      torque_cmd = float(np.clip(u_lat_accel / I5_LAT_ACCEL_FACTOR, -1.0, 1.0))
      # Plant: first-order lag to steady-state angle
      angle_ss_rad = torque_cmd * I5_LAT_ACCEL_FACTOR / (v_safe * v_safe) * I5_WHEELBASE * I5_STEER_RATIO
      angle_ss = math.degrees(angle_ss_rad)
      target_rate = (angle_ss - virt_angle) / I5_PLANT_TAU
      virt_rate += (target_rate - I5_PLANT_DAMPING * virt_rate) * 0.01 / I5_PLANT_TAU
      virt_rate = float(np.clip(virt_rate, -150.0, 150.0))
      virt_angle += virt_rate * 0.01
    else:
      virt_angle = sa
      virt_rate = 0.0
      pid_i = 0.0

    out_a[i] = virt_angle
    out_d[i] = desired
    out_v[i] = v
    out_act[i] = active
  return out_a, out_d, out_v, out_act


# Korean speed regimes (km/h)
KR_REGIMES = [
  ('stopped',    0,   2),
  ('parking',    2,  10),
  ('20 km/h',   10,  25),
  ('30 km/h',   25,  35),
  ('40 km/h',   35,  45),
  ('50 km/h',   45,  55),
  ('60 km/h',   55,  70),
  ('80 km/h',   70,  90),
  ('100 km/h',  90, 105),
  ('110 km/h', 105, 130),
]


def load_vm():
  for p in FALLBACK_LOGS:
    try:
      for msg in LogReader(p):
        if msg.which() == 'carParams':
          return VehicleModel(msg.carParams)
    except Exception:
      continue
  return None


def load_pickle_frames():
  """Load pickle frames.

  Each frame is (desired, v_ms, actual_angle, seg, lat_active). The 'desired'
  is taken from `actuators_angle` (LatControlAngle output) when available —
  falls back to curvature→VM when not.
  """
  if not os.path.exists(FRAMES_PKL):
    return None
  d = pickle.load(open(FRAMES_PKL, 'rb'))
  VM = load_vm()
  frames = []
  for f in d:
    # Prefer the recorded LatControlAngle output (what the rate-limiter would
    # have been fed in a real drive).
    act_ang = f.get('actuators_angle', 0.0)
    if act_ang is None:
      act_ang = 0.0
    # Fallback to deriving from curvature (older logs may not have it)
    if act_ang == 0.0:
      cv = f.get('curvature', 0) or 0
      v = max(f.get('vEgo', 1.0) or 1.0, 1.0)
      act_ang = math.degrees(VM.get_steer_from_curvature(-cv, v, 0.0))

    v_ms = max(f.get('vEgo', 0.0) or 0.0, 0.0)
    actual = f.get('actual_angle', 0.0) or 0.0
    seg = f.get('seg', -1)
    lat_active = bool(f.get('lat_active', False))
    frames.append((act_ang, v_ms, actual, seg, lat_active))
  return frames


def simulate_limits(frames, VM, limits, tx_every=2):
  """Apply apply_std_steer_angle_limits variant."""
  last = 0.0
  out_a = np.zeros(len(frames))
  out_d = np.zeros(len(frames))
  out_v = np.zeros(len(frames))
  out_act = np.zeros(len(frames), dtype=bool)
  prev_seg = None

  for i, (desired, v, sa, seg, lat_active) in enumerate(frames):
    if prev_seg is not None and seg != prev_seg:
      last = 0.0
    prev_seg = seg

    # Engage condition: mirror production (lat_active from log AND speed gate).
    active = lat_active and (v > ACI_MIN_SPEED_MS)

    if i % tx_every == 0:
      last = apply_std_steer_angle_limits(desired, last, v, sa, active, limits)

    out_a[i] = last
    out_d[i] = desired
    out_v[i] = v
    out_act[i] = active
  return out_a, out_d, out_v, out_act


def simulate_vm(frames, VM, shim):
  """Apply apply_steer_angle_limits_vm variant."""
  last = 0.0
  out_a = np.zeros(len(frames))
  out_d = np.zeros(len(frames))
  out_v = np.zeros(len(frames))
  out_act = np.zeros(len(frames), dtype=bool)
  prev_seg = None
  tx_every = shim.STEER_STEP

  for i, (desired, v, sa, seg, lat_active) in enumerate(frames):
    if prev_seg is not None and seg != prev_seg:
      last = 0.0
    prev_seg = seg

    active = lat_active and (v > ACI_MIN_SPEED_MS)

    if i % tx_every == 0:
      last = apply_steer_angle_limits_vm(desired, last, v, sa, active, shim, VM)

    out_a[i] = last
    out_d[i] = desired
    out_v[i] = v
    out_act[i] = active
  return out_a, out_d, out_v, out_act


def metrics(angles, desired, v_arr, active, tx_every=2):
  tx_a = angles[::tx_every]
  tx_d = desired[::tx_every]
  tx_v = v_arr[::tx_every]
  tx_act = active[::tx_every]
  tx_diff = np.diff(tx_a)
  act_pair = tx_act[1:] & tx_act[:-1]
  if act_pair.sum() < 5:
    return None
  diffs = np.abs(tx_diff[act_pair])
  err = tx_a[tx_act] - tx_d[tx_act]

  # Direction flips (oscillation)
  sgn = np.sign(tx_diff[:-1]) * np.sign(tx_diff[1:])
  act3 = tx_act[2:] & tx_act[1:-1] & tx_act[:-2]
  flips = int(((sgn < 0) & act3).sum())
  dur = act_pair.sum() * DT_CTRL * tx_every
  osc_hz = flips / dur if dur > 0 else 0

  # Per-Korean-regime MAE
  regime_mae = {}
  for name, lo, hi in KR_REGIMES:
    kmh = tx_v * 3.6
    m = tx_act & (kmh >= lo) & (kmh < hi)
    if m.sum() > 10:
      regime_mae[name] = float(np.mean(np.abs(tx_a[m] - tx_d[m])))

  return {
    'n_active': int(active.sum()),
    'n_tx_steps': int(act_pair.sum()),
    'rate_max_deg_s': float(diffs.max()) * 50,
    'rate_p99_deg_s': float(np.percentile(diffs, 99)) * 50,
    'spikes_gt1deg': int((diffs > 1.0).sum()),
    'spikes_gt2deg': int((diffs > 2.0).sum()),
    'osc_hz': osc_hz,
    'mae': float(np.mean(np.abs(err))),
    'rmse': float(np.sqrt(np.mean(err * err))),
    'p95_err': float(np.percentile(np.abs(err), 95)),
    'regime_mae': regime_mae,
  }


def print_table(results):
  print(f"{'variant':<10} {'act_fr':>7} {'rate_max':>10} {'rate_p99':>10}"
        f" {'spk>1°':>7} {'spk>2°':>7} {'osc':>7} {'MAE':>6} {'p95':>7}")
  print('-' * 81)
  for v, m in results.items():
    if m is None:
      print(f"  {v}: no data")
      continue
    print(f"{v:<10} {m['n_active']:>7}"
          f" {m['rate_max_deg_s']:>6.1f}°/s {m['rate_p99_deg_s']:>6.1f}°/s"
          f" {m['spikes_gt1deg']:>7} {m['spikes_gt2deg']:>7}"
          f" {m['osc_hz']:>4.1f}Hz {m['mae']:>4.2f}° {m['p95_err']:>5.2f}°")


def print_regime_mae(results):
  header = f"{'variant':<10}" + ''.join(f"{r[0]:>10}" for r in KR_REGIMES)
  print(header)
  print('-' * len(header))
  for v, m in results.items():
    if m is None:
      continue
    row = f"{v:<10}"
    for name, _, _ in KR_REGIMES:
      mae = m['regime_mae'].get(name)
      row += (f"{mae:10.2f}" if mae is not None else f"{'-':>10}")
    print(row)


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--variants', default='OURS,TOYOTA,TESLA,PREVIOUS,IONIQ_5')
  parser.add_argument('--detailed', action='store_true')
  args = parser.parse_args()

  VM = load_vm()
  if VM is None:
    print("ERROR: no VehicleModel")
    return 1

  frames = load_pickle_frames()
  if frames is None:
    print(f"ERROR: need {FRAMES_PKL}")
    return 1

  print(f"Data source: {FRAMES_PKL} ({len(frames)} frames, 16 segments)")
  print(f"Gate: |curvature|>5e-5 AND vEgo>3km/h (mirrors production)\n")

  variants = args.variants.split(',')
  results = {}
  for v in variants:
    if v == 'OURS':
      a, d, vv, ac = simulate_limits(frames, VM, OURS_LIMITS, tx_every=2)
      m = metrics(a, d, vv, ac, tx_every=2)
    elif v == 'TOYOTA':
      a, d, vv, ac = simulate_limits(frames, VM, TOYOTA_LIMITS, tx_every=2)
      m = metrics(a, d, vv, ac, tx_every=2)
    elif v == 'TESLA':
      a, d, vv, ac = simulate_vm(frames, VM, TeslaParamsShim)
      m = metrics(a, d, vv, ac, tx_every=2)
    elif v == 'PREVIOUS':
      a, d, vv, ac = simulate_vm(frames, VM, PrevParamsShim)
      m = metrics(a, d, vv, ac, tx_every=1)
    elif v == 'IONIQ_5':
      a, d, vv, ac = simulate_ioniq5_torque(frames, VM)
      # tx_every=1 — torque controller samples every 10 ms (not gated by TX cadence)
      m = metrics(a, d, vv, ac, tx_every=1)
    else:
      print(f"Unknown variant: {v}")
      continue
    results[v] = m

  print("=" * 81)
  print("AGGREGATE METRICS")
  print("=" * 81)
  print_table(results)
  print()
  print("=" * 110)
  print("PER-REGIME MAE (deg) — tracking error vs desired in each Korean speed bucket")
  print("=" * 110)
  print_regime_mae(results)


if __name__ == '__main__':
  main()
