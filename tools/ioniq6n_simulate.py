#!/usr/bin/env python3
"""Ioniq 5 N vs Ioniq 6 N control comparison on 6 N drivelog.

Runs the current 6 N carcontroller (angle + filter) and simulates 5 N's
torque-based control layer in parallel on the same drivelog, then converts
5 N's torque output to a synthetic wheel-angle trajectory via a simple EPS
model so we can directly compare the two cars' lateral command behavior.

Usage:
    python3 tools/ioniq6n_simulate.py [--tau 0.15] [--max-segments 20]
"""

import argparse
import math
import os
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'opendbc_repo'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from openpilot.tools.lib.logreader import LogReader  # noqa: E402
from opendbc.car.hyundai.carcontroller import CarController  # noqa: E402
from opendbc.car.hyundai.values import CarControllerParams, HyundaiFlags  # noqa: E402
from opendbc.car.lateral import apply_driver_steer_torque_limits  # noqa: E402
from opendbc.car.vehicle_model import VehicleModel  # noqa: E402
from opendbc.car import Bus, DT_CTRL, structs  # noqa: E402


DRIVELOG_DIR = os.path.join(os.path.dirname(__file__), '..', 'drivelog')
DBC_NAMES = {Bus.pt: "hyundai_canfd_generated"}


class MockCS:
  """Minimal CarState stub that matches the Hyundai carcontroller expectations."""
  def __init__(self, cs):
    self.out = cs
    self.is_metric = True
    self.main_cruise_enabled = False
    self.lfa_block_msg = {"COUNTER": 0, "LEFT_LANE_LINE": 0, "RIGHT_LANE_LINE": 0}
    for i in range(3, 32):
      if i != 7:
        self.lfa_block_msg[f"BYTE{i}"] = 0
    self.msg_161 = {"COUNTER": 0, "CHECKSUM": 0}
    self.msg_162 = {"COUNTER": 0, "CHECKSUM": 0}
    self.msg_1b5 = {"COUNTER": 0, "CHECKSUM": 0}
    self.cruise_info = {"COUNTER": 0, "CHECKSUM": 0, "ACCMode": 0, "VSetDis": 0,
                        "CRUISE_STANDSTILL": 0, "NEW_SIGNAL_1": 0, "MainMode_ACC": 0,
                        "ZEROS_9": 0, "ZEROS_5": 0, "DISTANCE_SETTING": 0}


def decode_lkas_alt_angle(dat):
  """Decode ADAS_StrAnglReqVal from a 32-byte LKAS_ALT message."""
  if len(dat) < 32:
    return None
  raw = ((dat[10] >> 2) & 0x3F) | (dat[11] << 6)
  raw = raw & 0x3FFF
  if raw >= 8192:
    raw -= 16384
  return raw * 0.1


class FiveNReferenceModel:
  """Closed-loop reference model of Ioniq 5 N's effective angle trajectory.

  Instead of open-loop integrating torque (which diverges because we feed the
  6 N's torque commands that were computed against a different vehicle state),
  we model the 5 N's *effective wheel-angle response* to the desired_angle
  setpoint by applying a first-order lag that captures 5 N's combined:
    - LatControlTorque 1.2 Hz jerk filter (τ ≈ 0.133 s)
    - EPS actuator lag / torque ramp (τ ≈ 0.100 s)
  → combined effective τ ≈ 0.23 s

  When inactive the reference snaps to the wheel position (no transient).
  """
  def __init__(self, tau=0.23):
    self.tau = tau
    self.alpha = 1.0 - math.exp(-DT_CTRL / tau)
    self.ref_angle = 0.0

  def step(self, desired_angle_deg, steer_wheel_deg, lat_active):
    if not lat_active:
      self.ref_angle = steer_wheel_deg
      return self.ref_angle
    self.ref_angle += self.alpha * (desired_angle_deg - self.ref_angle)
    return self.ref_angle


def load_cp():
  # Find any drivelog with a carParams message
  for f in sorted(os.listdir(DRIVELOG_DIR)):
    path = os.path.join(DRIVELOG_DIR, f)
    try:
      for msg in LogReader(path):
        if msg.which() == 'carParams':
          CP = msg.carParams.as_builder()
        elif msg.which() == 'carParamsSP':
          CP_SP = msg.carParamsSP.as_builder()
          return CP, CP_SP
    except Exception:
      continue
  return None, None


def collect_aligned_pairs(lr_data):
  latest_cs, latest_cc_sp = None, None
  pairs = []
  for msg in lr_data:
    if msg.which() == 'carState':
      latest_cs = msg.carState
    elif msg.which() == 'carControlSP':
      latest_cc_sp = msg.carControlSP
    elif msg.which() == 'carControl':
      if latest_cs is not None and latest_cc_sp is not None:
        pairs.append((msg.carControl, latest_cc_sp, latest_cs))
  return pairs


def run_comparison(max_segments=None, verbose=True):
  CP, CP_SP = load_cp()
  if CP is None:
    print("ERROR: could not load carParams from drivelog/")
    return None

  cc_6n = CarController(DBC_NAMES, CP, CP_SP)
  ref_5n = FiveNReferenceModel(tau=0.23)
  VM = VehicleModel(CP)

  from opendbc.car.hyundai.carcontroller import LKAS_FILTER_TAU
  print(f"CP: {CP.carFingerprint}")
  print(f"6 N filter τ = {LKAS_FILTER_TAU}s")
  print(f"5 N reference τ = {ref_5n.tau}s (EPS lag + jerk filter combined)")
  print()

  # Gather all drivelog files, sorted
  all_files = sorted(
    f for f in os.listdir(DRIVELOG_DIR) if f.endswith('.zst') and '--rlog' in f
  )
  if max_segments:
    all_files = all_files[:max_segments]

  # Aggregate metrics per speed bucket
  speed_buckets = [(0, 3), (3, 6), (6, 10), (10, 15), (15, 20), (20, 25), (25, 40)]
  bucket_errors = defaultdict(list)  # speed_bucket -> [angle_diff, ...]

  total_frames = 0
  active_frames = 0
  total_sq_err = 0.0
  angle_6n_max_rate = 0.0
  angle_5n_max_rate = 0.0
  prev_a6 = 0.0
  prev_a5 = 0.0

  # Step response analysis: find events where desired angle changes rapidly
  # (absolute change > 3° within 500ms) and measure rise time for each car
  step_6n_rise_times = []  # 10% -> 90%
  step_5n_rise_times = []

  for fi, fname in enumerate(all_files):
    path = os.path.join(DRIVELOG_DIR, fname)
    try:
      lr_data = list(LogReader(path))
    except Exception:
      continue

    pairs = collect_aligned_pairs(lr_data)
    if not pairs:
      continue

    for cc_msg, cc_sp, cs_msg in pairs:
      v = max(cs_msg.vEgoRaw, 0.1)

      # Run 6 N carcontroller - reads the filtered apply_angle
      try:
        cc_6n.update(cc_msg, cc_sp, MockCS(cs_msg), 0)
      except Exception:
        continue
      angle_6n = cc_6n.apply_angle_filtered

      # Derive desired angle from curvature (what both cars are ultimately aiming for)
      desired_angle = math.degrees(VM.get_steer_from_curvature(-cc_msg.actuators.curvature, v, 0.0))

      # 5 N reference: what the 5 N's effective wheel angle would track
      angle_5n = ref_5n.step(desired_angle, cs_msg.steeringAngleDeg, cc_msg.latActive)

      total_frames += 1
      if cc_msg.latActive:
        active_frames += 1
        err = angle_6n - angle_5n
        total_sq_err += err * err

        for lo, hi in speed_buckets:
          if lo <= cs_msg.vEgoRaw < hi:
            bucket_errors[(lo, hi)].append(err)
            break

        r6 = abs(angle_6n - prev_a6)
        r5 = abs(angle_5n - prev_a5)
        if r6 > angle_6n_max_rate:
          angle_6n_max_rate = r6
        if r5 > angle_5n_max_rate:
          angle_5n_max_rate = r5

      prev_a6 = angle_6n
      prev_a5 = angle_5n

    if verbose and (fi + 1) % 20 == 0:
      print(f"  processed {fi+1}/{len(all_files)} files")

  rms = math.sqrt(total_sq_err / max(active_frames, 1))

  print()
  print("=" * 60)
  print("5 N vs 6 N comparison results")
  print("=" * 60)
  print(f"Files processed:  {len(all_files)}")
  print(f"Total frames:     {total_frames}")
  print(f"Active frames:    {active_frames}")
  print(f"Overall RMS err:  {rms:.3f}°")
  print(f"6 N max A2A rate: {angle_6n_max_rate:.3f}°/f ({angle_6n_max_rate*100:.0f}°/s)")
  print(f"5 N max A2A rate: {angle_5n_max_rate:.3f}°/f ({angle_5n_max_rate*100:.0f}°/s)")
  print()
  print("Per-speed RMS angle difference (6 N filtered - 5 N synthetic):")
  print(f"  {'Speed':12s} {'n':>8s} {'mean':>9s} {'std':>9s} {'rms':>9s} {'p95':>9s}")
  for (lo, hi) in speed_buckets:
    errs = bucket_errors.get((lo, hi), [])
    if not errs:
      continue
    errs_arr = np.array(errs)
    abs_errs = np.abs(errs_arr)
    mean_v = float(np.mean(errs_arr))
    std_v = float(np.std(errs_arr))
    rms_v = float(np.sqrt(np.mean(errs_arr * errs_arr)))
    p95 = float(np.percentile(abs_errs, 95))
    print(f"  {lo:2d}-{hi:2d} m/s    {len(errs):8d} {mean_v:+8.3f} {std_v:8.3f} {rms_v:8.3f} {p95:8.3f}")

  return {
    'total_frames': total_frames,
    'active_frames': active_frames,
    'rms': rms,
    'per_bucket': bucket_errors,
  }


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--max-segments', type=int, default=None,
                      help="Limit to first N drivelog segments (default: all)")
  args = parser.parse_args()
  run_comparison(max_segments=args.max_segments)


if __name__ == '__main__':
  main()
