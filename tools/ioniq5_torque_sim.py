#!/usr/bin/env python3
"""Ioniq 5 (torque-control) cross-simulator for Ioniq 6 N angle-control logs.

Purpose: the Ioniq 6 N production stack is ANGLE-controlled; the Ioniq 5 uses
TORQUE control. To validate whether our Ioniq 6 N tuning (rate table, override
blending, low-speed hysteresis) matches a well-understood reference, we replay
the Ioniq 6 N drive log through a simplified full Ioniq 5 torque stack and
synthesize a virtual wheel angle trajectory. Side-by-side against the actual
Ioniq 6 N wheel angle reveals whether openpilot's angle command would have
been tracked at least as well by a torque controller tuned for Ioniq 5.

Scope: pseudoreference, not bit-exact. Plant is a first-order torque→angle lag
with steady-state gain from the published Ioniq 5 latAccelFactor. Sufficient
for qualitative A/B; not a substitute for on-car validation.

Usage:
  python3 tools/ioniq5_torque_sim.py /tmp/ccnc_drivelog_full/<route>--*--rlog.zst
  python3 tools/ioniq5_torque_sim.py  # defaults to route 0000002d
"""
import sys, os, glob, math
import numpy as np
from collections import defaultdict

sys.path.insert(0, '/home/user/openpilot')
sys.path.insert(0, '/home/user/openpilot/opendbc_repo')

from openpilot.tools.lib.logreader import LogReader


# ---- Ioniq 5 published parameters ----
# opendbc/car/hyundai/values.py:428-430 + torque_data/params.toml:27
I5_MASS_KG = 1948.0
I5_WHEELBASE_M = 2.97
I5_STEER_RATIO = 14.26
I5_TIRE_STIFFNESS_FACTOR = 0.65
I5_LAT_ACCEL_FACTOR = 3.172929      # lat_accel = torque * latAccelFactor
I5_FRICTION = 0.096019
I5_STEER_ACTUATOR_DELAY_S = 0.10

# ---- PID gains (selfdrive/controls/lib/latcontrol_torque.py:25-29, Hyundai default) ----
I5_KP_SPEED_BP   = [1.0, 1.5, 2.0, 3.0, 5.0, 7.5, 10.0, 15.0, 30.0]  # m/s
I5_KP_SPEED_VAL  = [250, 120, 65,   30,  11.5, 5.5, 3.5,  2.0,  0.8]
I5_KP_BASE = 0.8
I5_KI_BASE = 0.15
FRICTION_THRESHOLD = 0.2             # m/s² accel error within deadzone — linear friction interp
JERK_GAIN = 0.2                      # unused in simplified sim

# ---- Plant model ----
# First-order lag: d(wheel_rate)/dt = (K_plant * torque - wheel_rate) / tau
# tau ≈ steerActuatorDelay (0.1 s). K_plant derived from steady-state:
# at v_ego > 5 m/s, steady-state lat_accel = torque * latAccelFactor
# steering wheel angle producing that lat_accel via bicycle: angle = lat_accel
# * steerRatio * (wheelbase / v²) * (180/π)
# So K_plant (deg/s per unit_torque) depends on v_ego — we re-derive per frame.
I5_PLANT_TAU_S = 0.12                # first-order torque→angle-rate lag
I5_PLANT_DAMPING = 3.0               # deg/s per deg (self-aligning + EPS damping)


def pid_gains(v_ego: float):
  """Linear-interp speed schedule matching Hyundai default."""
  kp_scale = float(np.interp(v_ego, I5_KP_SPEED_BP, I5_KP_SPEED_VAL))
  return I5_KP_BASE * kp_scale, I5_KI_BASE * kp_scale


def friction(error, threshold=FRICTION_THRESHOLD):
  """Piecewise-linear friction compensation (latcontrol_torque.py get_friction)."""
  if abs(error) < threshold:
    return I5_FRICTION * (error / threshold)
  return I5_FRICTION * (1.0 if error > 0 else -1.0)


def curvature_from_angle(angle_deg: float, v_ego: float):
  """Bicycle model steady-state: curvature = angle_rad / (steerRatio * wheelbase) in the
  low-speed limit; at higher speed add understeer. Simplified — slip neglected."""
  angle_rad = math.radians(angle_deg) / I5_STEER_RATIO
  return angle_rad / I5_WHEELBASE_M


def angle_from_curvature(curvature: float, v_ego: float):
  """Inverse of above (no slip)."""
  angle_rad = curvature * I5_WHEELBASE_M
  return math.degrees(angle_rad * I5_STEER_RATIO)


class TorquePID:
  def __init__(self):
    self.i = 0.0
    self.last_t = None

  def update(self, error, v_ego, ff, dt=0.01, freeze_integrator=False):
    kp, ki = pid_gains(v_ego)
    p = kp * error
    if not freeze_integrator:
      self.i = float(np.clip(self.i + ki * error * dt, -2.5, 2.5))
    return ff + p + self.i


class TorquePlant:
  """Simplified EPS plant: torque_cmd → wheel_angle (deg).

  Uses first-order lag with steady-state gain matching the published
  latAccelFactor. Small damping term keeps things stable at standstill.
  """
  def __init__(self):
    self.wheel_angle = 0.0
    self.wheel_rate = 0.0  # deg/s

  def step(self, torque_cmd, v_ego, dt=0.01):
    # Steady-state: torque → lateral accel → curvature → wheel angle
    # angle_ss (deg) such that cmd_torque * latAccelFactor = v² * curvature_ss
    # and curvature_ss = (angle_ss_rad / steerRatio) / wheelbase
    v2 = max(v_ego, 0.5) ** 2
    angle_ss_rad = torque_cmd * I5_LAT_ACCEL_FACTOR / v2 * I5_WHEELBASE_M * I5_STEER_RATIO
    angle_ss_deg = math.degrees(angle_ss_rad)
    # First-order approach: target rate = (angle_ss - angle) / tau,
    # damp by -wheel_rate proportional.
    target_rate = (angle_ss_deg - self.wheel_angle) / I5_PLANT_TAU_S
    self.wheel_rate += (target_rate - I5_PLANT_DAMPING * self.wheel_rate) * dt / I5_PLANT_TAU_S
    # Clamp rate to a sane EPS limit (matches published STEER_ANGLE_RATE_MAX ~150°/s)
    self.wheel_rate = float(np.clip(self.wheel_rate, -150.0, 150.0))
    self.wheel_angle += self.wheel_rate * dt
    return self.wheel_angle


def simulate_route(log_paths, verbose=False):
  pid = TorquePID()
  plant = TorquePlant()
  metrics = defaultdict(list)
  last_cs = None

  for path in log_paths:
    # Initialize plant to first observed angle so the transient at log start
    # doesn't dominate the metrics.
    init = True
    for msg in LogReader(path):
      w = msg.which()
      if w != 'carControl':
        if w == 'carState':
          last_cs = msg.carState
          if init and last_cs.vEgo > 0:
            plant.wheel_angle = float(last_cs.steeringAngleDeg)
            plant.wheel_rate  = float(last_cs.steeringRateDeg)
            init = False
        continue
      if last_cs is None:
        continue
      cc = msg.carControl
      actuators = cc.actuators
      if actuators.curvature is None:
        continue

      v_ego   = float(last_cs.vEgo)
      actual  = float(last_cs.steeringAngleDeg)
      desired_curv = float(actuators.curvature)
      desired_lat_accel = desired_curv * v_ego * v_ego

      # Simulate Ioniq 5 torque loop (single 10 ms step per carControl message)
      measured_curv = curvature_from_angle(plant.wheel_angle, v_ego)
      measured_lat_accel = measured_curv * v_ego * v_ego
      err = desired_lat_accel - measured_lat_accel
      ff  = desired_lat_accel + friction(err)
      if not bool(cc.latActive):
        torque_cmd = 0.0
        plant.wheel_angle = actual  # snap when not actively steering (equivalent to clutch-out)
        plant.wheel_rate  = float(last_cs.steeringRateDeg)
        continue
      # PID operates in lat-accel units, convert to torque divider
      u_lat_accel = pid.update(err, v_ego, ff,
                               freeze_integrator=bool(last_cs.steeringPressed) or v_ego < 5)
      torque_cmd = float(np.clip(u_lat_accel / I5_LAT_ACCEL_FACTOR, -1.0, 1.0))
      virt_angle = plant.step(torque_cmd, v_ego)

      # Metrics bucket by speed
      v_kmh = v_ego * 3.6
      if v_kmh < 20:     bucket = '0-20'
      elif v_kmh < 40:   bucket = '20-40'
      elif v_kmh < 60:   bucket = '40-60'
      elif v_kmh < 80:   bucket = '60-80'
      else:              bucket = '80+'
      metrics[bucket].append((virt_angle, actual, desired_curv, v_ego))

  # Report
  print(f'\n{"Bucket":8s} {"Frames":>8s} {"|err|_p50":>10s} {"|err|_p95":>10s} {"|err|_p99":>10s}   I5_torque_sim_vs_actual_I6N (deg)')
  for bucket in ('0-20', '20-40', '40-60', '60-80', '80+'):
    rows = metrics.get(bucket, [])
    if len(rows) < 100:
      print(f'{bucket:8s} {len(rows):>8d}   (too few frames)')
      continue
    virt = np.array([r[0] for r in rows])
    actual = np.array([r[1] for r in rows])
    err = np.abs(virt - actual)
    print(f'{bucket:8s} {len(rows):>8d} {np.percentile(err,50):>10.2f} {np.percentile(err,95):>10.2f} {np.percentile(err,99):>10.2f}')

  return metrics


def main():
  if len(sys.argv) > 1:
    paths = sorted(sys.argv[1:])
  else:
    paths = sorted(glob.glob('/tmp/ccnc_drivelog_full/*0000002d*rlog.zst'))
  print(f'Simulating Ioniq 5 torque stack on {len(paths)} log file(s)...')
  simulate_route(paths, verbose=True)
  print('\nNote: metrics compare a *virtual* I5 torque controller output vs the\n'
        'actual I6N wheel trajectory in the log. Large errors at low speed are\n'
        'expected because the plant model is a first-order lag; treat as\n'
        'qualitative comparison only.')


if __name__ == '__main__':
  main()
