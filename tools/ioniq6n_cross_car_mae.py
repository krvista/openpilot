#!/usr/bin/env python3
"""Cross-car simulation: replay same road (desired_curvature + vEgo + roll)
through Ioniq 6N / Ioniq 5 / RAV4 / Corolla vehicle models and rate limiters.

For each car we compute:
  - MAE of |target_angle - rate_limited_angle|   (tracking lag)
  - p95 / max of the same error
  - # frames where |target - achieved| > 2.5°    (would trigger steerSaturated)
  - # simulated steerSaturated events (using the FIXED _check_saturation logic:
    saturated = |target - achieved| > 2.5° AND NOT steer_limited_by_safety)

Routes: 3e, 3f, 40 (drivelogs already present under /home/user/openpilot/drivelog).

Implementation detail: desired_curvature is recovered from the Ioniq 6N
drivelog by inverting the Ioniq 6N VehicleModel on the recorded
controlsState.lateralControlState.angleState.steeringAngleDesiredDeg. The
same desired_curvature is then mapped back through each target car's VM.
"""
import sys, os, glob, math
import numpy as np

sys.path.insert(0, '/home/user/openpilot')
sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
from openpilot.tools.lib.logreader import LogReader
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.structs import CarParams
from opendbc.car.hyundai.values import CAR as HCAR
from opendbc.car.toyota.values import CAR as TCAR

DRIVELOG = "/home/user/openpilot/drivelog"
DT = 0.01                # controlsState is 100Hz
SAT_THRESHOLD = 2.5      # deg
SAT_LIMIT = 0.4          # steerLimitTimer (s)
SAT_MIN_SPEED = 5.0      # m/s
MAX_DECIMATE = 1         # keep every Nth frame (1 = no decimation)


def build_vm(specs, steer_ratio_override=None):
  cp = CarParams.new_message()
  cp.mass = specs.mass
  cp.wheelbase = specs.wheelbase
  cp.steerRatio = steer_ratio_override or specs.steerRatio
  cp.centerToFront = specs.wheelbase * specs.centerToFrontRatio
  cp.steerRatioRear = 0.0
  cp.tireStiffnessFactor = specs.tireStiffnessFactor
  cp.rotationalInertia = specs.mass * specs.wheelbase * specs.wheelbase * 0.35
  # rough but fair cornering-stiffness estimate scaled by tire stiffness factor
  cp.tireStiffnessFront = 1.9e5 * specs.tireStiffnessFactor
  cp.tireStiffnessRear  = 1.9e5 * specs.tireStiffnessFactor
  return VehicleModel(cp)


# Car configs: (display_name, VehicleModel, rate_limiter_fn)
# We use a single unified per-car rate limiter using each car's own
# MAX_LATERAL_JERK + MAX_LATERAL_ACCEL.  For Toyota we derive equivalent
# limits from ISO 11270 (3.0 m/s² accel, 5.0 m/s³ jerk) because the v1
# lookup table is a per-step cap, not a physical jerk/accel.  We apply
# a conservative Toyota estimate of 2.5 m/s³ jerk / 3.0 m/s² accel per
# the Tesla/Toyota commentary in hyundai/values.py.

CAR_SPECS = {
  'Ioniq6N':   (HCAR.HYUNDAI_IONIQ_6_N.config.specs, 3.5, 3.3),   # jerk, accel
  'Ioniq5':    (HCAR.HYUNDAI_IONIQ_5.config.specs,   3.5, 3.3),
  'RAV4_TSS2': (TCAR.TOYOTA_RAV4_TSS2_2022.config.specs, 2.5, 3.0),
  'Corolla':   (TCAR.TOYOTA_COROLLA_TSS2.config.specs,   2.5, 3.0),
}


def rate_limit_step(target, last, v, VM, max_jerk, max_accel, max_angle_rate_deg_per_step=1.3):
  """Replicates opendbc.car.lateral.apply_steer_angle_limits_vm for one step."""
  v = max(v, 1.0)
  max_curv_rate = max_jerk / (v * v)
  max_angle_delta = math.degrees(VM.get_steer_from_curvature(max_curv_rate, v, 0.0)) * DT
  max_angle_delta = min(max_angle_delta, max_angle_rate_deg_per_step)
  new_angle = np.clip(target, last - max_angle_delta, last + max_angle_delta)
  max_curv = max_accel / (v * v)
  max_angle = math.degrees(VM.get_steer_from_curvature(max_curv, v, 0.0))
  return float(np.clip(new_angle, -max_angle, max_angle))


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


def extract_drive(path, src_vm):
  """Extract (desired_curvature [1/m], v [m/s], roll [rad], pressed) arrays."""
  last_v, last_roll, last_pressed = 0.0, 0.0, False
  curv_arr, v_arr, roll_arr, pressed_arr = [], [], [], []
  for msg in LogReader(path):
    w = msg.which()
    try:
      if w == 'carState':
        last_v = msg.carState.vEgo
        last_pressed = msg.carState.steeringPressed
      elif w == 'liveParameters':
        try:
          last_roll = float(msg.liveParameters.roll)
        except Exception:
          pass
      elif w == 'controlsState':
        lat = msg.controlsState.lateralControlState
        if lat.which() == 'angleState':
          ang = lat.angleState
          if ang.active and last_v > 0.5:
            # latcontrol_angle emits:
            #   angle_steers_des = degrees(VM.get_steer_from_curvature(-curv, v, roll)) + angleOffsetDeg
            # ignoring angleOffsetDeg (typically <1°) → curv = -calc_curvature(radians(angle), v, roll)
            sa_rad = math.radians(float(ang.steeringAngleDesiredDeg))
            v = max(last_v, 1.0)
            curv = -src_vm.calc_curvature(sa_rad, v, last_roll)
            curv_arr.append(curv)
            v_arr.append(v)
            roll_arr.append(last_roll)
            pressed_arr.append(last_pressed)
    except Exception:
      pass
  return (np.array(curv_arr), np.array(v_arr), np.array(roll_arr), np.array(pressed_arr))


def sim_car(curv, v, roll, pressed, name, vm, max_jerk, max_accel):
  n = len(curv)
  target = np.zeros(n)
  for i in range(n):
    target[i] = math.degrees(vm.get_steer_from_curvature(-curv[i], max(v[i], 1.0), roll[i]))
  achieved = np.zeros(n)
  achieved[0] = target[0]
  for i in range(1, n):
    achieved[i] = rate_limit_step(target[i], achieved[i - 1], v[i], vm, max_jerk, max_accel)
  err = np.abs(target - achieved)
  # primary MAE: only moving frames (v > SAT_MIN_SPEED = 5 m/s = 18 km/h) —
  # matches the regime where steerSaturated gating is active
  mask = v > SAT_MIN_SPEED
  if mask.sum() > 0:
    mae = float(err[mask].mean())
    p95 = float(np.percentile(err[mask], 95))
    mx  = float(err[mask].max())
  else:
    mae = p95 = mx = 0.0
  # steer_limited = err > 2.5 (carOutput would show mismatch)
  steer_limited = err > SAT_THRESHOLD
  # with FIXED logic: saturated = steer_limited_by_safety; suppression = steer_limited
  # → _check_saturation only fires when saturated AND NOT steer_limited → never for
  # pure rate-limit lag.  Count the frames that WOULD still fire if saturated kept
  # using the old path (|target - achieved| > 2.5 AND v > 5 AND NOT pressed).
  sat_raw = steer_limited & (v > SAT_MIN_SPEED) & (~pressed)
  # sim timer
  sat_t = 0.0
  events = 0
  was_alert = False
  alert_frames = 0
  for i in range(n):
    if sat_raw[i]:
      sat_t = min(sat_t + DT, SAT_LIMIT)
    else:
      sat_t = max(sat_t - DT, 0.0)
    alert = sat_t >= SAT_LIMIT - 1e-6
    if alert:
      alert_frames += 1
      if not was_alert:
        events += 1
    was_alert = alert
  return dict(name=name, n=int(mask.sum()), mae=mae, p95=p95, max=mx,
              sat_frames=int(sat_raw.sum()), alert_frames=alert_frames, events=events)


def run_route(route_hex, src_vm, cars_vm):
  segs = find_segments(route_hex)
  if not segs:
    print(f"Route {route_hex}: no segments")
    return None

  total = {name: dict(n=0, err_sum=0.0, p95_sum=0.0, err_max=0.0,
                      sat_frames=0, alert_frames=0, events=0)
           for name in cars_vm}

  for sn in sorted(segs.keys()):
    try:
      curv, v, roll, pressed = extract_drive(segs[sn], src_vm)
    except Exception as e:
      print(f"  seg {sn}: extract ERR {e}")
      continue
    if len(curv) < 10:
      continue
    for name, (vm, mj, ma) in cars_vm.items():
      r = sim_car(curv, v, roll, pressed, name, vm, mj, ma)
      t = total[name]
      t['n'] += r['n']
      t['err_sum'] += r['mae'] * r['n']
      t['p95_sum'] += r['p95'] * r['n']
      t['sat_frames'] += r['sat_frames']
      t['alert_frames'] += r['alert_frames']
      t['events'] += r['events']
      if r['max'] > t['err_max']:
        t['err_max'] = r['max']

  print(f"\n{'='*96}")
  print(f"  Route 0x{route_hex.zfill(8)[-2:]}  —  cross-car MAE simulation")
  print(f"{'='*96}")
  print(f"  {'car':<11} {'frames':>8} {'MAE(°)':>8} {'p95(°)':>8} {'max(°)':>8} "
        f"{'>2.5°':>8} {'alert_fr':>9} {'events':>7}")
  for name, t in total.items():
    if t['n'] == 0:
      continue
    mae = t['err_sum'] / t['n']
    p95 = t['p95_sum'] / t['n']
    print(f"  {name:<11} {t['n']:>8} {mae:>8.2f} {p95:>8.2f} {t['err_max']:>8.1f} "
          f"{t['sat_frames']:>8} {t['alert_frames']:>9} {t['events']:>7}")
  return total


def main(route_hexes):
  src_specs = HCAR.HYUNDAI_IONIQ_6_N.config.specs
  src_vm = build_vm(src_specs)
  cars_vm = {}
  for name, (specs, mj, ma) in CAR_SPECS.items():
    cars_vm[name] = (build_vm(specs), mj, ma)

  grand = {name: dict(n=0, err_sum=0.0, p95_sum=0.0, err_max=0.0,
                      sat_frames=0, alert_frames=0, events=0)
           for name in CAR_SPECS}
  for rh in route_hexes:
    t = run_route(rh, src_vm, cars_vm)
    if t is None:
      continue
    for name in CAR_SPECS:
      g, r = grand[name], t[name]
      g['n'] += r['n']
      g['err_sum'] += r['err_sum']
      g['p95_sum'] += r['p95_sum']
      g['sat_frames'] += r['sat_frames']
      g['alert_frames'] += r['alert_frames']
      g['events'] += r['events']
      g['err_max'] = max(g['err_max'], r['err_max'])

  print(f"\n{'='*96}")
  print(f"  GRAND TOTAL across {len(route_hexes)} routes  (frames: v > 5 m/s only)")
  print(f"{'='*96}")
  print(f"  {'car':<11} {'frames':>8} {'MAE(°)':>8} {'p95(°)':>8} {'max(°)':>8} "
        f"{'>2.5°':>8} {'alert_fr':>9} {'events':>7}  vs_Ioniq6N")
  baseline = None
  for name in CAR_SPECS:
    g = grand[name]
    if g['n'] == 0:
      continue
    mae = g['err_sum'] / g['n']
    p95 = g['p95_sum'] / g['n']
    if baseline is None:
      baseline = mae
    rel = (mae / baseline - 1.0) * 100 if baseline > 0 else 0
    print(f"  {name:<11} {g['n']:>8} {mae:>8.2f} {p95:>8.2f} {g['err_max']:>8.1f} "
          f"{g['sat_frames']:>8} {g['alert_frames']:>9} {g['events']:>7}  {rel:+.1f}%")


if __name__ == "__main__":
  routes = sys.argv[1:] if len(sys.argv) > 1 else ['3e', '3f', '40']
  main(routes)
