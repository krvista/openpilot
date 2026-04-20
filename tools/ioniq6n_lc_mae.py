#!/usr/bin/env python3
"""Lane-change segment MAE analysis across all drivelog routes.

Focus areas (user-reported issues):
  Issue 2/3: car doesn't steer well on curves AND lane-change centering.

This tool isolates LANE-CHANGE events and reports:

  A) preLaneChange → Starting      (Issue ③: "LC doesn't start")
     - preLaneChange duration per event
     - outcome distribution (started vs timed-out/off)
     - torque/blindspot/speed at decision point

  B) During LC (Starting + Finishing)
     - angle-tracking MAE target vs achieved
     - Fix E (curvature LPF τ=0.20) overlay — simulated

  C) Post-LC centering                (Issue ②: slow centering)
     - MAE for first 3s after LC → off
     - settling time until |err| stays < 1.5° for 500 ms

LC state is read from modelV2.meta.laneChangeState.  Angle target/achieved
come from controlsState.lateralControlState.angleState.

Run:  python3 tools/ioniq6n_lc_mae.py
"""
import sys, os, glob, math
import numpy as np
from collections import Counter, defaultdict

sys.path.insert(0, '/home/user/openpilot')
sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
from openpilot.tools.lib.logreader import LogReader
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.structs import CarParams
from opendbc.car.hyundai.values import CAR as HCAR

DRIVELOG = "/home/user/openpilot/drivelog"
DT_CTRL = 0.01
DT_MDL  = 0.05   # modelV2 at 20Hz

LPF_TAU = 0.20   # Fix E tau (s)

POST_LC_WINDOW_S   = 3.0
SETTLE_THRESH_DEG  = 1.5
SETTLE_DWELL_S     = 0.5


def build_vm_i6n():
  specs = HCAR.HYUNDAI_IONIQ_6_N.config.specs
  cp = CarParams.new_message()
  cp.mass = specs.mass
  cp.wheelbase = specs.wheelbase
  cp.steerRatio = specs.steerRatio
  cp.centerToFront = specs.wheelbase * specs.centerToFrontRatio
  cp.steerRatioRear = 0.0
  cp.tireStiffnessFactor = specs.tireStiffnessFactor
  cp.rotationalInertia = specs.mass * specs.wheelbase * specs.wheelbase * 0.35
  cp.tireStiffnessFront = 1.9e5 * specs.tireStiffnessFactor
  cp.tireStiffnessRear  = 1.9e5 * specs.tireStiffnessFactor
  return VehicleModel(cp)


def route_files():
  routes = defaultdict(list)
  for f in sorted(glob.glob(os.path.join(DRIVELOG, "*rlog.zst"))):
    base = os.path.basename(f)
    parts = base.split('--')
    if len(parts) < 4: continue
    route = parts[0]
    try:
      seg = int(parts[2])
    except ValueError:
      continue
    routes[route].append((seg, f))
  for r in routes:
    routes[r].sort()
  return routes


def extract_seg(path, vm):
  """Walk log, emit aligned arrays at 100 Hz.  LC state held-over from
  the most recent modelV2 frame.  Returns dict of equal-length numpy arrays."""
  t = []          # mono time (s)
  lc_state = []   # string
  v_ego = []
  roll = []
  left_blink = []
  right_blink = []
  left_bs = []    # blindspot
  right_bs = []
  steering_pressed = []
  steering_torque = []
  angle_des = []  # lat target (deg)
  angle_now = []  # current steering angle (deg)
  angle_active = []

  last_lc = 'off'
  last_v = 0.0
  last_roll = 0.0
  last_lb, last_rb = False, False
  last_lbs, last_rbs = False, False
  last_sp, last_st = False, 0.0
  last_ang = 0.0
  init_t = None

  for msg in LogReader(path):
    w = msg.which()
    try:
      if w == 'modelV2':
        last_lc = str(msg.modelV2.meta.laneChangeState)
      elif w == 'carState':
        cs = msg.carState
        last_v = cs.vEgo
        last_lb, last_rb = cs.leftBlinker, cs.rightBlinker
        last_lbs, last_rbs = cs.leftBlindspot, cs.rightBlindspot
        last_sp = cs.steeringPressed
        last_st = cs.steeringTorque
        last_ang = cs.steeringAngleDeg
      elif w == 'liveParameters':
        try: last_roll = float(msg.liveParameters.roll)
        except Exception: pass
      elif w == 'controlsState':
        lat = msg.controlsState.lateralControlState
        if lat.which() == 'angleState':
          ang = lat.angleState
          ts = msg.logMonoTime * 1e-9
          if init_t is None: init_t = ts
          t.append(ts - init_t)
          lc_state.append(last_lc)
          v_ego.append(last_v)
          roll.append(last_roll)
          left_blink.append(last_lb)
          right_blink.append(last_rb)
          left_bs.append(last_lbs)
          right_bs.append(last_rbs)
          steering_pressed.append(last_sp)
          steering_torque.append(last_st)
          angle_des.append(float(ang.steeringAngleDesiredDeg))
          angle_now.append(last_ang)
          angle_active.append(bool(ang.active))
    except Exception:
      pass

  if not t:
    return None
  return dict(
    t=np.array(t), lc=np.array(lc_state, dtype=object),
    v=np.array(v_ego), roll=np.array(roll),
    lb=np.array(left_blink), rb=np.array(right_blink),
    lbs=np.array(left_bs), rbs=np.array(right_bs),
    sp=np.array(steering_pressed), st=np.array(steering_torque),
    ang_des=np.array(angle_des), ang_now=np.array(angle_now),
    active=np.array(angle_active),
  )


def find_transitions(lc):
  """Return list of (start_idx, end_idx_exclusive, state_name) runs."""
  if len(lc) == 0: return []
  runs = []
  i = 0
  while i < len(lc):
    j = i
    while j < len(lc) and lc[j] == lc[i]: j += 1
    runs.append((i, j, lc[i]))
    i = j
  return runs


def lpf(series, tau, dt):
  if tau < 1e-4: return series.copy()
  alpha = dt / (tau + dt)
  y = np.zeros_like(series)
  y[0] = series[0]
  for k in range(1, len(series)):
    y[k] = alpha * series[k] + (1 - alpha) * y[k - 1]
  return y


def analyze_seg(data, stats):
  runs = find_transitions(data['lc'])
  n = len(data['lc'])
  if n < 10: return

  # Per-sample LPF of desired angle (proxy for Fix E on post-VM angle)
  ang_des_lpf = lpf(data['ang_des'], LPF_TAU, DT_CTRL)

  # ---- Iterate runs of preLaneChange + laneChangeStarting/Finishing + off ----
  for ri, (i0, i1, state) in enumerate(runs):
    dur = (i1 - i0) * DT_CTRL

    # (A) preLaneChange outcome analysis
    if state == 'preLaneChange':
      next_state = runs[ri + 1][2] if ri + 1 < len(runs) else 'off'
      # Capture conditions at end of preLaneChange
      idx = i1 - 1
      torque_applied_left  = data['sp'][idx] and data['st'][idx] > 0
      torque_applied_right = data['sp'][idx] and data['st'][idx] < 0
      blindspot = (data['lb'][idx] and data['lbs'][idx]) or \
                  (data['rb'][idx] and data['rbs'][idx])
      stats['pre_durations'].append(dur)
      stats['pre_outcomes'][next_state] += 1
      if next_state == 'laneChangeStarting':
        stats['pre_to_start'].append(dict(dur=dur, v=data['v'][idx]))
      elif next_state == 'off':
        stats['pre_to_off'].append(dict(
          dur=dur, v=data['v'][idx],
          torque=(torque_applied_left or torque_applied_right),
          blindspot=blindspot,
          blinker_dropped=(not (data['lb'][idx] or data['rb'][idx])),
          low_speed=(data['v'][idx] < 9.0),
        ))

    # (B) During-LC tracking MAE
    if state in ('laneChangeStarting', 'laneChangeFinishing'):
      mask = np.arange(i0, i1)
      active = data['active'][mask]
      if active.sum() < 5: continue
      sub = mask[active]
      target = data['ang_des'][sub]
      now    = data['ang_now'][sub]
      err    = np.abs(target - now)
      err_lpf = np.abs(ang_des_lpf[sub] - now)
      stats[f'lc_{state}_n']   += len(sub)
      stats[f'lc_{state}_err'] += float(err.sum())
      stats[f'lc_{state}_err_lpf'] += float(err_lpf.sum())

    # (C) Post-LC centering (right after Finishing→off)
    if state == 'off' and ri > 0 and runs[ri - 1][2] == 'laneChangeFinishing':
      w_end = min(i1, i0 + int(POST_LC_WINDOW_S / DT_CTRL))
      mask = np.arange(i0, w_end)
      if len(mask) < 10: continue
      active = data['active'][mask]
      if active.sum() < 5: continue
      sub = mask[active]
      target = data['ang_des'][sub]
      now    = data['ang_now'][sub]
      err    = np.abs(target - now)
      err_lpf = np.abs(ang_des_lpf[sub] - now)
      stats['post_n']   += len(sub)
      stats['post_err'] += float(err.sum())
      stats['post_err_lpf'] += float(err_lpf.sum())
      # settling time
      dwell_n = int(SETTLE_DWELL_S / DT_CTRL)
      settled_idx = None
      for k in range(len(sub) - dwell_n):
        if np.all(err[k:k + dwell_n] < SETTLE_THRESH_DEG):
          settled_idx = k; break
      if settled_idx is not None:
        stats['post_settle'].append(settled_idx * DT_CTRL)
      else:
        stats['post_unsettled'] += 1


def emit_report(stats):
  print("\n" + "=" * 74)
  print(" LANE-CHANGE MAE ANALYSIS — Ioniq 6N (9 routes / 283 segs)")
  print("=" * 74)

  # (A) preLaneChange
  pre_n = len(stats['pre_durations'])
  print(f"\n[A] preLaneChange events: {pre_n}")
  if pre_n:
    pd = np.array(stats['pre_durations'])
    print(f"    duration   mean={pd.mean():.2f}s  median={np.median(pd):.2f}s  max={pd.max():.2f}s")
    print(f"    outcomes: {dict(stats['pre_outcomes'])}")
    if stats['pre_to_off']:
      tot = len(stats['pre_to_off'])
      torque = sum(1 for e in stats['pre_to_off'] if e['torque'])
      bs     = sum(1 for e in stats['pre_to_off'] if e['blindspot'])
      drop   = sum(1 for e in stats['pre_to_off'] if e['blinker_dropped'])
      low    = sum(1 for e in stats['pre_to_off'] if e['low_speed'])
      no_torque = sum(1 for e in stats['pre_to_off']
                     if not e['torque'] and not e['blindspot']
                        and not e['blinker_dropped'] and not e['low_speed'])
      print(f"    → off {tot} times:")
      print(f"        torque applied anyway: {torque}  ({100*torque/tot:.0f}%)")
      print(f"        blindspot warning:     {bs}  ({100*bs/tot:.0f}%)")
      print(f"        blinker dropped:       {drop}  ({100*drop/tot:.0f}%)")
      print(f"        below 32 km/h:         {low}  ({100*low/tot:.0f}%)")
      print(f"        unexplained abandon:   {no_torque}  ({100*no_torque/tot:.0f}%)")

  # (B) During-LC MAE
  print(f"\n[B] During-LC tracking MAE:")
  for s in ('laneChangeStarting', 'laneChangeFinishing'):
    n = stats[f'lc_{s}_n']
    if n == 0:
      print(f"    {s}: no data")
      continue
    mae_base = stats[f'lc_{s}_err'] / n
    mae_lpf  = stats[f'lc_{s}_err_lpf'] / n
    delta = mae_lpf - mae_base
    pct = 100.0 * delta / max(mae_base, 1e-6)
    print(f"    {s}: N={n}  MAE={mae_base:.3f}°  Fix E→{mae_lpf:.3f}°  "
          f"(Δ{delta:+.3f}°, {pct:+.1f}%)")

  # (C) Post-LC centering
  n = stats['post_n']
  print(f"\n[C] Post-LC centering (first {POST_LC_WINDOW_S:.0f}s after →off):")
  if n:
    mae_base = stats['post_err'] / n
    mae_lpf  = stats['post_err_lpf'] / n
    delta = mae_lpf - mae_base
    pct = 100.0 * delta / max(mae_base, 1e-6)
    print(f"    N={n}  MAE={mae_base:.3f}°  Fix E→{mae_lpf:.3f}°  "
          f"(Δ{delta:+.3f}°, {pct:+.1f}%)")
    events = len(stats['post_settle']) + stats['post_unsettled']
    if events:
      settled = len(stats['post_settle'])
      print(f"    settled <{SETTLE_THRESH_DEG:.1f}° for {SETTLE_DWELL_S:.1f}s: "
            f"{settled}/{events} events ({100*settled/events:.0f}%)")
      if stats['post_settle']:
        st = np.array(stats['post_settle'])
        print(f"    settling time    mean={st.mean():.2f}s  "
              f"median={np.median(st):.2f}s  p95={np.percentile(st, 95):.2f}s")
      print(f"    unsettled (>{POST_LC_WINDOW_S:.0f}s): {stats['post_unsettled']} events")
  else:
    print("    no post-LC samples")

  print("\n" + "=" * 74)


def main():
  vm = build_vm_i6n()
  routes = route_files()
  print(f"Found {len(routes)} routes, "
        f"{sum(len(v) for v in routes.values())} segments")

  stats = dict(
    pre_durations=[], pre_outcomes=Counter(),
    pre_to_start=[], pre_to_off=[],
    lc_laneChangeStarting_n=0, lc_laneChangeStarting_err=0.0,
    lc_laneChangeStarting_err_lpf=0.0,
    lc_laneChangeFinishing_n=0, lc_laneChangeFinishing_err=0.0,
    lc_laneChangeFinishing_err_lpf=0.0,
    post_n=0, post_err=0.0, post_err_lpf=0.0,
    post_settle=[], post_unsettled=0,
  )

  total_segs = sum(len(v) for v in routes.values())
  done = 0
  for route, segs in sorted(routes.items()):
    route_pre = 0
    route_lc = 0
    for seg_idx, path in segs:
      done += 1
      try:
        data = extract_seg(path, vm)
      except Exception as e:
        print(f"  {os.path.basename(path)}: ERR {e}")
        continue
      if data is None: continue
      before_pre = len(stats['pre_durations'])
      before_lc = stats['lc_laneChangeStarting_n']
      analyze_seg(data, stats)
      route_pre += len(stats['pre_durations']) - before_pre
      route_lc += (stats['lc_laneChangeStarting_n'] - before_lc)
      if done % 20 == 0:
        print(f"  [{done}/{total_segs}] pre={len(stats['pre_durations'])} "
              f"lc_starting_n={stats['lc_laneChangeStarting_n']} "
              f"post_n={stats['post_n']}")
    print(f"  route {route[-4:]}: preLC={route_pre} lc_start_samples={route_lc}")

  emit_report(stats)


if __name__ == '__main__':
  main()
