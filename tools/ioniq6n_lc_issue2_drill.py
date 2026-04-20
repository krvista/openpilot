#!/usr/bin/env python3
"""Drill-down into Issue ② (slow post-LC centering) unsettled events.

For each laneChangeFinishing→off event where angle doesn't settle
within 3s, dump:
  - the post-LC 3s angle time-series (target, actual, error)
  - blinker state (another LC queued?)
  - curvature / angle trajectory (entering a curve?)
  - speed, steering pressed
  - lane_change_ll_prob recovery timing (from laneChangeFinishing duration)
"""
import sys, os, glob, math
import numpy as np
from collections import defaultdict

sys.path.insert(0, '/home/user/openpilot')
sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG = "/home/user/openpilot/drivelog"
DT_CTRL = 0.01

POST_LC_WINDOW_S  = 3.0
SETTLE_THRESH_DEG = 1.5
SETTLE_DWELL_S    = 0.5


def route_files():
  routes = defaultdict(list)
  for f in sorted(glob.glob(os.path.join(DRIVELOG, "*rlog.zst"))):
    base = os.path.basename(f)
    parts = base.split('--')
    if len(parts) < 4: continue
    route = parts[0]
    try: seg = int(parts[2])
    except ValueError: continue
    routes[route].append((seg, f))
  for r in routes: routes[r].sort()
  return routes


def extract_full(path):
  t = []
  lc_state = []
  v_ego = []
  lb = []; rb = []
  sp = []; st = []
  ang_des = []; ang_now = []
  active = []

  last_lc = 'off'
  last_v = 0.0; last_lb = False; last_rb = False
  last_sp = False; last_st = 0.0; last_ang = 0.0
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
        last_sp = cs.steeringPressed
        last_st = cs.steeringTorque
        last_ang = cs.steeringAngleDeg
      elif w == 'controlsState':
        lat = msg.controlsState.lateralControlState
        if lat.which() == 'angleState':
          ts = msg.logMonoTime * 1e-9
          if init_t is None: init_t = ts
          t.append(ts - init_t)
          lc_state.append(last_lc)
          v_ego.append(last_v)
          lb.append(last_lb); rb.append(last_rb)
          sp.append(last_sp); st.append(last_st)
          ang_des.append(float(lat.angleState.steeringAngleDesiredDeg))
          ang_now.append(last_ang)
          active.append(bool(lat.angleState.active))
    except Exception:
      pass

  if not t: return None
  return dict(
    t=np.array(t), lc=np.array(lc_state, dtype=object),
    v=np.array(v_ego),
    lb=np.array(lb), rb=np.array(rb),
    sp=np.array(sp), st=np.array(st),
    ang_des=np.array(ang_des), ang_now=np.array(ang_now),
    active=np.array(active),
  )


def find_runs(lc):
  runs = []
  i = 0
  while i < len(lc):
    j = i
    while j < len(lc) and lc[j] == lc[i]: j += 1
    runs.append((i, j, lc[i]))
    i = j
  return runs


def analyze_post_lc(data, i0_finish, i1_finish, i0_off, i1_off, seg_label, event_id, ri, runs):
  n_off = i1_off - i0_off
  w_end = min(i1_off, i0_off + int(POST_LC_WINDOW_S / DT_CTRL))
  sl = slice(i0_off, w_end)
  n = w_end - i0_off

  if n < 10: return False

  target = data['ang_des'][sl]
  actual = data['ang_now'][sl]
  err = np.abs(target - actual)
  act = data['active'][sl]

  if act.sum() < 5: return False

  # Check if settled
  dwell_n = int(SETTLE_DWELL_S / DT_CTRL)
  settled = False
  settle_time = None
  for k in range(len(err) - dwell_n):
    if np.all(err[k:k + dwell_n] < SETTLE_THRESH_DEG):
      settled = True
      settle_time = k * DT_CTRL
      break

  if settled:
    return False  # Only report unsettled events

  # === Unsettled event — dump details ===
  finish_dur = (i1_finish - i0_finish) * DT_CTRL
  v = data['v'][sl]
  blink_l = data['lb'][sl]; blink_r = data['rb'][sl]
  pressed = data['sp'][sl]; torque = data['st'][sl]
  lc_post = data['lc'][sl]

  # Was there another LC queued (blinker on during post-LC)?
  blinker_on_post = (blink_l | blink_r).any()
  # Did LC state re-enter preLaneChange during the window?
  re_entered_pre = any(s != 'off' for s in lc_post)
  # Was there a curve? (large absolute desired angle)
  max_abs_target = np.abs(target).max()
  mean_abs_target = np.abs(target).mean()
  # Driver intervened?
  driver_intervened = pressed.any()

  # Error statistics
  err_mean = err[act].mean()
  err_max  = err[act].max()
  err_p95  = np.percentile(err[act], 95)

  # Angle direction trend (is target sweeping one direction = curve)
  target_trend = target[-1] - target[0]

  print(f"\n  ── Unsettled Event {event_id}: {seg_label} ──")
  print(f"     Finishing duration:    {finish_dur:.2f}s")
  print(f"     Post-LC window:        {n * DT_CTRL:.1f}s  ({n} frames)")
  print(f"     Error:                 mean={err_mean:.2f}°  p95={err_p95:.2f}°  max={err_max:.2f}°")
  print(f"     Speed:                 avg={v.mean()*3.6:.1f} km/h  range=[{v.min()*3.6:.1f}, {v.max()*3.6:.1f}]")
  print(f"     Target angle:          mean|target|={mean_abs_target:.1f}°  max|target|={max_abs_target:.1f}°  trend={target_trend:+.1f}°")
  print(f"     Blinker on post-LC:    {blinker_on_post}")
  print(f"     Re-entered LC state:   {re_entered_pre}")
  print(f"     Driver intervened:     {driver_intervened} (peak torque={torque[np.argmax(np.abs(torque))]:.1f} Nm)")
  print(f"     Context:               [Finishing] → [off]", end='')
  if ri + 1 < len(runs):
    print(f" → [{runs[ri + 1][2]}]")
  else:
    print()

  # Classification
  reasons = []
  if re_entered_pre:
    reasons.append('ANOTHER_LC_QUEUED')
  if mean_abs_target > 5.0:
    reasons.append(f'CURVE (mean|target|={mean_abs_target:.1f}°)')
  if driver_intervened:
    reasons.append('DRIVER_OVERRIDE')
  if blinker_on_post and not re_entered_pre:
    reasons.append('BLINKER_STILL_ON')
  if not reasons:
    reasons.append('PURE_CENTERING_LAG')
  print(f"     ⚡ CAUSE:              {', '.join(reasons)}")

  # Compact time-series: every 0.5s
  step = max(1, int(0.5 / DT_CTRL))
  indices = list(range(0, n, step))
  if indices[-1] != n - 1: indices.append(n - 1)
  print(f"     Time →  ", end='')
  for idx in indices:
    print(f"{idx * DT_CTRL:5.1f}s", end=' ')
  print()
  print(f"     Target: ", end='')
  for idx in indices:
    print(f"{target[idx]:+5.1f}°", end=' ')
  print()
  print(f"     Actual: ", end='')
  for idx in indices:
    print(f"{actual[idx]:+5.1f}°", end=' ')
  print()
  print(f"     Error:  ", end='')
  for idx in indices:
    print(f"{err[idx]:5.2f}°", end=' ')
  print()

  return True


def main():
  routes = route_files()
  print(f"=== Issue ② deep-dive: unsettled post-LC centering ===")
  print(f"Routes: {len(routes)}, segments: {sum(len(v) for v in routes.values())}")
  print(f"Settle criterion: |error| < {SETTLE_THRESH_DEG}° for {SETTLE_DWELL_S}s within {POST_LC_WINDOW_S}s")

  event_id = 0
  unsettled = 0
  settled = 0
  done = 0
  total_segs = sum(len(v) for v in routes.values())

  for route, segs in sorted(routes.items()):
    for seg_idx, path in segs:
      done += 1
      try:
        data = extract_full(path)
      except Exception:
        continue
      if data is None: continue
      runs = find_runs(data['lc'])
      for ri, (i0, i1, state) in enumerate(runs):
        # Look for Finishing → off transitions
        if state != 'off': continue
        if ri < 1: continue
        prev_i0, prev_i1, prev_s = runs[ri - 1]
        if prev_s != 'laneChangeFinishing': continue

        event_id += 1
        seg_label = f"{route[-4:]} seg {seg_idx}"
        was_unsettled = analyze_post_lc(
          data, prev_i0, prev_i1, i0, i1, seg_label, event_id, ri, runs)
        if was_unsettled:
          unsettled += 1
        else:
          settled += 1

      if done % 50 == 0:
        print(f"  [{done}/{total_segs}] events={event_id} settled={settled} unsettled={unsettled}")

  print(f"\n=== Total: {event_id} post-LC events, {settled} settled, {unsettled} unsettled ===")


if __name__ == '__main__':
  main()
