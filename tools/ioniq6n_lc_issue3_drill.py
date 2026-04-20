#!/usr/bin/env python3
"""Drill-down into Issue ③ (LC doesn't start) abandoned preLaneChange events.

For each preLaneChange→off event, dump:
  - steeringTorque time series through the pre period
  - blinker state, blindspot, speed
  - preceding & following LC states
  - torque peak, steeringPressed timing
"""
import sys, os, glob, math
import numpy as np
from collections import defaultdict

sys.path.insert(0, '/home/user/openpilot')
sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG = "/home/user/openpilot/drivelog"
DT_CTRL = 0.01


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
  lbs = []; rbs = []
  sp = []; st = []
  ang_des = []; ang_now = []
  active = []

  last_lc = 'off'
  last_v = 0.0; last_lb = False; last_rb = False
  last_lbs = False; last_rbs = False
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
        last_lbs, last_rbs = cs.leftBlindspot, cs.rightBlindspot
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
          lbs.append(last_lbs); rbs.append(last_rbs)
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
    lbs=np.array(lbs), rbs=np.array(rbs),
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


def analyze_abort(data, i0, i1, seg_label, event_id, ri, runs):
  n = i1 - i0
  dur = n * DT_CTRL
  sl = slice(i0, i1)

  torque = data['st'][sl]
  pressed = data['sp'][sl]
  v = data['v'][sl]
  l_b = data['lb'][sl]; r_b = data['rb'][sl]
  l_bs = data['lbs'][sl]; r_bs = data['rbs'][sl]

  peak_torque = torque[np.argmax(np.abs(torque))]
  any_pressed = pressed.any()
  avg_v = v.mean()
  min_v = v.min()

  # Direction from blinker
  l_on = l_b.any()
  r_on = r_b.any()
  direction = 'LEFT' if l_on and not r_on else ('RIGHT' if r_on and not l_on else 'BOTH/NONE')

  # Torque alignment with direction
  if direction == 'LEFT':
    torque_aligned = (torque > 0).any() and pressed.any()
  elif direction == 'RIGHT':
    torque_aligned = (torque < 0).any() and pressed.any()
  else:
    torque_aligned = False

  # Blindspot at any point during pre
  bs_left  = (l_b & l_bs).any()
  bs_right = (r_b & r_bs).any()
  any_bs = bs_left or bs_right

  # Blinker at end
  blinker_end = l_b[-1] or r_b[-1]

  # Context: what happened before and after
  prev_state = runs[ri - 1][2] if ri > 0 else 'N/A'
  next_state = runs[ri + 1][2] if ri + 1 < len(runs) else 'N/A'

  # Reason classification
  reasons = []
  if min_v < 9.0: reasons.append(f'LOW_SPEED ({min_v*3.6:.1f} km/h)')
  if any_bs: reasons.append('BLINDSPOT')
  if not blinker_end: reasons.append('BLINKER_DROPPED')
  if any_pressed and not torque_aligned: reasons.append('TORQUE_WRONG_DIR')
  if not any_pressed: reasons.append('NO_TORQUE')
  if torque_aligned and blinker_end and not any_bs and min_v >= 9.0:
    reasons.append('UNEXPLAINED — torque OK, blinker on, no BS, speed OK')

  print(f"\n  ── Event {event_id}: {seg_label} ──")
  print(f"     duration:     {dur:.2f}s  ({n} frames)")
  print(f"     direction:    {direction}")
  print(f"     speed:        avg={avg_v*3.6:.1f} km/h  min={min_v*3.6:.1f} km/h")
  print(f"     torque:       peak={peak_torque:.1f} Nm  pressed={any_pressed}  aligned={torque_aligned}")
  print(f"     blindspot:    L={bs_left}  R={bs_right}")
  print(f"     blinker@end:  {blinker_end}")
  print(f"     context:      [{prev_state}] → preLaneChange → [{next_state}]")
  print(f"     ⚡ REASON:    {', '.join(reasons) if reasons else 'UNKNOWN'}")

  # Torque time-series (compact)
  if n <= 30:
    t_str = ','.join(f'{t:.1f}' for t in torque)
    p_str = ','.join('T' if p else '.' for p in pressed)
    print(f"     torque ts:    [{t_str}]")
    print(f"     pressed ts:   [{p_str}]")
  else:
    # Print first 10 + last 10
    t1 = ','.join(f'{t:.1f}' for t in torque[:10])
    t2 = ','.join(f'{t:.1f}' for t in torque[-10:])
    p1 = ','.join('T' if p else '.' for p in pressed[:10])
    p2 = ','.join('T' if p else '.' for p in pressed[-10:])
    print(f"     torque:       [{t1} ... {t2}]")
    print(f"     pressed:      [{p1} ... {p2}]")


def main():
  routes = route_files()
  print(f"=== Issue ③ deep-dive: preLaneChange→off events ===")
  print(f"Routes: {len(routes)}, segments: {sum(len(v) for v in routes.values())}")

  event_id = 0
  total_segs = sum(len(v) for v in routes.values())
  done = 0

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
        if state != 'preLaneChange': continue
        next_s = runs[ri + 1][2] if ri + 1 < len(runs) else 'off'
        if next_s != 'off': continue
        event_id += 1
        seg_label = f"{route[-4:]} seg {seg_idx}"
        analyze_abort(data, i0, i1, seg_label, event_id, ri, runs)

  print(f"\n=== Total preLaneChange→off events: {event_id} ===")


if __name__ == '__main__':
  main()
