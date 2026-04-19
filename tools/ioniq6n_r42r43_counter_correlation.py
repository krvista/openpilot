#!/usr/bin/env python3
"""Correlate `suppress_lfa` (0x362) COUNTER/cadence anomalies with
LFA-disappearance indicators on the actual routes 42 + 43 that the
user reported. Goal: establish (or refute) causal link before shipping
the counter-decouple fix.

LFA-disappearance proxies (in order of strength):
  1. onroadEvents[] containing 'steerTempUnavailable', 'steerUnavailable',
     'controlsUnresponsive', 'processNotRunning', 'selfdrivedLagging',
     'ldw', 'cruiseMismatch' (MADS-benign but counted for context).
  2. carState.cruiseState transitions / carState.steeringPressed flips.
  3. selfdriveState.enabled falling edge that is NOT user-initiated.

For each anomaly (counter gap or cadence violation), checks whether an
LFA-disappearance proxy fires within ±500 ms.
"""
import glob
import sys
from collections import Counter, defaultdict

sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG_DIR = '/home/user/openpilot/drivelog'
ROUTES = ['0000002a', '0000002b']
CORR_WINDOW_MS = 500.0  # ±500 ms matched-event window
SUPPRESS_ADDR = 0x362   # CAM_0x362 (HDA2-ALT)
SUPPRESS_PERIOD_MS = 50.0  # we TX at frame%5==0 of 100Hz → 20 Hz
GAP_THRESHOLD_MS = 75.0    # 1.5× expected


def load_route_segments(route_id):
  paths = sorted(glob.glob(
    f'{DRIVELOG_DIR}/99b215d21bbf8735_{route_id}--*--rlog.zst'))
  def seg_num(p):
    name = p.rsplit('/', 1)[-1]
    try: return int(name.split('--')[2])
    except: return 0
  return sorted(paths, key=seg_num)


def scan_route(route_id):
  segs = load_route_segments(route_id)
  print(f'\n=== Route {route_id}: {len(segs)} segments ===')
  if not segs:
    return None

  suppress_frames = []          # (t_ms, counter)
  onroad_fault_events = []       # (t_ms, event_name)
  steering_pressed_rising = []   # (t_ms,)
  enabled_falling = []           # (t_ms,)
  prev_enabled = None
  prev_press = None

  FAULT_EVENTS = {
    'steerTempUnavailable', 'steerUnavailable', 'controlsUnresponsive',
    'processNotRunning', 'selfdrivedLagging', 'ldw', 'cruiseMismatch',
    'accFaulted', 'lateralPlannerSolutionNaN', 'steerOverride',
  }

  t0_global = None
  for seg in segs:
    try:
      lr = LogReader(seg)
    except Exception:
      continue
    for m in lr:
      try:
        w = m.which()
      except Exception:
        continue
      if t0_global is None:
        t0_global = m.logMonoTime
      t_ms = (m.logMonoTime - t0_global) / 1e6

      if w == 'can':
        for c in m.can:
          if c.address == SUPPRESS_ADDR and c.src == 0 and len(c.dat) > 2:
            suppress_frames.append((t_ms, c.dat[2]))
      elif w == 'onroadEvents':
        for ev in m.onroadEvents:
          name = ev.name if hasattr(ev, 'name') else str(ev)
          if name in FAULT_EVENTS:
            onroad_fault_events.append((t_ms, name))
      elif w == 'carState':
        cs = m.carState
        # Steering override pressed rising
        p = bool(cs.steeringPressed)
        if prev_press is False and p is True:
          steering_pressed_rising.append((t_ms,))
        prev_press = p
      elif w == 'selfdriveState':
        en = bool(m.selfdriveState.enabled)
        if prev_enabled is True and en is False:
          enabled_falling.append((t_ms,))
        prev_enabled = en

  # ---- Analyze suppress TX ----
  gaps, counter_jumps = [], []
  for i in range(1, len(suppress_frames)):
    dt = suppress_frames[i][0] - suppress_frames[i-1][0]
    if dt > GAP_THRESHOLD_MS:
      gaps.append((suppress_frames[i-1][0], suppress_frames[i][0], dt))
    cprev, ccur = suppress_frames[i-1][1], suppress_frames[i][1]
    # Expect +1 per camera-native tick; we downsample, so tolerate +1..+3
    diff = (ccur - cprev) & 0xFF
    if diff == 0 or diff > 4:
      counter_jumps.append((suppress_frames[i][0], cprev, ccur, diff))

  # ---- Correlate ----
  anomaly_times = sorted(
    [g[1] for g in gaps] + [cj[0] for cj in counter_jumps])
  proxy_times = sorted(
    [t for (t, _) in onroad_fault_events] +
    [t for (t,) in enabled_falling])

  matched, unmatched_anomaly, unmatched_proxy = 0, 0, 0
  used_proxy_idx = set()
  for at in anomaly_times:
    best = None
    for idx, pt in enumerate(proxy_times):
      if idx in used_proxy_idx: continue
      if abs(pt - at) <= CORR_WINDOW_MS:
        if best is None or abs(pt - at) < abs(proxy_times[best] - at):
          best = idx
    if best is not None:
      matched += 1
      used_proxy_idx.add(best)
    else:
      unmatched_anomaly += 1
  unmatched_proxy = len(proxy_times) - len(used_proxy_idx)

  # ---- Report ----
  print(f'  suppress_lfa frames:     {len(suppress_frames)}')
  print(f'  cadence gaps (>{GAP_THRESHOLD_MS:.0f}ms): {len(gaps)}')
  if gaps[:5]:
    for g in gaps[:5]:
      print(f'    t≈{g[1]:9.1f}ms  gap={g[2]:5.1f}ms')
  print(f'  counter anomalies (diff=0 or >4): {len(counter_jumps)}')
  if counter_jumps[:5]:
    for cj in counter_jumps[:5]:
      print(f'    t≈{cj[0]:9.1f}ms  {cj[1]:#04x}→{cj[2]:#04x} (Δ={cj[3]})')

  ev_counts = Counter(n for _, n in onroad_fault_events)
  print(f'  onroadEvent faults ({len(onroad_fault_events)} total): {dict(ev_counts)}')
  print(f'  enabled-falling edges: {len(enabled_falling)}')
  print(f'  anomaly→proxy within ±{CORR_WINDOW_MS:.0f}ms:')
  print(f'    matched:             {matched}')
  print(f'    unmatched anomalies: {unmatched_anomaly}')
  print(f'    unmatched proxies:   {unmatched_proxy}')

  if matched == 0 and len(anomaly_times) > 0 and len(proxy_times) > 0:
    print('  ⚠ No temporal correlation — counter/gap anomaly likely NOT the fault trigger.')
  elif matched > 0:
    ratio = matched / max(len(anomaly_times), 1)
    print(f'  → correlation ratio (matched/anomalies) = {ratio:.2%}')

  return {
    'route': route_id,
    'n_suppress': len(suppress_frames),
    'n_gaps': len(gaps),
    'n_counter_anom': len(counter_jumps),
    'n_fault_events': len(onroad_fault_events),
    'n_enable_falls': len(enabled_falling),
    'matched': matched,
    'unmatched_anomaly': unmatched_anomaly,
    'unmatched_proxy': unmatched_proxy,
    'fault_breakdown': dict(ev_counts),
  }


if __name__ == '__main__':
  summaries = []
  for r in ROUTES:
    s = scan_route(r)
    if s: summaries.append(s)

  print('\n\n========== SUMMARY ==========')
  for s in summaries:
    print(f'Route {s["route"]}: frames={s["n_suppress"]}, '
          f'gaps={s["n_gaps"]}, counter_anom={s["n_counter_anom"]}, '
          f'faults={s["n_fault_events"]}, enable_falls={s["n_enable_falls"]}, '
          f'matched={s["matched"]}')
    if s['fault_breakdown']:
      print(f'  fault breakdown: {s["fault_breakdown"]}')
