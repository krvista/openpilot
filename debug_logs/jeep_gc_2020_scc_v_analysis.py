#!/usr/bin/env python3
"""SCC-V state machine + planner source + onroad event sweep
for Jeep GC drivelog v2 (4-route corpus on v2026.001.005 + our fixes).

Per-segment record:
  - SCC-V vision state time-share (disabled/enabled/entering/turning/leaving/overriding)
  - SCC-V vTarget vs cruiseState.speed (lowered duration + mean delta)
  - maxPredictedLateralAccel + currentLateralAccel distributions
  - entering trigger count + entering->turning transition rate
  - longitudinalPlanSource occupancy
  - onroadEvents EventName counter (immediate/soft disable filter)
  - carState basics: vEgo mean, cruiseState.speed mean
"""
import argparse
import glob
import json
import os
import re
import sys
from collections import Counter

sys.modules['smbus2'] = type(sys)('smbus2')
sys.modules['smbus2'].SMBus = object
sys.modules['serial'] = type(sys)('serial')
from openpilot.tools.lib.logreader import _LogFileReader

NAME_RE = re.compile(r'([0-9a-f]{16})_([0-9a-f]+)--([0-9a-f]+)--(\d+)--rlog\.zst$')

SCCV_STATES = ['disabled', 'enabled', 'entering', 'turning', 'leaving', 'overriding']
PLAN_SOURCES = ['cruise', 'sccVision', 'sccMap', 'speedLimitAssist']


def percentile(xs, p):
  if not xs:
    return 0.0
  xs = sorted(xs)
  idx = int(round((len(xs) - 1) * p))
  return xs[idx]


def process_segment(path):
  m = NAME_RE.search(path)
  if not m:
    return None
  dongle, route_hex, route_uuid, seg = m.group(1), m.group(2), m.group(3), int(m.group(4))
  route = f"{dongle}_{route_hex}--{route_uuid}"

  rec = {
    'route': route,
    'seg': seg,
    'path': os.path.basename(path),
    'duration_s': 0.0,
    # SCC-V vision state
    'sccv_state_samples': {s: 0 for s in SCCV_STATES},
    'sccv_total_samples': 0,
    'sccv_entering_trigger_count': 0,    # disabled/enabled -> entering rising edges
    'sccv_entering_to_turning': 0,        # entering -> turning transitions
    'sccv_entering_aborted': 0,            # entering -> enabled (no turn realized)
    'sccv_active_samples': 0,              # entering+turning+leaving (= active)
    'sccv_vtarget_min': None,
    'sccv_vtarget_max': None,
    'sccv_lower_than_cruise_samples': 0,   # samples where sccv.vTarget < cruise_set - 0.5 m/s
    'sccv_lower_than_cruise_sum_delta_mps': 0.0,
    'sccv_max_pred_lat_acc_p50': 0.0,
    'sccv_max_pred_lat_acc_p95': 0.0,
    'sccv_max_pred_lat_acc_p99': 0.0,
    'sccv_max_pred_lat_acc_max': 0.0,
    'sccv_cur_lat_acc_p95': 0.0,
    'sccv_cur_lat_acc_max': 0.0,
    # Plan source
    'plan_source_samples': {s: 0 for s in PLAN_SOURCES},
    'plan_source_total': 0,
    # Events from selfdrived
    'events': Counter(),
    'events_immediate_disable': Counter(),
    'events_soft_disable': Counter(),
    'events_user_disable': Counter(),
    'events_no_entry': Counter(),
    # carState
    'vego_samples_count': 0,
    'vego_sum_mps': 0.0,
    'vego_max_mps': 0.0,
    'cruise_speed_samples_count': 0,
    'cruise_speed_sum_mps': 0.0,
    'cruise_enabled_samples': 0,
    # engagement
    'engaged_samples': 0,                  # selfdriveState.enabled
    't0_mono': None,
    't_last': None,
  }

  max_pred_lat_acc = []
  cur_lat_acc = []
  prev_sccv_state = None
  prev_event_names_set = set()
  t0 = None
  prev_t = None
  state_last_t = None
  state_durations = {s: 0.0 for s in SCCV_STATES}

  try:
    lr = _LogFileReader(path)
  except Exception as e:
    rec['error'] = f'LogReader open: {type(e).__name__}: {e}'
    return rec

  try:
    for msg in lr:
      try:
        t_ns = msg.logMonoTime
      except Exception:
        continue
      if t0 is None:
        t0 = t_ns
        rec['t0_mono'] = t0
      t = (t_ns - t0) / 1e9
      rec['t_last'] = t

      which = msg.which()

      if which == 'longitudinalPlanSP':
        plan = msg.longitudinalPlanSP
        # SCC-V vision
        try:
          v = plan.smartCruiseControl.vision
          state_name = str(v.state)
        except Exception:
          state_name = None
        if state_name in SCCV_STATES:
          rec['sccv_state_samples'][state_name] += 1
          rec['sccv_total_samples'] += 1
          if state_name in ('entering', 'turning', 'leaving'):
            rec['sccv_active_samples'] += 1

          # state duration accumulation
          if state_last_t is not None and prev_sccv_state is not None:
            state_durations[prev_sccv_state] = state_durations.get(prev_sccv_state, 0.0) + (t - state_last_t)
          state_last_t = t

          # transitions
          if state_name == 'entering' and prev_sccv_state in (None, 'disabled', 'enabled'):
            rec['sccv_entering_trigger_count'] += 1
          if state_name == 'turning' and prev_sccv_state == 'entering':
            rec['sccv_entering_to_turning'] += 1
          if state_name == 'enabled' and prev_sccv_state == 'entering':
            rec['sccv_entering_aborted'] += 1

          prev_sccv_state = state_name

          # vTarget
          try:
            vt = float(v.vTarget)
          except Exception:
            vt = None
          if vt is not None and state_name != 'disabled':
            if rec['sccv_vtarget_min'] is None or vt < rec['sccv_vtarget_min']:
              rec['sccv_vtarget_min'] = vt
            if rec['sccv_vtarget_max'] is None or vt > rec['sccv_vtarget_max']:
              rec['sccv_vtarget_max'] = vt

          try:
            mpla = float(v.maxPredictedLateralAccel)
            max_pred_lat_acc.append(mpla)
          except Exception:
            pass
          try:
            cla = float(v.currentLateralAccel)
            cur_lat_acc.append(cla)
          except Exception:
            pass

        # Plan source
        try:
          src_name = str(plan.longitudinalPlanSource)
        except Exception:
          src_name = None
        if src_name in PLAN_SOURCES:
          rec['plan_source_samples'][src_name] += 1
          rec['plan_source_total'] += 1

      elif which in ('onroadEvents', 'onroadEventsSP'):
        try:
          if which == 'onroadEvents':
            ev_list = msg.onroadEvents
          else:
            ev_list = msg.onroadEventsSP.events
        except Exception:
          ev_list = []
        cur_event_names = set()
        for ev in ev_list:
          try:
            name = str(ev.name)
          except Exception:
            continue
          cur_event_names.add(name)
          # count rising-edge only
          if name not in prev_event_names_set:
            rec['events'][name] += 1
            try:
              if ev.immediateDisable:
                rec['events_immediate_disable'][name] += 1
              if ev.softDisable:
                rec['events_soft_disable'][name] += 1
              if ev.userDisable:
                rec['events_user_disable'][name] += 1
              if ev.noEntry:
                rec['events_no_entry'][name] += 1
            except Exception:
              pass
        prev_event_names_set = cur_event_names

      elif which == 'carState':
        cs = msg.carState
        try:
          ve = float(cs.vEgo)
          rec['vego_sum_mps'] += ve
          rec['vego_samples_count'] += 1
          if ve > rec['vego_max_mps']:
            rec['vego_max_mps'] = ve
        except Exception:
          pass
        try:
          cspeed = float(cs.cruiseState.speed)
          rec['cruise_speed_sum_mps'] += cspeed
          rec['cruise_speed_samples_count'] += 1
        except Exception:
          pass
        try:
          if cs.cruiseState.enabled:
            rec['cruise_enabled_samples'] += 1
        except Exception:
          pass

        # sccv vTarget vs cruise comparison (only if SCC-V active in same window)
        # done per-frame using last seen sccv vTarget — approximation
        # (cheaper than full time-align). skip cross-msg correlation for now.

      elif which == 'selfdriveState':
        try:
          if msg.selfdriveState.enabled:
            rec['engaged_samples'] += 1
        except Exception:
          pass

  except Exception as e:
    rec['error'] = f'iter: {type(e).__name__}: {e}'

  rec['duration_s'] = rec['t_last'] if rec['t_last'] is not None else 0.0

  if max_pred_lat_acc:
    rec['sccv_max_pred_lat_acc_p50'] = round(percentile(max_pred_lat_acc, 0.50), 4)
    rec['sccv_max_pred_lat_acc_p95'] = round(percentile(max_pred_lat_acc, 0.95), 4)
    rec['sccv_max_pred_lat_acc_p99'] = round(percentile(max_pred_lat_acc, 0.99), 4)
    rec['sccv_max_pred_lat_acc_max'] = round(max(max_pred_lat_acc), 4)
  if cur_lat_acc:
    rec['sccv_cur_lat_acc_p95'] = round(percentile(cur_lat_acc, 0.95), 4)
    rec['sccv_cur_lat_acc_max'] = round(max(cur_lat_acc), 4)

  rec['sccv_state_durations_s'] = {s: round(d, 3) for s, d in state_durations.items()}

  # Convert Counters to plain dicts for JSON
  for k in ('events', 'events_immediate_disable', 'events_soft_disable',
            'events_user_disable', 'events_no_entry'):
    rec[k] = dict(rec[k])

  return rec


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--drivelog-dir', required=True)
  ap.add_argument('--out', required=True)
  ap.add_argument('--limit', type=int, default=0)
  args = ap.parse_args()

  paths = sorted(glob.glob(os.path.join(args.drivelog_dir, '*--rlog.zst')))
  if args.limit:
    paths = paths[:args.limit]
  print(f'Found {len(paths)} rlog files', file=sys.stderr)

  os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
  out_records = []
  for i, p in enumerate(paths):
    if i % 25 == 0:
      print(f'  {i}/{len(paths)}', file=sys.stderr)
    rec = process_segment(p)
    if rec is not None:
      out_records.append(rec)

  with open(args.out, 'w') as f:
    json.dump(out_records, f)
  print(f'Wrote {len(out_records)} records to {args.out}', file=sys.stderr)


if __name__ == '__main__':
  main()
