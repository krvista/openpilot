#!/usr/bin/env python3
"""Aggregate SCC-V analysis results (per-route + fleet) and emit summary."""
import json
import sys
from collections import Counter, defaultdict

DEFAULT_PATH = '/tmp/drivelog_analysis/scc_v_results.json'
SCCV_STATES = ['disabled', 'enabled', 'entering', 'turning', 'leaving', 'overriding']
PLAN_SOURCES = ['cruise', 'sccVision', 'sccMap', 'speedLimitAssist']


def main():
  path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PATH
  with open(path) as f:
    results = json.load(f)

  errors = [r for r in results if r.get('error')]
  if errors:
    print(f'NOTE: {len(errors)}/{len(results)} segments errored during read', file=sys.stderr)
    print(f'      first: {errors[0]["error"][:120]}', file=sys.stderr)

  routes = defaultdict(lambda: {
    'segs': 0, 'dur': 0.0,
    'state_dur': defaultdict(float),
    'sccv_total': 0, 'sccv_active': 0,
    'entering_trig': 0, 'enter_to_turn': 0, 'enter_abort': 0,
    'pred_lat_acc_max': 0.0,
    'cur_lat_acc_max': 0.0,
    'plan_src': Counter(), 'plan_src_total': 0,
    'events': Counter(),
    'events_imm_dis': Counter(),
    'events_soft_dis': Counter(),
    'vego_sum': 0.0, 'vego_n': 0, 'vego_max': 0.0,
    'cruise_enabled_samples': 0,
    'engaged_samples': 0,
  })

  for r in results:
    rt = r['route']; m = routes[rt]
    m['segs'] += 1
    m['dur'] += r['duration_s']
    for s, d_ in r.get('sccv_state_durations_s', {}).items():
      m['state_dur'][s] += d_
    m['sccv_total'] += r['sccv_total_samples']
    m['sccv_active'] += r['sccv_active_samples']
    m['entering_trig'] += r['sccv_entering_trigger_count']
    m['enter_to_turn'] += r['sccv_entering_to_turning']
    m['enter_abort'] += r['sccv_entering_aborted']
    m['pred_lat_acc_max'] = max(m['pred_lat_acc_max'], r['sccv_max_pred_lat_acc_max'])
    m['cur_lat_acc_max'] = max(m['cur_lat_acc_max'], r['sccv_cur_lat_acc_max'])
    for s, n in r['plan_source_samples'].items():
      m['plan_src'][s] += n
    m['plan_src_total'] += r['plan_source_total']
    for k, v in r['events'].items():
      m['events'][k] += v
    for k, v in r['events_immediate_disable'].items():
      m['events_imm_dis'][k] += v
    for k, v in r['events_soft_disable'].items():
      m['events_soft_dis'][k] += v
    m['vego_sum'] += r['vego_sum_mps']
    m['vego_n'] += r['vego_samples_count']
    m['vego_max'] = max(m['vego_max'], r['vego_max_mps'])
    m['cruise_enabled_samples'] += r['cruise_enabled_samples']
    m['engaged_samples'] += r['engaged_samples']

  # Per-route table
  print('=' * 130)
  print('SCC-V state durations per route (seconds)')
  print('=' * 130)
  hdr = f'{"route":<48} {"segs":>5} {"dur_min":>8} {"vEgo_avg":>9} {"vEgo_max":>9} {"engaged":>9}  ' + ' '.join(f'{s:>10}' for s in SCCV_STATES)
  print(hdr)
  for rt, m in routes.items():
    vego_avg = (m['vego_sum'] / max(m['vego_n'], 1)) * 3.6
    vego_max = m['vego_max'] * 3.6
    row = f'{rt[:48]:<48} {m["segs"]:>5} {m["dur"]/60:>8.1f} {vego_avg:>9.1f} {vego_max:>9.1f} {m["engaged_samples"]:>9}  '
    row += ' '.join(f'{m["state_dur"].get(s, 0):>10.1f}' for s in SCCV_STATES)
    print(row)

  print()
  print('=' * 130)
  print('SCC-V entering trigger pattern per route')
  print('=' * 130)
  hdr = f'{"route":<48} {"entering":>10} {"->turning":>11} {"->aborted":>11} {"conv_rate":>10} {"pred_max":>9} {"cur_max":>8}'
  print(hdr)
  for rt, m in routes.items():
    conv = 100 * m['enter_to_turn'] / max(m['entering_trig'], 1)
    print(f'{rt[:48]:<48} {m["entering_trig"]:>10} {m["enter_to_turn"]:>11} {m["enter_abort"]:>11} {conv:>9.1f}% {m["pred_lat_acc_max"]:>9.2f} {m["cur_lat_acc_max"]:>8.2f}')

  print()
  print('=' * 130)
  print('Plan source occupancy per route')
  print('=' * 130)
  for rt, m in routes.items():
    total = max(m['plan_src_total'], 1)
    parts = ', '.join(f'{s}: {100*m["plan_src"][s]/total:.2f}%' for s in PLAN_SOURCES)
    print(f'  {rt[:48]}: {parts}')

  # Fleet
  print()
  print('=' * 130)
  print('FLEET aggregate (4 routes, 126 segments)')
  print('=' * 130)
  fleet_dur = sum(m['dur'] for m in routes.values())
  fleet_state = defaultdict(float)
  fleet_events = Counter()
  fleet_imm = Counter()
  fleet_soft = Counter()
  fleet_engaged = 0
  fleet_entering = 0
  fleet_e2t = 0
  fleet_eabort = 0
  for m in routes.values():
    for s, d_ in m['state_dur'].items():
      fleet_state[s] += d_
    for k, v in m['events'].items():
      fleet_events[k] += v
    for k, v in m['events_imm_dis'].items():
      fleet_imm[k] += v
    for k, v in m['events_soft_dis'].items():
      fleet_soft[k] += v
    fleet_engaged += m['engaged_samples']
    fleet_entering += m['entering_trig']
    fleet_e2t += m['enter_to_turn']
    fleet_eabort += m['enter_abort']
  print(f'Total duration: {fleet_dur/60:.1f} min ({fleet_dur/3600:.2f} hr)')
  print(f'engaged_samples (selfdriveState.enabled): {fleet_engaged}')
  print(f'SCC-V state durations (s):')
  for s in SCCV_STATES:
    pct = 100 * fleet_state[s] / max(fleet_dur, 1)
    print(f'  {s:<12}: {fleet_state[s]:>10.1f} s  ({pct:>6.2f}% of total drive time)')
  active = fleet_state['entering'] + fleet_state['turning'] + fleet_state['leaving']
  print(f'  -- active (e+t+l): {active:.1f} s ({100*active/max(fleet_dur,1):.3f}%)')
  print(f'Entering triggers: {fleet_entering}  -> turning: {fleet_e2t}  -> aborted: {fleet_eabort}  conv_rate={100*fleet_e2t/max(fleet_entering,1):.1f}%')
  print(f'Per-hour entering rate: {fleet_entering / (fleet_dur/3600):.2f}')
  print()
  print('=' * 130)
  print('Fleet top events (rising-edge count, NOT samples)')
  print('=' * 130)
  for name, cnt in fleet_events.most_common(25):
    print(f'  {name:<32} {cnt:>6}')
  print()
  print(f'immediateDisable: {dict(fleet_imm)}')
  print(f'softDisable: {dict(fleet_soft)}')
  fix_targets = {k: fleet_events.get(k, 0) for k in ('accFaulted', 'controlsMismatch', 'processNotRunning')}
  print()
  print(f'*** FIX TARGETS (Fix A/B/C trigger events) ***')
  for k, v in fix_targets.items():
    status = 'OK' if v == 0 else 'STILL TRIGGERING'
    print(f'  {k:<24} {v:>4}   [{status}]')


if __name__ == '__main__':
  main()
