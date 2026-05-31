#!/usr/bin/env python3
"""Aggregate rlog quant results per route + emit summary tables for
T2-B (CRUISE_BUTTONS) and T3-C (RX-check freshness)."""
import json
import sys
from collections import Counter, defaultdict

DEFAULT_PATH = '/tmp/drivelog_analysis/rlog_results.json'
RX_ADDRS = [
  (0x140, 50,  "ESP_1"),
  (0x1F4, 50,  "DAS_3"),
  (0x202, 100, "0x202"),
  (0x220, 100, "EPS_2"),
  (0x22F, 50,  "ECM_5"),
  (0x330, 1,   "TRACTION_BUTTON"),
]


def main():
  path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PATH
  with open(path) as f:
    results = json.load(f)

  errors = [r for r in results if r.get('error')]
  if errors:
    print(f'NOTE: {len(errors)} segments errored during read', file=sys.stderr)

  by_route = defaultdict(list)
  for r in results:
    by_route[r['route']].append(r)
  routes = sorted(by_route.keys())

  # ====================================================================
  # T2-B  CRUISE_BUTTONS tx
  # ====================================================================
  print('=' * 130)
  print('T2-B  CRUISE_BUTTONS tx (0x23B) per route — rlog full-rate sample (CORRECTED bit positions)')
  print('=' * 130)
  hdr = f'{"route":<42} {"segs":>5} {"tx_total":>10} {"boot_<10s":>10} {"rx_total":>10} {"cancel":>7} {"resume":>7} {"accel":>7} {"decel":>7} {"d_dec":>6} {"d_inc":>6} {"crOnOff":>8} {"accOnOff":>9}'
  print(hdr)
  totals = Counter()
  for route in routes:
    segs = by_route[route]
    tx = sum(r['cb_tx_total'] for r in segs)
    boot = sum(r['cb_boot_storm_count_t_lt_10s'] for r in segs)
    rx = sum(r.get('cb_rx_count', 0) for r in segs)
    sigs = Counter()
    for r in segs:
      sigs.update(r['cb_signal_counts'])
    print(f'  {route:<40} {len(segs):>5} {tx:>10} {boot:>10} {rx:>10} {sigs["cancel"]:>7} {sigs["resume"]:>7} {sigs["accel"]:>7} {sigs["decel"]:>7} {sigs.get("dist_dec",0):>6} {sigs.get("dist_inc",0):>6} {sigs.get("cruise_onoff",0):>8} {sigs.get("acc_onoff",0):>9}')
    totals['tx_total'] += tx
    totals['boot'] += boot
    totals['rx_total'] += rx
    for k, v in sigs.items():
      totals[k] += v
  print(f'  {"TOTAL":<40} {len(routes):>5} {totals["tx_total"]:>10} {totals["boot"]:>10} {totals["rx_total"]:>10} {totals["cancel"]:>7} {totals["resume"]:>7} {totals["accel"]:>7} {totals["decel"]:>7} {totals.get("dist_dec",0):>6} {totals.get("dist_inc",0):>6} {totals.get("cruise_onoff",0):>8} {totals.get("acc_onoff",0):>9}')

  # COUNTER delta histogram (entire fleet)
  print()
  print('=' * 110)
  print('T2-B  CRUISE_BUTTONS COUNTER delta histogram (fleet aggregate)')
  print('=' * 110)
  print('  delta value = (next_counter - prev_counter) mod 16')
  print('  ICBM icbm.py:43 says button_counter_offset = [1, 1, 0, None] -> with')
  print('  CS.button_counter cycling +1 per RX (50Hz stock), expect TX deltas:')
  print('    +1 (offset 1 -> offset 1 between sends) ~33%')
  print('    +0 (offset 1 -> offset 0)               ~33%')
  print('    +3 (offset 0 -> skip -> offset 1)       ~33%')
  print('    others ~0%')
  fleet_delta = Counter()
  for r in results:
    for k, v in r.get('cb_counter_delta_hist', {}).items():
      fleet_delta[int(k)] += v
  total = sum(fleet_delta.values()) or 1
  print(f'  {"delta":>8}  {"count":>10}  {"pct":>8}')
  for d in sorted(fleet_delta):
    pct = 100.0 * fleet_delta[d] / total
    print(f'  {d:>8}  {fleet_delta[d]:>10}  {pct:>7.2f}%')
  print(f'  {"TOTAL":>8}  {total:>10}')

  # Sample COUNTER sequence from richest segment
  print()
  print('=' * 110)
  print('T2-B  COUNTER sequence sample (segment with most cb_tx_total)')
  print('=' * 110)
  best = max(results, key=lambda r: r['cb_tx_total'])
  print(f'  segment: {best["route"]} seg {best["seg"]}  tx_total={best["cb_tx_total"]}')
  print(f'  first 80: {best["cb_counter_seq_first200"][:80]}')

  # ====================================================================
  # T3-C  RX-check freshness
  # ====================================================================
  print()
  print('=' * 110)
  print('T3-C  RX-check freshness per route (chrysler_rx_checks non-RAM)')
  print('=' * 110)
  for addr, hz, name in RX_ADDRS:
    thresh_ms = 3000.0 / hz
    print()
    print(f'--- {name}  (addr=0x{addr:03x}={addr}, expected {hz} Hz, threshold = 3/Hz = {thresh_ms:.1f} ms)')
    hdr = f'  {"route":<42} {"segs":>5} {"recv_total":>11} {"avg_p99_ms":>11} {"max_gap_ms":>11} {"max_gap_seg":>13} {">100ms":>8} {">500ms":>8} {">3/Hz":>8}'
    print(hdr)
    for route in routes:
      segs = by_route[route]
      total_recv = sum(r['rx'].get(str(addr), r['rx'].get(addr, {})).get('count', 0) for r in segs)
      p99s = [r['rx'].get(str(addr), r['rx'].get(addr, {})).get('iat_p99_ms', 0)
              for r in segs if r['rx'].get(str(addr), r['rx'].get(addr, {})).get('count', 0) > 100]
      avg_p99 = sum(p99s) / len(p99s) if p99s else 0
      worst = max(segs, key=lambda r: r['rx'].get(str(addr), r['rx'].get(addr, {})).get('iat_max_ms', 0))
      worst_gap = worst['rx'].get(str(addr), worst['rx'].get(addr, {})).get('iat_max_ms', 0)
      worst_seg = worst['seg']
      gap100 = sum(r['rx'].get(str(addr), r['rx'].get(addr, {})).get('gap_over_100ms', 0) for r in segs)
      gap500 = sum(r['rx'].get(str(addr), r['rx'].get(addr, {})).get('gap_over_500ms', 0) for r in segs)
      gapT  = sum(r['rx'].get(str(addr), r['rx'].get(addr, {})).get('gap_over_threshold', 0) for r in segs)
      print(f'  {route:<40} {len(segs):>5} {total_recv:>11} {avg_p99:>11.2f} {worst_gap:>11.2f} {worst_seg:>13} {gap100:>8} {gap500:>8} {gapT:>8}')

  # Boot-grace fidelity: when did first RX arrive per route segment 0?
  print()
  print('=' * 110)
  print('T3-C  First RX timestamp per route segment 0 (boot grace window check)')
  print('=' * 110)
  hdr = f'  {"route":<42}'
  for _, _, name in RX_ADDRS:
    hdr += f' {name[:10]:>11}'
  hdr += f' {"safety_inv":>11} {"first_inv_t":>12}'
  print(hdr)
  for route in routes:
    segs = sorted(by_route[route], key=lambda r: r['seg'])
    s0 = segs[0]
    row = f'  {route:<40}'
    for addr, _, _ in RX_ADDRS:
      ft = s0['rx'].get(str(addr), s0['rx'].get(addr, {})).get('first_t', None)
      row += f' {("%.3f"%ft if ft is not None else "-"):>11}'
    row += f' {s0["safety_rx_invalid_rising"]:>11}'
    fit = s0.get('safety_rx_invalid_first_t')
    row += f' {("%.3f"%fit if fit is not None else "-"):>12}'
    print(row)

  # safetyRxChecksInvalid aggregated
  print()
  print('=' * 110)
  print('T3-C  safetyRxChecksInvalid summary per route')
  print('=' * 110)
  hdr = f'  {"route":<42} {"segs":>5} {"total_rise":>11} {"total_true_samples":>20} {"max_per_seg":>12}'
  print(hdr)
  for route in routes:
    segs = by_route[route]
    rise = sum(r['safety_rx_invalid_rising'] for r in segs)
    samp = sum(r['safety_rx_invalid_samples_true'] for r in segs)
    mxp  = max(r['safety_rx_invalid_rising'] for r in segs)
    print(f'  {route:<40} {len(segs):>5} {rise:>11} {samp:>20} {mxp:>12}')


if __name__ == '__main__':
  main()
