#!/usr/bin/env python3
"""Aggregate quant results by route and write summary."""
import json
from collections import defaultdict, Counter

with open('/tmp/drivelog_analysis/quant_results.json') as f:
    results = json.load(f)

by_route = defaultdict(list)
for r in results:
    by_route[r['route']].append(r)

routes_sorted = sorted(by_route.keys())

# T3-B: boot-time ACC active timing across segment 0 of each route
print("=" * 90)
print("T3-B  Boot-time ACC state across route segment 0")
print("=" * 90)
print(f'{"route":<46} {"first_avail":>10} {"first_active":>12} {"active_<5s":>10}')
for route in routes_sorted:
    segs = sorted(by_route[route], key=lambda r: r['seg'])
    s0 = segs[0]
    fa = s0['first_acc_avail_t']
    fac = s0['first_acc_active_t']
    fa_s = f'{fa:.2f}s' if fa is not None else '-'
    fac_s = f'{fac:.2f}s' if fac is not None else '-'
    print(f'  {route:<44} {fa_s:>10} {fac_s:>12} {str(s0["acc_active_at_t0_5"]):>10}')

# T1-B: LKAS fault frequency (totals per route)
print()
print("=" * 90)
print("T1-B  LKAS fault frequency per route")
print("=" * 90)
print(f'{"route":<46} {"segs":>5} {"temp_rise":>10} {"perm_rise":>10} {"temp_true_samples":>18} {"perm_true_samples":>18}')
for route in routes_sorted:
    segs = by_route[route]
    n = len(segs)
    tr = sum(r['steer_fault_temp_rising'] for r in segs)
    pr = sum(r['steer_fault_perm_rising'] for r in segs)
    ts = sum(r['steer_fault_temp_samples_true'] for r in segs)
    ps = sum(r['steer_fault_perm_samples_true'] for r in segs)
    print(f'  {route:<44} {n:>5} {tr:>10} {pr:>10} {ts:>18} {ps:>18}')

# T2-B: CRUISE_BUTTONS sendcan byte0 distribution per route
print()
print("=" * 90)
print("T2-B  CRUISE_BUTTONS tx byte0 distribution (qlog sample-based, sparse)")
print("=" * 90)
print(f'{"route":<46} {"total_tx":>10} top_byte0_hist (hex:count)')
for route in routes_sorted:
    segs = by_route[route]
    tot = sum(r['cruise_btn_tx_count'] for r in segs)
    hist = Counter()
    for r in segs:
        hist.update(r['cruise_btn_byte0_hist'])
    top = ', '.join(f'{k}:{v}' for k, v in sorted(hist.items(), key=lambda x: -x[1])[:5])
    print(f'  {route:<44} {tot:>10}  {top}')

# T3-C: max carState gap per route
print()
print("=" * 90)
print("T3-C  Max carState gap per route (qlog sample lower-bound, ms)")
print("=" * 90)
print(f'{"route":<46} {"segs":>5} {"max_gap_seg":>13} {"max_gap_ms":>12}')
for route in routes_sorted:
    segs = by_route[route]
    worst = max(segs, key=lambda r: r['cs_max_gap_ms'])
    print(f'  {route:<44} {len(segs):>5} {worst["seg"]:>13} {worst["cs_max_gap_ms"]:>12.1f}')
