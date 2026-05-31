#!/usr/bin/env python3
"""
Drivelog quantitative analysis for 2020 Jeep GC review plan:
  T3-B: Boot-time ACC active timing distribution (route segment 0).
  T1-B: LKAS fault frequency (steerFaultTemporary, steerFaultPermanent).
  T2-B: CRUISE_BUTTONS tx byte0 distribution sampled from sendcan.
  T3-C: DAS_3/DAS_4 gap detection (qlog-sample-based, lower bound only).

All from existing qlog files in /tmp/drivelog_analysis/drivelog/.
"""
import sys, os, glob, json, re
from collections import defaultdict, Counter

sys.modules['smbus2'] = type(sys)('smbus2')
sys.modules['smbus2'].SMBus = object
sys.modules['serial'] = type(sys)('serial')
from openpilot.tools.lib.logreader import _LogFileReader

QDIR = '/tmp/drivelog_analysis/drivelog'

def seg_key(fn):
    m = re.match(r'(.+)--(\d+)--qlog\.zst$', os.path.basename(fn))
    return (m.group(1), int(m.group(2))) if m else ('', 0)

def parse_seg(fn):
    m = re.match(r'(.+)--(\d+)--qlog\.zst$', os.path.basename(fn))
    return m.group(1), int(m.group(2))

# Per-segment results
results = []
files = sorted(glob.glob(f'{QDIR}/*qlog.zst'), key=seg_key)
print(f'Analyzing {len(files)} qlogs...', file=sys.stderr)

for i, fn in enumerate(files):
    if i % 100 == 0:
        print(f'  {i}/{len(files)}', file=sys.stderr)
    try:
        msgs = sorted(list(_LogFileReader(fn)), key=lambda m: m.logMonoTime)
    except Exception as e:
        print(f'  SKIP {fn}: {e}', file=sys.stderr)
        continue
    if not msgs:
        continue
    t0 = msgs[0].logMonoTime
    route, seg = parse_seg(fn)

    rec = {
        'route': route,
        'seg': seg,
        'duration': (msgs[-1].logMonoTime - t0) / 1e9,
        # T3-B: timing of first ACC_ACTIVE=1 and first ACC_AVAILABLE=1 (segment 0 only)
        'first_acc_active_t': None,
        'first_acc_avail_t': None,
        'acc_active_at_t0_5': None,  # was ACC active within first 5s?
        # T1-B: LKAS faults (count of distinct fault-rising transitions, total True samples)
        'steer_fault_temp_rising': 0,
        'steer_fault_temp_samples_true': 0,
        'steer_fault_perm_rising': 0,
        'steer_fault_perm_samples_true': 0,
        # T2-B: CRUISE_BUTTONS sendcan byte0 distribution
        'cruise_btn_tx_count': 0,
        'cruise_btn_byte0_hist': {},  # byte0 hex -> count
        # T3-C: largest gap in carState (qlog-only proxy for CAN freshness)
        'cs_max_gap_ms': 0,
    }

    prev_temp = False
    prev_perm = False
    prev_cs_ts = None
    for m in msgs:
        ts = (m.logMonoTime - t0) / 1e9
        which = m.which()
        if which == 'carState':
            cs = m.carState
            # T1-B
            if cs.steerFaultTemporary and not prev_temp:
                rec['steer_fault_temp_rising'] += 1
            if cs.steerFaultPermanent and not prev_perm:
                rec['steer_fault_perm_rising'] += 1
            if cs.steerFaultTemporary:
                rec['steer_fault_temp_samples_true'] += 1
            if cs.steerFaultPermanent:
                rec['steer_fault_perm_samples_true'] += 1
            prev_temp = cs.steerFaultTemporary
            prev_perm = cs.steerFaultPermanent

            # T3-B - track first availability/active (only meaningful on seg 0)
            if cs.cruiseState.available and rec['first_acc_avail_t'] is None:
                rec['first_acc_avail_t'] = ts
            if cs.cruiseState.enabled and rec['first_acc_active_t'] is None:
                rec['first_acc_active_t'] = ts
            if ts < 5.0 and cs.cruiseState.enabled:
                rec['acc_active_at_t0_5'] = True

            # T3-C - carState gap
            if prev_cs_ts is not None:
                gap_ms = (ts - prev_cs_ts) * 1000
                if gap_ms > rec['cs_max_gap_ms']:
                    rec['cs_max_gap_ms'] = gap_ms
            prev_cs_ts = ts

        elif which == 'sendcan':
            # T2-B
            for c in m.sendcan:
                if c.address == 571:  # CRUISE_BUTTONS
                    rec['cruise_btn_tx_count'] += 1
                    if len(c.dat) >= 1:
                        b0 = f'{c.dat[0]:02x}'
                        rec['cruise_btn_byte0_hist'][b0] = rec['cruise_btn_byte0_hist'].get(b0, 0) + 1

    if rec['acc_active_at_t0_5'] is None:
        rec['acc_active_at_t0_5'] = False
    results.append(rec)

with open('/tmp/drivelog_analysis/quant_results.json', 'w') as f:
    json.dump(results, f, default=str, indent=1)
print(f'Done. Wrote {len(results)} records.', file=sys.stderr)
