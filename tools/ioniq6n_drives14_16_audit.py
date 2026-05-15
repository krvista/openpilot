#!/usr/bin/env python3
"""Patch #11 post-merge audit — drives 14, 15, 16 (commit e62a5257, branch i6n).

Scans all rlog segments for:
  - cloudlog.warning / .error strings (LFA_ICON transitions, FAULT_LFA, VM_LIMIT,
    snap_to_wheel, anything else op writes)
  - onroadEvents + onroadEventsSP (alerts surfaced to driver / suppressed)
  - panda safety counters (busOff, canSendErr, canFwdErr, faultsDetectedCount)
  - selfdriveState.alertText / mads state anomalies
  - carControl.actuators / carOutput sanity (NaN, oversized angles)

Output:
  - per-route summary table to stdout
  - /tmp/patch11_audit.json — full structured findings
"""
import glob
import json
import os
import re
import sys
from collections import Counter, defaultdict

import zstandard as zstd

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')
from cereal import log

DRIVELOG_DIR = '/home/user/openpilot/drivelog'
ROUTES = ('00000014', '00000015', '00000016')

# cloudlog keys we explicitly care about; everything else lumps into "other"
KNOWN_PATTERNS = [
    ('LFA_ICON',     re.compile(r'LFA_ICON transition')),
    ('FAULT_LFA',    re.compile(r'FAULT_LFA')),
    ('VM_LIMIT',     re.compile(r'VM_LIMIT_TRIP')),
    ('snap_wheel',   re.compile(r'snap_to_wheel')),
    ('CAMERA_STALE', re.compile(r'camera.*stale|cam.*stale|stale.*cam', re.I)),
    ('panda_err',    re.compile(r'panda.*(err|fault|reset)', re.I)),
    ('comm_issue',   re.compile(r'commIssue|commLost', re.I)),
    ('controlsInit', re.compile(r'controlsInit')),
    ('safety_err',   re.compile(r'safety.*err|tx.*block', re.I)),
]


def scan_segment(path):
    """Return a dict of findings for one rlog."""
    with open(path, 'rb') as f:
        raw = zstd.ZstdDecompressor().decompress(f.read(), max_output_size=500*1024*1024)
    out = {
        'path': os.path.basename(path),
        'cloudlogs': Counter(),
        'cloudlog_samples': defaultdict(list),  # first 3 samples per pattern
        'events': Counter(),
        'events_sp': Counter(),
        'panda_max': {'busOffCnt': 0, 'canSendErrs': 0, 'canFwdErrs': 0, 'faultsDetectedCount': 0},
        'panda_first': None,
        'cc_nan_count': 0,
        'cc_oversize_count': 0,
        'cs_oversize_count': 0,
        'oversize_threshold': 270.0,  # deg — > 270° wheel angle is extreme (3/4 lock)
        'mads_state_transitions': Counter(),
        'mads_state_prev': None,
        'frame_count_cs': 0,
        'frame_count_cc': 0,
        'frame_count_ss': 0,
        'monotime_first': None,
        'monotime_last': None,
        'git_commit': None,
    }
    pat_other = re.compile(r'')

    for msg in log.Event.read_multiple_bytes(raw):
        w = msg.which()
        if w == 'initData':
            out['git_commit'] = msg.initData.gitCommit[:8]
        elif w == 'androidLog':
            text = msg.androidLog.message or ''
            if not text:
                continue
            matched = False
            for key, pat in KNOWN_PATTERNS:
                if pat.search(text):
                    out['cloudlogs'][key] += 1
                    if len(out['cloudlog_samples'][key]) < 3:
                        out['cloudlog_samples'][key].append(text[:300])
                    matched = True
                    break
            # Capture WARNING/ERROR level lines we didn't classify
            if not matched and ('WARNING' in text or 'ERROR' in text):
                out['cloudlogs']['_uncategorized_warn'] += 1
                if len(out['cloudlog_samples']['_uncategorized_warn']) < 3:
                    out['cloudlog_samples']['_uncategorized_warn'].append(text[:300])
        elif w == 'onroadEvents':
            for ev in msg.onroadEvents:
                out['events'][str(ev.name)] += 1
        elif w == 'onroadEventsSP':
            try:
                for ev in msg.onroadEventsSP:
                    out['events_sp'][str(ev.name)] += 1
            except Exception:
                pass
        elif w == 'pandaStates':
            try:
                for ps in msg.pandaStates:
                    if out['panda_first'] is None:
                        out['panda_first'] = {
                            'busOffCnt': int(ps.busOffCnt),
                            'canSendErrs': int(ps.canSendErrs),
                            'canFwdErrs': int(ps.canFwdErrs),
                            'faultsDetectedCount': int(getattr(ps, 'faultsDetectedCount', 0)),
                        }
                    out['panda_max']['busOffCnt'] = max(out['panda_max']['busOffCnt'], int(ps.busOffCnt))
                    out['panda_max']['canSendErrs'] = max(out['panda_max']['canSendErrs'], int(ps.canSendErrs))
                    out['panda_max']['canFwdErrs'] = max(out['panda_max']['canFwdErrs'], int(ps.canFwdErrs))
                    out['panda_max']['faultsDetectedCount'] = max(
                        out['panda_max']['faultsDetectedCount'],
                        int(getattr(ps, 'faultsDetectedCount', 0))
                    )
            except Exception:
                pass
        elif w == 'carControl':
            out['frame_count_cc'] += 1
            try:
                ang = float(msg.carControl.actuators.steeringAngleDeg)
                if ang != ang:  # NaN
                    out['cc_nan_count'] += 1
                elif abs(ang) > out['oversize_threshold']:
                    out['cc_oversize_count'] += 1
            except Exception:
                pass
        elif w == 'carState':
            out['frame_count_cs'] += 1
            if out['monotime_first'] is None:
                out['monotime_first'] = msg.logMonoTime
            out['monotime_last'] = msg.logMonoTime
            try:
                ang = float(msg.carState.steeringAngleDeg)
                if abs(ang) > out['oversize_threshold']:
                    out['cs_oversize_count'] += 1
            except Exception:
                pass
        elif w == 'selfdriveStateSP':
            out['frame_count_ss'] += 1
            try:
                mads_state = (
                    int(msg.selfdriveStateSP.mads.enabled),
                    int(msg.selfdriveStateSP.mads.active),
                )
                if mads_state != out['mads_state_prev']:
                    out['mads_state_transitions'][f"{out['mads_state_prev']}→{mads_state}"] += 1
                    out['mads_state_prev'] = mads_state
            except Exception:
                pass
    # Convert defaultdict to dict for JSON
    out['cloudlog_samples'] = dict(out['cloudlog_samples'])
    out['cloudlogs'] = dict(out['cloudlogs'])
    out['events'] = dict(out['events'])
    out['events_sp'] = dict(out['events_sp'])
    out['mads_state_transitions'] = dict(out['mads_state_transitions'])
    return out


def main():
    paths = []
    for r in ROUTES:
        paths.extend(sorted(glob.glob(os.path.join(DRIVELOG_DIR, f'*_{r}--*--rlog.zst'))))
    print(f"Scanning {len(paths)} rlog segments…")

    findings = []
    for i, p in enumerate(paths):
        try:
            f = scan_segment(p)
        except Exception as e:
            print(f"  ERR {os.path.basename(p)}: {e}")
            continue
        findings.append(f)
        if (i+1) % 10 == 0:
            print(f"  {i+1}/{len(paths)}")

    # Per-route roll-up
    by_route = defaultdict(lambda: {
        'segs': 0, 'cloudlogs': Counter(), 'events': Counter(), 'events_sp': Counter(),
        'panda_busoff_delta': 0, 'panda_sendErr_delta': 0, 'panda_fwdErr_delta': 0,
        'panda_faults_delta': 0,
        'cc_nan': 0, 'cc_oversize': 0, 'cs_oversize': 0,
        'mads_transitions': Counter(),
        'samples': defaultdict(list),
    })
    for f in findings:
        # Extract route from filename: 99b215d21bbf8735_00000014--...
        route = f['path'].split('_')[1].split('--')[0]
        r = by_route[route]
        r['segs'] += 1
        for k, v in f['cloudlogs'].items():
            r['cloudlogs'][k] += v
        for k, v in f['events'].items():
            r['events'][k] += v
        for k, v in f['events_sp'].items():
            r['events_sp'][k] += v
        if f['panda_first'] is not None:
            r['panda_busoff_delta'] += (f['panda_max']['busOffCnt'] - f['panda_first']['busOffCnt'])
            r['panda_sendErr_delta'] += (f['panda_max']['canSendErrs'] - f['panda_first']['canSendErrs'])
            r['panda_fwdErr_delta'] += (f['panda_max']['canFwdErrs'] - f['panda_first']['canFwdErrs'])
            r['panda_faults_delta'] += (f['panda_max']['faultsDetectedCount'] - f['panda_first']['faultsDetectedCount'])
        r['cc_nan'] += f['cc_nan_count']
        r['cc_oversize'] += f['cc_oversize_count']
        r['cs_oversize'] += f['cs_oversize_count']
        for k, v in f['mads_state_transitions'].items():
            r['mads_transitions'][k] += v
        for k, samples in f['cloudlog_samples'].items():
            r['samples'][k].extend(samples[:1])  # 1 per seg

    print("\n=== PER-ROUTE SUMMARY ===")
    for route in sorted(by_route.keys()):
        r = by_route[route]
        print(f"\n--- Drive {route} ({r['segs']} segs) ---")
        print(f"  Panda Δ: busOff={r['panda_busoff_delta']} sendErr={r['panda_sendErr_delta']} fwdErr={r['panda_fwdErr_delta']} faults={r['panda_faults_delta']}")
        print(f"  CC: nan={r['cc_nan']} oversize={r['cc_oversize']}  CS oversize={r['cs_oversize']}")
        print(f"  Cloudlog WARNING categories:")
        for k, v in r['cloudlogs'].most_common():
            print(f"    {k:>30}: {v}")
        print(f"  onroadEvents:")
        for k, v in r['events'].most_common(20):
            print(f"    {k:>30}: {v}")
        if r['events_sp']:
            print(f"  onroadEventsSP:")
            for k, v in r['events_sp'].most_common(20):
                print(f"    {k:>30}: {v}")
        print(f"  MADS state transitions ({sum(r['mads_transitions'].values())} total):")
        for k, v in r['mads_transitions'].most_common(8):
            print(f"    {k:>30}: {v}")

    # Save
    out_obj = {
        'routes': {route: {
            'segs': r['segs'],
            'panda_delta': {
                'busOff': r['panda_busoff_delta'],
                'sendErr': r['panda_sendErr_delta'],
                'fwdErr': r['panda_fwdErr_delta'],
                'faults': r['panda_faults_delta'],
            },
            'cc_nan': r['cc_nan'],
            'cc_oversize': r['cc_oversize'],
            'cs_oversize': r['cs_oversize'],
            'cloudlogs': dict(r['cloudlogs']),
            'events': dict(r['events']),
            'events_sp': dict(r['events_sp']),
            'mads_transitions': dict(r['mads_transitions']),
            'cloudlog_samples': {k: v[:3] for k, v in r['samples'].items()},
        } for route, r in by_route.items()},
    }
    with open('/tmp/patch11_audit.json', 'w') as f:
        json.dump(out_obj, f, indent=2)
    print("\nSaved /tmp/patch11_audit.json")


if __name__ == '__main__':
    main()
