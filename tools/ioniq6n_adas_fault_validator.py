#!/usr/bin/env python3
"""Comprehensive ADAS fault validation across ALL drivelogs.

Checks every sendcan LKAS_ALT (0x110) frame for dangerous field
combinations that could trigger ADAS DRV faults on the Ioniq 6 N
(CCNC + CANFD_LKA_STEERING_ALT).

Six hazard classes checked per frame:
  H1: ACTIVE=2 but ACIGain=0  (known ADAS fault trigger)
  H2: ACTIVE=1 but ACIGain>0  (mismatch, confuses MDPS)
  H3: LKA_ASSIST≠ACTIVE parity (ASSIST=1 needs ACTIVE=2, ASSIST=0 needs ACTIVE≤1)
  H4: BYTE13=0x09 but ACTIVE≠2 (active-marker with passive ACTIVE)
  H5: BYTE7_BITS4_5=3 but ACTIVE≠2 (active-marker with passive ACTIVE)
  H6: Single-frame ACTIVE glitch (2→1→2 or 1→2→1 within 3 frames)

Also tracks:
  - latActive / steering_active distribution per route
  - ACC ON/OFF and MADS ON/OFF transitions
  - ACIGain distribution when ACTIVE=2
  - onroadEvents with noEntry=True
  - steerFaultTemporary counts
  - MDPS LKA_ANGLE_ACTIVE vs op TX comparison
"""

import glob
import re
import sys
from collections import Counter, defaultdict

import numpy as np

sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG_DIR = '/home/user/openpilot/drivelog'


def decode_lkas_alt_tx(dat):
    """Decode critical LKAS_ALT fields from raw CAN bytes (sendcan)."""
    if len(dat) < 32:
        return None
    # LKAS_ANGLE_ACTIVE : 77|2@0+ → byte 9, bits 5..4
    active = (dat[9] >> 4) & 0x3
    # LKA_ASSIST : 55|1@0+ → byte 6, bit 7
    assist = (dat[7] >> 6) & 0x1
    # ADAS_ACIAnglTqRedcGainVal : 96|8@1+ scale 0.004
    gain_raw = dat[12]
    gain = gain_raw * 0.004
    # LKAS_BYTE7_BITS4_5 : byte 7 bits 5..4
    byte7_45 = (dat[7] >> 4) & 0x3
    # LKAS_BYTE7_BIT7 : byte 7 bit 7
    byte7_7 = (dat[7] >> 7) & 0x1
    # LKAS_BYTE13
    byte13 = dat[13]
    # LKA_WARNING : 22|1@0+ → byte 2 bit 6
    lka_warn = (dat[2] >> 6) & 0x1
    # FCA_SYSWARN : 23|1@0+ → byte 2 bit 7
    fca_warn = (dat[2] >> 7) & 0x1
    return {
        'active': active,
        'assist': assist,
        'gain': gain,
        'byte7_45': byte7_45,
        'byte7_7': byte7_7,
        'byte13': byte13,
        'lka_warn': lka_warn,
        'fca_warn': fca_warn,
    }


def get_routes():
    """Find all route patterns in drivelog."""
    files = glob.glob(f'{DRIVELOG_DIR}/*--rlog.zst')
    routes = defaultdict(list)
    for f in files:
        m = re.search(r'_([0-9a-f]+)--([0-9a-f]+)--(\d+)--rlog\.zst$', f)
        if m:
            route_id, route_hash, seg = m.group(1), m.group(2), int(m.group(3))
            routes[(route_id, route_hash)].append((seg, f))
    for k in routes:
        routes[k].sort()
    return dict(sorted(routes.items()))


def validate_route(route_id, route_hash, seg_files, verbose=False):
    """Run all hazard checks on one route. Returns dict of counts."""
    c = Counter()
    tx_history = []  # last 3 ACTIVE values for glitch detection
    prev_lat = None
    prev_cruise_avail = None

    for seg_idx, seg_path in seg_files:
        try:
            for m in LogReader(seg_path):
                try:
                    w = m.which()
                except Exception:
                    continue

                if w == 'sendcan':
                    for msg in m.sendcan:
                        if msg.address == 0x110 and len(msg.dat) >= 32:
                            d = decode_lkas_alt_tx(bytes(msg.dat))
                            if d is None:
                                continue
                            c['tx_total'] += 1

                            # H1: ACTIVE=2 but gain=0
                            if d['active'] == 2 and d['gain'] < 0.005:
                                c['H1_active2_gain0'] += 1
                            # H2: ACTIVE≤1 but gain>0
                            if d['active'] <= 1 and d['gain'] > 0.005:
                                c['H2_passive_gain_nonzero'] += 1
                            # H3: ASSIST/ACTIVE parity
                            if d['assist'] == 1 and d['active'] != 2:
                                c['H3_assist1_active_not2'] += 1
                            if d['assist'] == 0 and d['active'] == 2:
                                c['H3_assist0_active2'] += 1
                            # H4: BYTE13=0x09 but ACTIVE≠2
                            if d['byte13'] == 0x09 and d['active'] != 2:
                                c['H4_byte13_active_mismatch'] += 1
                            # H5: BYTE7_45=3 but ACTIVE≠2
                            if d['byte7_45'] == 3 and d['active'] != 2:
                                c['H5_byte7_active_mismatch'] += 1
                            # H6: glitch detection (2→X→2 or 1→X→1 in 3 frames)
                            tx_history.append(d['active'])
                            if len(tx_history) > 3:
                                tx_history.pop(0)
                            if len(tx_history) == 3:
                                a, b, c_ = tx_history
                                if a == c_ and a != b:
                                    c['H6_single_frame_glitch'] += 1

                            # ACIGain distribution when ACTIVE=2
                            if d['active'] == 2:
                                c['active2_total'] += 1
                                if d['gain'] < 0.05:
                                    c['active2_gain_lt005'] += 1
                                elif d['gain'] < 0.10:
                                    c['active2_gain_005_010'] += 1
                                elif d['gain'] < 0.15:
                                    c['active2_gain_010_015'] += 1
                                elif d['gain'] < 0.20:
                                    c['active2_gain_015_020'] += 1
                                else:
                                    c['active2_gain_020_plus'] += 1

                            # Warning pass-through when ACTIVE=2
                            if d['active'] == 2 and d['lka_warn'] == 1:
                                c['active2_lka_warn_unsuppressed'] += 1
                            if d['active'] == 2 and d['fca_warn'] == 1:
                                c['active2_fca_warn_unsuppressed'] += 1

                elif w == 'carState':
                    cs = m.carState
                    c['cs_total'] += 1
                    if cs.steerFaultTemporary:
                        c['steerFaultTemp'] += 1
                    if cs.steerFaultPermanent:
                        c['steerFaultPerm'] += 1
                    # ACC transitions
                    avail = cs.cruiseState.available
                    if prev_cruise_avail is not None:
                        if avail and not prev_cruise_avail:
                            c['acc_on_transitions'] += 1
                        elif not avail and prev_cruise_avail:
                            c['acc_off_transitions'] += 1
                    prev_cruise_avail = avail

                elif w == 'carControl':
                    cc = m.carControl
                    if cc.latActive:
                        c['latActive_true'] += 1
                    else:
                        c['latActive_false'] += 1
                    # latActive transitions
                    lat = cc.latActive
                    if prev_lat is not None:
                        if lat and not prev_lat:
                            c['mads_on_transitions'] += 1
                        elif not lat and prev_lat:
                            c['mads_off_transitions'] += 1
                    prev_lat = lat

                elif w == 'onroadEvents':
                    for e in m.onroadEvents:
                        ne = getattr(e, 'noEntry', False)
                        if ne:
                            c[f'noEntry_{e.name}'] += 1
                        c['onroad_total'] += 1

                elif w == 'carStateSP':
                    sp = m.carStateSP
                    c[f'mdps_active_{sp.mdpsLkaAngleActive}'] += 1
                    if sp.mdpsLkaAngleFault:
                        c['mdps_angle_fault'] += 1

        except Exception as e:
            c['parse_errors'] += 1

    return dict(c)


def main():
    routes = get_routes()
    print(f"Found {len(routes)} routes, {sum(len(v) for v in routes.values())} total segments")
    print()

    all_results = {}
    hazard_summary = Counter()

    for (route_id, route_hash), seg_files in routes.items():
        route_label = f"0x{route_id} ({route_hash})"
        print(f"Scanning {route_label} ({len(seg_files)} segments)...", end=' ', flush=True)
        result = validate_route(route_id, route_hash, seg_files)
        all_results[route_label] = result

        # Tally hazards
        hazards_found = []
        for k in ['H1_active2_gain0', 'H2_passive_gain_nonzero',
                   'H3_assist1_active_not2', 'H3_assist0_active2',
                   'H4_byte13_active_mismatch', 'H5_byte7_active_mismatch',
                   'H6_single_frame_glitch']:
            v = result.get(k, 0)
            if v > 0:
                hazards_found.append(f"{k}={v}")
                hazard_summary[k] += v

        tx = result.get('tx_total', 0)
        lat_on = result.get('latActive_true', 0)
        lat_off = result.get('latActive_false', 0)
        mads_on_tr = result.get('mads_on_transitions', 0)
        mads_off_tr = result.get('mads_off_transitions', 0)
        acc_on = result.get('acc_on_transitions', 0)
        acc_off = result.get('acc_off_transitions', 0)
        faults = result.get('steerFaultTemp', 0)
        a2 = result.get('active2_total', 0)

        status = "CLEAN" if not hazards_found else "HAZARD"
        print(f"{status}  TX={tx:>6d}  latON={lat_on:>6d}  MADS±={mads_on_tr}/{mads_off_tr}  "
              f"ACC±={acc_on}/{acc_off}  faultT={faults}  ACTIVE2={a2}")
        if hazards_found:
            for h in hazards_found:
                print(f"  !! {h}")

    # ══════ Global Summary ══════
    print("\n" + "=" * 100)
    print(" GLOBAL HAZARD SUMMARY")
    print("=" * 100)

    total_tx = sum(r.get('tx_total', 0) for r in all_results.values())
    total_a2 = sum(r.get('active2_total', 0) for r in all_results.values())
    total_faults = sum(r.get('steerFaultTemp', 0) for r in all_results.values())
    total_mdps_fault = sum(r.get('mdps_angle_fault', 0) for r in all_results.values())

    print(f"  Total routes:          {len(routes)}")
    print(f"  Total TX frames:       {total_tx}")
    print(f"  Total ACTIVE=2 frames: {total_a2}")
    print(f"  Total steerFaultTemp:  {total_faults}")
    print(f"  Total mdps_angle_fault:{total_mdps_fault}")

    print(f"\n  Hazard counts across ALL routes:")
    hazard_keys = ['H1_active2_gain0', 'H2_passive_gain_nonzero',
                   'H3_assist1_active_not2', 'H3_assist0_active2',
                   'H4_byte13_active_mismatch', 'H5_byte7_active_mismatch',
                   'H6_single_frame_glitch']
    any_hazard = False
    for k in hazard_keys:
        v = hazard_summary.get(k, 0)
        label = "PASS" if v == 0 else f"FAIL ({v})"
        print(f"    {k:40s} {label}")
        if v > 0:
            any_hazard = True

    # ACIGain distribution when ACTIVE=2
    print(f"\n  ACIGain distribution (ACTIVE=2 frames):")
    gain_keys = ['active2_gain_lt005', 'active2_gain_005_010',
                 'active2_gain_010_015', 'active2_gain_015_020',
                 'active2_gain_020_plus']
    gain_labels = ['<0.05', '0.05-0.10', '0.10-0.15', '0.15-0.20', '≥0.20']
    for k, label in zip(gain_keys, gain_labels):
        v = sum(r.get(k, 0) for r in all_results.values())
        pct = v / max(total_a2, 1) * 100
        print(f"    {label:15s} {v:>8d}  ({pct:5.1f}%)")

    # Unsuppressed warnings during ACTIVE=2
    warn_lka = sum(r.get('active2_lka_warn_unsuppressed', 0) for r in all_results.values())
    warn_fca = sum(r.get('active2_fca_warn_unsuppressed', 0) for r in all_results.values())
    print(f"\n  Unsuppressed warnings during ACTIVE=2:")
    print(f"    LKA_WARNING=1:  {warn_lka}")
    print(f"    FCA_SYSWARN=1:  {warn_fca}")

    # MDPS active distribution
    print(f"\n  MDPS LKA_ANGLE_ACTIVE distribution (carStateSP):")
    for k in range(3):
        v = sum(r.get(f'mdps_active_{k}', 0) for r in all_results.values())
        print(f"    mdps={k}: {v:>8d}")

    # noEntry events
    print(f"\n  NO_ENTRY events (all routes):")
    ne_keys = sorted(set(k for r in all_results.values()
                        for k in r if k.startswith('noEntry_')),
                     key=lambda x: -sum(r.get(x, 0) for r in all_results.values()))
    for k in ne_keys[:10]:
        v = sum(r.get(k, 0) for r in all_results.values())
        print(f"    {k:40s} {v:>6d}")

    print("\n" + "=" * 100)
    if any_hazard:
        print(" RESULT: HAZARDS DETECTED — review required")
    else:
        print(" RESULT: ALL CLEAN — no dangerous field combinations found")
    print("=" * 100)


if __name__ == '__main__':
    main()
