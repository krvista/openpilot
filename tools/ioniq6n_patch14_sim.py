#!/usr/bin/env python3
"""Patch #14 sim — moderate-grip 50°+ driver-active yield entry.

Counts how often the new moderate snap-entry path would FIRE
(driver-cranked) vs SUPPRESS (op-driven via mismatch gate) in drives 14-16.

Entry conditions (carcontroller.py:686-705 extended):
  moderate_entry = |wheel| ≥ 50°
                   AND |torque| ≥ 30 Nm
                   AND not already_snapped
                   AND |apply_angle_last - wheel| ≥ 20°  ← discrimination
"""
import glob
import os
import sys
from collections import Counter, defaultdict

import numpy as np
import zstandard as zstd

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')
from cereal import log

ROUTES = ('00000014', '00000015', '00000016')
DRIVER_ACTIVE_STEERING_ANGLE_DEG = 50.0
DRIVER_ACTIVE_STEERING_TORQUE_NM = 30.0
DRIVER_ACTIVE_MISMATCH_DEG       = 20.0
DRIVER_ACTIVE_STAY_ANGLE_DEG     = 30.0
DRIVER_ACTIVE_STAY_TORQUE_NM     = 20.0

# override_factor formula (i6n CCNC angle path)
DEADZONE = 100.0
LOW_V_FULL = 180.0
HIGH_V_FULL = 350.0
LOW_V_SPEED = 8.0
HIGH_V_SPEED = 15.0
SNAP_ENTER_FACTOR = 0.90


def override_factor_of(torque, v_ms):
    full = float(np.interp(v_ms, [LOW_V_SPEED, HIGH_V_SPEED], [LOW_V_FULL, HIGH_V_FULL]))
    return float(np.clip((abs(torque) - DEADZONE) / max(full - DEADZONE, 1.0), 0.0, 1.0))


def scan_route(route):
    paths = sorted(glob.glob(f'/home/user/openpilot/drivelog/*_{route}--*--rlog.zst'))
    counts = Counter()
    samples = defaultdict(list)
    for p in paths:
        try:
            raw = zstd.ZstdDecompressor().decompress(open(p, 'rb').read(),
                                                     max_output_size=500 * 1024 * 1024)
        except Exception:
            continue
        latest_cs = None
        for msg in log.Event.read_multiple_bytes(raw):
            w = msg.which()
            if w == 'carState':
                latest_cs = msg.carState
            elif w == 'carControl' and latest_cs is not None:
                cc = msg.carControl
                if not cc.latActive:
                    continue
                wheel = float(latest_cs.steeringAngleDeg)
                tq = float(latest_cs.steeringTorque)
                op = float(cc.actuators.steeringAngleDeg)
                v_ms = float(latest_cs.vEgoRaw)
                abs_w = abs(wheel)
                abs_tq = abs(tq)
                mismatch = abs(op - wheel)
                ovf = override_factor_of(tq, v_ms)
                already_heavy = ovf >= SNAP_ENTER_FACTOR

                # moderate_entry conditions (assuming not_snapped — proxy with not_heavy)
                if abs_w >= DRIVER_ACTIVE_STEERING_ANGLE_DEG \
                   and abs_tq >= DRIVER_ACTIVE_STEERING_TORQUE_NM \
                   and not already_heavy:
                    if mismatch >= DRIVER_ACTIVE_MISMATCH_DEG:
                        counts['FIRES'] += 1
                        if len(samples['FIRES']) < 5:
                            samples['FIRES'].append({
                                'wheel': wheel, 'tq': tq, 'op': op,
                                'mismatch': mismatch, 'v_kmh': v_ms * 3.6, 'ovf': ovf,
                            })
                    else:
                        counts['SUPPRESSED'] += 1
                        if len(samples['SUPPRESSED']) < 5:
                            samples['SUPPRESSED'].append({
                                'wheel': wheel, 'tq': tq, 'op': op,
                                'mismatch': mismatch, 'v_kmh': v_ms * 3.6, 'ovf': ovf,
                            })

                # moderate_stay (snap exit suppression)
                if abs_w >= DRIVER_ACTIVE_STAY_ANGLE_DEG \
                   and abs_tq >= DRIVER_ACTIVE_STAY_TORQUE_NM:
                    counts['STAY_HOLDS'] += 1
    return counts, samples


def main():
    print("Scanning drives 14-16 for patch #14 trigger statistics...\n")
    totals = Counter()
    all_samples = defaultdict(list)
    for route in ROUTES:
        counts, samples = scan_route(route)
        print(f"=== drive {route} ===")
        for k in ('FIRES', 'SUPPRESSED', 'STAY_HOLDS'):
            print(f"  {k:>12}: {counts.get(k, 0):,}")
        for k, v in counts.items():
            totals[k] += v
        for k, v in samples.items():
            all_samples[k].extend(v[:2])

    n_fires = totals.get('FIRES', 0)
    n_suppr = totals.get('SUPPRESSED', 0)
    total = n_fires + n_suppr
    print(f"\n=== TOTAL (3 drives) ===")
    print(f"  moderate_entry candidates (|wheel|≥50° + |tq|≥30 Nm + not_heavy): {total:,}")
    print(f"    FIRES (mismatch ≥20° — driver-cranked):   {n_fires:,}  ({100*n_fires/max(total,1):.1f}%)")
    print(f"    SUPPRESSED (mismatch <20° — op-driven):   {n_suppr:,}  ({100*n_suppr/max(total,1):.1f}%)")
    print(f"  moderate_stay frames (|wheel|≥30° + |tq|≥20 Nm): {totals.get('STAY_HOLDS', 0):,}")

    # Gate verdicts
    fires_per_drive = n_fires / 3
    suppress_ratio = n_suppr / max(total, 1)
    print(f"\n=== Gate verdicts ===")
    print(f"  FIRES ≥ 50 per drive (avg {fires_per_drive:.1f}): {'PASS' if fires_per_drive >= 50 else 'FAIL'}")
    print(f"  SUPPRESSED ratio ≥ 30% ({100*suppress_ratio:.1f}%): "
          f"{'PASS' if suppress_ratio >= 0.30 else 'FAIL — mismatch discrimination too loose?'}")
    print(f"  SUPPRESSED ratio < 90% ({100*suppress_ratio:.1f}%): "
          f"{'PASS' if suppress_ratio < 0.90 else 'FAIL — almost all candidates are op-driven, patch may be unnecessary'}")

    print("\nSample FIRES (driver-cranked, op stale):")
    for s in all_samples['FIRES'][:5]:
        print(f"  wheel={s['wheel']:>+7.1f}° tq={s['tq']:>+6.1f}Nm op={s['op']:>+7.1f}° "
              f"mismatch={s['mismatch']:>5.1f}° v={s['v_kmh']:>5.1f}kph ovf={s['ovf']:.2f}")
    print("\nSample SUPPRESSED (op-driven, mismatch small):")
    for s in all_samples['SUPPRESSED'][:5]:
        print(f"  wheel={s['wheel']:>+7.1f}° tq={s['tq']:>+6.1f}Nm op={s['op']:>+7.1f}° "
              f"mismatch={s['mismatch']:>5.1f}° v={s['v_kmh']:>5.1f}kph ovf={s['ovf']:.2f}")


if __name__ == '__main__':
    main()
