#!/usr/bin/env python3
"""Patch #13 sim — light-grip 50°+ recovery trigger.

Extends patch #12 sim with the new wheel/torque-based trigger path.
Counts how many frames in drives 14-16 would trigger the new path,
how many are 'op-driven' (no driver torque, mismatch small — should
NOT trigger), and how many are 'driver-cranked' (real positives).
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
HANDS_OFF_RECOVERY_ANGLE_DEG = 50.0
HANDS_OFF_MISMATCH_DEG = 20.0

# Override factor formula constants (i6n CCNC angle, no-blinker)
DEADZONE  = 100.0
LOW_V_FULL  = 180.0
HIGH_V_FULL = 350.0
LOW_V_SPEED  = 8.0
HIGH_V_SPEED = 15.0


def override_factor_of(torque, v_ms):
    full = float(np.interp(v_ms, [LOW_V_SPEED, HIGH_V_SPEED], [LOW_V_FULL, HIGH_V_FULL]))
    return float(np.clip((abs(torque) - DEADZONE) / max(full - DEADZONE, 1.0), 0.0, 1.0))


def scan_route(route):
    paths = sorted(glob.glob(f'/home/user/openpilot/drivelog/*_{route}--*--rlog.zst'))
    counts = Counter()
    samples = defaultdict(list)
    for p in paths:
        try:
            raw = zstd.ZstdDecompressor().decompress(open(p,'rb').read(),
                                                     max_output_size=500*1024*1024)
        except Exception:
            continue
        latest_cs = None
        latest_sp = None
        for msg in log.Event.read_multiple_bytes(raw):
            w = msg.which()
            if w == 'carState': latest_cs = msg.carState
            elif w == 'selfdriveStateSP': latest_sp = msg.selfdriveStateSP
            elif w == 'carControl' and latest_cs is not None:
                cc = msg.carControl
                if not cc.latActive:
                    continue
                wheel = float(latest_cs.steeringAngleDeg)
                tq = float(latest_cs.steeringTorque)
                op_angle = float(cc.actuators.steeringAngleDeg)
                v_ms = float(latest_cs.vEgoRaw)
                ovf = override_factor_of(tq, v_ms)
                abs_wheel = abs(wheel)
                mismatch = abs(op_angle - wheel)
                mads_en = bool(latest_sp.mads.enabled) if latest_sp else False

                if abs_wheel >= HANDS_OFF_RECOVERY_ANGLE_DEG and ovf <= 0.1:
                    if mismatch >= HANDS_OFF_MISMATCH_DEG:
                        counts['trigger_FIRES'] += 1
                        if len(samples['trigger_FIRES']) < 5:
                            samples['trigger_FIRES'].append({
                                'wheel': wheel, 'tq': tq, 'op_angle': op_angle,
                                'mismatch': mismatch, 'v_kmh': v_ms*3.6, 'mads': mads_en,
                            })
                    else:
                        # op-driven 50°+ scenario (mismatch small): correctly suppressed
                        counts['trigger_SUPPRESSED'] += 1
                        if len(samples['trigger_SUPPRESSED']) < 5:
                            samples['trigger_SUPPRESSED'].append({
                                'wheel': wheel, 'tq': tq, 'op_angle': op_angle,
                                'mismatch': mismatch, 'v_kmh': v_ms*3.6, 'mads': mads_en,
                            })
                # Bonus: any frame with |wheel|>=50 + hands-off (regardless of mismatch)
                if abs_wheel >= HANDS_OFF_RECOVERY_ANGLE_DEG and ovf <= 0.1:
                    counts['all_50plus_hands_off'] += 1
    return counts, samples


def main():
    print("Scanning drives 14-16 for patch #13 trigger statistics…\n")
    total = Counter()
    all_samples = defaultdict(list)
    for route in ROUTES:
        counts, samples = scan_route(route)
        print(f"=== drive {route} ===")
        for k in ('all_50plus_hands_off', 'trigger_FIRES', 'trigger_SUPPRESSED'):
            print(f"  {k:>25}: {counts.get(k, 0):,}")
        for k, v in counts.items():
            total[k] += v
        for k, v in samples.items():
            all_samples[k].extend(v[:2])

    print(f"\n=== TOTAL (3 drives, ~660k carControl frames) ===")
    n_total_50 = total.get('all_50plus_hands_off', 0)
    n_fires = total.get('trigger_FIRES', 0)
    n_suppressed = total.get('trigger_SUPPRESSED', 0)
    print(f"  Frames with |wheel|>=50° + hands-off (lat_active=True): {n_total_50:,}")
    print(f"    of which mismatch≥20° (patch #13 FIRES — driver-cranked): {n_fires:,}  ({100*n_fires/max(n_total_50,1):.1f}%)")
    print(f"    of which mismatch<20° (patch #13 SUPPRESSED — op-driven): {n_suppressed:,}  ({100*n_suppressed/max(n_total_50,1):.1f}%)")

    print("\nSample 'FIRES' frames (driver-cranked, op stale):")
    for s in all_samples['trigger_FIRES'][:5]:
        print(f"  wheel={s['wheel']:>+7.1f}° tq={s['tq']:>+6.1f}Nm op_angle={s['op_angle']:>+7.1f}° "
              f"mismatch={s['mismatch']:>5.1f}° v={s['v_kmh']:>5.1f}kph mads={s['mads']}")
    print("\nSample 'SUPPRESSED' frames (op-driven, mismatch small):")
    for s in all_samples['trigger_SUPPRESSED'][:5]:
        print(f"  wheel={s['wheel']:>+7.1f}° tq={s['tq']:>+6.1f}Nm op_angle={s['op_angle']:>+7.1f}° "
              f"mismatch={s['mismatch']:>5.1f}° v={s['v_kmh']:>5.1f}kph mads={s['mads']}")


if __name__ == '__main__':
    main()
