#!/usr/bin/env python3
"""Patch #15 sim — ACIGain rate_up boost + city shelf raise.

Tests two changes targeting EPS authority recovery speed:

1. **B (rate_up boost)**: when steering_error > 1° or steering_torque < 30 Nm,
   boost rate_up from 0.004/frame to 0.02-0.04/frame so ACIGain climbs back
   to max in ~150ms instead of 1.25s after brief grip events.

2. **B' (city shelf raise)**: raise shelf (the mid-grip target plateau) at
   city speeds so brief grip events don't dip ACIGain as deep, reducing the
   subsequent climb needed.

Measures ACIGain trajectory across drives 14-16 at city speeds during
mismatch>5° events. Gate: ACIGain mean during drift events should rise
from current 0.50 to >0.80.
"""
import glob
import sys
from collections import Counter, defaultdict

import numpy as np
import zstandard as zstd

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')
from cereal import log

ROUTES = ('00000014', '00000015', '00000016')
QUANT = 0.004

SPEED_BINS = [(5.56, 8.33), (8.33, 11.1), (11.1, 13.89)]
BIN_LABELS = ['20-30', '30-40', '40-50']


def compute_aci_gain(v, tq, err, gain_prev, blinker, *,
                     rate_up_boost_err=False,
                     rate_up_boost_light=False,
                     city_shelf_raise=False):
    """Mirror carcontroller.py compute_aci_gain (ccnc_lka_alt=True, non-blinker branch)."""
    if blinker:
        bp_grip = 30.0
        bp_active = float(np.interp(v, [2., 11.], [100., 125.]))
        bp_heavy = float(np.interp(v, [2., 22.], [250., 350.]))
        target = float(np.interp(abs(tq), [0.0, bp_grip, bp_active, bp_heavy],
                                  [0.80, 0.55, 0.18, 0.08]))
        rate_dn = float(np.interp(abs(tq), [150., 350., 600.], [0.004, 0.014, 0.04]))
        rate_dn = max(rate_dn, 0.05)
        rate_up = max(0.004, 0.10)
    else:
        ceiling = float(np.interp(v, [0.5, 1.5], [1.0, 0.85]))
        if city_shelf_raise:
            shelf = float(np.interp(v, [2., 11.], [0.30, 0.40]))   # was [0.22, 0.30]
        else:
            shelf = float(np.interp(v, [2., 11.], [0.22, 0.30]))
        floor = float(np.interp(v, [2., 22.], [0.1, 0.3]))
        error_start = float(np.interp(v, [0., 5.56, 11.1, 33.3], [1.25, 0.5, 0.3, 0.2]))
        error_mult = float(np.interp(abs(err), [error_start, error_start * 2], [1.0, 2.0]))
        ceiling = min(1.0, ceiling * error_mult)
        bp1 = float(np.interp(v, [2., 11.], [30., 50.]))
        bp2 = float(np.interp(v, [2., 11.], [50., 70.]))
        bp3 = float(np.interp(v, [2., 11.], [150., 200.]))
        bp4 = float(np.interp(v, [2., 22.], [300., 450.]))
        target = float(np.interp(abs(tq), [bp1, bp2, bp3, bp4], [ceiling, shelf, shelf, floor]))

        rate_dn = float(np.interp(abs(tq), [150., 350., 600.], [0.004, 0.014, 0.04]))
        rate_up = 0.004
        # B: rate_up boost
        if rate_up_boost_err and abs(err) > 1.0:
            rate_up = max(rate_up, 0.04)
        if rate_up_boost_light and abs(tq) < 30.0:
            rate_up = max(rate_up, 0.02)

    gain = max(gain_prev - rate_dn, min(gain_prev + rate_up, target))
    return round(gain / QUANT) * QUANT


def bin_for(v):
    for (lo, hi), lab in zip(SPEED_BINS, BIN_LABELS):
        if lo <= v < hi:
            return lab
    return None


def main():
    # Three configurations:
    # 'current'    — baseline
    # 'B'          — rate_up boost only
    # 'B+B'''      — rate_up boost + city shelf raise
    configs = {
        'current':   dict(rate_up_boost_err=False, rate_up_boost_light=False, city_shelf_raise=False),
        'B':         dict(rate_up_boost_err=True,  rate_up_boost_light=True,  city_shelf_raise=False),
        'B+B prime': dict(rate_up_boost_err=True,  rate_up_boost_light=True,  city_shelf_raise=True),
    }

    gains_all = {k: {l: [] for l in BIN_LABELS} for k in configs}
    gains_drift = {k: {l: [] for l in BIN_LABELS} for k in configs}

    for route in ROUTES:
        paths = sorted(glob.glob(f'/home/user/openpilot/drivelog/*_{route}--*--rlog.zst'))
        for p in paths:
            try:
                raw = zstd.ZstdDecompressor().decompress(
                    open(p, 'rb').read(), max_output_size=500 * 1024 * 1024)
            except Exception:
                continue
            cs = None
            gain_state = {k: 0.0 for k in configs}
            for msg in log.Event.read_multiple_bytes(raw):
                w = msg.which()
                if w == 'carState':
                    cs = msg.carState
                elif w == 'carControl' and cs is not None:
                    cc = msg.carControl
                    if not cc.latActive:
                        gain_state = {k: 0.0 for k in configs}
                        continue
                    wheel = float(cs.steeringAngleDeg)
                    op = float(cc.actuators.steeringAngleDeg)
                    tq = float(cs.steeringTorque)
                    v = float(cs.vEgoRaw)
                    err = op - wheel
                    blinker = bool(cs.leftBlinker or cs.rightBlinker)
                    # Step each config
                    for k, kw in configs.items():
                        gain_state[k] = compute_aci_gain(v, tq, err, gain_state[k], blinker, **kw)
                    # Stats only at filtered frames
                    if abs(wheel) >= 30 or abs(op) >= 30:
                        continue
                    if abs(tq) >= 30:
                        continue
                    b = bin_for(v)
                    if not b:
                        continue
                    for k in configs:
                        gains_all[k][b].append(gain_state[k])
                        if abs(err) > 5:
                            gains_drift[k][b].append(gain_state[k])

    # Print
    print(f"=== ACIGain mean during ALL light-grip city frames ===\n")
    print(f"{'bin':>10} {'n':>7} {'current':>9} {'B':>9} {'B+B prime':>11} | {'ΔB':>7} {'ΔB+B prime':>9}")
    print("-" * 80)
    for b in BIN_LABELS:
        n = len(gains_all['current'][b])
        if n == 0:
            continue
        c = np.mean(gains_all['current'][b])
        b1 = np.mean(gains_all['B'][b])
        b2 = np.mean(gains_all['B+B prime'][b])
        print(f"{b:>10} {n:>7} {c:>8.3f} {b1:>8.3f} {b2:>10.3f} | {b1-c:>+6.3f} {b2-c:>+8.3f}")

    print(f"\n=== ACIGain mean during drift event frames (|err|>5°) ===\n")
    print(f"{'bin':>10} {'n':>7} {'current':>9} {'B':>9} {'B+B prime':>11} | {'ΔB':>7} {'ΔB+B prime':>9}")
    print("-" * 80)
    drift_cur = []
    drift_b = []
    drift_bb = []
    for b in BIN_LABELS:
        n = len(gains_drift['current'][b])
        if n == 0:
            continue
        c = np.mean(gains_drift['current'][b])
        b1 = np.mean(gains_drift['B'][b])
        b2 = np.mean(gains_drift['B+B prime'][b])
        drift_cur.extend(gains_drift['current'][b])
        drift_b.extend(gains_drift['B'][b])
        drift_bb.extend(gains_drift['B+B prime'][b])
        print(f"{b:>10} {n:>7} {c:>8.3f} {b1:>8.3f} {b2:>10.3f} | {b1-c:>+6.3f} {b2-c:>+8.3f}")

    if drift_cur:
        c = np.mean(drift_cur)
        b1 = np.mean(drift_b)
        b2 = np.mean(drift_bb)
        print(f"\n  Aggregate city drift (n={len(drift_cur):,}):")
        print(f"    Current  : mean {c:.3f}")
        print(f"    B        : mean {b1:.3f}  ({(b1-c)/c*100:+.1f}%)")
        print(f"    B+B prime: mean {b2:.3f}  ({(b2-c)/c*100:+.1f}%)")

        # Estimated EPS lag reduction (assuming lag ∝ 1/gain)
        lag_cur = 1.68
        lag_b = lag_cur * c / b1
        lag_bb = lag_cur * c / b2
        print(f"\n  Estimated apply→wheel lag (assuming lag ∝ 1/gain, baseline 1.68°):")
        print(f"    Current  : {lag_cur:.2f}°")
        print(f"    B        : {lag_b:.2f}°  ({(lag_b - lag_cur):+.2f}°, {(lag_cur-lag_b)/lag_cur*100:+.0f}%)")
        print(f"    B+B prime: {lag_bb:.2f}°  ({(lag_bb - lag_cur):+.2f}°, {(lag_cur-lag_bb)/lag_cur*100:+.0f}%)")

    # Gates
    print(f"\n=== Patch #15 gates (B+B prime) ===")
    if drift_cur:
        c = np.mean(drift_cur)
        b2 = np.mean(drift_bb)
        print(f"  ACIGain drift mean ≥ 0.80: {'PASS' if b2 >= 0.80 else 'FAIL'} ({b2:.3f})")
        print(f"  ACIGain drift mean increase ≥ 50%: {'PASS' if (b2-c)/c >= 0.50 else 'FAIL'} ({(b2-c)/c*100:+.1f}%)")


if __name__ == '__main__':
    main()
