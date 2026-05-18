#!/usr/bin/env python3
"""Naturalness anomaly scan across all drives.

Signatures measured per-drive:
  S1 oscillation: apply_angle Δ sign-flip rate (high freq → judder)
  S2 stutter:     apply_angle Δ p99 (large step jumps under STEER_REQ=1)
  S3 mismatch:    apply vs wheel mismatch p50/p90 (LPF lag or MDPS not following)
  S4 snap-binge:  override_snapped on/off transitions per second (instability)
  S5 latActive bursts: brief lat_active drops (< 200ms then re-engage = unnatural)
  S6 wheel jerk:  steeringAngleDeg dd p99 (passenger-perceived jerk)
  S7 op_curv churn: actuators.steeringAngleDeg Δ sign-flip rate (model uncertainty)
  S8 STEER_REQ flap: in saved CAN, would manifest as latActive transitions
"""
import glob, sys, os
import numpy as np
import zstandard as zstd
from collections import defaultdict

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')
from cereal import log

DRIVELOG = '/home/user/openpilot/drivelog'

ALL_ROUTES = sorted({os.path.basename(p).split('_')[1].split('--')[0]
                     for p in glob.glob(f'{DRIVELOG}/*--rlog.zst')})


def signflip_rate(arr, dt=0.01):
    """Sign flips per second."""
    if len(arr) < 2: return 0.0
    s = np.sign(arr[1:] - arr[:-1])
    return float((s[1:] * s[:-1] < 0).sum() / (len(arr) * dt))


def scan_drive(route):
    paths = sorted(glob.glob(f'{DRIVELOG}/*_{route}--*--rlog.zst'))
    if not paths: return None

    rows = []
    for p in paths:
        try:
            raw = zstd.ZstdDecompressor().decompress(open(p,'rb').read(), max_output_size=500*1024*1024)
        except Exception:
            continue
        cs = None
        for msg in log.Event.read_multiple_bytes(raw):
            w = msg.which()
            if w == 'carState':
                cs = msg.carState
            elif w == 'carControl' and cs is not None:
                cc = msg.carControl
                rows.append((cs.vEgoRaw, cs.steeringAngleDeg, cs.steeringTorque,
                             bool(cs.leftBlinker or cs.rightBlinker),
                             cc.latActive, cc.actuators.steeringAngleDeg))
    if not rows: return None
    v, wh, tq, bl, la, op = (np.array([r[i] for r in rows]) for i in range(6))
    bl = bl.astype(bool); la = la.astype(bool)

    # Active frames only for S1-S4
    ai = np.where(la)[0]
    if len(ai) < 100: return None

    op_a = op[ai]
    wh_a = wh[ai]
    v_a = v[ai]
    tq_a = tq[ai]

    # S1: op (≈apply in light-grip) Δ sign-flips per sec
    s1 = signflip_rate(op_a)
    # S3: mismatch
    mis = np.abs(op_a - wh_a)
    s3_p50 = float(np.percentile(mis, 50))
    s3_p90 = float(np.percentile(mis, 90))
    # S5: la transitions
    la_trans = int((np.diff(la.astype(int)) != 0).sum())
    # S6: wheel jerk (second derivative of wheel angle, /sec^2)
    wh_d = np.diff(wh) / 0.01
    wh_dd = np.diff(wh_d) / 0.01
    s6_p99 = float(np.percentile(np.abs(wh_dd), 99))
    # S7: op_curv sign-flips
    s7 = signflip_rate(op_a)  # same as s1 in this proxy; revise below
    # S2: op step jump p99
    s2_p99 = float(np.percentile(np.abs(np.diff(op_a)), 99))
    # Blinker-on active frames mismatch
    bl_a = bl[ai]
    bl_mis = mis[bl_a]
    bl_p90 = float(np.percentile(bl_mis, 90)) if len(bl_mis) > 50 else 0.0
    # Speed binning of active mismatch
    spd_bins = {'<20': (0, 5.56), '20-30': (5.56, 8.33), '30-40': (8.33, 11.1),
                '40-50': (11.1, 13.89), '50+': (13.89, 999)}
    spd_mis = {}
    for name, (lo, hi) in spd_bins.items():
        m = (v_a >= lo) & (v_a < hi)
        if m.sum() > 50:
            spd_mis[name] = (int(m.sum()), float(np.percentile(mis[m], 50)), float(np.percentile(mis[m], 90)))
        else:
            spd_mis[name] = (int(m.sum()), 0.0, 0.0)

    return dict(
        route=route, n_frames=len(rows), n_active=len(ai),
        s1_signflip=s1, s2_step_p99=s2_p99, s3_p50=s3_p50, s3_p90=s3_p90,
        s5_la_trans=la_trans, s6_jerk_p99=s6_p99,
        blinker_p90=bl_p90, blinker_active_frames=int(bl_a.sum()),
        spd_mis=spd_mis,
    )


def main():
    print(f"Scanning {len(ALL_ROUTES)} drives...")
    print()
    results = []
    for r in ALL_ROUTES:
        try:
            d = scan_drive(r)
        except Exception as e:
            print(f"  {r}: ERROR {e}")
            continue
        if d is None:
            continue
        results.append(d)

    # Sort by jerk p99 (most jerk-y drives first)
    print(f"\n{'Drive':<10} {'Active':>7} {'Mis_p50':>8} {'Mis_p90':>8} {'Jerk_p99':>10} "
          f"{'SignFlip':>10} {'Step_p99':>10} {'LaTrans':>8} {'BlnkP90':>8}")
    print("-" * 100)
    for d in sorted(results, key=lambda x: -x['s6_jerk_p99']):
        print(f"{d['route']:<10} {d['n_active']:>7} "
              f"{d['s3_p50']:>7.2f}° {d['s3_p90']:>7.2f}° "
              f"{d['s6_jerk_p99']:>9.1f} "
              f"{d['s1_signflip']:>9.1f}/s {d['s2_step_p99']:>9.2f}° "
              f"{d['s5_la_trans']:>8} {d['blinker_p90']:>7.2f}°")

    print(f"\n\nSpeed-bin mismatch breakdown (top mismatchers):")
    print(f"{'Drive':<10}", end='')
    for name in ['<20', '20-30', '30-40', '40-50', '50+']:
        print(f"{name:>16}", end='')
    print()
    print("-" * 90)
    for d in sorted(results, key=lambda x: -x['s3_p90']):
        print(f"{d['route']:<10}", end='')
        for name in ['<20', '20-30', '30-40', '40-50', '50+']:
            n, p50, p90 = d['spd_mis'].get(name, (0, 0, 0))
            print(f"{n:>5} p90:{p90:>6.2f}°", end=' ')
        print()


if __name__ == '__main__':
    main()
