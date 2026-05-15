#!/usr/bin/env python3
"""Patch #15 sim — VTAU LPF acceleration for city speeds (20-50 km/h).

Re-runs vtau LPF on actual op_curv_safe trajectories from drives 14-16
with both the current and proposed (extended speed_max_tau curve + lowered
city entry_th) settings. Measures the LPF tracking lag |op_curv - lpf|
binned by speed.

The previous data analysis showed 95% of drift events (mismatch>5° between
op_curv and wheel) occur at 20-50 km/h where current vtau=1.95s. This sim
quantifies how much the proposed change reduces the slow-mode LPF lag
that contributes to those events.
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
LPF_DT = 0.01  # 100 Hz

SPEED_BINS = [(0, 5.56), (5.56, 8.33), (8.33, 11.1), (11.1, 13.89),
              (13.89, 16.67), (16.67, 22.22), (22.22, 99)]
BIN_LABELS = ['<20', '20-30', '30-40', '40-50', '50-60', '60-80', '80+']


def vtau_current(v, ang_abs):
    angle_tau = float(np.interp(ang_abs, [0, 1, 3, 10], [3.5, 0.4, 0.20, 0.20]))
    speed_tau = float(np.interp(v, [0, 3, 5, 15], [0.5, 0.3, 0.20, 0.0]))
    cap = float(np.interp(v, [10.0, 25.0], [2.5, 0.22]))
    return min(max(angle_tau, speed_tau), cap)


def vtau_proposed(v, ang_abs):
    angle_tau = float(np.interp(ang_abs, [0, 1, 3, 10], [3.5, 0.4, 0.20, 0.20]))
    speed_tau = float(np.interp(v, [0, 3, 5, 15], [0.5, 0.3, 0.20, 0.0]))
    cap = float(np.interp(v, [5.0, 15.0, 25.0], [0.8, 0.35, 0.15]))
    return min(max(angle_tau, speed_tau), cap)


def entry_th_current(v):
    return float(np.interp(v, [4.0, 15.0, 25.0], [0.3, 0.5, 0.5]))


def entry_th_proposed(v):
    return float(np.interp(v, [4.0, 8.0, 15.0, 25.0], [0.3, 0.3, 0.35, 0.30]))


def simulate(op_seq, init, v_seq, vtau_fn, entry_fn, exit_th=0.3):
    """Re-simulate vtau_lpf trajectory with sustained_cnt cap mechanism."""
    lpf = init
    sustained_cnt = 0
    prev_sign = 0
    out = []
    trips = 0
    for op, v in zip(op_seq, v_seq):
        entry_th = entry_fn(v)
        entering = abs(op) > abs(lpf) + entry_th
        returning = abs(op) < abs(lpf) - exit_th
        if entering or returning:
            tau = 0.05
            sustained_cnt = 60
            if entering:
                trips += 1
        else:
            tau = vtau_fn(v, abs(lpf))
            cur_sign = 1 if op > lpf + 0.01 else (-1 if op < lpf - 0.01 else 0)
            if cur_sign != 0 and cur_sign == prev_sign:
                sustained_cnt = min(sustained_cnt + 1, 100)
            else:
                sustained_cnt = max(sustained_cnt - 2, 0)
            prev_sign = cur_sign
            # sustained_cnt cap: 0→vtau, 30→min(vtau,0.5), 60→min(vtau,0.1)
            tau = float(np.interp(sustained_cnt, [0, 30, 60],
                                  [tau, min(tau, 0.5), min(tau, 0.1)]))
        alpha = LPF_DT / (tau + LPF_DT)
        lpf = alpha * op + (1.0 - alpha) * lpf
        out.append(lpf)
    return out, trips


def bin_for(v):
    for (lo, hi), lab in zip(SPEED_BINS, BIN_LABELS):
        if lo <= v < hi:
            return lab
    return None


def scan():
    lag_current = defaultdict(list)
    lag_proposed = defaultdict(list)
    trips_current = 0
    trips_proposed = 0

    for route in ROUTES:
        paths = sorted(glob.glob(f'/home/user/openpilot/drivelog/*_{route}--*--rlog.zst'))
        for p in paths:
            try:
                raw = zstd.ZstdDecompressor().decompress(
                    open(p, 'rb').read(), max_output_size=500 * 1024 * 1024)
            except Exception:
                continue
            cs = None
            seq_op, seq_wheel, seq_v, seq_tq = [], [], [], []
            for msg in log.Event.read_multiple_bytes(raw):
                w = msg.which()
                if w == 'carState':
                    cs = msg.carState
                elif w == 'carControl' and cs is not None:
                    cc = msg.carControl
                    if not cc.latActive:
                        seq_op.clear(); seq_wheel.clear(); seq_v.clear(); seq_tq.clear()
                        continue
                    seq_op.append(float(cc.actuators.steeringAngleDeg))
                    seq_wheel.append(float(cs.steeringAngleDeg))
                    seq_v.append(float(cs.vEgoRaw))
                    seq_tq.append(float(cs.steeringTorque))
            if not seq_op:
                continue

            init = seq_wheel[0]
            lpf_cur, tc = simulate(seq_op, init, seq_v, vtau_current, entry_th_current)
            lpf_pro, tp = simulate(seq_op, init, seq_v, vtau_proposed, entry_th_proposed)
            trips_current += tc
            trips_proposed += tp

            for i, (op, wheel, v, tq) in enumerate(zip(seq_op, seq_wheel, seq_v, seq_tq)):
                if abs(wheel) >= 30 or abs(op) >= 30:
                    continue
                if abs(tq) >= 30:
                    continue
                lab = bin_for(v)
                if not lab:
                    continue
                # LPF lag
                lag_current[lab].append(abs(op - lpf_cur[i]))
                lag_proposed[lab].append(abs(op - lpf_pro[i]))

    return lag_current, lag_proposed, trips_current, trips_proposed


def stats(arr):
    if not arr:
        return 0, 0, 0
    a = np.array(arr)
    return float(a.mean()), float(np.percentile(a, 90)), float(np.percentile(a, 99))


def main():
    print("Re-simulating vtau LPF on drives 14-16 (current vs Patch #15 proposed)...\n")
    lc, lp, tc, tp = scan()

    print(f"{'speed':>10} {'n':>7} | {'Cur mean':>8} {'Cur p90':>8} {'Cur p99':>8} | "
          f"{'Pro mean':>8} {'Pro p90':>8} {'Pro p99':>8} | {'Δmean':>7} {'Δp90':>7}")
    print("-" * 110)
    city_n_cur = []
    city_n_pro = []
    for lab in BIN_LABELS:
        if not lc[lab]:
            continue
        mc, p90c, p99c = stats(lc[lab])
        mp, p90p, p99p = stats(lp[lab])
        dmean = mp - mc
        dp90 = p90p - p90c
        print(f"{lab:>10} {len(lc[lab]):>7} | {mc:>7.3f}° {p90c:>7.3f}° {p99c:>7.3f}° | "
              f"{mp:>7.3f}° {p90p:>7.3f}° {p99p:>7.3f}° | {dmean:>+6.3f}° {dp90:>+6.3f}°")
        if lab in ('20-30', '30-40', '40-50'):
            city_n_cur.extend(lc[lab])
            city_n_pro.extend(lp[lab])

    mc, p90c, p99c = stats(city_n_cur)
    mp, p90p, p99p = stats(city_n_pro)
    print(f"\n=== City aggregate (20-50 km/h) ===")
    print(f"  n: {len(city_n_cur):,}")
    print(f"  Current  : mean {mc:.3f}°  p90 {p90c:.3f}°  p99 {p99c:.3f}°")
    print(f"  Proposed : mean {mp:.3f}°  p90 {p90p:.3f}°  p99 {p99p:.3f}°")
    print(f"  Δ        : mean {mp-mc:+.3f}°  p90 {p90p-p90c:+.3f}°  p99 {p99p-p99c:+.3f}°")
    print(f"  mean reduction: {(mc-mp)/max(mc,1e-6)*100:+.1f}%")
    print(f"  p90 reduction:  {(p90c-p90p)/max(p90c,1e-6)*100:+.1f}%")

    print(f"\n=== entering_curve trips ===")
    print(f"  Current  : {tc:,}")
    print(f"  Proposed : {tp:,}")
    print(f"  Growth   : {(tp-tc)/max(tc,1)*100:+.1f}%")

    print(f"\n=== Patch #15 gates ===")
    mean_drop = (mc - mp) / max(mc, 1e-6) * 100
    p90_drop = (p90c - p90p) / max(p90c, 1e-6) * 100
    ec_growth = (tp - tc) / max(tc, 1) * 100
    print(f"  City mean lag reduction ≥ 20%: {'PASS' if mean_drop >= 20 else 'FAIL'} ({mean_drop:.1f}%)")
    print(f"  City p90 lag reduction ≥ 25%: {'PASS' if p90_drop >= 25 else 'FAIL'} ({p90_drop:.1f}%)")
    print(f"  entering_curve growth ≤ +40%: {'PASS' if ec_growth <= 40 else 'FAIL'} ({ec_growth:+.1f}%)")


if __name__ == '__main__':
    main()
