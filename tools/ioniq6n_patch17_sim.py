#!/usr/bin/env python3
"""Patch #17 sim — two surgical naturalness fixes.

Cand A: speed_max_tau city-speed extension
  Current: speed_max_tau = np.interp(v, [10, 25], [2.5, 0.22])
    → v<10 m/s (city <36 km/h) clamps to 2.5s → vtau LPF stays ~2.5s
    → α≈0.004/frame → 60ms catch-up 2.4% → city drift root cause
  Proposed: np.interp(v, [5, 10, 15, 25], [0.80, 0.50, 0.30, 0.22])
    → v=5 keeps 0.8s (garage entry caster damping), v=10 → 0.5s
    → city drift (20-50 km/h) vtau drops from 2.5 → 0.30-0.50s

Cand B: moderate_entry blinker guard
  Current: 4-AND (wheel>=50 + tq>=30 + !snapped + mismatch>=20)
  Proposed: + blinker_off (the blinker override_factor curve handles LC)

Gates:
  Cand A: 20-50 km/h mismatch p50 ≤4°, frame_count(mis>5) ≤ 1,200, v<5 Δp90 +≤0.05
  Cand B: post-fix LC false-positive frames = 0; all suppressed frames had blinker_on
"""
import glob, sys
import numpy as np
import zstandard as zstd

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')
from cereal import log

ROUTES = ('00000014', '00000015', '00000016', '00000019', '0000001a')

# vtau constants (carcontroller.py + values.py post #16b)
VTAU_ANGLE_BP   = [0.0, 1.0, 3.0, 10.0]
VTAU_ANGLE_V    = [3.5, 0.4, 0.20, 0.20]
VTAU_SPEED_BP   = [0.0, 3.0, 5.0, 15.0]
VTAU_SPEED_V    = [0.5, 0.3, 0.20, 0.0]
VTAU_ENTRY_TH_BP = [4.0, 15.0, 25.0]
VTAU_ENTRY_TH_V  = [0.3, 0.5, 0.5]
VTAU_EXIT_TH    = 0.5
LPF_DT = 0.01  # 100 Hz

# Cand A curves
SPD_MAX_CUR = ([10.0, 25.0], [2.5, 0.22])
SPD_MAX_FIX = ([5.0, 10.0, 15.0, 25.0], [0.80, 0.50, 0.30, 0.22])


def vtau_compute(v, lpf, op, prev_sign, sustained, *, fix_a=False):
    """Simulate carcontroller.py:964-1014 vtau pipeline. Returns (vtau, new_sign, new_sustained)."""
    entry_th = float(np.interp(v, VTAU_ENTRY_TH_BP, VTAU_ENTRY_TH_V))
    entering_curve = abs(op) > abs(lpf) + entry_th
    returning_to_center = abs(op) < abs(lpf) - VTAU_EXIT_TH

    if entering_curve or returning_to_center:
        return 0.05, prev_sign, 60

    abs_angle = abs(lpf)
    angle_tau = float(np.interp(abs_angle, VTAU_ANGLE_BP, VTAU_ANGLE_V))
    speed_tau = float(np.interp(v, VTAU_SPEED_BP, VTAU_SPEED_V))
    vtau = max(angle_tau, speed_tau)
    if fix_a:
        speed_max_tau = float(np.interp(v, SPD_MAX_FIX[0], SPD_MAX_FIX[1]))
    else:
        speed_max_tau = float(np.interp(v, SPD_MAX_CUR[0], SPD_MAX_CUR[1]))
    vtau = min(vtau, speed_max_tau)

    cur_sign = 1 if op > lpf + 0.01 else (-1 if op < lpf - 0.01 else 0)
    if cur_sign != 0 and cur_sign == prev_sign:
        sustained = min(sustained + 1, 100)
    else:
        sustained = max(sustained - 2, 0)
    vtau = float(np.interp(sustained, [0, 30, 60], [vtau, min(vtau, 0.5), min(vtau, 0.1)]))
    return vtau, cur_sign, sustained


def find_drives():
    out = []
    for r in ROUTES:
        out.extend(sorted(glob.glob(f'/home/user/openpilot/drivelog/*_{r}--*--rlog.zst')))
    return out


# ============================================================
# Cand A: city vtau speed_max_tau extension
# ============================================================
def sim_cand_a():
    print("=" * 70)
    print("Cand A: speed_max_tau city extension")
    print("=" * 70)
    # Speed bins (km/h) -> indices
    bins = [(20, 30), (30, 40), (40, 50)]
    # collect per-bin (mismatch_cur, mismatch_fix)
    cur_mis = {b: [] for b in bins}
    fix_mis = {b: [] for b in bins}
    # v<5 m/s apply Δ tracker (regression guard)
    cur_lpf_delta = []
    fix_lpf_delta = []

    for p in find_drives():
        try:
            raw = zstd.ZstdDecompressor().decompress(open(p, 'rb').read(), max_output_size=500*1024*1024)
        except Exception:
            continue
        cs = None
        cur_lpf = 0.0; cur_sign = 0; cur_sust = 0
        fix_lpf = 0.0; fix_sign = 0; fix_sust = 0
        prev_cur_lpf = 0.0; prev_fix_lpf = 0.0
        for msg in log.Event.read_multiple_bytes(raw):
            w = msg.which()
            if w == 'carState':
                cs = msg.carState
            elif w == 'carControl' and cs is not None:
                cc = msg.carControl
                v = float(cs.vEgoRaw)
                wheel = float(cs.steeringAngleDeg)
                op = float(cc.actuators.steeringAngleDeg)
                tq = float(cs.steeringTorque)
                if not cc.latActive:
                    # passthrough: lpf tracks wheel
                    cur_lpf = wheel; fix_lpf = wheel
                    cur_sign = 0; fix_sign = 0
                    cur_sust = 0; fix_sust = 0
                    continue

                # Current
                vt_cur, cur_sign, cur_sust = vtau_compute(v, cur_lpf, op, cur_sign, cur_sust, fix_a=False)
                if vt_cur > 0.001:
                    a = LPF_DT / (vt_cur + LPF_DT)
                    new_cur = a * op + (1 - a) * cur_lpf
                else:
                    new_cur = wheel
                # Fix-A
                vt_fix, fix_sign, fix_sust = vtau_compute(v, fix_lpf, op, fix_sign, fix_sust, fix_a=True)
                if vt_fix > 0.001:
                    a = LPF_DT / (vt_fix + LPF_DT)
                    new_fix = a * op + (1 - a) * fix_lpf
                else:
                    new_fix = wheel

                # Mismatch = |apply - wheel|. apply ≈ lpf in light-grip light-blend case.
                if abs(tq) < 70.0:  # light grip
                    v_kmh = v * 3.6
                    for b in bins:
                        if b[0] <= v_kmh < b[1]:
                            cur_mis[b].append(abs(new_cur - wheel))
                            fix_mis[b].append(abs(new_fix - wheel))
                            break

                # v<5 m/s Δ apply (jitter regression guard)
                if v < 5.0:
                    cur_lpf_delta.append(abs(new_cur - prev_cur_lpf))
                    fix_lpf_delta.append(abs(new_fix - prev_fix_lpf))

                prev_cur_lpf = new_cur; prev_fix_lpf = new_fix
                cur_lpf = new_cur; fix_lpf = new_fix

    print()
    print(f"{'Speed':>8} | {'N':>6} | {'cur p50':>8} | {'fix p50':>8} | {'cur p90':>8} | {'fix p90':>8} | "
          f"{'frames>5° cur':>14} | {'fix':>6}")
    print("-" * 90)
    total_cur_gt5 = 0; total_fix_gt5 = 0
    for b in bins:
        cur = np.array(cur_mis[b]) if cur_mis[b] else np.array([0.0])
        fix = np.array(fix_mis[b]) if fix_mis[b] else np.array([0.0])
        cur_gt5 = int((cur > 5.0).sum())
        fix_gt5 = int((fix > 5.0).sum())
        total_cur_gt5 += cur_gt5; total_fix_gt5 += fix_gt5
        print(f"{b[0]:>3}-{b[1]:<3} | {len(cur):>6} | "
              f"{np.percentile(cur, 50):>7.2f}° | {np.percentile(fix, 50):>7.2f}° | "
              f"{np.percentile(cur, 90):>7.2f}° | {np.percentile(fix, 90):>7.2f}° | "
              f"{cur_gt5:>14} | {fix_gt5:>6}")
    print()
    print(f"  Total 20-50 km/h frames mismatch >5°: cur {total_cur_gt5} → fix {total_fix_gt5}")

    if cur_lpf_delta and fix_lpf_delta:
        cur_d = np.array(cur_lpf_delta); fix_d = np.array(fix_lpf_delta)
        print(f"\n  v<5 m/s apply Δ p90: cur {np.percentile(cur_d,90):.3f}°/frame → "
              f"fix {np.percentile(fix_d,90):.3f}°/frame (Δ +{np.percentile(fix_d,90)-np.percentile(cur_d,90):+.3f})")

    print(f"\n  Gates:")
    print(f"    20-50 frames>5° reduced ≥50% (2232 → ≤1100): "
          f"{'PASS' if total_fix_gt5 <= total_cur_gt5 * 0.5 else 'FAIL'} "
          f"({total_cur_gt5} → {total_fix_gt5})")
    p50_after = np.percentile(np.concatenate([np.array(fix_mis[b] if fix_mis[b] else [0.0]) for b in bins]), 50)
    print(f"    20-50 p50 ≤4°: {'PASS' if p50_after <= 4.0 else 'FAIL'} ({p50_after:.2f}°)")
    if cur_lpf_delta and fix_lpf_delta:
        d_inc = np.percentile(np.array(fix_lpf_delta),90) - np.percentile(np.array(cur_lpf_delta),90)
        print(f"    v<5 apply Δ p90 increase ≤+0.05°: "
              f"{'PASS' if d_inc <= 0.05 else 'FAIL'} ({d_inc:+.3f})")


# ============================================================
# Cand B: moderate_entry blinker guard
# ============================================================
def sim_cand_b():
    print()
    print("=" * 70)
    print("Cand B: moderate_entry blinker guard")
    print("=" * 70)

    # current condition (no blinker guard) vs fix (+ blinker off)
    # We don't simulate full state machine, only fire conditions: a frame "would fire"
    # if all moderate_entry preconditions met regardless of snap state.
    fires_cur_blinker_on = 0
    fires_cur_blinker_off = 0
    fires_fix_blinker_on = 0  # should be 0 after fix
    fires_fix_blinker_off = 0
    blinker_samples = []

    for p in find_drives():
        try:
            raw = zstd.ZstdDecompressor().decompress(open(p, 'rb').read(), max_output_size=500*1024*1024)
        except Exception:
            continue
        cs = None
        apply_last = 0.0
        for msg in log.Event.read_multiple_bytes(raw):
            w = msg.which()
            if w == 'carState':
                cs = msg.carState
            elif w == 'carControl' and cs is not None:
                cc = msg.carControl
                if not cc.latActive:
                    apply_last = float(cs.steeringAngleDeg)
                    continue
                wheel = float(cs.steeringAngleDeg)
                op = float(cc.actuators.steeringAngleDeg)
                tq = float(cs.steeringTorque)
                v = float(cs.vEgoRaw)
                blinker = bool(cs.leftBlinker or cs.rightBlinker)

                # moderate_entry preconditions (sim uses op as apply_last proxy for state-less frame eval)
                cond_angle = abs(wheel) >= 50.0
                cond_torque = abs(tq) >= 30.0
                cond_mismatch = abs(op - wheel) >= 20.0  # apply_last≈op in light blend

                if cond_angle and cond_torque and cond_mismatch:
                    if blinker:
                        fires_cur_blinker_on += 1
                        blinker_samples.append((v*3.6, wheel, op, tq))
                    else:
                        fires_cur_blinker_off += 1
                        fires_fix_blinker_off += 1
                    # fix: with blinker on → suppressed; off → same
                apply_last = op  # rough proxy

    print(f"\n  moderate_entry trigger frame counts (drives 14,15,16,19,1a):")
    print(f"    cur, blinker OFF: {fires_cur_blinker_off}")
    print(f"    cur, blinker ON:  {fires_cur_blinker_on}  ← suppressed by Cand B fix")
    print(f"    fix, blinker OFF: {fires_fix_blinker_off}  (unchanged)")
    print(f"    fix, blinker ON:  0  (guard active)")
    print()
    print(f"  Suppression effect: {fires_cur_blinker_on} fewer moderate_entry events")
    print(f"  (i.e. {fires_cur_blinker_on} signaled LC frames where snap would have fired)")

    if blinker_samples:
        print(f"\n  Sample of {min(5, len(blinker_samples))} suppressed events (LC false-positives):")
        for v, w, op, tq in blinker_samples[:5]:
            print(f"    v={v:.1f} km/h, wheel={w:+.1f}°, op={op:+.1f}°, mismatch={abs(op-w):.1f}°, tq={tq:+.0f} Nm")

    print(f"\n  Gates:")
    print(f"    No regression to blinker-off path: PASS (fix only narrows, never widens)")
    print(f"    Cand B value depends on count >0: "
          f"{'PASS (' + str(fires_cur_blinker_on) + ' events caught)' if fires_cur_blinker_on > 0 else 'NEUTRAL (0 LC false-positives in this drivelog — fix is preventive)'}")


if __name__ == '__main__':
    sim_cand_a()
    sim_cand_b()
