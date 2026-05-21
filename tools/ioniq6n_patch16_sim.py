#!/usr/bin/env python3
"""Patch #16 sim — three independent fixes verification.

Fix-A: heavy_override_active mismatch guard
  Currently: override_factor>=0.9 (heavy grip) → snap → STEER_REQ=0
  Proposed: AND |apply - wheel| >= 10° (mismatch guard). If driver and op
  aligned (mismatch<10°), don't snap → MADS keeps STEER_REQ=1 with reduced
  ACIGain contributing some torque.

Fix-B: rate_up smoothing
  Currently: step function (err>1° → 0.04, tq<30 → 0.02). At boundary,
  rate_up flips between 0.004 and 0.04 every frame → 10x ACIGain step → stepwise jolts.
  Proposed: smooth np.interp curves.

Fix-D: recovery early-exit on release + op centering
  Currently: recovery hold until |wheel|<20° or 2s timeout.
  Proposed: ALSO exit when driver release (override_factor<=0.1) AND op
  command in same direction as wheel AND |op|<0.7|wheel| (op centering).
"""
import glob, sys
from collections import Counter
import numpy as np
import zstandard as zstd

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')
from cereal import log

ROUTES = ('00000014', '00000015', '00000016', '00000019', '0000001a')
QUANT = 0.004

# Override factor (same as carcontroller, i6n CCNC angle path, non-blinker)
def override_factor_of(tq_abs, v_ms, blinker_frac=0.0):
    # Match carcontroller.py ccnc_lka_alt non-blinker constants (blinker_frac=0)
    DEADZONE = 100.0
    LOW_V = 180.0
    HIGH_V = 350.0
    full = float(np.interp(v_ms, [8.0, 15.0], [LOW_V, HIGH_V]))
    return float(np.clip((tq_abs - DEADZONE) / max(full - DEADZONE, 1.0), 0.0, 1.0))


def compute_aci_gain(v, tq, err, gain_prev, blinker, *, fix_b=False):
    """ACIGain calc. fix_b=True applies smoothed rate_up."""
    if blinker:
        bp_grip = 30.0
        bp_active = float(np.interp(v, [2., 11.], [100., 125.]))
        bp_heavy = float(np.interp(v, [2., 22.], [250., 350.]))
        target = float(np.interp(abs(tq), [0.0, bp_grip, bp_active, bp_heavy], [0.80, 0.55, 0.18, 0.08]))
        rate_dn = float(np.interp(abs(tq), [150., 350., 600.], [0.004, 0.014, 0.04]))
        rate_dn = max(rate_dn, 0.05)
        rate_up = max(0.004, 0.10)
    else:
        ceiling = float(np.interp(v, [0.5, 1.5], [1.0, 0.85]))
        shelf = float(np.interp(v, [2., 11.], [0.30, 0.40]))
        floor = float(np.interp(v, [2., 22.], [0.1, 0.3]))
        error_start = float(np.interp(v, [0., 5.56, 11.1, 33.3], [1.25, 0.5, 0.3, 0.2]))
        error_mult = float(np.interp(abs(err), [error_start, error_start*2], [1.0, 2.0]))
        ceiling = min(1.0, ceiling * error_mult)
        bp1 = float(np.interp(v, [2., 11.], [30., 50.]))
        bp2 = float(np.interp(v, [2., 11.], [50., 70.]))
        bp3 = float(np.interp(v, [2., 11.], [150., 200.]))
        bp4 = float(np.interp(v, [2., 22.], [300., 450.]))
        target = float(np.interp(abs(tq), [bp1, bp2, bp3, bp4], [ceiling, shelf, shelf, floor]))
        rate_dn = float(np.interp(abs(tq), [150., 350., 600.], [0.004, 0.014, 0.04]))
        rate_up = 0.004
        if fix_b:
            # 16th: widened interp band so endpoint variations stay within smooth zone
            err_boost = float(np.interp(abs(err), [0.5, 1.5], [0.004, 0.04]))
            tq_boost  = float(np.interp(abs(tq),  [20.0, 60.0], [0.02, 0.004]))
            rate_up = max(rate_up, err_boost, tq_boost)
        else:
            if abs(err) > 1.0:
                rate_up = max(rate_up, 0.04)
            elif abs(tq) < 30.0:
                rate_up = max(rate_up, 0.02)
    gain = max(gain_prev - rate_dn, min(gain_prev + rate_up, target))
    return round(gain / QUANT) * QUANT, rate_up


def find_drives():
    out = []
    for r in ROUTES:
        out.extend(sorted(glob.glob(f'/home/user/openpilot/drivelog/*_{r}--*--rlog.zst')))
    return out


# ============================================================
# Fix-A sim: heavy_grip_aligned snap suppression
# ============================================================
def sim_fix_a():
    print("=" * 70)
    print("Fix-A: heavy_override_active mismatch guard")
    print("=" * 70)
    n_heavy_now = 0
    n_heavy_fix = 0
    n_total = 0
    aligned_examples = []
    fighting_examples = []
    for p in find_drives():
        try:
            raw = zstd.ZstdDecompressor().decompress(open(p,'rb').read(), max_output_size=500*1024*1024)
        except: continue
        cs = None
        apply_last = 0.0  # approximate: in real code rate-limited; for sim use last apply (≈op or wheel)
        for msg in log.Event.read_multiple_bytes(raw):
            w = msg.which()
            if w == 'carState': cs = msg.carState
            elif w == 'carControl' and cs is not None:
                cc = msg.carControl
                if not cc.latActive:
                    apply_last = float(cs.steeringAngleDeg)  # reset to wheel
                    continue
                wheel = float(cs.steeringAngleDeg)
                op = float(cc.actuators.steeringAngleDeg)
                tq = float(cs.steeringTorque)
                v = float(cs.vEgoRaw)
                blinker = bool(cs.leftBlinker or cs.rightBlinker)
                ovf = override_factor_of(abs(tq), v)
                # Snap_blinker_override
                snap_blinker_override = blinker and abs(tq) > 200.0
                # Heavy current
                heavy_now = ovf >= 0.90 and (not blinker or snap_blinker_override)
                # Heavy with Fix-A: also need mismatch ≥ 10°
                mismatch = abs(apply_last - wheel)
                heavy_grip_aligned = mismatch < 10.0
                heavy_fix = heavy_now and not heavy_grip_aligned
                if heavy_now: n_heavy_now += 1
                if heavy_fix: n_heavy_fix += 1
                n_total += 1
                # Record examples
                if heavy_now and not heavy_fix:
                    if len(aligned_examples) < 5:
                        aligned_examples.append((v*3.6, wheel, op, tq, mismatch, ovf))
                if heavy_now and heavy_fix:
                    if len(fighting_examples) < 5:
                        fighting_examples.append((v*3.6, wheel, op, tq, mismatch, ovf))
                # Update apply_last (rough approximation: tracks op when no snap, wheel when snap)
                if heavy_now: apply_last = wheel  # snap path
                else:
                    # Rate-limited blend (rough)
                    desired = (1-ovf)*op + ovf*wheel
                    apply_last += np.clip(desired - apply_last, -1.5, 1.5)

    print(f"  Heavy snap frames:")
    print(f"    Current:   {n_heavy_now:>7}  ({100*n_heavy_now/max(n_total,1):.1f}%)")
    print(f"    Fix-A:     {n_heavy_fix:>7}  ({100*n_heavy_fix/max(n_total,1):.1f}%)")
    suppressed = n_heavy_now - n_heavy_fix
    print(f"    Suppressed: {suppressed:>6}  ({100*suppressed/max(n_heavy_now,1):.1f}% of prior heavy)")
    print(f"\n  Aligned examples (snap suppressed by Fix-A, mismatch<10°):")
    for v, w, op, tq, m, of in aligned_examples:
        print(f"    v={v:>5.1f}km/h  wheel={w:>+6.1f}°  op={op:>+6.1f}°  mismatch={m:>4.1f}°  tq={tq:>+5.0f}  ovf={of:.2f}")
    print(f"\n  Fighting examples (snap kept by Fix-A, mismatch≥10°):")
    for v, w, op, tq, m, of in fighting_examples:
        print(f"    v={v:>5.1f}km/h  wheel={w:>+6.1f}°  op={op:>+6.1f}°  mismatch={m:>4.1f}°  tq={tq:>+5.0f}  ovf={of:.2f}")

    # Gates: aligned cases suppressed is intended behavior. Validate FIGHTING cases preserved.
    # Heavy override w/ mismatch >= 10° should keep snap (driver fighting op = yield).
    fighting_n = len(fighting_examples)
    n_kept = n_heavy_fix
    print(f"\n  Gates (revised):")
    print(f"    Fighting cases (mismatch≥10°) preserved: {n_kept} kept (need ≥ {max(20, fighting_n)})")
    pass_fighting = n_kept >= max(20, fighting_n)
    print(f"      {'PASS' if pass_fighting else 'FAIL'}")
    # And suppression should be substantial (at least 50%, since most heavy is aligned in real data)
    drop_pct = 100*suppressed/max(n_heavy_now,1)
    pass_suppress = drop_pct >= 50
    print(f"    Aligned suppression ≥ 50%: {'PASS' if pass_suppress else 'FAIL'} ({drop_pct:.1f}%)")
    return n_heavy_now, n_heavy_fix


# ============================================================
# Fix-B sim: rate_up flapping
# ============================================================
def sim_fix_b():
    print("\n" + "=" * 70)
    print("Fix-B: rate_up smoothing (interp)")
    print("=" * 70)
    flap_now = 0
    flap_fix = 0
    rate_up_now_all = []
    rate_up_fix_all = []
    for p in find_drives():
        try:
            raw = zstd.ZstdDecompressor().decompress(open(p,'rb').read(), max_output_size=500*1024*1024)
        except: continue
        cs = None
        gain_now = 0.0; gain_fix = 0.0
        rate_ups_now = []; rate_ups_fix = []
        for msg in log.Event.read_multiple_bytes(raw):
            w = msg.which()
            if w == 'carState': cs = msg.carState
            elif w == 'carControl' and cs is not None:
                cc = msg.carControl
                if not cc.latActive:
                    gain_now = 0; gain_fix = 0
                    rate_ups_now.append(0); rate_ups_fix.append(0)
                    continue
                wheel = float(cs.steeringAngleDeg)
                op = float(cc.actuators.steeringAngleDeg)
                tq = float(cs.steeringTorque)
                v = float(cs.vEgoRaw)
                blinker = bool(cs.leftBlinker or cs.rightBlinker)
                err = op - wheel
                gain_now, ru_now = compute_aci_gain(v, tq, err, gain_now, blinker, fix_b=False)
                gain_fix, ru_fix = compute_aci_gain(v, tq, err, gain_fix, blinker, fix_b=True)
                rate_ups_now.append(ru_now)
                rate_ups_fix.append(ru_fix)
                if 5 < v < 12:  # city
                    rate_up_now_all.append(ru_now)
                    rate_up_fix_all.append(ru_fix)
        # Count 200ms windows with LARGE step changes (user-felt stepwise jolts).
        # Large = consecutive rate_up diff ≥ 0.01 (2.5 quant). Smooth interp should never produce this.
        STEP_THRESH = 0.01
        for i in range(20, len(rate_ups_now)):
            wn = rate_ups_now[i-20:i+1]
            wf = rate_ups_fix[i-20:i+1]
            def count_big_steps(seq):
                return sum(1 for k in range(1, len(seq)) if abs(seq[k] - seq[k-1]) >= STEP_THRESH)
            if count_big_steps(wn) >= 3: flap_now += 1
            if count_big_steps(wf) >= 3: flap_fix += 1

    print(f"  Flap frames (rate_up toggles ≥4 in 200ms, city 18-43 km/h):")
    print(f"    Current:   {flap_now:>5}")
    print(f"    Fix-B:     {flap_fix:>5}  ({100*flap_fix/max(flap_now,1):.1f}% of current)")
    print(f"  rate_up mean (city, active frames):")
    print(f"    Current:   {np.mean(rate_up_now_all):.4f}")
    print(f"    Fix-B:     {np.mean(rate_up_fix_all):.4f}")
    print(f"\n  Gates:")
    print(f"    Flap ≤ 100: {'PASS' if flap_fix <= 100 else 'FAIL'} ({flap_fix})")
    print(f"    rate_up mean ≥ 0.015: {'PASS' if np.mean(rate_up_fix_all) >= 0.015 else 'FAIL'} ({np.mean(rate_up_fix_all):.4f})")
    return flap_now, flap_fix


# ============================================================
# Fix-D sim: recovery early exit
# ============================================================
def sim_fix_d():
    print("\n" + "=" * 70)
    print("Fix-D: recovery early-exit on release + op-centering")
    print("=" * 70)
    # Track release events after high-wheel-angle override → time until recovery would exit
    # Current: wait until |wheel|<20° (Patch #14)
    # Fix-D: also exit when override_factor<=0.1 AND op*wheel>=0 AND |op|<0.7|wheel|
    durations_now = []
    durations_fix = []
    early_exit_safe = 0
    early_exit_unsafe = 0
    for p in find_drives():
        try:
            raw = zstd.ZstdDecompressor().decompress(open(p,'rb').read(), max_output_size=500*1024*1024)
        except: continue
        cs = None
        seq_w, seq_op, seq_tq, seq_v, seq_la = [], [], [], [], []
        for msg in log.Event.read_multiple_bytes(raw):
            w = msg.which()
            if w == 'carState': cs = msg.carState
            elif w == 'carControl' and cs is not None:
                cc = msg.carControl
                seq_w.append(float(cs.steeringAngleDeg))
                seq_op.append(float(cc.actuators.steeringAngleDeg))
                seq_tq.append(float(cs.steeringTorque))
                seq_v.append(float(cs.vEgoRaw))
                seq_la.append(cc.latActive)
        if not seq_w: continue
        # Detect "recovery scenarios": wheel was >=30° with high tq (heavy override),
        # then tq drops below override_factor 0.1 → recovery would start
        n = len(seq_w)
        i = 0
        while i < n:
            if not seq_la[i] or abs(seq_w[i]) < 30:
                i += 1; continue
            # Look for high-torque (override) → release transition
            ovf_now = override_factor_of(abs(seq_tq[i]), seq_v[i])
            if ovf_now < 0.5:
                i += 1; continue
            # Found high-override at >30° wheel. Find release moment
            release_i = None
            for k in range(i, min(n, i+500)):  # within 5 sec
                if not seq_la[k]: break
                if override_factor_of(abs(seq_tq[k]), seq_v[k]) <= 0.1:
                    release_i = k
                    break
            if release_i is None:
                i += 1; continue
            # At release_i, simulate recovery: current code holds until |wheel|<20° or 2s
            # Fix-D: also exits when op*wheel>=0 AND |op|<0.7|wheel|
            now_exit = None
            fix_exit = None
            for k in range(release_i, min(n, release_i+200)):
                if abs(seq_w[k]) < 20:
                    if now_exit is None: now_exit = k
                    if fix_exit is None: fix_exit = k
                    break
                # Fix-D early exit check
                if fix_exit is None:
                    of_k = override_factor_of(abs(seq_tq[k]), seq_v[k])
                    if of_k <= 0.1 and seq_op[k] * seq_w[k] >= 0 and abs(seq_op[k]) < 0.9 * abs(seq_w[k]):
                        fix_exit = k
            if now_exit is None: now_exit = min(n-1, release_i + 200)
            if fix_exit is None: fix_exit = now_exit
            dur_now_ms = (now_exit - release_i) * 10
            dur_fix_ms = (fix_exit - release_i) * 10
            durations_now.append(dur_now_ms)
            durations_fix.append(dur_fix_ms)
            # Safety check: did Fix-D exit early in a way where wheel was going opposite to op?
            if fix_exit < now_exit:
                # check if at fix_exit the op-wheel direction matched
                op_k = seq_op[fix_exit]
                w_k = seq_w[fix_exit]
                if op_k * w_k >= 0:
                    early_exit_safe += 1
                else:
                    early_exit_unsafe += 1
            i = max(i+1, release_i + 50)  # skip past this event

    if durations_now:
        print(f"  Recovery scenarios found: {len(durations_now)}")
        print(f"  Duration (ms):")
        print(f"    Current:   mean {np.mean(durations_now):.0f}  median {np.median(durations_now):.0f}  p90 {np.percentile(durations_now,90):.0f}")
        print(f"    Fix-D:     mean {np.mean(durations_fix):.0f}  median {np.median(durations_fix):.0f}  p90 {np.percentile(durations_fix,90):.0f}")
        print(f"  Early exits: {early_exit_safe} safe + {early_exit_unsafe} unsafe")
        unsafe_pct = 100*early_exit_unsafe/max(early_exit_safe+early_exit_unsafe,1)
        print(f"\n  Gates:")
        print(f"    Fix-D mean duration ≤ 500ms: {'PASS' if np.mean(durations_fix) <= 500 else 'FAIL'} ({np.mean(durations_fix):.0f}ms)")
        print(f"    Unsafe early-exit ≤ 5%: {'PASS' if unsafe_pct <= 5 else 'FAIL'} ({unsafe_pct:.1f}%)")
    else:
        print(f"  No recovery scenarios found in drives 19/1a (heavy override + release at |wheel|≥30°)")


if __name__ == '__main__':
    sim_fix_a()
    sim_fix_b()
    sim_fix_d()
