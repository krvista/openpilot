#!/usr/bin/env python3
"""Patch #11 simulation — offline A/B against drivelog 00000013.

Loads /tmp/reanalysis_dbc_cache.pkl produced by ioniq6n_reanalysis_dbc.py
and replays two parallel filter banks on the same op_curv input stream:

  A = current code (PR #10 merged, commit 94749b9)
  B = patch #11 candidate (S1 + S2 + S3)

S1: carcontroller.py:793   `vtau = max(angle_tau, speed_tau)`
                        → `vtau = max(min(angle_tau, max(speed_tau, 0.5)), speed_tau)`
S2: values.py:128-129     VTAU_ENTRY_TH_BP/V — add 9 m/s notch (0.25°)
S3: carcontroller.py:729   `hands_off = not steeringPressed`
                        → `hands_off = (not steeringPressed) and (override_factor <= 0.5)`

Outputs:
  /tmp/patch11_sim_results.json   — per-bucket metrics
  stdout                          — human-readable summary + gate verdicts

Honest limits documented in /root/.claude/plans/ccnc-drivelog-deep-seal.md
Phase C — Open-loop only, no closed-loop physics.
"""
import os
import sys
import json
import pickle
from collections import defaultdict

import numpy as np

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')


# ---------------------------------------------------------------------------
# Constants mirroring opendbc_repo/opendbc/car/hyundai/values.py + carcontroller.py
# ---------------------------------------------------------------------------
LPF_DT = 0.01  # 100 Hz

# values.py:112-115
VTAU_ANGLE_BP = [0.0, 1.0, 3.0, 10.0]
VTAU_ANGLE_V  = [3.5, 0.4, 0.20, 0.20]
VTAU_SPEED_BP = [0.0, 3.0, 5.0, 15.0]
VTAU_SPEED_V  = [0.5, 0.3, 0.20, 0.0]

# values.py:128-129 — A (current) vs B (S2 patch)
VTAU_ENTRY_TH_BP_A = [4.0, 15.0, 25.0]
VTAU_ENTRY_TH_V_A  = [0.3, 0.5, 0.5]
VTAU_ENTRY_TH_BP_B = [4.0, 9.0, 15.0, 25.0]
VTAU_ENTRY_TH_V_B  = [0.3, 0.25, 0.4, 0.5]

VTAU_EXIT_TH  = 0.3

# values.py:189-191 (i6n CCNC angle, no-blinker baseline)
DRIVER_TORQUE_DEADZONE = 100.0
DRIVER_TORQUE_FULL_OVERRIDE_LOW_V  = 180.0
DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V = 350.0
DRIVER_TORQUE_LOW_V_SPEED  = 8.0
DRIVER_TORQUE_HIGH_V_SPEED = 15.0

# carcontroller.py:54-55
LOW_SPEED_PASSTHROUGH_ENTER_MS = 20.0 / 3.6  # 5.56 m/s
LOW_SPEED_PASSTHROUGH_EXIT_MS  = 22.0 / 3.6  # 6.11 m/s


def compute_override_factor(abs_torque, v_ms):
    """Mirrors carcontroller.py:622-633 (no-blinker path)."""
    full = float(np.interp(v_ms,
                           [DRIVER_TORQUE_LOW_V_SPEED, DRIVER_TORQUE_HIGH_V_SPEED],
                           [DRIVER_TORQUE_FULL_OVERRIDE_LOW_V,
                            DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V]))
    return float(np.clip((abs_torque - DRIVER_TORQUE_DEADZONE) /
                         max(full - DRIVER_TORQUE_DEADZONE, 1.0), 0.0, 1.0))


class VTauFilter:
    """Port of carcontroller.py:767-835.

    Mode 'A' uses current code (max(angle_tau, speed_tau) + entry_th_A).
    Mode 'B' uses S1+S2 patch.
    """
    def __init__(self, mode):
        assert mode in ('A', 'B')
        self.mode = mode
        self.vtau_lpf = 0.0
        self.vtau_sustained_cnt = 0
        self.vtau_prev_sign = 0

    def step(self, op_curv, v_ms, lat_active, in_passthrough, actual_wheel):
        if self.mode == 'A':
            bp, v = VTAU_ENTRY_TH_BP_A, VTAU_ENTRY_TH_V_A
        else:
            bp, v = VTAU_ENTRY_TH_BP_B, VTAU_ENTRY_TH_V_B
        entry_th = float(np.interp(v_ms, bp, v))
        entering_curve = abs(op_curv) > abs(self.vtau_lpf) + entry_th
        returning_to_center = abs(op_curv) < abs(self.vtau_lpf) - VTAU_EXIT_TH

        if entering_curve or returning_to_center:
            vtau = 0.05
            self.vtau_sustained_cnt = 60
        else:
            abs_angle = abs(self.vtau_lpf)
            angle_tau = float(np.interp(abs_angle, VTAU_ANGLE_BP, VTAU_ANGLE_V))
            speed_tau = float(np.interp(v_ms, VTAU_SPEED_BP, VTAU_SPEED_V))
            if self.mode == 'A':
                # current: angle dominates at center
                vtau = max(angle_tau, speed_tau)
            else:
                # S1: speed-tau caps angle-tau (floored at 0.5 s so low-speed
                # crawl behaviour is preserved).
                vtau = max(min(angle_tau, max(speed_tau, 0.5)), speed_tau)
            speed_max_tau = float(np.interp(v_ms, [10.0, 25.0], [2.5, 0.22]))
            vtau = min(vtau, speed_max_tau)

            cur_sign = 1 if op_curv > self.vtau_lpf + 0.01 else \
                       (-1 if op_curv < self.vtau_lpf - 0.01 else 0)
            if cur_sign != 0 and cur_sign == self.vtau_prev_sign:
                self.vtau_sustained_cnt = min(self.vtau_sustained_cnt + 1, 100)
            else:
                self.vtau_sustained_cnt = max(self.vtau_sustained_cnt - 2, 0)
            self.vtau_prev_sign = cur_sign
            vtau = float(np.interp(self.vtau_sustained_cnt, [0, 30, 60],
                                   [vtau, min(vtau, 0.5), min(vtau, 0.1)]))

        if lat_active and not in_passthrough and vtau > 0.001:
            alpha = LPF_DT / (vtau + LPF_DT)
            self.vtau_lpf = alpha * op_curv + (1.0 - alpha) * self.vtau_lpf
        else:
            self.vtau_lpf = actual_wheel
        return self.vtau_lpf, vtau, entry_th


def replay_segment(frames):
    """Run A/B vtau filters + S3 hands-off state machines over one segment."""
    fa = VTauFilter('A')
    fb = VTauFilter('B')

    low_speed_cam_latched_A = False
    low_speed_cam_latched_B = False

    out = []
    for fr in frames:
        v = fr['v_ms']
        op_curv = fr['desired']
        actual = fr['actual']
        lat = bool(fr['lat_active'])
        pressed = bool(fr['steering_pressed'])
        abs_tq = float(fr['pressed_torque'])

        # in_passthrough proxy: we don't have the latched flag direct in cache.
        # Approximate by `not lat_active` — close enough for vtau gate purposes.
        in_pt_proxy = not lat

        apply_A, vtau_A, entry_A = fa.step(op_curv, v, lat, in_pt_proxy, actual)
        apply_B, vtau_B, entry_B = fb.step(op_curv, v, lat, in_pt_proxy, actual)

        # S3: hands_off computation
        ovf = compute_override_factor(abs_tq, v)
        hands_off_A = not pressed
        hands_off_B = (not pressed) and (ovf <= 0.5)

        # low_speed_cam_latched state machine (post-PR #10 logic)
        # A: PR #10 hands_off; B: S3 hands_off
        if v < LOW_SPEED_PASSTHROUGH_ENTER_MS and not hands_off_A:
            low_speed_cam_latched_A = True
        elif v > LOW_SPEED_PASSTHROUGH_EXIT_MS or hands_off_A:
            low_speed_cam_latched_A = False
        if v < LOW_SPEED_PASSTHROUGH_ENTER_MS and not hands_off_B:
            low_speed_cam_latched_B = True
        elif v > LOW_SPEED_PASSTHROUGH_EXIT_MS or hands_off_B:
            low_speed_cam_latched_B = False

        out.append({
            'route': fr['route'], 'seg': fr['seg'], 'v_ms': v, 'v_kmh': fr['v_kmh'],
            'op_curv': op_curv, 'actual': actual, 'lat_active': lat,
            'pressed_torque': abs_tq, 'steering_pressed': pressed,
            'override_factor': ovf,
            'apply_A': apply_A, 'apply_B': apply_B,
            'vtau_A': vtau_A, 'vtau_B': vtau_B,
            'entry_A': entry_A, 'entry_B': entry_B,
            'hands_off_A': hands_off_A, 'hands_off_B': hands_off_B,
            'lscl_A': low_speed_cam_latched_A, 'lscl_B': low_speed_cam_latched_B,
        })
    return out


def metrics_s1_s2(rows):
    """Speed-bucketed |apply - op_curv| comparison."""
    bins = [(0,5), (5,10), (10,20), (20,30), (30,40), (40,50), (50,60), (60,80), (80,200)]
    out = []
    for lo, hi in bins:
        sel = [r for r in rows if lo <= r['v_kmh'] < hi and r['lat_active']]
        if not sel:
            continue
        err_A = np.array([abs(r['apply_A'] - r['op_curv']) for r in sel])
        err_B = np.array([abs(r['apply_B'] - r['op_curv']) for r in sel])
        out.append({
            'bucket': f'{lo}-{hi}', 'n': len(sel),
            'median_A': float(np.median(err_A)), 'median_B': float(np.median(err_B)),
            'p90_A': float(np.percentile(err_A, 90)),
            'p90_B': float(np.percentile(err_B, 90)),
            'improvement_pct': 100.0 * (np.median(err_A) - np.median(err_B)) / max(np.median(err_A), 1e-6),
        })
    return out


def metrics_s3(rows):
    """Count low_speed_cam_latched falling edges with active grip."""
    A_falls_active_grip = 0
    A_falls_total = 0
    B_falls_active_grip = 0
    B_falls_total = 0
    prev_A = False
    prev_B = False
    for r in rows:
        if prev_A and not r['lscl_A']:
            A_falls_total += 1
            if r['pressed_torque'] > 120:
                A_falls_active_grip += 1
        if prev_B and not r['lscl_B']:
            B_falls_total += 1
            if r['pressed_torque'] > 120:
                B_falls_active_grip += 1
        prev_A = r['lscl_A']
        prev_B = r['lscl_B']

    # Also count frames where lscl flipped solely due to coarse hands_off vs
    # tight S3 detection — these are the "saved" frames at active grip.
    saved_frames = sum(1 for r in rows
                       if r['lscl_B'] and not r['lscl_A']
                       and r['pressed_torque'] > 120
                       and r['v_ms'] < LOW_SPEED_PASSTHROUGH_EXIT_MS)

    # Distribution of override_factor at frames where A says hands_off but B doesn't.
    deltas = [r['override_factor'] for r in rows if r['hands_off_A'] and not r['hands_off_B']]

    return {
        'A_falling_edges_total': A_falls_total,
        'A_falling_edges_active_grip': A_falls_active_grip,
        'B_falling_edges_total': B_falls_total,
        'B_falling_edges_active_grip': B_falls_active_grip,
        'saved_frames_active_grip_low_speed': saved_frames,
        'hands_off_disagreement_count': len(deltas),
        'hands_off_disagreement_override_factor_p50': float(np.median(deltas)) if deltas else None,
        'hands_off_disagreement_override_factor_max': float(np.max(deltas)) if deltas else None,
    }


def main():
    cache = '/tmp/reanalysis_dbc_cache.pkl'
    if not os.path.exists(cache):
        print(f"ERROR: {cache} not found. Run tools/ioniq6n_reanalysis_dbc.py first.")
        sys.exit(1)
    with open(cache, 'rb') as f:
        data = pickle.load(f)
    if isinstance(data, tuple):
        all_frames = data[0]
    else:
        all_frames = data

    # Restrict to drivelog 00000013 only (8 segs: 0-3, 18-21).
    frames = [f for f in all_frames if f['route'] == '00000013']
    if not frames:
        print("ERROR: no 00000013 frames in cache.")
        sys.exit(1)
    print(f"Loaded {len(frames):,} frames from 00000013 across "
          f"{len(set(f['seg'] for f in frames))} segments.")

    # Group by segment and replay each independently (vtau state per seg).
    by_seg = defaultdict(list)
    for f in frames:
        by_seg[f['seg']].append(f)
    all_rows = []
    for seg in sorted(by_seg.keys()):
        seg_frames = by_seg[seg]
        rows = replay_segment(seg_frames)
        all_rows.extend(rows)
        print(f"  seg {seg}: {len(rows):,} frames replayed")

    print("\n=== S1+S2: |apply - op_curv| by speed bucket (A=current, B=patched) ===")
    print(f"{'bucket':>10} {'n':>8} {'med_A':>8} {'med_B':>8} {'p90_A':>8} {'p90_B':>8} {'Δmed%':>8}")
    s12 = metrics_s1_s2(all_rows)
    for r in s12:
        print(f"{r['bucket']:>10} {r['n']:>8} {r['median_A']:>8.3f} {r['median_B']:>8.3f} "
              f"{r['p90_A']:>8.3f} {r['p90_B']:>8.3f} {r['improvement_pct']:>8.1f}")

    print("\n=== S3: low_speed_cam_latched falling-edge analysis ===")
    s3 = metrics_s3(all_rows)
    for k, val in s3.items():
        print(f"  {k}: {val}")

    # Gate verdicts
    print("\n=== Gate verdicts ===")
    mid_bucket = next((r for r in s12 if r['bucket'] == '30-40'), None)
    mid40 = next((r for r in s12 if r['bucket'] == '40-50'), None)
    pass_s1s2 = False
    if mid_bucket and mid_bucket['improvement_pct'] >= 20:
        pass_s1s2 = True
    elif mid40 and mid40['improvement_pct'] >= 20:
        pass_s1s2 = True
    print(f"  S1+S2 30-50 km/h ≥20% median improvement: "
          f"{'PASS' if pass_s1s2 else 'FAIL'}")

    pass_s3 = s3['saved_frames_active_grip_low_speed'] > 0
    print(f"  S3 saves ≥1 active-grip low-speed frame: "
          f"{'PASS' if pass_s3 else 'FAIL (no active-grip false-positive observed)'}")

    out = {'s1_s2': s12, 's3': s3, 'gates': {'s1_s2': pass_s1s2, 's3': pass_s3}}
    with open('/tmp/patch11_sim_results.json', 'w') as f:
        json.dump(out, f, indent=2)
    print("\nResults: /tmp/patch11_sim_results.json")


if __name__ == '__main__':
    main()
