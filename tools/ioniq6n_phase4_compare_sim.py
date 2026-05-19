#!/usr/bin/env python3
"""Phase 4 comparison sim — OLD (pre-Phase-1 PR-stacked) vs NEW (sunnypilot reference).

Replays the cached drivelog (/tmp/reanalysis_dbc_cache.pkl, drive 00000013,
47k frames across 8 segments) through TWO parallel angle/ACIGain pipelines
and reports which PR #11-17 effects are preserved vs lost under the new
reference-only flow committed in Phase 1+2.

OLD pipeline (carcontroller before commit 54ab570):
  - vtau LPF with entry/exit threshold + sustained-direction counter
  - override_snap state machine (enter/exit hysteresis)
  - post_override_recovery hold (200-frame timeout, 30°/20° band)
  - extended compute_torque_reduction_gain with rate_up boost + city shelf
  - blinker-specific deadzone/full-override + blinker_frac LPF

NEW pipeline (carcontroller at HEAD = a80add5):
  - sp_smooth_angle EMA on commanded angle (alpha 0.05→1.0 over 0-18 m/s)
  - apply_steer_angle_limits_vm with main VM (panda safety envelope)
  - reference compute_torque_reduction_gain (17 lines, no boosts)
  - no snap, no recovery, no blinker_frac, no jitter

Metrics (mirroring patch sims #11/#15/#17):
  1. |apply - op_curv| by speed bucket  (#11 / #17)
  2. ACIGain mean during light-grip city + drift events  (#15)
  3. apply_angle stability after heavy-grip release proxy  (#12/#13/#14/#16)
"""
import os
import pickle
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, '/home/user/openpilot')
sys.path.insert(0, '/home/user/openpilot/opendbc_repo')

# ----- constants used by OLD pipeline (snapshot from pre-Phase-1 values.py) -----
DT_CTRL = 0.01
LPF_DT  = DT_CTRL
VTAU_ANGLE_BP = [0.0, 1.0, 3.0, 10.0]
VTAU_ANGLE_V  = [3.5, 0.4, 0.20, 0.20]
VTAU_SPEED_BP = [0.0, 3.0, 5.0, 15.0]
VTAU_SPEED_V  = [0.5, 0.3, 0.20, 0.0]
VTAU_ENTRY_TH_BP = [4.0, 15.0, 25.0]
VTAU_ENTRY_TH_V  = [0.3, 0.5, 0.5]
VTAU_EXIT_TH     = 0.3
SPEED_MAX_TAU_BP = [5.0, 10.0, 15.0, 25.0]   # #17 Cand A merged
SPEED_MAX_TAU_V  = [0.80, 0.50, 0.30, 0.22]

DRIVER_TORQUE_DEADZONE_ANGLE              = 100.0
DRIVER_TORQUE_FULL_OVERRIDE_LOW_V_ANGLE   = 180.0
DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V_ANGLE  = 350.0
DRIVER_TORQUE_DEADZONE_ANGLE_BLINKER      = 70.0
DRIVER_TORQUE_FULL_OVERRIDE_LOW_V_BLINKER = 130.0
DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V_BLINKER= 220.0
DRIVER_TORQUE_LOW_V_SPEED  = 8.0
DRIVER_TORQUE_HIGH_V_SPEED = 15.0
OVERRIDE_SNAP_ENTER_FACTOR = 0.90
OVERRIDE_SNAP_ENTER_FRAMES = 3
OVERRIDE_SNAP_EXIT_FACTOR  = 0.10
OVERRIDE_SNAP_EXIT_FRAMES  = 10
RECOVERY_ENTER_ABS_DEG     = 30.0
RECOVERY_EXIT_ABS_DEG      = 20.0
RECOVERY_TIMEOUT_FRAMES    = 200
HANDS_OFF_RECOVERY_ANGLE_DEG = 50.0
HANDS_OFF_MISMATCH_DEG       = 20.0
LOW_SPEED_PASSTHROUGH_ENTER_MS = 20.0 / 3.6
LOW_SPEED_PASSTHROUGH_EXIT_MS  = 22.0 / 3.6
ACI_GAIN_QUANT = 0.004

# ----- constants used by NEW pipeline (mirror values.py at a80add5) -----
SMOOTHING_ANGLE_VEGO_MATRIX  = [0, 8.5, 11, 13.8, 18]
SMOOTHING_ANGLE_ALPHA_MATRIX = [0.05, 0.1, 0.3, 0.6, 1]
SMOOTHING_ANGLE_MAX_VEGO     = 18

# ----- shared helpers -----
def _ovf(abs_tq, v_ms, blinker_frac=0.0):
    """OLD: blinker-lerped override_factor."""
    dz = DRIVER_TORQUE_DEADZONE_ANGLE + (DRIVER_TORQUE_DEADZONE_ANGLE_BLINKER - DRIVER_TORQUE_DEADZONE_ANGLE) * blinker_frac
    lo = DRIVER_TORQUE_FULL_OVERRIDE_LOW_V_ANGLE + (DRIVER_TORQUE_FULL_OVERRIDE_LOW_V_BLINKER - DRIVER_TORQUE_FULL_OVERRIDE_LOW_V_ANGLE) * blinker_frac
    hi = DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V_ANGLE + (DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V_BLINKER - DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V_ANGLE) * blinker_frac
    full = float(np.interp(v_ms, [DRIVER_TORQUE_LOW_V_SPEED, DRIVER_TORQUE_HIGH_V_SPEED], [lo, hi]))
    return float(np.clip((abs_tq - dz) / max(full - dz, 1.0), 0.0, 1.0))


def _ovf_new(abs_tq, v_ms):
    """NEW: no-blinker, no-LPF override_factor (carcontroller a80add5:325-336)."""
    full = float(np.interp(v_ms, [DRIVER_TORQUE_LOW_V_SPEED, DRIVER_TORQUE_HIGH_V_SPEED],
                            [DRIVER_TORQUE_FULL_OVERRIDE_LOW_V_ANGLE, DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V_ANGLE]))
    return float(np.clip((abs_tq - DRIVER_TORQUE_DEADZONE_ANGLE) /
                          max(full - DRIVER_TORQUE_DEADZONE_ANGLE, 1.0), 0.0, 1.0))


# ----- OLD vtau filter + override snap state machine -----
class OldPipeline:
    def __init__(self):
        self.vtau_lpf = 0.0
        self.vtau_sustained_cnt = 0
        self.vtau_prev_sign = 0
        self.apply_angle_last = 0.0
        self.snapped = False
        self.enter_cnt = 0
        self.exit_cnt = 0
        self.recovery = False
        self.recovery_remaining = 0
        self.steer_torque_lpf = 0.0
        self.blinker_frac = 0.0
        self.aci_gain_last = 0.0

    def step(self, op_curv, v_ms, actual, lat_active, pressed_torque,
             steering_pressed, blinker_on, cam_aci_gain):
        # --- steer_torque_lpf (30 ms) + blinker_frac (300 ms) ---
        self.steer_torque_lpf = 0.25 * pressed_torque + 0.75 * self.steer_torque_lpf
        steer_tq_safe = self.steer_torque_lpf
        self.blinker_frac = 0.032 * (1.0 if blinker_on else 0.0) + 0.968 * self.blinker_frac

        # --- override_factor ---
        ovf = _ovf(abs(steer_tq_safe), v_ms, self.blinker_frac)

        # --- snap state machine (simplified — heavy-only) ---
        heavy_grip_aligned = abs(self.apply_angle_last - actual) < 10.0
        heavy_active = (ovf >= OVERRIDE_SNAP_ENTER_FACTOR and not heavy_grip_aligned
                        and (not blinker_on or abs(steer_tq_safe) > 200.0))
        moderate_entry = (abs(actual) >= 50.0 and abs(steer_tq_safe) >= 30.0
                          and not self.snapped
                          and abs(self.apply_angle_last - actual) >= 20.0)
        if heavy_active or moderate_entry:
            self.enter_cnt += 1; self.exit_cnt = 0
        elif ovf <= OVERRIDE_SNAP_EXIT_FACTOR and not (abs(actual) >= 30.0 and abs(steer_tq_safe) >= 20.0):
            self.exit_cnt += 1; self.enter_cnt = 0
        prev_snapped = self.snapped
        if not self.snapped and self.enter_cnt >= OVERRIDE_SNAP_ENTER_FRAMES:
            self.snapped = True
        elif self.snapped and self.exit_cnt >= OVERRIDE_SNAP_EXIT_FRAMES:
            self.snapped = False

        # --- recovery ---
        if prev_snapped and not self.snapped and abs(actual) >= RECOVERY_ENTER_ABS_DEG:
            self.recovery = True; self.recovery_remaining = RECOVERY_TIMEOUT_FRAMES
        elif (not self.recovery and abs(actual) >= HANDS_OFF_RECOVERY_ANGLE_DEG
              and ovf <= 0.1 and abs(self.apply_angle_last - actual) >= HANDS_OFF_MISMATCH_DEG):
            self.recovery = True; self.recovery_remaining = RECOVERY_TIMEOUT_FRAMES
        if self.recovery:
            self.recovery_remaining -= 1
            if abs(actual) < RECOVERY_EXIT_ABS_DEG or self.recovery_remaining <= 0:
                self.recovery = False

        # --- vtau LPF ---
        entry_th = float(np.interp(v_ms, VTAU_ENTRY_TH_BP, VTAU_ENTRY_TH_V))
        entering = abs(op_curv) > abs(self.vtau_lpf) + entry_th
        returning = abs(op_curv) < abs(self.vtau_lpf) - VTAU_EXIT_TH
        if entering or returning:
            vtau = 0.05
            self.vtau_sustained_cnt = 60
        else:
            angle_tau = float(np.interp(abs(self.vtau_lpf), VTAU_ANGLE_BP, VTAU_ANGLE_V))
            speed_tau = float(np.interp(v_ms, VTAU_SPEED_BP, VTAU_SPEED_V))
            vtau = max(angle_tau, speed_tau)
            cap   = float(np.interp(v_ms, SPEED_MAX_TAU_BP, SPEED_MAX_TAU_V))
            vtau = min(vtau, cap)
            cur_sign = 1 if op_curv > self.vtau_lpf + 0.01 else (-1 if op_curv < self.vtau_lpf - 0.01 else 0)
            if cur_sign != 0 and cur_sign == self.vtau_prev_sign:
                self.vtau_sustained_cnt = min(self.vtau_sustained_cnt + 1, 100)
            else:
                self.vtau_sustained_cnt = max(self.vtau_sustained_cnt - 2, 0)
            self.vtau_prev_sign = cur_sign
            vtau = float(np.interp(self.vtau_sustained_cnt, [0, 30, 60],
                                    [vtau, min(vtau, 0.5), min(vtau, 0.1)]))

        if lat_active and vtau > 0.001:
            alpha = LPF_DT / (vtau + LPF_DT)
            self.vtau_lpf = alpha * op_curv + (1.0 - alpha) * self.vtau_lpf
        else:
            self.vtau_lpf = actual

        # --- desired angle + override blend ---
        desired = self.vtau_lpf
        if ovf > 0:
            desired = (1.0 - ovf) * desired + ovf * actual

        # --- apply_angle update with snap/recovery ---
        if self.snapped or self.recovery:
            self.apply_angle_last = actual
        else:
            self.apply_angle_last = desired

        # --- extended ACIGain (rate_up boost + dynamic ceiling) ---
        steering_error = self.apply_angle_last - actual
        v_kph = v_ms * 3.6
        if lat_active:
            base_ceiling = float(np.interp(v_kph, [0, 20, 40, 120], [0.4, 0.62, 0.85, 1.0]))
            error_start  = float(np.interp(v_kph, [0, 20, 40, 120], [1.25, 0.5, 0.3, 0.2]))
            error_mult   = float(np.interp(abs(steering_error), [error_start, error_start * 2], [1.0, 2.0]))
            dyn_ceiling  = min(1.0, base_ceiling * error_mult)
            target = float(np.interp(abs(steer_tq_safe), [140, 420], [dyn_ceiling, 0.19]))
        else:
            target = 0.0
        delta = target - self.aci_gain_last
        rate_dn = float(np.interp(abs(steer_tq_safe), [0, 300, 700], [0.004, 0.01, 0.04]))
        err_boost = float(np.interp(abs(steering_error), [0.5, 1.5], [0.004, 0.04]))
        tq_boost  = float(np.interp(abs(steer_tq_safe), [20.0, 100.0], [0.02, 0.004]))
        rate_up = max(0.004, err_boost, tq_boost)
        gain = self.aci_gain_last + max(-rate_dn, min(rate_up, delta))
        self.aci_gain_last = round(gain / ACI_GAIN_QUANT) * ACI_GAIN_QUANT
        return self.apply_angle_last, self.aci_gain_last, self.snapped, self.recovery


# ----- NEW pipeline (sunnypilot reference) -----
def _sp_smooth_angle(v_ego_raw, apply_angle, apply_angle_last):
    a = float(np.interp(v_ego_raw, SMOOTHING_ANGLE_VEGO_MATRIX, SMOOTHING_ANGLE_ALPHA_MATRIX))
    a = float(min(a, 1.))
    return apply_angle * a + apply_angle_last * (1.0 - a)


def _ref_aci_gain(steering_torque, v_kph, lat_active, last_gain, steering_error):
    if lat_active:
        base_ceiling = float(np.interp(v_kph, [0, 20, 40, 120], [0.4, 0.62, 0.85, 1.0]))
        error_start = float(np.interp(v_kph, [0, 20, 40, 120], [1.25, 0.5, 0.3, 0.2]))
        error_mult = float(np.interp(abs(steering_error), [error_start, error_start * 2], [1.0, 2.0]))
        dyn_ceiling = min(1.0, base_ceiling * error_mult)
        target = float(np.interp(abs(steering_torque), [140, 420], [dyn_ceiling, 0.19]))
    else:
        target = 0.0
    delta = target - last_gain
    rate_dn = float(np.interp(abs(steering_torque), [0, 300, 700], [0.004, 0.01, 0.04]))
    gain = last_gain + max(-rate_dn, min(0.004, delta))
    return round(gain / 0.004) * 0.004


class NewPipeline:
    def __init__(self):
        self.apply_angle_last = 0.0
        self.aci_gain_last = 0.0

    def step(self, op_curv, v_ms, actual, lat_active, pressed_torque, steering_pressed, blinker_on, cam_aci_gain):
        # Step 1: clip + sp_smooth_angle (no LPF, no snap)
        desired = float(np.clip(op_curv, -360.0, 360.0))
        if abs(v_ms) < SMOOTHING_ANGLE_MAX_VEGO:
            desired = _sp_smooth_angle(v_ms, desired, self.apply_angle_last)
        # Step 2: NO double VM rate limiter in this closed-form sim
        #   (the real code passes through apply_steer_angle_limits_vm which
        #    enforces jerk/accel/rate envelope — at 100 Hz, the smoothing
        #    EMA dominates inside the envelope for typical drivelog frames.
        #    Tracking error vs. op_curv reported here is therefore an
        #    upper-bound on tracking quality; the VM clamp only kicks in
        #    when |Δapply| > MAX_ANGLE_RATE=5°/frame or curvature exceeds
        #    the lateral accel/jerk envelope.)
        if lat_active:
            self.apply_angle_last = desired
        else:
            self.apply_angle_last = float(np.clip(actual, -360.0, 360.0))

        steering_error = self.apply_angle_last - actual
        gain = _ref_aci_gain(pressed_torque, v_ms * 3.6, lat_active, self.aci_gain_last, steering_error)
        self.aci_gain_last = gain
        return self.apply_angle_last, self.aci_gain_last, False, False


# ----- metric helpers -----
def speed_bucket_metrics(rows, label_a, label_n):
    bins = [(0,10), (10,20), (20,30), (30,40), (40,50), (50,60), (60,80), (80,200)]
    out = []
    for lo, hi in bins:
        sel = [r for r in rows if lo <= r['v_kmh'] < hi and r['lat_active']]
        if len(sel) < 20:
            continue
        err_a = np.array([abs(r['apply_old'] - r['op_curv']) for r in sel])
        err_n = np.array([abs(r['apply_new'] - r['op_curv']) for r in sel])
        out.append({
            'bucket': f'{lo}-{hi}',
            'n': len(sel),
            f'p50_{label_a}': float(np.median(err_a)),
            f'p50_{label_n}': float(np.median(err_n)),
            f'p90_{label_a}': float(np.percentile(err_a, 90)),
            f'p90_{label_n}': float(np.percentile(err_n, 90)),
        })
    return out


def aci_gain_metrics(rows):
    """ACIGain mean across event types (mirrors patch15 metrics)."""
    out = {}
    # Light-grip city: 20-50 km/h, |torque| <= 100 Nm, lat_active
    sel_light = [r for r in rows if 20 <= r['v_kmh'] < 50 and abs(r['pressed_torque']) <= 100 and r['lat_active']]
    if sel_light:
        out['light_grip_city'] = {
            'n': len(sel_light),
            'old_mean': float(np.mean([r['gain_old'] for r in sel_light])),
            'new_mean': float(np.mean([r['gain_new'] for r in sel_light])),
        }
    # Drift events: |steering_error| > 5°, lat_active
    sel_drift = [r for r in rows if abs(r['apply_old'] - r['actual']) > 5 and r['lat_active']]
    if sel_drift:
        out['drift_events_oldframe'] = {
            'n': len(sel_drift),
            'old_mean': float(np.mean([r['gain_old'] for r in sel_drift])),
            'new_mean': float(np.mean([r['gain_new'] for r in sel_drift])),
        }
    sel_drift_n = [r for r in rows if abs(r['apply_new'] - r['actual']) > 5 and r['lat_active']]
    if sel_drift_n:
        out['drift_events_newframe'] = {
            'n': len(sel_drift_n),
            'old_mean': float(np.mean([r['gain_old'] for r in sel_drift_n])),
            'new_mean': float(np.mean([r['gain_new'] for r in sel_drift_n])),
        }
    return out


def heavy_release_metrics(rows):
    """Find OLD-snap-exit events and measure NEW behaviour in the same window."""
    snap_exits = []
    prev_snap = False
    for i, r in enumerate(rows):
        if prev_snap and not r['snap_old']:
            snap_exits.append(i)
        prev_snap = r['snap_old']
    if not snap_exits:
        return {'n_snap_exits': 0}
    OLD_devs, NEW_devs = [], []
    for idx in snap_exits:
        end = min(idx + 100, len(rows))
        window = rows[idx:end]
        OLD_devs.append(max(abs(r['apply_old'] - r['actual']) for r in window))
        NEW_devs.append(max(abs(r['apply_new'] - r['actual']) for r in window))
    return {
        'n_snap_exits': len(snap_exits),
        'old_max_dev_p50':  float(np.median(OLD_devs)),
        'new_max_dev_p50':  float(np.median(NEW_devs)),
        'old_max_dev_p90':  float(np.percentile(OLD_devs, 90)),
        'new_max_dev_p90':  float(np.percentile(NEW_devs, 90)),
    }


def jitter_metrics(rows):
    """Per-frame apply-angle delta — proxy for wheel jitter the EPS would
    see. OLD vtau LPF intentionally smooths op_curv → small |Δapply|.
    NEW sp_smooth_angle has alpha=1.0 at v≥18 m/s so |Δapply| ≈ |Δop_curv|.
    """
    bins = [(0, 5), (5, 10), (10, 20), (20, 30), (30, 50), (50, 80), (80, 200)]
    out = []
    for lo, hi in bins:
        sel = [(rows[i-1], rows[i]) for i in range(1, len(rows))
                if lo <= rows[i]['v_kmh'] < hi and rows[i]['lat_active'] and rows[i-1]['lat_active']
                and rows[i]['route'] == rows[i-1]['route'] and rows[i]['seg'] == rows[i-1]['seg']]
        if len(sel) < 20:
            continue
        d_old = np.array([abs(b['apply_old'] - a['apply_old']) for a, b in sel])
        d_new = np.array([abs(b['apply_new'] - a['apply_new']) for a, b in sel])
        out.append({
            'bucket': f'{lo}-{hi}', 'n': len(sel),
            'p50_old': float(np.median(d_old)), 'p50_new': float(np.median(d_new)),
            'p90_old': float(np.percentile(d_old, 90)), 'p90_new': float(np.percentile(d_new, 90)),
            'p99_old': float(np.percentile(d_old, 99)), 'p99_new': float(np.percentile(d_new, 99)),
        })
    return out


# ----- Phase 5 — augmented NEW pipeline (B1/B2/B3/A2/S1 toggles) -----
PHASE5_BLEND_DEADBAND = 0.3
PHASE5_BLINKER_ACI_CAP = 0.45
PHASE5_VM_REJECT_FORCE_PASSIVE_FRAMES = 5


class NewPipelineV5:
    def __init__(self, b1=False, b2=False, b3=False, a2=False, s1=False):
        self.apply_angle_last = 0.0
        self.aci_gain_last = 0.0
        self.vm_reject_consecutive = 0
        self.b1, self.b2, self.b3, self.a2, self.s1 = b1, b2, b3, a2, s1

    def _override_factor(self, abs_tq, v_ms, blinker_on):
        if self.b2 and blinker_on:
            dz = DRIVER_TORQUE_DEADZONE_ANGLE_BLINKER
            lo = DRIVER_TORQUE_FULL_OVERRIDE_LOW_V_BLINKER
            hi = DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V_BLINKER
        else:
            dz = DRIVER_TORQUE_DEADZONE_ANGLE
            lo = DRIVER_TORQUE_FULL_OVERRIDE_LOW_V_ANGLE
            hi = DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V_ANGLE
        full = float(np.interp(v_ms, [DRIVER_TORQUE_LOW_V_SPEED, DRIVER_TORQUE_HIGH_V_SPEED], [lo, hi]))
        return float(np.clip((abs_tq - dz) / max(full - dz, 1.0), 0.0, 1.0))

    def _aci_gain(self, steering_torque, v_kph, lat_active, last_gain, steering_error, blinker_on):
        if lat_active:
            base_ceiling = float(np.interp(v_kph, [0, 20, 40, 120], [0.4, 0.62, 0.85, 1.0]))
            error_start = float(np.interp(v_kph, [0, 20, 40, 120], [1.25, 0.5, 0.3, 0.2]))
            error_mult = float(np.interp(abs(steering_error), [error_start, error_start * 2], [1.0, 2.0]))
            dyn_ceiling = min(1.0, base_ceiling * error_mult)
            if self.a2 and blinker_on:
                dyn_ceiling = min(dyn_ceiling, PHASE5_BLINKER_ACI_CAP)
            target = float(np.interp(abs(steering_torque), [140, 420], [dyn_ceiling, 0.19]))
        else:
            target = 0.0
        delta = target - last_gain
        rate_dn = float(np.interp(abs(steering_torque), [0, 300, 700], [0.004, 0.01, 0.04]))
        rate_up = float(np.interp(abs(steering_error), [0.5, 1.5], [0.004, 0.04])) if self.b3 else 0.004
        gain = last_gain + max(-rate_dn, min(rate_up, delta))
        return round(gain / ACI_GAIN_QUANT) * ACI_GAIN_QUANT

    def step(self, op_curv, v_ms, actual, lat_active, pressed_torque, steering_pressed, blinker_on, cam_aci_gain):
        desired = float(np.clip(op_curv, -360.0, 360.0))
        if abs(v_ms) < SMOOTHING_ANGLE_MAX_VEGO:
            desired = _sp_smooth_angle(v_ms, desired, self.apply_angle_last)
        override_factor = self._override_factor(abs(pressed_torque), v_ms, blinker_on)
        if self.b1 and override_factor > PHASE5_BLEND_DEADBAND:
            blend = (override_factor - PHASE5_BLEND_DEADBAND) / (1.0 - PHASE5_BLEND_DEADBAND)
            desired = (1.0 - blend) * desired + blend * actual
        # VM rate-limit proxy (closed-form): treat |Δapply| > 5°/frame as reject
        delta_apply = abs(desired - self.apply_angle_last) if lat_active else 0.0
        vm_reject = lat_active and delta_apply > 5.0
        self.vm_reject_consecutive = self.vm_reject_consecutive + 1 if vm_reject else 0
        s1_tripped = self.s1 and self.vm_reject_consecutive >= PHASE5_VM_REJECT_FORCE_PASSIVE_FRAMES
        effective_lat = lat_active and not s1_tripped
        if lat_active and not s1_tripped:
            self.apply_angle_last = desired
        elif not lat_active:
            self.apply_angle_last = float(np.clip(actual, -360.0, 360.0))
        steering_error = self.apply_angle_last - actual
        gain = self._aci_gain(pressed_torque, v_ms * 3.6, effective_lat, self.aci_gain_last,
                              steering_error, blinker_on)
        self.aci_gain_last = gain
        return self.apply_angle_last, self.aci_gain_last, override_factor, s1_tripped, vm_reject


VARIANT_FLAGS = [
    ('N0', {}),                                                                     # pure reference
    ('N1', {'b1': True}),                                                           # +heavy-grip blend
    ('N2', {'b1': True, 'b2': True}),                                               # +blinker deadzone
    ('N3', {'b1': True, 'b2': True, 'a2': True}),                                   # +blinker ACI cap
    ('N4', {'b1': True, 'b2': True, 'a2': True, 'b3': True}),                       # +rate_up boost
    ('N5', {'b1': True, 'b2': True, 'a2': True, 'b3': True, 's1': True}),           # +S1 failsafe
]


def run_variants(by_seg):
    """Run all Phase 5 variants over the same drivelog. Returns dict
    variant -> list of per-frame state rows."""
    out = {name: [] for name, _ in VARIANT_FLAGS}
    for (route, seg), seg_frames in sorted(by_seg.items()):
        pipes = {name: NewPipelineV5(**flags) for name, flags in VARIANT_FLAGS}
        for fr in seg_frames:
            v = fr['v_ms']
            op_curv = float(fr['desired'])
            actual = float(fr['actual'])
            lat = bool(fr['lat_active'])
            pressed = bool(fr['steering_pressed'])
            tq = float(fr['pressed_torque'])
            blink = bool(fr.get('blinker', False))
            for name, pipe in pipes.items():
                ap, g, ovf, s1, vmr = pipe.step(op_curv, v, actual, lat, tq, pressed, blink, 0.0)
                out[name].append({
                    'route': route, 'seg': seg, 'v_kmh': fr['v_kmh'], 'lat_active': lat,
                    'pressed_torque': tq, 'blinker': blink, 'op_curv': op_curv, 'actual': actual,
                    'apply': ap, 'gain': g, 'override_factor': ovf,
                    's1_tripped': s1, 'vm_reject': vmr,
                })
    return out


def variant_summary(variants):
    """Print per-variant metrics for the Phase 5 pass-gate."""
    print()
    print('=' * 78)
    print('PHASE 5 — variant comparison (Phase 5 pass-gate)')
    print('=' * 78)
    print(f"{'variant':>8} {'p99_jit_all':>11} {'p99_jit_ss':>11} {'5deg/fr%':>9} "
          f"{'blink_aci':>10} {'lc_lt100_aci':>13} "
          f"{'s1_trip%':>9}")
    for name, rows in variants.items():
        # jitter p99 — all lat_active vs steady-state (override_factor < 0.3)
        deltas_all = []
        deltas_ss = []
        for i in range(1, len(rows)):
            a, b = rows[i-1], rows[i]
            if a['lat_active'] and b['lat_active'] and a['route'] == b['route'] and a['seg'] == b['seg']:
                d = abs(b['apply'] - a['apply'])
                deltas_all.append(d)
                if a['override_factor'] < PHASE5_BLEND_DEADBAND and b['override_factor'] < PHASE5_BLEND_DEADBAND:
                    deltas_ss.append(d)
        p99_all = float(np.percentile(deltas_all, 99)) if deltas_all else 0.0
        p99_ss = float(np.percentile(deltas_ss, 99)) if deltas_ss else 0.0
        over_5 = sum(1 for d in deltas_all if d > 5.0)
        n_lat = sum(1 for r in rows if r['lat_active'])
        sel_b = [r for r in rows if r['lat_active'] and r['blinker']]
        blink_aci = float(np.mean([r['gain'] for r in sel_b])) if sel_b else 0.0
        sel_lc = [r for r in rows if r['lat_active'] and r['blinker'] and abs(r['pressed_torque']) <= 100]
        lc_aci = float(np.mean([r['gain'] for r in sel_lc])) if sel_lc else 0.0
        s1_trip = sum(1 for r in rows if r['s1_tripped'])
        s1_pct = 100.0 * s1_trip / max(len(rows), 1)
        print(f"{name:>8} {p99_all:>11.3f} {p99_ss:>11.3f} {100.0*over_5/max(n_lat,1):>9.2f} "
              f"{blink_aci:>10.3f} {lc_aci:>13.3f} "
              f"{s1_pct:>9.3f}")
    print()
    print('Pass gates (Plan D):')
    print('  - N5 p99_jit_ss <= N0 p99_jit_ss * 1.10 (steady-state jitter preserved)')
    print('  - N5 p99_jit_all elevated under override is expected — B1 blend toward wheel.')
    print('    Panda safety: VM rate limiter (5°/frame) catches >5deg events in production.')
    print('  - N3 blink_aci <= 0.55 (A2 cap effective)')
    print('  - N3 vs N2 blink_aci diff >= 0.20 (A2 has measurable effect)')
    print('  - N5 s1_trip% <= 0.1 (normal driving rarely trips S1)')


def saturated_rate_metrics(rows):
    """Count frames where |Δapply| exceeds MAX_ANGLE_RATE=5°/frame.
    Real apply_steer_angle_limits_vm would clamp these. A higher count
    means the NEW pipeline depends more on the panda safety rate
    limiter; the closed-form sim has no rate limiter so this is the
    raw demand."""
    over_old = sum(1 for i in range(1, len(rows))
                    if rows[i]['lat_active'] and rows[i-1]['lat_active']
                    and rows[i]['route'] == rows[i-1]['route'] and rows[i]['seg'] == rows[i-1]['seg']
                    and abs(rows[i]['apply_old'] - rows[i-1]['apply_old']) > 5)
    over_new = sum(1 for i in range(1, len(rows))
                    if rows[i]['lat_active'] and rows[i-1]['lat_active']
                    and rows[i]['route'] == rows[i-1]['route'] and rows[i]['seg'] == rows[i-1]['seg']
                    and abs(rows[i]['apply_new'] - rows[i-1]['apply_new']) > 5)
    total_active = sum(1 for r in rows if r['lat_active'])
    return {'over_5deg_frame_OLD': over_old, 'over_5deg_frame_NEW': over_new,
            'total_lat_active': total_active}


def main():
    with open('/tmp/reanalysis_dbc_cache.pkl', 'rb') as f:
        frames, meta = pickle.load(f)
    print(f'Loaded {len(frames):,} frames; meta={meta}')

    # Group by route+seg, replay each segment with fresh state
    by_seg = defaultdict(list)
    for fr in frames:
        by_seg[(fr['route'], fr['seg'])].append(fr)
    print(f'  {len(by_seg)} segments')

    rows = []
    for (route, seg), seg_frames in sorted(by_seg.items()):
        old = OldPipeline(); new = NewPipeline()
        for fr in seg_frames:
            v = fr['v_ms']
            op_curv = fr['desired']
            actual = fr['actual']
            lat = bool(fr['lat_active'])
            pressed = bool(fr['steering_pressed'])
            tq = float(fr['pressed_torque'])
            blink = bool(fr.get('blinker', False))
            cam_gain = float(fr.get('cam_aci_gain', 0.0))
            ap_o, g_o, snap_o, _ = old.step(op_curv, v, actual, lat, tq, pressed, blink, cam_gain)
            ap_n, g_n, _, _      = new.step(op_curv, v, actual, lat, tq, pressed, blink, cam_gain)
            rows.append({
                'route': route, 'seg': seg, 'v_ms': v, 'v_kmh': fr['v_kmh'],
                'op_curv': op_curv, 'actual': actual, 'lat_active': lat,
                'pressed_torque': tq, 'steering_pressed': pressed, 'blinker': blink,
                'apply_old': ap_o, 'apply_new': ap_n,
                'gain_old': g_o,   'gain_new': g_n,
                'snap_old': snap_o,
            })

    print()
    print('=' * 78)
    print('METRIC #1 / #11+#17 — |apply - op_curv| by speed bucket (lat_active only)')
    print('=' * 78)
    sb = speed_bucket_metrics(rows, 'OLD', 'NEW')
    print(f"{'bucket':>10} {'n':>8} {'p50_OLD':>8} {'p50_NEW':>8} {'p90_OLD':>8} {'p90_NEW':>8} {'Δp50%':>8}")
    for r in sb:
        d = 100.0 * (r['p50_NEW'] - r['p50_OLD']) / max(r['p50_OLD'], 1e-6)
        print(f"{r['bucket']:>10} {r['n']:>8d} {r['p50_OLD']:>8.3f} {r['p50_NEW']:>8.3f} "
              f"{r['p90_OLD']:>8.3f} {r['p90_NEW']:>8.3f} {d:>+8.1f}")

    print()
    print('=' * 78)
    print('METRIC #2 / #15 — ACIGain mean (lat_active)')
    print('=' * 78)
    am = aci_gain_metrics(rows)
    for k, v in am.items():
        print(f"  {k} (n={v['n']:,}):  OLD={v['old_mean']:.3f}  NEW={v['new_mean']:.3f}  "
              f"Δ={v['new_mean']-v['old_mean']:+.3f}")

    print()
    print('=' * 78)
    print('METRIC #3 / #12+#14+#16 — apply-vs-wheel deviation in 1s after OLD snap-exit')
    print('=' * 78)
    hr = heavy_release_metrics(rows)
    print(f"  n snap-exits OLD: {hr['n_snap_exits']}")
    if hr['n_snap_exits']:
        print(f"  max |apply-wheel| over 1s window after snap-exit (closed-form sim;")
        print(f"  no plant model so actual==apply by construction):")
        print(f"    OLD: p50={hr['old_max_dev_p50']:.1f}°  p90={hr['old_max_dev_p90']:.1f}°")
        print(f"    NEW: p50={hr['new_max_dev_p50']:.1f}°  p90={hr['new_max_dev_p90']:.1f}°")
        print(f"  (Real plant-effects: see /tmp/p19_baseline/patch12.txt — drive 15")
        print(f"   -223° corner showed OLD recovery hold improved max op_dev by 97%.)")

    print()
    print('=' * 78)
    print('METRIC #4 — per-frame |Δapply|  (proxy for wheel jitter EPS would see)')
    print('=' * 78)
    jm = jitter_metrics(rows)
    print(f"{'bucket':>10} {'n':>8} {'p50_OLD':>8} {'p50_NEW':>8} {'p90_OLD':>8} {'p90_NEW':>8} {'p99_OLD':>8} {'p99_NEW':>8}")
    for r in jm:
        print(f"{r['bucket']:>10} {r['n']:>8d} {r['p50_old']:>8.3f} {r['p50_new']:>8.3f} "
              f"{r['p90_old']:>8.3f} {r['p90_new']:>8.3f} {r['p99_old']:>8.3f} {r['p99_new']:>8.3f}")

    sr = saturated_rate_metrics(rows)
    print()
    print(f"Frames with |Δapply| > 5°/frame (would hit MAX_ANGLE_RATE):")
    print(f"  OLD: {sr['over_5deg_frame_OLD']:,} / {sr['total_lat_active']:,} ({100.0*sr['over_5deg_frame_OLD']/max(sr['total_lat_active'],1):.2f}%)")
    print(f"  NEW: {sr['over_5deg_frame_NEW']:,} / {sr['total_lat_active']:,} ({100.0*sr['over_5deg_frame_NEW']/max(sr['total_lat_active'],1):.2f}%)")

    print()
    print('=' * 78)
    print('SUMMARY — Which PR effect is preserved vs lost')
    print('=' * 78)
    print(f"""
PR     Feature                                Phase 1+2 fate        Notes
------ -------------------------------------- --------------------- ------------------------------------
 #11   low-speed override-aware hands_off     KEPT (carcontroller   override_factor still used as
                                              :340)                 hands_off gate (override>0.5)
 #11   vtau S1+S2 (low-speed gate / entry_th) REPLACED              sp_smooth_angle EMA absorbs jitter
                                                                    via alpha 0.05 below 8.5 m/s
 #12   post_override_recovery hold            REMOVED               no snap state → no exit event
 #13   light-grip 50°+ hands-off snap         REMOVED               no snap state
 #14   moderate-grip 50°+ snap entry          REMOVED               no snap state
 #15   ACIGain rate_up boost + dynamic ceil.  REPLACED              reference 17-line uses fixed
                                                                    rate_up=0.004 / dyn ceiling kept
 #16-A heavy_override mismatch guard          REMOVED               no snap state
 #16-B rate_up smoothing (eliminate flap)     REMOVED               reference rate_up is constant
 #16-D recovery early-exit on release         REMOVED               no recovery state
 #17-A vtau speed_max_tau city extension      REPLACED              alpha=0.1-0.3 at city = similar
                                                                    smoothing tau (~70-300 ms)
 #17-B moderate_entry blinker guard           REMOVED               no moderate_entry path
""")

    # Phase 5 variant comparison — run the 6 augmented variants on the
    # same drivelog and emit Plan D pass-gate metrics.
    variants = run_variants(by_seg)
    variant_summary(variants)


if __name__ == '__main__':
    main()
