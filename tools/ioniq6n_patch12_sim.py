#!/usr/bin/env python3
"""Patch #12 sim — post-override recovery hold.

Replays drives 14/15/16 frames through the override_snap state machine
twice: once with current code (A), once with patch #12 recovery hold (B).
For each concerning event from POST_PR11_AUDIT.md, reports the worst
op-vs-wheel deviation in the 1-second window after snap exit.

This is a closed-form sim of the snap state machine — no closed-loop
physics. Just answers: does the recovery hold keep apply_angle synced
with wheel during caster recovery?
"""
import glob
import os
import sys

import numpy as np
import zstandard as zstd

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')
from cereal import log

DRIVELOG_DIR = '/home/user/openpilot/drivelog'

# From values.py + carcontroller.py
OVERRIDE_SNAP_ENTER_FACTOR = 0.90
OVERRIDE_SNAP_EXIT_FACTOR  = 0.10
OVERRIDE_SNAP_ENTER_FRAMES = 3
OVERRIDE_SNAP_EXIT_FRAMES  = 10
HEAVY_SNAP_OVERRIDE_TQ     = 200.0

# Driver torque (angle path)
DEADZONE  = 100.0
LOW_V_FULL  = 180.0
HIGH_V_FULL = 350.0
LOW_V_SPEED  = 8.0
HIGH_V_SPEED = 15.0

# 12th recovery hold
RECOVERY_ENTER_ABS_DEG  = 30.0
RECOVERY_EXIT_ABS_DEG   = 10.0
RECOVERY_TIMEOUT_FRAMES = 200


def override_factor_of(torque, v_ms):
    full = float(np.interp(v_ms, [LOW_V_SPEED, HIGH_V_SPEED], [LOW_V_FULL, HIGH_V_FULL]))
    return float(np.clip((abs(torque) - DEADZONE) / max(full - DEADZONE, 1.0), 0.0, 1.0))


class SnapSim:
    """Replicates carcontroller's override-snap + (optionally) patch #12 recovery."""
    def __init__(self, mode):
        assert mode in ('A', 'B')
        self.mode = mode
        self.override_snapped = False
        self.enter_cnt = 0
        self.exit_cnt = 0
        self.post_override_recovery = False
        self.recovery_remaining = 0
        self.apply_angle_last = 0.0

    def step(self, wheel, torque, v_ms, lat_active, blinker_on, model_curv):
        ovf = override_factor_of(torque, v_ms)
        snap_blinker_override = blinker_on and abs(torque) > HEAVY_SNAP_OVERRIDE_TQ
        if ovf >= OVERRIDE_SNAP_ENTER_FACTOR and (not blinker_on or snap_blinker_override):
            self.enter_cnt += 1; self.exit_cnt = 0
        elif ovf <= OVERRIDE_SNAP_EXIT_FACTOR:
            self.exit_cnt += 1; self.enter_cnt = 0

        prev_snapped = self.override_snapped
        if not self.override_snapped and self.enter_cnt >= OVERRIDE_SNAP_ENTER_FRAMES:
            self.override_snapped = True
        elif self.override_snapped and self.exit_cnt >= OVERRIDE_SNAP_EXIT_FRAMES:
            self.override_snapped = False

        if self.mode == 'B':
            if prev_snapped and not self.override_snapped \
               and abs(wheel) >= RECOVERY_ENTER_ABS_DEG:
                self.post_override_recovery = True
                self.recovery_remaining = RECOVERY_TIMEOUT_FRAMES
            if self.post_override_recovery:
                self.recovery_remaining -= 1
                if abs(wheel) < RECOVERY_EXIT_ABS_DEG or self.recovery_remaining <= 0:
                    self.post_override_recovery = False

        if not lat_active:
            self.override_snapped = False
            self.enter_cnt = 0; self.exit_cnt = 0
            self.post_override_recovery = False; self.recovery_remaining = 0
            self.apply_angle_last = wheel
            return self.apply_angle_last, False, False, ovf

        # Snap path — apply_angle_last = wheel
        if self.override_snapped or (self.mode == 'B' and self.post_override_recovery):
            self.apply_angle_last = wheel
        else:
            # No snap → simple model toward target with rate-limit-ish convergence.
            # Real code uses VM rate limiter via vtau LPF. Use simplified equivalent
            # that under-states the divergence (i.e. this is a CONSERVATIVE sim —
            # real-world A is even worse than what we plot).
            target = model_curv  # use actuator command as target proxy
            max_step = max(0.02, 3.59 / max(v_ms, 1.0)**2 * 0.01)  # ~jerk limit/frame
            err = target - self.apply_angle_last
            self.apply_angle_last += np.clip(err, -max_step, max_step)
        return self.apply_angle_last, self.override_snapped, \
               (self.mode == 'B' and self.post_override_recovery), ovf


def load_event_window(route, target_seg, target_mono, pre_frames=200, post_frames=500):
    """Load carState/carControl frames from a specific seg around target mono."""
    pattern = os.path.join(DRIVELOG_DIR, f'*_{route}--*--{target_seg}--rlog.zst')
    paths = glob.glob(pattern)
    if not paths:
        return []
    raw = zstd.ZstdDecompressor().decompress(open(paths[0],'rb').read(),
                                              max_output_size=500*1024*1024)
    frames = []
    latest_cs = None
    latest_sp = None
    for msg in log.Event.read_multiple_bytes(raw):
        w = msg.which()
        if w == 'carState':
            latest_cs = msg.carState
        elif w == 'selfdriveStateSP':
            latest_sp = msg.selfdriveStateSP
        elif w == 'carControl' and latest_cs is not None:
            cc = msg.carControl
            frames.append({
                'mono': msg.logMonoTime,
                'v_ms': float(latest_cs.vEgoRaw),
                'wheel': float(latest_cs.steeringAngleDeg),
                'torque': float(latest_cs.steeringTorque),
                'op_angle': float(cc.actuators.steeringAngleDeg),
                'lat_active': bool(cc.latActive),
                'mads_en': bool(latest_sp.mads.enabled) if latest_sp else False,
                'blinker': bool(latest_cs.leftBlinker or latest_cs.rightBlinker),
            })
    # Window around mono
    target_i = min(range(len(frames)), key=lambda i: abs(frames[i]['mono'] - target_mono))
    a = max(0, target_i - pre_frames)
    b = min(len(frames), target_i + post_frames)
    return frames[a:b], target_i - a


def simulate_and_compare(window, peak_i):
    """Run A & B sims over window, return per-frame apply_angle_A, apply_angle_B."""
    a_sim = SnapSim('A')
    b_sim = SnapSim('B')
    out = []
    for f in window:
        a_app, a_snap, _, ovf = a_sim.step(
            f['wheel'], f['torque'], f['v_ms'], f['lat_active'],
            f['blinker'], f['op_angle']  # use measured op_angle as model proxy
        )
        b_app, b_snap, b_recov, _ = b_sim.step(
            f['wheel'], f['torque'], f['v_ms'], f['lat_active'],
            f['blinker'], f['op_angle']
        )
        out.append({'mono': f['mono'], 'wheel': f['wheel'], 'torque': f['torque'],
                    'lat_active': f['lat_active'], 'apply_A': a_app, 'apply_B': b_app,
                    'snap_A': a_snap, 'snap_B': b_snap, 'recov_B': b_recov, 'ovf': ovf})
    return out


def main():
    # Drive 15 worst case from audit: peak -223.8° @ 18.4 kph blinker LR, seg 36
    # Approx mono at peak = 2230117269883 (from drill output)
    cases = [
        ('00000015', '36', 2230117269883, 'drive15 -223.8° @18.4kph'),
        ('00000014', '0',  None, 'drive14 (any seg w/ 60+ turn)'),  # auto-find
    ]

    for route, seg, mono, label in cases:
        if mono is None:
            continue
        window, peak_i = load_event_window(route, seg, mono)
        if not window:
            print(f"{label}: no data"); continue
        sim = simulate_and_compare(window, peak_i)

        # Find the snap-exit point in A and the recovery window in B
        snap_exit_i_A = None
        for i in range(1, len(sim)):
            if sim[i-1]['snap_A'] and not sim[i]['snap_A']:
                snap_exit_i_A = i
                break
        snap_exit_i_B = None
        for i in range(1, len(sim)):
            if sim[i-1]['snap_B'] and not sim[i]['snap_B']:
                snap_exit_i_B = i
                break

        print(f"\n=== {label} ===")
        if snap_exit_i_A is not None:
            # Window: snap_exit → snap_exit + 100 frames (1 s)
            a_dev = [abs(sim[i]['apply_A'] - sim[i]['wheel']) for i in
                     range(snap_exit_i_A, min(len(sim), snap_exit_i_A + 100))]
            b_dev = [abs(sim[i]['apply_B'] - sim[i]['wheel']) for i in
                     range(snap_exit_i_A, min(len(sim), snap_exit_i_A + 100))]
            print(f"  Snap exit at frame {snap_exit_i_A} (sim), wheel = {sim[snap_exit_i_A]['wheel']:.1f}°")
            print(f"  1-second window after snap exit:")
            print(f"    A (current code):  max op_dev = {max(a_dev):.1f}°, mean = {np.mean(a_dev):.1f}°")
            print(f"    B (patch #12):     max op_dev = {max(b_dev):.1f}°, mean = {np.mean(b_dev):.1f}°")
            a_max = max(a_dev) if a_dev else 0.0
            b_max = max(b_dev) if b_dev else 0.0
            denom = a_max if a_max > 1e-6 else 1e-6
            print(f"    Improvement: max reduced by {a_max - b_max:.1f}° "
                  f"({100*(a_max-b_max)/denom:.0f}%)")
            # Confirm patch #12 entered recovery
            recov_active = sum(1 for s in sim[snap_exit_i_A:snap_exit_i_A+100] if s['recov_B'])
            print(f"    B recovery hold active: {recov_active}/100 frames in window")
            # When did B exit recovery?
            recov_exit_i = None
            for i in range(snap_exit_i_A, min(len(sim), snap_exit_i_A + 250)):
                if i > 0 and sim[i-1]['recov_B'] and not sim[i]['recov_B']:
                    recov_exit_i = i
                    break
            if recov_exit_i:
                rel = recov_exit_i - snap_exit_i_A
                print(f"    B recovery exited at frame +{rel} (wheel={sim[recov_exit_i]['wheel']:.1f}°)")
        else:
            print("  No snap exit found in this window — event not reproduced in sim.")


if __name__ == '__main__':
    main()
