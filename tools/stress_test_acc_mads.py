#!/usr/bin/env python3
"""ACC/MADS ON/OFF random stress test — simulates rapid state transitions
and checks for invariant violations in the CCNC angle-control pipeline."""
import numpy as np
import sys

np.random.seed(42)

# ---- Replicate key carcontroller state machine ----
class CarControllerSim:
    """Minimal simulation of carcontroller.py CCNC angle-control state."""
    # Constants from values.py
    ACI_GAIN_CEILING = 0.5
    ACI_GAIN_RATE_DOWN = -0.028
    ACI_GAIN_RATE_UP = 0.008
    ACI_GAIN_QUANT = 0.004
    OVERRIDE_SNAP_ENTER_FACTOR = 0.95
    OVERRIDE_SNAP_ENTER_FRAMES = 3
    OVERRIDE_SNAP_EXIT_FACTOR = 0.05
    OVERRIDE_SNAP_EXIT_FRAMES = 25
    DZ_ANGLE = 100.0
    FULL_LOW = 300.0
    FULL_HIGH = 500.0

    def __init__(self):
        self.aci_gain_last = 0.0
        self.aci_gain_ramp = 0.0
        self.aci_active_latched = False
        self.override_snapped = False
        self.override_enter_cnt = 0
        self.override_exit_cnt = 0
        self.apply_angle_last = 0.0
        self.low_speed_cam_latched = False
        self.passthrough_latched = False
        self.frame = 0

    def compute_dtf(self, tq, v):
        bp1 = float(np.interp(v, [3., 8., 15., 22.], [200., 220., 175., 80.]))
        bp2 = float(np.interp(v, [3., 8., 15., 22.], [300., 280., 260., 200.]))
        bp3 = float(np.interp(v, [3., 8., 15., 22.], [380., 330., 290., 290.]))
        bp4 = float(np.interp(v, [3., 8., 15., 22.], [500., 470., 420., 400.]))
        shelf = float(np.interp(v, [3., 22.], [0.65, 0.80]))
        floor = float(np.interp(v, [3., 22.], [0.15, 0.35]))
        return float(np.interp(abs(tq), [bp1, bp2, bp3, bp4], [1.0, shelf, shelf, floor]))

    def step(self, lat_active, mads_enabled, cruise_available, cruise_enabled,
             v_ego, steer_angle, steer_torque, cam_aci_gain=0.0):
        """Run one 50Hz frame. Returns dict of outputs for invariant checking."""
        self.frame += 1

        # Speed blend
        ACI_SPEED_ZERO = 1.0 / 3.6
        ACI_SPEED_FULL = 3.0 / 3.6
        speed_blend = float(np.clip((v_ego - ACI_SPEED_ZERO) / (ACI_SPEED_FULL - ACI_SPEED_ZERO), 0.0, 1.0))

        # Override factor (simplified — uses angle-control thresholds)
        full_override = float(np.interp(v_ego, [8.0, 15.0], [self.FULL_LOW, self.FULL_HIGH]))
        override_factor = float(np.clip((abs(steer_torque) - self.DZ_ANGLE) /
                                         max(full_override - self.DZ_ANGLE, 1.0), 0.0, 1.0))

        # Override snap state machine
        if override_factor >= self.OVERRIDE_SNAP_ENTER_FACTOR:
            self.override_enter_cnt += 1
            self.override_exit_cnt = 0
        elif override_factor <= self.OVERRIDE_SNAP_EXIT_FACTOR:
            self.override_exit_cnt += 1
            self.override_enter_cnt = 0

        if not self.override_snapped and self.override_enter_cnt >= self.OVERRIDE_SNAP_ENTER_FRAMES:
            self.override_snapped = True
        elif self.override_snapped and self.override_exit_cnt >= self.OVERRIDE_SNAP_EXIT_FRAMES:
            self.override_snapped = False

        if not lat_active:
            self.override_snapped = False
            self.override_enter_cnt = 0
            self.override_exit_cnt = 0
            self.aci_gain_last = 0.0

        # apply_steer_req (>85° fault avoidance simplified)
        apply_steer_req = abs(steer_angle) < 85

        # ACI engagement
        self.aci_active_latched = bool(lat_active)

        # ACI gain ramp
        if self.aci_active_latched:
            self.aci_gain_ramp = min(1.0, self.aci_gain_ramp + 1.0 / 30.0)
        else:
            self.aci_gain_ramp = 0.0

        # Effective lat active (Fix 3)
        effective_lat_active = lat_active and not self.override_snapped and apply_steer_req
        steering_active = effective_lat_active

        # ACIGain computation
        dtf = self.compute_dtf(steer_torque, v_ego) if lat_active else 0.0
        if steering_active:
            raw_gain = max(speed_blend, 0.20) * self.aci_gain_ramp * dtf * 1.0  # lon_comfort=1
            target = float(np.clip(raw_gain, 0.10, self.ACI_GAIN_CEILING))
        else:
            target = cam_aci_gain

        # Rate limit
        effective = max(self.aci_gain_last + self.ACI_GAIN_RATE_DOWN,
                       min(target, self.aci_gain_last + self.ACI_GAIN_RATE_UP))
        # Quantize
        q = self.ACI_GAIN_QUANT
        effective = round(effective / q) * q
        self.aci_gain_last = effective

        # LKA icon (Fix 4)
        if cruise_available:
            lka_icon = 2 if mads_enabled else 0
        else:
            lka_icon = None  # camera passthrough

        # HOD bypass (Fix 2)
        hod_bypass = mads_enabled

        # Angle tracking
        if not lat_active:
            self.apply_angle_last = steer_angle

        return {
            'frame': self.frame,
            'effective_lat_active': effective_lat_active,
            'steering_active': steering_active,
            'aci_gain': effective,
            'override_snapped': self.override_snapped,
            'lka_icon': lka_icon,
            'hod_bypass': hod_bypass,
            'apply_steer_req': apply_steer_req,
            'lat_active': lat_active,
            'mads_enabled': mads_enabled,
        }


def check_invariants(out, prev_out):
    """Check safety and consistency invariants. Returns list of violations."""
    violations = []

    # I1: ACIGain must be in [0, ceiling]
    if out['aci_gain'] < -0.001 or out['aci_gain'] > 0.504:
        violations.append(f"I1: aci_gain={out['aci_gain']:.4f} out of [0, 0.5]")

    # I2: When not lat_active, gain should decay toward 0
    if not out['lat_active'] and out['aci_gain'] > (prev_out['aci_gain'] + 0.01 if prev_out else 0.01):
        violations.append(f"I2: gain increasing while not latActive: {out['aci_gain']:.4f}")

    # I3: When override_snapped, effective_lat_active must be False
    if out['override_snapped'] and out['effective_lat_active']:
        violations.append(f"I3: override_snapped but effective_lat_active=True")

    # I4: When angle > 85°, effective_lat_active must be False
    if not out['apply_steer_req'] and out['effective_lat_active']:
        violations.append(f"I4: angle>85° but effective_lat_active=True")

    # I5: Frame-over-frame gain delta must respect rate limits
    if prev_out:
        delta = out['aci_gain'] - prev_out['aci_gain']
        if delta > 0.012:  # 0.008 + quantization tolerance
            violations.append(f"I5: gain delta={delta:+.4f} exceeds rate_up")
        if delta < -0.032:  # -0.028 - quantization tolerance
            violations.append(f"I5: gain delta={delta:+.4f} exceeds rate_down")

    # I6: LKA icon consistency
    if out['mads_enabled'] and out['lka_icon'] is not None and out['lka_icon'] != 2:
        violations.append(f"I6: mads_enabled but lka_icon={out['lka_icon']} (expected 2)")
    if not out['mads_enabled'] and out['lka_icon'] is not None and out['lka_icon'] != 0:
        violations.append(f"I6: mads_disabled but lka_icon={out['lka_icon']} (expected 0)")

    # I7: HOD bypass must match mads_enabled
    if out['hod_bypass'] != out['mads_enabled']:
        violations.append(f"I7: hod_bypass={out['hod_bypass']} != mads={out['mads_enabled']}")

    return violations


# ---- Run stress test ----
N_FRAMES = 50000
sim = CarControllerSim()

# Random state generation
acc_main_on = False
cruise_enabled = False
mads_enabled = False
lat_active = False

all_violations = []
prev_out = None
state_changes = 0

# State counters
stats = {'acc_on': 0, 'acc_off': 0, 'cruise_on': 0, 'cruise_off': 0,
         'mads_on': 0, 'mads_off': 0, 'override_events': 0, 'big_angle_events': 0}

for i in range(N_FRAMES):
    # Random events (each frame ~5% chance of state change)
    r = np.random.random()
    if r < 0.02:  # ACC main toggle
        acc_main_on = not acc_main_on
        if not acc_main_on:
            cruise_enabled = False
            mads_enabled = False
            lat_active = False
            stats['acc_off'] += 1
        else:
            stats['acc_on'] += 1
    elif r < 0.04:  # Cruise toggle
        if acc_main_on:
            cruise_enabled = not cruise_enabled
            if cruise_enabled:
                mads_enabled = True  # ACC ON → MADS ON
                lat_active = True
                stats['cruise_on'] += 1
            else:
                # Cancel only disables cruise, not MADS (per user clarification)
                stats['cruise_off'] += 1
    elif r < 0.05:  # MADS toggle (LFA button)
        if acc_main_on:
            mads_enabled = not mads_enabled
            lat_active = mads_enabled
            if mads_enabled:
                stats['mads_on'] += 1
            else:
                stats['mads_off'] += 1

    # Random driving conditions
    v_ego = np.random.uniform(0, 30)
    steer_torque = np.random.choice([
        np.random.uniform(0, 50),      # light grip (70%)
        np.random.uniform(100, 300),    # moderate (20%)
        np.random.uniform(400, 800),    # heavy override (10%)
    ], p=[0.7, 0.2, 0.1])
    steer_angle = np.random.choice([
        np.random.uniform(-40, 40),     # normal (80%)
        np.random.uniform(-120, -80),   # large angle (10%)
        np.random.uniform(80, 180),     # parking lock (10%)
    ], p=[0.8, 0.1, 0.1])

    if np.random.random() < 0.05:
        stats['override_events'] += 1
        steer_torque = np.random.uniform(500, 900)
    if np.random.random() < 0.03:
        stats['big_angle_events'] += 1
        steer_angle = np.random.uniform(100, 180)

    out = sim.step(lat_active, mads_enabled, acc_main_on, cruise_enabled,
                   v_ego, steer_angle, steer_torque)

    violations = check_invariants(out, prev_out)
    if violations:
        for v in violations:
            all_violations.append(f"frame {i}: {v}")
    prev_out = out

print(f"=== ACC/MADS Stress Test: {N_FRAMES} frames ===")
print(f"State changes: {stats}")
print(f"Total violations: {len(all_violations)}")
if all_violations:
    print("\nFirst 20 violations:")
    for v in all_violations[:20]:
        print(f"  {v}")
else:
    print("ALL INVARIANTS PASSED — no violations detected")
