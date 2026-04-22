#!/usr/bin/env python3
"""Full stress test: ACC/MADS state transitions + aggressive steering scenarios.
Tests rapid left-right wheel oscillation, full-lock parking, and transition combos."""
import numpy as np
np.random.seed(42)

class CarControllerSim:
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
    MAX_ANGLE = 85

    def __init__(self):
        self.aci_gain_last = 0.0
        self.aci_gain_ramp = 0.0
        self.override_snapped = False
        self.override_enter_cnt = 0
        self.override_exit_cnt = 0
        self.apply_angle_last = 0.0
        self.frame = 0
        self.prev_lat_active = False

    def compute_dtf(self, tq, v):
        bp1 = float(np.interp(v, [3., 8., 15., 22.], [200., 220., 175., 80.]))
        bp2 = float(np.interp(v, [3., 8., 15., 22.], [300., 280., 260., 200.]))
        bp3 = float(np.interp(v, [3., 8., 15., 22.], [380., 330., 290., 290.]))
        bp4 = float(np.interp(v, [3., 8., 15., 22.], [500., 470., 420., 400.]))
        shelf = float(np.interp(v, [3., 22.], [0.65, 0.80]))
        floor = float(np.interp(v, [3., 22.], [0.15, 0.35]))
        return float(np.interp(abs(tq), [bp1, bp2, bp3, bp4], [1.0, shelf, shelf, floor]))

    def step(self, lat_active, mads_enabled, cruise_available, v_ego, steer_angle, steer_torque, cam_gain=0.0):
        self.frame += 1
        speed_blend = float(np.clip((v_ego - 1.0/3.6) / (3.0/3.6 - 1.0/3.6), 0.0, 1.0))
        full_ov = float(np.interp(v_ego, [8.0, 15.0], [self.FULL_LOW, self.FULL_HIGH]))
        ov_factor = float(np.clip((abs(steer_torque) - self.DZ_ANGLE) / max(full_ov - self.DZ_ANGLE, 1.0), 0.0, 1.0))

        if ov_factor >= self.OVERRIDE_SNAP_ENTER_FACTOR:
            self.override_enter_cnt += 1; self.override_exit_cnt = 0
        elif ov_factor <= self.OVERRIDE_SNAP_EXIT_FACTOR:
            self.override_exit_cnt += 1; self.override_enter_cnt = 0
        if not self.override_snapped and self.override_enter_cnt >= self.OVERRIDE_SNAP_ENTER_FRAMES:
            self.override_snapped = True
        elif self.override_snapped and self.override_exit_cnt >= self.OVERRIDE_SNAP_EXIT_FRAMES:
            self.override_snapped = False

        disengage_edge = self.prev_lat_active and not lat_active
        if not lat_active:
            self.override_snapped = False
            self.override_enter_cnt = 0; self.override_exit_cnt = 0
            self.aci_gain_last = 0.0

        apply_steer_req = abs(steer_angle) < self.MAX_ANGLE
        if self.aci_gain_ramp < 1.0 and lat_active:
            self.aci_gain_ramp = min(1.0, self.aci_gain_ramp + 1.0/30.0)
        elif not lat_active:
            self.aci_gain_ramp = 0.0

        effective_lat = lat_active and not self.override_snapped and apply_steer_req
        if effective_lat:
            raw = max(speed_blend, 0.20) * self.aci_gain_ramp * self.compute_dtf(steer_torque, v_ego)
            target = float(np.clip(raw, 0.10, self.ACI_GAIN_CEILING))
        else:
            target = cam_gain
        eff = max(self.aci_gain_last + self.ACI_GAIN_RATE_DOWN, min(target, self.aci_gain_last + self.ACI_GAIN_RATE_UP))
        q = self.ACI_GAIN_QUANT
        eff = round(eff / q) * q
        self.aci_gain_last = eff
        if not lat_active: self.apply_angle_last = steer_angle
        icon = (2 if mads_enabled else 0) if cruise_available else None
        self.prev_lat_active = lat_active
        return {'gain': eff, 'eff_lat': effective_lat, 'snap': self.override_snapped,
                'icon': icon, 'hod': mads_enabled, 'steer_req': apply_steer_req,
                'lat': lat_active, 'mads': mads_enabled, 'disengage': disengage_edge,
                'angle': steer_angle, 'torque': steer_torque, 'v': v_ego}

def check(out, prev):
    v = []
    if out['gain'] < -0.001 or out['gain'] > 0.504:
        v.append(f'I1: gain={out["gain"]:.4f}')
    if out['snap'] and out['eff_lat']:
        v.append('I3: snap+active')
    if not out['steer_req'] and out['eff_lat']:
        v.append('I4: >85+active')
    if prev and not out['disengage']:
        d = out['gain'] - prev['gain']
        if d > 0.012: v.append(f'I5up: {d:+.4f}')
        if d < -0.032: v.append(f'I5dn: {d:+.4f}')
    if out['mads'] and out['icon'] is not None and out['icon'] != 2:
        v.append(f'I6: icon={out["icon"]}')
    if not out['mads'] and out['icon'] is not None and out['icon'] != 0:
        v.append(f'I6b: icon={out["icon"]}')
    if out['hod'] != out['mads']:
        v.append('I7: hod')
    return v

# ========== Scenario Generator ==========
def gen_scenario(frame, scenario_type, phase):
    """Generate driving conditions for specific scenarios."""
    if scenario_type == 'rapid_lr':
        # Rapid left-right wheel oscillation (slalom / lane change)
        freq = np.random.uniform(1, 5)  # 1-5 Hz
        amp_angle = np.random.uniform(30, 150)
        amp_torque = np.random.uniform(200, 600)
        v = np.random.uniform(5, 25)
        angle = amp_angle * np.sin(2 * np.pi * freq * frame / 50.0)
        torque = amp_torque * np.sin(2 * np.pi * freq * frame / 50.0)
        return v, angle, torque
    elif scenario_type == 'full_lock_right':
        # Parking: gradual turn to full lock (0→450° over 2s)
        v = np.random.uniform(0, 3)
        progress = min(phase / 100.0, 1.0)
        angle = progress * 180  # up to 180° (beyond sensor range, but testing)
        torque = 300 + progress * 500
        return v, angle, torque
    elif scenario_type == 'full_lock_left':
        v = np.random.uniform(0, 3)
        progress = min(phase / 100.0, 1.0)
        angle = -progress * 180
        torque = 300 + progress * 500
        return v, angle, torque
    elif scenario_type == 'whip':
        # Fast whip: center → hard right → center → hard left
        cycle = phase % 100
        if cycle < 25:
            angle = (cycle / 25.0) * 120
            torque = (cycle / 25.0) * 500
        elif cycle < 50:
            angle = ((50 - cycle) / 25.0) * 120
            torque = ((50 - cycle) / 25.0) * 500
        elif cycle < 75:
            angle = -((cycle - 50) / 25.0) * 120
            torque = ((cycle - 50) / 25.0) * 500
        else:
            angle = -(( 100 - cycle) / 25.0) * 120
            torque = ((100 - cycle) / 25.0) * 500
        v = np.random.uniform(8, 20)
        return v, angle, torque
    else:  # normal
        v = np.random.uniform(0, 30)
        tq = np.random.choice([np.random.uniform(0,50), np.random.uniform(100,300), np.random.uniform(400,800)], p=[0.7,0.2,0.1])
        ang = np.random.choice([np.random.uniform(-40,40), np.random.uniform(-120,-80), np.random.uniform(80,180)], p=[0.8,0.1,0.1])
        return v, ang, tq

# ========== Run ==========
N = 200000
sim = CarControllerSim()
acc, cruise, mads, lat = False, False, False, False
violations = []
prev = None
scenario = 'normal'
scenario_phase = 0
scenario_duration = 0
stats = {'transitions': 0, 'rapid_lr': 0, 'full_lock_r': 0, 'full_lock_l': 0, 'whip': 0,
         'normal': 0, 'override_snap_count': 0, 'big_angle_frames': 0, 'passive_frames': 0}

for i in range(N):
    # State transitions
    r = np.random.random()
    if r < 0.015:
        acc = not acc
        if not acc: cruise = mads = lat = False
        stats['transitions'] += 1
    elif r < 0.035 and acc:
        cruise = not cruise
        if cruise: mads = lat = True
        stats['transitions'] += 1
    elif r < 0.04 and acc:
        mads = not mads; lat = mads
        stats['transitions'] += 1

    # Scenario switching
    scenario_phase += 1
    if scenario_phase >= scenario_duration:
        scenario = np.random.choice(['normal', 'rapid_lr', 'full_lock_right', 'full_lock_left', 'whip'],
                                     p=[0.5, 0.15, 0.1, 0.1, 0.15])
        scenario_duration = np.random.randint(50, 500)
        scenario_phase = 0
        stats[scenario.replace('full_lock_right', 'full_lock_r').replace('full_lock_left', 'full_lock_l')] = \
            stats.get(scenario.replace('full_lock_right', 'full_lock_r').replace('full_lock_left', 'full_lock_l'), 0) + 1

    v, ang, tq = gen_scenario(i, scenario, scenario_phase)

    out = sim.step(lat, mads, acc, v, ang, tq)
    errs = check(out, prev)
    for e in errs: violations.append(f'f{i}({scenario}): {e}')
    if out['snap']: stats['override_snap_count'] += 1
    if abs(ang) >= 85: stats['big_angle_frames'] += 1
    if out['lat'] and not out['eff_lat']: stats['passive_frames'] += 1
    prev = out

print(f"=== Full Stress Test: {N} frames ===")
print(f"Stats: {stats}")
print(f"Violations: {len(violations)}")
if violations:
    print("\nFirst 20 violations:")
    for v in violations[:20]:
        print(f"  {v}")
else:
    print("\nALL INVARIANTS PASSED")
    print(f"\nKey metrics:")
    print(f"  Override snap activations: {stats['override_snap_count']} frames")
    print(f"  |angle|>=85° frames: {stats['big_angle_frames']}")
    print(f"  Passive frames (lat=True but eff=False): {stats['passive_frames']}")
    print(f"  State transitions: {stats['transitions']}")
