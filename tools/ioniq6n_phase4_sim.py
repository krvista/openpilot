#!/usr/bin/env python3
"""Phase 4 steering simulation: VM-based jerk/accel limiter + low-speed LPF.

Replays 1.24M drivelog frames through old (v1 rate table + camera blend) and
new (VM jerk/accel + LPF, op-only) pipelines. Computes steering metrics per
speed bucket and sweeps parameter space to find optimal initial values.

Depends on the DBC-accurate cache from tools/ioniq6n_reanalysis_dbc.py.
Usage: python tools/ioniq6n_phase4_sim.py
"""
import math
import pickle
import sys
import numpy as np

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')

from opendbc.car import DT_CTRL, structs
from opendbc.car.lateral import AngleSteeringLimits, apply_std_steer_angle_limits
from opendbc.car.vehicle_model import VehicleModel

CACHE_PATH = '/tmp/reanalysis_dbc_cache.pkl'

BUCKETS = [
    ('low_speed', 0, 15),
    ('city', 15, 60),
    ('highway', 60, 200),
]

# Ioniq 6 N vehicle params
MASS = 2175 + 136  # curb + cargo
WHEELBASE = 2.965
STEER_RATIO = 14.26
CENTER_TO_FRONT = WHEELBASE * 0.4
TIRE_STIFFNESS_FACTOR = 1.1

# Old v1 rate limits (current production)
V1_LIMITS = AngleSteeringLimits(
    176.7,
    ([0., 3., 7., 12., 18., 25., 30.], [0.6, 0.9, 1.3, 1.0, 0.6, 0.4, 0.25]),
    ([0., 3., 7., 12., 18., 25., 30.], [0.8, 1.1, 1.5, 1.2, 0.75, 0.55, 0.35]),
)

# Camera blend table (old Stage 4)
CAMREF_ALPHA_BP = [0., 5., 10., 20., 30.]
CAMREF_ALPHA_V = [0.95, 0.90, 0.85, 0.70, 0.60]

TESLA_BENCHMARKS = {
    "highway": {"jerk_rms": 20.0, "rate_p95": 8.0, "tracking_mae": 0.3, "oscillation_pct": 3.0},
    "city":    {"jerk_rms": 30.0, "rate_p95": 20.0, "tracking_mae": 0.5, "oscillation_pct": 5.0},
    "low_speed": {"jerk_rms": 50.0, "rate_p95": 40.0, "tracking_mae": 1.0, "oscillation_pct": 8.0},
}


def make_vm():
    CP = structs.CarParams()
    CP.mass = MASS
    CP.wheelbase = WHEELBASE
    CP.steerRatio = STEER_RATIO
    CP.centerToFront = CENTER_TO_FRONT
    CP.steerRatioRear = 0.0
    from opendbc.car.interfaces import scale_rot_inertia, scale_tire_stiffness
    CP.rotationalInertia = scale_rot_inertia(CP.mass, CP.wheelbase)
    CP.tireStiffnessFront, CP.tireStiffnessRear = scale_tire_stiffness(
        CP.mass, CP.wheelbase, CP.centerToFront, TIRE_STIFFNESS_FACTOR)
    return VehicleModel(CP)


def vm_rate_limit(apply_angle, apply_angle_last, v_ego, VM, jerk, accel, max_rate, steer_step=2):
    v = max(v_ego, 1.0)
    max_curvature_rate = jerk / (v ** 2)
    max_angle_rate_sec = math.degrees(VM.get_steer_from_curvature(max_curvature_rate, v, 0))
    max_angle_delta = max_angle_rate_sec * (DT_CTRL * steer_step)
    max_angle_delta = min(max_angle_delta, max_rate)
    new = np.clip(apply_angle, apply_angle_last - max_angle_delta, apply_angle_last + max_angle_delta)
    max_angle = math.degrees(VM.get_steer_from_curvature(accel / (v ** 2), v, 0))
    new = np.clip(new, -max_angle, max_angle)
    return float(np.clip(new, -176.7, 176.7))


def compute_metrics(angles, cmds, dts, name):
    if len(angles) < 100:
        return None
    a = np.array(angles)
    c = np.array(cmds)
    dt = np.array(dts)
    valid = (dt > 0.005) & (dt < 0.5)
    a, c, dt = a[valid], c[valid], dt[valid]
    if len(a) < 100:
        return None

    # 5-point MA smoothing on angles (MDPS quantization)
    kernel = np.ones(5) / 5
    a_smooth = np.convolve(a, kernel, mode='same')
    rate = np.diff(a_smooth) / dt[1:]
    rate = np.clip(rate, -600, 600)
    rate_smooth = np.convolve(rate, kernel, mode='same')
    jerk = np.diff(rate_smooth) / dt[2:]
    jerk = np.clip(jerk, -3000, 3000)

    tracking_err = np.abs(c - a)

    # Oscillation: high-freq residual energy / total variance
    if np.var(a_smooth) > 1e-6:
        low_freq = np.convolve(a_smooth, np.ones(25) / 25, mode='same')
        residual = a_smooth - low_freq
        osc_pct = np.var(residual) / np.var(a_smooth) * 100
    else:
        osc_pct = 0.0

    return {
        'name': name,
        'n_frames': len(a),
        'jerk_rms': float(np.sqrt(np.mean(jerk ** 2))),
        'rate_p95': float(np.percentile(np.abs(rate), 95)),
        'tracking_mae': float(np.mean(tracking_err)),
        'oscillation_pct': float(osc_pct),
    }


def simulate_pipeline(frames, VM, params):
    """Simulate a steering pipeline on drivelog frames.

    params dict:
      pipeline: 'old_blend' | 'op_only_v1' | 'vm_lpf'
      jerk, accel, max_rate: VM params (for 'vm_lpf')
      lpf_tau_bp, lpf_tau_v: LPF breakpoints (for 'vm_lpf')
    """
    pipeline = params['pipeline']
    results = {b[0]: {'angles': [], 'cmds': [], 'dts': []} for b in BUCKETS}
    apply_last = 0.0
    lpf_last = 0.0
    prev_time = None
    dt_nominal = DT_CTRL * 2  # 20 ms at 50 Hz

    for f in frames:
        v_ms = f['v_ms']
        actual = f['actual']
        op_desired = f['desired'] if f['desired'] is not None else actual
        cam_angle = f['cam_angle']
        lat_active = f['lat_active'] and f['cruise_on']
        v_kmh = f['v_kmh']

        if not lat_active or v_ms < 0.5:
            apply_last = actual
            lpf_last = actual
            continue

        bucket = None
        for bname, lo, hi in BUCKETS:
            if lo <= v_kmh < hi:
                bucket = bname
                break
        if bucket is None:
            apply_last = actual
            lpf_last = actual
            continue

        if pipeline == 'old_blend':
            alpha = float(np.interp(v_ms, CAMREF_ALPHA_BP, CAMREF_ALPHA_V))
            desired = alpha * cam_angle + (1.0 - alpha) * op_desired
            apply_last = apply_std_steer_angle_limits(
                desired, apply_last, v_ms, actual, True, V1_LIMITS)
        elif pipeline == 'op_only_v1':
            apply_last = apply_std_steer_angle_limits(
                op_desired, apply_last, v_ms, actual, True, V1_LIMITS)
        elif pipeline == 'vm_lpf':
            desired = op_desired
            tau_s = float(np.interp(v_ms, params['lpf_tau_bp'], params['lpf_tau_v']))
            if tau_s > 0.001:
                alpha_lpf = dt_nominal / (tau_s + dt_nominal)
                desired = alpha_lpf * desired + (1.0 - alpha_lpf) * lpf_last
            lpf_last = desired
            apply_last = vm_rate_limit(
                desired, apply_last, v_ms, VM,
                params['jerk'], params['accel'], params['max_rate'])

        results[bucket]['angles'].append(actual)
        results[bucket]['cmds'].append(apply_last)
        results[bucket]['dts'].append(dt_nominal)

    return results


def print_comparison(all_results):
    print(f"\n{'Pipeline':<20} {'Bucket':<12} {'Frames':>8} {'Jerk RMS':>10} {'Rate p95':>10} {'MAE':>8} {'Osc %':>8} {'Tesla':>8}")
    print("─" * 96)
    for pipeline_name, bucket_data in all_results:
        for bname, _, _ in BUCKETS:
            d = bucket_data[bname]
            m = compute_metrics(d['angles'], d['cmds'], d['dts'], bname)
            if m is None:
                continue
            tesla = TESLA_BENCHMARKS.get(bname, {})
            t_jerk = tesla.get('jerk_rms', 0)
            verdict = "★★" if m['jerk_rms'] <= t_jerk and t_jerk > 0 else "  "
            print(f"{pipeline_name:<20} {bname:<12} {m['n_frames']:>8,} "
                  f"{m['jerk_rms']:>8.1f}  {m['rate_p95']:>8.1f}  "
                  f"{m['tracking_mae']:>6.2f}  {m['oscillation_pct']:>6.1f}  {verdict}")


def sweep_params(frames, VM):
    """Grid search over VM + LPF parameters."""
    best_score = float('inf')
    best_params = None
    results_log = []

    jerk_range = [2.5, 3.0, 3.5, 4.0]
    accel_range = [2.8, 3.0, 3.3, 3.6]
    max_rate_range = [1.0, 1.3, 1.6]
    tau_max_range = [0.06, 0.10, 0.12, 0.16]

    total = len(jerk_range) * len(accel_range) * len(max_rate_range) * len(tau_max_range)
    print(f"\n=== Parameter sweep: {total} combinations ===")
    count = 0

    for jerk in jerk_range:
        for accel in accel_range:
            for max_rate in max_rate_range:
                for tau_max in tau_max_range:
                    count += 1
                    params = {
                        'pipeline': 'vm_lpf',
                        'jerk': jerk, 'accel': accel, 'max_rate': max_rate,
                        'lpf_tau_bp': [0.0, 4.17],
                        'lpf_tau_v': [tau_max, 0.0],
                    }
                    bucket_data = simulate_pipeline(frames, VM, params)

                    score = 0.0
                    n_valid = 0
                    for bname, _, _ in BUCKETS:
                        m = compute_metrics(
                            bucket_data[bname]['angles'],
                            bucket_data[bname]['cmds'],
                            bucket_data[bname]['dts'], bname)
                        if m is None:
                            continue
                        tesla = TESLA_BENCHMARKS.get(bname, {})
                        w_jerk = 1.0 if bname == 'low_speed' else 0.5
                        w_mae = 0.5
                        w_rate = 0.3
                        score += w_jerk * (m['jerk_rms'] / max(tesla.get('jerk_rms', 50), 1))
                        score += w_mae * (m['tracking_mae'] / max(tesla.get('tracking_mae', 1), 0.1))
                        score += w_rate * (m['rate_p95'] / max(tesla.get('rate_p95', 20), 1))
                        n_valid += 1

                    if n_valid > 0:
                        score /= n_valid
                        results_log.append((score, jerk, accel, max_rate, tau_max))
                        if score < best_score:
                            best_score = score
                            best_params = (jerk, accel, max_rate, tau_max)

                    if count % 48 == 0:
                        print(f"  [{count}/{total}] best so far: score={best_score:.3f} "
                              f"jerk={best_params[0]} accel={best_params[1]} "
                              f"max_rate={best_params[2]} tau={best_params[3]}")

    results_log.sort(key=lambda x: x[0])
    print(f"\n=== Top 10 parameter sets ===")
    print(f"{'Rank':>4} {'Score':>7} {'Jerk':>6} {'Accel':>7} {'MaxRate':>8} {'LPF tau':>8}")
    for i, (sc, j, a, mr, t) in enumerate(results_log[:10]):
        marker = " ◄" if i == 0 else ""
        print(f"{i+1:>4} {sc:>7.3f} {j:>6.1f} {a:>7.1f} {mr:>8.1f} {t:>8.3f}{marker}")

    return best_params


def main():
    print("Loading cache...")
    with open(CACHE_PATH, 'rb') as f:
        data = pickle.load(f)
    frames = data[0] if isinstance(data, tuple) else data
    print(f"Loaded {len(frames):,} frames")

    # Filter to op-active frames for speed
    op_frames = [f for f in frames if f['lat_active'] and f['cruise_on'] and f['v_ms'] > 0.5]
    print(f"Op-active frames: {len(op_frames):,}")

    VM = make_vm()

    # 1. Baseline comparison: old_blend vs op_only_v1 vs vm_lpf (default params)
    print("\n=== Baseline comparison ===")
    pipelines = [
        ('old_blend', {'pipeline': 'old_blend'}),
        ('op_only_v1', {'pipeline': 'op_only_v1'}),
        ('vm_lpf (default)', {
            'pipeline': 'vm_lpf',
            'jerk': 3.5, 'accel': 3.3, 'max_rate': 1.3,
            'lpf_tau_bp': [0.0, 4.17],
            'lpf_tau_v': [0.12, 0.0],
        }),
    ]

    all_results = []
    for name, params in pipelines:
        print(f"  Simulating {name}...")
        bucket_data = simulate_pipeline(frames, VM, params)
        all_results.append((name, bucket_data))

    print_comparison(all_results)

    # 2. Parameter sweep
    best = sweep_params(frames, VM)
    if best:
        jerk, accel, max_rate, tau_max = best
        print(f"\n=== OPTIMAL PARAMS ===")
        print(f"  MAX_LATERAL_JERK  = {jerk}")
        print(f"  MAX_LATERAL_ACCEL = {accel}")
        print(f"  MAX_ANGLE_RATE    = {max_rate}")
        print(f"  LPF tau (0 km/h)  = {tau_max}")
        print(f"\nRunning optimal pipeline...")

        opt_params = {
            'pipeline': 'vm_lpf',
            'jerk': jerk, 'accel': accel, 'max_rate': max_rate,
            'lpf_tau_bp': [0.0, 4.17],
            'lpf_tau_v': [tau_max, 0.0],
        }
        opt_data = simulate_pipeline(frames, VM, opt_params)
        print_comparison([
            all_results[0],  # old_blend
            ('vm_lpf (optimal)', opt_data),
        ])

    print("\nDone.")


if __name__ == '__main__':
    main()
