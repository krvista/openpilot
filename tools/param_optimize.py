#!/usr/bin/env python3
"""Parameter optimization sweep for CURV_LPF_TAU, LOWSPEED_LPF_TAU, ANGLE_RATE_V.
Simulates the carcontroller angle pipeline on drivelog data and measures MAE."""
import sys, os, glob
import numpy as np
from itertools import product
sys.path.insert(0, '/home/user/openpilot')
sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG = '/home/user/openpilot/drivelog'
segs = sorted(glob.glob(os.path.join(DRIVELOG, '*rlog.zst')))

# ---- Extract data ----
print("Extracting drivelog data...", file=sys.stderr)
des_all, act_all, v_all, ts_all = [], [], [], []
for i, path in enumerate(segs):
    try:
        last_v, last_ang = 0.0, 0.0
        for msg in LogReader(path):
            w = msg.which()
            if w == 'carState':
                last_v = msg.carState.vEgo
                last_ang = msg.carState.steeringAngleDeg
            elif w == 'controlsState':
                lat = msg.controlsState.lateralControlState
                if lat.which() == 'angleState' and bool(lat.angleState.active) and last_v > 0.5:
                    des_all.append(float(lat.angleState.steeringAngleDesiredDeg))
                    act_all.append(last_ang)
                    v_all.append(last_v)
                    ts_all.append(msg.logMonoTime * 1e-9)
    except:
        pass
    if (i+1) % 20 == 0:
        print(f'  {i+1}/{len(segs)}', file=sys.stderr)

des = np.array(des_all)
act = np.array(act_all)
v = np.array(v_all)
N = len(des)
print(f"Extracted {N} frames", file=sys.stderr)

DT = 0.02  # 50 Hz

# ---- Simulate pipeline ----
def simulate(curv_tau, ls_tau_max, ls_tau_speed, rate_v, rate_bp):
    """Simulate the angle command pipeline and return per-frame commanded angles."""
    curv_lpf = 0.0
    lpf_last = 0.0
    apply_last = act[0]
    commands = np.zeros(N)

    for i in range(N):
        d = des[i]
        vi = v[i]
        a = act[i]

        # Phase 6: curvature LPF
        if curv_tau > 0.001:
            alpha_c = DT / (curv_tau + DT)
            curv_lpf = alpha_c * d + (1 - alpha_c) * curv_lpf
        else:
            curv_lpf = d
        angle_deg = curv_lpf

        # Phase 4-B: low-speed LPF
        ls_tau = float(np.interp(vi, [0.0, ls_tau_speed], [ls_tau_max, 0.0]))
        if ls_tau > 0.001:
            alpha_ls = DT / (ls_tau + DT)
            angle_deg = alpha_ls * angle_deg + (1 - alpha_ls) * lpf_last
        lpf_last = angle_deg

        # Phase 4-C: per-step rate cap
        cap = float(np.interp(vi, rate_bp, rate_v))
        angle_deg = float(np.clip(angle_deg, apply_last - cap, apply_last + cap))

        # Simplified VM limit: just clamp to reasonable range
        # (actual VM jerk/accel limit is speed-dependent, but for relative comparison
        # the rate cap is the dominant constraint at low speed)
        apply_last = angle_deg
        commands[i] = angle_deg

    return commands

def compute_mae(commands, speed_lo=0.5, speed_hi=100):
    mask = (v >= speed_lo) & (v < speed_hi)
    if mask.sum() < 100:
        return 999.0
    return np.abs(commands[mask] - act[mask]).mean()

def compute_mae_buckets(commands):
    buckets = {}
    for lo, hi, label in [(3,8,'11-29'), (8,15,'29-54'), (15,22,'54-79'), (22,35,'79+')]:
        m = (v >= lo) & (v < hi)
        if m.sum() > 100:
            buckets[label] = np.abs(commands[m] - act[m]).mean()
    return buckets

# ---- Current baseline ----
CURRENT_RATE_BP = [0., 7., 11., 17., 23., 30.]
CURRENT_RATE_V = [2.5, 2.5, 2.5, 2.3, 2.3, 1.5]

print("Running baseline...", file=sys.stderr)
baseline = simulate(0.20, 0.16, 4.17, CURRENT_RATE_V, CURRENT_RATE_BP)
baseline_mae = compute_mae(baseline, 0.5)
baseline_buckets = compute_mae_buckets(baseline)
print(f"\n=== BASELINE (current params) ===")
print(f"  CURV_LPF_TAU=0.20, LS_TAU_MAX=0.16, LS_TAU_SPEED=4.17 m/s (15 km/h)")
print(f"  RATE_V={CURRENT_RATE_V}")
print(f"  Overall MAE: {baseline_mae:.3f}°")
for k, v_ in baseline_buckets.items():
    print(f"  {k} km/h: {v_:.3f}°")

# ---- Parameter sweep ----
print("\nRunning parameter sweep...", file=sys.stderr)

# Sweep ranges
curv_taus = [0.05, 0.10, 0.15, 0.20]
ls_tau_maxs = [0.04, 0.08, 0.12, 0.16]
ls_tau_speeds = [4.17]  # keep fixed (15 km/h)

# Rate cap configurations (named)
rate_configs = {
    'current':   [2.5, 2.5, 2.5, 2.3, 2.3, 1.5],
    'loose_low': [3.0, 3.0, 2.8, 2.3, 2.3, 1.5],   # loosen 0-11 m/s
    'loose_all': [3.0, 3.0, 3.0, 2.8, 2.5, 1.8],   # loosen everywhere
    'very_loose':[3.0, 3.0, 3.0, 3.0, 3.0, 2.0],   # aggressive
}

results = []
total = len(curv_taus) * len(ls_tau_maxs) * len(rate_configs)
done = 0

for ct in curv_taus:
    for lt in ls_tau_maxs:
        for rname, rv in rate_configs.items():
            cmds = simulate(ct, lt, 4.17, rv, CURRENT_RATE_BP)
            mae = compute_mae(cmds, 0.5)
            buckets = compute_mae_buckets(cmds)
            results.append({
                'curv_tau': ct, 'ls_tau': lt, 'rate': rname,
                'mae': mae, 'buckets': buckets
            })
            done += 1
            if done % 10 == 0:
                print(f'  {done}/{total}', file=sys.stderr)

# Sort by MAE
results.sort(key=lambda x: x['mae'])

print(f"\n=== TOP 15 CONFIGURATIONS (sorted by overall MAE) ===")
print(f"{'Rank':>4} {'curv_τ':>7} {'ls_τ':>6} {'rate_cfg':>11} {'MAE':>7} {'11-29':>7} {'29-54':>7} {'54-79':>7} {'79+':>6} {'vs base':>8}")
print('-' * 85)
for i, r in enumerate(results[:15]):
    b = r['buckets']
    pct = (r['mae'] / baseline_mae - 1) * 100
    print(f"{i+1:>4} {r['curv_tau']:>7.2f} {r['ls_tau']:>6.2f} {r['rate']:>11} {r['mae']:>7.3f} "
          f"{b.get('11-29',0):>7.2f} {b.get('29-54',0):>7.2f} {b.get('54-79',0):>7.2f} {b.get('79+',0):>6.2f} {pct:>+7.1f}%")

print(f"\n=== WORST 5 (for reference) ===")
for i, r in enumerate(results[-5:]):
    b = r['buckets']
    pct = (r['mae'] / baseline_mae - 1) * 100
    print(f"  {r['curv_tau']:.2f} / {r['ls_tau']:.2f} / {r['rate']:>11} → MAE={r['mae']:.3f}° ({pct:+.1f}%)")

# Best config details
best = results[0]
print(f"\n=== RECOMMENDED CONFIG ===")
print(f"  CURV_LPF_TAU = {best['curv_tau']}")
print(f"  LOWSPEED_LPF_TAU_V = [{best['ls_tau']}, 0.0]")
print(f"  ANGLE_RATE_V = {rate_configs[best['rate']]}  ('{best['rate']}')")
print(f"  MAE: {baseline_mae:.3f}° → {best['mae']:.3f}° ({(best['mae']/baseline_mae-1)*100:+.1f}%)")
for k in ['11-29', '29-54', '54-79', '79+']:
    old = baseline_buckets.get(k, 0)
    new = best['buckets'].get(k, 0)
    print(f"    {k} km/h: {old:.3f}° → {new:.3f}° ({(new/old-1)*100:+.1f}%)" if old > 0 else f"    {k}: n/a")
