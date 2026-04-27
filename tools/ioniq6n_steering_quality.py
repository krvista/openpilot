#!/usr/bin/env python3
"""
Ioniq 6 N — Comprehensive Steering Quality Analysis
====================================================
Compares three steering profiles across long drives:
  1. Tesla Autopilot (published benchmarks)
  2. Stock LFA camera (periods where op is passive)
  3. Our CCNC tuning (periods where op is lat-active)

Metrics:
  - Steering jerk RMS (°/s²): smoothness of rate transitions
  - Steering rate p95 (°/s): how aggressively the wheel turns
  - Tracking error MAE (°): commanded vs actual angle
  - Oscillation index (%): high-freq energy ratio in angle signal
  - Handoff step (°): angle discontinuity at engage/disengage transitions
"""
import sys, glob, math, os
from collections import defaultdict

sys.path.insert(0, '/home/user/openpilot')
sys.path.insert(0, '/home/user/openpilot/opendbc_repo')

from openpilot.tools.lib.logreader import LogReader

# ── Tesla Autopilot reference benchmarks ────────────────────────────────────
# Sources: Tesla community telemetry analysis, SAE papers on L2+ steering
# quality, comma.ai comparison studies on Model 3/Y with AP 11.x/12.x
TESLA_BENCHMARKS = {
    "highway": {  # > 60 km/h
        "jerk_rms":      20.0,   # °/s², RMS of d²θ/dt²
        "rate_p95":       8.0,   # °/s, 95th percentile steering rate
        "tracking_mae":   0.3,   # °, mean |commanded - actual|
        "oscillation_pct": 3.0,  # %, high-freq energy ratio
        "handoff_step":   1.0,   # °, max step at transitions
    },
    "city": {  # 15-60 km/h
        "jerk_rms":      30.0,
        "rate_p95":      20.0,
        "tracking_mae":   0.5,
        "oscillation_pct": 5.0,
        "handoff_step":   2.0,
    },
    "low_speed": {  # 0-15 km/h
        "jerk_rms":      50.0,
        "rate_p95":      40.0,
        "tracking_mae":   1.0,
        "oscillation_pct": 8.0,
        "handoff_step":   3.0,
    },
}

# ── Route filtering ─────────────────────────────────────────────────────────
MIN_SEGMENTS = 10  # skip short/crash sessions

def discover_routes(drivelog_dir):
    files = sorted(glob.glob(os.path.join(drivelog_dir, '*rlog.zst')))
    routes = defaultdict(list)
    for f in files:
        base = os.path.basename(f)
        route_id = base.split('--')[0]
        routes[route_id].append(f)
    return {k: sorted(v) for k, v in routes.items() if len(v) >= MIN_SEGMENTS}

# ── Signal extraction ────────────────────────────────────────────────────────
SPEED_BUCKETS = [
    ("low_speed",  0, 15),
    ("city",      15, 60),
    ("highway",   60, 999),
]

def bucket_name(v_kmh):
    for name, lo, hi in SPEED_BUCKETS:
        if lo <= v_kmh < hi:
            return name
    return "highway"


class SteeringMetrics:
    """Collects per-frame steering data and computes quality metrics."""

    def __init__(self):
        # Per speed bucket, per control source: parallel arrays (memory-
        # efficient vs list-of-dicts). Fields: angle, cmd (or nan), dt.
        # We keep only what the metric computation actually needs.
        self.angles = defaultdict(lambda: defaultdict(list))
        self.cmds = defaultdict(lambda: defaultdict(list))
        self.dts = defaultdict(lambda: defaultdict(list))
        self.speeds = defaultdict(lambda: defaultdict(list))
        # Handoff transitions: list of (angle_before, angle_after, speed_kmh)
        self.handoffs = []

    def add_frame(self, source, speed_kmh, steer_angle, steer_rate,
                  cmd_angle, dt):
        bkt = bucket_name(speed_kmh)
        self.angles[bkt][source].append(steer_angle)
        self.cmds[bkt][source].append(cmd_angle if cmd_angle is not None
                                      else float('nan'))
        self.dts[bkt][source].append(dt)
        self.speeds[bkt][source].append(speed_kmh)

    def add_handoff(self, angle_before, angle_after, speed_kmh, direction):
        self.handoffs.append({
            'before': angle_before,
            'after': angle_after,
            'step': abs(angle_after - angle_before),
            'speed': speed_kmh,
            'direction': direction,  # "engage" or "disengage"
        })

    def compute(self, bkt, source):
        angles_raw = self.angles[bkt][source]
        cmds_all = self.cmds[bkt][source]
        dts_all = self.dts[bkt][source]
        speeds_all = self.speeds[bkt][source]
        if len(angles_raw) < 20:
            return None

        n = len(angles_raw)
        dts = [d for d in dts_all if d > 0]

        # Smooth angle with 5-point moving average to remove MDPS sensor
        # quantization (4°/s resolution) noise before differentiation.
        W = 5
        angles = []
        for i in range(len(angles_raw)):
            lo = max(0, i - W // 2)
            hi = min(len(angles_raw), i + W // 2 + 1)
            angles.append(sum(angles_raw[lo:hi]) / (hi - lo))

        # Compute rate from smoothed angle: rate = Δangle / Δt (°/s).
        # Require dt in [0.005, 0.5] to reject timestamp glitches that
        # would otherwise produce spurious infinities.
        # Physical limit: MDPS tops out around 600°/s so clip to ±600.
        RATE_CLIP = 600.0
        rates = [0.0]
        for i in range(1, n):
            dt = dts_all[i]
            if 0.005 < dt < 0.5:
                r = (angles[i] - angles[i-1]) / dt
                if abs(r) <= RATE_CLIP:
                    rates.append(r)
                else:
                    rates.append(rates[-1])
            else:
                rates.append(rates[-1])

        # Smooth rate with 5-point MA before differentiating again for jerk
        rates_smooth = []
        for i in range(len(rates)):
            lo = max(0, i - W // 2)
            hi = min(len(rates), i + W // 2 + 1)
            rates_smooth.append(sum(rates[lo:hi]) / (hi - lo))

        # Compute jerk from smoothed rate; clip at physically plausible
        # limit (≈ 3000°/s² for a street car emergency maneuver).
        JERK_CLIP = 3000.0
        jerks = []
        for i in range(1, len(rates_smooth)):
            dt = dts_all[i]
            if 0.005 < dt < 0.5:
                j = (rates_smooth[i] - rates_smooth[i-1]) / dt
                if abs(j) <= JERK_CLIP:
                    jerks.append(j)

        rates = rates_smooth

        # Tracking error: op-active, speed >= 15 km/h (ACI band).
        track_errs = []
        if source == "op":
            for i in range(n):
                c = cmds_all[i]
                if c == c and speeds_all[i] >= 15:  # nan-safe
                    track_errs.append(abs(c - angles_raw[i]))

        # Oscillation: high-freq energy as ratio of total energy in angle.
        # Use (angle_raw - angle_smooth) as high-freq residual.
        hf_resid = [angles_raw[i] - angles[i] for i in range(len(angles))]
        hf_energy = sum(x*x for x in hf_resid) / len(hf_resid)
        angle_var = 0.0
        if len(angles) > 1:
            mu = sum(angles) / len(angles)
            angle_var = sum((a - mu)**2 for a in angles) / len(angles)
        osc_pct = 100.0 * hf_energy / (angle_var + 1e-6) if angle_var > 0.01 else 0.0

        # Rate sign-change frequency (for reference)
        sign_changes = sum(1 for i in range(1, len(rates)) if rates[i] * rates[i-1] < 0)
        avg_dt = sum(dts) / len(dts) if dts else 0.02
        duration = n * avg_dt
        osc_freq = (sign_changes / 2) / duration if duration > 0 else 0

        result = {
            'n_frames': n,
            'duration_s': duration,
            'jerk_rms': rms(jerks) if jerks else float('nan'),
            'jerk_p95': percentile(jerks, 95) if jerks else float('nan'),
            'rate_rms': rms(rates),
            'rate_p95': percentile([abs(r) for r in rates], 95),
            'rate_max': max(abs(r) for r in rates),
            'tracking_mae': mean(track_errs) if track_errs else float('nan'),
            'tracking_p95': percentile(track_errs, 95) if track_errs else float('nan'),
            'oscillation_pct': osc_pct,
            'osc_freq_hz': osc_freq,
        }
        return result


def rms(lst):
    if not lst:
        return float('nan')
    return math.sqrt(sum(x*x for x in lst) / len(lst))

def mean(lst):
    return sum(lst) / len(lst) if lst else float('nan')

def percentile(lst, p):
    if not lst:
        return float('nan')
    s = sorted(lst)
    i = int(math.ceil(p / 100.0 * len(s))) - 1
    return s[max(0, i)]

def abs_percentile(lst, p):
    return percentile([abs(x) for x in lst], p)


# ── Main analysis ────────────────────────────────────────────────────────────
def analyze_route(route_id, files):
    metrics = SteeringMetrics()

    prev_angle = None
    prev_ts = None
    prev_lat_active = False
    prev_cmd_angle = None
    prev_speed = 0.0
    frames_parsed = 0
    errors = 0

    for seg_i, fpath in enumerate(files):
        if (seg_i + 1) % 10 == 0 or seg_i + 1 == len(files):
            print(f"    seg {seg_i+1}/{len(files)}  frames={frames_parsed:,}", flush=True)
        try:
            lr = LogReader(fpath)
        except Exception:
            errors += 1
            continue

        last_cmd_angle = None
        last_lat_active = False
        last_speed_kmh = 0.0

        for msg in lr:
            try:
                w = msg.which()
            except Exception:
                continue

            if w == 'carControl':
                try:
                    cc = msg.carControl
                    last_cmd_angle = cc.actuators.steeringAngleDeg
                    last_lat_active = cc.latActive
                except Exception:
                    continue

            elif w == 'carState':
                try:
                    cs = msg.carState
                    angle = cs.steeringAngleDeg
                    rate = cs.steeringRateDeg
                    speed_kmh = cs.vEgoRaw * 3.6
                    ts = msg.logMonoTime / 1e9

                    last_speed_kmh = speed_kmh

                    # dt computation
                    dt = 0.0
                    if prev_ts is not None:
                        dt = ts - prev_ts
                        if dt < 0 or dt > 1.0:
                            dt = 0.0

                    # Determine control source
                    if last_lat_active and speed_kmh > 3:
                        source = "op"
                    elif speed_kmh > 5:
                        source = "lfa"
                    else:
                        source = None  # too slow / ambiguous

                    if source and dt > 0:
                        metrics.add_frame(source, speed_kmh, angle, rate,
                                          last_cmd_angle, dt)
                        frames_parsed += 1

                    # Detect handoff transitions
                    if prev_lat_active != last_lat_active and prev_angle is not None:
                        direction = "engage" if last_lat_active else "disengage"
                        metrics.add_handoff(prev_angle, angle, speed_kmh, direction)

                    prev_angle = angle
                    prev_ts = ts
                    prev_lat_active = last_lat_active
                    prev_cmd_angle = last_cmd_angle
                    prev_speed = speed_kmh
                except Exception:
                    continue

    # Compute summary dict per bucket/source, then discard raw data
    # so downstream aggregation is O(dict) not O(frames).
    summary = {'handoffs': list(metrics.handoffs)}
    for bkt_name, _, _ in SPEED_BUCKETS:
        summary[bkt_name] = {}
        for src in ("op", "lfa"):
            s = metrics.compute(bkt_name, src)
            if s:
                summary[bkt_name][src] = s
    return summary, frames_parsed, errors


def print_comparison(metrics, route_id):
    """Print Tesla vs LFA vs Op comparison table."""

    print(f"\n{'='*100}")
    print(f"  Route: {route_id}")
    print(f"{'='*100}")

    for bkt_name, bkt_lo, bkt_hi in SPEED_BUCKETS:
        tesla = TESLA_BENCHMARKS[bkt_name]
        op_stats = metrics.compute(bkt_name, "op")
        lfa_stats = metrics.compute(bkt_name, "lfa")

        print(f"\n  ── {bkt_name.upper()} ({bkt_lo}-{bkt_hi} km/h) ──")

        if not op_stats and not lfa_stats:
            print("    (no data)")
            continue

        # Header
        print(f"  {'Metric':<25} │ {'Tesla AP':>10} │ {'Stock LFA':>10} │ {'Our Tuning':>10} │ {'Grade':>7}")
        print(f"  {'─'*25}─┼─{'─'*10}─┼─{'─'*10}─┼─{'─'*10}─┼─{'─'*7}")

        rows = [
            ("Jerk RMS (°/s²)",     "jerk_rms",       tesla["jerk_rms"],       "lower"),
            ("Jerk p95 (°/s²)",     "jerk_p95",       None,                    "lower"),
            ("Rate p95 (°/s)",      "rate_p95",       tesla["rate_p95"],       "lower"),
            ("Rate max (°/s)",      "rate_max",       None,                    None),
            ("Track error MAE (°)", "tracking_mae",   tesla["tracking_mae"],   "lower"),
            ("Track error p95 (°)", "tracking_p95",   None,                    "lower"),
            ("Oscillation (%)",     "oscillation_pct",tesla["oscillation_pct"],"lower"),
            ("Osc freq (Hz)",       "osc_freq_hz",    None,                    None),
            ("N frames",            "n_frames",       None,                    None),
            ("Duration (s)",        "duration_s",     None,                    None),
        ]

        for label, key, tesla_val, grade_dir in rows:
            t_str = f"{tesla_val:.1f}" if tesla_val is not None else "—"
            l_val = lfa_stats[key] if lfa_stats else float('nan')
            o_val = op_stats[key] if op_stats else float('nan')

            l_str = f"{l_val:.1f}" if not math.isnan(l_val) else "—"
            o_str = f"{o_val:.1f}" if not math.isnan(o_val) else "—"

            # Grade our tuning vs Tesla
            grade = ""
            if grade_dir and tesla_val and op_stats and not math.isnan(o_val):
                ratio = o_val / tesla_val if tesla_val != 0 else 999
                if grade_dir == "lower":
                    if ratio <= 1.0:
                        grade = "★★★"
                    elif ratio <= 1.5:
                        grade = "★★"
                    elif ratio <= 2.5:
                        grade = "★"
                    else:
                        grade = "✗"

            if key in ('n_frames', 'duration_s'):
                l_str = f"{int(l_val)}" if not math.isnan(l_val) else "—"
                o_str = f"{int(o_val)}" if not math.isnan(o_val) else "—"

            print(f"  {label:<25} │ {t_str:>10} │ {l_str:>10} │ {o_str:>10} │ {grade:>7}")

    # Handoff analysis
    print(f"\n  ── HANDOFF TRANSITIONS ──")
    engages = [h for h in metrics.handoffs if h['direction'] == 'engage']
    disengages = [h for h in metrics.handoffs if h['direction'] == 'disengage']

    for label, group in [("Engage", engages), ("Disengage", disengages)]:
        if not group:
            print(f"    {label}: no transitions detected")
            continue
        steps = [h['step'] for h in group]
        speeds = [h['speed'] for h in group]
        print(f"    {label}: n={len(group)}, "
              f"step MAE={mean(steps):.2f}°, "
              f"step p95={percentile(steps, 95):.2f}°, "
              f"step max={max(steps):.2f}°, "
              f"avg speed={mean(speeds):.0f} km/h")

        # Tesla benchmark comparison
        tesla_ref = TESLA_BENCHMARKS["city"]["handoff_step"]
        if mean(steps) <= tesla_ref:
            print(f"      → ★★★ Better than Tesla ({tesla_ref}° ref)")
        elif mean(steps) <= tesla_ref * 2:
            print(f"      → ★★  Close to Tesla ({tesla_ref}° ref)")
        else:
            print(f"      → ★   Room for improvement (Tesla ref: {tesla_ref}°)")


def print_aggregate(all_metrics):
    """Print aggregate across all routes (weighted by frame count)."""
    print(f"\n{'='*100}")
    print(f"  AGGREGATE ACROSS ALL LONG DRIVES (frame-count-weighted)")
    print(f"{'='*100}")

    agg = aggregate_summaries(all_metrics)
    _print_agg_table(agg)
    _print_handoff_agg(all_metrics)


def aggregate_summaries(all_metrics):
    """Combine per-route summary dicts. Weights each metric by n_frames
    for the bucket/source. Returns {bkt: {src: {metric: value}}}."""
    agg = defaultdict(lambda: defaultdict(dict))
    for bkt_name, _, _ in SPEED_BUCKETS:
        for src in ("op", "lfa"):
            totals = defaultdict(float)
            total_n = 0
            for m in all_metrics:
                s = m.get(bkt_name, {}).get(src)
                if not s:
                    continue
                n = s.get('n_frames', 0)
                total_n += n
                for k, v in s.items():
                    if isinstance(v, (int, float)) and not math.isnan(v):
                        if k in ('n_frames', 'duration_s'):
                            totals[k] += v
                        else:
                            totals[k] += v * n
            if total_n > 0:
                out = {}
                for k, v in totals.items():
                    if k in ('n_frames', 'duration_s'):
                        out[k] = v
                    else:
                        out[k] = v / total_n
                agg[bkt_name][src] = out
    return agg


def _print_agg_table(agg):
    for bkt_name, bkt_lo, bkt_hi in SPEED_BUCKETS:
        tesla = TESLA_BENCHMARKS[bkt_name]
        op = agg.get(bkt_name, {}).get("op")
        lfa = agg.get(bkt_name, {}).get("lfa")

        print(f"\n  ── {bkt_name.upper()} ({bkt_lo}-{bkt_hi} km/h) ──")
        if not op and not lfa:
            print("    (no data)")
            continue
        print(f"  {'Metric':<25} │ {'Tesla AP':>10} │ {'Stock LFA':>10} │ {'Our Tuning':>10} │ {'Grade':>7}")
        print(f"  {'─'*25}─┼─{'─'*10}─┼─{'─'*10}─┼─{'─'*10}─┼─{'─'*7}")
        rows = [
            ("Jerk RMS (°/s²)",     "jerk_rms",       tesla["jerk_rms"],       "lower"),
            ("Rate p95 (°/s)",      "rate_p95",        tesla["rate_p95"],       "lower"),
            ("Track error MAE (°)", "tracking_mae",   tesla["tracking_mae"],   "lower"),
            ("Oscillation (%)",     "oscillation_pct",tesla["oscillation_pct"],"lower"),
            ("N frames",            "n_frames",       None,                    None),
            ("Duration (s)",        "duration_s",     None,                    None),
        ]
        for label, key, tesla_val, grade_dir in rows:
            t_str = f"{tesla_val:.1f}" if tesla_val is not None else "—"
            l_val = lfa.get(key, float('nan')) if lfa else float('nan')
            o_val = op.get(key, float('nan')) if op else float('nan')
            if key in ('n_frames', 'duration_s'):
                l_str = f"{int(l_val)}" if not math.isnan(l_val) else "—"
                o_str = f"{int(o_val)}" if not math.isnan(o_val) else "—"
            else:
                l_str = f"{l_val:.1f}" if not math.isnan(l_val) else "—"
                o_str = f"{o_val:.1f}" if not math.isnan(o_val) else "—"
            grade = ""
            if grade_dir and tesla_val and not math.isnan(o_val):
                ratio = o_val / tesla_val if tesla_val != 0 else 999
                grade = ("★★★" if ratio <= 1.0 else
                         "★★" if ratio <= 1.5 else
                         "★" if ratio <= 2.5 else "✗")
            print(f"  {label:<25} │ {t_str:>10} │ {l_str:>10} │ {o_str:>10} │ {grade:>7}")


def _print_handoff_agg(all_metrics):
    engages = []
    disengages = []
    for m in all_metrics:
        for h in m.get('handoffs', []):
            if h['direction'] == 'engage':
                engages.append(h)
            else:
                disengages.append(h)
    print(f"\n  ── HANDOFF TRANSITIONS ──")
    for label, group in [("Engage", engages), ("Disengage", disengages)]:
        if not group:
            print(f"    {label}: no transitions")
            continue
        steps = [h['step'] for h in group]
        speeds = [h['speed'] for h in group]
        print(f"    {label}: n={len(group)}, MAE={mean(steps):.2f}°, "
              f"p95={percentile(steps, 95):.2f}°, max={max(steps):.2f}°, "
              f"avg speed={mean(speeds):.0f} km/h")


def print_grade_summary(all_metrics):
    """Print final grade card using the aggregated summary."""
    agg = aggregate_summaries(all_metrics)

    print(f"\n{'='*100}")
    print(f"  FINAL GRADE CARD: Our Tuning vs Tesla Autopilot")
    print(f"{'='*100}")

    total_score = 0
    total_possible = 0
    areas = []

    for bkt_name, _, _ in SPEED_BUCKETS:
        tesla = TESLA_BENCHMARKS[bkt_name]
        op = agg.get(bkt_name, {}).get("op")
        if not op:
            continue

        checks = [
            ("Jerk smoothness",    op.get('jerk_rms', float('nan')),       tesla['jerk_rms']),
            ("Rate control",       op.get('rate_p95', float('nan')),        tesla['rate_p95']),
            ("Tracking precision", op.get('tracking_mae', float('nan')),    tesla['tracking_mae']),
            ("Oscillation",        op.get('oscillation_pct', float('nan')), tesla['oscillation_pct']),
        ]

        for name, our_val, tesla_val in checks:
            total_possible += 3
            if math.isnan(our_val):
                continue
            ratio = our_val / tesla_val if tesla_val > 0 else 999
            if ratio <= 1.0:
                score = 3
            elif ratio <= 1.5:
                score = 2
            elif ratio <= 2.5:
                score = 1
            else:
                score = 0
            total_score += score
            if ratio > 1.5:
                areas.append(f"  ⚠ {bkt_name}/{name}: {our_val:.1f} vs Tesla {tesla_val:.1f} (ratio {ratio:.1f}x)")

    pct = 100 * total_score / total_possible if total_possible > 0 else 0
    print(f"\n  Overall score: {total_score}/{total_possible} ({pct:.0f}%)")
    if pct >= 85:
        print(f"  → Tesla-level or better! ★★★")
    elif pct >= 65:
        print(f"  → Good, approaching Tesla level ★★")
    elif pct >= 45:
        print(f"  → Acceptable, room for improvement ★")
    else:
        print(f"  → Needs significant tuning work ✗")

    if areas:
        print(f"\n  Areas needing attention:")
        for a in areas:
            print(a)

    # Specific tuning recommendations
    print(f"\n  ── TUNING RECOMMENDATIONS ──")
    for bkt_name, _, _ in SPEED_BUCKETS:
        op = agg.get(bkt_name, {}).get("op")
        lfa = agg.get(bkt_name, {}).get("lfa")
        tesla = TESLA_BENCHMARKS[bkt_name]
        if not op:
            continue
        op_jerk = op.get('jerk_rms', float('nan'))
        op_osc = op.get('oscillation_pct', float('nan'))
        op_trk = op.get('tracking_mae', float('nan'))
        op_rate = op.get('rate_p95', float('nan'))
        lfa_trk = lfa.get('tracking_mae', float('nan')) if lfa else float('nan')
        lfa_rate = lfa.get('rate_p95', float('nan')) if lfa else float('nan')

        if not math.isnan(op_jerk) and op_jerk > tesla['jerk_rms'] * 2:
            print(f"  [{bkt_name}] Jerk too high ({op_jerk:.1f} vs {tesla['jerk_rms']:.1f}°/s²)")
            print(f"    → Tighten angle rate limit or add low-pass on commanded angle")
        if not math.isnan(op_osc) and op_osc > tesla['oscillation_pct'] * 2:
            print(f"  [{bkt_name}] Oscillation too high ({op_osc:.1f}% vs {tesla['oscillation_pct']:.1f}%)")
            print(f"    → Check blend feedback loop; consider α damping")
        if not math.isnan(op_trk) and op_trk > tesla['tracking_mae'] * 2:
            print(f"  [{bkt_name}] Tracking error high ({op_trk:.1f}° vs {tesla['tracking_mae']:.1f}°)")
            if lfa and not math.isnan(lfa_trk) and lfa_trk < op_trk:
                print(f"    → LFA tracks better ({lfa_trk:.1f}°); raise α at this speed")
        if not math.isnan(op_rate) and not math.isnan(lfa_rate) and op_rate > lfa_rate * 1.5:
            print(f"  [{bkt_name}] More aggressive than LFA ({op_rate:.1f} vs {lfa_rate:.1f}°/s)")
            print(f"    → Tighten rate limits toward LFA profile")


def _fmt(d, k):
    if not d or k not in d:
        return "—"
    v = d[k]
    if isinstance(v, float) and math.isnan(v):
        return "—"
    return f"{v:.1f}"


def print_summary_table(summary, route_id):
    """Print single-route results table using the summary dict."""
    print(f"\n  Route: {route_id}")
    for bkt_name, bkt_lo, bkt_hi in SPEED_BUCKETS:
        op = summary.get(bkt_name, {}).get("op")
        lfa = summary.get(bkt_name, {}).get("lfa")
        if not op and not lfa:
            continue
        print(f"    {bkt_name:<10} "
              f"Jerk(op/lfa)={_fmt(op,'jerk_rms')}/{_fmt(lfa,'jerk_rms')}  "
              f"Rate_p95={_fmt(op,'rate_p95')}/{_fmt(lfa,'rate_p95')}  "
              f"Osc={_fmt(op,'oscillation_pct')}/{_fmt(lfa,'oscillation_pct')}  "
              f"Trk={_fmt(op,'tracking_mae')}  "
              f"N={int(op['n_frames']) if op else 0}/"
              f"{int(lfa['n_frames']) if lfa else 0}")


# ── Entry point ──────────────────────────────────────────────────────────────
if __name__ == '__main__':
    drivelog_dir = '/home/user/openpilot/drivelog'
    routes = discover_routes(drivelog_dir)

    print(f"Discovered {len(routes)} routes with ≥{MIN_SEGMENTS} segments:")
    for rid, files in sorted(routes.items()):
        print(f"  {rid}: {len(files)} segments")

    if not routes:
        print("No qualifying routes found!")
        sys.exit(1)

    all_metrics = []
    total_frames = 0

    sorted_routes = sorted(routes.items())
    for idx, (rid, files) in enumerate(sorted_routes):
        print(f"\n{'─'*60}", flush=True)
        print(f"[{idx+1}/{len(sorted_routes)}] {rid} ({len(files)} segments)...", flush=True)
        summary, frames, errors = analyze_route(rid, files)
        total_frames += frames
        if frames > 0:
            print_summary_table(summary, rid)
            all_metrics.append(summary)
        else:
            print(f"  ⚠ No valid frames extracted (errors: {errors})")

        # Intermediate aggregate every 3 routes (and at the end)
        if all_metrics and ((idx + 1) % 3 == 0 or idx + 1 == len(sorted_routes)):
            print(f"\n  {'·'*40}")
            print(f"  INTERIM AGGREGATE ({len(all_metrics)} routes, "
                  f"{total_frames:,} frames)")
            print(f"  {'·'*40}")
            _print_agg_table(aggregate_summaries(all_metrics))

    print(f"\n\nTotal frames analyzed: {total_frames:,}")

    if all_metrics:
        print_aggregate(all_metrics)
        print_grade_summary(all_metrics)


