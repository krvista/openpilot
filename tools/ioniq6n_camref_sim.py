#!/usr/bin/env python3
"""Stage 4a simulator: camera-referenced feedforward tuning.

Reads the DBC-accurate cache produced by tools/ioniq6n_reanalysis_dbc.py
(~1.24M frames across routes 00000028-0000002d). For each op-mode frame
we have three angles:

  * op_curv_angle  — what openpilot's LatControlAngle produced from curvature
                     (= cc.actuators.steeringAngleDeg in the log)
  * cam_angle      — what the camera simultaneously commanded via its own
                     LKAS_ALT on bus 2 (ADAS_StrAnglReqVal, DBC-decoded)
  * actual         — steeringAngleDeg reported by the EPS

Stage 0 showed err_camref = |cam - actual| is 3-10x smaller than err_curv.
The open question Stage 4a answers:

    "What α₀(v) table and trust-multiplier q give the best balance
     between matching camera accuracy AND preserving op's navigation
     intent when the two disagree?"

Simulation metrics (per speed bucket):
  1. MAE_blend_vs_cam:
       |blend(α,q) − cam_angle| averaged over op frames
     → lower is better when cam is the trusted reference
  2. NavPreserve:
       when |cam − op_curv| > nav_threshold (e.g. 3°), how close does
       blend stay to op_curv? High % = nav intent preserved.
  3. Disagreement stats:
       distribution of |cam − op_curv| per bucket, so we know how often
       navigation disagreement happens.
  4. Online q dynamics:
       simulate rolling 30s RMSE of (cam - actual) during lfa_passthrough
       and map it to q ∈ [q_min, 1.0]; report q distribution per bucket.

Candidate α₀(v) tables are specified as lists of (v_ms, α) breakpoints.
Grid search covers low-speed-high vs high-speed-low combinations.

Cache: /tmp/camref_sim_cache.pkl (reuses Stage 0 cache).
Usage: python tools/ioniq6n_camref_sim.py [--refresh]
"""
import os
import pickle
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')

STAGE0_CACHE = '/tmp/reanalysis_dbc_cache.pkl'

# ─────── Candidate α₀(v) tables ────────
# Format: list of (name, breakpoints_ms, alphas). Interpolated linearly.

ALPHA_CANDIDATES = [
  # name,                v_ms breakpoints,        α (camera weight), nav_gate (cap α when |cam-op_curv|>NAV_THRESH)
  ('A-conservative',     [0,  5, 10, 20, 30],     [0.30, 0.30, 0.30, 0.30, 0.30], None),
  ('C-flat0.7',          [0,  5, 10, 20, 30],     [0.70, 0.70, 0.70, 0.70, 0.70], None),
  ('F-all-high',         [0,  5, 10, 20, 30],     [0.80, 0.80, 0.80, 0.70, 0.60], None),
  ('F+nav_gate0.3',      [0,  5, 10, 20, 30],     [0.80, 0.80, 0.80, 0.70, 0.60], 0.30),
  ('F+nav_gate0.5',      [0,  5, 10, 20, 30],     [0.80, 0.80, 0.80, 0.70, 0.60], 0.50),
  ('C+nav_gate0.3',      [0,  5, 10, 20, 30],     [0.70, 0.70, 0.70, 0.70, 0.70], 0.30),
  ('J-mild+nav',         [0,  3,  7, 15, 25, 30], [0.60, 0.60, 0.65, 0.70, 0.75, 0.75], 0.35),
  ('K-agg+nav',          [0,  3,  7, 15, 25, 30], [0.75, 0.75, 0.75, 0.75, 0.70, 0.65], 0.30),
  ('H-camalways',        [0,  5, 10, 20, 30],     [1.00, 1.00, 1.00, 1.00, 1.00], None),
  ('I-curvalways',       [0,  5, 10, 20, 30],     [0.00, 0.00, 0.00, 0.00, 0.00], None),
]

BUCKETS = [
  ('parking', 0, 10),
  ('20km/h', 10, 25),
  ('30km/h', 25, 35),
  ('40km/h', 35, 45),
  ('50km/h', 45, 55),
  ('60-70',  55, 75),
  ('80-90',  75, 95),
]

NAV_DISAGREEMENT_DEG = 3.0    # |cam - op_curv| > 3° = meaningful planner divergence
TRUST_WINDOW_S = 30
TRUST_Q_MIN = 0.2
TRUST_Q_MAX = 1.0
TRUST_REFERENCE_RMSE = 1.5    # deg — above this, q collapses toward Q_MIN


def classify_mode(f):
  if f['pressed_torque'] > 200:
    return 'manual'
  if f['cruise_on'] and f['lat_active']:
    return 'op'
  if f['cruise_on'] and not f['lat_active']:
    return 'lfa_override'
  if not f['cruise_on'] and f['pressed_torque'] < 150:
    return 'lfa_passthrough'
  return 'manual'


def bucket_of(v_kmh):
  for name, lo, hi in BUCKETS:
    if lo <= v_kmh < hi:
      return name
  return None


def interp_alpha(v_ms, bps, alphas):
  return float(np.interp(v_ms, bps, alphas))


def load_frames():
  print(f"Loading Stage 0 cache from {STAGE0_CACHE}…")
  with open(STAGE0_CACHE, 'rb') as f:
    return pickle.load(f)


def report_disagreement(frames):
  """Report how often cam and op_curv disagree by bucket."""
  print("\n=== Cam vs op_curv disagreement distribution (op mode only) ===")
  g = defaultdict(list)
  for f in frames:
    if classify_mode(f) != 'op':
      continue
    b = bucket_of(f['v_kmh'])
    if b is None:
      continue
    diff = abs(f['cam_angle'] - (f['desired'] or 0.0))
    g[b].append(diff)
  print(f"{'bucket':<9} {'n':>8} {'p50':>7} {'p95':>7} {'p99':>7} {'% > nav_th':>11}")
  for name, _, _ in BUCKETS:
    arr = np.array(g.get(name, []))
    if len(arr) < 500:
      continue
    over = (arr > NAV_DISAGREEMENT_DEG).sum() / len(arr) * 100
    print(f"{name:<9} {len(arr):>8,} "
          f"{np.percentile(arr,50):>5.2f}° "
          f"{np.percentile(arr,95):>5.2f}° "
          f"{np.percentile(arr,99):>5.2f}° "
          f"{over:>10.1f}%")


def simulate_trust_q(frames, window_s=TRUST_WINDOW_S):
  """Simulate the online q trust multiplier using rolling RMSE on lfa_passthrough.

  Idea: during times when camera is driving (passthrough), we can observe how
  tightly cam_angle tracks actual. Big RMSE → untrustworthy camera → q↓.

  We actually want to run this 'always on' (including during op), by tracking
  RMSE of (cam_angle - actual) regardless of who's commanding. When op is
  steering, cam_angle is advisory — its tracking of actual reflects a combination
  of (a) camera's own accuracy and (b) the fact that actual is being driven by
  op. We include only lfa_passthrough and lfa_override frames in the RMSE.
  """
  print("\n=== Simulated online trust q distribution ===")
  # Sort frames by (route, seg, time-order assumed preserved within seg)
  route_seg_order = defaultdict(list)
  for i, f in enumerate(frames):
    route_seg_order[(f['route'], f['seg'])].append(i)

  window_frames = int(window_s * 100)   # 100 Hz
  q_per_bucket = defaultdict(list)

  for (route, seg), idxs in route_seg_order.items():
    buf = []                            # rolling list of (cam-actual) errors
    for i in idxs:
      f = frames[i]
      mode = classify_mode(f)
      # Only accumulate from lfa passthrough / override (where cam drives the wheel)
      if mode in ('lfa_passthrough', 'lfa_override'):
        buf.append(f['cam_angle'] - f['actual'])
        if len(buf) > window_frames:
          buf.pop(0)
      # Compute q based on current buffer
      if len(buf) >= 100:
        rmse = float(np.sqrt(np.mean(np.array(buf) ** 2)))
        q = TRUST_Q_MAX - (rmse / TRUST_REFERENCE_RMSE) * (TRUST_Q_MAX - TRUST_Q_MIN)
        q = float(np.clip(q, TRUST_Q_MIN, TRUST_Q_MAX))
      else:
        q = TRUST_Q_MAX  # default to full trust before enough data
      # Record q for this frame if op
      if mode == 'op':
        b = bucket_of(f['v_kmh'])
        if b:
          q_per_bucket[b].append(q)

  print(f"{'bucket':<9} {'n':>8} {'q_p5':>6} {'q_p50':>7} {'q_p95':>7} {'q_mean':>7}")
  for name, _, _ in BUCKETS:
    arr = np.array(q_per_bucket.get(name, []))
    if len(arr) < 100:
      continue
    print(f"{name:<9} {len(arr):>8,} "
          f"{np.percentile(arr,5):>5.2f} "
          f"{np.percentile(arr,50):>5.2f} "
          f"{np.percentile(arr,95):>5.2f} "
          f"{arr.mean():>5.2f}")


def simulate_alpha_candidate(frames, name, bps, alphas, nav_gate=None):
  """For each candidate α(v) table, compute per-bucket metrics.

  If `nav_gate` is not None, it's the maximum α applied whenever
  |cam − op_curv| > NAV_DISAGREEMENT_DEG (i.e. the planner is disagreeing
  with the camera about where to go — probably a navigation maneuver).
  """
  metrics = defaultdict(lambda: {
    'n': 0,
    'abs_err_vs_cam': [],
    'abs_err_vs_op': [],
    'nav_event_deviations': [],
    'nav_gate_active': 0,
  })

  for f in frames:
    if classify_mode(f) != 'op':
      continue
    b = bucket_of(f['v_kmh'])
    if b is None:
      continue
    cam = f['cam_angle']
    op_curv = f['desired'] or 0.0
    alpha = interp_alpha(f['v_ms'], bps, alphas)
    disagree = abs(cam - op_curv) > NAV_DISAGREEMENT_DEG
    if nav_gate is not None and disagree:
      alpha = min(alpha, nav_gate)
    blend = alpha * cam + (1 - alpha) * op_curv
    d = metrics[b]
    d['n'] += 1
    d['abs_err_vs_cam'].append(abs(blend - cam))
    d['abs_err_vs_op'].append(abs(blend - op_curv))
    if disagree:
      d['nav_event_deviations'].append(abs(blend - op_curv))
      if nav_gate is not None:
        d['nav_gate_active'] += 1

  return metrics


def print_alpha_result(name, bps, alphas, metrics):
  """Print one candidate's summary."""
  print(f"\n--- {name}  α={dict(zip(bps, alphas))} ---")
  print(f"{'bucket':<9} {'n':>8} "
        f"{'MAE_cam':>9} {'MAE_op':>8}  "
        f"{'nav_n':>6} {'nav_dev_p50':>12} {'nav_dev_p95':>12}")
  for bname, _, _ in BUCKETS:
    d = metrics.get(bname)
    if not d or d['n'] < 500:
      continue
    ecam = np.array(d['abs_err_vs_cam'])
    eop = np.array(d['abs_err_vs_op'])
    nav = np.array(d['nav_event_deviations'])
    nav_str = (f"{np.percentile(nav,50):>10.2f}°   {np.percentile(nav,95):>10.2f}°"
               if len(nav) > 50 else f"{'—':>10}    {'—':>10}    ")
    print(f"{bname:<9} {d['n']:>8,} "
          f"{ecam.mean():>7.2f}°  {eop.mean():>6.2f}°  "
          f"{len(nav):>6,} {nav_str}")


def summary_table(all_results):
  """Side-by-side MAE_cam per bucket across candidates."""
  print("\n=== Summary: MAE_blend_vs_cam per bucket (lower = closer to stock-LFA accuracy) ===")
  header = f"{'bucket':<9}"
  for entry in ALPHA_CANDIDATES:
    header += f" {entry[0]:>17}"
  print(header)
  for bname, _, _ in BUCKETS:
    row = f"{bname:<9}"
    for entry in ALPHA_CANDIDATES:
      cand_name = entry[0]
      m = all_results[cand_name].get(bname)
      if m is None or m['n'] < 500:
        row += f" {'—':>17}"
      else:
        row += f" {np.array(m['abs_err_vs_cam']).mean():>15.2f}°"
    print(row)

  print("\n=== Summary: nav-event max deviation p95 (lower = planner intent preserved) ===")
  print(header)
  for bname, _, _ in BUCKETS:
    row = f"{bname:<9}"
    for entry in ALPHA_CANDIDATES:
      cand_name = entry[0]
      m = all_results[cand_name].get(bname)
      if m is None or m['n'] < 500:
        row += f" {'—':>17}"
      else:
        nav = np.array(m['nav_event_deviations'])
        if len(nav) < 20:
          row += f" {'—':>17}"
        else:
          row += f" {np.percentile(nav,95):>15.2f}°"
    print(row)


def main():
  frames, by_route = load_frames()
  print(f"Loaded {len(frames):,} frames from {len(by_route)} routes")

  report_disagreement(frames)
  simulate_trust_q(frames)

  all_results = {}
  for entry in ALPHA_CANDIDATES:
    name, bps, alphas, nav_gate = entry
    m = simulate_alpha_candidate(frames, name, bps, alphas, nav_gate)
    all_results[name] = m
    print_alpha_result(name, bps, alphas, m)

  summary_table(all_results)


if __name__ == '__main__':
  main()
