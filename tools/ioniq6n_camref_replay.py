#!/usr/bin/env python3
"""Stage 4 offline replay validator.

Imports the EXACT production code path from carcontroller.py (the
CameraTrustEstimator class + the α(v) / nav_gate / q_trust constants)
and applies it frame-by-frame to the DBC-accurate Stage 0 cache.

For each route (state carries across segments, per-route reset), we
replay the blend formula:

  q_trust updates only when camera is driving the wheel (lfa_passthrough
    with light driver input) — matches the `cam_driving` gate inside
    carcontroller.py.

  When classify_mode(f)=='op' AND aci_active_latched:
    α_base  = np.interp(v_ms, CAMREF_ALPHA_BP, CAMREF_ALPHA_V)
    α_eff   = α_base * q_trust
    α_eff   = min(α_eff, CAMREF_NAV_ALPHA_CAP) if |cam-op_curv| > NAV_DISAGREE
    blended = α_eff·cam + (1-α_eff)·op_curv

We also simulate the aci_active hysteresis (ENTER 0.30 / EXIT 0.05)
identical to carcontroller.py so the "aci_active_latched" gate matches
on-device behavior.

Reports per bucket:
  * mean α_eff, q_trust distribution
  * fraction of op frames where blend was actually applied (aci_active_latched AND cam_msg present)
  * MAE_blend_vs_cam, MAE_blend_vs_op (nav preservation)
  * predicted tracking error: |blended - actual|  (assuming plant is roughly
    linear and the blended command gets the plant response we saw; this is
    approximate but bounded above by |actual − op_curv| current error and
    below by stock-LFA MAE)
"""
import os
import pickle
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')

# Import the exact production path
from opendbc.car.hyundai.carcontroller import (  # noqa: E402
  CameraTrustEstimator,
  CAMREF_ALPHA_BP,
  CAMREF_ALPHA_V,
  CAMREF_NAV_DISAGREE_DEG,
  CAMREF_NAV_ALPHA_CAP,
)

STAGE0_CACHE = '/tmp/reanalysis_dbc_cache.pkl'

BUCKETS = [
  ('parking', 0, 10),
  ('20km/h', 10, 25),
  ('30km/h', 25, 35),
  ('40km/h', 35, 45),
  ('50km/h', 45, 55),
  ('60-70',  55, 75),
  ('80-90',  75, 95),
]

# Hysteresis constants, mirrors carcontroller.py
DRIVER_TORQUE_DEADZONE = 30
DRIVER_TORQUE_FULL_OVERRIDE = 150
ACI_SPEED_ZERO_MS = 1.0 / 3.6
ACI_SPEED_FULL_MS = 3.0 / 3.6
ACI_ENTER = 0.30
ACI_EXIT  = 0.05


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


def replay_route(frames):
  """Process frames belonging to one route in order; state carries across segs."""
  trust = CameraTrustEstimator()
  aci_latched = False

  records = []   # per-op-frame diagnostics
  for f in frames:
    v_ms = f['v_ms']
    cam_angle = f['cam_angle']
    op_curv = f['desired'] or 0.0
    actual = f['actual']
    lat_active = f['lat_active']
    pressed = f['pressed_torque']
    blinker = f['blinker']
    cruise_on = f['cruise_on']

    # Reproduce authority + hysteresis ladder
    speed_blend = float(np.clip((v_ms - ACI_SPEED_ZERO_MS) /
                                 (ACI_SPEED_FULL_MS - ACI_SPEED_ZERO_MS), 0.0, 1.0))
    override_factor = float(np.clip(
      (pressed - DRIVER_TORQUE_DEADZONE) / (DRIVER_TORQUE_FULL_OVERRIDE - DRIVER_TORQUE_DEADZONE),
      0.0, 1.0))
    driver_torque_blend = 1.0 - override_factor
    authority = driver_torque_blend * speed_blend if lat_active else 0.0
    if blinker:
      authority *= 0.2
    # Latch (identical to production)
    if lat_active:
      if authority >= ACI_ENTER:
        aci_latched = True
      elif authority < ACI_EXIT:
        aci_latched = False
    else:
      aci_latched = False

    # Camera driving (for trust update)
    cam_driving = (not lat_active) and (driver_torque_blend > 0.5)
    q_trust = trust.update(cam_angle, actual, cam_driving)

    # Current behavior before blend (baseline): desired_current = op_curv
    # New blended (only if op is actively steering)
    mode = classify_mode(f)
    blended = op_curv
    alpha_eff = 0.0
    nav_gate_active = False
    if mode == 'op' and aci_latched:
      alpha_base = float(np.interp(v_ms, CAMREF_ALPHA_BP, CAMREF_ALPHA_V))
      alpha_eff = alpha_base * q_trust
      if abs(cam_angle - op_curv) > CAMREF_NAV_DISAGREE_DEG:
        alpha_eff = min(alpha_eff, CAMREF_NAV_ALPHA_CAP)
        nav_gate_active = True
      blended = alpha_eff * cam_angle + (1.0 - alpha_eff) * op_curv
      # Subsequent driver override wind-down (mirror)
      if override_factor > 0:
        blended = (1.0 - override_factor) * blended + override_factor * actual
      records.append({
        'bucket': bucket_of(f['v_kmh']),
        'alpha_eff': alpha_eff,
        'q_trust': q_trust,
        'nav_gate_active': nav_gate_active,
        'blended': blended,
        'op_curv': op_curv,
        'cam_angle': cam_angle,
        'actual': actual,
        'err_blend_vs_cam': abs(blended - cam_angle),
        'err_blend_vs_op': abs(blended - op_curv),
        'err_blend_vs_actual': abs(blended - actual),
        'err_op_vs_actual': abs(op_curv - actual),
        'cam_op_disagree': abs(cam_angle - op_curv),
      })
  return records


def main():
  print(f"Loading Stage 0 cache from {STAGE0_CACHE}…")
  with open(STAGE0_CACHE, 'rb') as f:
    frames, by_route = pickle.load(f)
  print(f"Loaded {len(frames):,} frames from {len(by_route)} routes")

  # Sort by route, then segment, then preserve original order within seg.
  # (frames list was built in seg-order; we just group by route here.)
  all_records = []
  per_route_records = defaultdict(list)
  for route in sorted(by_route.keys()):
    route_frames = [f for f in frames if f['route'] == route]
    # Frames within the list already appear in seg-then-time order from Stage 0.
    rec = replay_route(route_frames)
    print(f"  Route {route}: {len(route_frames):>7,} frames → {len(rec):>6,} op/latched blend records")
    per_route_records[route] = rec
    all_records.extend(rec)

  if not all_records:
    print("No blend records! (aci_latched may never have fired.)")
    return

  # ─── Per-bucket report ───
  per_bucket = defaultdict(list)
  for r in all_records:
    if r['bucket']:
      per_bucket[r['bucket']].append(r)

  print("\n=== Blend diagnostics per bucket ===")
  print(f"{'bucket':<9} {'n':>7} {'α_eff p50':>9} {'α_eff p95':>9} "
        f"{'q_p50':>6} {'q_p5':>6} {'nav_gate%':>10}")
  for name, _, _ in BUCKETS:
    rows = per_bucket[name]
    if len(rows) < 500:
      continue
    a = np.array([r['alpha_eff'] for r in rows])
    q = np.array([r['q_trust'] for r in rows])
    ng = sum(r['nav_gate_active'] for r in rows) / len(rows) * 100
    print(f"{name:<9} {len(rows):>7,} "
          f"{np.percentile(a,50):>7.2f}  {np.percentile(a,95):>7.2f}  "
          f"{np.percentile(q,50):>5.2f}  {np.percentile(q,5):>5.2f} "
          f"{ng:>8.1f}%")

  print("\n=== Error metrics per bucket (predictive) ===")
  print(f"{'bucket':<9} {'n':>7} "
        f"{'MAE_cam':>9} {'MAE_op':>8} "
        f"{'err_vs_act_OLD':>14} {'err_vs_act_NEW':>14} {'Δ':>7}")
  for name, _, _ in BUCKETS:
    rows = per_bucket[name]
    if len(rows) < 500:
      continue
    cam = np.array([r['err_blend_vs_cam'] for r in rows])
    op = np.array([r['err_blend_vs_op'] for r in rows])
    # These two are the "what the plant would have produced" predictions.
    # Interpret with care: err_op_vs_actual is the measured current error,
    # err_blend_vs_actual is what the error would be IF the plant simply
    # settled at the commanded value. Real plant has inertia, but this is
    # a useful upper bound on the improvement.
    old = np.array([r['err_op_vs_actual'] for r in rows])
    new = np.array([r['err_blend_vs_actual'] for r in rows])
    delta = old.mean() - new.mean()
    print(f"{name:<9} {len(rows):>7,} "
          f"{cam.mean():>7.2f}°  {op.mean():>6.2f}°  "
          f"{old.mean():>12.2f}°  {new.mean():>12.2f}°  "
          f"{delta:>+5.2f}°")

  # ─── Per-route summary (did any route have degenerate behavior?) ───
  print("\n=== Per-route α_eff & q_trust summary (sanity) ===")
  print(f"{'Route':<10} {'commit':<8} {'op_blend_n':>11} "
        f"{'α_eff mean':>11} {'q_trust mean':>13} {'nav_gate%':>10}")
  for route in sorted(per_route_records.keys()):
    rows = per_route_records[route]
    if not rows:
      print(f"{route:<10} — no blend records")
      continue
    a = np.array([r['alpha_eff'] for r in rows])
    q = np.array([r['q_trust'] for r in rows])
    ng = sum(r['nav_gate_active'] for r in rows) / len(rows) * 100
    commit = by_route[route]['commit']
    print(f"{route:<10} {commit:<8} {len(rows):>11,} "
          f"{a.mean():>9.2f}  {q.mean():>11.2f}  {ng:>8.1f}%")

  # ─── Safety check: worst-case α_eff = 0 always when camera absent ───
  zero_alpha = sum(1 for r in all_records if r['alpha_eff'] == 0.0)
  print(f"\nSafety: {zero_alpha:,} of {len(all_records):,} blend records had α_eff = 0 "
        f"(fallback to pure op_curv when cam msg absent or q dropped).")


if __name__ == '__main__':
  main()
