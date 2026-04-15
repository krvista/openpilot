#!/usr/bin/env python3
"""Low-speed tick verification & stock-LFA quality comparison.

Answers two user questions using the DBC-accurate Stage 0 cache:

  1. Is the "턱턱 걸리는" low-speed tick actually eliminated by the
     81c451f hysteresis + aci_gain_ramp + lat_active-latching fixes
     plus the Stage 4 camera-reference blend? We measure tick-proxy
     metrics on four signals frame-by-frame:

        A. cam_angle       — stock LFA (camera's ADAS_StrAnglReqVal)
        B. op_curv (PRE)   — pre-81c451f raw curvature-derived angle
        C. op_hyst (81c451f) — above, but with hysteresis + aci latch
                               simulated on top (tick-level fixes)
        D. op_camref (73d87ec) — C, plus Stage 4 camera-ref blend

  2. Side-by-side steering quality: stock LFA vs op at each bucket.

Tick metrics (higher = worse):
  * |Δcmd|/10ms  p50, p95, p99  — per-frame angle step distribution
  * |Δ²cmd|/10ms² p95          — jerk (direction-change acceleration)
  * dir_reversal_Hz             — direction flips per second
  * frac_tick                   — fraction of frames with |Δ|>0.3°
                                  AND direction just reversed
  * LKAS_ANGLE_ACTIVE flip rate (for the op path only)
  * |cmd - actual| MAE          — tracking accuracy

Reads /tmp/reanalysis_dbc_cache.pkl (Stage 0). No drivelog re-parse.
"""
import os
import pickle
import sys
from collections import defaultdict, deque

import numpy as np

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')

from opendbc.car.hyundai.carcontroller import (  # noqa: E402
  CameraTrustEstimator,
  CAMREF_ALPHA_BP,
  CAMREF_ALPHA_V,
  CAMREF_NAV_DISAGREE_DEG,
  CAMREF_NAV_ALPHA_CAP,
)

STAGE0_CACHE = '/tmp/reanalysis_dbc_cache.pkl'

# Focus on low-speed buckets for tick analysis, full set for LFA comparison
LOW_SPEED_BUCKETS = [('parking', 0, 10), ('20km/h', 10, 25), ('30km/h', 25, 35)]
FULL_BUCKETS = [
  ('parking', 0, 10),
  ('20km/h', 10, 25),
  ('30km/h', 25, 35),
  ('40km/h', 35, 45),
  ('50km/h', 45, 55),
  ('60-70',  55, 75),
  ('80-90',  75, 95),
]

# ACI hysteresis constants mirror carcontroller.py
DRIVER_TORQUE_DEADZONE = 30
DRIVER_TORQUE_FULL_OVERRIDE = 150
ACI_SPEED_ZERO_MS = 1.0 / 3.6
ACI_SPEED_FULL_MS = 3.0 / 3.6
ACI_ENTER = 0.30
ACI_EXIT  = 0.05
ACI_GAIN_RAMP_TAU_FRAMES = 30.0
TICK_DELTA_THRESHOLD = 0.3   # deg per 10 ms frame


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


def bucket_of(v_kmh, buckets):
  for name, lo, hi in buckets:
    if lo <= v_kmh < hi:
      return name
  return None


def compute_signals(frames):
  """Replay hysteresis + Stage 4 blend across each route; attach to frames.

  Produces 4 command signals per frame:
    - A: cam_angle (stock LFA passthrough reference)
    - B: op_curv_raw (the `desired` from pre-fix build = actuators.steeringAngleDeg)
    - C: op_hyst (=op_curv_raw when aci_active_latched; else holds actual)
    - D: op_camref (=Stage-4 blend when latched; else holds actual)

  Per-route state carries across segments.
  """
  per_route = defaultdict(list)
  for f in frames:
    per_route[f['route']].append(f)

  enriched = []
  for route in sorted(per_route.keys()):
    route_frames = per_route[route]
    trust = CameraTrustEstimator()
    aci_latched = False
    aci_gain_ramp = 0.0
    # For C and D, when not latched, "op_hyst" holds current actual wheel
    # (mirrors carcontroller behaviour with rate_lat_active=False and
    # apply_angle_last tracking actual).

    for f in route_frames:
      v_ms = f['v_ms']
      cam_angle = f['cam_angle']
      op_curv = f['desired'] or 0.0
      actual = f['actual']
      lat_active = f['lat_active']
      pressed = f['pressed_torque']
      blinker = f['blinker']

      speed_blend = float(np.clip((v_ms - ACI_SPEED_ZERO_MS) /
                                   (ACI_SPEED_FULL_MS - ACI_SPEED_ZERO_MS), 0.0, 1.0))
      override_factor = float(np.clip(
        (pressed - DRIVER_TORQUE_DEADZONE) / (DRIVER_TORQUE_FULL_OVERRIDE - DRIVER_TORQUE_DEADZONE),
        0.0, 1.0))
      driver_torque_blend = 1.0 - override_factor
      authority = driver_torque_blend * speed_blend if lat_active else 0.0
      if blinker:
        authority *= 0.2

      # Hysteresis latch (81c451f)
      if lat_active:
        if authority >= ACI_ENTER:
          aci_latched = True
        elif authority < ACI_EXIT:
          aci_latched = False
      else:
        aci_latched = False

      # Trust estimator update
      cam_driving = (not lat_active) and (driver_torque_blend > 0.5)
      q_trust = trust.update(cam_angle, actual, cam_driving)

      # A: stock LFA reference (camera)
      cmd_A = cam_angle
      # B: pre-fix op path — pure op_curv
      cmd_B = op_curv
      # C: 81c451f op path — when not latched, hold actual (tick fix)
      cmd_C = op_curv if aci_latched else actual
      # D: 73d87ec op path — blend
      if aci_latched:
        alpha_base = float(np.interp(v_ms, CAMREF_ALPHA_BP, CAMREF_ALPHA_V))
        alpha_eff = alpha_base * q_trust
        if abs(cam_angle - op_curv) > CAMREF_NAV_DISAGREE_DEG:
          alpha_eff = min(alpha_eff, CAMREF_NAV_ALPHA_CAP)
        cmd_D = alpha_eff * cam_angle + (1.0 - alpha_eff) * op_curv
      else:
        cmd_D = actual

      f2 = dict(f)
      f2['cmd_A'] = cmd_A
      f2['cmd_B'] = cmd_B
      f2['cmd_C'] = cmd_C
      f2['cmd_D'] = cmd_D
      f2['_aci_latched'] = aci_latched
      enriched.append(f2)
  return enriched


def compute_tick_metrics(frames, signal_key, mode_filter, buckets):
  """For each bucket, compute tick-proxy metrics on `signal_key`.

  mode_filter: either a single mode string or a set; only frames where
               classify_mode(f) in mode_filter count.
  """
  if isinstance(mode_filter, str):
    mode_filter = {mode_filter}
  per_route_seg = defaultdict(list)
  for f in frames:
    if classify_mode(f) not in mode_filter:
      continue
    per_route_seg[(f['route'], f['seg'])].append(f)

  out = defaultdict(lambda: {
    'n': 0, 'deltas': [], 'jerks': [], 'dirs': [], 'tick_count': 0, 'errors_vs_actual': [],
  })

  for key, seq in per_route_seg.items():
    prev = None
    pprev = None
    prev_sign = 0
    for f in seq:
      b = bucket_of(f['v_kmh'], buckets)
      if b is None:
        prev = f
        continue
      if prev is not None:
        delta = f[signal_key] - prev[signal_key]
        out[b]['deltas'].append(abs(delta))
        sign = 1 if delta > 0 else (-1 if delta < 0 else 0)
        if prev_sign != 0 and sign != 0 and sign != prev_sign:
          out[b]['dirs'].append(1)  # reversal flag
          if abs(delta) > TICK_DELTA_THRESHOLD:
            out[b]['tick_count'] += 1
        else:
          out[b]['dirs'].append(0)
        if sign != 0:
          prev_sign = sign
        if pprev is not None:
          # jerk = 2nd difference
          jerk = (f[signal_key] - 2 * prev[signal_key] + pprev[signal_key])
          out[b]['jerks'].append(abs(jerk))
      out[b]['errors_vs_actual'].append(abs(f[signal_key] - f['actual']))
      out[b]['n'] += 1
      pprev = prev
      prev = f
  return out


def report_tick_comparison(enriched, buckets, title):
  """Four signals, all op-mode frames (A uses lfa_passthrough for stock)."""
  print(f"\n=== {title} ===")
  labels = [
    ('cmd_A_camref_stock_LFA', 'lfa_passthrough'),  # stock LFA
    ('cmd_B_op_pre81c451f',    'op'),               # current baseline
    ('cmd_C_op_hyst_81c451f',  'op'),               # hysteresis fix
    ('cmd_D_op_camref_73d87ec','op'),               # Stage 4 full
  ]
  for signal_key, mode in labels:
    sig = signal_key.split('_')[0]   # 'cmd'
    actual_sig = {'cmd_A': 'cmd_A', 'cmd_B': 'cmd_B', 'cmd_C': 'cmd_C', 'cmd_D': 'cmd_D'}[signal_key[:5]]
    result = compute_tick_metrics(enriched, actual_sig, mode, buckets)
    print(f"\n--- Signal: {signal_key}  (mode={mode}) ---")
    print(f"{'bucket':<9} {'n':>7} {'|Δ|_p50':>8} {'|Δ|_p95':>8} {'|Δ|>0.3':>9} "
          f"{'jerk_p95':>9} {'reversals/s':>13} {'tick_frac':>10} {'MAE':>6}")
    for bname, _, _ in buckets:
      d = result.get(bname)
      if not d or d['n'] < 500:
        continue
      deltas = np.array(d['deltas'])
      jerks = np.array(d['jerks'])
      dirs = np.array(d['dirs'])
      errs = np.array(d['errors_vs_actual'])
      rev_hz = dirs.sum() / (len(dirs) * 0.01) if len(dirs) else 0
      tick_frac = d['tick_count'] / d['n'] * 100 if d['n'] else 0
      over_3 = (deltas > TICK_DELTA_THRESHOLD).sum() / len(deltas) * 100 if len(deltas) else 0
      print(f"{bname:<9} {d['n']:>7,} "
            f"{np.percentile(deltas,50):>6.3f}° {np.percentile(deltas,95):>6.3f}° "
            f"{over_3:>7.1f}% "
            f"{np.percentile(jerks,95):>7.3f}° "
            f"{rev_hz:>11.1f}Hz "
            f"{tick_frac:>8.2f}% "
            f"{errs.mean():>4.2f}°")


def report_quality_vs_lfa(enriched):
  """Condensed op (all variants) vs stock LFA per bucket — quality comparison."""
  print("\n=== OP steering-quality variants vs Stock LFA (per bucket) ===")
  print("Lower is better for all metrics.  MAE is vs measured wheel angle.")
  print()
  print(f"{'bucket':<9} {'signal':<22} "
        f"{'|Δ|>0.3%':>10} {'reversals/s':>12} {'tick_frac%':>11} {'MAE':>6}")
  cases = [
    ('Stock LFA (cam)',      'cmd_A', 'lfa_passthrough'),
    ('OP pre-81c451f',       'cmd_B', 'op'),
    ('OP +hysteresis only',  'cmd_C', 'op'),
    ('OP +hysteresis +S4',   'cmd_D', 'op'),
  ]
  for bname, _, _ in FULL_BUCKETS:
    first = True
    for label, sig, mode in cases:
      res = compute_tick_metrics(enriched, sig, mode, FULL_BUCKETS)
      d = res.get(bname)
      if not d or d['n'] < 500:
        continue
      deltas = np.array(d['deltas'])
      dirs = np.array(d['dirs'])
      errs = np.array(d['errors_vs_actual'])
      rev_hz = dirs.sum() / (len(dirs) * 0.01) if len(dirs) else 0
      tick_frac = d['tick_count'] / d['n'] * 100 if d['n'] else 0
      over_3 = (deltas > TICK_DELTA_THRESHOLD).sum() / len(deltas) * 100
      bcol = bname if first else ''
      print(f"{bcol:<9} {label:<22} "
            f"{over_3:>8.2f}% {rev_hz:>10.1f}Hz {tick_frac:>9.2f}% {errs.mean():>4.2f}°")
      first = False
    print()


def report_low_speed_focus(enriched):
  """Zoomed-in low-speed tick: 0-2, 2-5, 5-10 km/h × 4 signals."""
  print("\n=== LOW-SPEED TICK ZOOM (0-2 / 2-5 / 5-10 km/h) ===")
  zoom = [('0-2 km/h', 0, 2), ('2-5 km/h', 2, 5), ('5-10 km/h', 5, 10)]
  cases = [
    ('Stock LFA', 'cmd_A', 'lfa_passthrough'),
    ('OP pre',    'cmd_B', 'op'),
    ('OP +hyst',  'cmd_C', 'op'),
    ('OP +S4',    'cmd_D', 'op'),
  ]
  print(f"{'bucket':<10} {'signal':<14} {'|Δ|>0.3%':>10} {'|Δ|>1.0%':>10} "
        f"{'reversals/s':>12} {'tick_frac%':>11}")
  for bname, _, _ in zoom:
    for label, sig, mode in cases:
      res = compute_tick_metrics(enriched, sig, mode, zoom)
      d = res.get(bname)
      if not d or d['n'] < 200:
        continue
      deltas = np.array(d['deltas'])
      dirs = np.array(d['dirs'])
      rev_hz = dirs.sum() / (len(dirs) * 0.01) if len(dirs) else 0
      tick_frac = d['tick_count'] / d['n'] * 100 if d['n'] else 0
      over_3 = (deltas > TICK_DELTA_THRESHOLD).sum() / len(deltas) * 100
      over_1 = (deltas > 1.0).sum() / len(deltas) * 100
      print(f"{bname:<10} {label:<14} {over_3:>8.2f}% {over_1:>8.2f}% "
            f"{rev_hz:>10.1f}Hz {tick_frac:>9.2f}%")
    print()


def main():
  print(f"Loading Stage 0 cache from {STAGE0_CACHE}…")
  with open(STAGE0_CACHE, 'rb') as f:
    frames, by_route = pickle.load(f)
  print(f"Loaded {len(frames):,} frames from {len(by_route)} routes")

  print("\nReplaying hysteresis + Stage 4 blend per route…")
  enriched = compute_signals(frames)
  print(f"Done. {len(enriched):,} frames enriched.")

  report_low_speed_focus(enriched)
  report_quality_vs_lfa(enriched)


if __name__ == '__main__':
  main()
