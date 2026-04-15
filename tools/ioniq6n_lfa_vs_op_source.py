#!/usr/bin/env python3
"""Why is stock LFA smooth at low speed while op has tick?

Compares the raw command signals frame-by-frame at 0-10 km/h to isolate
the source of the difference. Five diagnostic angles, all from the
Stage 0 DBC cache (no simulation — pure measurement):

  1. Raw source smoothness: distribution of |Δ|/10 ms for
        - cam_angle      (what the camera commands, bus 2)
        - op_curv        (op's actuators.steeringAngleDeg, pre-fix)
        - actual wheel   (EPS feedback)
  2. Frequency content: direction-change rate per second for each signal.
  3. Plant response: does cam_angle lead or lag actual? (cross-correlation)
  4. Commanded rate: the camera's own commanded angle-rate vs op's —
     shows whose SOURCE is smoother.
  5. Activation / gain signaling: how often each signal changes the
     authority bits MDPS sees.

Purpose: the tick isn't an MDPS artifact (the same MDPS drives both
signals). Either the camera sends a smoother command, or its smoothing
pipeline differs from ours upstream.
"""
import pickle
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')

STAGE0_CACHE = '/tmp/reanalysis_dbc_cache.pkl'
LOW_SPEED_BUCKETS = [('0-2 km/h', 0, 2), ('2-5 km/h', 2, 5), ('5-10 km/h', 5, 10)]


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
  for name, lo, hi in LOW_SPEED_BUCKETS:
    if lo <= v_kmh < hi:
      return name
  return None


def per_seg_deltas(frames, field_name, mode_filter):
  """Return a list of Δ samples per bucket, segmented to avoid cross-seg jumps."""
  per_seg = defaultdict(list)
  for f in frames:
    if classify_mode(f) != mode_filter:
      continue
    key = (f['route'], f['seg'])
    per_seg[key].append(f)
  out = defaultdict(lambda: {'deltas': [], 'signs': []})
  for key, seq in per_seg.items():
    prev = None
    for f in seq:
      b = bucket_of(f['v_kmh'])
      if b is None:
        prev = f
        continue
      if prev is not None:
        d = f[field_name] - prev[field_name]
        out[b]['deltas'].append(d)
        out[b]['signs'].append(1 if d > 0 else (-1 if d < 0 else 0))
      prev = f
  return out


def smoothness_table(frames):
  print("\n=== 1. Raw command smoothness per 10 ms (low-speed, no simulation) ===")
  print("Comparing command signals frame-to-frame in their native modes.")
  print()
  cases = [
    ('cam_angle (LFA)',   'cam_angle',   'lfa_passthrough'),
    ('op_curv (pre)',     'desired',     'op'),
    ('actual wheel (LFA)','actual',      'lfa_passthrough'),
    ('actual wheel (op)', 'actual',      'op'),
  ]
  for bname, _, _ in LOW_SPEED_BUCKETS:
    print(f"--- bucket: {bname} ---")
    print(f"{'signal':<22} {'n':>8} {'|Δ|_p50':>9} {'|Δ|_p95':>9} {'|Δ|_p99':>9} "
          f"{'|Δ|>0.3%':>10} {'|Δ|>1.0%':>10} {'reversals/s':>12}")
    for label, field, mode in cases:
      res = per_seg_deltas(frames, field, mode)
      d = res.get(bname)
      if not d or len(d['deltas']) < 200:
        continue
      deltas = np.abs(np.array(d['deltas']))
      # reversal = consecutive non-zero signs differ
      signs = np.array(d['signs'])
      rev = 0
      prev_s = 0
      for s in signs:
        if prev_s != 0 and s != 0 and s != prev_s:
          rev += 1
        if s != 0:
          prev_s = s
      rev_hz = rev / (len(signs) * 0.01) if len(signs) else 0
      over_3 = (deltas > 0.3).sum() / len(deltas) * 100
      over_1 = (deltas > 1.0).sum() / len(deltas) * 100
      print(f"{label:<22} {len(deltas):>8,} "
            f"{np.percentile(deltas,50):>7.3f}° {np.percentile(deltas,95):>7.3f}° "
            f"{np.percentile(deltas,99):>7.3f}° "
            f"{over_3:>8.1f}% {over_1:>8.1f}% {rev_hz:>10.1f}Hz")
    print()


def command_vs_wheel_lag(frames):
  """Cross-correlation peak lag: cam_angle vs actual wheel response, and
  op_curv vs actual. Tells us whether each signal leads (command) or
  lags (measurement) the wheel.
  """
  print("\n=== 2. Lead/lag of command vs wheel response (0-10 km/h) ===")
  print("Cross-correlation peak: negative lag = signal LEADS wheel "
        "(it's a setpoint, wheel catches up); positive = lags wheel.")
  print()
  # Collect contiguous op and lfa_passthrough sequences per (route, seg).
  def collect(mode):
    per_seg = defaultdict(list)
    for f in frames:
      if classify_mode(f) != mode:
        continue
      if f['v_kmh'] > 10:
        continue
      per_seg[(f['route'], f['seg'])].append(f)
    return per_seg

  def lag_peak(cmd_seq, act_seq, max_lag=20):
    a = np.array(cmd_seq) - np.mean(cmd_seq)
    b = np.array(act_seq) - np.mean(act_seq)
    if np.std(a) < 1e-6 or np.std(b) < 1e-6:
      return None
    a /= np.std(a)
    b /= np.std(b)
    best_lag = 0
    best_c = -1.0
    n = len(a)
    for lag in range(-max_lag, max_lag + 1):
      if lag >= 0:
        xa = a[:n - lag]
        xb = b[lag:]
      else:
        xa = a[-lag:]
        xb = b[:n + lag]
      if len(xa) < 20:
        continue
      c = float(np.mean(xa * xb))
      if c > best_c:
        best_c = c
        best_lag = lag
    return best_lag, best_c

  for mode, cmd_field in [('lfa_passthrough', 'cam_angle'), ('op', 'desired')]:
    lags = []
    corrs = []
    for seq in collect(mode).values():
      if len(seq) < 200:
        continue
      cmd = [f[cmd_field] for f in seq]
      act = [f['actual'] for f in seq]
      res = lag_peak(cmd, act)
      if res is not None:
        lags.append(res[0])
        corrs.append(res[1])
    if lags:
      lags = np.array(lags)
      corrs = np.array(corrs)
      print(f"  {mode:<18} cmd={cmd_field:<11}  peak lag p50 = {np.median(lags):>+3.0f} frames "
            f"({np.median(lags)*10:>+4.0f} ms), mean corr = {corrs.mean():.3f}, n_segs = {len(lags)}")


def activation_signaling(frames):
  """How often does the 'active' byte change per second in each mode?"""
  print("\n=== 3. LKAS_ANGLE_ACTIVE signaling stability (0-10 km/h) ===")
  print("Flip rate tells MDPS how stable the source is (flips = re-arm events).")
  per_mode = defaultdict(lambda: defaultdict(lambda: {'flips': 0, 'n': 0}))
  prev_by_seg_mode = {}
  for f in frames:
    if f['v_kmh'] >= 10:
      continue
    mode = classify_mode(f)
    key = (f['route'], f['seg'], mode)
    prev = prev_by_seg_mode.get(key)
    prev_by_seg_mode[key] = f
    if prev is None:
      continue
    b = bucket_of(f['v_kmh'])
    if not b:
      continue
    d = per_mode[mode][b]
    d['n'] += 1
    # For lfa_passthrough, the bytes come from camera (cam_lka_active).
    # For op, from op's TX (op_lka_active).
    cur = f['cam_lka_active'] if mode == 'lfa_passthrough' else f['op_lka_active']
    prv = prev['cam_lka_active'] if mode == 'lfa_passthrough' else prev['op_lka_active']
    if cur != prv:
      d['flips'] += 1

  print(f"{'mode':<18} {'bucket':<10} {'n':>8} {'flips':>7} {'flips/min':>11}")
  for mode in ('lfa_passthrough', 'op'):
    for bname, _, _ in LOW_SPEED_BUCKETS:
      d = per_mode[mode].get(bname)
      if not d or d['n'] < 200:
        continue
      flip_min = d['flips'] / (d['n'] / 6000)
      print(f"{mode:<18} {bname:<10} {d['n']:>8,} {d['flips']:>7,} {flip_min:>9.1f}")


def aci_gain_signaling(frames):
  """What ACIGain does each source send at low speed?"""
  print("\n=== 4. ADAS_ACIAnglTqRedcGainVal at low speed (authority signal to MDPS) ===")
  per_mode = defaultdict(lambda: defaultdict(list))
  for f in frames:
    if f['v_kmh'] >= 10:
      continue
    mode = classify_mode(f)
    if mode not in ('op', 'lfa_passthrough'):
      continue
    b = bucket_of(f['v_kmh'])
    if not b:
      continue
    gain = f['op_aci_gain'] if mode == 'op' else f['cam_aci_gain']
    per_mode[mode][b].append(gain)
  print(f"{'mode':<18} {'bucket':<10} {'n':>8} {'p5':>6} {'p50':>6} {'p95':>6} {'mean':>6}")
  for mode in ('lfa_passthrough', 'op'):
    for bname, _, _ in LOW_SPEED_BUCKETS:
      arr = np.array(per_mode[mode].get(bname, []))
      if len(arr) < 200:
        continue
      print(f"{mode:<18} {bname:<10} {len(arr):>8,} "
            f"{np.percentile(arr,5):>5.3f} {np.percentile(arr,50):>5.3f} "
            f"{np.percentile(arr,95):>5.3f} {arr.mean():>5.3f}")


def frequency_content(frames):
  """Compare the high-frequency content of cam_angle vs op_curv (low-speed)."""
  print("\n=== 5. High-frequency content (power in 10-50 Hz band) ===")
  print("Rough spectral energy estimate: sum of squared 2nd differences")
  print("  (proxy for ≥10 Hz content since at 100 Hz sample rate the Nyquist is 50 Hz).")
  cases = [
    ('cam_angle', 'cam_angle', 'lfa_passthrough'),
    ('op_curv',   'desired',   'op'),
    ('actual (LFA)',  'actual', 'lfa_passthrough'),
    ('actual (op)',   'actual', 'op'),
  ]
  per = defaultdict(lambda: defaultdict(list))
  for label, field, mode in cases:
    per_seg = defaultdict(list)
    for f in frames:
      if classify_mode(f) != mode or f['v_kmh'] >= 10:
        continue
      per_seg[(f['route'], f['seg'])].append(f)
    for seq in per_seg.values():
      if len(seq) < 20:
        continue
      ys = np.array([f[field] for f in seq])
      d2 = np.diff(ys, 2)    # 2nd difference ~ high freq energy
      b = bucket_of(seq[0]['v_kmh']) or 'other'
      per[label][b].extend(d2.tolist())
  print(f"{'signal':<16} {'bucket':<10} {'n':>8} {'rms_d2 (°)':>10}")
  for label, _, _ in cases:
    for bname, _, _ in LOW_SPEED_BUCKETS:
      arr = np.array(per[label].get(bname, []))
      if len(arr) < 200:
        continue
      rms = float(np.sqrt(np.mean(arr ** 2)))
      print(f"{label:<16} {bname:<10} {len(arr):>8,} {rms:>8.4f}")


def main():
  print(f"Loading {STAGE0_CACHE}…")
  with open(STAGE0_CACHE, 'rb') as f:
    frames, by_route = pickle.load(f)
  print(f"{len(frames):,} frames")
  smoothness_table(frames)
  command_vs_wheel_lag(frames)
  activation_signaling(frames)
  aci_gain_signaling(frames)
  frequency_content(frames)


if __name__ == '__main__':
  main()
