#!/usr/bin/env python3
"""Lock-to-lock sweep analysis: find physical steering angle max on Ioniq 6 N.

User drove wheel left-lock-to-right-lock several times at the end of route
0000002d. We scan the last 6 segments (32-37) and report:

  * max / min / p99 abs(steeringAngleDeg) from carState
  * when each extreme occurs (timestamp within seg)
  * speed range during the sweep (should be ~0)
  * what ADAS_StrAnglReqVal the camera was commanding at the same moments
  * comparison vs current STEER_ANGLE_MAX (176.7°)
"""
import glob
import sys
import zstandard as zstd
import numpy as np

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')

from cereal import log
from opendbc.can.parser import CANParser

DRIVELOG_DIR = '/home/user/openpilot/drivelog'
ROUTE = '0000002d'
LAST_N_SEGS = 6

# Current STEER_ANGLE_MAX from values.py
STEER_ANGLE_MAX_CURRENT = 176.7


def scan_seg(path):
  """Return per-frame records with time and all angle signals."""
  with open(path, 'rb') as f:
    raw = zstd.ZstdDecompressor().decompress(f.read(), max_output_size=500 * 1024 * 1024)

  p_cam = CANParser('hyundai_canfd_generated', [('LKAS_ALT', 0)], 2)

  samples = []
  latest_cs = None
  latest_cam = 0.0
  t0 = None
  for m in log.Event.read_multiple_bytes(raw):
    w = m.which()
    if w == 'carState':
      latest_cs = m.carState
    elif w == 'can':
      msgs = [(c.address, bytes(c.dat), c.src) for c in m.can if c.src == 2]
      if msgs:
        p_cam.update([0, msgs])
        latest_cam = float(p_cam.vl.get('LKAS_ALT', {}).get('ADAS_StrAnglReqVal', 0.0))
    if latest_cs is not None and w == 'carState':
      t_ns = m.logMonoTime
      if t0 is None:
        t0 = t_ns
      samples.append({
        't_s': (t_ns - t0) / 1e9,
        'angle': float(latest_cs.steeringAngleDeg),
        'v_kmh': float(latest_cs.vEgoRaw * 3.6),
        'torque': float(latest_cs.steeringTorque),
        'cam': latest_cam,
        'pressed': bool(latest_cs.steeringPressed),
      })
  return samples


def main():
  all_segs = glob.glob(f'{DRIVELOG_DIR}/*{ROUTE}*rlog.zst')
  # sort numerically by seg index (parts[2] = seg number)
  def seg_idx(p):
    try:
      return int(p.split('--')[2])
    except (ValueError, IndexError):
      return -1
  all_segs = sorted(all_segs, key=seg_idx)
  segs = all_segs[-LAST_N_SEGS:]
  print(f"Scanning last {LAST_N_SEGS} segments of route {ROUTE} (numeric sort)")
  for s in segs:
    print(f"  {s.split('/')[-1]}")
  print()

  all_samples = []
  per_seg_stats = []
  for path in segs:
    try:
      samples = scan_seg(path)
    except Exception as e:
      print(f"  ERR {path}: {e}")
      continue
    if not samples:
      continue
    angs = np.array([s['angle'] for s in samples])
    vs = np.array([s['v_kmh'] for s in samples])
    seg_id = path.split('--')[2]
    per_seg_stats.append({
      'seg': seg_id,
      'n': len(samples),
      'min': float(angs.min()),
      'max': float(angs.max()),
      'p99abs': float(np.percentile(np.abs(angs), 99)),
      'v_max': float(vs.max()),
      'v_mean': float(vs.mean()),
    })
    all_samples.extend(samples)

  print("=== Per-segment max|angle| and speed ===")
  print(f"{'seg':<5} {'frames':>7} {'v_max':>8} {'v_mean':>8} "
        f"{'ang_min':>9} {'ang_max':>9} {'|ang|_p99':>10}")
  for s in per_seg_stats:
    print(f"{s['seg']:<5} {s['n']:>7,} {s['v_max']:>6.1f}kmh {s['v_mean']:>6.1f}kmh "
          f"{s['min']:>+7.1f}° {s['max']:>+7.1f}° {s['p99abs']:>8.1f}°")

  print(f"\nTotal samples across {LAST_N_SEGS} segs: {len(all_samples):,}")

  # Find the sweep region: typically speed ~0 and large |angle| swings
  # Scan for contiguous blocks where |angle| > 50° at v<3 km/h
  print("\n=== Lock-to-lock sweep detection (v<3 km/h, |angle|>50°) ===")
  in_sweep = False
  sweep_start = None
  sweeps = []
  for i, s in enumerate(all_samples):
    sweep_candidate = abs(s['angle']) > 50.0 and s['v_kmh'] < 3.0
    if sweep_candidate and not in_sweep:
      in_sweep = True
      sweep_start = i
    elif not sweep_candidate and in_sweep:
      in_sweep = False
      if i - sweep_start > 10:  # at least 0.1 s
        sweeps.append((sweep_start, i))
  if in_sweep:
    sweeps.append((sweep_start, len(all_samples)))

  print(f"  Detected {len(sweeps)} sweep-like regions")
  for start, end in sweeps[:20]:
    seg = all_samples[start:end]
    angs = np.array([s['angle'] for s in seg])
    cams = np.array([s['cam'] for s in seg])
    torqs = np.array([s['torque'] for s in seg])
    dur = end - start
    print(f"  frames {start:>6}..{end:>6} ({dur:>4} = {dur/100:.1f}s)  "
          f"angle {angs.min():>+7.1f}..{angs.max():>+7.1f}°  "
          f"|cam|_max {np.max(np.abs(cams)):>6.1f}°  "
          f"|torque|_max {np.max(np.abs(torqs)):>6.0f}")

  # Overall extremes across all last-N-seg samples
  all_angs = np.array([s['angle'] for s in all_samples])
  all_torqs = np.array([s['torque'] for s in all_samples])
  all_cams = np.array([s['cam'] for s in all_samples])
  all_vs = np.array([s['v_kmh'] for s in all_samples])

  print("\n=== Physical extremes (absolute) ===")
  print(f"  steeringAngleDeg  min = {all_angs.min():>+8.2f}°   max = {all_angs.max():>+8.2f}°")
  print(f"                   |p99| = {np.percentile(np.abs(all_angs), 99):.2f}°   |p999| = {np.percentile(np.abs(all_angs), 99.9):.2f}°")
  print(f"  steeringTorque    min = {all_torqs.min():>+8.0f}     max = {all_torqs.max():>+8.0f}   (driver-applied)")
  print(f"  cam_angle        min = {all_cams.min():>+8.2f}°   max = {all_cams.max():>+8.2f}°")
  print(f"  vEgoRaw          min = {all_vs.min():>+8.2f}kmh max = {all_vs.max():>+8.2f}kmh")

  # The ask: compare to STEER_ANGLE_MAX_CURRENT
  print()
  print(f"=== Current STEER_ANGLE_MAX setting = {STEER_ANGLE_MAX_CURRENT}° ===")
  max_abs = max(abs(all_angs.min()), abs(all_angs.max()))
  print(f"  Observed max |wheel angle| = {max_abs:.1f}°")
  if max_abs > STEER_ANGLE_MAX_CURRENT:
    print(f"  ⚠️  Wheel CAN exceed current limit by {max_abs - STEER_ANGLE_MAX_CURRENT:+.1f}°")
    print(f"  → STEER_ANGLE_MAX should be raised toward {int(max_abs) + 10}° to match physical EPS range")
  else:
    print(f"  ✅ Current limit covers observed max (headroom {STEER_ANGLE_MAX_CURRENT - max_abs:+.1f}°)")

  # Peak rates during sweeps
  if sweeps:
    # Find the single sweep with the biggest angle excursion
    best_range = 0
    best_sweep = None
    for start, end in sweeps:
      seg = all_samples[start:end]
      angs = np.array([s['angle'] for s in seg])
      rng = angs.max() - angs.min()
      if rng > best_range:
        best_range = rng
        best_sweep = (start, end)
    if best_sweep:
      start, end = best_sweep
      seg = all_samples[start:end]
      ts = np.array([s['t_s'] for s in seg])
      angs = np.array([s['angle'] for s in seg])
      if len(ts) > 1:
        dt = np.diff(ts)
        dangs = np.diff(angs)
        rates = np.abs(dangs / dt)
        print(f"\n=== Largest single sweep analysis ===")
        print(f"  frames {start}-{end} ({len(seg)} samples, {ts[-1]-ts[0]:.1f}s)")
        print(f"  range: {angs.min():.1f}° → {angs.max():.1f}°  (excursion {best_range:.1f}°)")
        print(f"  peak slew rate:  max = {rates.max():.1f}°/s  p95 = {np.percentile(rates, 95):.1f}°/s")
        print(f"  Human lock-to-lock: ~{best_range:.0f}° total wheel travel from stop to stop")


if __name__ == '__main__':
  main()
