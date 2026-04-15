#!/usr/bin/env python3
"""Ioniq 6 N: Stage 0 — DBC-accurate objective re-analysis.

Fixes the bugs in ioniq6n_reanalysis.py's heuristic decoders:
  * CCNC_0x161 is on bus 1 (not bus 2). Alerts were invisible.
  * ADAS_StrAnglReqVal / ADAS_ACIAnglTqRedcGainVal bit positions were
    wrong (byte 8 & byte 6 nibble). Now uses the real DBC.

Extracts per-frame data by running opendbc's CANParser against the
hyundai_canfd_generated.dbc on the raw can events in drivelogs.

Captured fields per 10 ms frame (synced on carControl):
  * carState: vEgoRaw, steeringAngleDeg, steeringTorque, steeringPressed,
              cruiseState.enabled, blinker l/r
  * carControl: latActive, actuators.steeringAngleDeg, actuators.curvature
  * CCNC_0x161 (bus 1, camera alerts): ALERTS_2/3/5, SOUNDS_2/4, LKA_ICON
  * LKAS_ALT   (bus 2, camera→ADAS):   ADAS_StrAnglReqVal, ADAS_ACIAnglTqRedcGainVal,
              LKAS_ANGLE_ACTIVE, LKA_ASSIST, LKA_ICON
  * op's own TX (bus 128 = sendcan):    same fields, shows what op commanded
"""
import os
import sys
import glob
import pickle
import time
import numpy as np
import zstandard as zstd
from collections import defaultdict, Counter

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')

from cereal import log
from opendbc.can.parser import CANParser

DRIVELOG_DIR = '/home/user/openpilot/drivelog'
CACHE = '/tmp/reanalysis_dbc_cache.pkl'
DBC = 'hyundai_canfd_generated'


def extract_frames(path):
  with open(path, 'rb') as f:
    raw = zstd.ZstdDecompressor().decompress(f.read(), max_output_size=500 * 1024 * 1024)

  # Parsers: CCNC_0x161 + CCNC_0x162 on bus 1, LKAS_ALT on bus 2 (camera ref),
  # LKAS_ALT on bus 0 is openpilot's TX echo (sendcan → bus 128 → echo bus 0).
  # We actually decode op's output from bus 128 (sendcan) directly: parse as bus 0.
  p_ccnc = CANParser(DBC, [('CCNC_0x161', 0)], 1)
  p_cam  = CANParser(DBC, [('LKAS_ALT', 0)], 2)
  p_op   = CANParser(DBC, [('LKAS_ALT', 0)], 0)   # sendcan bus will be relabeled

  latest_cs = None
  frames = []
  git_commit = None
  git_branch = None

  for msg in log.Event.read_multiple_bytes(raw):
    w = msg.which()
    if w == 'initData':
      git_commit = msg.initData.gitCommit
      git_branch = msg.initData.gitBranch
    elif w == 'carState':
      latest_cs = msg.carState
    elif w == 'can':
      msgs_b1 = [(c.address, bytes(c.dat), c.src) for c in msg.can if c.src == 1]
      msgs_b2 = [(c.address, bytes(c.dat), c.src) for c in msg.can if c.src == 2]
      if msgs_b1:
        p_ccnc.update([0, msgs_b1])
      if msgs_b2:
        p_cam.update([0, msgs_b2])
    elif w == 'sendcan':
      # op's own outbound — all addresses labelled with their target bus
      # (typically 0 for ECAN on HDA2-ALT). Re-label as bus 0 for p_op.
      msgs_send = [(c.address, bytes(c.dat), 0) for c in msg.sendcan if c.address == 0x110]
      if msgs_send:
        p_op.update([0, msgs_send])
    elif w == 'carControl' and latest_cs is not None:
      cc = msg.carControl
      v161 = p_ccnc.vl.get('CCNC_0x161', {})
      vcam = p_cam.vl.get('LKAS_ALT', {})
      vop = p_op.vl.get('LKAS_ALT', {})
      # Stage 4 diagnostics present in new logs only. Older logs fall back to 0.
      camref_alpha = 0.0
      camref_q = 0.0
      try:
        camref_alpha = float(getattr(cc.actuators, 'camrefAlpha', 0.0) or 0.0)
        camref_q = float(getattr(cc.actuators, 'camrefQTrust', 0.0) or 0.0)
      except Exception:
        pass
      frames.append({
        'v_kmh': latest_cs.vEgoRaw * 3.6,
        'v_ms': latest_cs.vEgoRaw,
        'actual': latest_cs.steeringAngleDeg,
        'desired': cc.actuators.steeringAngleDeg,
        'curvature': cc.actuators.curvature,
        'lat_active': cc.latActive,
        'cruise_on': latest_cs.cruiseState.enabled,
        'pressed_torque': abs(latest_cs.steeringTorque),
        'steering_pressed': latest_cs.steeringPressed,
        'blinker': bool(latest_cs.leftBlinker or latest_cs.rightBlinker),
        # camera (bus 2)
        'cam_angle': float(vcam.get('ADAS_StrAnglReqVal', 0.0)),
        'cam_aci_gain': float(vcam.get('ADAS_ACIAnglTqRedcGainVal', 0.0)),
        'cam_lka_active': int(vcam.get('LKAS_ANGLE_ACTIVE', 0)),
        'cam_lka_assist': int(vcam.get('LKA_ASSIST', 0)),
        # op's own TX (sendcan)
        'op_angle': float(vop.get('ADAS_StrAnglReqVal', 0.0)),
        'op_aci_gain': float(vop.get('ADAS_ACIAnglTqRedcGainVal', 0.0)),
        'op_lka_active': int(vop.get('LKAS_ANGLE_ACTIVE', 0)),
        'op_lka_assist': int(vop.get('LKA_ASSIST', 0)),
        # CCNC alerts (bus 1)
        'alert2': int(v161.get('ALERTS_2', 0)),
        'alert3': int(v161.get('ALERTS_3', 0)),
        'alert5': int(v161.get('ALERTS_5', 0)),
        'sound2': int(v161.get('SOUNDS_2', 0)),
        'sound4': int(v161.get('SOUNDS_4', 0)),
        'lka_icon': int(v161.get('LKA_ICON', 0)),
        # Stage 4 on-car diagnostics (0 on pre-Stage-4 builds)
        'camref_alpha': camref_alpha,
        'camref_q_trust': camref_q,
      })
  return frames, git_commit, git_branch


def load_or_cache(force=False):
  if os.path.exists(CACHE) and not force:
    print(f"Loading cache from {CACHE}…")
    with open(CACHE, 'rb') as f:
      return pickle.load(f)

  files = sorted(glob.glob(os.path.join(DRIVELOG_DIR, '*rlog.zst')))
  print(f"Processing {len(files)} segments…")
  all_frames = []
  by_route = defaultdict(lambda: {'segs': 0, 'commit': None, 'branch': None})
  t0 = time.time()
  for i, path in enumerate(files):
    fn = os.path.basename(path)
    parts = fn.split('--')
    route_id = parts[0].split('_')[1]
    seg = int(parts[2])
    try:
      frames, gc, gb = extract_frames(path)
    except Exception as e:
      print(f"  ERR {fn}: {e}")
      continue
    for f in frames:
      f['route'] = route_id
      f['seg'] = seg
    all_frames.extend(frames)
    by_route[route_id]['segs'] += 1
    if gc:
      by_route[route_id]['commit'] = gc[:7]
      by_route[route_id]['branch'] = gb
    if (i + 1) % 20 == 0:
      print(f"  {i+1}/{len(files)}  {len(all_frames):,} frames  {time.time()-t0:.1f}s")

  print(f"\nTotal: {len(all_frames):,} frames in {time.time()-t0:.1f}s")
  for r, info in sorted(by_route.items()):
    print(f"  Route {r}: {info['segs']} segs  commit={info['commit']}")

  with open(CACHE, 'wb') as f:
    pickle.dump((all_frames, dict(by_route)), f)
  return all_frames, dict(by_route)


BUCKETS = [
  ('parking', 0, 10),
  ('20km/h', 10, 25),
  ('30km/h', 25, 35),
  ('40km/h', 35, 45),
  ('50km/h', 45, 55),
  ('60-70', 55, 75),
  ('80-90', 75, 95),
  ('100+', 95, 200),
]


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


def report_alerts(frames):
  """Issue: takeover / HDP audible alerts on gentle corners in op."""
  print("\n=== ALERTS (CCNC_0x161 — proper DBC decode) ===")
  per_route_mode = defaultdict(lambda: defaultdict(lambda: {
      'total': 0, 'a2_1_2': 0, 'a3_11_12': 0, 'a3_17': 0, 'a5_2_5': 0
  }))
  for f in frames:
    m = classify_mode(f)
    d = per_route_mode[f['route']][m]
    d['total'] += 1
    if f['alert2'] in (1, 2):
      d['a2_1_2'] += 1            # KEEP_HANDS_ON_STEERING_WHEEL[_RED]
    if f['alert3'] in (11, 12):
      d['a3_11_12'] += 1          # HDP_DEACTIVATED_AUDIBLE / KEEP_EYES_ON_ROAD
    if f['alert3'] == 17:
      d['a3_17'] += 1             # FAULT_DAS_CLUSTER_ALERT (suppressed always)
    if f['alert5'] in (2, 5):
      d['a5_2_5'] += 1

  print(f"{'Route':<10} {'mode':<14} {'frames':>8} "
        f"{'A2{1,2}%':>9} {'A3{11,12}%':>11} {'A3{17}%':>8} {'A5{2,5}%':>9}")
  for r in sorted(per_route_mode.keys()):
    for mode in ('op', 'lfa_passthrough', 'manual'):
      d = per_route_mode[r][mode]
      if d['total'] < 500:
        continue
      print(f"{r:<10} {mode:<14} {d['total']:>8,} "
            f"{100*d['a2_1_2']/d['total']:>7.2f}% "
            f"{100*d['a3_11_12']/d['total']:>9.2f}% "
            f"{100*d['a3_17']/d['total']:>6.2f}% "
            f"{100*d['a5_2_5']/d['total']:>7.2f}%")


def report_op_vs_camera(frames):
  """Compare op's outbound command vs camera's reference (both on ADAS_StrAnglReqVal)."""
  print("\n=== OP TX vs CAMERA reference (LKAS_ALT ADAS_StrAnglReqVal, sendcan vs bus2) ===")
  g = defaultdict(list)
  for f in frames:
    m = classify_mode(f)
    if m not in ('op', 'lfa_passthrough'):
      continue
    b = bucket_of(f['v_kmh'])
    if b is None:
      continue
    g[(m, b)].append({
      'op_angle': f['op_angle'],
      'cam_angle': f['cam_angle'],
      'actual': f['actual'],
      'cam_err': abs(f['cam_angle'] - f['actual']),
      'op_err': abs(f['op_angle'] - f['actual']) if f['op_lka_active'] >= 2 else None,
      'op_aci': f['op_aci_gain'],
      'cam_aci': f['cam_aci_gain'],
      'op_active': f['op_lka_active'],
      'cam_active': f['cam_lka_active'],
    })

  print(f"{'mode':<15} {'bucket':<9} {'n':>7} {'cam_MAE':>8} {'cam_p95':>8} "
        f"{'op_MAE':>8} {'op_p95':>8} {'op_aci':>7} {'cam_aci':>8}")
  for mode in ('op', 'lfa_passthrough'):
    for name, _, _ in BUCKETS:
      rows = g.get((mode, name), [])
      if len(rows) < 500:
        continue
      cam_e = np.array([r['cam_err'] for r in rows])
      op_e = np.array([r['op_err'] for r in rows if r['op_err'] is not None])
      op_aci = np.array([r['op_aci'] for r in rows])
      cam_aci = np.array([r['cam_aci'] for r in rows])
      op_stat = (f"{op_e.mean():>6.2f}° {np.percentile(op_e,95):>6.2f}°"
                 if len(op_e) > 100 else f"{'—':>6}  {'—':>6}  ")
      print(f"{mode:<15} {name:<9} {len(rows):>7,} "
            f"{cam_e.mean():>6.2f}° {np.percentile(cam_e,95):>6.2f}°  "
            f"{op_stat}  "
            f"{op_aci.mean():>6.3f} {cam_aci.mean():>7.3f}")


def report_aci_gain_profile(frames):
  """What value does the camera actually command for ACIGain? vs what op sends?"""
  print("\n=== ADAS_ACIAnglTqRedcGainVal profile (0..1, camera vs op) ===")
  print(f"{'mode':<15} {'bucket':<9} {'n':>7} "
        f"{'cam_p50':>8} {'cam_p95':>8} {'cam_max':>8} "
        f"{'op_p50':>8} {'op_p95':>8} {'op_max':>8}")
  g = defaultdict(list)
  for f in frames:
    m = classify_mode(f)
    if m not in ('op', 'lfa_passthrough'):
      continue
    b = bucket_of(f['v_kmh'])
    if b is None:
      continue
    g[(m, b)].append((f['cam_aci_gain'], f['op_aci_gain']))
  for mode in ('op', 'lfa_passthrough'):
    for name, _, _ in BUCKETS:
      data = g.get((mode, name), [])
      if len(data) < 500:
        continue
      cam = np.array([d[0] for d in data])
      op = np.array([d[1] for d in data])
      print(f"{mode:<15} {name:<9} {len(data):>7,} "
            f"{np.percentile(cam,50):>6.3f}  {np.percentile(cam,95):>6.3f}  {cam.max():>6.3f}  "
            f"{np.percentile(op,50):>6.3f}  {np.percentile(op,95):>6.3f}  {op.max():>6.3f}")


def report_angle_active_transitions(frames):
  """Count LKAS_ANGLE_ACTIVE 1↔2 flips in op TX frames (the tick source)."""
  print("\n=== LKAS_ANGLE_ACTIVE transitions in op mode (per route) ===")
  prev_by_seg = {}
  per_route_bucket = defaultdict(lambda: defaultdict(lambda: {'total': 0, 'flips': 0}))
  for f in frames:
    key = (f['route'], f['seg'])
    p = prev_by_seg.get(key)
    prev_by_seg[key] = f
    if p is None or classify_mode(f) != 'op' or classify_mode(p) != 'op':
      continue
    b = bucket_of(f['v_kmh'])
    if b is None:
      continue
    d = per_route_bucket[f['route']][b]
    d['total'] += 1
    if f['op_lka_active'] != p['op_lka_active']:
      d['flips'] += 1
  print(f"{'Route':<10} {'bucket':<9} {'op_frames':>10} {'flips':>8} {'flip/min':>9}")
  for r in sorted(per_route_bucket.keys()):
    for name, _, _ in BUCKETS:
      d = per_route_bucket[r][name]
      if d['total'] < 500:
        continue
      # flips per minute (100 Hz → 6000 frames/min)
      flip_min = d['flips'] / (d['total'] / 6000)
      print(f"{r:<10} {name:<9} {d['total']:>10,} {d['flips']:>8} {flip_min:>7.1f}")


def report_cam_tracking_accuracy(frames):
  """Stock LFA accuracy: |cam_angle - actual_angle| (the hardware ceiling)."""
  print("\n=== Stock LFA tracking accuracy (|cam_angle − actual|) — HARDWARE CEILING ===")
  g = defaultdict(list)
  for f in frames:
    m = classify_mode(f)
    if m != 'lfa_passthrough':
      continue
    # Only count when camera is actually driving (LKAS_ANGLE_ACTIVE >= 2)
    if f['cam_lka_active'] < 2:
      continue
    b = bucket_of(f['v_kmh'])
    if b is None:
      continue
    g[b].append(abs(f['cam_angle'] - f['actual']))
  print(f"{'bucket':<9} {'n':>8} {'MAE':>7} {'p50':>6} {'p95':>6} {'p99':>6} {'max':>7}")
  for name, _, _ in BUCKETS:
    arr = np.array(g.get(name, []))
    if len(arr) < 500:
      continue
    print(f"{name:<9} {len(arr):>8,} {arr.mean():>5.2f}° "
          f"{np.median(arr):>4.2f}° {np.percentile(arr,95):>4.2f}° "
          f"{np.percentile(arr,99):>4.2f}° {arr.max():>5.1f}°")


def report_op_tracking_vs_camref(frames):
  """
  Key Stage-0 deliverable: compare three tracking errors on op-active frames.

    err_curv   = |desired (from LatControlAngle/curvature) − actual|
    err_op     = |op_angle_sent (sendcan LKAS_ALT) − actual|  (what ADAS saw)
    err_camref = |cam_angle − actual|  (what the camera would have sent)

    If err_camref is systematically smaller than err_curv,
    the master plan Stage 4 "camera-referenced feedforward" is justified.
  """
  print("\n=== Op mode: tracking-error comparison per bucket ===")
  g = defaultdict(list)
  for f in frames:
    if classify_mode(f) != 'op':
      continue
    b = bucket_of(f['v_kmh'])
    if b is None:
      continue
    err_curv = abs((f['desired'] or 0.0) - f['actual'])
    err_op = abs(f['op_angle'] - f['actual']) if f['op_lka_active'] >= 2 else None
    err_camref = abs(f['cam_angle'] - f['actual'])  # what if we'd used cam_angle
    g[b].append((err_curv, err_op, err_camref))
  print(f"{'bucket':<9} {'n':>7} "
        f"{'err_curv':>8} {'err_op':>8} {'err_camref':>11}  "
        f"{'curv p95':>9} {'op p95':>8} {'camref p95':>11}")
  for name, _, _ in BUCKETS:
    rows = g.get(name, [])
    if len(rows) < 500:
      continue
    curv = np.array([r[0] for r in rows])
    camref = np.array([r[2] for r in rows])
    op_arr = np.array([r[1] for r in rows if r[1] is not None])
    op_stat = f"{op_arr.mean():>6.2f}°" if len(op_arr) > 100 else f"{'—':>6}  "
    op_p95 = f"{np.percentile(op_arr,95):>6.2f}°" if len(op_arr) > 100 else f"{'—':>6}  "
    print(f"{name:<9} {len(rows):>7,} "
          f"{curv.mean():>6.2f}° {op_stat} {camref.mean():>9.2f}°   "
          f"{np.percentile(curv,95):>7.2f}° {op_p95} {np.percentile(camref,95):>9.2f}°")


def report_camref_blend_live(frames):
  """Stage 4 on-car diagnostics, if present (camref_alpha / camref_q_trust).

  On logs built from the Stage-4 code path (or later), these carry the
  per-frame blend state that the device actually used. Comparing this to
  the offline replay prediction is the definitive on-car validation.
  """
  print("\n=== Stage 4 on-car diagnostics (camref_alpha / q_trust) ===")
  per_bucket = defaultdict(list)
  for f in frames:
    if classify_mode(f) != 'op':
      continue
    b = bucket_of(f['v_kmh'])
    if b is None:
      continue
    if f.get('camref_alpha', 0) == 0 and f.get('camref_q_trust', 0) == 0:
      continue  # legacy logs where the fields are always zero
    per_bucket[b].append((f['camref_alpha'], f['camref_q_trust']))
  if not any(per_bucket.values()):
    print("  (no non-zero samples — pre-Stage-4 drivelog, skipping)")
    return
  print(f"{'bucket':<9} {'n':>7} {'α_p50':>6} {'α_p95':>6} {'q_p50':>6} {'q_p5':>6}")
  for name, _, _ in BUCKETS:
    data = per_bucket.get(name, [])
    if len(data) < 200:
      continue
    a = np.array([d[0] for d in data])
    q = np.array([d[1] for d in data])
    print(f"{name:<9} {len(data):>7,} "
          f"{np.percentile(a,50):>5.2f}  {np.percentile(a,95):>5.2f}  "
          f"{np.percentile(q,50):>5.2f}  {np.percentile(q,5):>5.2f}")


def main():
  frames, by_route = load_or_cache(force=('--refresh' in sys.argv))
  print(f"\nTotal frames loaded: {len(frames):,}")
  print(f"Routes: {sorted(by_route.keys())}")
  report_alerts(frames)
  report_op_vs_camera(frames)
  report_aci_gain_profile(frames)
  report_angle_active_transitions(frames)
  report_cam_tracking_accuracy(frames)
  report_op_tracking_vs_camref(frames)
  report_camref_blend_live(frames)


if __name__ == '__main__':
  main()
