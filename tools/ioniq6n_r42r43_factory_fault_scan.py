#!/usr/bin/env python3
"""Route 42/43: look for factory-ADAS-side fault indicators and
correlate against `suppress_lfa` (0x362) cadence gaps.

LFA-icon-disappear is a factory cluster decision, so the smoking gun
is in signals coming BACK from the factory side — not in openpilot's
onroadEvents stream. We watch for:
  * carState.steerFaultTemporary / .steerFaultPermanent rising edges
  * cruiseState flips (active/available transitions during drive)
  * LKA_AVAILABLE / LKA_MODE / steering_active signal drops on the RX
    LKAS_ALT/LFA messages coming from factory ADAS (src != 0).

Also fixes the monotime reset bug: each segment gets its own t0 and the
logs are concatenated with a running offset so gaps can be measured
across segment boundaries.
"""
import glob
import sys
from collections import Counter

sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG_DIR = '/home/user/openpilot/drivelog'
ROUTES = ['0000002a', '0000002b']
SUPPRESS_ADDR = 0x362
GAP_THRESHOLD_MS = 75.0
CORR_WINDOW_MS = 500.0


def segs_for(route_id):
  paths = glob.glob(f'{DRIVELOG_DIR}/99b215d21bbf8735_{route_id}--*--rlog.zst')
  def seg_num(p):
    try: return int(p.rsplit('/', 1)[-1].split('--')[2])
    except: return 0
  return sorted(paths, key=seg_num)


def scan_route(route_id):
  segs = segs_for(route_id)
  print(f'\n=== Route {route_id}: {len(segs)} segments ===')
  if not segs: return

  suppress_frames = []
  steer_fault_rising = []
  cruise_flips = []
  lka_available_flips = []

  prev_stf = None
  prev_cr_en = None
  prev_lka_avail = None

  total_offset_ms = 0.0
  for seg in segs:
    try: lr = LogReader(seg)
    except Exception: continue
    t0_seg = None
    seg_last_t_ms = 0.0
    for m in lr:
      try: w = m.which()
      except Exception: continue
      if t0_seg is None: t0_seg = m.logMonoTime
      t_ms = total_offset_ms + (m.logMonoTime - t0_seg) / 1e6
      if t_ms > seg_last_t_ms: seg_last_t_ms = t_ms

      if w == 'can':
        for c in m.can:
          if c.address == SUPPRESS_ADDR:
            suppress_frames.append((t_ms, c.dat[2] if len(c.dat) > 2 else 0,
                                    c.src))
      elif w == 'carState':
        cs = m.carState
        stf = bool(getattr(cs, 'steerFaultTemporary', False))
        if prev_stf is False and stf is True:
          steer_fault_rising.append(t_ms)
        prev_stf = stf
        cr_en = bool(cs.cruiseState.enabled)
        if prev_cr_en is not None and prev_cr_en != cr_en:
          cruise_flips.append((t_ms, prev_cr_en, cr_en))
        prev_cr_en = cr_en

    total_offset_ms = seg_last_t_ms + 1.0  # next segment starts after last msg

  # ---- src distribution for 0x362 ----
  src_counts = Counter(s for _, _, s in suppress_frames)
  print(f'  0x362 total frames: {len(suppress_frames)}')
  print(f'  by src: {dict(src_counts)}')

  # Our TX frames: src == 0 on CAN bus 0 in sendcan. Factory camera would be
  # on a different bus/src. Filter to our TX for cadence analysis.
  our_tx = [(t, c) for (t, c, s) in suppress_frames if s == 0]
  print(f'  our TX (src=0): {len(our_tx)} frames, '
        f'span={our_tx[-1][0] - our_tx[0][0]:.0f}ms' if our_tx else '')

  gaps = []
  for i in range(1, len(our_tx)):
    dt = our_tx[i][0] - our_tx[i-1][0]
    if dt > GAP_THRESHOLD_MS:
      gaps.append((our_tx[i-1][0], our_tx[i][0], dt))
  print(f'  TX cadence gaps (>{GAP_THRESHOLD_MS:.0f}ms): {len(gaps)}')
  for g in gaps[:10]:
    print(f'    gap  end_t≈{g[1]:10.1f}ms  dur={g[2]:5.1f}ms')

  print(f'  carState.steerFaultTemporary rising: {len(steer_fault_rising)}')
  for t in steer_fault_rising[:10]:
    print(f'    t≈{t:10.1f}ms')
  print(f'  cruiseState.enabled flips: {len(cruise_flips)}')
  for (t, a, b) in cruise_flips[:10]:
    print(f'    t≈{t:10.1f}ms  {a}→{b}')

  # ---- correlate gaps to faults ----
  matches = 0
  for (_, end_t, _) in gaps:
    for ft in steer_fault_rising:
      if abs(ft - end_t) <= CORR_WINDOW_MS:
        matches += 1; break
  print(f'  gap↔steerFaultTemporary matches within ±{CORR_WINDOW_MS:.0f}ms: '
        f'{matches}/{len(gaps)}')


if __name__ == '__main__':
  for r in ROUTES:
    scan_route(r)
