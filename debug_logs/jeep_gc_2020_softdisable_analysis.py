#!/usr/bin/env python3
"""Read-only sweep of transient softDisable events: commIssue,
commIssueAvgFreq, posenetInvalid, locationdTemporaryError,
paramsdTemporaryError.

Used to validate Fix D (PUBLISHER_WARMUP_GRACE=20s in selfdrived.py).
Reports rising-edge counts split into boot (t<10s) / mid (t>=10s),
per-segment distribution, cooccurrence patterns, and timing percentiles.
Re-run after a drive with Fix D applied — expect counts in seg=0 to
drop to 0 (the gate is in selfdrived.py:394+413, gating the event add
until PUBLISHER_WARMUP_GRACE seconds of selfdrived uptime).

Usage:
  PYTHONPATH=/home/user/openpilot \
    python3 debug_logs/jeep_gc_2020_softdisable_analysis.py \
      --drivelog-dir /tmp/wk2-data/drivelog
"""
import argparse
import glob
import os
import re
import sys
from collections import Counter, defaultdict

sys.modules['smbus2'] = type(sys)('smbus2')
sys.modules['smbus2'].SMBus = object
sys.modules['serial'] = type(sys)('serial')
from openpilot.tools.lib.logreader import _LogFileReader

TARGETS = {'commIssue', 'commIssueAvgFreq', 'posenetInvalid',
           'locationdTemporaryError', 'paramsdTemporaryError'}
NAME_RE = re.compile(r'([0-9a-f]{16})_([0-9a-f]+)--([0-9a-f]+)--(\d+)--rlog\.zst$')
BUCKETS = [0, 5, 10, 15, 20, 25, 30, 40, 50, 60, 90, 120, 200]
BOOT_CUTOFF = 10.0  # Fix B boot_grace boundary (for reference)


def percentile(xs, p):
  if not xs:
    return 0.0
  xs = sorted(xs)
  return xs[int(round((len(xs) - 1) * p))]


def process(paths):
  rises = defaultdict(list)            # name -> [(route, seg, t)]
  cooc = Counter()                      # frozenset(target_names) -> count
  seg_dist = defaultdict(Counter)       # name -> Counter({seg_index: count})
  inputs_ok_false = defaultdict(list)   # route -> [(seg, t)]
  posenet_ok_false = defaultdict(list)
  liveparams_invalid = defaultdict(list)

  for i, path in enumerate(paths):
    m = NAME_RE.search(path)
    if not m:
      continue
    route_hex, seg = m.group(2), int(m.group(4))
    if i % 25 == 0:
      print(f'  {i}/{len(paths)}', file=sys.stderr)
    try:
      lr = _LogFileReader(path)
    except Exception:
      continue
    t0 = None
    prev_evts = set()
    for msg in lr:
      try:
        t_ns = msg.logMonoTime
      except Exception:
        continue
      if t0 is None:
        t0 = t_ns
      t = (t_ns - t0) / 1e9
      which = msg.which()

      if which in ('onroadEvents', 'onroadEventsSP'):
        try:
          ev_list = msg.onroadEvents if which == 'onroadEvents' else msg.onroadEventsSP.events
        except Exception:
          continue
        cur = set()
        for ev in ev_list:
          try:
            cur.add(str(ev.name))
          except Exception:
            continue
        rising = (cur - prev_evts) & TARGETS
        for nm in rising:
          rises[nm].append((route_hex, seg, t))
          seg_dist[nm][seg] += 1
        if len(rising) >= 2:
          cooc[frozenset(rising)] += 1
        prev_evts = cur

      elif which == 'livePose':
        try:
          lp = msg.livePose
          if t > 0:
            if not lp.posenetOK:
              posenet_ok_false[route_hex].append((seg, t))
            if not lp.inputsOK:
              inputs_ok_false[route_hex].append((seg, t))
        except Exception:
          pass
      elif which == 'liveParameters':
        try:
          if t > 0 and not msg.liveParameters.valid:
            liveparams_invalid[route_hex].append((seg, t))
        except Exception:
          pass

  return rises, cooc, seg_dist, inputs_ok_false, posenet_ok_false, liveparams_invalid


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--drivelog-dir', required=True)
  ap.add_argument('--limit', type=int, default=0)
  args = ap.parse_args()

  paths = sorted(glob.glob(os.path.join(args.drivelog_dir, '*--rlog.zst')))
  if args.limit:
    paths = paths[:args.limit]
  print(f'Found {len(paths)} rlog files', file=sys.stderr)

  rises, cooc, seg_dist, inputs_ok_false, posenet_ok_false, liveparams_invalid = process(paths)

  # =========================================================================
  # Rising-edge counts: boot vs mid vs seg=0 vs seg>=1
  # =========================================================================
  print('=' * 110)
  print('Rising-edge counts per event (boot vs mid-drive; seg=0 vs seg>=1)')
  print('=' * 110)
  hdr = f'{"event":<28} {"total":>6} {"boot<10s":>9} {"mid>=10s":>9} {"seg=0":>6} {"seg>=1":>7}'
  print(hdr)
  for nm in sorted(TARGETS):
    items = rises[nm]
    total = len(items)
    boot = sum(1 for (_, _, t) in items if t < BOOT_CUTOFF)
    mid = total - boot
    seg0 = sum(1 for (_, s, _) in items if s == 0)
    segp = total - seg0
    print(f'{nm:<28} {total:>6} {boot:>9} {mid:>9} {seg0:>6} {segp:>7}')

  # =========================================================================
  # Per-seg distribution
  # =========================================================================
  print()
  print('=' * 110)
  print('Per-segment distribution (top 5 segs per event)')
  print('=' * 110)
  for nm in sorted(TARGETS):
    top = seg_dist[nm].most_common(5)
    print(f'  {nm:<28} {top}')

  # =========================================================================
  # Timing histogram inside seg=0 (rise time since seg start)
  # =========================================================================
  print()
  print('=' * 110)
  print('Rise-time histogram inside seg=0 (seconds since seg start)')
  print('=' * 110)
  bucket_hdr = ['event'] + [f'<{BUCKETS[i+1]}' for i in range(len(BUCKETS) - 1)]
  print(' '.join(f'{h:>10}' for h in bucket_hdr))
  for nm in sorted(TARGETS):
    h = Counter()
    for (_, s, t) in rises[nm]:
      if s != 0:
        continue
      for i in range(len(BUCKETS) - 1):
        if BUCKETS[i] <= t < BUCKETS[i + 1]:
          h[BUCKETS[i]] += 1
          break
    row = [nm] + [str(h.get(BUCKETS[i], 0)) for i in range(len(BUCKETS) - 1)]
    print(' '.join(f'{x:>10}' for x in row))

  # =========================================================================
  # Percentiles per event (seg=0 rise times)
  # =========================================================================
  print()
  print('=' * 110)
  print('Rise-time percentiles in seg=0')
  print('=' * 110)
  for nm in sorted(TARGETS):
    rt = [t for (_, s, t) in rises[nm] if s == 0]
    if rt:
      print(f'  {nm:<28} n={len(rt):>3}  min={min(rt):.1f}s  p25={percentile(rt, .25):.1f}s  '
            f'median={percentile(rt, .5):.1f}s  p75={percentile(rt, .75):.1f}s  max={max(rt):.1f}s')

  # =========================================================================
  # Cooccurrence (target events rising together in same frame)
  # =========================================================================
  print()
  print('=' * 110)
  print('Cooccurrence (>=2 target events rising in same frame)')
  print('=' * 110)
  for k, v in cooc.most_common(10):
    print(f'  {sorted(k)}: {v}')

  # =========================================================================
  # Underlying publisher-validity flags (root-cause indicators)
  # =========================================================================
  print()
  print('=' * 110)
  print('Publisher validity sample counts (per route)')
  print('=' * 110)
  for label, dct in (('livePose.posenetOK=False', posenet_ok_false),
                     ('livePose.inputsOK=False', inputs_ok_false),
                     ('liveParameters.valid=False', liveparams_invalid)):
    if not dct:
      print(f'  {label}: (none)')
      continue
    print(f'  {label}:')
    for r, lst in dct.items():
      boot = sum(1 for (s, t) in lst if t < BOOT_CUTOFF)
      mid = sum(1 for (s, t) in lst if t >= BOOT_CUTOFF)
      seg0 = sum(1 for (s, _) in lst if s == 0)
      print(f'    route {r}: total={len(lst)}  boot={boot}  mid={mid}  seg0={seg0}')

  # =========================================================================
  # Fix D acceptance criterion
  # =========================================================================
  print()
  print('=' * 110)
  print('Fix D acceptance check (post-build re-sweep)')
  print('=' * 110)
  total_target = sum(len(rises[nm]) for nm in TARGETS)
  seg0_total = sum(sum(1 for (_, s, _) in rises[nm] if s == 0) for nm in TARGETS)
  print(f'  Total target rising-edges:       {total_target}')
  print(f'  Of which in seg=0:               {seg0_total}')
  print(f'  PUBLISHER_WARMUP_GRACE = 20.0s   (selfdrived.py near line 50)')
  print(f'  Pass after fix: seg=0 count drops to 0;  seg>=1 count unchanged.')


if __name__ == '__main__':
  main()
