#!/usr/bin/env python3
"""Check whether newer-build drivelog segments (0x2e, 0x30) still show
the TX dropout pattern seen on r42/r43."""
import glob, sys
from collections import Counter

sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG = '/home/user/openpilot/drivelog'

for route_id in ['0000002e', '00000030']:
  paths = sorted(glob.glob(f'{DRIVELOG}/99b215d21bbf8735_{route_id}--*--rlog.zst'),
                 key=lambda p: int(p.rsplit('/',1)[-1].split('--')[2]))
  if not paths: continue
  print(f'\n=== Route {route_id} ({len(paths)} segs) ===')

  # Get build
  try:
    for m in LogReader(paths[0]):
      try:
        if m.which() == 'initData':
          print(f'  version: {m.initData.version}  commit: {str(m.initData.gitCommit)[:10]}')
          break
      except Exception: continue
  except Exception: pass

  # Count TX per segment (only sample first 10 and last 5)
  sample = paths[:10] + (paths[-5:] if len(paths) > 15 else [])
  for p in sample:
    segnum = int(p.rsplit('/',1)[-1].split('--')[2])
    tx = Counter()
    try: lr = LogReader(p)
    except Exception: continue
    for m in lr:
      try: w = m.which()
      except Exception: continue
      if w == 'can':
        for c in m.can:
          if c.src == 0 and c.address in (0x110, 0x362, 0x50, 0x12a):
            tx[c.address] += 1
    print(f'  seg {segnum:3d}: LFA={tx[0x50]:5d}  LKAS_ALT={tx[0x110]:5d}  '
          f'LFA_alt={tx[0x12a]:5d}  suppress_lfa={tx[0x362]:5d}')
