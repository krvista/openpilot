#!/usr/bin/env python3
"""Look at `sendcan` service — that's openpilot's actual TX stream —
instead of `can` (which logs panda-bus RX)."""
import glob, sys
from collections import Counter

sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG = '/home/user/openpilot/drivelog'
TARGETS = {0x110: 'LKAS_ALT', 0x362: 'suppress_lfa'}

for route_id in ['0000002a']:
  paths = sorted(glob.glob(f'{DRIVELOG}/99b215d21bbf8735_{route_id}--*--rlog.zst'),
                 key=lambda p: int(p.rsplit('/',1)[-1].split('--')[2]))
  print(f'\n=== Route {route_id}: sendcan-stream per segment ===')
  for p in paths[:8] + paths[-3:]:
    segnum = int(p.rsplit('/',1)[-1].split('--')[2])
    sc_cnt = 0
    by_addr = Counter()
    counter_gaps = []
    last_counter = None
    try: lr = LogReader(p)
    except Exception: continue
    for m in lr:
      try: w = m.which()
      except Exception: continue
      if w == 'sendcan':
        sc_cnt += 1
        for c in m.sendcan:
          if c.address in TARGETS:
            by_addr[c.address] += 1
            if c.address == 0x362 and len(c.dat) > 2:
              cur = c.dat[2]
              if last_counter is not None:
                diff = (cur - last_counter) & 0xFF
                if diff == 0 or diff > 2:
                  counter_gaps.append((last_counter, cur, diff))
              last_counter = cur
    print(f'  seg {segnum:3d}: sendcan_events={sc_cnt:5d}  '
          f'LKAS_ALT={by_addr[0x110]:5d}  suppress_lfa={by_addr[0x362]:5d}  '
          f'0x362_counter_gaps={len(counter_gaps)}')
    for cg in counter_gaps[:3]:
      print(f'      counter {cg[0]:#04x}->{cg[1]:#04x} Δ={cg[2]}')
