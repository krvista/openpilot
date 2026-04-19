#!/usr/bin/env python3
"""Per-segment count of ALL openpilot TX addresses (src=0) to see if
the TX dropout is suppress_lfa-only or affects the whole TX pipeline."""
import glob, sys
from collections import Counter

sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG = '/home/user/openpilot/drivelog'
KEY_ADDRS = {
  0x50:  'LFA',
  0x110: 'LKAS_ALT',
  0x12a: 'LFA_alt',
  0x161: 'CCNC_0x161',
  0x162: 'CCNC_0x162',
  0x362: 'suppress_lfa',
  0x1cf: 'CRUISE_BUTTONS',
}

for route_id in ['0000002a']:
  paths = sorted(glob.glob(f'{DRIVELOG}/99b215d21bbf8735_{route_id}--*--rlog.zst'),
                 key=lambda p: int(p.rsplit('/',1)[-1].split('--')[2]))
  print(f'\n=== Route {route_id}: TX-by-addr per segment ===')
  print(f'  {"seg":>3} ' + ' '.join(f'{n:>9}' for n in KEY_ADDRS.values()))
  for p in paths:
    segnum = int(p.rsplit('/',1)[-1].split('--')[2])
    tx = Counter()
    try: lr = LogReader(p)
    except Exception: continue
    for m in lr:
      try: w = m.which()
      except Exception: continue
      if w == 'can':
        for c in m.can:
          if c.src == 0 and c.address in KEY_ADDRS:
            tx[c.address] += 1
    print(f'  {segnum:>3} ' + ' '.join(f'{tx[a]:>9d}' for a in KEY_ADDRS))
