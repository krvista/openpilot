#!/usr/bin/env python3
"""Probe: confirm suppress_lfa (0x362) TX dropout by counting per-segment.
If the scan reader is fine, each segment should contribute ~1200 src=0
frames (20 Hz * 60 s). If dropout is real, only early segments will have
TX and later ones will have ~0."""
import glob, sys
from collections import Counter

sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG = '/home/user/openpilot/drivelog'


def count_seg(path):
  by_src = Counter()
  n_can_msgs = 0
  try: lr = LogReader(path)
  except Exception as e:
    return {'err': str(e)}
  for m in lr:
    try: w = m.which()
    except Exception: continue
    if w == 'can':
      n_can_msgs += 1
      for c in m.can:
        if c.address == 0x362:
          by_src[c.src] += 1
  return {'total_can_msgs': n_can_msgs, 'by_src': dict(by_src)}


for route_id in ['0000002a', '0000002b']:
  paths = sorted(glob.glob(f'{DRIVELOG}/99b215d21bbf8735_{route_id}--*--rlog.zst'),
                 key=lambda p: int(p.rsplit('/',1)[-1].split('--')[2]))
  print(f'\n=== Route {route_id} ({len(paths)} segs) ===')
  for p in paths:
    segnum = int(p.rsplit('/',1)[-1].split('--')[2])
    r = count_seg(p)
    if 'err' in r:
      print(f'  seg {segnum:2d}: READ ERROR {r["err"][:60]}')
    else:
      tx = r['by_src'].get(0, 0)
      rx2 = r['by_src'].get(2, 0)
      rx128 = r['by_src'].get(128, 0)
      total_can = r['total_can_msgs']
      print(f'  seg {segnum:2d}: can_msgs={total_can:6d}  0x362 '
            f'src0(TX)={tx:5d}  src2={rx2:5d}  src128={rx128:5d}')
