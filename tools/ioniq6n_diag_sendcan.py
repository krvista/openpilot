#!/usr/bin/env python3
"""Full sendcan audit for routes 32 and 33 — what does panda TX?"""
import glob
import sys
from collections import Counter, defaultdict

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG_DIR = '/home/user/openpilot/drivelog'


def scan(route_id):
  segs = sorted(glob.glob(f'{DRIVELOG_DIR}/*{route_id}*rlog.zst'),
                key=lambda p: int(p.split('--')[-2]))
  print(f"\n{'='*70}")
  print(f"Route {route_id}: {len(segs)} segments")
  print(f"{'='*70}")

  t_start = None
  tx_addrs = Counter()  # (bus, addr) -> count
  tx_first = {}         # (bus, addr) -> first t_rel
  # Also track pandaState for safety mode
  safety_modes = []
  # canState errors from pandaStates
  panda_errors = []

  for p in segs[:6]:
    print(f"  scan {p.split('/')[-1]}")
    lr = LogReader(p)
    for m in lr:
      try:
        w = m.which()
      except Exception:
        continue
      t_ns = m.logMonoTime
      if t_start is None:
        t_start = t_ns
      rel = (t_ns - t_start) / 1e9

      if w == 'sendcan':
        for c in m.sendcan:
          key = (c.src, c.address)
          tx_addrs[key] += 1
          if key not in tx_first:
            tx_first[key] = rel

      elif w == 'pandaStates':
        for ps in m.pandaStates:
          if hasattr(ps, 'safetyModel'):
            safety_modes.append((rel, ps.safetyModel, getattr(ps, 'safetyParam', 0)))
          # check faults
          if hasattr(ps, 'faults'):
            faults = list(ps.faults)
            if faults:
              panda_errors.append((rel, faults))
              if len(panda_errors) <= 5:
                print(f"  *** pandaState fault at t={rel:.2f}s: {faults}")

      elif w == 'carState':
        cs = m.carState
        if not cs.canValid and rel < 10:
          pass  # already checked in previous script

  print(f"\n--- sendcan summary ---")
  for (bus, addr), cnt in sorted(tx_addrs.items(), key=lambda x: -x[1]):
    first = tx_first[(bus, addr)]
    print(f"  bus {bus}  0x{addr:03x}  {cnt:>6} frames  first_at={first:.2f}s")

  if safety_modes:
    print(f"\n--- pandaState safety modes ---")
    modes_seen = set()
    for t, model, param in safety_modes:
      key = (model, param)
      if key not in modes_seen:
        print(f"  t={t:.2f}s  safetyModel={model}  safetyParam={param}")
        modes_seen.add(key)

  if panda_errors:
    print(f"\n--- panda faults: {len(panda_errors)} events ---")
  else:
    print(f"\n--- no panda faults ---")


if __name__ == '__main__':
  for rid in ['00000032--a29a2fe9eb', '00000033--b1b24a0987']:
    scan(rid)
