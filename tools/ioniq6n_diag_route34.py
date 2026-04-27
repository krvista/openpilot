#!/usr/bin/env python3
"""Comprehensive diagnosis for route 34 — what went wrong this time?

Checks EVERYTHING: canValid, accFaulted, cruiseState, carEvents,
controlsState, sendcan inventory, raw TCS ACCEnable, LKAS_ALT payloads
at transition points, panda faults, and HOD bypass activity.
"""
import glob
import sys
from collections import Counter, defaultdict

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

ROUTE = '00000034--73ccfeb6a4'
DRIVELOG_DIR = '/home/user/openpilot/drivelog'


def main():
  segs = sorted(glob.glob(f'{DRIVELOG_DIR}/*{ROUTE}*rlog.zst'),
                key=lambda p: int(p.split('--')[-2]))
  print(f"Route {ROUTE}: {len(segs)} segments")

  t0 = None
  # State tracking
  prev = {}
  # Sendcan inventory
  tx_addrs = Counter()
  tx_first_t = {}
  # LKAS_ALT payloads around key times
  lkas_all = []  # (rel, dat_hex)
  # TCS ACCEnable raw
  prev_acc_enable = None
  # panda faults
  panda_faults = 0
  fault_types = Counter()
  # HOD bypass (0x208)
  hod_count = 0
  # controlsState
  prev_ctrl = {}
  # carEvents
  events = []

  for p in segs:
    print(f"  scan {p.split('/')[-1]}")
    lr = LogReader(p)
    for m in lr:
      try:
        w = m.which()
      except:
        continue
      t_ns = m.logMonoTime
      if t0 is None:
        t0 = t_ns
      rel = (t_ns - t0) / 1e9

      if w == 'carState':
        cs = m.carState
        for attr, val in [
          ('canValid', cs.canValid),
          ('cruise.enabled', cs.cruiseState.enabled),
          ('cruise.available', cs.cruiseState.available),
          ('accFaulted', getattr(cs, 'accFaulted', None)),
          ('latActive', getattr(cs, 'latActive', None)),
        ]:
          if attr not in prev or prev[attr] != val:
            print(f"  t={rel:7.2f}s  {attr} -> {val}")
            prev[attr] = val

      elif w == 'controlsState':
        try:
          ctrl = m.controlsState
          for attr in ['enabled', 'active', 'alertText1', 'alertText2', 'alertType', 'alertSize']:
            val = getattr(ctrl, attr, None)
            if val and (attr not in prev_ctrl or prev_ctrl[attr] != val):
              if attr in ('alertText1', 'alertText2') and not val:
                continue
              print(f"  t={rel:7.2f}s  ctrl.{attr} -> {val}")
              prev_ctrl[attr] = val
        except:
          pass

      elif w == 'carEvents':
        for ev in m.carEvents:
          n = ev.name
          if n in ('startup', 'startupNoCar', 'startupNoControl', 'startupNoFw'):
            continue
          events.append((rel, n, ev.noEntry, ev.softDisable, ev.immediateDisable))
          print(f"  t={rel:7.2f}s  EVENT: {n}  noEntry={ev.noEntry} soft={ev.softDisable} imm={ev.immediateDisable}")

      elif w == 'sendcan':
        for c in m.sendcan:
          key = (c.src, c.address)
          tx_addrs[key] += 1
          if key not in tx_first_t:
            tx_first_t[key] = rel
          if c.address == 0x208:
            hod_count += 1
          if c.address == 0x110:
            lkas_all.append((rel, bytes(c.dat)))

      elif w == 'can':
        for c in m.can:
          if c.src == 1 and c.address == 0x175:
            dat = bytes(c.dat)
            if len(dat) > 7:
              ae = dat[7] & 0x03
              if ae != prev_acc_enable:
                print(f"  t={rel:7.2f}s  TCS.ACCEnable raw = {ae}")
                prev_acc_enable = ae

      elif w == 'pandaStates':
        for ps in m.pandaStates:
          if hasattr(ps, 'faults'):
            fl = list(ps.faults)
            if fl:
              panda_faults += 1
              for f in fl:
                fault_types[str(f)] += 1

  # Summary
  print(f"\n{'='*70}")
  print(f"=== SUMMARY ===")
  print(f"{'='*70}")
  print(f"HOD bypass (0x208) TX count: {hod_count}")
  print(f"Panda faults: {panda_faults}  types: {dict(fault_types)}")
  print(f"carEvents: {len(events)}")
  for t, n, *_ in events[:20]:
    print(f"  t={t:.2f}s  {n}")

  print(f"\n--- Sendcan summary (top 10) ---")
  for (bus, addr), cnt in sorted(tx_addrs.items(), key=lambda x: -x[1])[:10]:
    print(f"  bus {bus}  0x{addr:03x}  {cnt:>6} frames  first={tx_first_t[(bus,addr)]:.2f}s")

  # LKAS_ALT payload transitions (byte 3+ changes)
  print(f"\n--- LKAS_ALT (0x110) payload transitions (first 15) ---")
  prev_pay = None
  shown = 0
  for rel, dat in lkas_all:
    pay = dat[3:]  # skip CRC + counter
    if prev_pay is not None and pay != prev_pay:
      print(f"  t={rel:7.2f}s  {dat.hex()}")
      shown += 1
      if shown >= 15:
        break
    prev_pay = pay


if __name__ == '__main__':
  main()
