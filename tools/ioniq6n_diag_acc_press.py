#!/usr/bin/env python3
"""Track exactly what happens when ACC is pressed — routes 32/33."""
import glob
import sys
from collections import Counter

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
  # Track cruise state
  prev_cruise_enabled = None
  prev_cruise_available = None
  # Track ALL carEvents
  all_events = []
  # Track controlsState
  prev_enabled = None
  prev_active = None
  # Track accFaulted
  prev_acc_faulted = None
  # Track 0x110 (LKAS_ALT) sendcan — first few payloads
  lkas_first_payloads = []
  # Track carState button events
  btn_events = []

  for p in segs[:6]:
    print(f"  scan {p.split('/')[-1]}")
    lr = LogReader(p)
    for m in lr:
      try:
        w = m.which()
      except:
        continue
      t_ns = m.logMonoTime
      if t_start is None:
        t_start = t_ns
      rel = (t_ns - t_start) / 1e9

      if w == 'carState':
        cs = m.carState
        ce = cs.cruiseState.enabled
        ca = cs.cruiseState.available
        af = cs.accFaulted if hasattr(cs, 'accFaulted') else None
        if ce != prev_cruise_enabled:
          print(f"  t={rel:7.2f}s  cruiseState.enabled: {prev_cruise_enabled} -> {ce}")
          prev_cruise_enabled = ce
        if ca != prev_cruise_available:
          print(f"  t={rel:7.2f}s  cruiseState.available: {prev_cruise_available} -> {ca}")
          prev_cruise_available = ca
        if af != prev_acc_faulted:
          print(f"  t={rel:7.2f}s  accFaulted: {prev_acc_faulted} -> {af}")
          prev_acc_faulted = af
        for be in cs.buttonEvents:
          print(f"  t={rel:7.2f}s  buttonEvent: type={be.type}  pressed={be.pressed}")

      elif w == 'carEvents':
        for ev in m.carEvents:
          n = ev.name
          if n in ('startup', 'startupNoCar', 'startupNoControl', 'startupNoFw'): continue
          all_events.append((rel, n))
          print(f"  t={rel:7.2f}s  carEvent: {n}  noEntry={ev.noEntry} soft={ev.softDisable} imm={ev.immediateDisable} perm={ev.permanentAlert}")

      elif w == 'controlsState':
        try:
          cs = m.controlsState
          en = cs.enabled
          act = cs.active if hasattr(cs, 'active') else None
          if en != prev_enabled:
            print(f"  t={rel:7.2f}s  controlsState.enabled: {prev_enabled} -> {en}")
            prev_enabled = en
          if act != prev_active:
            print(f"  t={rel:7.2f}s  controlsState.active: {prev_active} -> {act}")
            prev_active = act
        except:
          pass

      elif w == 'sendcan':
        for c in m.sendcan:
          if c.address == 0x110 and len(lkas_first_payloads) < 5:
            lkas_first_payloads.append((rel, c.src, bytes(c.dat).hex()[:40]))

  print(f"\n--- First 5 LKAS_ALT sendcan payloads ---")
  for t, bus, h in lkas_first_payloads:
    print(f"  t={t:.2f}s  bus={bus}  {h}")
  print(f"\nAll carEvents: {len(all_events)}")
  evnames = Counter(n for _, n in all_events)
  for n, c in evnames.most_common(10):
    print(f"  {n}: {c}")


if __name__ == '__main__':
  for rid in ['00000032--a29a2fe9eb', '00000033--b1b24a0987']:
    scan(rid)
