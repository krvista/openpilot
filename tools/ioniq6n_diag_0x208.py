#!/usr/bin/env python3
"""Quick diagnosis of routes 32 and 33 — what happens around ACC engage?

Check:
1. Is 0x208 being TX'd by openpilot (bus 0 or sendcan)?
2. Does canError / canValid change at the moment of ACC engage?
3. Are there any new addresses appearing on the bus at engage time?
4. What's the sequence: ACC button → latActive → 0x208 TX → fault?
"""
import glob
import sys
from collections import Counter, defaultdict

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')

from openpilot.tools.lib.logreader import LogReader

DRIVELOG_DIR = '/home/user/openpilot/drivelog'


def scan_route(route_id):
  segs = sorted(glob.glob(f'{DRIVELOG_DIR}/*{route_id}*rlog.zst'),
                key=lambda p: int(p.split('--')[-2]))
  print(f"\n{'='*70}")
  print(f"Route {route_id}: {len(segs)} segments")
  print(f"{'='*70}")

  t_start = None
  # Track sendcan for 0x208
  sendcan_0x208_count = 0
  sendcan_0x208_first_t = None
  # Track can (RX) 0x208 by bus
  rx_0x208_by_bus = Counter()
  # Track carState events
  car_state_events = []
  # Track canError
  can_errors = []
  # Track controlsState for latActive transitions
  lat_transitions = []
  prev_lat_active = None

  for p in segs[:6]:  # first 6 segs only for speed
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
          if c.address == 0x208:
            sendcan_0x208_count += 1
            if sendcan_0x208_first_t is None:
              sendcan_0x208_first_t = rel
              print(f"  *** SENDCAN 0x208 first TX at t={rel:.2f}s  bus={c.src}  dat={bytes(c.dat).hex()}")

      elif w == 'can':
        for c in m.can:
          if c.address == 0x208:
            rx_0x208_by_bus[(c.src)] += 1

      elif w == 'carState':
        cs = m.carState
        if cs.canErrorCounter > 0 or not cs.canValid:
          can_errors.append((rel, cs.canErrorCounter, cs.canValid))
          if len(can_errors) <= 5:
            print(f"  *** canError at t={rel:.2f}s  errorCounter={cs.canErrorCounter}  canValid={cs.canValid}")

      elif w == 'controlsState':
        try:
          ctrl = m.controlsState
          lat = ctrl.lateralActive if hasattr(ctrl, 'lateralActive') else None
          if lat is None:
            lat = ctrl.active  # fallback
          if prev_lat_active is not None and lat != prev_lat_active:
            lat_transitions.append((rel, lat))
            print(f"  *** latActive transition at t={rel:.2f}s  -> {lat}")
          prev_lat_active = lat
        except Exception:
          pass

      elif w == 'carEvents':
        for ev in m.carEvents:
          if ev.name not in ('startup', 'startupNoCar', 'startupNoControl',
                             'startupNoFw', 'dashcamMode', 'silentLkasLock'):
            car_state_events.append((rel, ev.name, ev.noEntry, ev.softDisable,
                                     ev.immediateDisable, ev.permanentAlert))
            if len(car_state_events) <= 20:
              print(f"  *** carEvent at t={rel:.2f}s  {ev.name}  "
                    f"noEntry={ev.noEntry} soft={ev.softDisable} imm={ev.immediateDisable}")

  print(f"\n--- Summary for {route_id} ---")
  print(f"sendcan 0x208 TX count: {sendcan_0x208_count}")
  if sendcan_0x208_first_t is not None:
    print(f"sendcan 0x208 first TX at: t={sendcan_0x208_first_t:.2f}s")
  print(f"RX 0x208 by bus: {dict(rx_0x208_by_bus)}")
  print(f"canError events: {len(can_errors)}")
  if can_errors:
    print(f"  first: t={can_errors[0][0]:.2f}s  errCnt={can_errors[0][1]}  valid={can_errors[0][2]}")
  print(f"latActive transitions: {len(lat_transitions)}")
  print(f"carEvents (non-startup): {len(car_state_events)}")
  for rel, name, *_ in car_state_events[:10]:
    print(f"  t={rel:.2f}s  {name}")


if __name__ == '__main__':
  for rid in ['00000032--a29a2fe9eb', '00000033--b1b24a0987']:
    scan_route(rid)
