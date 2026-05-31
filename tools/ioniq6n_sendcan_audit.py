#!/usr/bin/env python3
"""Offline simulation: replay drivelogs through the current carcontroller
logic and verify no dangerous sendcan patterns emerge.

Checks:
  1. 0x1A0 (SCC_CONTROL) TX count must be 0 on ccnc_lka_alt
  2. 0x110 (LKAS_ALT) must stay in passthrough when ACC is off
  3. No E-CAN (bus 1) addresses that overlap with factory ECUs

This doesn't run the full carcontroller — it replays the ACTUAL sendcan
from the logs (which reflect what the code DID generate).
"""
import glob
import sys
from collections import Counter, defaultdict

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG_DIR = '/home/user/openpilot/drivelog'

# Factory E-CAN (bus 1) addresses that must NEVER be TX'd by openpilot
# on HDA2-ALT + CCNC to avoid dual-publisher faults.
FACTORY_ECAN_CRITICAL = {
    0x1A0,  # SCC_CONTROL
    0x161,  # CCNC_0x161
    0x162,  # CCNC_0x162
    0x0EA,  # MDPS
    0x125,  # STEERING_SENSORS
    0x175,  # TCS
    0x0A0,  # WHEEL_SPEEDS
}

# Addresses openpilot is ALLOWED to TX on bus 1
ALLOWED_ECAN = {
    0x1CF,  # CRUISE_BUTTON
}

# Addresses on bus 0 (ACAN) that openpilot sends
ALLOWED_ACAN = {
    0x110,  # LKAS_ALT
    0x362,  # CAM_0x362
}


def analyze_route(route_id):
  segs = sorted(glob.glob(f'{DRIVELOG_DIR}/*{route_id}*rlog.zst'),
                key=lambda p: int(p.split('--')[-2]))
  if not segs:
    return

  print(f"\n{'='*60}")
  print(f"Route {route_id}: {len(segs)} segments")
  print(f"{'='*60}")

  t0 = None
  tx_by_bus_addr = Counter()
  violations = []
  lkas_active_while_acc_off = 0
  lkas_passthrough_count = 0
  acc_enabled = False

  for p in segs:
    lr = LogReader(p)
    for m in lr:
      try:
        w = m.which()
      except:
        continue
      if t0 is None:
        t0 = m.logMonoTime
      rel = (m.logMonoTime - t0) / 1e9

      if w == 'carState':
        cs = m.carState
        acc_enabled = cs.cruiseState.enabled

      elif w == 'sendcan':
        for c in m.sendcan:
          bus = c.src
          addr = c.address
          tx_by_bus_addr[(bus, addr)] += 1

          # Check 1: factory E-CAN address TX?
          if bus == 1 and addr in FACTORY_ECAN_CRITICAL:
            violations.append((rel, bus, addr, "FACTORY_ECAN_COLLISION"))

          # Check 2: unknown bus 1 address?
          if bus == 1 and addr not in ALLOWED_ECAN and addr not in FACTORY_ECAN_CRITICAL:
            if addr < 0x700:  # ignore UDS diagnostic
              violations.append((rel, bus, addr, "UNEXPECTED_ECAN_TX"))

          # Check 3: LKAS_ALT active bits while ACC off?
          if addr == 0x110 and bus == 0:
            dat = bytes(c.dat)
            if len(dat) >= 14:
              byte13 = dat[13]
              if byte13 == 0x09 and not acc_enabled:
                lkas_active_while_acc_off += 1
              else:
                lkas_passthrough_count += 1

  # Report
  print(f"\nSendcan inventory:")
  for (bus, addr), cnt in sorted(tx_by_bus_addr.items(), key=lambda x: -x[1]):
    if cnt < 3 and addr >= 0x700:
      continue  # skip one-off UDS
    flag = ""
    if bus == 1 and addr in FACTORY_ECAN_CRITICAL:
      flag = " *** VIOLATION: factory E-CAN collision!"
    print(f"  bus {bus}  0x{addr:03x}  {cnt:>6}{flag}")

  print(f"\nViolations: {len(violations)}")
  for t, bus, addr, kind in violations[:10]:
    print(f"  t={t:.2f}s  bus {bus} 0x{addr:03x}  {kind}")
  if len(violations) > 10:
    print(f"  ... and {len(violations) - 10} more")

  print(f"\nLKAS_ALT (0x110): active-while-ACC-off = {lkas_active_while_acc_off}, "
        f"passthrough = {lkas_passthrough_count}")

  ok = len(violations) == 0 and lkas_active_while_acc_off == 0
  print(f"\n{'✅ PASS' if ok else '❌ FAIL'}")
  return ok


if __name__ == '__main__':
  routes = set()
  for f in glob.glob(f'{DRIVELOG_DIR}/*rlog.zst'):
    parts = f.split('/')[-1].split('--')
    route_id = f"{'--'.join(parts[:2])}"
    routes.add(route_id)

  results = {}
  for rid in sorted(routes):
    short = rid.split('_')[-1]
    r = analyze_route(short)
    if r is not None:
      results[short] = r

  print(f"\n{'='*60}")
  print(f"OVERALL SUMMARY")
  print(f"{'='*60}")
  for rid, ok in results.items():
    print(f"  {rid}: {'✅ PASS' if ok else '❌ FAIL'}")
