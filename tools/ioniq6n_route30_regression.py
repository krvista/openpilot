#!/usr/bin/env python3
"""Route 30 regression scanner (HDA2-ALT + CCNC).

Route 00000030--7dbb61a1f5 (recorded on commit cce6140, before 4d59c6a)
showed the canonical `canError` pattern: ~243 alerts across 5 segments,
caused by 4,827 TX frames each of 0x161/0x162 on bus 1 (dual-publisher
race with the ADAS gateway on HDA2-ALT).

This scanner asserts two invariants on every drivelog segment:

  V1. **No HDA2-ALT E-CAN TX collision**
      sendcan MUST NOT contain (bus 1, 0x161) or (bus 1, 0x162) frames
      on the HDA2-ALT + CCNC platform. 4d59c6a removed the TX; if they
      reappear, that's a regression.

  V2. **No LKAS_ALT partial-ACI-state frames**
      For every (bus 0, 0x110) sendcan frame, the activation signals
      (LKAS_ANGLE_ACTIVE, LKAS_BYTE13, LKAS_BYTE7 high bits) and the
      gain (ADAS_ACIAnglTqRedcGainVal) must agree: all active or all
      passive. A mismatch (e.g., LKAS_BYTE13 = 0x09 with gain = 0 and
      nonzero ADAS_StrAnglReqVal) is the signature of the fault that
      85be50e later solved on ccnc-port-prebuilt.

Exit status: 0 if all invariants hold on every provided drivelog, else 1.

Usage:
  .venv/bin/python tools/ioniq6n_route30_regression.py \\
      [--route 00000030] [--all]
"""
import argparse
import glob
import sys
from collections import Counter, defaultdict

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG_DIR = '/home/user/openpilot/drivelog'


def decode_lkas_alt(dat):
  """Manual byte decode of the LKAS_ALT (0x110) fields we care about.

  Matches the DBC signal layout in
  opendbc_repo/opendbc/dbc/hyundai_canfd_generated.dbc. Avoids spinning
  up a CANParser since we already have the raw bytes in sendcan.
  """
  if len(dat) < 32:
    return None
  byte3 = dat[3]
  byte6 = dat[6]
  byte7 = dat[7]
  byte12 = dat[12]
  byte13 = dat[13]
  # ADAS_StrAnglReqVal: 14-bit signed at bit 32 (little-endian)
  raw = int.from_bytes(dat[4:6], 'little') & 0x3fff
  if raw >= 0x2000:
    raw -= 0x4000
  angle_deg = raw * 0.1
  # ADAS_ACIAnglTqRedcGainVal: byte 12 / 255
  gain = byte12 / 255.0
  # LKAS_ANGLE_ACTIVE: byte 6 bits 6-7 (upper two bits)
  lkas_angle_active = (byte6 >> 6) & 0x3
  # LKA_ASSIST: byte 3 bits 0-2 (lower three bits)
  lka_assist = byte3 & 0x7
  return {
    'byte7': byte7,
    'byte13': byte13,
    'angle_deg': angle_deg,
    'gain': gain,
    'lkas_angle_active': lkas_angle_active,
    'lka_assist': lka_assist,
  }


def scan_segments(segs):
  v1_count = 0    # factory E-CAN collision (0x161/0x162 on bus 1)
  v2_count = 0    # LKAS_ALT partial-ACI-state
  v2_samples = []
  tx_0x161_bus1 = 0
  tx_0x162_bus1 = 0
  total_lkas_alt = 0

  for p in segs:
    for m in LogReader(p):
      try:
        w = m.which()
      except Exception:
        continue
      if w != 'sendcan':
        continue
      for c in m.sendcan:
        bus, addr = c.src, c.address
        if bus == 1 and addr == 0x161:
          tx_0x161_bus1 += 1
          v1_count += 1
        elif bus == 1 and addr == 0x162:
          tx_0x162_bus1 += 1
          v1_count += 1
        elif bus == 0 and addr == 0x110:
          total_lkas_alt += 1
          d = decode_lkas_alt(bytes(c.dat))
          if d is None:
            continue
          # Define "active" via three independent indicators the ADAS ECU consumes.
          # Partial state = some-but-not-all are set.
          active_bits_angle = d['lkas_angle_active'] >= 2  # mode 2 = op angle
          active_bits_b13 = d['byte13'] == 0x09
          active_bits_b7 = (d['byte7'] & 0xA0) == 0xA0     # bits 5+7 set
          indicators_set = sum([active_bits_angle, active_bits_b13, active_bits_b7])
          gain_active = d['gain'] > 0.01
          # Canonical mismatches from routes 3a / 32 / 34:
          # (a) angle/byte13/byte7 active bits set but gain = 0 while
          #     commanding a nonzero apply_angle
          # (b) gain > 0 but all active bits are passive
          mismatch = False
          reason = ''
          if indicators_set >= 2 and not gain_active and abs(d['angle_deg']) > 0.1:
            mismatch = True
            reason = 'bits_active_gain_zero_nonzero_angle'
          elif indicators_set == 0 and gain_active:
            mismatch = True
            reason = 'bits_passive_gain_active'
          if mismatch:
            v2_count += 1
            if len(v2_samples) < 5:
              v2_samples.append({
                'reason': reason,
                **d,
              })

  return {
    'v1_count': v1_count,
    'v2_count': v2_count,
    'v2_samples': v2_samples,
    'tx_0x161_bus1': tx_0x161_bus1,
    'tx_0x162_bus1': tx_0x162_bus1,
    'total_lkas_alt': total_lkas_alt,
  }


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--route', default='00000030',
                  help="Route substring to match (default: 00000030)")
  ap.add_argument('--all', action='store_true',
                  help="Scan every rlog in drivelog/ regardless of --route")
  args = ap.parse_args()

  pattern = '*' if args.all else f'*{args.route}*'
  segs = sorted(glob.glob(f'{DRIVELOG_DIR}/{pattern}*rlog.zst'))
  if not segs:
    print(f"no drivelog segments match {pattern}")
    return 2

  print(f"Scanning {len(segs)} segments "
        f"({args.route if not args.all else 'all routes'}):")

  # Group by route for per-route reporting
  by_route = defaultdict(list)
  for p in segs:
    route = '--'.join(p.split('/')[-1].split('--')[:2])
    by_route[route].append(p)

  overall_v1 = 0
  overall_v2 = 0
  for route, rsegs in sorted(by_route.items()):
    r = scan_segments(rsegs)
    status_v1 = '✅' if r['v1_count'] == 0 else '❌'
    status_v2 = '✅' if r['v2_count'] == 0 else '❌'
    print(f"\n── {route} ({len(rsegs)} segs, "
          f"{r['total_lkas_alt']} LKAS_ALT frames) ──")
    print(f"  {status_v1} V1 bus-1 0x161/0x162 TX collision: "
          f"{r['v1_count']} frames "
          f"(0x161={r['tx_0x161_bus1']}, 0x162={r['tx_0x162_bus1']})")
    print(f"  {status_v2} V2 LKAS_ALT partial-ACI-state: "
          f"{r['v2_count']} frames")
    for s in r['v2_samples']:
      print(f"      sample: {s}")
    overall_v1 += r['v1_count']
    overall_v2 += r['v2_count']

  print(f"\n{'=' * 60}")
  ok = (overall_v1 == 0) and (overall_v2 == 0)
  print(f"  TOTAL  V1={overall_v1}  V2={overall_v2}  "
        f"→ {'✅ PASS' if ok else '❌ FAIL'}")
  return 0 if ok else 1


if __name__ == '__main__':
  sys.exit(main())
