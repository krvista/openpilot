#!/usr/bin/env python3
"""Find the HOD (hands-on detection) message on Ioniq 6 N.

User confirmed the car has a capacitive HOD sensor — touching the wheel
clears the hands-off warning. The DBC's HOD_FD_01_100ms (0x2AF / 687)
is absent from all buses in route 00000030, so this generation must
use a different address.

Strategy:
  1. Build a full per-bus address rate map from all 5 segments.
  2. Mark each address as either "known in DBC" or "UNKNOWN".
  3. Focus on unknown addresses on bus 1 (E-CAN) with frame counts
     consistent with 10–25 Hz (HOD-like publishing rate).
  4. For each candidate, compute payload-change rate per byte position
     and per bit to find signals that toggle slowly (HOD sensor values
     change with hand grip events, typically on the order of seconds).
  5. Also scan KNOWN addresses that might have HOD bits embedded
     (MDPS 0xEA, STEERING_SENSORS 0x125, CCNC_0x161, LFA 0x12A,
     FR_CMR messages) and print byte-level entropy.
"""
import glob
import sys
from collections import Counter, defaultdict

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')

from openpilot.tools.lib.logreader import LogReader

ROUTE = '00000030'
DRIVELOG_DIR = '/home/user/openpilot/drivelog'

# Known addresses from DBC (hyundai_canfd_generated.dbc) — any bus 1
# address NOT in this set is a candidate for "undocumented" HOD.
KNOWN_ADDRS = {
  0x000, 0x035, 0x04a, 0x050, 0x053, 0x060, 0x064, 0x065, 0x069, 0x06f,
  0x070, 0x0a0, 0x0cb, 0x0ea, 0x0f5, 0x100, 0x105, 0x10a, 0x10b, 0x110,
  0x11a, 0x120, 0x125, 0x12a, 0x130, 0x145, 0x155, 0x15b, 0x160, 0x161,
  0x162, 0x165, 0x16a, 0x170, 0x175, 0x17a, 0x180, 0x185, 0x18b, 0x18c,
  0x18d, 0x19a, 0x1a0, 0x1aa, 0x1b0, 0x1ca, 0x1cf, 0x1d0, 0x1da, 0x1e0,
  0x1f0, 0x200, 0x201, 0x202, 0x210, 0x211, 0x212, 0x213, 0x214, 0x215,
  0x216, 0x217, 0x218, 0x219, 0x21a, 0x21b, 0x21c, 0x21d, 0x21e, 0x21f,
  0x240, 0x251, 0x2a2, 0x2a3, 0x2a4, 0x2af, 0x2ba, 0x2bb, 0x2bc, 0x2bd,
  0x2be, 0x36a, 0x1ea, 0x362,
}

# Key addresses to byte-level dump regardless of coverage
KEY_DUMP = [0x0ea, 0x125, 0x161, 0x162, 0x12a, 0x11a, 0x180, 0x185, 0x170,
            0x175, 0x16a, 0x19a]


def main():
  segs = sorted(glob.glob(f'{DRIVELOG_DIR}/*{ROUTE}*rlog.zst'))
  print(f"Route {ROUTE}: {len(segs)} segments")

  rate = Counter()            # (bus, addr) -> frame count
  last_payload = {}
  change_counts = Counter()   # (bus, addr) -> payload-change count
  byte_variance = defaultdict(lambda: defaultdict(set))  # [(bus,addr)][i] = set of bytes
  bit_transitions = defaultdict(lambda: defaultdict(int))  # [(bus,addr)][bit_idx] = transitions
  last_bits = {}
  dlc_map = {}

  for p in segs:
    print(f"  scan {p.split('/')[-1]}")
    lr = LogReader(p)
    for m in lr:
      try:
        if m.which() != 'can':
          continue
      except Exception:
        continue
      for c in m.can:
        src = c.src
        addr = c.address
        dat = bytes(c.dat)
        key = (src, addr)
        rate[key] += 1
        dlc_map[key] = len(dat)
        prev = last_payload.get(key)
        if prev is not None and prev != dat:
          change_counts[key] += 1
        last_payload[key] = dat
        for i, b in enumerate(dat):
          byte_variance[key][i].add(b)
        bits = int.from_bytes(dat, 'little')
        prev_bits = last_bits.get(key)
        if prev_bits is not None:
          xor = bits ^ prev_bits
          bl = len(dat) * 8
          for bit_idx in range(bl):
            if xor & (1 << bit_idx):
              bit_transitions[key][bit_idx] += 1
        last_bits[key] = bits

  # Report rates + unknowns on bus 1
  print(f"\n{'='*74}")
  print(f"=== Bus 1 address rate table (all addresses) ===")
  print(f"{'='*74}")
  bus1_addrs = sorted([a for (s, a) in rate if s == 1], key=lambda a: -rate[(1, a)])
  print(f"{'addr':>6} {'frames':>7} {'est Hz':>8} {'dlc':>4} {'chg':>6} {'known':>6}")
  for addr in bus1_addrs:
    n = rate[(1, addr)]
    est_hz = n / 240.0  # ~240 s of log
    chg = change_counts[(1, addr)]
    known = 'Y' if addr in KNOWN_ADDRS else 'NEW'
    print(f"  0x{addr:03x} {n:>7} {est_hz:>6.1f}Hz {dlc_map.get((1,addr),0):>4} {chg:>6}  {known}")

  # Focus: HOD-like candidates = bus 1 AND unknown AND rate in [5,30] Hz
  print(f"\n{'='*74}")
  print(f"=== HOD candidates: bus 1, UNKNOWN, 5–30 Hz ===")
  print(f"{'='*74}")
  candidates = []
  for addr in bus1_addrs:
    n = rate[(1, addr)]
    est_hz = n / 240.0
    if addr not in KNOWN_ADDRS and 5 <= est_hz <= 30:
      candidates.append((addr, n, est_hz))
  if not candidates:
    print("  (none in 5–30 Hz band)")
  for addr, n, hz in candidates:
    print(f"  0x{addr:03x}  {n} frames  ~{hz:.1f} Hz  dlc={dlc_map.get((1,addr),0)}")

  # Expand search: any UNKNOWN bus 1 address with SOME payload changes
  # (excludes static config broadcasts)
  print(f"\n{'='*74}")
  print(f"=== All UNKNOWN bus 1 addresses with time-varying payload ===")
  print(f"{'='*74}")
  print(f"{'addr':>6} {'frames':>7} {'est Hz':>8} {'dlc':>4} {'chg':>7} {'byte_entropy (distinct vals per byte)'}")
  for addr in bus1_addrs:
    if addr in KNOWN_ADDRS:
      continue
    n = rate[(1, addr)]
    chg = change_counts[(1, addr)]
    if chg < 5:
      continue
    est_hz = n / 240.0
    dlc = dlc_map.get((1, addr), 0)
    ent = [len(byte_variance[(1, addr)][i]) for i in range(dlc)]
    print(f"  0x{addr:03x} {n:>7} {est_hz:>6.1f}Hz {dlc:>4} {chg:>7}  {ent}")

  # Also bit-transition ranking for each UNKNOWN candidate — HOD bits
  # toggle slowly (hand on/off), not every frame. We want bits with
  # small but nonzero transition counts.
  print(f"\n{'='*74}")
  print(f"=== Slow-toggling bits (candidate HOD sensor bits) ===")
  print(f"    Looking for bits with 2–200 transitions in ~240 s of log")
  print(f"    — i.e. flips with 1–100 s dwell time, consistent with")
  print(f"    intermittent hand grip events.")
  print(f"{'='*74}")
  for addr in bus1_addrs:
    if addr in KNOWN_ADDRS:
      continue
    trans = bit_transitions[(1, addr)]
    if not trans:
      continue
    slow_bits = [(bit, cnt) for bit, cnt in trans.items() if 2 <= cnt <= 200]
    if not slow_bits:
      continue
    slow_bits.sort(key=lambda x: x[1])
    print(f"\n  0x{addr:03x}  (total frames {rate[(1,addr)]}):")
    for bit, cnt in slow_bits[:12]:
      byte_idx = bit // 8
      bit_in_byte = bit % 8
      print(f"    bit {bit:>3} (byte {byte_idx}, bit {bit_in_byte}):  {cnt} transitions")

  # Also scan KNOWN addresses for SLOW bits — HOD might be hidden in
  # MDPS or STEERING_SENSORS as unused bits
  print(f"\n{'='*74}")
  print(f"=== Slow-toggling bits in KEY KNOWN addresses ===")
  print(f"    (HOD may be embedded as unused bits inside these)")
  print(f"{'='*74}")
  for addr in KEY_DUMP:
    key = (1, addr)
    if key not in rate:
      continue
    trans = bit_transitions[key]
    if not trans:
      continue
    slow_bits = [(bit, cnt) for bit, cnt in trans.items() if 2 <= cnt <= 200]
    if not slow_bits:
      print(f"\n  0x{addr:03x}: no slow bits (all frames change or no change)")
      continue
    slow_bits.sort(key=lambda x: x[1])
    print(f"\n  0x{addr:03x}  (total frames {rate[key]}):")
    for bit, cnt in slow_bits[:12]:
      byte_idx = bit // 8
      bit_in_byte = bit % 8
      print(f"    bit {bit:>3} (byte {byte_idx}, bit {bit_in_byte}):  {cnt} transitions")

  # Bus 2 / bus 0 totals summary
  print(f"\n{'='*74}")
  print(f"=== Unknown-address summary across other buses ===")
  print(f"{'='*74}")
  for src in (0, 2, 128, 129, 130, 193):
    unk = [(addr, rate[(src, addr)]) for (s, addr) in rate
           if s == src and addr not in KNOWN_ADDRS]
    unk.sort(key=lambda x: -x[1])
    if unk:
      print(f"\n  bus {src}: {len(unk)} unknown addresses (top 15):")
      for addr, n in unk[:15]:
        est_hz = n / 240.0
        print(f"    0x{addr:03x}  {n} frames  ~{est_hz:.1f} Hz")


if __name__ == '__main__':
  main()
