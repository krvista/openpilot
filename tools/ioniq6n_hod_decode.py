#!/usr/bin/env python3
"""Decode the top HOD candidates (0x35c, 0x3e3, 0x35a, 0x35b) as time
series and sanity-check them against the user's grip test protocol.

If one of these addresses holds the capacitive HOD state, we expect:
  - Many discrete value transitions during Phase A (stationary)
  - Low-level noise/stability during Phase B (driving, hands on)
  - 13 event peaks total (8 strong grips + 5 light touches)
  - dwell times clustering near 10-20 s (user's grip/release timing)

We dump byte 0..3 of each candidate over time (downsampled) and histogram
the observed 8-bit and 16-bit values during Phase A vs Phase B.
"""
import glob
import sys
from collections import Counter, defaultdict

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')

from openpilot.tools.lib.logreader import LogReader

ROUTE = '00000031--85ea5c34a8'
DRIVELOG_DIR = '/home/user/openpilot/drivelog'

WHEEL_SPEEDS_ADDR = 0xa0

CANDIDATES = [0x35c, 0x3e3, 0x35a, 0x35b, 0x208, 0x476, 0x400]


def decode_speed(dat):
  if len(dat) < 10:
    return None
  raw = int.from_bytes(dat, 'little')
  return ((raw >> 64) & 0x3fff) * 0.03125


def main():
  segs = sorted(glob.glob(f'{DRIVELOG_DIR}/*{ROUTE}*rlog.zst'),
                key=lambda p: int(p.split('--')[-2]))

  # Pass 1: collect per-address frames
  frames = defaultdict(list)   # addr -> (t_ns, dat)
  speed_samples = []
  t_start = None
  for p in segs:
    print(f"  scan {p.split('/')[-1]}")
    lr = LogReader(p)
    for m in lr:
      try:
        if m.which() != 'can':
          continue
      except Exception:
        continue
      t_ns = m.logMonoTime
      if t_start is None:
        t_start = t_ns
      for c in m.can:
        if c.src != 1:
          continue
        if c.address == WHEEL_SPEEDS_ADDR:
          v = decode_speed(bytes(c.dat))
          if v is not None:
            speed_samples.append((t_ns, v))
        if c.address in CANDIDATES:
          frames[c.address].append((t_ns, bytes(c.dat)))

  # Build stationary mask (same rules as correlator)
  speed_samples.sort()
  phaseA = []
  cur_start = None
  for t, v in speed_samples:
    if v <= 0.5:
      if cur_start is None:
        cur_start = t
    else:
      if cur_start is not None:
        phaseA.append((cur_start, t))
        cur_start = None
  if cur_start is not None:
    phaseA.append((cur_start, speed_samples[-1][0]))
  merged = []
  for w in phaseA:
    if merged and (w[0] - merged[-1][1]) < 2e9:
      merged[-1] = (merged[-1][0], w[1])
    else:
      merged.append(w)
  phaseA = [w for w in merged if (w[1] - w[0]) > 10e9]

  def in_A(t):
    for a, b in phaseA:
      if a <= t <= b:
        return True
      if t < a:
        return False
    return False

  # For each candidate, print:
  #   (1) Phase A state-change timeline of byte 0 and byte 1
  #   (2) Value histogram (byte 0 + byte 1 + their 3-bit sub-fields)
  for addr in CANDIDATES:
    if addr not in frames:
      continue
    print(f"\n{'='*78}")
    print(f"=== 0x{addr:03x}   ({len(frames[addr])} frames) ===")
    print(f"{'='*78}")

    # state-change timeline (byte 0 and byte 1 combined = 16-bit value)
    # Track when byte 0, byte 1, byte 2 change. Collapse consecutive
    # identical values.
    print("\nPhase A: byte0/byte1/byte2 state changes (relative t_s from log start):")
    last_key = None
    events = []
    for t, dat in frames[addr]:
      if not in_A(t):
        continue
      if len(dat) < 3:
        continue
      key = (dat[0], dat[1], dat[2])
      if key != last_key:
        rel = (t - t_start) / 1e9
        events.append((rel, dat[0], dat[1], dat[2]))
        last_key = key
    print(f"  {len(events)} distinct byte[0..2] states during Phase A")
    # Print first 40 and last 20 if long
    show = events if len(events) <= 60 else events[:40] + [("...", "...", "...", "...")] + events[-20:]
    for ev in show:
      if ev[0] == "...":
        print("  ...")
        continue
      rel, b0, b1, b2 = ev
      print(f"  t={rel:7.1f}s  b0=0x{b0:02x}  b1=0x{b1:02x}  b2=0x{b2:02x}    "
            f"b0={b0:3d}  b1={b1:3d}  b2={b2:3d}")

    # Histograms
    histA = Counter()
    histB = Counter()
    for t, dat in frames[addr]:
      if len(dat) < 3:
        continue
      key = (dat[0], dat[1], dat[2])
      if in_A(t):
        histA[key] += 1
      else:
        histB[key] += 1

    print(f"\nTop byte[0..2] values during Phase A (13+ event transitions expected):")
    for (b0, b1, b2), cnt in histA.most_common(8):
      print(f"  ({b0:3d}, {b1:3d}, {b2:3d})  0x{b0:02x}{b1:02x}{b2:02x}  {cnt:>6}")
    print(f"Top byte[0..2] values during Phase B (driving):")
    for (b0, b1, b2), cnt in histB.most_common(8):
      print(f"  ({b0:3d}, {b1:3d}, {b2:3d})  0x{b0:02x}{b1:02x}{b2:02x}  {cnt:>6}")


if __name__ == '__main__':
  main()
