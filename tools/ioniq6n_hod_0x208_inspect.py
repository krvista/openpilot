#!/usr/bin/env python3
"""Inspect 0x208 frame structure to assess HOD spoof feasibility.

We confirmed 0x208 byte[10] is HOD_Dir_Status on Ioniq 6 N. Now we need
to know:
  1. Does 0x208 carry a counter/CRC like other CAN-FD messages?
     (If byte[0..1] or byte[2] look CRC-like, spoofing needs to forge it.)
  2. What are the other 15 bytes doing?
  3. At state==4 (GRIP_STRONG), is the rest of the payload stable?
     A stable payload => we can record a known-good frame and just
     edit byte[10] when spoofing.
"""
import glob
import sys
from collections import Counter, defaultdict

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')

from openpilot.tools.lib.logreader import LogReader

ROUTE = '00000031--85ea5c34a8'
DRIVELOG_DIR = '/home/user/openpilot/drivelog'
ADDR = 0x208


def main():
  segs = sorted(glob.glob(f'{DRIVELOG_DIR}/*{ROUTE}*rlog.zst'),
                key=lambda p: int(p.split('--')[-2]))

  frames = []
  for p in segs:
    lr = LogReader(p)
    for m in lr:
      try:
        if m.which() != 'can':
          continue
      except Exception:
        continue
      for c in m.can:
        if c.src == 1 and c.address == ADDR:
          frames.append((m.logMonoTime, bytes(c.dat)))

  print(f"0x208: {len(frames)} frames, DLC={len(frames[0][1])} bytes")

  # 1. Per-byte distinct values
  dlc = len(frames[0][1])
  per_byte = defaultdict(Counter)
  for _t, dat in frames:
    for i, b in enumerate(dat):
      per_byte[i][b] += 1

  print(f"\nPer-byte cardinality:")
  for i in range(dlc):
    nvals = len(per_byte[i])
    top = per_byte[i].most_common(3)
    print(f"  byte {i:>2}: {nvals:>3} distinct values  "
          f"top3={[(f'0x{v:02x}', c) for v, c in top]}")

  # 2. Sample 4 frames per HOD state
  print(f"\n{'='*74}")
  print(f"Sample frames by HOD state (byte 10):")
  print(f"{'='*74}")
  for state in (0, 1, 2, 3, 4):
    matches = [(t, d) for t, d in frames if len(d) > 10 and d[10] == state]
    print(f"\n--- state {state} ({len(matches)} frames) ---")
    for t, d in matches[:4]:
      print(f"  {' '.join(f'{b:02x}' for b in d)}")
    if len(matches) > 4:
      print("  ...")
      for t, d in matches[-2:]:
        print(f"  {' '.join(f'{b:02x}' for b in d)}")

  # 3. Check counter hypothesis: is any byte incrementing modulo?
  # A CAN-FD counter typically increments 1 per frame, wraps at 16 or 256.
  print(f"\n{'='*74}")
  print(f"Counter search: byte-wise delta histogram across consecutive frames")
  print(f"{'='*74}")
  prev_dat = None
  per_byte_delta = defaultdict(Counter)
  for _t, d in frames:
    if prev_dat is not None:
      for i in range(min(len(d), len(prev_dat))):
        delta = (d[i] - prev_dat[i]) & 0xff
        per_byte_delta[i][delta] += 1
    prev_dat = d

  for i in range(dlc):
    top = per_byte_delta[i].most_common(3)
    top_str = ", ".join(f"Δ{delta:+d}: {n}" for delta, n in top)
    print(f"  byte {i:>2}  deltas: {top_str}")

  # 4. Byte 11 (next to state) distribution — often the "warning level" or
  # secondary flag for the HOD group.
  print(f"\n{'='*74}")
  print(f"Cross-tab: byte[10]=state vs byte[11] value")
  print(f"{'='*74}")
  xtab = defaultdict(Counter)
  for _t, d in frames:
    if len(d) > 11:
      xtab[d[10]][d[11]] += 1
  for state in sorted(xtab):
    top = xtab[state].most_common(5)
    top_str = ", ".join(f"0x{v:02x}:{n}" for v, n in top)
    print(f"  state {state}: byte[11] -> {top_str}")


if __name__ == '__main__':
  main()
