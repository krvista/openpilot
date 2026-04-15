#!/usr/bin/env python3
"""Verify that Hyundai CAN FD's standard CRC algorithm (hkg_can_fd_checksum)
computes byte[0..1] correctly for 0x208 frames captured in the HOD
drivelog. If so, no CRC reverse-engineering is needed — we can spoof
directly using the existing openpilot helper.
"""
import glob
import sys

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')

from openpilot.tools.lib.logreader import LogReader
from opendbc.car.hyundai.hyundaicanfd import hkg_can_fd_checksum

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
          frames.append(bytes(c.dat))

  print(f"Total 0x208 frames: {len(frames)}")

  # For each frame: extract the on-wire CRC (byte 0-1, little-endian),
  # compute expected CRC via hkg_can_fd_checksum, compare.
  match = 0
  mismatch = 0
  first_mismatches = []
  first_matches = []
  for dat in frames:
    if len(dat) < 2:
      continue
    wire_crc = dat[0] | (dat[1] << 8)  # little-endian
    # Byte order: hkg_can_fd_checksum uses d[2:], so byte 0-1 are
    # expected CRC bytes. We pass the full 16-byte frame.
    computed = hkg_can_fd_checksum(ADDR, None, bytearray(dat))
    if computed == wire_crc:
      match += 1
      if len(first_matches) < 3:
        first_matches.append((dat, wire_crc, computed))
    else:
      mismatch += 1
      if len(first_mismatches) < 5:
        first_mismatches.append((dat, wire_crc, computed))

  print(f"Match:    {match}")
  print(f"Mismatch: {mismatch}")
  if match:
    print("\n--- Matching frames (first 3) ---")
    for dat, wire, comp in first_matches:
      print(f"  wire=0x{wire:04x}  computed=0x{comp:04x}  "
            f"hex={dat.hex()}")
  if mismatch:
    print("\n--- Mismatching frames (first 5) ---")
    for dat, wire, comp in first_mismatches:
      print(f"  wire=0x{wire:04x}  computed=0x{comp:04x}  "
            f"hex={dat.hex()}")

    # If all mismatch, try big-endian or byte-swapped CRC
    print("\n--- Trying big-endian interpretation ---")
    match2 = 0
    for dat in frames[:100]:
      if len(dat) < 2:
        continue
      wire_be = (dat[0] << 8) | dat[1]
      computed = hkg_can_fd_checksum(ADDR, None, bytearray(dat))
      if computed == wire_be:
        match2 += 1
    print(f"  big-endian match: {match2}/100")


if __name__ == '__main__':
  main()
