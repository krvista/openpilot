#!/usr/bin/env python3
"""Correlate candidate HOD bits against the labelled grip/release timeline.

Route 00000031--85ea5c34a8 (16 segments) contains:

  Phase A  (stationary, engine on, parked, seatbelt on):
    - ~8 strong-grip cycles (user uncertain on exact count)
    - ~5 light-touch cycles (fingertip only)
    - left-blinker then right-blinker (sequence marker at end of Phase A)
  Phase B  (driving):
    - one lap around a block, both hands likely on wheel most of the time

Strategy:
  1. Segment the log into Phase A (stationary) vs Phase B (driving) using
     WHEEL_SPEEDS (0xa0, bits 64-78 = WHL_SpdFLVal @ 0.03125 km/h).
  2. For every (bus, addr, bit) on bus 1, count transitions separately in
     Phase A vs Phase B.
  3. True HOD bits have high Phase-A transitions (matches ~13 grip cycles
     = ~26 transitions) and LOW Phase-B transitions (driving = hands
     stable = few toggles).
  4. Score = phaseA_transitions / (phaseB_transitions + 1). Rank
     descending, filter to reasonable phaseA counts (8-40).
  5. Also report dwell-time distribution for top bits — HOD cycles last
     10-30 s so dwell-time mode should be in that band.
"""
import glob
import sys
from collections import Counter, defaultdict
from statistics import median

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')

from openpilot.tools.lib.logreader import LogReader

ROUTE = '00000031--85ea5c34a8'
DRIVELOG_DIR = '/home/user/openpilot/drivelog'

WHEEL_SPEEDS_ADDR = 0xa0  # 160, DLC 24, bus 1
# WHL_SpdFLVal : start_bit 64, length 14, little-endian, scale 0.03125 km/h

STATIONARY_KM_H = 0.5   # below this we consider stationary


def decode_speed(dat):
  """WHL_SpdFLVal: little-endian, bits 64..77 (byte 8 bit 0 through byte 9 bit 5),
  scale 0.03125 km/h. Returns km/h or None.
  """
  if len(dat) < 10:
    return None
  raw = int.from_bytes(dat, 'little')
  val = (raw >> 64) & ((1 << 14) - 1)
  return val * 0.03125


def main():
  segs = sorted(glob.glob(f'{DRIVELOG_DIR}/*{ROUTE}*rlog.zst'),
                key=lambda p: int(p.split('--')[-2]))
  print(f"Route {ROUTE}: {len(segs)} segments")

  # Pass 1: find stationary intervals via WHEEL_SPEEDS
  # Collect (t_ns, speed_kmh) samples from bus 1 0xa0
  speed_samples = []   # list of (t_ns, kmh)
  # Also record absolute log time bounds
  t_start_ns = None
  t_end_ns = None

  # Pass 2: per-bit transition tracking on bus 1 — we do both passes in
  # one sweep to avoid re-reading the logs.
  bus1_frames = defaultdict(list)   # addr -> list of (t_ns, bits_int, dlc)

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
      if t_start_ns is None:
        t_start_ns = t_ns
      t_end_ns = t_ns
      for c in m.can:
        if c.src != 1:
          continue
        addr = c.address
        dat = bytes(c.dat)
        if addr == WHEEL_SPEEDS_ADDR:
          v = decode_speed(dat)
          if v is not None:
            speed_samples.append((t_ns, v))
        # Only retain addresses in the interesting short-list to keep
        # memory bounded. Dynamic expansion: we scan all bus-1 addrs
        # on first pass then filter later.
        bits = int.from_bytes(dat, 'little')
        bus1_frames[addr].append((t_ns, bits, len(dat)))

  total_span_s = (t_end_ns - t_start_ns) / 1e9
  print(f"\nTotal log span: {total_span_s:.1f} s")
  print(f"WHEEL_SPEEDS samples: {len(speed_samples)}")

  if not speed_samples:
    print("ERROR: no WHEEL_SPEEDS frames — cannot segment by speed.")
    sys.exit(1)

  # Build moving flag timeline: array of (t_ns, moving_bool)
  speed_samples.sort()
  # For transition classification we need: for any t_ns, is the car
  # currently stationary? We'll carry the most recent speed forward.
  phaseA_windows = []  # list of (start_ns, end_ns) where stationary
  cur_start = None
  last_v = 0.0
  for t_ns, v in speed_samples:
    moving = v > STATIONARY_KM_H
    if not moving:
      if cur_start is None:
        cur_start = t_ns
      last_v = v
    else:
      if cur_start is not None:
        phaseA_windows.append((cur_start, t_ns))
        cur_start = None
      last_v = v
  if cur_start is not None:
    phaseA_windows.append((cur_start, t_end_ns))

  # Merge tiny stationary gaps (<2 s) caused by wheel speed noise
  merged = []
  for w in phaseA_windows:
    if merged and (w[0] - merged[-1][1]) < 2e9:
      merged[-1] = (merged[-1][0], w[1])
    else:
      merged.append(w)
  phaseA_windows = [w for w in merged if (w[1] - w[0]) > 10e9]  # drop <10s slivers

  print(f"\nStationary windows (>{STATIONARY_KM_H} km/h threshold, >10 s):")
  for i, (a, b) in enumerate(phaseA_windows):
    ta = (a - t_start_ns) / 1e9
    tb = (b - t_start_ns) / 1e9
    print(f"  [{i}] t={ta:7.1f} .. {tb:7.1f}  ({tb-ta:5.1f} s)")

  # Total phaseA and phaseB durations
  total_A = sum((b - a) for a, b in phaseA_windows) / 1e9
  total_B = total_span_s - total_A
  print(f"\nPhase A (stationary) total: {total_A:.1f} s")
  print(f"Phase B (driving)    total: {total_B:.1f} s")

  def in_phaseA(t_ns):
    # O(log n) via bisect would be nicer; linear scan OK for ~15 windows.
    for a, b in phaseA_windows:
      if a <= t_ns <= b:
        return True
      if t_ns < a:
        return False
    return False

  # Per-address, per-bit transition counting split by phase
  # Plus dwell times: list of (phase, duration_s) per bit
  print(f"\n{'='*70}")
  print(f"=== Scoring every bus-1 bit by (phaseA toggles)/(phaseB toggles + 1) ===")
  print(f"{'='*70}")

  addr_bit_A = defaultdict(lambda: defaultdict(int))   # [addr][bit] = phaseA transitions
  addr_bit_B = defaultdict(lambda: defaultdict(int))
  addr_bit_dwells_A = defaultdict(lambda: defaultdict(list))  # dwell durations during A
  addr_rate = {}

  for addr, frames in bus1_frames.items():
    if len(frames) < 10:
      continue
    frames.sort()
    addr_rate[addr] = len(frames)
    prev_bits = None
    prev_t = None
    # track per-bit last-flip time to compute dwell
    last_flip_t = {}  # bit -> t_ns
    dlc_bits = frames[0][2] * 8
    for t_ns, bits, dlc in frames:
      if prev_bits is not None and bits != prev_bits:
        xor = bits ^ prev_bits
        isA = in_phaseA(t_ns)
        for bit in range(dlc_bits):
          if xor & (1 << bit):
            if isA:
              addr_bit_A[addr][bit] += 1
            else:
              addr_bit_B[addr][bit] += 1
            if isA and bit in last_flip_t:
              dur = (t_ns - last_flip_t[bit]) / 1e9
              if dur < 120:
                addr_bit_dwells_A[addr][bit].append(dur)
            last_flip_t[bit] = t_ns
      prev_bits = bits

  # Ranking: for each addr, list the top bits by score
  rows = []
  for addr, bitA in addr_bit_A.items():
    for bit, A in bitA.items():
      B = addr_bit_B[addr].get(bit, 0)
      if A < 5:  # too few to be a meaningful grip pattern
        continue
      score = A / (B + 1)
      rows.append((score, addr, bit, A, B))

  rows.sort(reverse=True)

  # Filter to reasonable phaseA counts — 13 grip events => 26 bit
  # transitions if perfectly clean; real capacitive HOD may be filtered,
  # so broaden to 10..60.
  print(f"\n{'addr':>6} {'bit':>4} {'byte':>4} {'bit_in_byte':>3}  "
        f"{'A':>5} {'B':>5} {'score':>7}  dwell_median/min/max (Phase A)")
  shown = 0
  for score, addr, bit, A, B in rows:
    if not (8 <= A <= 60):
      continue
    if score < 3:
      continue
    d = addr_bit_dwells_A[addr][bit]
    if d:
      dm = median(d)
      dmin = min(d)
      dmax = max(d)
      dstr = f"{dm:6.1f} / {dmin:5.1f} / {dmax:5.1f}"
    else:
      dstr = "   --      --      --"
    print(f"  0x{addr:03x} {bit:>4} {bit//8:>4} {bit%8:>3}     "
          f"{A:>5} {B:>5} {score:>7.1f}   {dstr}")
    shown += 1
    if shown >= 40:
      break

  # Also focused dump on prior top candidates
  CANDIDATES = [0x1b5, 0x1ba, 0x1e5, 0x3e5, 0x3d4, 0x175, 0x170, 0x171, 0x172]
  print(f"\n{'='*70}")
  print(f"=== Focused bit-by-bit phase split for prior top candidates ===")
  print(f"{'='*70}")
  for addr in CANDIDATES:
    if addr not in addr_bit_A and addr not in addr_bit_B:
      continue
    print(f"\n  0x{addr:03x}  (frames={addr_rate.get(addr, 0)}):")
    bits = set(addr_bit_A.get(addr, {}).keys()) | set(addr_bit_B.get(addr, {}).keys())
    rows_c = []
    for bit in bits:
      A = addr_bit_A[addr].get(bit, 0)
      B = addr_bit_B[addr].get(bit, 0)
      if A + B < 4:
        continue
      rows_c.append((A / (B + 1), A, B, bit))
    rows_c.sort(reverse=True)
    print(f"    {'bit':>4}  {'byte':>4} {'b':>3}  {'A':>5} {'B':>5}  score   dwell(P_A) s")
    for score, A, B, bit in rows_c[:15]:
      d = addr_bit_dwells_A[addr].get(bit, [])
      if d:
        dsample = sorted(d)
        dsum = ", ".join(f"{x:.1f}" for x in dsample[:8])
        if len(dsample) > 8:
          dsum += f", ... ({len(dsample)} total)"
      else:
        dsum = "(no A dwells)"
      print(f"    {bit:>4}  {bit//8:>4} {bit%8:>3}  {A:>5} {B:>5} {score:>6.1f}  {dsum}")


if __name__ == '__main__':
  main()
