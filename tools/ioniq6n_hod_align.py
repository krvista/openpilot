#!/usr/bin/env python3
"""Find the true HOD state register by aligning bus-1 bit transitions
against the event timeline recovered from 0x35c byte-2 (a monotonic
event-sequence counter that cleanly indexes capacitive-HOD state
changes).

Why: 0x35a/0x35b/0x35c all have byte[2] = event counter and byte[0..1] =
pseudo-random (likely SecOC MAC / CRC). They *signal* HOD events but the
state value itself is elsewhere.

Algorithm:
  1. Extract the set of event timestamps T = {t_ns : 0x35c byte[2] just
     incremented}. Expect ~30 timestamps corresponding to grip-on, grip-
     off, touch-on, touch-off transitions (13 events × 2).
  2. For each (bus=1, addr), compute the *value history* of each byte.
     For each byte with <=8 distinct values (low cardinality = state
     flag, not CRC / counter), record all state-transition times.
  3. For each such byte, compute the *alignment score*:
        aligned = # state transitions within |Δt| < 1.0 s of any T event
        total   = # state transitions
        score   = aligned / total
     HOD bits will have score near 1.0 (every change tied to an event).
  4. Report top-ranked (addr, byte) pairs with value distributions.
"""
import glob
import sys
from collections import Counter, defaultdict

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')

from openpilot.tools.lib.logreader import LogReader

ROUTE = '00000031--85ea5c34a8'
DRIVELOG_DIR = '/home/user/openpilot/drivelog'

EVENT_ADDR = 0x35c   # byte 2 is the event counter
WHEEL_SPEEDS_ADDR = 0xa0

ALIGN_WIN_NS = int(1.0e9)   # ±1 s alignment window


def decode_speed(dat):
  if len(dat) < 10:
    return None
  raw = int.from_bytes(dat, 'little')
  return ((raw >> 64) & 0x3fff) * 0.03125


def main():
  segs = sorted(glob.glob(f'{DRIVELOG_DIR}/*{ROUTE}*rlog.zst'),
                key=lambda p: int(p.split('--')[-2]))

  # Pass 1: extract event timeline from 0x35c byte-2 increments
  event_times = []
  last_b2 = None
  # Also collect bus-1 per-(addr,byte) value histories for alignment check
  addr_byte_history = defaultdict(lambda: defaultdict(list))  # [addr][byte] = [(t_ns, val), ...]
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
        dat = bytes(c.dat)
        addr = c.address
        if addr == WHEEL_SPEEDS_ADDR:
          v = decode_speed(dat)
          if v is not None:
            speed_samples.append((t_ns, v))
        if addr == EVENT_ADDR:
          if len(dat) >= 3:
            b2 = dat[2]
            if last_b2 is not None and b2 != last_b2:
              event_times.append(t_ns)
            last_b2 = b2
        # Record (byte, value) changes for every bus-1 addr
        for i, b in enumerate(dat):
          hist = addr_byte_history[addr][i]
          if not hist or hist[-1][1] != b:
            hist.append((t_ns, b))

  print(f"\nRecovered {len(event_times)} event timestamps from 0x35c.")
  print("First 20 event times (s from log start):")
  for t in event_times[:20]:
    print(f"  {(t - t_start) / 1e9:7.2f}")
  print("Last 10 event times:")
  for t in event_times[-10:]:
    print(f"  {(t - t_start) / 1e9:7.2f}")

  # Build sorted event times + bisect-able binary structure for alignment
  import bisect
  et_sorted = sorted(event_times)

  def is_aligned(t_ns):
    # Is there any event within ALIGN_WIN_NS of t_ns?
    i = bisect.bisect_left(et_sorted, t_ns)
    for j in (i - 1, i):
      if 0 <= j < len(et_sorted):
        if abs(et_sorted[j] - t_ns) <= ALIGN_WIN_NS:
          return True
    return False

  # Exclude the 0x35a/b/c cluster from search — they are the event
  # messenger, not the HOD state
  EXCLUDE = {0x35a, 0x35b, 0x35c}

  print(f"\n{'='*74}")
  print(f"=== Bus-1 bytes with low cardinality & high event-alignment ===")
  print(f"{'='*74}")

  rows = []
  for addr, byte_map in addr_byte_history.items():
    if addr in EXCLUDE:
      continue
    for byte_idx, hist in byte_map.items():
      if len(hist) < 5:
        continue
      distinct = set(v for _, v in hist)
      if len(distinct) > 8:   # CRC / counter / analog = skip
        continue
      # Alignment score
      transition_times = [t for t, _ in hist[1:]]  # skip initial state
      if not transition_times:
        continue
      aligned = sum(1 for t in transition_times if is_aligned(t))
      total = len(transition_times)
      score = aligned / total
      if aligned < 6:  # too few to be HOD
        continue
      rows.append((score, aligned, total, len(distinct), addr, byte_idx, sorted(distinct)))

  rows.sort(reverse=True)
  print(f"\n{'score':>6}  {'aligned':>7} {'total':>5} {'#vals':>5}  {'addr':>6} {'byte':>4}   values")
  shown = 0
  for score, aligned, total, nvals, addr, byte_idx, vals in rows:
    if score < 0.5:
      break
    vstr = "[" + ", ".join(f"0x{v:02x}" for v in vals) + "]"
    print(f"  {score:>5.2f}   {aligned:>7} {total:>5} {nvals:>5}   0x{addr:03x} {byte_idx:>4}   {vstr}")
    shown += 1
    if shown >= 40:
      break

  # Top-5 deep dive: print value time-series aligned with events
  print(f"\n{'='*74}")
  print(f"=== Top-5 candidate deep dive: value time-series ===")
  print(f"{'='*74}")
  for score, aligned, total, nvals, addr, byte_idx, vals in rows[:5]:
    hist = addr_byte_history[addr][byte_idx]
    print(f"\n--- 0x{addr:03x} byte {byte_idx}   score={score:.2f}  "
          f"aligned={aligned}/{total}  distinct={vals} ---")
    # show the first 40 changes and last 10
    changes = hist
    if len(changes) > 60:
      show = changes[:40] + [("skip", "skip")] + changes[-10:]
    else:
      show = changes
    for t, v in show:
      if t == "skip":
        print("   ... (truncated) ...")
        continue
      rel = (t - t_start) / 1e9
      near = is_aligned(t) if t != "skip" else False
      mark = "*" if near else " "
      print(f"  {mark} t={rel:7.2f}s   byte{byte_idx}=0x{v:02x} ({v})")


if __name__ == '__main__':
  main()
