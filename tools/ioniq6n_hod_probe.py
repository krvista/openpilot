#!/usr/bin/env python3
"""HOD / hands-on spoofing feasibility probe.

Scans route 00000030 to answer:
  1. Is CCNC_0x2AF (687, HOD_FD_01_100ms) actually present on any bus?
  2. Which bus(es) / cycle time / source can we infer?
  3. What values does HOD_Dir_Status take while the car was idle
     (driver's hand naturally resting on / off the wheel)?
  4. For comparison: 0x11A (282, FR_CMR_01_10ms, DAW_WrnMsgSta) — is this
     the camera's hands-off-call route?
  5. What DOES CCNC consume that is hands-on related? Scan every address
     that appears on bus 1 / bus 0 with byte patterns that change when
     the user grabs/releases the wheel.
  6. Are any of these addresses on a bus we can TX to (bus 0 A-CAN)?
"""
import glob
import sys
from collections import Counter, defaultdict

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')

from openpilot.tools.lib.logreader import LogReader

ROUTE = '00000030'
DRIVELOG_DIR = '/home/user/openpilot/drivelog'

# Candidate addresses of interest (from DBC sweep)
HOD_ADDR = 0x2AF   # 687 HOD_FD_01_100ms — HOD_Dir_Status
FR_CMR_01 = 0x11A  # 282 FR_CMR_01_10ms — DAW_WrnMsgSta (Hands-off TMS call)
CCNC_161 = 0x161   # 353 CCNC_0x161 — ALERTS_2/3
CCNC_162 = 0x162   # 354 CCNC_0x162
MDPS_ADDR = 0x12A  # placeholder for MDPS / steering torque message
# Also log any address with bit-level change signature matching "hands-on toggle"


def scan_seg(path, per_addr_rate, per_addr_bus, hod_samples, fr_cmr_samples,
             mdps_samples, addr_first_byte_variance, change_counts):
  lr = LogReader(path)
  last_payload = {}
  for m in lr:
    try:
      w = m.which()
    except Exception:
      continue
    if w != 'can':
      continue
    t_ns = m.logMonoTime
    for c in m.can:
      src = c.src
      addr = c.address
      dat = bytes(c.dat)
      key = (src, addr)
      per_addr_rate[key] += 1
      per_addr_bus[addr].add(src)

      # Track byte-0 variability as a cheap proxy for "message carries
      # time-varying scalar content" (i.e. not a static config message).
      if dat:
        addr_first_byte_variance[key].add(dat[0])

      prev = last_payload.get(key)
      if prev is not None and prev != dat:
        change_counts[key] += 1
      last_payload[key] = dat

      # Dedicated capture for candidates
      if addr == HOD_ADDR:
        hod_samples.append((t_ns, src, dat))
      elif addr == FR_CMR_01:
        fr_cmr_samples.append((t_ns, src, dat))
      elif addr == MDPS_ADDR:
        mdps_samples.append((t_ns, src, dat))


def decode_hod(dat):
  """HOD_FD_01_100ms — HOD_Dir_Status @ bit 18, 3 bits, MSB=0 (Motorola).
  DBC: SG_ HOD_Dir_Status : 18|3@0+ — Motorola (start bit 18, length 3).
  In payload bytes, this is byte 2 (bits 16-23): bit 18 is bit position 2
  from the LSB side when we map Motorola to little-endian bytes. For a
  Motorola-MSB-0 signal with start=18 length=3, the bits are bits 18,17,16
  i.e. bits 2,1,0 of byte 2.
  """
  if len(dat) < 3:
    return None
  b = dat[2]
  return b & 0x07


DIR_MEANING = {
  0: 'HANDS_OFF',
  1: 'TOUCH_SOFT',
  2: 'TOUCH_STRONG',
  3: 'GRIP_SOFT',
  4: 'GRIP_STRONG',
  5: 'RESERVED5',
  6: 'RESERVED6',
  7: 'RESERVED7',
}


def decode_daw_wrn(dat):
  """DAW_WrnMsgSta @ bit 59 length 3 little-endian
  byte 7 bits [3..5] (bit 59 = byte 7 bit 3), zero-indexed.
  DBC SG_ DAW_WrnMsgSta : 59|3@1+ — little endian.
  start bit 59 = byte 7 bit 3.  (bits 59,60,61)
  """
  if len(dat) < 8:
    return None
  b = dat[7]
  return (b >> 3) & 0x07


def main():
  segs = sorted(glob.glob(f'{DRIVELOG_DIR}/*{ROUTE}*rlog.zst'))
  print(f"Route {ROUTE}: {len(segs)} segments")

  per_addr_rate = Counter()
  per_addr_bus = defaultdict(set)
  hod_samples = []
  fr_cmr_samples = []
  mdps_samples = []
  addr_first_byte_variance = defaultdict(set)
  change_counts = Counter()

  for p in segs:
    print(f"  scan {p.split('/')[-1]}")
    scan_seg(p, per_addr_rate, per_addr_bus, hod_samples, fr_cmr_samples,
             mdps_samples, addr_first_byte_variance, change_counts)

  # --- 0x2AF HOD ---
  print(f"\n{'='*70}")
  print(f"=== 0x2AF (687) HOD_FD_01_100ms ===")
  print(f"{'='*70}")
  buses = sorted(per_addr_bus.get(HOD_ADDR, []))
  if not buses:
    print("  NOT PRESENT on any bus.")
  else:
    for src in buses:
      cnt = per_addr_rate[(src, HOD_ADDR)]
      print(f"  bus {src}: {cnt:,} frames")

    if hod_samples:
      # time-based stats from first bus
      src0 = buses[0]
      t_samples = [t for (t, s, _) in hod_samples if s == src0]
      if len(t_samples) >= 2:
        span_s = (t_samples[-1] - t_samples[0]) / 1e9
        rate_hz = len(t_samples) / max(span_s, 1e-9)
        print(f"  bus {src0} effective rate: {rate_hz:.1f} Hz over {span_s:.0f}s  "
              f"(DBC says 100 ms → 10 Hz)")

      # decode HOD_Dir_Status over time
      status_counts = Counter()
      for _t, _s, dat in hod_samples:
        v = decode_hod(dat)
        if v is not None:
          status_counts[v] += 1
      print("\n  HOD_Dir_Status distribution (across all HOD frames):")
      for v, cnt in status_counts.most_common():
        print(f"    {v} {DIR_MEANING.get(v, '?'):<14}  {cnt:>7}")

      # Show first/last 3 frames
      print("\n  first 3 HOD frames:")
      for t, s, dat in hod_samples[:3]:
        print(f"    bus{s} t={t/1e9:.2f}  {dat.hex()}  -> {DIR_MEANING.get(decode_hod(dat))}")
      print("  last 3 HOD frames:")
      for t, s, dat in hod_samples[-3:]:
        print(f"    bus{s} t={t/1e9:.2f}  {dat.hex()}  -> {DIR_MEANING.get(decode_hod(dat))}")

  # --- 0x11A FR_CMR_01_10ms ---
  print(f"\n{'='*70}")
  print(f"=== 0x11A (282) FR_CMR_01_10ms (DAW_WrnMsgSta) ===")
  print(f"{'='*70}")
  buses = sorted(per_addr_bus.get(FR_CMR_01, []))
  if not buses:
    print("  NOT PRESENT on any bus.")
  else:
    for src in buses:
      cnt = per_addr_rate[(src, FR_CMR_01)]
      print(f"  bus {src}: {cnt:,} frames")
    if fr_cmr_samples:
      status_counts = Counter()
      for _t, _s, dat in fr_cmr_samples:
        v = decode_daw_wrn(dat)
        if v is not None:
          status_counts[v] += 1
      print("  DAW_WrnMsgSta distribution:")
      names = {0: 'NoWarning', 1: 'RestRecommend', 2: 'HandsOff-TMS', 3: 'R3',
               4: 'R4', 5: 'R5', 6: 'R6', 7: 'Error'}
      for v, cnt in status_counts.most_common():
        print(f"    {v} {names.get(v, '?'):<16}  {cnt:>7}")

  # --- CCNC_0x161/0x162 ---
  print(f"\n{'='*70}")
  print(f"=== 0x161 / 0x162 (CCNC_0x161 / CCNC_0x162) ===")
  print(f"{'='*70}")
  for addr in (CCNC_161, CCNC_162):
    buses = sorted(per_addr_bus.get(addr, []))
    print(f"  0x{addr:03x}:  buses={buses}")
    for src in buses:
      print(f"    bus {src}: {per_addr_rate[(src, addr)]:,} frames")

  # --- Dense bus-1 / bus-0 change-rate ranking ---
  # Messages most likely to carry fast-changing hands-on data.
  print(f"\n{'='*70}")
  print(f"=== Top 40 payload-changing addresses per bus (possible HOD carriers) ===")
  print(f"{'='*70}")
  by_bus = defaultdict(Counter)
  for (src, addr), cnt in change_counts.items():
    by_bus[src][addr] = cnt
  for src in sorted(by_bus):
    print(f"\n  --- bus {src} (top 40 by payload-change count) ---")
    for addr, cnt in by_bus[src].most_common(40):
      n_frames = per_addr_rate[(src, addr)]
      variance_bytes = len(addr_first_byte_variance[(src, addr)])
      marker = ''
      if addr == HOD_ADDR: marker = ' ← HOD'
      elif addr == FR_CMR_01: marker = ' ← FR_CMR_01'
      elif addr == CCNC_161: marker = ' ← CCNC_0x161'
      elif addr == CCNC_162: marker = ' ← CCNC_0x162'
      elif addr == 0x373: marker = ' ← STEERING_SENSORS'
      elif addr == 0x340: marker = ' ← STEERING'
      elif addr == 0x50: marker = ' ← LKAS'
      elif addr == 0x110: marker = ' ← LKAS_ALT'
      print(f"    0x{addr:03x}  {n_frames:>6} frames  {cnt:>6} changes  "
            f"{variance_bytes:>3} distinct byte[0]{marker}")

  # --- HOD absence hypothesis: maybe the capacitive sensor is on bus 2 (CAM)
  # --- or on a buried sub-bus.
  print(f"\n{'='*70}")
  print(f"=== Summary: feasibility of HOD spoof ===")
  print(f"{'='*70}")
  hod_buses = sorted(per_addr_bus.get(HOD_ADDR, []))
  if not hod_buses:
    print("  * 0x2AF (HOD) NOT observed on buses 0/1/2.")
    print("    → Either this car does not implement capacitive HOD at all")
    print("      (torque-only hands-on detection, like older Hyundais), or")
    print("      the HOD sensor lives behind a gateway on a sub-bus we don't see.")
    print("    → In either case, spoofing 0x2AF via panda gives nothing to")
    print("      consume — infeasible as a hands-on suppression route.")
  elif 0 in hod_buses:
    print("  * 0x2AF on bus 0 (A-CAN): panda CAN TX to bus 0, no relay interposition")
    print("    → Dual-publisher; same failure mode as 0x161 unless we can block source.")
  elif 1 in hod_buses:
    print("  * 0x2AF on bus 1 (E-CAN): same architectural trap as CCNC_0x161.")
    print("    → Native publisher uninterruptible, spoof creates bus 1 flicker.")
  elif 2 in hod_buses:
    print("  * 0x2AF ONLY on bus 2 (CAM): promising!")
    print("    → Panda relay can intercept bus 2 → bus 0/1 forwarding.")
    print("    → If CCNC/ADAS consumes HOD forwarded from bus 2, check_relay=true")
    print("      TX with our spoofed HAND_GRIP value would be the clean path.")


if __name__ == '__main__':
  main()
