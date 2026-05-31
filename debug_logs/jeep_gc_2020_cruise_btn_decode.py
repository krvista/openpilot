#!/usr/bin/env python3
"""Hand-decode spot check for one rlog segment's CRUISE_BUTTONS (0x23B)
TX (sendcan) and RX (can src=0).

Verifies the bit positions used by jeep_gc_2020_rlog_analysis.py against
DBC _stellantis_common.dbc:BO_ 571.

Usage:
  python3 jeep_gc_2020_cruise_btn_decode.py <path-to-rlog.zst>
"""
import sys

sys.modules['smbus2'] = type(sys)('smbus2')
sys.modules['smbus2'].SMBus = object
sys.modules['serial'] = type(sys)('serial')
from openpilot.tools.lib.logreader import _LogFileReader

CRUISE_BUTTONS_ADDR = 0x23B  # 571


def decode(payload):
  if not payload:
    return {}
  b0 = payload[0]
  b1 = payload[1] if len(payload) > 1 else 0
  b2 = payload[2] if len(payload) > 2 else 0
  return {
    'b0_hex': f'0x{b0:02x}',
    'b1_hex': f'0x{b1:02x}',
    'b2_hex': f'0x{b2:02x}',
    # high-nibble COUNTER per DBC SG_ COUNTER : 15|4@0+
    'COUNTER_high':  (b1 >> 4) & 0x0F,
    # low-nibble (legacy/buggy decode) — should always be 0 (reserved/unused)
    'low_nibble':    b1 & 0x0F,
    'CHECKSUM':      b2,
    # Per DBC bit positions (see opendbc/dbc/generator/chrysler/_stellantis_common.dbc):
    'ACC_Cancel':       bool(b0 & 0x01),  # bit 0
    'ACC_Distance_Dec': bool(b0 & 0x02),  # bit 1
    'ACC_Accel':        bool(b0 & 0x04),  # bit 2
    'ACC_Decel':        bool(b0 & 0x08),  # bit 3
    'ACC_Resume':       bool(b0 & 0x10),  # bit 4
    'Cruise_OnOff':     bool(b0 & 0x40),  # bit 6
    'ACC_OnOff':        bool(b0 & 0x80),  # bit 7
    'ACC_Distance_Inc': bool(b1 & 0x01),  # bit 8
  }


def fmt(d):
  bits = []
  for name in ('ACC_Cancel', 'ACC_Distance_Dec', 'ACC_Accel', 'ACC_Decel',
               'ACC_Resume', 'Cruise_OnOff', 'ACC_OnOff', 'ACC_Distance_Inc'):
    if d.get(name):
      bits.append(name)
  return f'b0={d["b0_hex"]} b1={d["b1_hex"]} b2={d["b2_hex"]} | COUNTER(high)={d["COUNTER_high"]:>2} low_nibble={d["low_nibble"]} CHECKSUM=0x{d["CHECKSUM"]:02x} | {"+".join(bits) if bits else "(idle)"}'


def main():
  if len(sys.argv) != 2:
    print(__doc__, file=sys.stderr)
    sys.exit(1)
  path = sys.argv[1]
  print(f'Decoding {path}\n')

  lr = _LogFileReader(path)
  tx_msgs = []
  rx_msgs = []
  rx_count_per_bus = {}
  rx_count_total = 0
  tx_count_total = 0
  t0 = None

  for msg in lr:
    try:
      t = msg.logMonoTime
    except Exception:
      continue
    if t0 is None:
      t0 = t
    rel_t = (t - t0) / 1e9
    which = msg.which()
    if which == 'sendcan':
      for c in msg.sendcan:
        if c.address == CRUISE_BUTTONS_ADDR:
          tx_count_total += 1
          if len(tx_msgs) < 30:
            tx_msgs.append((rel_t, c.src, decode(c.dat)))
    elif which == 'can':
      for c in msg.can:
        if c.address == CRUISE_BUTTONS_ADDR:
          rx_count_total += 1
          rx_count_per_bus[c.src] = rx_count_per_bus.get(c.src, 0) + 1
          if len(rx_msgs) < 30:
            rx_msgs.append((rel_t, c.src, decode(c.dat)))

  print(f'=== Segment summary ===')
  print(f'  TX (sendcan)            count = {tx_count_total}')
  print(f'  RX (can, all buses)     count = {rx_count_total}')
  print(f'  RX per bus              {rx_count_per_bus}')
  print()

  print(f'=== First {len(tx_msgs)} TX (sendcan) ===')
  for rel_t, bus, d in tx_msgs:
    print(f'  t={rel_t:>8.3f}s bus={bus}  {fmt(d)}')

  print()
  print(f'=== First {len(rx_msgs)} RX (can src=any) ===')
  for rel_t, bus, d in rx_msgs:
    print(f'  t={rel_t:>8.3f}s bus={bus}  {fmt(d)}')

  print()
  print(f'=== Verification ===')
  if tx_msgs:
    counters = [d['COUNTER_high'] for _, _, d in tx_msgs[:12]]
    print(f'  First 12 TX high-nibble COUNTER: {counters}')
    if all(d['low_nibble'] == 0 for _, _, d in tx_msgs):
      print('  All TX low_nibble = 0 (confirms reserved/unused — old script bug)')
    base = tx_msgs[0][2]['COUNTER_high']
    diffs = [(c - p) & 0x0F for p, c in zip(counters[:-1], counters[1:])]
    print(f'  Consecutive deltas: {diffs}')
    print(f'  ICBM expects [+1, +1, +0, skip] cycle from base CS.button_counter')
    print(f'    if RX count = 0  -> base = 0 (init), tx COUNTER cycles 1, 1, 0, skip')
    print(f'    if RX count > 0  -> base follows latest RX COUNTER')
    print(f'  Observed pattern matches: {"yes" if diffs[:6] in ([1,15,1,1,15,1],[0,1,15,1,1,15],[1,1,15,1,1,15]) else "inspect manually"}')


if __name__ == '__main__':
  main()
