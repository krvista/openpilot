#!/usr/bin/env python3
"""rlog quant sweep for Jeep GC drivelog — T2-B (CRUISE_BUTTONS tx) +
T3-C (chrysler RX-check freshness).

Iterates every .rlog.zst under --drivelog-dir, collects per-segment counters,
writes one JSON record per segment to --out.
"""
import argparse
import glob
import json
import os
import re
import sys
from collections import Counter

sys.modules['smbus2'] = type(sys)('smbus2')
sys.modules['smbus2'].SMBus = object
sys.modules['serial'] = type(sys)('serial')
from openpilot.tools.lib.logreader import _LogFileReader

# CRUISE_BUTTONS tx (non-RAM chrysler, bus 0)
CRUISE_BUTTONS_ADDR = 0x23B

# RX_CHECKS for chrysler (non-RAM) — from
# opendbc_repo/opendbc/safety/modes/chrysler.h:179
# (addr, expected_hz)
RX_CHECKS = [
  (0x220, 100, "EPS_2"),
  (0x140, 50,  "ESP_1"),
  (0x202, 100, "addr_514"),
  (0x22F, 50,  "ECM_5"),
  (0x1F4, 50,  "DAS_3"),
  (0x330, 1,   "TRACTION_BUTTON"),
]
RX_ADDRS = {a for a, _, _ in RX_CHECKS}
ADDR_NAME = {a: n for a, _, n in RX_CHECKS}
ADDR_HZ = {a: hz for a, hz, _ in RX_CHECKS}

NAME_RE = re.compile(r'([0-9a-f]{16})_([0-9a-f]+)--([0-9a-f]+)--(\d+)--rlog\.zst$')


def percentile(xs, p):
  if not xs:
    return 0.0
  xs = sorted(xs)
  idx = int(round((len(xs) - 1) * p))
  return xs[idx]


def parse_cruise_buttons_byte0(payload):
  """Decode CRUISE_BUTTONS (0x23B) per
  opendbc/dbc/generator/chrysler/_stellantis_common.dbc:BO_ 571.

    SG_ ACC_Cancel       :  0|1@1+   (bit 0)
    SG_ ACC_Distance_Dec :  1|1@1+   (bit 1)
    SG_ ACC_Accel        :  2|1@1+   (bit 2)
    SG_ ACC_Decel        :  3|1@1+   (bit 3)
    SG_ ACC_Resume       :  4|1@0+   (bit 4)
    SG_ Cruise_OnOff     :  6|1@1+   (bit 6)
    SG_ ACC_OnOff        :  7|1@1+   (bit 7)
    SG_ ACC_Distance_Inc :  8|1@1+   (byte1 bit 0)
    SG_ COUNTER          : 15|4@0+   (byte1 bits 7-4 — Motorola, high nibble)
    SG_ CHECKSUM         : 23|8@0+   (byte2)
  """
  if not payload:
    return None, None, None
  b0 = payload[0] if isinstance(payload[0], int) else payload[0]
  b1 = payload[1] if len(payload) > 1 else 0
  signals = {
    'cancel':       bool(b0 & 0x01),
    'dist_dec':     bool(b0 & 0x02),
    'accel':        bool(b0 & 0x04),
    'decel':        bool(b0 & 0x08),
    'resume':       bool(b0 & 0x10),
    'cruise_onoff': bool(b0 & 0x40),
    'acc_onoff':    bool(b0 & 0x80),
    'dist_inc':     bool(b1 & 0x01),
  }
  counter = (b1 >> 4) & 0x0F
  return b0, signals, counter


def process_segment(path):
  m = NAME_RE.search(path)
  if not m:
    return None
  dongle, route_hex, route_uuid, seg = m.group(1), m.group(2), m.group(3), int(m.group(4))
  route = f"{dongle}_{route_hex}--{route_uuid}"

  rec = {
    'route': route,
    'seg': seg,
    'path': os.path.basename(path),
    # T2-B
    'cb_tx_total': 0,
    'cb_signal_counts': {'cancel': 0, 'resume': 0, 'accel': 0, 'decel': 0,
                         'dist_dec': 0, 'dist_inc': 0,
                         'cruise_onoff': 0, 'acc_onoff': 0, 'idle': 0},
    'cb_rx_count': 0,
    'cb_rx_first_t': None,
    'cb_bus_hist': {},
    'cb_byte0_hist': {},
    'cb_counter_seq_first200': [],
    'cb_counter_delta_hist': {},
    'cb_boot_storm_count_t_lt_10s': 0,
    'cb_first_tx_t': None,
    # T3-C — per addr
    'rx': {a: {
      'count': 0,
      'iat_p95_ms': 0.0,
      'iat_p99_ms': 0.0,
      'iat_max_ms': 0.0,
      'iat_mean_ms': 0.0,
      'gap_over_100ms': 0,
      'gap_over_500ms': 0,
      'gap_over_threshold': 0,   # over 3× expected period
      'first_t': None,
    } for a in RX_ADDRS},
    'safety_rx_invalid_rising': 0,
    'safety_rx_invalid_first_t': None,
    'safety_rx_invalid_samples_true': 0,
    't0_mono': None,
    't_last': None,
  }

  rx_iat = {a: [] for a in RX_ADDRS}
  rx_last_t = {a: None for a in RX_ADDRS}
  cruise_counters = []

  prev_safety_invalid = False
  t0 = None

  try:
    lr = _LogFileReader(path)
  except Exception as e:
    rec['error'] = f'LogReader open: {type(e).__name__}: {e}'
    return rec

  try:
    for msg in lr:
      try:
        t_ns = msg.logMonoTime
      except Exception:
        continue
      if t0 is None:
        t0 = t_ns
        rec['t0_mono'] = t0
      t = (t_ns - t0) / 1e9
      rec['t_last'] = t

      which = msg.which()

      if which == 'sendcan':
        for c in msg.sendcan:
          if c.address == CRUISE_BUTTONS_ADDR:
            rec['cb_tx_total'] += 1
            rec['cb_bus_hist'][c.src] = rec['cb_bus_hist'].get(c.src, 0) + 1
            b0, sigs, ctr = parse_cruise_buttons_byte0(c.dat)
            if b0 is not None:
              key = f'0x{b0:02x}'
              rec['cb_byte0_hist'][key] = rec['cb_byte0_hist'].get(key, 0) + 1
            if sigs:
              any_pressed = False
              for k, v in sigs.items():
                if v:
                  rec['cb_signal_counts'][k] += 1
                  any_pressed = True
              if not any_pressed:
                rec['cb_signal_counts']['idle'] += 1
            if ctr is not None:
              cruise_counters.append(ctr)
            if t < 10.0:
              rec['cb_boot_storm_count_t_lt_10s'] += 1
            if rec['cb_first_tx_t'] is None:
              rec['cb_first_tx_t'] = t

      elif which == 'can':
        for c in msg.can:
          if c.src != 0:
            continue
          if c.address == CRUISE_BUTTONS_ADDR:
            rec['cb_rx_count'] += 1
            if rec['cb_rx_first_t'] is None:
              rec['cb_rx_first_t'] = t
          if c.address in RX_ADDRS:
            last = rx_last_t[c.address]
            if last is not None:
              iat = (t - last) * 1000.0
              rx_iat[c.address].append(iat)
              if iat > 100.0:
                rec['rx'][c.address]['gap_over_100ms'] += 1
              if iat > 500.0:
                rec['rx'][c.address]['gap_over_500ms'] += 1
              # threshold = 3× expected period
              hz = ADDR_HZ[c.address]
              thresh_ms = 3000.0 / hz
              if iat > thresh_ms:
                rec['rx'][c.address]['gap_over_threshold'] += 1
            rx_last_t[c.address] = t
            rec['rx'][c.address]['count'] += 1
            if rec['rx'][c.address]['first_t'] is None:
              rec['rx'][c.address]['first_t'] = t

      elif which == 'pandaStates':
        for ps in msg.pandaStates:
          inv = bool(getattr(ps, 'safetyRxChecksInvalid', False))
          if inv:
            rec['safety_rx_invalid_samples_true'] += 1
            if not prev_safety_invalid:
              rec['safety_rx_invalid_rising'] += 1
              if rec['safety_rx_invalid_first_t'] is None:
                rec['safety_rx_invalid_first_t'] = t
          prev_safety_invalid = inv
      elif which == 'pandaState':
        ps = msg.pandaState
        inv = bool(getattr(ps, 'safetyRxChecksInvalid', False))
        if inv:
          rec['safety_rx_invalid_samples_true'] += 1
          if not prev_safety_invalid:
            rec['safety_rx_invalid_rising'] += 1
            if rec['safety_rx_invalid_first_t'] is None:
              rec['safety_rx_invalid_first_t'] = t
        prev_safety_invalid = inv

  except Exception as e:
    rec['error'] = f'iter: {type(e).__name__}: {e}'

  # Aggregate iat stats
  for a in RX_ADDRS:
    arr = rx_iat[a]
    if arr:
      rec['rx'][a]['iat_mean_ms'] = round(sum(arr) / len(arr), 3)
      rec['rx'][a]['iat_p95_ms']  = round(percentile(arr, 0.95), 3)
      rec['rx'][a]['iat_p99_ms']  = round(percentile(arr, 0.99), 3)
      rec['rx'][a]['iat_max_ms']  = round(max(arr), 3)

  # Cruise counter sequence + delta hist
  rec['cb_counter_seq_first200'] = cruise_counters[:200]
  if len(cruise_counters) >= 2:
    deltas = Counter()
    for a, b in zip(cruise_counters[:-1], cruise_counters[1:]):
      d = (b - a) & 0x0F   # 4-bit wraparound
      deltas[d] += 1
    rec['cb_counter_delta_hist'] = {str(k): v for k, v in sorted(deltas.items())}

  return rec


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--drivelog-dir', required=True)
  ap.add_argument('--out', required=True)
  ap.add_argument('--limit', type=int, default=0)
  args = ap.parse_args()

  paths = sorted(glob.glob(os.path.join(args.drivelog_dir, '*--rlog.zst')))
  if args.limit:
    paths = paths[:args.limit]
  print(f'Found {len(paths)} rlog files', file=sys.stderr)

  os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
  out_records = []
  for i, p in enumerate(paths):
    if i % 25 == 0:
      print(f'  {i}/{len(paths)}', file=sys.stderr)
    rec = process_segment(p)
    if rec is not None:
      out_records.append(rec)

  with open(args.out, 'w') as f:
    json.dump(out_records, f)
  print(f'Wrote {len(out_records)} records to {args.out}', file=sys.stderr)


if __name__ == '__main__':
  main()
