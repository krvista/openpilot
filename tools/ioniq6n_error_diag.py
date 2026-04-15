#!/usr/bin/env python3
"""Diagnose 'Unknown Vehicle Variant' + ADAS flicker on route 00000030.

Looks for:
  * onroadEvents / carEvents (any alerts, especially unknown/relayMalfunction)
  * controlsState alert fields
  * managerState / initData for startup info
  * carParams fingerprint
  * Safety-relevant CAN addresses seen on each bus
  * Any Panda USB/safety health flags (relayMalfunction, safetyRxChecksInvalid)
"""
import glob
import sys
from collections import Counter, defaultdict
import zstandard as zstd

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')

from cereal import log

ROUTE = '00000030'
DRIVELOG_DIR = '/home/user/openpilot/drivelog'


def _safe(fn, default=None):
  try:
    return fn()
  except Exception:
    return default


def scan_seg(path, out):
  with open(path, 'rb') as f:
    raw = zstd.ZstdDecompressor().decompress(f.read(), max_output_size=500 * 1024 * 1024)

  for m in log.Event.read_multiple_bytes(raw):
    try:
      w = m.which()
    except Exception:
      continue

    try:
      if w == 'onroadEvents':
        for ev in m.onroadEvents:
          out['events'][_safe(lambda: str(ev.name), 'unknown')] += 1

      elif w == 'carParams':
        if out['carParams'] is None:
          cp = m.carParams
          out['carParams'] = {
            'carFingerprint': _safe(lambda: cp.carFingerprint, ''),
            'carVin': _safe(lambda: cp.carVin, ''),
            'alternativeExperience': _safe(lambda: cp.alternativeExperience, 0),
            'notCar': _safe(lambda: cp.notCar, False),
          }

      elif w == 'pandaStates':
        for i, ps in enumerate(m.pandaStates):
          if _safe(lambda: ps.safetyRxChecksInvalid, False):
            out['panda'][f'p{i}_safetyRxChecksInvalid'] += 1
          if _safe(lambda: ps.controlsAllowed, True):
            out['panda'][f'p{i}_controlsAllowed'] += 1
          sm = _safe(lambda: str(ps.safetyModel), 'err')
          out['panda_safetyModel'][f'p{i}_sm={sm}'] += 1
          sp = _safe(lambda: int(ps.safetyParam), -1)
          out['panda_safetyParam'][f'p{i}_sp={sp}'] += 1
          fs = _safe(lambda: int(ps.faultStatus), 0)
          if fs:
            out['panda'][f'p{i}_faultStatus_nonzero'] += 1
            out['panda_faultStatus_vals'][f'p{i}_fs={fs}'] += 1
          for f in _safe(lambda: list(ps.faults), []) or []:
            out['panda_faults'][f'p{i}:{f}'] += 1
          if _safe(lambda: ps.ignitionCan, True):
            out['panda'][f'p{i}_ignitionCan_true'] += 1
          out['panda'][f'p{i}_total'] += 1

      elif w == 'carState':
        # Check for carState.canValid flag
        if _safe(lambda: m.carState.canValid, True):
          out['cs']['canValid_true'] += 1
        if _safe(lambda: m.carState.canTimeout, False):
          out['cs']['canTimeout_true'] += 1
        err = _safe(lambda: m.carState.canErrorCounter, 0)
        out['cs']['canErrorCounter_last'] = err
        out['cs']['total'] += 1

      elif w == '__SKIP__carState':  # handled above
        pass

      elif w == 'controlsState':
        if _safe(lambda: m.controlsState.enabled, False):
          out['ct']['enabled'] += 1
        if _safe(lambda: m.controlsState.active, False):
          out['ct']['active'] += 1
        out['ct']['total'] += 1
        t = _safe(lambda: m.controlsState.alertText1, '')
        if t:
          out['alertText1'][t] += 1

      elif w == 'selfdriveState':
        t = _safe(lambda: m.selfdriveState.alertText1, '')
        if t:
          out['alertText1'][t] += 1
        if _safe(lambda: m.selfdriveState.enabled, False):
          out['ss']['enabled'] += 1
        out['ss']['total'] += 1

      elif w == 'can':
        for c in m.can:
          key = (_safe(lambda: c.src, 0), _safe(lambda: c.address, 0))
          out['can_addrs'][key] += 1
    except Exception as ex:
      out['errors'][f'{w}: {type(ex).__name__}'] += 1
      continue


def main():
  segs = sorted(glob.glob(f'{DRIVELOG_DIR}/*{ROUTE}*rlog.zst'))
  print(f"Route {ROUTE}: {len(segs)} segments")

  out = {
    'events': Counter(),
    'alertText1': Counter(),
    'can_addrs': Counter(),
    'panda': Counter(),
    'panda_faults': Counter(),
    'panda_faultStatus_vals': Counter(),
    'panda_safetyModel': Counter(),
    'panda_safetyParam': Counter(),
    'cs': Counter(),
    'ct': Counter(),
    'ss': Counter(),
    'errors': Counter(),
    'carParams': None,
  }

  for p in segs:
    print(f"  scan {p.split('/')[-1]}")
    try:
      scan_seg(p, out)
    except Exception as e:
      print(f"    ERR: {e}")

  print("\n=== carParams ===")
  print(out['carParams'])

  print("\n=== onroadEvents (top 30) ===")
  for name, cnt in out['events'].most_common(30):
    print(f"  {name:<40}  {cnt:>6}")

  print("\n=== controlsState.alertText1 ===")
  for t, cnt in out['alertText1'].most_common(20):
    print(f"  [{cnt:>5}]  {t}")

  print("\n=== pandaStates per-panda flags ===")
  for k, cnt in sorted(out['panda'].items()):
    print(f"  {k:<40}  {cnt}")
  print("\n=== safetyModel (0=NOOUTPUT, 8=ALLOUTPUT, 28=HYUNDAI_CANFD) ===")
  for k, cnt in out['panda_safetyModel'].most_common():
    print(f"  {k:<30}  {cnt}")
  print("\n=== safetyParam ===")
  for k, cnt in out['panda_safetyParam'].most_common():
    print(f"  {k:<30}  {cnt}")
  print("\n=== pandaStates faultStatus values (1=relayMalfunction, 2=unusedInterrupt, 3=inputVoltageCritical) ===")
  for v, cnt in out['panda_faultStatus_vals'].most_common():
    print(f"  faultStatus={v}  count={cnt}")
  print("\n=== pandaStates faults list entries ===")
  for k, cnt in out['panda_faults'].most_common():
    print(f"  {k}  {cnt}")

  print("\n=== carState / controlsState / selfdriveState totals ===")
  print(f"  cs: {dict(out['cs'])}")
  print(f"  ct: {dict(out['ct'])}")
  print(f"  ss: {dict(out['ss'])}")

  print("\n=== parse errors (per message type) ===")
  for k, cnt in out['errors'].most_common(15):
    print(f"  {k:<40}  {cnt}")

  print("\n=== CAN address counts per bus (top 40) ===")
  by_bus = defaultdict(Counter)
  for (src, addr), cnt in out['can_addrs'].items():
    by_bus[src][addr] = cnt
  for src in sorted(by_bus):
    print(f"\n  --- bus {src} ---")
    for addr, cnt in by_bus[src].most_common(30):
      print(f"    0x{addr:03x} ({addr:>4})  {cnt:>7}")

  # Particular focus: did WE TX 0x161 / 0x162 on bus 1?
  print("\n=== DEBUG: 0x161 / 0x162 TX presence ===")
  for addr in (0x161, 0x162, 0x110, 0x50, 0x2a4):
    print(f"  0x{addr:03x} on bus0={out['can_addrs'].get((0,addr),0):>6}  "
          f"bus1={out['can_addrs'].get((1,addr),0):>6}  "
          f"bus2={out['can_addrs'].get((2,addr),0):>6}  "
          f"bus128={out['can_addrs'].get((128,addr),0):>6}  "
          f"bus129={out['can_addrs'].get((129,addr),0):>6}  "
          f"bus130={out['can_addrs'].get((130,addr),0):>6}")


if __name__ == '__main__':
  main()
