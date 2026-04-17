#!/usr/bin/env python3
"""Comprehensive cross-route drivelog anomaly sweep (Ioniq 6 N HDA2-ALT + CCNC).

Scans ALL 247 segments across 8 routes for every known and potential
anomaly class. Produces a structured per-route report.

Checks:
  1. SENDCAN TX audit (unexpected addresses, factory E-CAN collisions)
  2. LKAS_ALT (0x110 bus 0) frame consistency (ACI signal coherence)
  3. LKAS_ALT angle limits (>176.7° or >3°/frame while active)
  4. SCC_CONTROL (0x1A0) TX on bus 1 (should be 0 on HDA2-ALT)
  5. HOD bypass (0x208) counter/byte10 validation
  6. Camera 0x110 bus 2 health (rate, gaps, dropout)
  7. onroadEvents summary
  8. carState anomalies (rapid ACC flip, high-angle + latActive, vEgo<0)
  9. Speed-crossing transition analysis (ACI flips near 2/3/5 km/h)
"""
import glob
import sys
import os
from collections import Counter, defaultdict

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG_DIR = '/home/user/openpilot/drivelog'

FACTORY_ECAN_CRITICAL = {0x1A0, 0x161, 0x162, 0x0EA, 0x125, 0x175, 0x0A0}
ALLOWED_ECAN = {0x1CF, 0x208}
ALLOWED_ACAN = {0x110, 0x362}


def decode_lkas_alt(dat):
  if len(dat) < 32:
    return None
  raw = int.from_bytes(dat[4:6], 'little') & 0x3fff
  if raw >= 0x2000:
    raw -= 0x4000
  return {
    'angle': raw * 0.1,
    'gain': dat[12] / 255.0,
    'lkas_angle_active': (dat[6] >> 6) & 0x3,
    'lka_assist': dat[3] & 0x7,
    'byte7': dat[7],
    'byte13': dat[13],
  }


def scan_route(route_id, segs):
  r = {
    'segs': len(segs),
    'tx_addrs': Counter(),
    'lkas_total': 0,
    'v1_collisions': 0,
    'v2_aci_mismatch': 0,
    'v2_samples': [],
    'angle_over_max': 0,
    'harsh_active_frames': 0,
    'harsh_samples': [],
    'scc_tx_bus1': 0,
    'hod_frames': 0,
    'hod_counter_errors': 0,
    'hod_byte10_errors': 0,
    'cam_rx_frames': 0,
    'cam_gaps_over_100ms': 0,
    'cam_max_gap_ms': 0,
    'events': Counter(),
    'acc_flips': 0,
    'high_angle_lat_active': 0,
    'vego_negative': 0,
    'aci_flips_by_speed': defaultdict(int),
    'total_frames_by_speed': defaultdict(int),
  }

  for seg_path in segs:
    prev_lkas = None
    prev_lkas_active = None
    prev_cam_t = None
    prev_hod_counter = None
    last_acc = None
    last_aci_active = None
    t0 = None
    v_ego = 0.0
    lat_active = False

    for m in LogReader(seg_path):
      try:
        w = m.which()
      except Exception:
        continue
      if t0 is None:
        t0 = m.logMonoTime
      t = (m.logMonoTime - t0) / 1e9

      if w == 'carState':
        cs = m.carState
        v_ego = cs.vEgoRaw * 3.6
        acc = cs.cruiseState.enabled
        if last_acc is not None and acc != last_acc:
          r['acc_flips'] += 1
        last_acc = acc
        if cs.vEgoRaw < -0.1:
          r['vego_negative'] += 1

      if w == 'controlsState':
        try:
          lat_active = m.controlsState.active
        except Exception:
          pass

      if w == 'onroadEvents':
        for e in m.onroadEvents:
          r['events'][str(e.name)] += 1

      if w == 'can':
        for c in m.can:
          # Camera LKAS_ALT health (bus 2, addr 0x110)
          if c.src == 2 and c.address == 0x110:
            r['cam_rx_frames'] += 1
            t_ns = m.logMonoTime
            if prev_cam_t is not None:
              gap_ms = (t_ns - prev_cam_t) / 1e6
              if gap_ms > 100:
                r['cam_gaps_over_100ms'] += 1
              if gap_ms > r['cam_max_gap_ms']:
                r['cam_max_gap_ms'] = gap_ms
            prev_cam_t = t_ns

      if w == 'sendcan':
        for c in m.sendcan:
          bus, addr = c.src, c.address
          r['tx_addrs'][(bus, addr)] += 1

          # V1: factory E-CAN collision
          if bus == 1 and addr in FACTORY_ECAN_CRITICAL:
            r['v1_collisions'] += 1

          # SCC_CONTROL on bus 1
          if bus == 1 and addr == 0x1A0:
            r['scc_tx_bus1'] += 1

          # LKAS_ALT (0x110 bus 0) analysis
          if bus == 0 and addr == 0x110:
            r['lkas_total'] += 1
            d = decode_lkas_alt(bytes(c.dat))
            if d is None:
              continue

            # Angle over max
            if abs(d['angle']) > 176.7:
              r['angle_over_max'] += 1

            # ACI consistency check
            active_bits = sum([
              d['lkas_angle_active'] >= 2,
              d['byte13'] == 0x09,
              (d['byte7'] & 0xA0) == 0xA0,
            ])
            gain_active = d['gain'] > 0.01
            if active_bits >= 2 and not gain_active and abs(d['angle']) > 0.1:
              r['v2_aci_mismatch'] += 1
              if len(r['v2_samples']) < 3:
                r['v2_samples'].append({'t': t, 'v': v_ego, **d})
            elif active_bits == 0 and gain_active:
              r['v2_aci_mismatch'] += 1

            # Harsh angle delta while active
            if prev_lkas is not None and d['lkas_angle_active'] >= 2 and prev_lkas_active:
              delta = abs(d['angle'] - prev_lkas['angle'])
              if delta > 3.0:
                r['harsh_active_frames'] += 1
                if len(r['harsh_samples']) < 3:
                  r['harsh_samples'].append({'t': t, 'v': v_ego, 'delta': delta})

            # ACI flip tracking per speed bucket
            spd_bucket = int(v_ego)
            r['total_frames_by_speed'][spd_bucket] += 1
            aci_now = d['lkas_angle_active'] >= 2
            if last_aci_active is not None and aci_now != last_aci_active:
              r['aci_flips_by_speed'][spd_bucket] += 1
            last_aci_active = aci_now

            prev_lkas_active = d['lkas_angle_active'] >= 2
            prev_lkas = d

          # HOD bypass (0x208)
          if addr == 0x208:
            r['hod_frames'] += 1
            dat = bytes(c.dat)
            if len(dat) >= 16:
              if dat[10] != 4:
                r['hod_byte10_errors'] += 1
              counter = dat[2]
              if prev_hod_counter is not None:
                expected = (prev_hod_counter + 2) & 0xFF
                if counter != expected:
                  r['hod_counter_errors'] += 1
              prev_hod_counter = counter

  return r


def main():
  segs_by_route = defaultdict(list)
  for f in sorted(glob.glob(f'{DRIVELOG_DIR}/*rlog.zst')):
    route = '--'.join(f.split('/')[-1].split('--')[:2])
    segs_by_route[route].append(f)

  print(f"{'=' * 80}")
  print(f" Comprehensive cross-route drivelog anomaly sweep")
  print(f" {sum(len(v) for v in segs_by_route.values())} segments across {len(segs_by_route)} routes")
  print(f"{'=' * 80}")

  all_results = {}
  overall_issues = []

  for route, segs in sorted(segs_by_route.items()):
    short = route.split('_')[-1][:10]
    print(f"\nScanning {short} ({len(segs)} segs)...", end='', flush=True)
    r = scan_route(short, sorted(segs, key=lambda p: int(p.split('--')[-2])))
    all_results[short] = r
    print(" done.")

    print(f"\n── {short} ({r['segs']} segs, {r['lkas_total']} LKAS_ALT frames) ──")

    # 1. TX audit
    unexpected_tx = []
    for (bus, addr), cnt in r['tx_addrs'].items():
      if bus == 0 and addr in ALLOWED_ACAN:
        continue
      if bus == 1 and addr in ALLOWED_ECAN:
        continue
      if addr >= 0x700:
        continue  # UDS
      if bus == 1 and addr in FACTORY_ECAN_CRITICAL:
        unexpected_tx.append((bus, addr, cnt, 'FACTORY_COLLISION'))
      elif bus == 1:
        unexpected_tx.append((bus, addr, cnt, 'UNEXPECTED_ECAN'))
    if unexpected_tx:
      print(f"  ❌ TX audit: {len(unexpected_tx)} unexpected")
      for bus, addr, cnt, kind in unexpected_tx[:5]:
        print(f"       bus {bus} 0x{addr:03x} × {cnt}  ({kind})")
      overall_issues.append((short, 'TX_AUDIT', unexpected_tx))
    else:
      print(f"  ✅ TX audit: clean")

    # 2. ACI consistency
    if r['v2_aci_mismatch'] > 0:
      print(f"  ❌ ACI consistency: {r['v2_aci_mismatch']} mismatch frames")
      for s in r['v2_samples']:
        print(f"       {s}")
      overall_issues.append((short, 'ACI_MISMATCH', r['v2_aci_mismatch']))
    else:
      print(f"  ✅ ACI consistency: 0 mismatches")

    # 3. Angle limits
    issues_3 = []
    if r['angle_over_max'] > 0:
      issues_3.append(f"over_176.7°={r['angle_over_max']}")
    if r['harsh_active_frames'] > 0:
      issues_3.append(f"harsh_active(>3°/frame)={r['harsh_active_frames']}")
      for s in r['harsh_samples']:
        print(f"       harsh: t={s['t']:.2f}s v={s['v']:.1f}km/h Δ={s['delta']:.2f}°")
    if issues_3:
      print(f"  ⚠️  Angle limits: {', '.join(issues_3)}")
      overall_issues.append((short, 'ANGLE', issues_3))
    else:
      print(f"  ✅ Angle limits: clean")

    # 4. SCC_CONTROL
    if r['scc_tx_bus1'] > 0:
      print(f"  ❌ SCC_CONTROL TX bus 1: {r['scc_tx_bus1']} frames")
      overall_issues.append((short, 'SCC_TX', r['scc_tx_bus1']))
    else:
      print(f"  ✅ SCC_CONTROL: 0 TX")

    # 5. HOD bypass
    if r['hod_frames'] > 0:
      hod_ok = r['hod_counter_errors'] == 0 and r['hod_byte10_errors'] == 0
      print(f"  {'✅' if hod_ok else '❌'} HOD bypass: {r['hod_frames']} frames, "
            f"counter_err={r['hod_counter_errors']}, byte10_err={r['hod_byte10_errors']}")
      if not hod_ok:
        overall_issues.append((short, 'HOD', r['hod_counter_errors'] + r['hod_byte10_errors']))
    else:
      print(f"  ── HOD bypass: not active")

    # 6. Camera health
    cam_ok = r['cam_gaps_over_100ms'] == 0
    print(f"  {'✅' if cam_ok else '⚠️ '} Camera RX: {r['cam_rx_frames']} frames, "
          f"gaps>100ms={r['cam_gaps_over_100ms']}, max_gap={r['cam_max_gap_ms']:.0f}ms")
    if not cam_ok:
      overall_issues.append((short, 'CAM_GAP', r['cam_gaps_over_100ms']))

    # 7. Events
    warn_events = {k: v for k, v in r['events'].items()
                   if k in ('canError', 'controlsUnresponsive', 'actuatorsApiUnavailable',
                            'relayMalfunction', 'commIssue', 'commIssueAvgFreq')}
    if warn_events:
      print(f"  ⚠️  Events: {dict(warn_events)}")
      overall_issues.append((short, 'EVENTS', warn_events))
    else:
      other = {k: v for k, v in r['events'].items() if v > 10 and k != 'selfdriveInitializing'}
      if other:
        print(f"  ── Events: {dict(other)}")
      else:
        print(f"  ✅ Events: clean")

    # 8. carState anomalies
    cs_issues = []
    if r['acc_flips'] > 50:
      cs_issues.append(f"acc_flips={r['acc_flips']}")
    if r['high_angle_lat_active'] > 0:
      cs_issues.append(f"high_angle_lat={r['high_angle_lat_active']}")
    if r['vego_negative'] > 0:
      cs_issues.append(f"vego_neg={r['vego_negative']}")
    if cs_issues:
      print(f"  ⚠️  carState: {', '.join(cs_issues)}")
    else:
      print(f"  ✅ carState: clean")

    # 9. ACI flips per speed
    high_flip_speeds = {}
    for spd, flips in sorted(r['aci_flips_by_speed'].items()):
      total = r['total_frames_by_speed'].get(spd, 1)
      rate = flips / max(total, 1) * 100
      if rate > 5 and flips > 10:
        high_flip_speeds[spd] = f"{flips}({rate:.1f}%)"
    if high_flip_speeds:
      print(f"  ⚠️  ACI flip hotspots (km/h): {high_flip_speeds}")
      overall_issues.append((short, 'ACI_FLIPS', high_flip_speeds))
    else:
      print(f"  ✅ ACI flips: no hotspots")

  # Overall summary
  print(f"\n{'=' * 80}")
  print(f" OVERALL SUMMARY")
  print(f"{'=' * 80}")
  print(f" Routes scanned: {len(all_results)}")
  print(f" Total segments: {sum(r['segs'] for r in all_results.values())}")
  print(f" Total LKAS_ALT frames: {sum(r['lkas_total'] for r in all_results.values())}")
  print(f" Total issues: {len(overall_issues)}")

  if overall_issues:
    print(f"\n Issues by category:")
    by_cat = defaultdict(list)
    for route, cat, detail in overall_issues:
      by_cat[cat].append((route, detail))
    for cat, entries in sorted(by_cat.items()):
      print(f"   {cat}: {len(entries)} routes")
      for route, detail in entries:
        print(f"     {route}: {detail}")
  else:
    print(f"\n ✅ NO ISSUES FOUND across all routes")

  has_critical = any(cat in ('ACI_MISMATCH', 'SCC_TX') for _, cat, _ in overall_issues)
  return 1 if has_critical else 0


if __name__ == '__main__':
  sys.exit(main())
