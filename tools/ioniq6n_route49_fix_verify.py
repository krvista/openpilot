#!/usr/bin/env python3
"""Route 0x49 before/after simulation of the driver-override threshold fix.

Reads the same rlog data as ioniq6n_route49_symptom_correlation.py but
simulates TWO variants of the driver-override logic side by side:

  OLD: DZ=25, Full=60/120 (torque-control calibration, current buggy path)
  NEW: DZ=100, Full=300/500 (angle-control calibration applied to ccnc_lka_alt)

  OLD blinker: authority *= 0.2 unconditional
  NEW blinker: authority *= 0.3 only when driver_torque_blend < 0.7

Expected outcome (success criteria):
  - DTB=0.0 fraction: OLD 47% → NEW <20%
  - DTB=1.0 fraction: OLD 28% → NEW >60%
  - steering_active=False during latActive: OLD 24% → NEW <10%
  - No transitions in DTB computation that would zero-divide / crash
"""

import glob
import sys
from collections import Counter

import numpy as np

sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG_DIR = '/home/user/openpilot/drivelog'
ROUTE_HASH = '433dad5bb2'

ACI_ENTER = 0.30
ACI_EXIT = 0.05
ACI_SPEED_FULL_MS = 3.0 / 3.6
ACI_SPEED_ZERO_MS = 1.0 / 3.6

# OLD (torque-control)
DZ_OLD, FULL_LO_OLD, FULL_HI_OLD = 25.0, 60.0, 120.0
# NEW (angle-control)
DZ_NEW, FULL_LO_NEW, FULL_HI_NEW = 100.0, 300.0, 500.0

LO_V, HI_V = 8.0, 15.0


def dtb(torque, v, dz, full_lo, full_hi):
    at = abs(torque)
    full = float(np.interp(v, [LO_V, HI_V], [full_lo, full_hi]))
    if at < dz:
        return 1.0
    if at >= full:
        return 0.0
    return float(np.clip(1.0 - (at - dz) / max(full - dz, 1.0), 0.0, 1.0))


def authority(latActive, d, v, blinker):
    sb = float(np.clip((v - ACI_SPEED_ZERO_MS) /
                       (ACI_SPEED_FULL_MS - ACI_SPEED_ZERO_MS), 0.0, 1.0))
    a = d * sb if latActive else 0.0
    return a, sb


def latch_evolve(latched, a, latActive):
    if latActive:
        if a >= ACI_ENTER:
            return True
        if a < ACI_EXIT:
            return False
    else:
        return False
    return latched


def get_files():
    pat = f'{DRIVELOG_DIR}/*{ROUTE_HASH}*rlog.zst'
    return sorted(glob.glob(pat))


def main():
    files = get_files()
    if not files:
        print("No rlogs found")
        return
    print(f"Loading {len(files)} segments...")

    frames = []
    for seg in files:
        for m in LogReader(seg):
            try:
                w = m.which()
            except Exception:
                continue
            if w == 'carState':
                cs = m.carState
                frames.append({
                    'vEgo': cs.vEgo,
                    'torque': cs.steeringTorque,
                    'left': cs.leftBlinker,
                    'right': cs.rightBlinker,
                    'latActive': False,
                })
            elif w == 'carControl':
                if frames:
                    frames[-1]['latActive'] = m.carControl.latActive

    print(f"  parsed {len(frames)} frames")

    # ── Simulate both variants ──
    old_latched = new_latched = False
    stats = {
        'old': {'dtb0': 0, 'dtb1': 0, 'dtb_mid': 0,
                'lat_on_steer_off': 0, 'lat_on': 0, 'latched_on': 0},
        'new': {'dtb0': 0, 'dtb1': 0, 'dtb_mid': 0,
                'lat_on_steer_off': 0, 'lat_on': 0, 'latched_on': 0},
    }
    transitions = {'old_enter': 0, 'old_exit': 0,
                   'new_enter': 0, 'new_exit': 0}
    prev_old = prev_new = False

    for f in frames:
        v = f['vEgo']
        t = f['torque']
        la = f['latActive']
        bk = f['left'] or f['right']

        d_old = dtb(t, v, DZ_OLD, FULL_LO_OLD, FULL_HI_OLD)
        d_new = dtb(t, v, DZ_NEW, FULL_LO_NEW, FULL_HI_NEW)

        a_old, sb = authority(la, d_old, v, bk)
        a_new, _ = authority(la, d_new, v, bk)

        if bk:
            a_old *= 0.2
            if d_new < 0.7:
                a_new *= 0.3

        old_latched = latch_evolve(old_latched, a_old, la)
        new_latched = latch_evolve(new_latched, a_new, la)

        for key, d, lat in (('old', d_old, old_latched), ('new', d_new, new_latched)):
            if la:
                stats[key]['lat_on'] += 1
                if d < 0.01:
                    stats[key]['dtb0'] += 1
                elif d > 0.99:
                    stats[key]['dtb1'] += 1
                else:
                    stats[key]['dtb_mid'] += 1
                steer_active = lat and sb > 0.1
                if lat:
                    stats[key]['latched_on'] += 1
                if not steer_active:
                    stats[key]['lat_on_steer_off'] += 1

        if old_latched != prev_old:
            transitions['old_enter' if old_latched else 'old_exit'] += 1
        if new_latched != prev_new:
            transitions['new_enter' if new_latched else 'new_exit'] += 1
        prev_old, prev_new = old_latched, new_latched

    print("\n" + "=" * 80)
    print(" DRIVER-OVERRIDE THRESHOLD FIX — BEFORE/AFTER")
    print("=" * 80)
    for variant in ('old', 'new'):
        s = stats[variant]
        n = s['lat_on']
        if n == 0:
            continue
        label = ('OLD (DZ=25, 60/120, blink×0.2 uncond)' if variant == 'old'
                 else 'NEW (DZ=100, 300/500, blink×0.3 iff DTB<0.7)')
        print(f"\n{label}")
        print(f"  latActive frames:          {n}")
        print(f"  DTB=0.0 (full override):   {s['dtb0']:>7d}  ({s['dtb0']/n*100:5.1f}%)")
        print(f"  DTB=1.0 (no override):     {s['dtb1']:>7d}  ({s['dtb1']/n*100:5.1f}%)")
        print(f"  DTB in (0,1) (ramping):    {s['dtb_mid']:>7d}  ({s['dtb_mid']/n*100:5.1f}%)")
        print(f"  aci_latched=True:          {s['latched_on']:>7d}  ({s['latched_on']/n*100:5.1f}%)")
        print(f"  latActive & steer_active=False: {s['lat_on_steer_off']:>7d}  "
              f"({s['lat_on_steer_off']/n*100:5.1f}%)")

    print(f"\nlatched transitions:  OLD enter={transitions['old_enter']:4d} "
          f"exit={transitions['old_exit']:4d}   "
          f"NEW enter={transitions['new_enter']:4d} exit={transitions['new_exit']:4d}")

    # Success criteria
    so, sn = stats['old'], stats['new']
    if so['lat_on'] > 0 and sn['lat_on'] > 0:
        old_dtb0 = so['dtb0'] / so['lat_on'] * 100
        new_dtb0 = sn['dtb0'] / sn['lat_on'] * 100
        old_dtb1 = so['dtb1'] / so['lat_on'] * 100
        new_dtb1 = sn['dtb1'] / sn['lat_on'] * 100
        print("\n" + "-" * 80)
        print(" SUCCESS CRITERIA CHECK")
        print("-" * 80)
        print(f"  DTB=0.0 drops <20%:       old={old_dtb0:.1f}% → new={new_dtb0:.1f}%  "
              f"{'PASS' if new_dtb0 < 20 else 'FAIL'}")
        print(f"  DTB=1.0 rises >60%:       old={old_dtb1:.1f}% → new={new_dtb1:.1f}%  "
              f"{'PASS' if new_dtb1 > 60 else 'FAIL'}")


if __name__ == '__main__':
    main()
