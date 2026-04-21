#!/usr/bin/env python3
"""Route 0x49 (433dad5bb2) symptom correlation scan.

Three symptoms on new build test drive:
  A) Intermittent ADAS error + LFA icon disappear
  B) White (not green) LFA icon despite ACC+LFA ON — no steering assist
  C) Lane change doesn't smoothly hand over

Primary hypothesis: `blinker_on → authority *= 0.2` vs `ACI_ENTER=0.30`
hysteresis trap. This scan extracts per-frame time series and produces
evidence tables for each hypothesis.
"""

import glob
import struct
import sys
from collections import Counter, defaultdict

import numpy as np

sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG_DIR = '/home/user/openpilot/drivelog'
ROUTE_ID = 0x49
ROUTE_HASH = '433dad5bb2'

# Carcontroller constants (mirror from code)
ACI_ENTER = 0.30
ACI_EXIT = 0.05
ACI_SPEED_FULL_MS = 3.0 / 3.6
ACI_SPEED_ZERO_MS = 1.0 / 3.6
DRIVER_TORQUE_DEADZONE = 25
FULL_OVERRIDE_LOW_V_TORQUE = 60
FULL_OVERRIDE_LOW_V_SPEED = 8.0
FULL_OVERRIDE_HIGH_V_TORQUE = 120
FULL_OVERRIDE_HIGH_V_SPEED = 15.0
LOW_SPEED_PASSTHROUGH_ENTER_MS = 2.0 / 3.6
LOW_SPEED_PASSTHROUGH_EXIT_MS = 3.0 / 3.6


def compute_driver_torque_blend(steer_torque, v_ego):
    abs_torque = abs(steer_torque)
    full_override = np.interp(v_ego,
                              [FULL_OVERRIDE_LOW_V_SPEED, FULL_OVERRIDE_HIGH_V_SPEED],
                              [FULL_OVERRIDE_LOW_V_TORQUE, FULL_OVERRIDE_HIGH_V_TORQUE])
    if abs_torque < DRIVER_TORQUE_DEADZONE:
        return 1.0
    if abs_torque >= full_override:
        return 0.0
    return float(np.clip(1.0 - (abs_torque - DRIVER_TORQUE_DEADZONE) /
                         max(full_override - DRIVER_TORQUE_DEADZONE, 1.0), 0.0, 1.0))


def get_route_files():
    pattern = f'{DRIVELOG_DIR}/99b215d21bbf8735_{ROUTE_ID:08x}--{ROUTE_HASH}--*--rlog.zst'
    files = sorted(glob.glob(pattern))
    if not files:
        pattern = f'{DRIVELOG_DIR}/*{ROUTE_HASH}*rlog.zst'
        files = sorted(glob.glob(pattern))
    return files


def parse_lkas_alt_tx(dat_bytes):
    """Decode LKAS_ALT (0x110) TX fields from raw CAN bytes."""
    if len(dat_bytes) < 14:
        return None
    lkas_angle_active = (dat_bytes[9] >> 5) & 0x3
    lka_assist = (dat_bytes[7] >> 6) & 0x1
    acigain_raw = dat_bytes[12]
    acigain = acigain_raw / 254.0 if acigain_raw < 255 else 1.0
    angle_raw = (dat_bytes[10] << 8) | dat_bytes[11]
    if angle_raw > 0x7FFF:
        angle_raw -= 0x10000
    angle = angle_raw * 0.1
    return {
        'lkas_angle_active': lkas_angle_active,
        'lka_assist': lka_assist,
        'acigain': acigain,
        'angle': angle,
    }


def run_scan():
    files = get_route_files()
    if not files:
        print(f"ERROR: No rlog files found for route 0x{ROUTE_ID:02x} ({ROUTE_HASH})")
        return
    print(f"Route 0x{ROUTE_ID:02x} ({ROUTE_HASH}): {len(files)} segments")

    # ── Collectors ──
    frames = []           # per-frame state
    lkas_alt_tx = []      # raw CAN 0x110 TX
    onroad_events_all = Counter()
    onroad_events_ts = []

    # ── State for simulation ──
    aci_active_latched = False
    low_speed_cam_latched = False
    t0 = None

    for seg_idx, seg_path in enumerate(files):
        for m in LogReader(seg_path):
            try:
                w = m.which()
            except Exception:
                continue

            t_ns = m.logMonoTime
            if t0 is None:
                t0 = t_ns

            t_s = (t_ns - t0) / 1e9

            if w == 'carState':
                cs = m.carState
                frame = {
                    't_s': t_s,
                    'seg': seg_idx,
                    'vEgo': cs.vEgo,
                    'steeringTorque': cs.steeringTorque,
                    'steeringAngleDeg': cs.steeringAngleDeg,
                    'leftBlinker': cs.leftBlinker,
                    'rightBlinker': cs.rightBlinker,
                    'cruiseEnabled': cs.cruiseState.enabled,
                    'cruiseAvailable': cs.cruiseState.available,
                    'canValid': cs.canValid,
                    'steerFaultTemp': cs.steerFaultTemporary,
                    'steerFaultPerm': cs.steerFaultPermanent,
                }
                frames.append(frame)

            elif w == 'carControl':
                cc = m.carControl
                if frames:
                    f = frames[-1]
                    f['latActive'] = cc.latActive
                    f['longActive'] = cc.longActive
                    f['cc_leftBlinker'] = cc.leftBlinker
                    f['cc_rightBlinker'] = cc.rightBlinker

            elif w == 'carStateSP':
                csp = m.carStateSP
                if frames:
                    f = frames[-1]
                    f['mdpsLkaAngleActive'] = csp.mdpsLkaAngleActive
                    f['mdpsLkaAngleFault'] = csp.mdpsLkaAngleFault
                    f['mdpsCounter'] = csp.mdpsCounter

            elif w == 'modelV2':
                mv = m.modelV2
                if frames:
                    f = frames[-1]
                    f['laneChangeState'] = str(mv.meta.laneChangeState)
                    f['laneChangeDirection'] = str(mv.meta.laneChangeDirection)

            elif w == 'onroadEvents':
                for evt in m.onroadEvents:
                    name = str(evt.name)
                    onroad_events_all[name] += 1
                    onroad_events_ts.append({'t_s': t_s, 'name': name})

            elif w == 'can':
                for c in m.can:
                    if c.address == 0x110 and c.src == 0 and len(c.dat) >= 14:
                        parsed = parse_lkas_alt_tx(bytes(c.dat))
                        if parsed:
                            parsed['t_s'] = t_s
                            lkas_alt_tx.append(parsed)

    print(f"Parsed: {len(frames)} carState frames, {len(lkas_alt_tx)} LKAS_ALT TX, "
          f"{sum(onroad_events_all.values())} onroadEvents")

    # ── Simulate aci_active_latched, low_speed_cam_latched ──
    aci_active_latched = False
    low_speed_cam_latched = False
    for f in frames:
        v = f.get('vEgo', 0.0)
        lat_active = f.get('latActive', False)
        torque = f.get('steeringTorque', 0.0)
        blinker_on = f.get('leftBlinker', False) or f.get('rightBlinker', False)

        speed_blend = float(np.clip((v - ACI_SPEED_ZERO_MS) /
                                    (ACI_SPEED_FULL_MS - ACI_SPEED_ZERO_MS), 0.0, 1.0))
        dtb = compute_driver_torque_blend(torque, v)
        authority = dtb * speed_blend if lat_active else 0.0
        if blinker_on:
            authority *= 0.2

        if lat_active:
            if authority >= ACI_ENTER:
                aci_active_latched = True
            elif authority < ACI_EXIT:
                aci_active_latched = False
        else:
            aci_active_latched = False

        if v < LOW_SPEED_PASSTHROUGH_ENTER_MS:
            low_speed_cam_latched = True
        elif v > LOW_SPEED_PASSTHROUGH_EXIT_MS:
            low_speed_cam_latched = False

        steering_active = lat_active and aci_active_latched and speed_blend > 0.1

        f['speed_blend'] = speed_blend
        f['dtb'] = dtb
        f['authority'] = authority
        f['aci_active_latched'] = aci_active_latched
        f['low_speed_cam_latched'] = low_speed_cam_latched
        f['steering_active_sim'] = steering_active
        f['blinker_on'] = blinker_on

    # ══════════════════════════════════════════════════════════
    # Report 1: onroadEvents summary
    # ══════════════════════════════════════════════════════════
    print("\n" + "=" * 100)
    print(" REPORT 1: onroadEvents summary")
    print("=" * 100)
    for name, cnt in sorted(onroad_events_all.items(), key=lambda x: -x[1])[:25]:
        print(f"  {name:40s} {cnt:6d}")

    # ══════════════════════════════════════════════════════════
    # Report 2: LKAS_ANGLE_ACTIVE transitions (2↔1) from CAN TX
    # ══════════════════════════════════════════════════════════
    print("\n" + "=" * 100)
    print(" REPORT 2: LKAS_ANGLE_ACTIVE transitions in CAN TX (0x110)")
    print("=" * 100)
    transitions = []
    for i in range(1, len(lkas_alt_tx)):
        prev = lkas_alt_tx[i - 1]['lkas_angle_active']
        curr = lkas_alt_tx[i]['lkas_angle_active']
        if prev != curr:
            transitions.append({
                't_s': lkas_alt_tx[i]['t_s'],
                'from': prev,
                'to': curr,
                'acigain': lkas_alt_tx[i]['acigain'],
                'angle': lkas_alt_tx[i]['angle'],
            })

    print(f"  Total LKAS_ANGLE_ACTIVE transitions: {len(transitions)}")
    for tr in transitions[:50]:
        # Find nearest carState frame for context
        ctx = "?"
        for f in frames:
            if abs(f['t_s'] - tr['t_s']) < 0.1:
                ctx = (f"v={f.get('vEgo',0):.1f} latAct={f.get('latActive','-')} "
                       f"blink={f.get('blinker_on','-')} dtb={f.get('dtb',0):.2f} "
                       f"auth={f.get('authority',0):.3f} aci_latch={f.get('aci_active_latched','-')} "
                       f"lcState={f.get('laneChangeState','?')}")
                break
        print(f"  t={tr['t_s']:8.2f}s  {tr['from']}→{tr['to']}  "
              f"ACIgain={tr['acigain']:.3f}  angle={tr['angle']:.1f}°  || {ctx}")

    # ══════════════════════════════════════════════════════════
    # Report 3: Blinker events + aci_active_latched evolution
    # ══════════════════════════════════════════════════════════
    print("\n" + "=" * 100)
    print(" REPORT 3: Blinker events + aci_active_latched evolution")
    print("=" * 100)
    blinker_events = []
    in_blinker = False
    blinker_start_idx = 0
    for i, f in enumerate(frames):
        if f.get('blinker_on', False) and not in_blinker:
            in_blinker = True
            blinker_start_idx = i
        elif not f.get('blinker_on', False) and in_blinker:
            in_blinker = False
            blinker_events.append((blinker_start_idx, i - 1))

    print(f"  Total blinker-ON events: {len(blinker_events)}")
    for be_idx, (start, end) in enumerate(blinker_events):
        dur = frames[end]['t_s'] - frames[start]['t_s']
        aci_at_start = frames[start].get('aci_active_latched', None)
        lat_at_start = frames[start].get('latActive', None)
        v_at_start = frames[start].get('vEgo', 0)
        lc_states = set()
        aci_false_count = 0
        aci_true_count = 0
        min_auth = 1.0
        min_dtb = 1.0
        for i in range(start, min(end + 1, len(frames))):
            f = frames[i]
            lc_states.add(f.get('laneChangeState', '?'))
            if f.get('aci_active_latched', False):
                aci_true_count += 1
            else:
                aci_false_count += 1
            min_auth = min(min_auth, f.get('authority', 1.0))
            min_dtb = min(min_dtb, f.get('dtb', 1.0))
        total = aci_true_count + aci_false_count
        pct_false = 100 * aci_false_count / max(total, 1)

        dropped = "YES ← TRAP" if (aci_at_start and aci_false_count > 0) else \
                  "no (was already off)" if not aci_at_start else "no"

        print(f"\n  Blinker #{be_idx}: t={frames[start]['t_s']:.1f}–{frames[end]['t_s']:.1f}s  "
              f"dur={dur:.1f}s  v={v_at_start:.1f} m/s  latActive={lat_at_start}")
        print(f"    LC states seen: {lc_states}")
        print(f"    aci_active_latched: True={aci_true_count}, False={aci_false_count} "
              f"({pct_false:.0f}% off)")
        print(f"    min authority={min_auth:.3f}  min dtb={min_dtb:.2f}")
        print(f"    ACI dropped during blinker? {dropped}")

    # ══════════════════════════════════════════════════════════
    # Report 4: steering_active_sim vs latActive discrepancies
    # ══════════════════════════════════════════════════════════
    print("\n" + "=" * 100)
    print(" REPORT 4: Frames where latActive=True but steering_active=False (sim)")
    print("=" * 100)
    discrepant = []
    for f in frames:
        if f.get('latActive', False) and not f.get('steering_active_sim', True):
            discrepant.append(f)
    print(f"  Total discrepant frames: {len(discrepant)} / {len(frames)} "
          f"({100*len(discrepant)/max(len(frames),1):.1f}%)")
    reasons = Counter()
    for f in discrepant:
        if not f.get('aci_active_latched', True):
            if f.get('blinker_on', False):
                reasons['blinker_authority_trap'] += 1
            elif f.get('dtb', 1.0) < 0.5:
                reasons['driver_override'] += 1
            else:
                reasons['low_speed_blend'] += 1
        elif f.get('speed_blend', 1.0) <= 0.1:
            reasons['speed_below_1kph'] += 1
        else:
            reasons['other'] += 1
    for reason, cnt in sorted(reasons.items(), key=lambda x: -x[1]):
        print(f"    {reason:35s} {cnt:6d}")

    # Sample discrepant frames (first 10 + mid + last)
    sample_indices = list(range(min(5, len(discrepant))))
    if len(discrepant) > 10:
        sample_indices += [len(discrepant) // 2]
    if len(discrepant) > 5:
        sample_indices += [len(discrepant) - 1]
    for idx in sample_indices:
        f = discrepant[idx]
        print(f"    t={f['t_s']:8.2f}s v={f.get('vEgo',0):.1f} blink={f.get('blinker_on','-')} "
              f"dtb={f.get('dtb',0):.2f} auth={f.get('authority',0):.3f} "
              f"aci_latch={f.get('aci_active_latched','-')} "
              f"spd_blend={f.get('speed_blend',0):.2f} lc={f.get('laneChangeState','?')}")

    # ══════════════════════════════════════════════════════════
    # Report 5: Camera staleness proxy (mdpsCounter stalls)
    # ══════════════════════════════════════════════════════════
    print("\n" + "=" * 100)
    print(" REPORT 5: MDPS counter stalls (proxy for camera staleness)")
    print("=" * 100)
    counter_vals = [(f['t_s'], f.get('mdpsCounter', -1)) for f in frames
                    if f.get('mdpsCounter', -1) >= 0]
    stalls = []
    if counter_vals:
        run_start = 0
        for i in range(1, len(counter_vals)):
            if counter_vals[i][1] == counter_vals[i - 1][1]:
                continue
            run_len = i - run_start
            if run_len >= 25:
                stalls.append({
                    't_start': counter_vals[run_start][0],
                    't_end': counter_vals[i - 1][0],
                    'dur_frames': run_len,
                    'counter_val': counter_vals[run_start][1],
                })
            run_start = i
    print(f"  MDPS counter stalls >=25 frames: {len(stalls)}")
    for s in stalls[:10]:
        print(f"    t={s['t_start']:.2f}–{s['t_end']:.2f}s  dur={s['dur_frames']} frames  "
              f"counter={s['counter_val']}")

    # ══════════════════════════════════════════════════════════
    # Report 6: Low-speed passthrough flapping
    # ══════════════════════════════════════════════════════════
    print("\n" + "=" * 100)
    print(" REPORT 6: Low-speed passthrough latch flapping (1–3 km/h)")
    print("=" * 100)
    flaps = 0
    prev_latch = None
    for f in frames:
        cur_latch = f.get('low_speed_cam_latched', None)
        if prev_latch is not None and cur_latch is not None and cur_latch != prev_latch:
            flaps += 1
        prev_latch = cur_latch
    low_speed_frames = sum(1 for f in frames if 0.3 < f.get('vEgo', 0) < 1.5)
    print(f"  Latch transitions: {flaps}")
    print(f"  Frames in 1-5 km/h range: {low_speed_frames}")

    # ══════════════════════════════════════════════════════════
    # Report 7: Lane change state distribution
    # ══════════════════════════════════════════════════════════
    print("\n" + "=" * 100)
    print(" REPORT 7: Lane change state distribution")
    print("=" * 100)
    lc_counts = Counter()
    for f in frames:
        lc = f.get('laneChangeState', 'unknown')
        lc_counts[lc] += 1
    for lc, cnt in sorted(lc_counts.items(), key=lambda x: -x[1]):
        print(f"  {lc:25s} {cnt:6d}")

    # ══════════════════════════════════════════════════════════
    # Report 8: ADAS-related onroadEvents timeline
    # ══════════════════════════════════════════════════════════
    print("\n" + "=" * 100)
    print(" REPORT 8: ADAS-related onroadEvents timeline")
    print("=" * 100)
    adas_events = ['commIssue', 'steerSaturated', 'steerTempUnavailable',
                   'steerUnavailable', 'canError', 'processNotRunning',
                   'controlsUnresponsive', 'cameraMalfunction',
                   'steerFaultTemporary', 'steerFaultPermanent',
                   'controlsdLagging', 'actuatorsApiUnavailable']
    for evt in onroad_events_ts:
        if any(a in evt['name'] for a in adas_events):
            print(f"  t={evt['t_s']:8.2f}s  {evt['name']}")

    # ══════════════════════════════════════════════════════════
    # Summary
    # ══════════════════════════════════════════════════════════
    print("\n" + "=" * 100)
    print(" SUMMARY")
    print("=" * 100)
    print(f"  Drive duration: {frames[-1]['t_s'] - frames[0]['t_s']:.0f}s "
          f"({(frames[-1]['t_s'] - frames[0]['t_s'])/60:.1f} min)")
    print(f"  Total frames: {len(frames)}")
    lat_active_frames = sum(1 for f in frames if f.get('latActive', False))
    print(f"  latActive frames: {lat_active_frames} "
          f"({100*lat_active_frames/max(len(frames),1):.1f}%)")
    print(f"  Blinker events: {len(blinker_events)}")
    blinker_trap_events = sum(1 for be_idx, (start, end) in enumerate(blinker_events)
                              if frames[start].get('aci_active_latched', False) and
                              any(not frames[i].get('aci_active_latched', True)
                                  for i in range(start, min(end + 1, len(frames)))))
    print(f"  Blinker events with ACI trap: {blinker_trap_events}")
    print(f"  LKAS_ANGLE_ACTIVE transitions: {len(transitions)}")
    print(f"  latActive=True but steering_active=False: {len(discrepant)} frames")
    print(f"  Reasons: {dict(reasons)}")
    print(f"  MDPS counter stalls (>=25 frames): {len(stalls)}")
    print(f"  Low-speed latch flaps: {flaps}")
    print()


if __name__ == '__main__':
    run_scan()
