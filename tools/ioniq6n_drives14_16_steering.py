#!/usr/bin/env python3
"""Patch #11 post-merge analysis — drives 14, 15, 16.

Goals (from user's review):
  T2  Detect 60°+ steering events where the driver applies torque, and
      analyze what op does during the *return to center* phase when the
      driver releases. Want to know if op intervenes aggressively / causes
      vehicle to feel unstable in that window.
  T3  Verify CCNC_0x161.LFA_ICON now reaches GREEN(2) under MADS, i.e.
      the user-reported "normalized" cluster icon. PR #10/#11 ostensibly
      did nothing here — confirm the natural state.

For each turn event we extract:
  - peak |steeringAngleDeg|
  - peak |steeringTorque|
  - whether op was active (CC.latActive) at peak
  - the 1-second post-peak window: op-vs-driver torque trajectory,
    op's apply_angle command vs actual wheel angle, hands_off events.

Output: stdout summary + /tmp/drives14_16_steering.json.
"""
import glob
import json
import os
import sys
from collections import defaultdict, Counter

import numpy as np
import zstandard as zstd

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')
from cereal import log
from opendbc.can.parser import CANParser

DRIVELOG_DIR = '/home/user/openpilot/drivelog'
DBC = 'hyundai_canfd_generated'
ROUTES = ('00000014', '00000015', '00000016')

TURN_PEAK_DEG  = 60.0   # |wheel angle| to qualify as a "turn event"
SETTLE_DEG     = 5.0    # |wheel angle| considered back to center
TORQUE_HANDS_OFF = 30.0 # below this — driver effectively released
POST_PEAK_FRAMES = 300  # 3 s at 100 Hz


def extract(path):
    raw = zstd.ZstdDecompressor().decompress(open(path,'rb').read(),
                                              max_output_size=500*1024*1024)
    # Need LFA_ICON from bus 1, CCNC_0x161
    p161 = CANParser(DBC, [('CCNC_0x161', 0)], 1)
    latest_cs = None
    latest_sp = None
    latest_lfa_icon = 0
    frames = []
    for msg in log.Event.read_multiple_bytes(raw):
        w = msg.which()
        if w == 'carState':
            latest_cs = msg.carState
        elif w == 'selfdriveStateSP':
            latest_sp = msg.selfdriveStateSP
        elif w == 'can':
            msgs_b1 = [(c.address, bytes(c.dat), 1) for c in msg.can
                       if c.src == 1 and c.address == 353]
            if msgs_b1:
                p161.update([0, msgs_b1])
                v = p161.vl.get('CCNC_0x161', {})
                if v:
                    latest_lfa_icon = int(v.get('LFA_ICON', 0))
        elif w == 'carControl' and latest_cs is not None:
            cc = msg.carControl
            mads_en = bool(latest_sp.mads.enabled) if latest_sp else False
            mads_active = bool(latest_sp.mads.active) if latest_sp else False
            frames.append({
                'mono': msg.logMonoTime,
                'v_ms': float(latest_cs.vEgoRaw),
                'v_kmh': float(latest_cs.vEgoRaw * 3.6),
                'wheel': float(latest_cs.steeringAngleDeg),
                'torque': float(latest_cs.steeringTorque),
                'pressed': bool(latest_cs.steeringPressed),
                'op_angle': float(cc.actuators.steeringAngleDeg),
                'lat_active': bool(cc.latActive),
                'mads_en': mads_en,
                'mads_active': mads_active,
                'lfa_icon': latest_lfa_icon,
                'blinker_l': bool(latest_cs.leftBlinker),
                'blinker_r': bool(latest_cs.rightBlinker),
            })
    return frames


def find_turn_events(frames):
    """Identify peaks where |wheel| > TURN_PEAK_DEG, then track post-peak."""
    out = []
    i = 0
    n = len(frames)
    while i < n:
        if abs(frames[i]['wheel']) >= TURN_PEAK_DEG:
            # Find the peak in this contiguous region (until wheel drops below SETTLE_DEG)
            peak_i = i
            peak_v = abs(frames[i]['wheel'])
            j = i
            while j < n and abs(frames[j]['wheel']) >= SETTLE_DEG:
                if abs(frames[j]['wheel']) > peak_v:
                    peak_v = abs(frames[j]['wheel'])
                    peak_i = j
                j += 1
            # Post-peak: from peak_i to settle (wheel reaches < SETTLE)
            settle_i = min(peak_i + POST_PEAK_FRAMES, n - 1)
            for k in range(peak_i, min(n, peak_i + POST_PEAK_FRAMES)):
                if abs(frames[k]['wheel']) < SETTLE_DEG:
                    settle_i = k
                    break
            out.append({
                'start_i': i, 'peak_i': peak_i, 'settle_i': settle_i,
                'peak_wheel': frames[peak_i]['wheel'],
                'peak_torque': frames[peak_i]['torque'],
                'peak_pressed': frames[peak_i]['pressed'],
                'peak_lat_active': frames[peak_i]['lat_active'],
                'peak_mads_en': frames[peak_i]['mads_en'],
                'peak_v_kmh': frames[peak_i]['v_kmh'],
                'peak_blinker': frames[peak_i]['blinker_l'] or frames[peak_i]['blinker_r'],
                'duration_frames': settle_i - peak_i,
            })
            i = settle_i + 1
        else:
            i += 1
    return out


def analyze_return_to_center(frames, turn):
    """For one turn, examine post-peak return-to-center for op intervention."""
    a = turn['peak_i']
    b = turn['settle_i']
    if b - a < 5:
        return None
    seg = frames[a:b+1]

    # Find when driver releases (torque drops below 30 Nm for >5 frames)
    release_i = None
    for i in range(5, len(seg)):
        if all(abs(seg[i-k]['torque']) < TORQUE_HANDS_OFF for k in range(5)):
            release_i = i
            break

    if release_i is None:
        return {'mode': 'no_release', 'turn': turn}

    # Window: 50 frames (500 ms) AFTER driver release
    win_start = release_i
    win_end = min(release_i + 50, len(seg))
    win = seg[win_start:win_end]
    if len(win) < 5:
        return {'mode': 'no_window', 'turn': turn}

    op_active_in_window = any(f['lat_active'] for f in win)
    mads_in_window = any(f['mads_en'] for f in win)

    # Op's commanded angle deviation from actual wheel during release window
    op_dev = np.array([f['op_angle'] - f['wheel'] for f in win])
    wheel_traj = np.array([f['wheel'] for f in win])
    wheel_change = wheel_traj[-1] - wheel_traj[0]  # change over window

    # Did wheel snap back too fast? Compute wheel angular velocity
    if len(wheel_traj) >= 10:
        max_dwheel_per_frame = float(np.max(np.abs(np.diff(wheel_traj))))
    else:
        max_dwheel_per_frame = 0.0

    return {
        'mode': 'analyzed',
        'turn': turn,
        'release_offset_frames': release_i,
        'window_frames': len(win),
        'op_active_in_window': op_active_in_window,
        'mads_in_window': mads_in_window,
        'wheel_at_release_deg': float(seg[release_i]['wheel']),
        'wheel_at_window_end_deg': float(seg[release_i + len(win) - 1]['wheel']),
        'op_deviation_max_abs': float(np.max(np.abs(op_dev))),
        'op_deviation_p50_abs': float(np.median(np.abs(op_dev))),
        'wheel_max_step_per_frame_deg': max_dwheel_per_frame,
    }


def lfa_icon_summary(frames):
    """Per-(mads_state, icon_value) frame counts."""
    by_state = Counter()
    icons = Counter()
    transitions = []
    prev_icon = None
    for f in frames:
        state = 'mads_on' if f['mads_en'] else ('cruise_on' if False else 'off')
        # Just use mads_en for grouping; cruise state not parsed here.
        key = ('mads_on' if f['mads_en'] else 'mads_off', f['lfa_icon'])
        by_state[key] += 1
        icons[f['lfa_icon']] += 1
        if prev_icon is not None and f['lfa_icon'] != prev_icon:
            transitions.append({'from': prev_icon, 'to': f['lfa_icon'],
                                'mads_en': f['mads_en'], 'mono': f['mono']})
        prev_icon = f['lfa_icon']
    return {
        'icon_counts': dict(icons),
        'mads_x_icon_counts': {f"{k[0]}|icon={k[1]}": v for k, v in by_state.items()},
        'transitions_count': len(transitions),
        'reached_green_under_mads': sum(1 for t in transitions
                                        if t['to'] == 2 and t['mads_en']) > 0
                                    or (('mads_on', 2) in by_state and by_state[('mads_on', 2)] > 0),
    }


def main():
    paths = []
    for r in ROUTES:
        paths.extend(sorted(glob.glob(os.path.join(DRIVELOG_DIR, f'*_{r}--*--rlog.zst'))))
    print(f"Scanning {len(paths)} rlog segments…")

    by_route = defaultdict(lambda: {
        'frames': 0,
        'turn_events': [],
        'analyzed': [],
        'lfa_icon_counts': Counter(),
        'mads_x_icon': Counter(),
        'mads_on_green_frames': 0,
        'mads_on_gray_frames': 0,
        'mads_on_hidden_frames': 0,
        'mads_off_gray_frames': 0,
    })

    for i, p in enumerate(paths):
        route = os.path.basename(p).split('_')[1].split('--')[0]
        try:
            frames = extract(p)
        except Exception as e:
            print(f"  ERR {os.path.basename(p)}: {e}")
            continue
        r = by_route[route]
        r['frames'] += len(frames)
        # Turn events
        turns = find_turn_events(frames)
        for t in turns:
            r['turn_events'].append(t)
            ana = analyze_return_to_center(frames, t)
            if ana and ana.get('mode') == 'analyzed':
                r['analyzed'].append(ana)
        # LFA_ICON
        summ = lfa_icon_summary(frames)
        for k, v in summ['icon_counts'].items():
            r['lfa_icon_counts'][k] += v
        for k, v in summ['mads_x_icon_counts'].items():
            r['mads_x_icon'][k] += v
        if (i+1) % 10 == 0:
            print(f"  {i+1}/{len(paths)}")

    print("\n=== TASK 3 — LFA_ICON NORMALIZATION ===")
    for route in sorted(by_route.keys()):
        r = by_route[route]
        print(f"\n--- drive {route} ({r['frames']:,} frames) ---")
        print(f"  Icon distribution (0=HIDDEN, 1=GRAY, 2=GREEN, 3=WHITE, 5=CYAN):")
        for k, v in sorted(r['lfa_icon_counts'].items()):
            pct = 100.0 * v / r['frames']
            label = {0:'HIDDEN', 1:'GRAY', 2:'GREEN', 3:'WHITE', 5:'CYAN'}.get(k, f'??{k}')
            print(f"    {k} ({label:>6}): {v:>8,} frames  ({pct:5.1f}%)")
        print(f"  MADS×ICON:")
        for k, v in sorted(r['mads_x_icon'].items()):
            pct = 100.0 * v / r['frames']
            print(f"    {k:>30}: {v:>8,} ({pct:5.1f}%)")

    print("\n=== TASK 2 — 60°+ TURN EVENTS & RETURN-TO-CENTER ===")
    for route in sorted(by_route.keys()):
        r = by_route[route]
        print(f"\n--- drive {route}: {len(r['turn_events'])} turns ≥60°, {len(r['analyzed'])} analyzed ---")
        # Bucket by peak angle + mads state
        b_buckets = Counter()
        for t in r['turn_events']:
            bin_lo = int(abs(t['peak_wheel']) // 30) * 30
            b_buckets[(f">={bin_lo}°", 'mads' if t['peak_mads_en'] else 'off')] += 1
        for k, v in sorted(b_buckets.items()):
            print(f"    {k[0]:>6} {k[1]:>6}: {v}")

        # Find concerning analyzed cases
        concerning = []
        for a in r['analyzed']:
            t = a['turn']
            # Concerning: op was active AND wheel moved fast AND op deviated significantly
            if a['op_active_in_window'] and abs(a['wheel_max_step_per_frame_deg']) > 1.5 \
               and a['op_deviation_max_abs'] > 5.0:
                concerning.append(a)
        print(f"    concerning return-to-center events (op active, wheel step >1.5°/f, op-wheel dev >5°): {len(concerning)}")
        for c in concerning[:5]:
            t = c['turn']
            print(f"      peak={t['peak_wheel']:>+7.1f}° v={t['peak_v_kmh']:>5.1f} kph "
                  f"mads={t['peak_mads_en']} blinker={t['peak_blinker']}: "
                  f"after release op_dev_max={c['op_deviation_max_abs']:.1f}° "
                  f"wheel_step_max={c['wheel_max_step_per_frame_deg']:.2f}°/f")

    # Save
    out = {
        'routes': {
            route: {
                'frames': r['frames'],
                'turn_events_total': len(r['turn_events']),
                'analyzed_total': len(r['analyzed']),
                'lfa_icon_counts': dict(r['lfa_icon_counts']),
                'mads_x_icon': dict(r['mads_x_icon']),
                'concerning_returns': [
                    {
                        'peak_wheel_deg': a['turn']['peak_wheel'],
                        'peak_v_kmh': a['turn']['peak_v_kmh'],
                        'mads_en': a['turn']['peak_mads_en'],
                        'blinker': a['turn']['peak_blinker'],
                        'op_active_in_window': a['op_active_in_window'],
                        'release_offset_frames': a['release_offset_frames'],
                        'wheel_max_step_per_frame_deg': a['wheel_max_step_per_frame_deg'],
                        'op_deviation_max_abs': a['op_deviation_max_abs'],
                        'op_deviation_p50_abs': a['op_deviation_p50_abs'],
                    }
                    for a in r['analyzed']
                    if a['op_active_in_window']
                    and abs(a['wheel_max_step_per_frame_deg']) > 1.5
                    and a['op_deviation_max_abs'] > 5.0
                ][:50],
            } for route, r in by_route.items()},
    }
    with open('/tmp/drives14_16_steering.json','w') as f:
        json.dump(out, f, indent=2)
    print("\nSaved /tmp/drives14_16_steering.json")


if __name__ == '__main__':
    main()
