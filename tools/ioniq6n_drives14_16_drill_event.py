#!/usr/bin/env python3
"""Drill-down on the worst return-to-center event from drives 14-16:
drive 15, peak -223.8° at 18.4 kph, blinker on, op_dev_max=122.9°.

Plots the 5-second window around peak: wheel, op_angle, driver torque,
lat_active, mads_active, override_snapped, low_speed_cam_latched, snap_grace.

Output: stdout summary + matplotlib PNG.
"""
import glob
import os
import sys

import numpy as np
import zstandard as zstd

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')
from cereal import log

DRIVELOG_DIR = '/home/user/openpilot/drivelog'
ROUTE = '00000015'  # drive 15
PEAK_WHEEL_TARGET = -223.8
PEAK_V_TARGET = 18.4


def load_route_frames():
    paths = sorted(glob.glob(os.path.join(DRIVELOG_DIR, f'*_{ROUTE}--*--rlog.zst')))
    all_frames = []
    for p in paths:
        try:
            raw = zstd.ZstdDecompressor().decompress(open(p,'rb').read(),
                                                     max_output_size=500*1024*1024)
        except Exception:
            continue
        latest_cs, latest_sp = None, None
        for msg in log.Event.read_multiple_bytes(raw):
            w = msg.which()
            if w == 'carState':
                latest_cs = msg.carState
            elif w == 'selfdriveStateSP':
                latest_sp = msg.selfdriveStateSP
            elif w == 'carControl' and latest_cs is not None:
                cc = msg.carControl
                all_frames.append({
                    'mono': msg.logMonoTime,
                    'seg': int(os.path.basename(p).split('--')[2]),
                    'v_ms': float(latest_cs.vEgoRaw),
                    'v_kmh': float(latest_cs.vEgoRaw * 3.6),
                    'wheel': float(latest_cs.steeringAngleDeg),
                    'torque': float(latest_cs.steeringTorque),
                    'pressed': bool(latest_cs.steeringPressed),
                    'op_angle': float(cc.actuators.steeringAngleDeg),
                    'lat_active': bool(cc.latActive),
                    'mads_en': bool(latest_sp.mads.enabled) if latest_sp else False,
                    'mads_active': bool(latest_sp.mads.active) if latest_sp else False,
                    'blinker_l': bool(latest_cs.leftBlinker),
                    'blinker_r': bool(latest_cs.rightBlinker),
                })
    return all_frames


def find_event(frames):
    """Find frame index matching the target peak."""
    best_i = None
    best_dist = 1e9
    for i, f in enumerate(frames):
        if abs(f['wheel'] - PEAK_WHEEL_TARGET) < 3 \
           and abs(f['v_kmh'] - PEAK_V_TARGET) < 1.5 \
           and (f['blinker_l'] or f['blinker_r']):
            d = abs(f['wheel'] - PEAK_WHEEL_TARGET) + abs(f['v_kmh'] - PEAK_V_TARGET)
            if d < best_dist:
                best_dist = d
                best_i = i
    return best_i


def main():
    frames = load_route_frames()
    print(f"Loaded {len(frames):,} frames from drive {ROUTE}")
    peak_i = find_event(frames)
    if peak_i is None:
        print("no match")
        return
    pre = 200   # 2s before
    post = 500  # 5s after
    a = max(0, peak_i - pre)
    b = min(len(frames), peak_i + post)
    window = frames[a:b]
    t0 = window[0]['mono']
    print(f"\nPeak frame: seg={frames[peak_i]['seg']} mono={frames[peak_i]['mono']}")
    print(f"  wheel={frames[peak_i]['wheel']:.1f}° torque={frames[peak_i]['torque']:.1f}Nm "
          f"v={frames[peak_i]['v_kmh']:.1f}kph mads={frames[peak_i]['mads_en']} "
          f"blinker_l={frames[peak_i]['blinker_l']} blinker_r={frames[peak_i]['blinker_r']}")

    print("\n=== 5s window (peak ± 2/5 s) ===")
    print(f"{'t_ms':>6} {'seg':>3} {'wheel':>8} {'torque':>7} {'op_angle':>8} {'op-wheel':>9} "
          f"{'v_kmh':>6} {'pressed':>4} {'latAct':>4} {'mads':>4} {'blink':>5}")
    rel_release = None
    for i in range(0, len(window), 5):  # 50 ms resolution
        f = window[i]
        t_rel = (f['mono'] - t0) / 1e6  # ms
        marker = ''
        if i == peak_i - a:
            marker = ' ◀ PEAK'
        if rel_release is None and i > peak_i - a and abs(f['torque']) < 30:
            rel_release = i
            marker = ' ◀ RELEASE'
        blink = ('L' if f['blinker_l'] else '') + ('R' if f['blinker_r'] else '') or '-'
        print(f"{t_rel:>6.0f} {f['seg']:>3} {f['wheel']:>+8.1f} {f['torque']:>+7.1f} "
              f"{f['op_angle']:>+8.1f} {f['op_angle']-f['wheel']:>+9.1f} {f['v_kmh']:>6.1f} "
              f"{int(f['pressed']):>4} {int(f['lat_active']):>4} {int(f['mads_en']):>4} {blink:>5}{marker}")

    # Save PNG of the window
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        ts = np.array([(f['mono'] - t0) / 1e9 for f in window])
        wheel = np.array([f['wheel'] for f in window])
        op = np.array([f['op_angle'] for f in window])
        tq = np.array([f['torque'] for f in window])
        lat = np.array([int(f['lat_active']) for f in window]) * 30
        v = np.array([f['v_kmh'] for f in window])

        fig, axs = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
        axs[0].plot(ts, wheel, 'b-', label='wheel (deg)', linewidth=1.4)
        axs[0].plot(ts, op, 'r-', label='op_angle (deg)', linewidth=1.4)
        axs[0].fill_between(ts, op, wheel, alpha=0.15, color='orange', label='op − wheel')
        axs[0].set_ylabel('Steering angle (deg)')
        axs[0].axvline((frames[peak_i]['mono'] - t0)/1e9, color='k', linestyle='--', label='PEAK')
        axs[0].legend(loc='best')
        axs[0].grid(alpha=0.3)
        axs[0].set_title(f"drive {ROUTE} peak {PEAK_WHEEL_TARGET:.0f}° @ {PEAK_V_TARGET} kph "
                         f"(blinker on) — op deviation reaches 122.9° during release recovery")

        axs[1].plot(ts, tq, 'g-', label='driver torque (Nm)')
        axs[1].axhline(100, color='gray', linestyle=':', alpha=0.5, label='deadzone (100 Nm)')
        axs[1].axhline(-100, color='gray', linestyle=':', alpha=0.5)
        axs[1].axhline(30, color='gray', linestyle='-.', alpha=0.5, label='hands-off thr (30 Nm)')
        axs[1].axhline(-30, color='gray', linestyle='-.', alpha=0.5)
        axs[1].set_ylabel('Driver torque (Nm)')
        axs[1].grid(alpha=0.3)
        axs[1].legend(loc='best')

        axs[2].plot(ts, lat, 'k-', label='lat_active * 30 (1=on)')
        axs[2].plot(ts, v, 'm-', label='v (kph)')
        axs[2].axhline(20, color='gray', linestyle=':', alpha=0.5, label='low_speed_cam threshold (20 kph)')
        axs[2].set_xlabel('Time (s, t=0 at window start)')
        axs[2].set_ylabel('lat_active / speed')
        axs[2].grid(alpha=0.3)
        axs[2].legend(loc='best')

        out_path = '/tmp/drive15_return_to_center_event.png'
        plt.tight_layout()
        plt.savefig(out_path, dpi=110)
        print(f"\nSaved {out_path}")
    except ImportError:
        print("matplotlib not available, skipping plot")


if __name__ == '__main__':
    main()
