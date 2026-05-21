#!/usr/bin/env python3
"""IONIQ 6N CCNC FORK: ADAS FAULT ROOT CAUSE ANALYSIS
   Deep diagnostic scan for suppress_lfa TX synchronization issues.

HYPOTHESIS: The instrument cluster briefly displays "ADAS error" and the
LFA icon completely disappears, indicating the stock ADAS ECU has declared the system
unavailable (fault level, not just suspended).

ROOT CAUSE: Suppress_lfa (CAM_0x362) COUNTER discontinuities or timing gaps cause
ADAS to detect frame format corruption or desynchronization from openpilot, triggering
a system fault that cascades through ACC/ADAS ECUs, resulting in cluster warnings and
eventual LFA disable.

TECHNICAL DETAILS:
  - suppress_lfa is TX'd at frame % 5 (50 Hz control rate = 20 Hz suppress rate)
  - We copy COUNTER from camera's CAM_0x362 frame (via lfa_block_msg)
  - If suppress_lfa COUNTER jumps (e.g., 0x1a → 0x1c), ADAS detects frame corruption
  - Long gaps (>100ms) between suppress_lfa TX can cause ADAS timeout/watchdog faults
  - Activation/gain mismatches (e.g., LKAS_ANGLE_ACTIVE=1 but ACIGain=0) also trigger

INVESTIGATION TARGETS:
  1. suppress_lfa COUNTER continuity and timing gaps
  2. LKAS_ALT activation/gain signal mismatches
  3. suppress_lfa skip frames (frame % 5 == 0 condition in carcontroller)
  4. Camera staleness (CAM_0x362 received but not forwarded)
"""

import glob
import sys
from collections import Counter, defaultdict
sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG_DIR = '/home/user/openpilot/drivelog'

# ===== 1. Suppress_lfa TX Quality =====
def analyze_suppress_lfa_quality(route_files):
    """Check suppress_lfa (0x362/0x2a4) COUNTER continuity and timing."""
    results = {'0x362': {}, '0x2a4': {}}
    
    for addr in [0x362, 0x2a4]:
        addr_hex = f'0x{addr:x}'
        frames = []
        t0 = None
        
        for seg_path in route_files:
            for m in LogReader(seg_path):
                try:
                    w = m.which()
                except:
                    continue
                
                if t0 is None and w == 'can':
                    t0 = m.logMonoTime
                
                if w == 'can' and t0:
                    t_ms = (m.logMonoTime - t0) / 1e6
                    for c in m.can:
                        if c.address == addr and c.src == 0 and len(c.dat) > 2:
                            frames.append({'t_ms': t_ms, 'counter': c.dat[2]})
        
        if not frames:
            continue
        
        # Analyze gaps
        gaps = []
        for i in range(1, len(frames)):
            gaps.append(frames[i]['t_ms'] - frames[i-1]['t_ms'])
        
        # Analyze counter continuity
        counter_jumps = []
        for i in range(1, len(frames)):
            prev = frames[i-1]['counter']
            curr = frames[i]['counter']
            expected = (prev + 1) & 0xFF
            if curr != expected:
                counter_jumps.append({
                    'i': i,
                    'prev': prev,
                    'curr': curr,
                    'expected': expected,
                    't_ms': frames[i]['t_ms'],
                    'gap_ms': gaps[i-1] if i-1 < len(gaps) else None
                })
        
        results[addr_hex] = {
            'total_frames': len(frames),
            'gaps': gaps,
            'gap_avg_ms': sum(gaps) / len(gaps) if gaps else 0,
            'gap_max_ms': max(gaps) if gaps else 0,
            'counter_jumps': counter_jumps
        }
    
    return results


# ===== 2. LKAS_ALT Activation/Gain Analysis =====
def analyze_lkas_alt_signals(route_files):
    """Check for LKAS_ANGLE_ACTIVE vs ACIGain mismatches."""
    our_tx = []
    t0 = None
    
    for seg_path in route_files:
        for m in LogReader(seg_path):
            try:
                w = m.which()
            except:
                continue
            
            if t0 is None and w == 'can':
                t0 = m.logMonoTime
            
            if w == 'can' and t0:
                t_ms = (m.logMonoTime - t0) / 1e6
                for c in m.can:
                    if c.address == 0x110 and c.src == 0 and len(c.dat) >= 14:
                        dat = bytes(c.dat)
                        lkas_angle_active = (dat[9] >> 5) & 0x3
                        lka_assist = (dat[7] >> 6) & 0x1
                        acigain = dat[12]
                        
                        # Flag mismatches
                        mismatch = (lkas_angle_active == 2 or lka_assist == 1) and acigain == 0
                        our_tx.append({
                            't_ms': t_ms,
                            'lkas_angle_active': lkas_angle_active,
                            'lka_assist': lka_assist,
                            'acigain': acigain,
                            'mismatch': mismatch
                        })
    
    mismatches = [f for f in our_tx if f['mismatch']]
    return {
        'total_frames': len(our_tx),
        'mismatches': mismatches
    }


# ===== 3. onroad Events & Timing =====
def analyze_onroad_events(route_files):
    """Enumerate all onroad events."""
    event_counts = Counter()
    
    for seg_path in route_files:
        for m in LogReader(seg_path):
            try:
                w = m.which()
            except:
                continue
            
            if w == 'onroadEvents':
                for evt in m.onroadEvents:
                    evt_name = str(evt.name)
                    event_counts[evt_name] += 1
    
    return event_counts


def main():
    print("=" * 120)
    print(" IONIQ 6N CCNC: ADAS FAULT ROOT CAUSE ANALYSIS")
    print("=" * 120)
    
    for route_num, route_id in [(42, 0x2a), (43, 0x2b)]:
        pattern = f'{DRIVELOG_DIR}/99b215d21bbf8735_{route_id:08x}--*--rlog.zst'
        files = sorted(glob.glob(pattern))
        
        if not files:
            print(f"\nRoute {route_num} (0x{route_id:02x}): No files found")
            continue
        
        print(f"\n{'='*120}")
        print(f"ROUTE {route_num} (ID: 0x{route_id:02x}): {len(files)} segments")
        print(f"{'='*120}")
        
        # Suppress_lfa analysis
        print("\n[1] SUPPRESS_LFA TX QUALITY (0x362/0x2a4)")
        print("-" * 120)
        suppress_results = analyze_suppress_lfa_quality(files)
        
        for addr_hex in ['0x362', '0x2a4']:
            res = suppress_results.get(addr_hex, {})
            if not res:
                continue
            
            print(f"\n  {addr_hex}:")
            print(f"    Total frames: {res['total_frames']}")
            print(f"    Gap avg: {res['gap_avg_ms']:.1f} ms (expected ~50 ms @ 20 Hz)")
            print(f"    Gap max: {res['gap_max_ms']:.1f} ms")
            
            if res['counter_jumps']:
                print(f"    COUNTER DISCONTINUITIES: {len(res['counter_jumps'])}")
                for jump in res['counter_jumps'][:10]:
                    print(f"      @ t={jump['t_ms']:.0f}ms: 0x{jump['prev']:02x} → 0x{jump['curr']:02x} "
                          f"(expected 0x{jump['expected']:02x}), gap={jump['gap_ms']:.1f}ms")
                print(f"\n    ROOT CAUSE: Counter discontinuity indicates frame was skipped or")
                print(f"    double-sent. ADAS detects this as frame corruption → fault trigger.")
            else:
                print(f"    ✓ COUNTER continuous (good)")
        
        # LKAS_ALT analysis
        print("\n[2] LKAS_ALT ACTIVATION/GAIN SYNC")
        print("-" * 120)
        lkas_results = analyze_lkas_alt_signals(files)
        print(f"  Total 0x110 TX frames: {lkas_results['total_frames']}")
        print(f"  Activation/Gain mismatches: {len(lkas_results['mismatches'])}")
        if lkas_results['mismatches']:
            for m in lkas_results['mismatches'][:5]:
                print(f"    @ t={m['t_ms']:.0f}ms: LKAS_ANGLE_ACTIVE={m['lkas_angle_active']} "
                      f"LKA_ASSIST={m['lka_assist']} ACIGain={m['acigain']}")
            print(f"\n  ROOT CAUSE: Activation bits say 'active' but gain=0 means 'idle'.")
            print(f"  This contradicts the protocol spec → ADAS faults.")
        else:
            print(f"  ✓ No mismatches detected (good)")
        
        # Onroad events
        print("\n[3] ONROAD EVENTS")
        print("-" * 120)
        events = analyze_onroad_events(files)
        for evt_name, count in sorted(events.items(), key=lambda x: -x[1])[:15]:
            print(f"  {evt_name}: {count}")
    
    print("\n" + "=" * 120)
    print("SUMMARY & RECOMMENDATIONS")
    print("=" * 120)
    print("""
If suppress_lfa COUNTER discontinuities are present:
  - Likely cause: carcontroller's `frame % 5 == 0` skip condition
  - Or: lfa_block_msg is stale and COUNTER hasn't advanced
  - Fix: Ensure suppress_lfa COUNTER is incremented monotonically,
    even if lfa_block_msg hasn't changed

If LKAS_ALT activation/gain mismatches are present:
  - Root cause: steering_active boolean in create_steering_messages() is
    False while activation bits indicate active (or vice versa)
  - Check: Is the single-source-of-truth (lines 98-107 of hyundaicanfd.py)
    being violated by early returns or special cases?

If both are clean:
  - The issue may be timing-dependent (race condition with camera messages)
  - Or early boot sequence (suppression skipped before CAM parser ready)
  - Recommend: Enable detailed logging of frame formatting in controlsd
""")
    
    print("=" * 120)


if __name__ == '__main__':
    main()
