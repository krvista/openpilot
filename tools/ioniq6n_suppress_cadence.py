#!/usr/bin/env python3
"""Check if suppress_lfa TX is properly cadenced (every 5 frames = 20 Hz)."""

import glob
import sys
sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader
from collections import defaultdict

DRIVELOG_DIR = '/home/user/openpilot/drivelog'

def check_suppress_cadence():
    for route_id in [0x2a, 0x2b]:
        pattern = f'{DRIVELOG_DIR}/99b215d21bbf8735_{route_id:08x}--*--rlog.zst'
        files = sorted(glob.glob(pattern))
        if not files:
            continue
        
        print(f"\nRoute 0x{route_id:02x}: suppress_lfa cadence check")
        
        suppress_frames = []
        t0 = None
        
        for seg_path in files[:3]:
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
                        if c.address == 0x362 and c.src == 0 and len(c.dat) > 2:  # Our TX suppress
                            counter = c.dat[2]
                            suppress_frames.append({'t_ms': t_ms, 'counter': counter})
        
        print(f"  Total suppress_lfa TX frames: {len(suppress_frames)}")
        
        # Check inter-frame gaps
        gaps = []
        for i in range(1, min(50, len(suppress_frames))):
            delta_ms = suppress_frames[i]['t_ms'] - suppress_frames[i-1]['t_ms']
            gaps.append(delta_ms)
        
        if gaps:
            avg_gap = sum(gaps) / len(gaps)
            max_gap = max(gaps)
            min_gap = min(gaps)
            print(f"  Gap statistics (first 50 frames):")
            print(f"    Min: {min_gap:.1f}ms, Max: {max_gap:.1f}ms, Avg: {avg_gap:.1f}ms")
            print(f"    Expected: ~50ms (20 Hz)")
            
            # Find outliers
            outliers = [g for g in gaps if g > 75]  # > 1.5x expected
            if outliers:
                print(f"    Outliers (>75ms): {len(outliers)}")
                for i, gap in enumerate(outliers[:5]):
                    print(f"      [{i+1}] gap={gap:.1f}ms")
        
        # Check COUNTER continuity
        counter_issues = []
        for i in range(1, len(suppress_frames)):
            prev_counter = suppress_frames[i-1]['counter']
            curr_counter = suppress_frames[i]['counter']
            expected = (prev_counter + 1) & 0xFF
            if curr_counter != expected:
                counter_issues.append({
                    'i': i,
                    'prev': prev_counter,
                    'curr': curr_counter,
                    'expected': expected,
                    't_ms': suppress_frames[i]['t_ms']
                })
        
        print(f"  COUNTER discontinuities: {len(counter_issues)}")
        if counter_issues:
            for issue in counter_issues[:5]:
                print(f"    @ t={issue['t_ms']:.0f}ms: {issue['prev']:02x} → {issue['curr']:02x} "
                      f"(expected {issue['expected']:02x})")

check_suppress_cadence()
