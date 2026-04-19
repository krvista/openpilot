#!/usr/bin/env python3
"""Track 0x160 (HDA2 LFA_STATUS) to find disappearance events."""

import glob
import sys
from collections import Counter, defaultdict
sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG_DIR = '/home/user/openpilot/drivelog'

def analyze_0x160():
    for route_id in [0x2a, 0x2b]:
        pattern = f'{DRIVELOG_DIR}/99b215d21bbf8735_{route_id:08x}--*--rlog.zst'
        files = sorted(glob.glob(pattern))
        if not files:
            continue
        
        print(f"\nRoute 0x{route_id:02x}: 0x160 LFA_STATUS analysis")
        
        status_values = Counter()
        transitions = defaultdict(list)
        t0 = None
        last_status = None
        last_time = None
        
        for seg_path in files[:3]:  # First 3 segments
            for m in LogReader(seg_path):
                try:
                    w = m.which()
                except:
                    continue
                
                if t0 is None and w == 'can':
                    t0 = m.logMonoTime
                
                if w == 'can' and t0:
                    t_sec = (m.logMonoTime - t0) / 1e9
                    for c in m.can:
                        if c.address == 0x160 and c.src == 2 and len(c.dat) > 0:
                            # Byte 0 likely contains main LFA status
                            status_byte = c.dat[0]
                            status_values[f'0x{status_byte:02x}'] += 1
                            
                            if last_status is not None and status_byte != last_status:
                                transitions[f'{last_status:02x}→{status_byte:02x}'].append(t_sec)
                                if (status_byte == 0 or last_status == 0) and last_time:
                                    print(f"  TRANSITION @ t={t_sec:.2f}s: "
                                          f"0x{last_status:02x}→0x{status_byte:02x} "
                                          f"(interval={t_sec - last_time:.3f}s)")
                            
                            last_status = status_byte
                            last_time = t_sec
        
        print(f"  Unique 0x160 byte[0] values:")
        for val, count in sorted(status_values.items()):
            print(f"    {val}: {count}")
        
        print(f"\n  Transitions (>5 occurrences):")
        for trans, times in sorted(transitions.items(), key=lambda x: -len(x[1])):
            if len(times) >= 5:
                print(f"    {trans}: {len(times)} times")

analyze_0x160()
