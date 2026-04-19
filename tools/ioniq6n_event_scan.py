#!/usr/bin/env python3
"""Scan for onroad events and see if any correlate with LFA icon disappearance."""

import glob
import sys
from collections import Counter
sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG_DIR = '/home/user/openpilot/drivelog'

def scan_events():
    for route_id in [0x2a, 0x2b]:
        pattern = f'{DRIVELOG_DIR}/99b215d21bbf8735_{route_id:08x}--*--rlog.zst'
        files = sorted(glob.glob(pattern))
        if not files:
            continue
        
        print(f"\nRoute 0x{route_id:02x}: Event enumeration")
        
        event_counts = Counter()
        event_times = {}
        
        for seg_path in files:
            for m in LogReader(seg_path):
                try:
                    w = m.which()
                except:
                    continue
                
                if w == 'onroadEvents':
                    for evt in m.onroadEvents:
                        evt_name = str(evt.name)
                        event_counts[evt_name] += 1
                        if evt_name not in event_times:
                            event_times[evt_name] = []
                        # event_times[evt_name].append(m.logMonoTime)
        
        print(f"  Total unique event types: {len(event_counts)}")
        for evt_name, count in sorted(event_counts.items(), key=lambda x: -x[1])[:30]:
            print(f"    {evt_name}: {count}")

scan_events()
