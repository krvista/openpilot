#!/usr/bin/env python3
"""Enumerate all CAN messages in the logs."""

import glob
import sys
from collections import Counter
sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG_DIR = '/home/user/openpilot/drivelog'

def enum_messages():
    for route_id in [0x2a]:
        pattern = f'{DRIVELOG_DIR}/99b215d21bbf8735_{route_id:08x}--*--rlog.zst'
        files = sorted(glob.glob(pattern))
        if not files:
            continue
        
        print(f"Route 0x{route_id:02x}: CAN message enumeration (first segment)")
        
        messages = Counter()
        messages_by_src = {}
        
        for seg_path in files[:1]:
            for m in LogReader(seg_path):
                try:
                    w = m.which()
                except:
                    continue
                
                if w == 'can':
                    for c in m.can:
                        key = f'0x{c.address:03x}'
                        messages[key] += 1
                        
                        if key not in messages_by_src:
                            messages_by_src[key] = {}
                        if c.src not in messages_by_src[key]:
                            messages_by_src[key][c.src] = 0
                        messages_by_src[key][c.src] += 1
        
        print(f"\nTop 40 CAN messages:")
        for msg, count in messages.most_common(40):
            src_info = messages_by_src.get(msg, {})
            src_str = ", ".join(f"src{s}:{c}" for s, c in sorted(src_info.items()))
            print(f"  {msg}: {count} ({src_str})")

enum_messages()
