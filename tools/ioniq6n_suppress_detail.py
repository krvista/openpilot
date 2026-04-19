#!/usr/bin/env python3
"""Detailed suppress_lfa TX audit including gaps and counter issues."""

import glob
import sys
sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG_DIR = '/home/user/openpilot/drivelog'

def detailed_suppress_audit():
    for route_id in [0x2a, 0x2b]:
        pattern = f'{DRIVELOG_DIR}/99b215d21bbf8735_{route_id:08x}--*--rlog.zst'
        files = sorted(glob.glob(pattern))
        if not files:
            continue
        
        print(f"\n{'='*120}")
        print(f"Route 0x{route_id:02x}: Detailed suppress_lfa audit")
        print('='*120)
        
        frames = {'0x362': [], '0x2a4': []}
        t0 = None
        
        for seg_path in files:
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
                        if c.address == 0x362 and c.src == 0 and len(c.dat) > 2:
                            frames['0x362'].append({'t_ms': t_ms, 'counter': c.dat[2]})
                        elif c.address == 0x2a4 and c.src == 0 and len(c.dat) > 2:
                            frames['0x2a4'].append({'t_ms': t_ms, 'counter': c.dat[2]})
        
        for addr in ['0x362', '0x2a4']:
            if not frames[addr]:
                continue
            
            print(f"\n  {addr} frames: {len(frames[addr])}")
            
            # Find counter jumps
            jumps = []
            gaps_long = []
            
            for i in range(1, len(frames[addr])):
                f_prev = frames[addr][i-1]
                f_curr = frames[addr][i]
                
                prev_counter = f_prev['counter']
                curr_counter = f_curr['counter']
                delta_ms = f_curr['t_ms'] - f_prev['t_ms']
                
                expected_counter = (prev_counter + 1) & 0xFF
                
                if curr_counter != expected_counter:
                    jumps.append({
                        'i': i,
                        'prev': prev_counter,
                        'curr': curr_counter,
                        'expected': expected_counter,
                        't_ms': f_curr['t_ms'],
                        'delta_ms': delta_ms
                    })
                
                if delta_ms > 75:
                    gaps_long.append({
                        'i': i,
                        'delta_ms': delta_ms,
                        't_ms': f_curr['t_ms'],
                        'prev_counter': prev_counter,
                        'curr_counter': curr_counter
                    })
            
            print(f"  Counter jumps (discontinuities): {len(jumps)}")
            for jump in jumps[:20]:
                print(f"    @ i={jump['i']} t={jump['t_ms']:.0f}ms delta={jump['delta_ms']:.1f}ms: "
                      f"{jump['prev']:02x} → {jump['curr']:02x} (expected {jump['expected']:02x})")
            
            print(f"  Long gaps (>75ms): {len(gaps_long)}")
            for gap in gaps_long[:20]:
                print(f"    @ i={gap['i']} t={gap['t_ms']:.0f}ms: delta={gap['delta_ms']:.1f}ms "
                      f"counter={gap['prev_counter']:02x}→{gap['curr_counter']:02x}")

detailed_suppress_audit()
