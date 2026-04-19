#!/usr/bin/env python3
"""Minimal focused scan: suppress COUNTER staleness audit."""

import glob
import sys
sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG_DIR = '/home/user/openpilot/drivelog'

def scan_suppress_counter_staleness():
    """Find instances where our suppress_lfa TX has stale COUNTER.
    
    Symptom: camera's CAM_0x362 COUNTER advanced since our last suppress_lfa TX,
    so we emit with outdated COUNTER → ADAS detects frame format mismatch.
    """
    
    # We send suppress at frame % 5 (20 Hz), so we have 5-frame TX windows: 0-4, 5-9, 10-14, ...
    # If camera's COUNTER jumps 2+ within a 5-frame window, our suppress is stale.
    
    for route_id in [0x2a, 0x2b]:
        pattern = f'{DRIVELOG_DIR}/99b215d21bbf8735_{route_id:08x}--*--rlog.zst'
        files = sorted(glob.glob(pattern))
        if not files:
            continue
        
        print(f"\n{'='*100}")
        print(f"Route 0x{route_id:02x}: Analyzing suppress_lfa COUNTER staleness")
        print('='*100)
        
        # Build timeline of CAM_0x362 frames
        cam_frames = []
        suppress_frames = []
        t0 = None
        
        for seg_path in files[:5]:  # Start with first 5 segments
            for m in LogReader(seg_path):
                try:
                    w = m.which()
                except:
                    continue
                
                if t0 is None and w in ('can', 'carControl'):
                    t0 = m.logMonoTime
                
                if w == 'can' and t0:
                    t_ms = (m.logMonoTime - t0) / 1e6
                    for c in m.can:
                        # Camera 0x362 (src=2)
                        if c.address == 0x362 and c.src == 2 and len(c.dat) > 2:
                            counter = c.dat[2]
                            cam_frames.append({'t_ms': t_ms, 'counter': counter, 'src': 2})
                        
                        # Our TX 0x362 (src=0)
                        elif c.address == 0x362 and c.src == 0 and len(c.dat) > 2:
                            counter = c.dat[2]
                            suppress_frames.append({'t_ms': t_ms, 'counter': counter, 'src': 0})
        
        if not cam_frames or not suppress_frames:
            print(f"  Not enough data (cam={len(cam_frames)}, tx={len(suppress_frames)})")
            continue
        
        print(f"  Camera 0x362 frames: {len(cam_frames)}")
        print(f"  Suppress TX frames: {len(suppress_frames)}")
        
        # For each suppress TX, check if we copied a stale COUNTER
        stale_count = 0
        for tx in suppress_frames[:20]:  # Check first 20 TX
            # Find the closest prior camera frame
            prior_cam = [f for f in cam_frames if f['t_ms'] < tx['t_ms']]
            if not prior_cam:
                continue
            
            latest_cam = max(prior_cam, key=lambda f: f['t_ms'])
            delay_ms = tx['t_ms'] - latest_cam['t_ms']
            
            # Stale if: (a) delay > 50ms and (b) camera COUNTER has advanced since then
            if delay_ms > 50:
                future_cam = [f for f in cam_frames if latest_cam['t_ms'] < f['t_ms'] < tx['t_ms']]
                if future_cam:
                    latest_future = max(future_cam, key=lambda f: f['t_ms'])
                    if latest_future['counter'] != latest_cam['counter']:
                        stale_count += 1
                        print(f"  STALE @ t={tx['t_ms']:.0f}ms: "
                              f"TX counter=0x{tx['counter']:02x} but camera advanced from "
                              f"0x{latest_cam['counter']:02x} to 0x{latest_future['counter']:02x} "
                              f"(delay from cam={delay_ms:.0f}ms)")
        
        print(f"  → Found {stale_count} stale suppress COUNTER instances in first 20 TX")

if __name__ == '__main__':
    scan_suppress_counter_staleness()
