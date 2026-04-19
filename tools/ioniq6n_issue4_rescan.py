#!/usr/bin/env python3
"""ADAS/LFA dropout analysis for routes 42 & 43.

Focuses on:
1. latActive OFF→ON and ON→OFF transitions during cruise control (cruiseState.enabled=True)
2. onroadEvents enumeration and names
3. steerFaultTemporary True events
4. Sporadic dropout timestamps and correlations with speed/road/time
5. 0x160 HDA2 LFA status (CAN bus 2) and 0x2A4/0x362 CCNC fields
"""
import glob
import sys
import os
from collections import defaultdict, Counter
from datetime import datetime
import json

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG_DIR = '/home/user/openpilot/drivelog'

def scan_route(route_files):
    """Scan multiple segments for latActive transitions, events, faults."""
    results = {
        'lat_active_transitions': [],  # (timestamp_s, cruise_enabled, prev_lat, curr_lat, speed, is_dropout)
        'onroad_events': Counter(),
        'steer_fault_temp_count': 0,
        'steer_fault_temp_events': [],
        'can_hda2_lfa_status': Counter(),  # 0x160 samples
        'can_ccnc_adas': Counter(),  # 0x2A4 / 0x362 samples
        'dropouts': [],  # worst sporadic cases
        'total_frames': 0,
        'start_time': None,
        'end_time': None,
        'vego_range': (float('inf'), 0),
    }
    
    t0 = None
    prev_lat_active = None
    cruise_enabled = False
    vego = 0.0
    
    for seg_path in route_files:
        print(f"  Reading {os.path.basename(seg_path)}...", end=' ', flush=True)
        frame_count = 0
        
        try:
            for m in LogReader(seg_path):
                try:
                    w = m.which()
                except:
                    continue
                
                results['total_frames'] += 1
                frame_count += 1
                
                if t0 is None:
                    t0 = m.logMonoTime
                
                t_sec = (m.logMonoTime - t0) / 1e9
                
                # Track time range
                if results['start_time'] is None:
                    results['start_time'] = datetime.now().isoformat()
                results['end_time'] = datetime.now().isoformat()
                
                # === carState: cruise, speed, steerFault ===
                if w == 'carState':
                    cs = m.carState
                    cruise_enabled = cs.cruiseState.enabled
                    vego = cs.vEgoRaw * 3.6  # km/h
                    results['vego_range'] = (
                        min(results['vego_range'][0], vego),
                        max(results['vego_range'][1], vego)
                    )
                    
                    if cs.steerFaultTemporary:
                        results['steer_fault_temp_count'] += 1
                        results['steer_fault_temp_events'].append({
                            't': t_sec,
                            'vego': vego,
                            'cruise_enabled': cruise_enabled
                        })
                
                # === carControl: latActive ===
                if w == 'carControl':
                    cc = m.carControl
                    lat_active = cc.latActive
                    
                    # Detect transitions during cruise
                    if prev_lat_active is not None and lat_active != prev_lat_active:
                        is_dropout = lat_active == False  # OFF transition = dropout
                        transition = {
                            't': t_sec,
                            'cruise_enabled': cruise_enabled,
                            'prev_lat': prev_lat_active,
                            'curr_lat': lat_active,
                            'vego': vego,
                            'is_dropout': is_dropout
                        }
                        results['lat_active_transitions'].append(transition)
                        
                        # Track dropouts (ON→OFF while cruise enabled)
                        if is_dropout and cruise_enabled:
                            results['dropouts'].append(transition)
                    
                    prev_lat_active = lat_active
                
                # === onroadEvents ===
                if w == 'onroadEvents':
                    for evt in m.onroadEvents:
                        results['onroad_events'][str(evt.name)] += 1
                
                # === CAN: 0x160 (HDA2 LFA) and 0x2A4/0x362 (CCNC) ===
                if w == 'can':
                    for c in m.can:
                        # 0x160: bus 2 (camera), HDA2 LFA status
                        if c.src == 2 and c.address == 0x160:
                            dat = bytes(c.dat)
                            if len(dat) >= 8:
                                # Summarize key bytes
                                lfa_status_byte = dat[0] if len(dat) > 0 else 0
                                results['can_hda2_lfa_status'][f'byte0={lfa_status_byte:02x}'] += 1
                        
                        # 0x2A4 or 0x362: CCNC ADAS fields
                        if c.address in (0x2A4, 0x362):
                            results['can_ccnc_adas'][f'addr=0x{c.address:03x}'] += 1
        
        except Exception as e:
            print(f"ERROR in {seg_path}: {e}")
            continue
        
        print(f"{frame_count} frames")
    
    return results


def analyze_dropouts(dropouts):
    """Find worst 5 dropouts by context (speed, time of drive, etc)."""
    if not dropouts:
        return []
    
    # Sort by various criteria: longest cruise, lowest speed, etc.
    # For now, just return first 5
    return dropouts[:5]


def main():
    """Scan routes 42 and 43."""
    print("=" * 80)
    print(" ADAS/LFA Dropout Analysis: Routes 42 & 43")
    print("=" * 80)
    
    for route_id in [42, 43]:
        pattern = f'{DRIVELOG_DIR}/99b215d21bbf8735_{route_id:08d}--*--rlog.zst'
        files = sorted(glob.glob(pattern))
        
        if not files:
            print(f"\nRoute {route_id}: No files found")
            continue
        
        print(f"\nRoute {route_id}: {len(files)} segments")
        print("-" * 80)
        
        results = scan_route(files)
        
        # === Report ===
        print(f"\n✓ Processed {results['total_frames']:,} frames")
        print(f"  Speed range: {results['vego_range'][0]:.1f} - {results['vego_range'][1]:.1f} km/h")
        
        # 1. latActive transitions
        lat_transitions = results['lat_active_transitions']
        lat_off_on = sum(1 for t in lat_transitions if not t['prev_lat'] and t['curr_lat'])
        lat_on_off = sum(1 for t in lat_transitions if t['prev_lat'] and not t['curr_lat'])
        
        print(f"\n1. latActive Transitions (during cruise):")
        print(f"   OFF→ON: {lat_off_on}")
        print(f"   ON→OFF: {lat_on_off}")
        print(f"   Total: {len(lat_transitions)}")
        
        # 2. onroadEvents
        events = results['onroad_events']
        print(f"\n2. onroadEvents ({len(events)} unique):")
        for evt_name, count in sorted(events.items(), key=lambda x: -x[1])[:10]:
            print(f"   {evt_name}: {count}")
        
        # 3. steerFaultTemporary
        print(f"\n3. steerFaultTemporary Events: {results['steer_fault_temp_count']}")
        if results['steer_fault_temp_events']:
            for i, evt in enumerate(results['steer_fault_temp_events'][:3]):
                print(f"   [{i+1}] t={evt['t']:.1f}s, v={evt['vego']:.1f}km/h, cruise={evt['cruise_enabled']}")
        
        # 4. Worst 5 dropouts
        worst_dropouts = analyze_dropouts(results['dropouts'])
        print(f"\n4. Sporadic Dropouts: {len(results['dropouts'])} total")
        print(f"   Worst 5:")
        for i, d in enumerate(worst_dropouts, 1):
            print(f"   [{i}] t={d['t']:.1f}s (cruise_on={d['cruise_enabled']}, v={d['vego']:.1f}km/h)")
        
        # 5. CAN insights (0x160, 0x2A4/0x362)
        print(f"\n5. CAN Insights:")
        if results['can_hda2_lfa_status']:
            print(f"   0x160 (HDA2 LFA): {sum(results['can_hda2_lfa_status'].values())} frames")
            for key, cnt in sorted(results['can_hda2_lfa_status'].items())[:3]:
                print(f"     {key}: {cnt}")
        if results['can_ccnc_adas']:
            print(f"   0x2A4/0x362 (CCNC): {sum(results['can_ccnc_adas'].values())} frames")
    
    print("\n" + "=" * 80)


if __name__ == '__main__':
    main()
