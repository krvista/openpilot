#!/usr/bin/env python3
"""
Ioniq 6N Issue Analysis: Low-speed override fight, lane-change overshoot, rapid accel instability
Routes 42 & 43 on branch ccnc-port-prebuilt
"""
import sys
import glob
from collections import defaultdict
sys.path.insert(0, '/home/user/openpilot')

from openpilot.tools.lib.logreader import LogReader
import numpy as np

DRIVELOG_PATH = '/home/user/openpilot/drivelog'

def get_route_segments(route_num):
    """Get all segment files for a route."""
    pattern = f'{DRIVELOG_PATH}/99b215d21bbf8735_000000{route_num:02d}--*--rlog.zst'
    return sorted(glob.glob(pattern))

def process_route(route_num):
    """Process a single route and extract issue data."""
    segments = get_route_segments(route_num)
    if not segments:
        return None
    
    issue1_events = []
    issue2_lanechanges = []
    issue3_accel_events = []
    
    frame_count = 0
    prev_speed = 0
    prev_desired_angle = 0
    prev_lane_state = None
    lanechange_start_ts = None
    
    for seg_idx, segment_path in enumerate(segments):
        try:
            lr = LogReader(segment_path)
        except Exception as e:
            print(f"  WARNING: Could not read segment {segment_path}: {e}")
            continue
        
        msgs = list(lr)
        for msg_idx, msg in enumerate(msgs):
            frame_count += 1
            
            # Extract data
            if msg.which() == 'carState':
                cs = msg.carState
                ts = msg.logMonoTime / 1e9
                speed = cs.vEgo
                steering_angle = cs.steeringAngleDeg
                driver_torque = cs.steeringTorque
                steering_pressed = cs.steeringPressed
                
                # ISSUE 1: Low-speed override fight (~30 km/h, 90° turn)
                # Speed bucket: 5-12 m/s (18-43 km/h)
                if abs(steering_angle) > 90 and 5 <= speed <= 12 and abs(driver_torque) > 0.5:
                    issue1_events.append({
                        'ts': ts, 'seg': seg_idx, 'idx': msg_idx,
                        'speed': speed, 'angle': steering_angle, 'torque': driver_torque
                    })
                
                # ISSUE 2: Lane-change overshoot tracking
                # Note: laneChangeState not available in capnp schema for this route
                
                # ISSUE 3: Rapid acceleration instability
                # Find frames where vEgo increases by > 5 m/s within 2s (~60 frames @ 30Hz)
                accel = (speed - prev_speed) / 0.033  # ~30 Hz frame rate
                if accel > 2.5 and speed > 0:
                    issue3_accel_events.append({
                        'ts': ts, 'seg': seg_idx, 'speed': speed, 'accel': accel
                    })
                
                prev_speed = speed
                
            elif msg.which() == 'carControl':
                cc = msg.carControl
                desired_angle = cc.actuators.steeringAngleDeg
                lat_active = cc.latActive
                
                # For ISSUE 1: compute steering error
                if issue1_events and abs(issue1_events[-1]['ts'] - msg.logMonoTime / 1e9) < 0.05:
                    steering_error = abs(desired_angle - issue1_events[-1]['angle'])
                    issue1_events[-1]['error'] = steering_error
                    issue1_events[-1]['lat_active'] = lat_active
                
                prev_desired_angle = desired_angle
    
    return {
        'route': route_num,
        'segments': len(segments),
        'frames': frame_count,
        'issue1': issue1_events,
        'issue2': issue2_lanechanges,
        'issue3': issue3_accel_events
    }

def analyze_issue1(data):
    """Analyze Issue 1: Low-speed override fight."""
    events = data['issue1']
    if not events:
        return "No events detected"
    
    errors = [e.get('error', 0) for e in events if 'error' in e]
    avg_error = np.mean(errors) if errors else 0
    peak_error = max(errors) if errors else 0
    
    # Top 5 worst events
    top_events = sorted(events, key=lambda x: x.get('error', 0), reverse=True)[:5]
    
    summary = f"Issue 1 (Low-speed override fight @~30 km/h, >90° turns):\n"
    summary += f"  Total high-steering events: {len(events)}\n"
    summary += f"  Avg steering error: {avg_error:.2f}°, Peak: {peak_error:.2f}°\n"
    if top_events:
        summary += f"  Top 5 worst conflicts:\n"
        for i, evt in enumerate(top_events[:5], 1):
            lat_active_str = evt.get('lat_active', False)
            summary += f"    {i}. Seg{evt['seg']}, ts={evt['ts']:.1f}s, v={evt['speed']:.1f}m/s, error={evt.get('error', 0):.2f}°, latActive={lat_active_str}\n"
    return summary

def analyze_issue2(data):
    """Analyze Issue 2: Lane-change overshoot."""
    lc_events = data['issue2']
    
    summary = f"Issue 2 (Lane-change overshoot):\n"
    if not lc_events:
        summary += "  No lane changes detected (laneChangeState unavailable in schema)\n"
    else:
        summary += f"  Lane changes found: {len(lc_events)}\n"
    return summary

def analyze_issue3(data):
    """Analyze Issue 3: Rapid acceleration instability."""
    events = data['issue3']
    if not events:
        return "No rapid acceleration events (>2.5 m/s²) detected"
    
    accels = [e['accel'] for e in events]
    avg_accel = np.mean(accels) if accels else 0
    peak_accel = max(accels) if accels else 0
    
    summary = f"Issue 3 (Rapid acceleration instability):\n"
    summary += f"  High-accel events (>2.5 m/s²): {len(events)}\n"
    summary += f"  Avg accel: {avg_accel:.2f} m/s², Peak: {peak_accel:.2f} m/s²\n"
    
    # Peak events
    top_events = sorted(events, key=lambda x: x['accel'], reverse=True)[:3]
    if top_events:
        summary += f"  Top 3 peak accelerations:\n"
        for i, evt in enumerate(top_events, 1):
            summary += f"    {i}. Seg{evt['seg']}, ts={evt['ts']:.1f}s, v={evt['speed']:.1f}m/s, a={evt['accel']:.2f}m/s²\n"
    return summary

def main():
    print("=" * 75)
    print("IONIQ 6N MULTI-ISSUE ANALYSIS - Routes 42 & 43")
    print("Branch: ccnc-port-prebuilt | Device: 99b215d21bbf8735")
    print("=" * 75)
    print()
    
    results = []
    for route_num in [42, 43]:
        print(f"Processing Route {route_num}...")
        data = process_route(route_num)
        if data is None:
            print(f"  Route {route_num} not found")
            continue
        
        results.append(data)
        print(f"  Segments: {data['segments']}, Total frames: {data['frames']}")
        print(f"  Issue 1 events: {len(data['issue1'])}, Issue 3 events: {len(data['issue3'])}")
    
    print("\n" + "=" * 75)
    print("ANALYSIS RESULTS")
    print("=" * 75)
    
    for data in results:
        print(f"\nROUTE {data['route']}:")
        print(analyze_issue1(data))
        print(analyze_issue2(data))
        print(analyze_issue3(data))
    
    print("\n" + "=" * 75)
    print("LIMITATIONS & NOTES")
    print("=" * 75)
    print("- laneChangeState field unavailable in capnp schema (Issue 2 cannot be fully analyzed)")
    print("- Steering error approximation uses frame timestamps (<50ms tolerance)")
    print("- Lateral acceleration computation requires curvature/heading (not extracted)")
    print("- Analysis captures discrete high-torque steering + high-accel frames only")

if __name__ == '__main__':
    main()
