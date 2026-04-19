#!/usr/bin/env python3
"""
Ioniq 6N issues 4, 5, 6 analysis: LFA dropouts, planning lag, high-speed steering rate-limit.
Routes 42 (36 seg) and 43 (34 seg). Analyzes ~1.4 GB of logreader data.
"""
import sys
import os
import glob
import numpy as np
from collections import defaultdict

sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

# ============= Issue 4: LFA/ADAS Dropouts =============
def analyze_issue_4(routes):
    """Scan for ADAS/LFA alert dropouts and stock LFA authority loss."""
    alerts_seen = defaultdict(int)
    adas_errors = []
    lat_control_toggles = []
    mads_state_history = []
    
    for route_num in routes:
        pattern = f'/home/user/openpilot/drivelog/99b215d21bbf8735_{route_num:08d}--*--rlog.zst'
        segment_files = sorted(glob.glob(pattern))
        
        for seg_file in segment_files:
            try:
                reader = LogReader(seg_file)
                prev_lat_active = None
                prev_cruise_enabled = None
                
                for msg in reader:
                    # Track ADAS errors in controlsState
                    if msg.which() == 'controlsState':
                        cs = msg.controlsState
                        if cs.alertText1:
                            alerts_seen[cs.alertText1] += 1
                            if any(x in cs.alertText1.lower() for x in ['steer', 'lka', 'lfa', 'adas']):
                                adas_errors.append({
                                    'route': route_num,
                                    'alert': cs.alertText1,
                                    't': msg.logMonoTime / 1e9
                                })
                        
                        # Track lat_active (MADS steering) toggles
                        if cs.lateralActive is not None:
                            if prev_lat_active is not None and cs.lateralActive != prev_lat_active:
                                lat_control_toggles.append({
                                    'route': route_num,
                                    't': msg.logMonoTime / 1e9,
                                    'to': cs.lateralActive,
                                    'lat_active': cs.lateralActive
                                })
                            prev_lat_active = cs.lateralActive
                    
                    # Track cruise state
                    if msg.which() == 'carState':
                        cs = msg.carState
                        if cs.cruiseState.enabled is not None:
                            if prev_cruise_enabled is not None and cs.cruiseState.enabled != prev_cruise_enabled:
                                mads_state_history.append({
                                    'route': route_num,
                                    't': msg.logMonoTime / 1e9,
                                    'enabled': cs.cruiseState.enabled
                                })
                            prev_cruise_enabled = cs.cruiseState.enabled
            except Exception as e:
                pass
    
    return {
        'alerts': alerts_seen,
        'adas_errors': adas_errors,
        'lat_toggles': lat_control_toggles,
        'mads_toggles': mads_state_history
    }

# ============= Issue 5: Planning Lag (Steering Reaction Time) =============
def analyze_issue_5(routes):
    """Measure steering lag: delta-time between model curvature prediction and actual steering."""
    lags = []
    curve_entries = 0
    
    for route_num in routes:
        pattern = f'/home/user/openpilot/drivelog/99b215d21bbf8735_{route_num:08d}--*--rlog.zst'
        segment_files = sorted(glob.glob(pattern))
        
        for seg_file in segment_files:
            try:
                reader = LogReader(seg_file)
                model_history = {}
                control_history = {}
                frame_count = 0
                
                for msg in reader:
                    frame_count += 1
                    t = msg.logMonoTime / 1e9
                    
                    # Collect model orientation predictions
                    if msg.which() == 'modelV2':
                        if msg.modelV2.orientationRate.z:
                            model_history[frame_count] = {
                                't': t,
                                'orient_z': list(msg.modelV2.orientationRate.z)
                            }
                    
                    # Collect steering commands
                    if msg.which() == 'carControl':
                        control_history[frame_count] = {
                            't': t,
                            'steer_angle': msg.carControl.actuators.steeringAngleDeg
                        }
                
                # Identify curve entries and measure lag (simplified: look for model z-rate spikes)
                for fc in sorted(model_history.keys()):
                    orient = model_history[fc]['orient_z']
                    if len(orient) > 0 and abs(orient[0]) > 0.005:  # curvature prediction
                        curve_entries += 1
                        # Find first steering response within next 3 seconds
                        for fc_steer in range(fc, min(fc + 150, max(control_history.keys()) + 1)):
                            if fc_steer in control_history:
                                lag_ms = (control_history[fc_steer]['t'] - model_history[fc]['t']) * 1000
                                if 0 <= lag_ms <= 1000:
                                    lags.append(lag_ms)
                                    break
            except Exception as e:
                pass
    
    return {
        'lags_ms': lags,
        'curve_entries': curve_entries,
        'avg_lag_ms': np.mean(lags) if lags else None,
        'max_lag_ms': np.max(lags) if lags else None
    }

# ============= Issue 6: High-Speed Steering Rate Limiting (40-60 km/h off-ramps) =============
def analyze_issue_6(routes):
    """Identify steering rate-limit events: demand > current at v ∈ [11,17] m/s with large Δangle."""
    
    # ANGLE_RATE_BP/V from carcontroller
    ANGLE_RATE_BP = [0., 7., 11., 17., 23., 30.]
    ANGLE_RATE_V = [2.5, 2.5, 2.0, 1.5, 1.3, 1.0]
    
    rate_limit_events = []
    
    for route_num in routes:
        pattern = f'/home/user/openpilot/drivelog/99b215d21bbf8735_{route_num:08d}--*--rlog.zst'
        segment_files = sorted(glob.glob(pattern))
        
        for seg_file in segment_files:
            try:
                reader = LogReader(seg_file)
                prev_steer_angle = 0
                prev_desired_angle = 0
                
                for msg in reader:
                    v_ego = None
                    steer_angle = None
                    desired_angle = None
                    t = msg.logMonoTime / 1e9
                    
                    if msg.which() == 'carState':
                        v_ego = msg.carState.vEgo
                        steer_angle = msg.carState.steeringAngleDeg
                    
                    if msg.which() == 'carControl':
                        desired_angle = msg.carControl.actuators.steeringAngleDeg
                    
                    if v_ego is not None and desired_angle is not None and steer_angle is not None:
                        # Check if in target speed range: 40-60 km/h ≈ 11-17 m/s
                        if 11 <= v_ego <= 17:
                            # Large steering correction needed: >70° demand delta
                            angle_delta_demand = abs(desired_angle - steer_angle)
                            if angle_delta_demand > 70:
                                # Get cap at this speed
                                rate_cap = float(np.interp(v_ego, ANGLE_RATE_BP, ANGLE_RATE_V))
                                max_step = rate_cap  # deg per 20ms
                                
                                # Estimate if rate-limited
                                actual_delta = abs(steer_angle - prev_steer_angle)
                                if actual_delta < max_step * 0.8 and actual_delta > 0.1:
                                    rate_limit_events.append({
                                        'route': route_num,
                                        't': t,
                                        'v': v_ego,
                                        'demand_delta': angle_delta_demand,
                                        'rate_cap_deg_per_20ms': rate_cap,
                                        'max_possible_swing_90deg_sec': 90 / (rate_cap * 50)
                                    })
                        
                        prev_steer_angle = steer_angle
                        prev_desired_angle = desired_angle
            except Exception as e:
                pass
    
    return {
        'events': rate_limit_events,
        'event_count': len(rate_limit_events),
        'worst_case': sorted(rate_limit_events, key=lambda x: x['demand_delta'], reverse=True)[:3] if rate_limit_events else []
    }

# ============= MAIN =============
if __name__ == '__main__':
    routes = [42, 43]
    
    print("=" * 70)
    print("IONIQ 6N ISSUES 4-6 ANALYSIS")
    print("=" * 70)
    
    # Issue 4
    print("\n[ISSUE 4] LFA/ADAS Error Dropouts (Non-Driver-Induced)")
    print("-" * 70)
    result_4 = analyze_issue_4(routes)
    
    steering_alerts = {k: v for k, v in result_4['alerts'].items() if any(x in k.lower() for x in ['steer', 'lka', 'lfa', 'adas'])}
    print(f"Steering/ADAS alerts: {sum(steering_alerts.values())} instances")
    for alert, count in sorted(steering_alerts.items(), key=lambda x: -x[1])[:5]:
        print(f"  {alert}: {count}x")
    
    print(f"Lateral active toggles (MADS): {len(result_4['lat_toggles'])} events")
    if result_4['lat_toggles'][:3]:
        for evt in result_4['lat_toggles'][:3]:
            print(f"  Route {evt['route']} @ t={evt['t']:.1f}s → {evt['to']}")
    
    # Issue 5
    print("\n[ISSUE 5] Planning Lag (Steering Reaction Time)")
    print("-" * 70)
    result_5 = analyze_issue_5(routes)
    
    if result_5['avg_lag_ms'] is not None:
        print(f"Curve entries detected: {result_5['curve_entries']}")
        print(f"Avg steering lag: {result_5['avg_lag_ms']:.0f} ms")
        print(f"Max steering lag: {result_5['max_lag_ms']:.0f} ms")
        print(f"Samples: {len(result_5['lags_ms'])}")
    else:
        print("No sufficient curve data or lag samples found.")
    
    # Issue 6
    print("\n[ISSUE 6] High-Speed Ramp Steering Rate Limiting (11-17 m/s, >70° demand)")
    print("-" * 70)
    result_6 = analyze_issue_6(routes)
    
    print(f"Rate-limit events: {result_6['event_count']}")
    if result_6['worst_case']:
        print("\nWorst cases (top 3):")
        for i, evt in enumerate(result_6['worst_case'], 1):
            print(f"  {i}. Route {evt['route']} @ t={evt['t']:.1f}s | v={evt['v']:.1f}m/s | " +
                  f"demand_Δ={evt['demand_delta']:.1f}° | rate_cap={evt['rate_cap_deg_per_20ms']:.1f}°/20ms | " +
                  f"90°_swing_time={evt['max_possible_swing_90deg_sec']:.2f}s")
    
    print("\n" + "=" * 70)
    print("ANALYSIS COMPLETE")
    print("=" * 70)
