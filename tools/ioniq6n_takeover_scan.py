#!/usr/bin/env python3
"""Fast scan — count steerSaturated/steerRequired per segment."""
import sys, os, glob
import numpy as np
from collections import defaultdict

sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG = "/home/user/openpilot/drivelog"


def find_segments(route_hex):
  rh = route_hex.strip().lower().zfill(8)
  pattern = os.path.join(DRIVELOG, f"*_{rh}--*--rlog.zst")
  files = sorted(glob.glob(pattern))
  segs = {}
  for f in files:
    parts = os.path.basename(f).split('--')
    if len(parts) >= 3:
      segs[int(parts[-2])] = f
  return segs


def scan_seg(path):
  r = {'lat_active': 0, 'saturated': 0, 'steer_sat_alert': 0,
       'steer_override': 0, 'speed_mean': 0, 'errs': [],
       'speeds_at_sat': [], 'desired_at_sat': [], 'actual_at_sat': [],
       'crashes': 0}
  speeds = []
  for msg in LogReader(path):
    w = msg.which()
    if w == 'controlsState':
      cs = msg.controlsState
      lat = cs.lateralControlState
      if lat.which() == 'angleState':
        ang = lat.angleState
        if ang.active:
          r['lat_active'] += 1
          err = abs(ang.steeringAngleDesiredDeg - ang.steeringAngleDeg)
          r['errs'].append(err)
          if ang.saturated:
            r['saturated'] += 1
    elif w == 'carState':
      try:
        speeds.append(msg.carState.vEgo * 3.6)
      except Exception:
        pass
    elif w == 'selfdriveState':
      try:
        at = msg.selfdriveState.alertType
        if 'steerSaturated' in at:
          r['steer_sat_alert'] += 1
          if speeds:
            r['speeds_at_sat'].append(speeds[-1])
        if 'steerOverride' in at:
          r['steer_override'] += 1
      except Exception:
        pass
    elif w == 'logMessage':
      try:
        m = str(msg.logMessage)
        if 'crash' in m.lower():
          r['crashes'] += 1
      except Exception:
        pass
  r['speed_mean'] = np.mean(speeds) if speeds else 0
  return r


def main(route_hexes):
  for rh in route_hexes:
    segs = find_segments(rh)
    if not segs:
      print(f"Route {rh}: not found")
      continue
    print(f"\n{'='*90}")
    print(f"  Route 0x{rh.zfill(8)[-2:]} — {len(segs)} segments")
    print(f"{'='*90}")
    print(f"  {'seg':>4} {'dur':>5} {'v_mean':>7} {'active':>7} {'sat':>5} {'sat%':>6} "
          f"{'steerSat':>9} {'override':>9} {'err_mean':>9} {'err_p95':>8} {'crash':>6}")
    tot_sat_alert = 0
    tot_override = 0
    tot_active = 0
    tot_saturated = 0
    tot_crashes = 0
    all_errs = []
    sat_speeds = []
    for sn in sorted(segs.keys()):
      try:
        r = scan_seg(segs[sn])
      except Exception as e:
        print(f"  {sn:4d}  ERROR: {e}")
        continue
      pct = 100 * r['saturated'] / max(r['lat_active'], 1)
      em = np.mean(r['errs']) if r['errs'] else 0
      e95 = np.percentile(r['errs'], 95) if r['errs'] else 0
      flag = "⚠" if r['steer_sat_alert'] > 0 else " "
      if r['crashes'] > 0:
        flag = "❌"
      print(f" {flag}{sn:4d} {60:5d}s {r['speed_mean']:6.1f} {r['lat_active']:7d} "
            f"{r['saturated']:5d} {pct:5.1f}% {r['steer_sat_alert']:9d} {r['steer_override']:9d} "
            f"{em:9.2f} {e95:8.2f} {r['crashes']:6d}")
      tot_sat_alert += r['steer_sat_alert']
      tot_override += r['steer_override']
      tot_active += r['lat_active']
      tot_saturated += r['saturated']
      tot_crashes += r['crashes']
      all_errs.extend(r['errs'])
      sat_speeds.extend(r['speeds_at_sat'])

    print(f"\n  TOTALS:")
    print(f"    steerSaturated alerts: {tot_sat_alert}")
    print(f"    steerOverride frames:  {tot_override}")
    print(f"    Saturation frames:     {tot_saturated} / {tot_active} ({100*tot_saturated/max(tot_active,1):.1f}%)")
    print(f"    Card crashes:          {tot_crashes}")
    if all_errs:
      e = np.array(all_errs)
      print(f"    Angle err: mean={e.mean():.2f}° p95={np.percentile(e,95):.2f}° max={e.max():.2f}°")
      over = (e > 2.5).sum()
      print(f"    >2.5° error: {over} ({100*over/len(e):.1f}%)")
    if sat_speeds:
      print(f"    Speed at steerSaturated: mean={np.mean(sat_speeds):.1f} km/h "
            f"min={np.min(sat_speeds):.1f} max={np.max(sat_speeds):.1f}")


if __name__ == "__main__":
  routes = sys.argv[1:] if len(sys.argv) > 1 else ['3f', '40']
  main(routes)
