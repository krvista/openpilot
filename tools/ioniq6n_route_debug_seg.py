#!/usr/bin/env python3
"""Debug a single segment — extract raw field names and alert types."""
import sys, os, glob
import numpy as np
from collections import Counter

sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG = "/home/user/openpilot/drivelog"


def find_seg(route_hex, seg_num):
  rh = route_hex.strip().lower().zfill(8)
  pattern = os.path.join(DRIVELOG, f"*_{rh}--*--{seg_num}--rlog.zst")
  files = glob.glob(pattern)
  return files[0] if files else None


def debug_seg(path):
  alerts = Counter()
  visual_alerts = Counter()
  msg_types = Counter()
  speed_samples = []
  lat_errors = []
  saturated_count = 0
  lat_active_count = 0
  steer_limited_count = 0
  onroad_events = Counter()

  for msg in LogReader(path):
    w = msg.which()
    msg_types[w] += 1

    if w == 'controlsState':
      cs = msg.controlsState
      try:
        v = cs.vEgo
        speed_samples.append(v * 3.6)
      except Exception:
        pass
      lat = cs.lateralControlState
      lw = lat.which()
      if lw == 'angleState':
        ang = lat.angleState
        if ang.active:
          lat_active_count += 1
          err = abs(ang.steeringAngleDesiredDeg - ang.steeringAngleDeg)
          lat_errors.append(err)
        if ang.saturated:
          saturated_count += 1

    elif w == 'carState':
      cs2 = msg.carState
      try:
        v = cs2.vEgo
        if v > 0.1:
          speed_samples.append(v * 3.6)
      except Exception:
        pass

    elif w == 'carControl':
      cc = msg.carControl
      try:
        va = str(cc.hudControl.visualAlert)
        if va and va != 'none':
          visual_alerts[va] += 1
      except Exception:
        pass

    elif w == 'selfdriveState':
      sd = msg.selfdriveState
      try:
        at = sd.alertType
        if at:
          alerts[at] += 1
      except Exception:
        pass
      try:
        for e in sd.alertText1, sd.alertText2:
          pass
      except Exception:
        pass

    elif w == 'onroadEvents':
      try:
        for e in msg.onroadEvents:
          onroad_events[str(e.name)] += 1
      except Exception:
        pass

    elif w == 'carOutput':
      try:
        co = msg.carOutput
        if hasattr(co, 'steerLimitedBySteerRate') and co.steerLimitedBySteerRate:
          steer_limited_count += 1
      except Exception:
        pass

  print(f"  Total messages per type (top 15):")
  for k, v in msg_types.most_common(15):
    print(f"    {v:6d}  {k}")

  print(f"\n  Lat active: {lat_active_count}  Saturated: {saturated_count}  SteerLimited: {steer_limited_count}")
  if speed_samples:
    sp = np.array(speed_samples)
    print(f"  Speed: min={sp.min():.1f}  max={sp.max():.1f}  mean={sp.mean():.1f} km/h  ({len(sp)} samples)")
  else:
    print(f"  Speed: no samples!")
  if lat_errors:
    e = np.array(lat_errors)
    over25 = (e > 2.5).sum()
    print(f"  Angle error: mean={e.mean():.2f}°  p95={np.percentile(e,95):.2f}°  max={e.max():.2f}°  >2.5°={over25}")

  if alerts:
    print(f"\n  selfdriveState alertType:")
    for k, v in alerts.most_common(20):
      print(f"    {v:5d}  {k}")

  if visual_alerts:
    print(f"\n  hudControl visualAlert:")
    for k, v in visual_alerts.most_common(20):
      print(f"    {v:5d}  {k}")

  if onroad_events:
    print(f"\n  onroadEvents:")
    for k, v in onroad_events.most_common(20):
      print(f"    {v:5d}  {k}")


if __name__ == "__main__":
  route = sys.argv[1] if len(sys.argv) > 1 else '3f'
  seg = sys.argv[2] if len(sys.argv) > 2 else '11'
  path = find_seg(route, seg)
  if not path:
    print(f"Seg not found: route={route} seg={seg}")
    sys.exit(1)
  print(f"Analyzing {os.path.basename(path)}")
  debug_seg(path)
