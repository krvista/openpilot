#!/usr/bin/env python3
"""Scan for cam_stale events — when LKAS_ALT COUNTER freezes."""
import sys, os, glob
import numpy as np

sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG = "/home/user/openpilot/drivelog"
CAM_STALE_FRAMES = 25


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
  last_counter = -1
  stale_since = 0
  frame = 0
  stale_events = []
  in_stale = False
  stale_start = 0
  last_v = 0
  lkas_alt_msgs = 0
  counter_changes = 0

  for msg in LogReader(path):
    w = msg.which()
    try:
      if w == 'can':
        for m in msg.can:
          if m.address == 0x110 and m.src == 2:
            lkas_alt_msgs += 1
            dat = bytes(m.dat)
            if len(dat) >= 2:
              counter = dat[1] >> 4
              if counter != last_counter:
                counter_changes += 1
                stale_since = frame
                if in_stale:
                  stale_events.append((stale_start, frame, last_v))
                  in_stale = False
                last_counter = counter
              elif (frame - stale_since) > CAM_STALE_FRAMES and not in_stale:
                in_stale = True
                stale_start = frame
        frame += 1
      elif w == 'carState':
        last_v = msg.carState.vEgo * 3.6
    except Exception:
      pass

  if in_stale:
    stale_events.append((stale_start, frame, last_v))

  return {
    'lkas_alt_msgs': lkas_alt_msgs,
    'counter_changes': counter_changes,
    'stale_events': stale_events,
    'total_frames': frame,
  }


def main(route_hexes):
  for rh in route_hexes:
    segs = find_segments(rh)
    if not segs:
      continue
    print(f"\n{'='*70}")
    print(f"  Route 0x{rh.zfill(8)[-2:]} — cam_stale scan")
    print(f"{'='*70}")
    total_stale = 0
    for sn in sorted(segs.keys()):
      try:
        r = scan_seg(segs[sn])
      except Exception:
        continue
      if r['stale_events']:
        for start, end, v in r['stale_events']:
          dur = (end - start) / 100.0
          total_stale += 1
          print(f"  ⚠ seg {sn:3d}  stale frame {start}-{end} ({dur:.1f}s)  v≈{v:.0f}km/h  "
                f"cam_msgs={r['lkas_alt_msgs']}  ctr_changes={r['counter_changes']}")
    if total_stale == 0:
      print(f"  ✅ No cam_stale events detected across {len(segs)} segments")
    else:
      print(f"\n  Total cam_stale events: {total_stale}")


if __name__ == "__main__":
  routes = sys.argv[1:] if len(sys.argv) > 1 else ['3f', '40']
  main(routes)
