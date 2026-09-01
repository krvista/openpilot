#!/usr/bin/env python3
"""List local drive routes with segment count + start/end GPS, so you can pick a diverse
set (different trips: commute, garage, outing) for the replay comparison.

    python3 list_routes.py [/data/media/0/realdata]
"""
import glob
import os
import re
import sys

import zstandard
import capnp

capnp.remove_import_hook()
CER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../cereal")
log_capnp = capnp.load(os.path.join(CER, "log.capnp"), imports=[CER, os.path.join(CER, "..")])


def gps_fix(qlog_path, want_last=False):
  """First (or last) valid GPS fix in a segment's qlog."""
  if not os.path.exists(qlog_path):
    return None
  try:
    data = zstandard.ZstdDecompressor().stream_reader(open(qlog_path, "rb")).read()
    res = None
    for e in log_capnp.Event.read_multiple_bytes(data):
      w = e.which()
      if w in ("gpsLocationExternal", "gpsLocation"):
        g = e.gpsLocationExternal if w == "gpsLocationExternal" else e.gpsLocation
        if getattr(g, "hasFix", False) or (g.latitude and g.longitude):
          res = (g.latitude, g.longitude)
          if not want_last:
            return res
    return res
  except Exception:
    return None


def main():
  root = sys.argv[1] if len(sys.argv) > 1 else "/data/media/0/realdata"
  segdirs = [os.path.dirname(p) for p in glob.glob(os.path.join(root, "*", "fcamera.hevc"))]
  routes = {}
  for d in segdirs:
    routes.setdefault(re.sub(r"--\d+$", "", os.path.basename(d)), []).append(d)

  print(f"{'route':<26}{'segs':>5}   {'start (lat,lon)':<20} {'end (lat,lon)':<20}")
  rows = []
  for r in sorted(routes):
    segs = sorted(routes[r], key=lambda d: int(d.rsplit("--", 1)[1]))
    s = gps_fix(os.path.join(segs[0], "qlog.zst"))
    e = gps_fix(os.path.join(segs[-1], "qlog.zst"), want_last=True)
    ss = f"{s[0]:.4f},{s[1]:.4f}" if s else "?"
    ee = f"{e[0]:.4f},{e[1]:.4f}" if e else "?"
    print(f"{r:<26}{len(segs):>5}   {ss:<20} {ee:<20}")
    rows.append(r)
  print(f"\n{len(rows)} routes. Pick diverse ones (distinct start/end) and pass to")
  print("batch_diverse.sh via ROUTES=\"routeA routeB ...\" (and SAMPLES_PER_ROUTE=all for full coverage).")


if __name__ == "__main__":
  main()
