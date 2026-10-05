"""§33 corner probe, step 1: modelV2 lane geometry at 20 Hz (same t0 as route_extract): inner line y at x = 0 / 10 / 30 m, probs, edges, plan y.
Usage: corner_lanes_extract.py <route> <out.npy>; column order is read by probes/corner_inside.py.
"""
import glob, sys
import numpy as np
from openpilot.tools.lib.logreader import LogReader
r, out = sys.argv[1], sys.argv[2]
files = sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
t0 = None; rows = []
XS = (0.0, 10.0, 30.0)
for f in files:
  try:
    for m in LogReader(f):
      t = m.logMonoTime * 1e-9
      if t0 is None: t0 = t
      if m.which() != "modelV2": continue
      d = m.modelV2
      if len(d.laneLines) < 4 or len(d.position.x) < 10: continue
      L, R = d.laneLines[1], d.laneLines[2]
      lx = np.asarray(L.x); px = np.asarray(d.position.x)
      yl = np.interp(XS, lx, np.asarray(L.y)); yr = np.interp(XS, np.asarray(R.x), np.asarray(R.y))
      py = np.interp(XS, px, np.asarray(d.position.y))
      el = np.interp(XS[0], np.asarray(d.roadEdges[0].x), np.asarray(d.roadEdges[0].y)) if len(d.roadEdges) >= 2 else np.nan
      er = np.interp(XS[0], np.asarray(d.roadEdges[1].x), np.asarray(d.roadEdges[1].y)) if len(d.roadEdges) >= 2 else np.nan
      rows.append((t - t0, *yl, *yr, *py, d.laneLineProbs[1], d.laneLineProbs[2], el, er, d.action.desiredCurvature,
                   L.y[0] if len(L.y) else np.nan, d.laneLineStds[1] if len(d.laneLineStds) > 2 else np.nan))
  except Exception as e:  # noqa: BLE001
    print(f.split("/")[-1], type(e).__name__, e)
A = np.array(rows, float)
np.save(out, A)
print(r, A.shape)
