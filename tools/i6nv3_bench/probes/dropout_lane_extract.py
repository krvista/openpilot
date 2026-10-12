"""Lane-dropout probe, step 1: modelV2 lane detail at 20 Hz (same t0 as route_extract) + road camera exposure + modeld process.
cols: t, p0..p3, s0..s3 (laneLineStds), y0..y3 @x=0, y1,y2 @x=30, edge L/R y0, edge std L/R, plan k, lcs, frameDropPerc,
      cam exposure%, cam gain, cam grey fraction (nearest roadCameraState)"""
import glob, sys
import numpy as np
from openpilot.tools.lib.logreader import LogReader
r, out = sys.argv[1], sys.argv[2]
files = sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
t0 = None; rows = []; cam = (np.nan, np.nan, np.nan); procs = set()
for f in files:
  try:
    for m in LogReader(f):
      t = m.logMonoTime * 1e-9
      if t0 is None: t0 = t
      w = m.which()
      if w == "roadCameraState":
        c = m.roadCameraState
        cam = (getattr(c, "exposureValPercent", np.nan), getattr(c, "gain", np.nan), getattr(c, "measuredGreyFraction", np.nan))
      elif w == "managerState" and not procs:
        procs = {p.name for p in m.managerState.processes if p.running and "model" in p.name}
      elif w == "modelV2":
        d = m.modelV2
        if len(d.laneLines) < 4: continue
        p = list(d.laneLineProbs); s = list(d.laneLineStds) if len(d.laneLineStds) >= 4 else [np.nan] * 4
        y0 = [float(np.interp(0.0, np.asarray(l.x), np.asarray(l.y))) for l in d.laneLines]
        y30 = [float(np.interp(30.0, np.asarray(d.laneLines[i].x), np.asarray(d.laneLines[i].y))) for i in (1, 2)]
        if len(d.roadEdges) >= 2:
          e = [float(np.interp(0.0, np.asarray(x.x), np.asarray(x.y))) for x in d.roadEdges]
          es = list(d.roadEdgeStds)[:2] if len(d.roadEdgeStds) >= 2 else [np.nan, np.nan]
        else:
          e = [np.nan, np.nan]; es = [np.nan, np.nan]
        rows.append((t - t0, *p, *s, *y0, *y30, *e, *es, d.action.desiredCurvature, int(d.meta.laneChangeState.raw),
                     d.frameDropPerc, *cam))
  except Exception as ex:  # noqa: BLE001
    print(f"  {f.split('/')[-1]}: {type(ex).__name__}: {ex}")
np.savez_compressed(out, L=np.array(rows, float), procs=np.array(sorted(procs)))
print(r, len(rows), sorted(procs))
