"""Split camera EOF -> modelV2 publish using wideRoadCameraState (the only camera state in rlog): camerad part (EOF -> wide
cameraState publish) and modeld part (wide frame ready -> modelV2 publish = road-frame wait + inputs + execution + post).
Also controlsd pickup: modelV2 publish -> first controlsState after it (100 Hz loop phase)."""
import sys
import numpy as np
from openpilot.tools.lib.logreader import LogReader
wide, rows, pick = {}, [], []; pending = None
for f in sys.argv[1:]:
  for m in LogReader(f):
    w = m.which(); t = m.logMonoTime
    if w == "wideRoadCameraState": wide[m.wideRoadCameraState.frameId] = (t, m.wideRoadCameraState.timestampEof)
    elif w == "modelV2":
      md = m.modelV2; pending = t
      if md.frameIdExtra in wide:
        wp, weof = wide[md.frameIdExtra]
        rows.append(((wp - weof) * 1e-6, (t - wp) * 1e-6, md.modelExecutionTime * 1e3, (t - md.timestampEof) * 1e-6))
    elif w == "controlsState" and pending is not None:
      pick.append((t - pending) * 1e-6); pending = None
a = np.array(rows)
for i, n in enumerate(["camerad: EOF -> wide frame published", "modeld: frame ready -> modelV2 publish", "  modelExecutionTime",
                       "total EOF -> modelV2 publish"]):
  print(f"  {n:40} p50 {np.percentile(a[:,i],50):6.2f}  p90 {np.percentile(a[:,i],90):6.2f}  p99 {np.percentile(a[:,i],99):6.2f} ms")
o = a[:,1] - a[:,2]; print(f"  {'modeld outside execution':40} p50 {np.percentile(o,50):6.2f}  p90 {np.percentile(o,90):6.2f}  p99 {np.percentile(o,99):6.2f} ms")
p = np.array(pick); print(f"  {'modelV2 -> first controlsState after':40} p50 {np.percentile(p,50):6.2f}  p90 {np.percentile(p,90):6.2f}  p99 {np.percentile(p,99):6.2f} ms")
